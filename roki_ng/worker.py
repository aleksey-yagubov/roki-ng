"""Worker process entry point. Hardware operations stay outside supervisor."""

import collections
import select
import socket
import sys
import time
import traceback

from .wire import Fault, IPC_LIMIT, pack, unpack


def run(role, fds):
    normal, urgent, logs = [socket.socket(fileno=fd) for fd in fds]
    for sock in (normal, urgent, logs):
        sock.setblocking(False)
    outgoing = {normal: collections.deque(maxlen=64), urgent: collections.deque(maxlen=16)}
    running = True
    driver = None
    logs_dropped = 0
    latest_body_telemetry = None

    def send(sock, message):
        queue = outgoing[sock]
        if len(queue) == queue.maxlen:
            raise RuntimeError("IPC output full; supervisor is not draining")
        queue.append(pack(message, IPC_LIMIT))

    def emit(op, body):
        nonlocal latest_body_telemetry
        if op == "body.telemetry":
            latest_body_telemetry = {"abi": 1, "kind": "event", "op": op, "body": body}
            return
        send(normal, {"abi": 1, "kind": "event", "op": op, "body": body})

    def log(level, message):
        nonlocal logs_dropped
        try:
            logs.send(pack({"level": level, "message": str(message)[:8192]}, IPC_LIMIT))
        except BlockingIOError:
            logs_dropped += 1

    next_heartbeat = 0
    try:
        while running or any(outgoing.values()):
            if running and latest_body_telemetry is not None and not outgoing[normal]:
                send(normal, latest_body_telemetry)
                latest_body_telemetry = None
            device_fds = driver.filenos() if running and driver and hasattr(driver, "filenos") else []
            readable, writable, _ = select.select(
                [urgent, normal, *device_fds] if running else [],
                [sock for sock, queue in outgoing.items() if queue], [], 0.01)
            for sock in writable:
                try:
                    data = outgoing[sock][0]
                    if sock.send(data) != len(data):
                        raise OSError("Short seqpacket write")
                    outgoing[sock].popleft()
                except BlockingIOError:
                    pass
            # Priority path is always serviced first, once per loop.
            for sock in (urgent, normal):
                if sock not in readable:
                    continue
                try:
                    raw = sock.recv(IPC_LIMIT + 1)
                except BlockingIOError:
                    continue
                if not raw:
                    return
                request = unpack(raw, IPC_LIMIT)
                response = {"abi": 1, "kind": "response", "id": request.get("id")}
                try:
                    if request.get("abi") != 1 or request.get("kind") != "request":
                        raise Fault("bad_message", "Invalid IPC envelope")
                    if request["deadline_ns"] < time.monotonic_ns():
                        raise Fault("expired", "IPC command expired")
                    op, body = request["op"], request["body"]
                    if op == "initialize":
                        if driver is not None:
                            raise Fault("busy", "Already initialized")
                        if role == "motherboard":
                            from .body import Body
                            driver = Body(body, emit, log)
                        elif role == "stream":
                            from .stream import Streams
                            driver = Streams(body, emit, log)
                        elif role == "camera":
                            from .camera import Camera
                            driver = Camera(body, emit, log)
                        elif role == "detection":
                            from .detection import Detection
                            driver = Detection(body, emit, log)
                        elif role == "localisation":
                            from .localisation_worker import Localisation
                            driver = Localisation(body, emit, log)
                        else:
                            raise Fault("invalid_argument", "Unknown worker")
                        response["result"] = driver.state()
                    elif op == "shutdown":
                        if driver is not None:
                            driver.close()
                            driver = None
                        running = False
                        response["result"] = {"stopped": True}
                    elif driver is None:
                        raise Fault("not_ready", "Worker not initialized")
                    elif op == 'source.list':
                        describe = getattr(driver, 'video_source', None)
                        source = describe() if describe else None
                        response['result'] = {'items': [source] if source else []}
                    else:
                        response["result"] = driver.command(op, body)
                        if role == "motherboard" and sock is urgent and op in ("control.release", "control.takeover", "motion.stop_hard"):
                            # Reject commands queued before the ownership/stop barrier.
                            while True:
                                try:
                                    pending = normal.recv(IPC_LIMIT + 1)
                                except BlockingIOError:
                                    break
                                if not pending:
                                    return
                                old = unpack(pending, IPC_LIMIT)
                                send(normal, {"abi": 1, "kind": "response", "id": old.get("id"),
                                              "error": Fault("cancelled", "Command discarded at stop barrier").as_dict()})
                except Fault as exc:
                    response["error"] = exc.as_dict()
                except Exception as exc:
                    traceback.print_exc()
                    response["error"] = Fault("hardware_error", str(exc)).as_dict()
                send(sock, response)
            if running and driver is not None:
                for fd in device_fds:
                    if fd in readable and fd in driver.filenos():
                        driver.ready(fd)
                driver.tick()
                if time.monotonic() >= next_heartbeat:
                    emit("heartbeat", driver.state() | {"logs_dropped": logs_dropped})
                    next_heartbeat = time.monotonic() + 0.5
    finally:
        if driver is not None:
            driver.close()
        for sock in (normal, urgent, logs):
            sock.close()


if __name__ == "__main__":
    run(sys.argv[1], [int(value) for value in sys.argv[2:5]])
