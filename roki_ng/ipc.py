"""Static connected subprocess channels; no broker or discovery."""

import asyncio
import os
import socket
import sys
import time

from .wire import Fault, IPC_LIMIT, pack, unpack


class WorkerPeer:
    def __init__(self, role, config, event, log):
        self.role, self.config, self.event, self.log = role, config, event, log
        self.pending = {}
        self.counter = 0
        self.last_heartbeat = time.monotonic()
        self.state = {"state": "starting"}
        self.tasks = []
        self.alive = False
        self.queues = {}
        self.close_lock = asyncio.Lock()
        self.closed = False

    async def start(self):
        ends, children = [], []
        for name in ("normal", "urgent", "logs"):
            parent, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
            parent.setblocking(False)
            ends.append((name, parent))
            children.append(child)
        try:
            self.process = await asyncio.create_subprocess_exec(
                sys.executable, "-u", "-m", "roki_ng.worker", self.role,
                *(str(s.fileno()) for s in children),
                pass_fds=tuple(s.fileno() for s in children),
                stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.PIPE,
                env=os.environ | {"PYTHONUNBUFFERED": "1"})
        except BaseException:
            for _, sock in ends:
                sock.close()
            raise
        finally:
            for child in children:
                child.close()
        self.sockets = dict(ends)
        self.alive = True
        for name, sock in ends:
            self.tasks.append(asyncio.create_task(self._read(name, sock)))
            if name != "logs":
                queue = asyncio.Queue(maxsize=32)
                self.queues[name] = queue
                self.tasks.append(asyncio.create_task(self._write(sock, queue)))
        for source, pipe in (("stdout", self.process.stdout), ("stderr", self.process.stderr)):
            self.tasks.append(asyncio.create_task(self._capture(source, pipe)))
        self.tasks.append(asyncio.create_task(self._wait()))
        await self.call("initialize", self.config, timeout=15)

    async def _write(self, sock, queue):
        loop = asyncio.get_running_loop()
        try:
            while True:
                data = await queue.get()
                # One send is one seqpacket; never split a record.
                while True:
                    try:
                        if sock.send(data) != len(data):
                            raise OSError("Short seqpacket write")
                        break
                    except BlockingIOError:
                        future = loop.create_future()
                        loop.add_writer(sock.fileno(), lambda: not future.done() and future.set_result(None))
                        try:
                            await future
                        finally:
                            loop.remove_writer(sock.fileno())
        except (OSError, EOFError) as exc:
            self._failed(str(exc))

    async def call(self, op, body=None, *, urgent=False, timeout=3):
        if not self.alive:
            raise Fault("worker_unavailable", self.role)
        if len(self.pending) >= 32:
            raise Fault("busy", "Worker request limit", True)
        self.counter += 1
        ident = self.counter
        future = asyncio.get_running_loop().create_future()
        self.pending[ident] = future
        message = {"abi": 1, "kind": "request", "id": ident, "op": op,
                   "deadline_ns": time.monotonic_ns() + int(timeout * 1e9), "body": body or {}}
        try:
            self.queues["urgent" if urgent else "normal"].put_nowait(pack(message, IPC_LIMIT))
            return await asyncio.wait_for(future, timeout)
        except asyncio.QueueFull as exc:
            raise Fault("busy", "Worker channel full", True) from exc
        except TimeoutError as exc:
            raise Fault("worker_timeout", f"{self.role}: {op}") from exc
        finally:
            self.pending.pop(ident, None)

    async def _read(self, name, sock):
        try:
            while True:
                data = await asyncio.get_running_loop().sock_recv(sock, IPC_LIMIT + 1)
                if not data:
                    raise EOFError("Worker channel closed")
                message = unpack(data, IPC_LIMIT)
                if name == "logs":
                    self.log(self.role, message.get("level", "INFO"), message.get("message", ""))
                elif message.get("kind") == "response":
                    future = self.pending.get(message.get("id"))
                    if future is not None and not future.done():
                        if "error" in message:
                            error = message["error"]
                            future.set_exception(Fault(error["code"], error["message"], error.get("retryable", False)))
                        else:
                            future.set_result(message.get("result", {}))
                elif message.get("op") == "heartbeat":
                    self.last_heartbeat = time.monotonic()
                    self.state = message["body"]
                else:
                    self.event(self.role, message.get("op", "unknown"), message.get("body", {}))
        except (OSError, EOFError, Fault) as exc:
            self._failed(str(exc))

    async def _capture(self, source, pipe):
        pending = b""
        while data := await pipe.read(2048):
            pending += data
            while b"\n" in pending or len(pending) >= 512:
                cut = pending.find(b"\n")
                cut = min(cut, 512) if cut >= 0 else 512
                line, pending = pending[:cut], pending[cut + (1 if pending[cut:cut+1] == b"\n" else 0):]
                self.log(f"{self.role}/{source}", "INFO", line.decode(errors="replace"))
        if pending:
            self.log(f"{self.role}/{source}", "INFO", pending.decode(errors="replace"))

    async def _wait(self):
        code = await self.process.wait()
        self._failed(f"Process exited: {code}")

    def _failed(self, reason):
        if not self.alive:
            return
        self.alive = False
        self.state = {"state": "fault", "error": reason}
        for future in self.pending.values():
            if not future.done():
                future.set_exception(Fault("worker_unavailable", reason))
        self.event(self.role, "worker.fault", self.state)

    async def close(self):
        async with self.close_lock:
            if self.closed:
                return
            await self._close()
            self.closed = True

    async def _close(self):
        if not hasattr(self, "process"):
            return
        if self.alive:
            try:
                await self.call("shutdown", urgent=True, timeout=2)
            except Fault:
                pass
        try:
            await asyncio.wait_for(self.process.wait(), 2)
        except TimeoutError:
            self.process.terminate()
            try:
                await asyncio.wait_for(self.process.wait(), 1)
            except TimeoutError:
                self.process.kill()
                await self.process.wait()
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        for sock in self.sockets.values():
            sock.close()
