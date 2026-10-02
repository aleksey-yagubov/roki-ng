import asyncio
import json
import time

import pytest

from roki_ng.body import Body
from roki_ng.client import Client
from roki_ng.parameters import Parameters
from roki_ng.stream import Streams, pipeline_description
from roki_ng.video_sources import capture_source, settings_for
from roki_ng.supervisor import Supervisor, Session
from roki_ng.wire import Fault, UDP_LIMIT, udp_socket, unpack


def test_parameters_persist_and_reject_invalid(tmp_path):
    params = Parameters(tmp_path)
    params.set("walk.max_step_mm", 30)
    assert Parameters(tmp_path).values["walk.max_step_mm"] == 30
    for value in (True, float("nan"), -1, "24"):
        with pytest.raises(Fault):
            params.set("walk.max_step_mm", value)
    assert json.loads(params.path.read_text())["walk.max_step_mm"] == 30


def test_body_busy_cancel_and_recovery(tmp_path):
    events = []
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *args: events.append(args), lambda *args: None)
    body.command("control.acquire", {})
    slot = body.command("motion.slot", {"name": "Initial_Pose"})
    body.tick()
    with pytest.raises(Fault, match="already running"):
        body.command("motion.jump", {"direction": "left"})
    body.command("motion.stop_hard", {})
    sent = body.hardware.sent
    body.tick()
    assert body.hardware.sent == sent
    assert body.jobs[slot["job_id"]]["status"] == "cancelled"
    assert body.pose == "unknown"
    drive = body.command("motion.drive", {"x": 1})
    assert drive["job_id"] == body.active
    body.command("motion.stop_hard", {})
    body.command("control.release", {})
    with pytest.raises(Fault, match="released"):
        body.command("motion.pose", {"name": "crouch"})


def test_jump_turn_only_commands_legs(tmp_path):
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *a: None, lambda *a: None)
    body.command("control.acquire", {})
    for direction in ("turn_left", "turn_right"):
        body.command("motion.jump", {"direction": direction, "fraction": 0.25})
        frames = [step for step in body.plan if step[0] == "servo"]
        assert len(frames) == 3
        for _, values, _, _ in frames:
            assert {(v.Id, v.Sio) for v in values} == {
                (servo, bus) for servo in (5, 6, 7, 8, 9, 10, 13) for bus in (1, 2)}
        body.command("motion.stop_hard", {})


def test_walk_release_settles(tmp_path):
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values}, lambda *a: None, lambda *a: None)
    body.command("control.acquire", {})
    body.pose = "crouch"
    job = body.command("motion.drive", {"x": 1, "crouch": "on"})
    body.tick()
    body.command("control.release", {})
    with pytest.raises(Fault, match="still stopping"):
        body.command("control.acquire", {})
    deadline = time.monotonic() + 1
    while body.active and time.monotonic() < deadline:
        body.tick()
        time.sleep(0.01)
    assert body.pose == "crouch"
    assert body.jobs[job["job_id"]]["status"] == "completed"


def test_video_lifecycle_does_not_open_on_query():
    events = []
    video = Streams({"simulate": True}, lambda *a: events.append(a), lambda *a: None)
    video.command("videostream.capabilities", {})
    source = capture_source() | {"name": "stream"}
    video.command("sources.configure", {"items": [source]})
    assert not video.state()["active_streams"]
    assert all(p.Gst is None for p in video.pipelines.values())
    text = pipeline_description(source, source["settings"])
    assert "width=1600,height=1300,depth=10" in text
    assert "width=800,height=650,framerate=60/1" in text
    assert "libcamerasrc" in text and "v4l2h264enc" in text
    destination = {'name':'stream', 'session_id':1, 'host':'127.0.0.1', 'rtp_port':5004}
    first = video.command("videostream.subscribe", destination)
    second = video.command("videostream.subscribe", destination)
    assert first["run_id"] == second["run_id"] and second["receivers"] == 1
    video.command("videostream.stop", {"name": "stream"})
    video.command("videostream.subscribe", destination)
    video.close()
    assert not video.state()["active_streams"]
    with pytest.raises(Fault):
        settings_for(source, {"width": 801})


def test_udp_processes(tmp_path):
    async def exercise():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await server.start()
        client = Client("127.0.0.1", server.sock.getsockname()[1])
        observer = Client("127.0.0.1", server.sock.getsockname()[1])
        try:
            assert server.mode == "IDLE"
            welcome = await client.connect()
            assert welcome["state"] == "IDLE"
            await observer.connect()
            assert set(server.workers) == {"motherboard", "stream", "camera", "detection", "localisation"}
            assert all(w.process.pid for w in server.workers.values())
            await client.request("control.acquire")
            with pytest.raises(Fault, match="Another operator"):
                await observer.request("control.acquire")
            await client.request("mode.set", {"mode": "MANUAL"})
            await client.request("log.subscribe", {"after": 0})
            await client.request("data.subscribe", {"topic": "motion.state", "rate_hz": 10})
            pose = await client.request("motion.pose", {"name": "base_stand"})
            with pytest.raises(Fault, match="running"):
                await client.request("motion.jump", {"direction": "left"})
            await client.request("motion.stop_hard")
            status = await client.request("job.status", {"job_id": pose["job_id"]})
            assert status["status"] == "cancelled"
            await client.request("motion.pose", {"name": "crouch"})
            await asyncio.sleep(0.2)
            await client.drive(x=0.5, crouch="on")
            await asyncio.sleep(0.7)
            state = await client.request("data.snapshot", {"topic": "motion.state"})
            assert state["data"]["pose"] == "crouch"
            await client.request("videostream.subscribe", {"name": "stream", "rtp_port": 5004})
            await asyncio.sleep(0.15)
            assert (await client.request("videostream.status", {"name": "stream"}))["state"] == "running"
            # A malformed datagram cannot stop the service.
            await asyncio.get_running_loop().sock_sendto(client.sock, b"\xc1", client.address)
            assert (await client.request("system.status"))["state"] == "MANUAL"
            await client.request("control.release")
            assert server.owner is None
            await observer.request("control.acquire")
            await observer.request("mode.set", {"mode": "MANUAL"})
        finally:
            await client.close()
            await observer.close()
            await server.close()
        assert all(w.process.returncode is not None for w in server.workers.values())
    asyncio.run(exercise())


def test_worker_fault_keeps_control_endpoint_alive(tmp_path):
    async def exercise():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await server.start()
        client = Client("127.0.0.1", server.sock.getsockname()[1])
        try:
            await client.connect()
            await client.request("control.acquire")
            await client.request("mode.set", {"mode": "MANUAL"})
            worker = server.workers["stream"]
            old_pid = worker.process.pid
            worker.process.kill()
            await worker.process.wait()
            await asyncio.sleep(0.1)
            assert (await client.request("system.status"))["state"] == "FAULT"
            await client.request("control.acquire")
            await client.request("system.restart_stream_worker")
            assert server.workers["stream"].process.pid != old_pid
            assert (await client.request("system.status"))["state"] == "IDLE"
            await client.request("mode.set", {"mode": "MANUAL"})
            assert (await client.request("videostream.list"))["total"] == 3
            await worker.close()
        finally:
            await client.close()
            await server.close()
    asyncio.run(exercise())


def test_unicode_logs_preserved_and_fit_datagram(tmp_path):
    async def run():
        server = Supervisor({"state_dir": str(tmp_path)})
        receiver = udp_socket()
        receiver.bind(("127.0.0.1", 0))
        server.sock = udp_socket()
        server.sock.bind(("127.0.0.1", 0))
        session = Session(2**64 - 1, 2**64 - 1, receiver.getsockname(), "logs")
        server.sessions[session.id] = session
        await server.dispatch(session, "log.subscribe", {"after": 0})
        message = "Ошибка камеры, подробное описание. " * 100
        server.log("камера/" * 9, "INFO", message)
        sender = asyncio.create_task(server._maintenance())
        records = []

        async def receive():
            while len("".join(r["message"] for r in records)) < len(message):
                raw, address = await asyncio.get_running_loop().sock_recvfrom(receiver, 65535)
                assert address == server.sock.getsockname()
                assert len(raw) <= UDP_LIMIT
                packet = unpack(raw)
                assert packet["op"] == "log.sample"
                assert packet["session"] == session.id and packet["token"] == session.token
                assert packet["body"]["dropped"] == 0
                records.extend(packet["body"]["records"])

        try:
            await asyncio.wait_for(receive(), 3)
            assert "".join(r["message"] for r in records) == message
            sequences = [r["record_sequence"] for r in records]
            assert sequences == list(range(sequences[0], sequences[0] + len(sequences)))
            assert server.counters["oversize"] == 0
        finally:
            sender.cancel()
            await asyncio.gather(sender, return_exceptions=True)
            receiver.close()
            server.sock.close()
    asyncio.run(run())


def test_dead_client_cleanup(tmp_path):
    async def exercise():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await server.start()
        client = Client("127.0.0.1", server.sock.getsockname()[1])
        try:
            await client.connect()
            await client.request("control.acquire")
            await client.request("mode.set", {"mode": "MANUAL"})
            await client.request("videostream.subscribe", {"name": "stream", "rtp_port": 5004})
            for task in client.tasks:
                task.cancel()
            await asyncio.gather(*client.tasks, return_exceptions=True)
            client.sock.close()
            await asyncio.sleep(2.6)
            assert not server.sessions
            assert server.owner is None
            state = await server.workers["stream"].call("state")
            assert not state["active_streams"]
        finally:
            await server.close()
    asyncio.run(exercise())
