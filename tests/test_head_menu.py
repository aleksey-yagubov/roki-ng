import asyncio
import time
from types import SimpleNamespace

import pytest

from roki_ng.head_menu import HeadButtons, HeadMenu, menu_tree
from roki_ng.supervisor import Session, Supervisor
from roki_ng.client import Client
from roki_ng.wire import Fault


class Harness:
    def __init__(self):
        self.spoken, self.calls, self.logs = [], [], []
        self.status = "running"
        self.released = 0
        self.failure = None
        self.gate = None
        self.menu = HeadMenu(self.command, self.release, self.spoken.append,
                             lambda *args: self.logs.append(args))

    async def command(self, op, args):
        self.calls.append((op, args))
        if op.endswith(".start"):
            if self.gate:
                await self.gate.wait()
            if self.failure:
                raise self.failure
            return {"job_id": "test1"}
        if op == "job.cancel":
            self.status = "cancelled"
        return {"job_id": "test1", "status": self.status}

    async def release(self):
        self.released += 1

    def select_test(self, index=0):
        self.menu.press("right")
        self.menu.press("ok")
        for _ in range(index):
            self.menu.press("right")


def test_tree_and_voice_navigation():
    h = Harness()
    h.menu.press("ok")
    assert h.spoken[-1] == "Football"
    h.menu.press("ok")
    assert h.spoken[-1] == "Goalkeeper"
    h.menu.press("ok")
    assert h.menu.selected.args["strategy"] == "FIRA_penalty_Goalkeeper"
    h.menu.press("back")
    h.menu.press("right")
    h.menu.press("ok")
    assert h.spoken[-1] == "Left"
    h.menu.press("right")
    h.menu.press("ok")
    h.menu.press("right")
    assert h.menu.selected.args == {"strategy": "forward", "entry": "center", "delay_seconds": 10}
    assert not h.calls  # Navigating/selecting does not move or open the camera.
    root = menu_tree()
    forward = root.children[0].children[0].children[1]
    assert [len(item.children) for item in forward.children] == [1, 2, 1]
    assert [i.label for i in root.children[1].children[:5]] == [
        "Rotation right", "Short run", "Long run", "Spot run", "Kick test"]


def test_key_press_not_hold_or_power():
    h = Harness()
    buttons = HeadButtons(h.menu, lambda *a: None)
    def event(code, value):
        buttons.event(SimpleNamespace(type=1, code=code, value=value))
    event(186, 1)
    assert h.spoken == ["Tests"]
    for _ in range(30):
        event(186, 2)
        event(186, 1)
        event(116, 1)  # Dedicated power button belongs to the system service.
    assert h.spoken == ["Tests"]
    event(186, 0)
    event(185, 1)
    assert h.spoken[-1] == "Rotation right"
    event(184, 1)
    assert h.spoken[-1] == "Tests"
    event(183, 1)
    assert h.spoken[-1] == "Game"


@pytest.mark.parametrize("index,args", [
    (0, {"name": "rotation_test"}),
    (1, {"name": "run_test", "mode": "short"}),
    (2, {"name": "run_test", "mode": "long"}),
    (3, {"name": "run_test", "mode": "spot"}),
    (4, {"name": "kick_test", "mode": "regular"}),
    (5, {"name": "run_test", "mode": "side_left"}),
    (6, {"name": "run_test", "mode": "side_right"}),
    (7, {"name": "get_up_test"}),
])
def test_start_cancel_and_repeat(index, args):
    async def run():
        h = Harness()
        h.select_test(index)
        h.menu.press("ok")
        for _ in range(10):
            h.menu.press("ok")
        await h.menu.action
        assert h.calls[0] == ("test.start", args)
        assert sum(op == "test.start" for op, _ in h.calls) == 1
        h.menu.press("back")
        await h.menu.action
        assert h.menu.job is None
        assert h.spoken[-1] == "Stopped"
        assert h.released == 1
        assert h.menu.selected.args == args
        await h.menu.close()
    asyncio.run(run())


def test_back_during_start_and_early_completion():
    async def run():
        h = Harness()
        h.gate = asyncio.Event()
        h.select_test(1)
        h.menu.press("ok")
        await asyncio.sleep(0)
        h.menu.press("back")
        h.gate.set()
        await h.menu.action
        assert h.spoken[-1] == "Stopped"
        assert h.menu.job is None
        assert any(op == "job.cancel" for op, _ in h.calls)

        h.status = "completed"
        h.menu.press("ok")
        await h.menu.action
        assert h.spoken[-1] == "Test completed"
        assert h.menu.job is None
        await h.menu.close()
    asyncio.run(run())


@pytest.mark.parametrize("code,speech", [
    ("body_unavailable", "Body unavailable"),
    ("busy", "Control busy"),
    ("football_unavailable", "Football not available yet"),
])
def test_failure_does_not_leave_menu_busy(code, speech):
    async def run():
        h = Harness()
        h.failure = Fault(code, "failure")
        h.select_test()
        h.menu.press("ok")
        await h.menu.action
        assert h.spoken[-1] == speech
        assert h.menu.job is None and h.released == 1
        h.menu.press("back")
        assert h.spoken[-1] == "Tests"
        await h.menu.close()
    asyncio.run(run())


def test_local_menu_lease_and_worker_completion(tmp_path):
    async def run():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await server.start()
        client = Client("127.0.0.1", server.sock.getsockname()[1])
        spoken = []
        menu = HeadMenu(server._head_command, server._head_release, spoken.append, lambda *a: None)
        server.head_menu = menu
        try:
            await client.connect()
            await client.request("control.acquire")
            menu.press("right")
            menu.press("ok")
            menu.press("right")  # Short run.
            menu.press("ok")
            await menu.action
            assert spoken[-1] == "Control busy"
            assert server.owner == client.session
            await client.request("control.release")
            menu.press("ok")
            await menu.action
            assert server.owner == server.local_session.id
            with pytest.raises(Fault, match="Another operator"):
                await client.request("control.acquire")
            menu.press("back")
            await menu.action
            deadline = time.monotonic() + 3
            while menu.job or menu.finishing:
                assert time.monotonic() < deadline
                await asyncio.sleep(0.01)
            assert server.owner is None and spoken[-1] == "Stopped"
            await client.request("control.acquire")
        finally:
            await client.close()
            await server.close()
    asyncio.run(run())


@pytest.mark.parametrize("fail", [False, True])
def test_ready_only_after_all_worker_initializations(tmp_path, monkeypatch, fail):
    from roki_ng import head_menu, supervisor
    events = []

    class Worker:
        def __init__(self, role, *_):
            self.role = role
            self.alive = False
            self.last_heartbeat = time.monotonic()
            self.state = {"state": "ready"}

        async def start(self):
            if fail and self.role == "detection":
                raise RuntimeError("initialize failed")
            self.alive = True
            events.append(self.role)

        async def close(self):
            self.alive = False

    class Voice:
        def __init__(self, log):
            pass

        def say(self, text):
            events.append(text)

        async def close(self):
            pass

    class Buttons:
        def __init__(self, *_):
            pass

        def open(self):
            events.append("buttons")

        async def run(self):
            await asyncio.Future()

        def close(self):
            pass

    monkeypatch.setattr(supervisor, "WorkerPeer", Worker)
    monkeypatch.setattr(head_menu, "Voice", Voice)
    monkeypatch.setattr(head_menu, "HeadButtons", Buttons)

    async def run():
        server = Supervisor({"skip_bootstrap": True, "state_dir": str(tmp_path),
                             "host": "127.0.0.1", "port": 0})
        try:
            await server.start()
            if fail:
                assert server.mode == "FAULT" and "Ready" not in events
                assert "buttons" not in events
            else:
                assert set(events[:-2]) == set(server.workers)
                assert events[-2:] == ["buttons", "Ready"]
        finally:
            await server.close()
    asyncio.run(run())


def test_operator_can_take_control_from_menu_and_another_operator(tmp_path):
    async def run():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await server.start()
        a = Client("127.0.0.1", server.sock.getsockname()[1])
        b = Client("127.0.0.1", server.sock.getsockname()[1])
        spoken = []
        menu = HeadMenu(server._head_command, server._head_release, spoken.append, lambda *a: None)
        server.head_menu = menu
        try:
            await a.connect()
            await b.connect()
            menu.press("right")
            menu.press("ok")
            menu.press("right")
            menu.press("ok")
            await menu.action
            job, old_epoch = menu.job, server.lease_epoch
            reply = await a.request("control.acquire", {"force": True})
            assert reply["stop_confirmed"] and reply["motion_ready"]
            assert server.owner == a.session and server.lease_epoch > old_epoch
            assert menu.job is None and "Operator control" in spoken
            status = await a.request("job.status", {"job_id": job})
            assert status["status"] == "cancelled"
            assert (await server.workers["motherboard"].call("state"))["active_job"] is None
            with pytest.raises(Fault) as error:
                await server.dispatch(server.local_session, "test.start", {
                    "name": "run_test", "mode": "short", "lease_epoch": old_epoch})
            assert error.value.code == "not_owner"

            await b.request("control.acquire", {"force": True})
            assert server.owner == b.session
            with pytest.raises(Fault) as error:
                await a.request("motion.pose", {"name": "base_stand"})
            assert error.value.code == "not_owner"
            while True:
                event = await asyncio.wait_for(a.events.get(), 1)
                if event["op"] == "control.revoked":
                    break
            assert event["body"]["reason"] == "operator_takeover"
        finally:
            await a.close()
            await b.close()
            await server.close()
    asyncio.run(run())


def test_takeover_with_failed_stop_grants_lease_but_blocks_motion(tmp_path):
    class Worker:
        alive = True
        fail = True

        async def call(self, op, *args, **kwargs):
            assert op == "control.takeover" and kwargs["urgent"]
            if self.fail:
                raise Fault("worker_timeout", "motherboard: control.takeover")
            return {"stop_confirmed": True}

    async def run():
        server = Supervisor({"state_dir": str(tmp_path)})
        worker = Worker()
        server.workers["motherboard"] = worker
        session = Session(123, 0, (), "test")
        server.mode = "MANUAL"
        reply = await server.dispatch(session, "control.acquire", {"force": True})
        assert server.owner == session.id
        assert not reply["motion_ready"] and not reply["stop_confirmed"]
        assert reply["stop_error"]["code"] == "worker_timeout"
        with pytest.raises(Fault) as error:
            await server.dispatch(session, "motion.pose", {
                "name": "base_stand", "lease_epoch": reply["lease_epoch"]})
        assert error.value.code == "stop_unconfirmed"
        assert not (await server.dispatch(session, "control.acquire", {}))["motion_ready"]
        worker.fail = False
        reply = await server.dispatch(session, "control.acquire", {"force": True})
        assert reply["motion_ready"] and reply["stop_confirmed"]
    asyncio.run(run())
