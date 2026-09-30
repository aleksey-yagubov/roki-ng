import pytest

from types import SimpleNamespace

from roki_ng.body import Body, RokiHardware, SimHardware
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


def make_body(tmp_path):
    events, logs = [], []
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda op, data: events.append((op, data)),
                lambda *args: logs.append(args))
    body.command("control.acquire", {})
    return body, events, logs


def fail():
    raise Fault("hardware_error", "ack timeout")


def test_missing_body_keeps_worker_usable(tmp_path, monkeypatch):
    monkeypatch.setattr("roki_ng.body.RokiHardware", SimHardware)
    body = Body({"parameters": Parameters(tmp_path).values}, lambda *a: None, lambda *a: None)
    body.hardware.connect_body = fail
    for _ in range(10):
        body.reconnect_at = 0
        body.tick()
    assert body.state()["state"] == "degraded"
    assert body.retry_delay == 5
    assert body.command("motion.slots", {})["total"] == 57
    body.command("control.acquire", {})
    with pytest.raises(Fault, match="disconnected"):
        body.command("motion.jump", {})


def test_loss_cancels_motion_and_reconnect_does_not_resume(tmp_path):
    body, events, logs = make_body(tmp_path)
    ident = body.command("motion.jump", {})["job_id"]
    body.hardware.probe = fail
    body.tick()
    assert body.jobs[ident]["status"] == "failed"
    assert body.plan is None and body.drive is None and body.engine is None
    assert not body.body_connected
    assert any(op == "body.connection" for op, _ in events)
    assert logs[-1][0] == "ERROR"
    body.hardware.probe = lambda: None
    for _ in range(3):
        body.reconnect_at = 0
        body.tick()
    assert body.body_connected and body.active is None
    assert body.hardware.sent == 0
    new_job = body.command("motion.jump", {})["job_id"]
    assert new_job != ident
    assert body.active == new_job


def test_command_failure_is_connection_failure(tmp_path):
    body, _, _ = make_body(tmp_path)
    body.hardware.head = lambda *args: fail()
    with pytest.raises(Fault):
        body.command("motion.head", {})
    assert not body.body_connected
    assert body.pose == "unknown"


def test_retries_do_not_spam_and_shutdown_handles_reset_failure(tmp_path):
    body, events, logs = make_body(tmp_path)
    body.hardware.probe = fail
    body.hardware.connect_body = fail
    body.hardware.reset = fail
    for _ in range(5):
        body.reconnect_at = 0
        body.tick()
    assert len([op for op, _ in events if op == "body.connection"]) == 1
    assert len([level for level, _ in logs if level == "ERROR"]) == 1
    body.close()


def test_single_spurious_ack_does_not_restore_connection(tmp_path):
    body, _, _ = make_body(tmp_path)
    body._link_lost(Fault("hardware_error", "disconnected"))
    body.reconnect_at = 0
    body.tick()
    assert not body.body_connected
    body.hardware.connect_body = fail
    body.reconnect_at = 0
    body.tick()
    assert body.probe_successes == 0


def test_walk_from_unknown_prepares_crouch_without_separate_command(tmp_path):
    body, _, _ = make_body(tmp_path)
    body.command("motion.drive", {"x": 1})
    assert body.pose == "unknown"
    kind, values, frames, pause = next(body.plan)
    assert kind == "servo" and values  # Simulated initial crouch targets.
    assert (frames, pause) == (4, 3)
    assert next(body.plan) == ("drain",)
    next(body.plan)
    assert body.pose == "crouch"


def test_stand_from_unknown_uses_base_pose(tmp_path):
    body, _, _ = make_body(tmp_path)
    body.command("motion.pose", {"name": "stand"})
    steps = list(body.plan)
    assert any(step[0] == "servo" for step in steps)
    assert body.pose == "base_stand"
    assert body.head == {"pan": 0, "tilt": 0}


def test_kinematics_fault_does_not_become_link_failure(tmp_path):
    body, events, _ = make_body(tmp_path)
    body.error = "No valid leg solution; movement stopped"
    with pytest.raises(Fault) as error:
        body.command("motion.jump", {})
    assert error.value.code == "motion_fault"
    assert body.body_connected
    assert not events


def test_busy_probe_preserves_connection_and_pending_motion(tmp_path):
    body, events, _ = make_body(tmp_path)
    job = body.command("motion.jump", {})["job_id"]
    resets = body.hardware.resets

    def busy():
        raise Fault("body_busy", "UART8 is executing a queued command", True)

    body.hardware.probe = busy
    body.tick()
    assert body.body_connected and body.active == job
    assert body.hardware.resets == resets
    assert not any(op == "body.connection" for op, _ in events)


def test_native_busy_is_not_transport_failure():
    hardware = object.__new__(RokiHardware)
    hardware.mb = SimpleNamespace(GetLastStatus=lambda: 5)
    source = SimpleNamespace(GetError=lambda: "Busy")
    with pytest.raises(Fault) as error:
        hardware.check(False, source)
    assert error.value.code == "body_busy" and error.value.retryable


def test_failed_queued_command_cancels_even_during_interpolation(tmp_path):
    body, events, _ = make_body(tmp_path)
    job = body.command("motion.jump", {})["job_id"]
    body.next_at = float("inf")
    body.hardware.check_queue = fail
    body.tick()
    assert body.jobs[job]["status"] == "failed"
    assert not body.body_connected and body.plan is None


def test_empty_queue_with_active_uart_is_not_drained():
    hardware = object.__new__(RokiHardware)
    hardware.busy_until = 0
    hardware.body_failures = 0
    state = {"body_queue_size": 0, "body_busy": True, "body_failures": 0, "last_body_error": 0}
    hardware.mb = SimpleNamespace(GetStatus=lambda: state.copy())
    assert not hardware.drained()
    state["body_busy"] = False
    assert hardware.drained()
    state["body_failures"] = 1  # Last reply can already be successful.
    with pytest.raises(Fault, match="0 -> 1"):
        hardware.drained()
