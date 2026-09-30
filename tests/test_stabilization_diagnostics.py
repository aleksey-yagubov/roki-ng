from types import SimpleNamespace

from roki_ng.body import Body
from roki_ng.parameters import Parameters


def setup(tmp_path, monkeypatch):
    clock = [10.0]
    monkeypatch.setattr("time.monotonic", lambda: clock[0])
    logs = []
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *a: None, lambda *a: logs.append(a))
    body.reconnect_at = float("inf")
    body.pose = "crouch"
    body.parameters["stabilization.diagnostics_enabled"] = True
    values = [SimpleNamespace(Id=7, Sio=b, Data=7600 if b == 1 else 7400) for b in (1, 2)]
    body._send_targets(values, 2, 1)
    return body, clock, logs


def test_diagnostics_polls_and_logs_without_servo_commands(tmp_path, monkeypatch):
    body, clock, logs = setup(tmp_path, monkeypatch)
    count = body.hardware.sent
    for _ in range(10):
        clock[0] += 0.21
        body.tick()
    assert body.hardware.sent == count
    assert body.body_servos.sequence == 10
    assert body.stabilization_diagnostics.result["status"] == "persistent_difference"
    assert any(level == "WARNING" and "ID7" in text for level, text in logs)
    assert len([text for _, text in logs if text.startswith("Stab diag:")]) <= 3
    body.parameters["stabilization.diagnostics_enabled"] = False
    before = len(logs)
    clock[0] += 1
    body.tick()
    assert body.body_servos.sequence == 10 and len(logs) == before
    assert body.stabilization_diagnostics.result == {"enabled": False}


def test_matching_positions_and_stale_samples(tmp_path, monkeypatch):
    body, clock, _ = setup(tmp_path, monkeypatch)
    actual = [0] * 30
    actual[14] = actual[15] = 153
    body.hardware.body_positions = lambda: actual
    clock[0] += 0.4
    body.tick()
    assert body.stabilization_diagnostics.result["status"] == "within_tolerance"
    body.servo_watch = False
    body.body_servos.next_at = float("inf")
    clock[0] += 1
    body.tick()
    assert body.stabilization_diagnostics.result["status"] == "stale"
    assert body.stabilization_diagnostics.counts == [0, 0]


def test_no_repeat_count_for_same_sample_or_unknown_targets(tmp_path, monkeypatch):
    body, clock, _ = setup(tmp_path, monkeypatch)
    clock[0] += 0.4
    body.tick()
    for _ in range(20):
        body.stabilization_diagnostics.tick(body, clock[0])
    assert body.stabilization_diagnostics.counts == [1, 1]
    body.command("motion.stop_hard", {})
    clock[0] += 0.3
    body.tick()
    assert body.stabilization_diagnostics.result["status"] == "no_targets"


def test_recent_or_changed_goals_are_not_reported_as_failed_tracking(tmp_path, monkeypatch):
    body, clock, _ = setup(tmp_path, monkeypatch)
    clock[0] += 0.05
    body.tick()
    assert body.stabilization_diagnostics.result["status"] == "changing_target"
    clock[0] += 0.4
    body.body_servos.read(body)
    body.sent_targets[7, 1] += 10
    body.stabilization_diagnostics.tick(body, clock[0])
    assert body.stabilization_diagnostics.result["status"] == "changing_target"
    assert body.stabilization_diagnostics.counts == [0, 0]
