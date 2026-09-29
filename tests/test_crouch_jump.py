from types import SimpleNamespace

import pytest

from roki_ng.body import Body
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


@pytest.fixture
def body(tmp_path):
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *args: None, lambda *args: None)
    body.command("control.acquire", {})
    body.hardware.drained = lambda: True
    body.pose = "crouch"
    body.engine = object()
    body.servo_targets = {(s, b): 7500 + 3*s + b for s, b, *_ in body.model.ACTIVESERVOS}
    return body


def finish(body):
    for _ in range(100):
        if body.active is None:
            return
        body.next_at = 0
        body.tick()
    pytest.fail("Motion did not complete")


@pytest.mark.parametrize("direction,ids", [
    ("turn_left", {5, 10}), ("turn_right", {5, 10}),
    ("forward", {9, 10}), ("backward", {9, 10}),
    ("left", {6, 10}), ("right", {6, 10}),
])
@pytest.mark.parametrize("fraction", [0.1, 0.5, 1.0])
def test_relative_jump_returns_only_selected_joints(body, direction, ids, fraction):
    baseline = dict(body.servo_targets)
    engine = body.engine
    sent = []
    body.hardware.send = lambda values, frames, pause: sent.append(values)
    job = body.command("motion.jump", {"direction": direction, "fraction": fraction,
                                       "hold_crouch": True})
    finish(body)
    assert body.jobs[job["job_id"]]["status"] == "completed"
    assert sent
    selected = {(s, b) for s in ids for b in (1, 2)}
    assert all({(v.Id, v.Sio) for v in frame} == selected for frame in sent)
    assert any(v.Data != baseline[v.Id, v.Sio] for frame in sent for v in frame)
    assert {(v.Id, v.Sio): v.Data for v in sent[-1]} == {k: baseline[k] for k in selected}
    assert body.servo_targets == baseline
    assert body.pose == "crouch" and body.engine is engine
    if direction.startswith("turn_"):
        assert len(sent) == 2
    if direction in ("forward", "backward"):
        ankle = {(v.Id, v.Sio): v.Data - baseline[v.Id, v.Sio] for v in sent[0]}
        assert ankle[9, 1] == -ankle[9, 2] != 0


def test_hard_stop_does_not_restore_crouch(body):
    body.command("motion.jump", {"hold_crouch": True})
    body.tick()
    assert body.pose == "unknown"
    body.command("motion.stop_hard", {})
    finish(body)
    assert body.pose == "unknown" and body.engine is None
    assert body.servo_targets == {}


def test_reject_before_sending_without_pose_or_targets(body):
    body.pose = "stand"
    with pytest.raises(Fault, match="requires crouch"):
        body.command("motion.jump", {"hold_crouch": True})
    body.pose = "crouch"
    body.servo_targets.clear()
    with pytest.raises(Fault, match="Missing commanded"):
        body.command("motion.jump", {"hold_crouch": True})
    assert body.active is None and body.hardware.sent == 0


def test_reject_out_of_range_before_any_motion(body):
    body.servo_targets[10, 1] = 0
    with pytest.raises(Fault, match="protocol range"):
        body.command("motion.jump", {"direction": "turn_left", "hold_crouch": True})
    assert body.active is None and body.hardware.sent == 0


def test_failed_send_discards_targets(body):
    def fail(*args):
        raise OSError("link lost")
    body.hardware.send = fail
    job = body.command("motion.jump", {"hold_crouch": True})
    finish(body)
    assert body.jobs[job["job_id"]]["status"] == "failed"
    assert body.pose == "unknown" and body.engine is None
    assert body.servo_targets == {}


def test_successful_servo_send_records_target(body):
    body.servo_targets.clear()
    value = SimpleNamespace(Id=9, Sio=1, Data=8000)
    body._start("test", iter([("servo", [value], 2, 1), ("drain",)]))
    finish(body)
    assert body.servo_targets == {(9, 1): 8000}


def test_legacy_jump_still_uses_absolute_slot(body):
    sent = []
    body.hardware.send = lambda values, frames, pause: sent.append(values)
    body.command("motion.jump", {"direction": "turn_left"})
    finish(body)
    assert any(v.Id == 8 for v in sent[0])
    assert body.pose == "unknown" and body.engine is None
