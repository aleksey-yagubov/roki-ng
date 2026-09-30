import math
import sys

import pytest

from roki_ng.body import Body
from roki_ng.motion.engine import Engine
from roki_ng.motion.model import Robot
from roki_ng.motion.slots import decode, position, splits
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


@pytest.fixture
def body(tmp_path):
    pytest.importorskip("starkit")
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *a: None, lambda *a: None)
    body.command("control.acquire", {})
    body.hardware.drained = lambda: True
    # Real trajectory calculations, with sends still routed to SimHardware.
    body.simulated = False
    return body


def finish(body):
    for _ in range(1000):
        if not body.active:
            return
        body.next_at = 0
        body.tick()
    pytest.fail("Unfinished job")


def run(body, op, args):
    result = body.command(op, args)
    finish(body)
    job = body.jobs[result["job_id"]]
    assert job["status"] == "completed", job
    return job


def test_individual_units_and_independent_knees():
    slot = {"units": "zubr", "joint_space": "individual", "frames": [
        {"duration": 20, "targets": {"right_knee": 1536, "right_knee_bot": 3072,
                                   "left_knee": 1536, "left_knee_bot": 3072}}]}
    values = decode(slot, Robot())[0][1]
    assert {(v.Id, v.Sio): v.Data for v in values} == {
        (8, 1): 8500, (13, 1): 9500, (8, 2): 6500, (13, 2): 5500}
    del slot["units"]
    assert decode(slot, Robot())[0][1][0].Data == 9036


@pytest.mark.parametrize("units,angle", [("degrees", 0), ("zubr", math.nan),
                                       ("kondo", 10000), ("zubr", True)])
def test_invalid_units_or_value(units, angle):
    with pytest.raises(Fault):
        position(angle, units, "right_knee")


@pytest.mark.parametrize("height", [140, 160, 180, 200, 210])
@pytest.mark.parametrize("centered", [False, True])
def test_crouch_targets_reachable(body, height, centered):
    body.parameters["walk.gait_height_mm"] = height
    engine = Engine(body.parameters)
    values = engine.crouch_target(centered)
    assert len(values) == 23
    assert all(0 <= v.Data <= 16383 for v in values)
    if centered:
        assert engine.ytr == -engine.ytl
        assert engine.ztr == engine.ztl == -height


def test_center_then_shift_then_walk(body):
    run(body, "motion.pose", {"name": "crouch", "crouch": "centered"})
    assert body.pose == "crouch_centered"
    assert body.engine.ytr == -body.engine.ytl
    old = body.engine
    body.simulated = False
    body._start("motion.drive", body._walk(cycles=1, fixed={
        "x": 0.2, "y": 0, "yaw": 0, "speed": .5, "crouch": "centered"}))
    finish(body)
    assert body.pose == "crouch_centered"
    assert body.engine is not old
    assert body.engine.ytr == -body.engine.ytl
    assert body.hardware.sent > 3


@pytest.mark.parametrize("mode", ["on", "centered"])
@pytest.mark.parametrize("direction", ["turn_left", "turn_right", "forward", "backward", "left", "right"])
def test_jump_prepares_from_stand(body, mode, direction):
    body.pose = "stand"
    sent = []
    body.hardware.send = lambda v, f, p: sent.append(v)
    run(body, "motion.jump", {"direction": direction, "crouch": mode})
    assert body.pose == ("crouch" if mode == "on" else "crouch_centered")
    baseline = {(v.Id, v.Sio): v.Data for v in sent[0]}
    assert any(v.Id == 13 for v in sent[0])
    assert all(v.Id not in (8, 13) for frame in sent[1:] for v in frame)
    assert all(v.Data == baseline[v.Id, v.Sio] for v in sent[-1])


@pytest.mark.parametrize("mode", ["on", "centered"])
@pytest.mark.parametrize("direction,ids", [
    ("turn_left", {5, 10}), ("turn_right", {5, 10}),
    ("forward", {9, 10}), ("backward", {9, 10}),
    ("left", {6, 10}), ("right", {6, 10}),
])
def test_base_stand_jump_does_not_prepare_crouch(body, mode, direction, ids):
    run(body, "motion.pose", {"name": "base_stand"})
    baseline = dict(body.servo_targets)
    sent = []
    body.hardware.send = lambda v, f, p: sent.append(v)
    for _ in range(2):
        run(body, "motion.jump", {"direction": direction, "crouch": mode})
        assert body.pose == "base_stand" and body.engine is None
        assert body.servo_targets == baseline
    assert sent
    selected = {(s, b) for s in ids for b in (1, 2)}
    assert all({(v.Id, v.Sio) for v in frame} == selected for frame in sent)


@pytest.mark.parametrize("kind", ["small", "big"])
@pytest.mark.parametrize("mode", ["off", "on", "centered"])
@pytest.mark.parametrize("exit_pose", ["crouch", "stand"])
def test_splits_hold_and_explicit_exit(body, kind, mode, exit_pose):
    body.pose = "stand"
    sent = []
    body.hardware.send = lambda v, f, p: sent.append((v, f))
    run(body, "motion.splits", {"kind": kind, "crouch": mode})
    assert body.pose == f"splits_{kind}" and body.engine is None
    assert len(sent) == 1 + len(splits(kind == "big")["frames"])
    assert sent[1][1] == 80  # Deep entry is not an instantaneous jump.
    previous = len(sent)
    run(body, "motion.pose", {"name": exit_pose})
    assert body.pose == exit_pose
    assert len(sent) == previous + 1


@pytest.mark.parametrize("mode", ["off", "on", "centered"])
@pytest.mark.parametrize("posture,q", [("back", (0, 0, 0, 1)),
    ("stomach", (1, 0, 0, 0)), ("left", (0, 2**-.5, 0, 2**-.5)),
    ("right", (0, -2**-.5, 0, 2**-.5))])
def test_manual_getup_direct_finish(body, mode, posture, q):
    samples = iter([q] * 3)
    body.hardware.body_quaternion = lambda: next(samples, (2**-.5, 0, 0, 2**-.5))
    body.command("motion.get_up", {"crouch": mode})
    steps = list(body.plan)
    # Only 2x50ms for sampling before and after, no half-second settling loop.
    assert sum(step[1] for step in steps if step[0] == "sleep") == pytest.approx(.2)
    assert body.pose == {"off": "stand", "on": "crouch", "centered": "crouch_centered"}[mode]
    servos = [s[1] for s in steps if s[0] == "servo"]
    if mode != "off":
        knee = lambda frame: [v.Data for v in frame if v.Id in (8, 13)]
        assert any(v != 7500 for v in knee(servos[-1]))
        if posture in ("left", "right"):
            assert all(any(v != 7500 for v in knee(frame)) for frame in servos)
        assert body.engine is not None
    assert not body.recovery_active


@pytest.mark.parametrize("op,args", [("motion.get_up", {"crouch": "centered"}),
    ("motion.splits", {"kind": "big"}), ("motion.jump", {"crouch": "centered"})])
def test_manual_hard_stop_cannot_restore_pose(body, op, args):
    body.hardware.body_quaternion = lambda: (0, 0, 0, 1)
    job = body.command(op, args)
    for _ in range(4):
        body.next_at = 0
        body.tick()
    body.command("motion.stop_hard", {})
    finish(body)
    assert body.jobs[job["job_id"]]["status"] == "cancelled"
    assert body.pose == "unknown" and body.engine is None
    assert not body.recovery_active


def test_heading_hold_latches_corrects_and_relatches(body):
    class Gait:
        def walk_Cycle(self, x, y, yaw, *rest):
            corrections.append(yaw)
            yield "drain",
    body.pose, body.engine, body.simulated = "crouch", Gait(), False
    angle = [3.1]
    def imu():
        c, s = math.cos(angle[0]/2), math.sin(angle[0]/2)
        return (c/2**.5, s/2**.5, s/2**.5, c/2**.5)
    body.hardware.body_quaternion = imu
    drive = {"x": 1, "y": 0, "yaw": 0, "speed": .5, "crouch": "on", "heading_hold": True}
    corrections = []
    body._start("motion.drive", body._walk(cycles=5, fixed=drive))
    # Two drains per cycle: gait output and heading-feedback synchronization.
    for a, yaw in [(3.1, 0), (-3.1, 0), (-3.1, 1), (1, 0), (1.1, 0)]:
        angle[0], drive["yaw"] = a, yaw
        next(body.plan)
        next(body.plan)
    list(body.plan)
    assert corrections[:5] == pytest.approx([0, (2*math.pi-6.2)*1.1, .075, 0, .11])


def test_heading_disabled_does_not_poll_imu(body):
    body.pose = "crouch"
    body.hardware.body_quaternion = lambda: pytest.fail("Unexpected IMU request")
    body._start("motion.drive", body._walk(cycles=2, fixed={
        "x": 1, "y": 0, "yaw": 0, "speed": .5, "crouch": "on"}))
    finish(body)


def test_simulation_new_motions_without_native_libraries(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, "starkit", None)
    monkeypatch.setitem(sys.modules, "Roki", None)
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *a: None, lambda *a: None)
    body.command("control.acquire", {})
    body.hardware.drained = lambda: True
    for mode in ("on", "centered"):
        run(body, "motion.pose", {"name": "crouch", "crouch": mode})
        run(body, "motion.jump", {"direction": "turn_left", "crouch": mode})
        run(body, "motion.splits", {"kind": "big", "crouch": mode})
        run(body, "motion.pose", {"name": "stand"})
        samples = iter([(0, 0, 0, 1)] * 3)
        body.hardware.body_quaternion = lambda: next(samples, (2**-.5, 0, 0, 2**-.5))
        run(body, "motion.get_up", {"crouch": mode})
        assert body.engine is None


@pytest.mark.parametrize("op", ["motion.drive", "motion.jump", "motion.get_up", "motion.splits"])
@pytest.mark.parametrize("mode", [True, False, "invalid", None])
def test_invalid_crouch_rejected_before_motion(body, op, mode):
    with pytest.raises(Fault):
        body.command(op, {"crouch": mode})
    assert body.active is None and body.hardware.sent == 0


def test_drive_default_stands_after_stop(body):
    body.command("motion.drive", {"x": 1})
    # Expire the drive after preparation; missing crouch must mean off, not on.
    body.drive_deadline = 0
    finish(body)
    assert body.pose == "stand"
    assert body.engine is None


def test_heading_invalid_imu_fails_without_walking(body):
    body.pose = "crouch"
    body.hardware.body_quaternion = lambda: (0, 0, 0, 0)
    job = body.command("motion.drive", {"x": 1, "heading_hold": True})
    finish(body)
    assert body.jobs[job["job_id"]]["status"] == "failed"
    assert body.hardware.sent == 0
    assert body.pose == "unknown" and body.engine is None
