import math

import pytest

from roki_ng.body import Body
from roki_ng.calibration import describe
from roki_ng.parameters import Parameters
from roki_ng.recovery import body_position, SLOTS
from roki_ng.wire import Fault


S = math.sqrt(0.5)
POSITIONS = {
    "upright": (S, 0, 0, S),
    "back": (0, 0, 0, 1),
    "stomach": (1, 0, 0, 0),
    "right": (0, -S, 0, S),
    "left": (0, S, 0, S),
}


def fixture(tmp_path, initial, final="upright"):
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *a: None, lambda *a: None)
    body.command("control.acquire", {})
    body.hardware.drained = lambda: True
    samples = iter([POSITIONS[initial]] * 3)
    body.hardware.body_quaternion = lambda: next(samples, POSITIONS[final])
    return body


def finish(body):
    for _ in range(10000):
        if not body.active:
            return
        body.next_at = 0
        body.tick()
    raise AssertionError("Recovery did not terminate")


@pytest.mark.parametrize("position", POSITIONS)
def test_mounted_imu_axes(position):
    assert body_position(POSITIONS[position]) == position
    assert body_position(tuple(-v for v in POSITIONS[position])) == position


def test_upright_is_independent_of_yaw_and_bad_data_rejected():
    for angle in (-3, -1, 0, 1, 3):
        c, s = math.cos(angle/2), math.sin(angle/2)
        assert body_position((S*c, S*s, S*s, S*c)) == "upright"
    for q in ((0, 0, 0, 0), (math.nan, 0, 0, 1)):
        with pytest.raises(Fault) as exc:
            body_position(q)
        assert exc.value.code == "imu_invalid"
    # Inverted and intermediate postures must not select a random recovery slot.
    for q in ((-S, 0, 0, S), (math.sin(0.4), 0, 0, math.cos(0.4))):
        with pytest.raises(Fault) as exc:
            body_position(q)
        assert exc.value.code == "posture_uncertain"


@pytest.mark.parametrize("position", SLOTS)
def test_single_recovery_slot_then_stand_check(tmp_path, position):
    body = fixture(tmp_path, position)
    result = body.command("test.start", {"name": "get_up_test"})
    finish(body)
    job = body.jobs[result["job_id"]]
    assert job["status"] == "completed"
    assert job["recovery_slot"] == SLOTS[position] and job["posture_verified"]
    assert job["initial_posture"] == position
    assert body.hardware.sent > 0
    assert not body.recovery_active
    assert body.pose == "stand" and not job.get("calibration")


def test_already_upright_is_noop(tmp_path):
    body = fixture(tmp_path, "upright")
    result = body.command("test.start", {"name": "get_up_test"})
    finish(body)
    assert body.jobs[result["job_id"]]["status"] == "completed"
    assert body.hardware.sent == 0
    assert "recovery_slot" not in body.jobs[result["job_id"]]


@pytest.mark.parametrize("name", ["run_test", "jump_test", "rotation_test", "kick_test", "get_up_test"])
def test_invalid_imu_blocks_every_test_before_servo_commands(tmp_path, name):
    body = fixture(tmp_path, "upright")
    body.hardware.body_quaternion = lambda: (0, 0, 0, 0)
    result = body.command("test.start", {"name": name})
    finish(body)
    assert describe(name)["body_imu"]
    assert body.jobs[result["job_id"]]["status"] == "failed"
    assert body.hardware.sent == 0


def test_failed_getup_does_not_walk_or_retry(tmp_path):
    body = fixture(tmp_path, "back", "back")
    result = body.command("test.start", {"name": "run_test"})
    finish(body)
    job = body.jobs[result["job_id"]]
    assert job["status"] == "failed" and "no automatic retry" in job["reason"]
    assert job["progress"] == 0
    assert job["stage"] == "getting_up"
    assert not body.recovery_active


def test_successful_getup_precedes_walk(tmp_path):
    body = fixture(tmp_path, "stomach")
    result = body.command("test.start", {"name": "run_test", "mode": "short"})
    finish(body)
    job = body.jobs[result["job_id"]]
    assert job["status"] == "completed" and job["progress"] == 11
    assert job["posture_verified"] and job["recovery_slot"] == SLOTS["stomach"]


@pytest.mark.parametrize("stop", ["job.cancel", "control.release", "control.takeover"])
def test_cancel_during_getup_discards_remaining_rows(tmp_path, stop):
    body = fixture(tmp_path, "back")
    result = body.command("test.start", {"name": "run_test"})
    while not body.recovery_active:
        body.next_at = 0
        body.tick()
        assert body.active
    sent = body.hardware.sent
    body.command(stop, {"job_id": result["job_id"]})
    finish(body)
    assert body.jobs[result["job_id"]]["status"] == "cancelled"
    assert body.hardware.sent == sent
    assert not body.recovery_active and body.pose == "unknown"


def test_changing_posture_fails_without_movement(tmp_path):
    body = fixture(tmp_path, "back")
    samples = iter([POSITIONS[p] for p in ("back", "upright", "back")])
    body.hardware.body_quaternion = lambda: next(samples)
    result = body.command("test.start", {"name": "get_up_test"})
    finish(body)
    assert body.jobs[result["job_id"]]["status"] == "failed"
    assert body.hardware.sent == 0
