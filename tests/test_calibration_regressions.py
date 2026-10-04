import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from roki_ng.body import Body
from roki_ng.motion.engine import Engine
from roki_ng.parameters import Parameters
from roki_ng.supervisor import Supervisor, Session
from roki_ng.wire import pack, envelope


@pytest.mark.parametrize("right_first,rotation,coefficient", [
    (True, -.1, "right"), (False, -.1, "left"),
    (True, .1, "left"), (False, .1, "right")])
def test_yield_follows_physical_direction_not_signed_argument(
        tmp_path, right_first, rotation, coefficient):
    params = Parameters(tmp_path).values
    def amplitude(values):
        engine = Engine(values)
        engine.first_Leg_Is_Right_Leg = right_first
        angles = []
        def capture(hands_on=True):
            angles.append(engine.wr)
            return [0.] * 21
        engine.computeAlphaForWalk = capture
        list(engine.walk_Cycle(0, 0, rotation, 1, 3))
        return max(abs(angle) for angle in angles)
    baseline = amplitude(params)
    assert baseline > 0
    changed = params | {f"motion.rotation_yield_{coefficient}": .46}
    assert amplitude(changed) == pytest.approx(baseline / 2)
    other = "left" if coefficient == "right" else "right"
    assert amplitude(params | {f"motion.rotation_yield_{other}": .46}) == baseline


def test_spot_measurement_adds_captured_correction_and_is_idempotent(tmp_path):
    async def exercise():
        s = Supervisor({"simulate": True, "state_dir": str(tmp_path)})
        session = Session(1, 2, (), "operator")
        s.owner, s.lease_epoch = 1, 3
        params = s.params.values | {"motion.shift_x_mm": 100, "motion.shift_y_mm": -50}
        body = Body({"simulate": True, "parameters": params}, lambda *args: None, lambda *args: None)
        body.command("control.acquire", {})
        response = body.command("test.start", {"name": "run_test", "mode": "spot"})
        job = body.jobs[response["job_id"]]
        assert job["measurement_baseline"] == {"motion.shift_x_mm": 100, "motion.shift_y_mm": -50}
        job["status"] = "completed"  # The measurement handler consumes only completed results.
        s.workers["motherboard"] = SimpleNamespace(call=AsyncMock(return_value=job))
        for _ in range(2):
            result = await s.dispatch(session, "test.measure", {
                "job_id": job["job_id"], "lease_epoch": 3,
                "values": {"motion.shift_x_mm": 35, "motion.shift_y_mm": -18}})
            assert result["saved"] == {"motion.shift_x_mm": 135, "motion.shift_y_mm": -68}
        assert Parameters(tmp_path).values["motion.shift_x_mm"] == 135
    asyncio.run(exercise())


def test_fall_during_test_cancels_remaining_motion_and_does_not_latch_fault(tmp_path, body_clock):
    events = []
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *args: events.append(args), lambda *args: None)
    body.command("control.acquire", {})
    job = body.command("test.start", {"name": "jump_test", "count": 10})["job_id"]
    for _ in range(2000):
        body.tick()
        if body.jobs[job]["progress"] >= 1:
            break
    assert body.jobs[job]["status"] == "running"
    body.hardware.body_quaternion = lambda: (0, 0, 0, 1)  # Lying on back.
    for _ in range(20):
        body.tick()
        if not body.active:
            break
    assert body.jobs[job]["status"] == "failed" and "fell" in body.jobs[job]["reason"]
    assert body.body_connected and not body.error and body.pose == "unknown"
    sent = body.hardware.sent
    for _ in range(50):
        body.tick()
    assert body.hardware.sent == sent
    assert not any(op == "calibration.ready" for op, _ in events)
    body.command("motion.get_up", {})  # Failure must not lock out recovery.


def test_motion_status_errors_fit_udp_budget(tmp_path):
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *args: None, lambda *args: None)
    body.tick()
    body.error = body.imu_error = body.last_important_error = "x" * 240
    body.body_imu.invalidate("x" * 240)
    pack(envelope("response", "motion.state", {"result": body.state()}))
