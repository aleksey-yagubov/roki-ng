import math
import struct
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from roki_ng.body import Body, BodyTransmissionError, RokiHardware
from roki_ng.calibration import TestPlan as Plan
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


@pytest.fixture
def rig(tmp_path, body_clock):
    events, logs = [], []
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *args: events.append(args), lambda *args: logs.append(args))
    body.reconnect_at = math.inf
    body.command("control.acquire", {})
    return body, body_clock, events, logs


def quaternion(yaw):
    c, s = math.cos(yaw / 2) / math.sqrt(2), math.sin(yaw / 2) / math.sqrt(2)
    return c, s, s, c


def test_cache_consumers_do_not_issue_reads_and_publish_without_gui(rig):
    body, _, events, _ = rig
    body.hardware.body_quaternion = MagicMock(return_value=quaternion(.4))
    body.tick()
    for _ in range(10):
        assert body.read_body_quaternion() == quaternion(.4)
        assert body.command("body.telemetry.read", {})["body.imu"]["valid"]
        assert Plan(body, {"name": "run_test"}).yaw() == pytest.approx(.4)
    body.hardware.body_quaternion.assert_called_once()
    assert any(op == "body.telemetry" and data["body.imu"]["valid"] for op, data in events)
    assert not body.telemetry_watch and body.hardware.sent == 0


def test_manual_fall_stops_queue_without_getup_or_held_control_restart(rig):
    body, now, _, _ = rig
    body.pose = 'crouch'
    q = [quaternion(0)]
    body.hardware.body_quaternion = lambda: q[0]
    ident = body.command('motion.drive', {'x': 1})['job_id']
    for _ in range(30):
        body.command('motion.drive', {'x': 1})
        body.tick()
    assert body.jobs[ident].get('posture_verified') and body.active == ident
    q[0] = (0, 0, 0, 1)
    for _ in range(10): body.tick()
    assert body.jobs[ident]['status'] == 'failed'
    assert 'fell' in body.jobs[ident]['reason']
    assert body.pose == 'unknown' and body.plan is None and body.drive is None
    assert body.body_connected and not body.error and body.state()['important_errors_5s'] == 0
    q[0] = quaternion(0)
    now[0] += .05
    body.tick()
    with pytest.raises(Fault, match='Release walking controls'):
        body.command('motion.drive', {'x': 1})
    assert all(job['operation'] != 'motion.get_up' for job in body.jobs.values())
    body.command('motion.drive', {'x': 0})
    assert body.command('motion.drive', {'x': 1})['job_id'] != ident


def test_manual_start_lying_never_sends_preparation_or_getup(rig):
    body, _, _, _ = rig
    body.hardware.body_quaternion = lambda: (0, 0, 0, 1)
    ident = body.command('motion.drive', {'x': 1})['job_id']
    for _ in range(30): body.tick()
    assert body.jobs[ident]['status'] == 'failed' and body.hardware.sent == 0
    assert body.drive_rearm_required and not body.recovery_active


def test_manual_missing_imu_cancels_motion_without_body_fault(rig):
    body, now, _, _ = rig
    body.pose = 'crouch'
    ident = body.command('motion.drive', {'x': 1})['job_id']
    for _ in range(25):
        body.command('motion.drive', {'x': 1})
        body.tick()
    assert body.jobs[ident]['posture_verified']
    body.hardware.telemetry_ready = lambda: False
    now[0] += .2
    body.tick()
    assert body.jobs[ident]['status'] == 'failed' and body.drive_rearm_required
    assert body.body_connected and body.state()['important_errors_5s'] == 0


def test_manual_single_fallen_sample_is_not_a_confirmed_fall(rig):
    body, now, _, _ = rig
    body.pose = 'crouch'
    body.command('motion.drive', {'x': 1})
    for _ in range(25):
        body.command('motion.drive', {'x': 1})
        body.tick()
    q = [0, 0, 0, 1]
    body.hardware.body_quaternion = lambda: q
    now[0] += .02
    body.tick()
    assert body.fall_samples == 1 and body.active
    body.hardware.telemetry_ready = lambda: False
    body.tick()  # The same cached quaternion is not a second observation.
    assert body.fall_samples == 1 and body.active
    body.hardware.telemetry_ready = lambda: True
    q[:] = quaternion(0)
    now[0] += .02
    body.tick()
    assert body.fall_samples == 0 and body.active


@pytest.mark.parametrize("angle", [-7.2, 7.2])
def test_whole_turns_accumulate_even_without_test_reads(rig, angle):
    body, now, _, _ = rig
    measured = [0.0]
    body.hardware.body_quaternion = lambda: quaternion(measured[0])
    body.tick()
    plan = Plan(body, {"name": "rotation_test"})
    initial = plan.yaw()
    for index in range(1, 81):
        measured[0] = angle * index / 80
        now[0] += .02
        body.tick()
    assert plan.yaw() - initial == pytest.approx(angle)
    assert body.body_imu.sequence == 81


def test_lost_polling_continuity_cannot_save_rotation(rig):
    body, now, _, _ = rig
    body.tick()
    plan = Plan(body, {"name": "rotation_test"})
    plan.yaw()
    now[0] += .2
    with pytest.raises(Fault, match="fresh polled"):
        body.read_body_quaternion()
    body.tick()
    with pytest.raises(Fault) as error:
        plan.yaw()
    assert error.value.code == "imu_discontinuous"


def test_poll_failures_are_logged_but_do_not_enter_important_budget(rig):
    body, now, _, logs = rig
    body.command("body.telemetry.watch", {"enabled": True, "servos": True, "power": True})
    for method in ("body_quaternion", "body_positions", "body_power_adc"):
        setattr(body.hardware, method, MagicMock(side_effect=Fault("hardware_error", method)))
    for _ in range(5):
        now[0] += 1.01
        body.tick()
    assert body.body_connected and not body.error
    assert body.state()["important_errors_5s"] == 0
    assert len(logs) == 16  # Initial info plus every failed read (5 x 3).
    assert all(level == "WARNING" for level, _ in logs[1:])
    assert not body.body_imu.state(now[0])["valid"]
    assert not body.body_servos.state(now[0])["valid"]


def test_three_failed_movements_not_three_requests_and_no_replay(rig):
    body, now, _, logs = rig
    body.hardware.send = MagicMock(side_effect=Fault("hardware_error", "send uncertain"))
    for failed in range(1, 4):
        job = body.command("motion.jump", {})["job_id"]
        body.tick()
        assert body.jobs[job]["status"] == "failed"
        assert body.active is body.plan is None
        assert body.pose == "unknown"
        assert body.body_connected == (failed < 3)
        assert body.hardware.send.call_count == failed
        if failed < 3:
            for _ in range(10):
                now[0] += .02
                body.tick()  # Successful polling never clears important failures.
            assert body.state()["important_errors_5s"] == failed
    assert body.error == "send uncertain"
    assert len([msg for _, msg in logs if "Important body operation failed" in msg]) == 3


def test_error_window_expires_and_telemetry_does_not_reset_it(rig):
    body, now, _, _ = rig
    body.hardware.head = MagicMock(side_effect=Fault("hardware_error", "head failed"))
    def fail_head():
        with pytest.raises(Fault):
            body.command("motion.head", {})
    fail_head()
    now[0] += 4.9
    fail_head()
    assert body.state()["important_errors_5s"] == 2
    now[0] += .11
    fail_head()
    assert body.body_connected and body.state()["important_errors_5s"] == 2
    fail_head()
    assert not body.body_connected


def test_unconfirmed_queue_reset_blocks_motion_before_threshold(rig):
    body, now, _, _ = rig
    body.hardware.send = MagicMock(side_effect=Fault("hardware_error", "send uncertain"))
    body.hardware.reset = MagicMock(side_effect=Fault("hardware_error", "reset unconfirmed"))
    body.command("motion.jump", {})
    body.tick()
    assert body.body_connected and body.queue_reset_pending
    assert body.state()["important_errors_5s"] == 2
    with pytest.raises(Fault, match="reset not confirmed"):
        body.command("motion.jump", {})
    body.hardware.reset.side_effect = None
    now[0] += .1
    body.tick()
    assert not body.queue_reset_pending and body.body_connected
    assert body.hardware.send.call_count == 1


def test_busy_is_rejection_not_uncertain_execution(rig):
    body, now, _, logs = rig
    body.hardware.send = MagicMock(side_effect=[Fault("body_busy", "busy", True), None])
    job = body.command("motion.jump", {})["job_id"]
    body.tick()
    pending = body.instruction
    assert pending[0] == "servo" and body.jobs[job]["status"] == "running"
    now[0] += .03
    body.tick()
    assert body.hardware.send.call_count == 2
    assert body.hardware.send.call_args_list[0] == body.hardware.send.call_args_list[1]
    assert body.state()["important_errors_5s"] == 0
    assert any(level == "WARNING" and "busy" in msg for level, msg in logs)


def test_single_report_can_contain_three_failed_queue_transactions(rig):
    body, _, _, _ = rig
    body.hardware.check_queue = MagicMock(side_effect=BodyTransmissionError("3 queued failures", 3))
    body.tick()
    assert not body.body_connected


def test_nested_stop_does_not_double_count_one_failure(rig):
    body, _, _, _ = rig
    body.hardware.reset = MagicMock(side_effect=[Fault("hardware_error", "reset failed"), None])
    with pytest.raises(Fault):
        body.command("control.takeover", {})
    assert body.state()["important_errors_5s"] == 1


@pytest.mark.parametrize("status", [8, 9, 10])
def test_failed_telemetry_is_not_rediscovered_as_queue_failure(status):
    hardware = object.__new__(RokiHardware)
    state = dict(body_failures=0, body_queue_size=0, body_busy=False, last_body_error=0)
    hardware.body_failures = 0
    hardware.mb = SimpleNamespace(GetStatus=lambda: dict(state), GetLastStatus=lambda: status)
    def failed_read(*args):
        state.update(body_failures=state["body_failures"] + 1, last_body_error=status)
        return False, b""
    hardware.rcb = SimpleNamespace(moveRamToComCmdSynchronize=failed_read, GetError=lambda: "read failed")
    with pytest.raises(Fault):
        hardware.body_quaternion()
    hardware.check_queue()  # This transaction belongs to telemetry, not movement.
    state.update(body_failures=3, last_body_error=0)
    with pytest.raises(BodyTransmissionError) as error:
        hardware.check_queue()
    assert error.value.count == 2  # Do not hide later queued execution errors.


def test_polling_waits_for_uart_but_not_servo_interpolation():
    hardware = object.__new__(RokiHardware)
    state = dict(body_failures=0, body_queue_size=1, body_busy=False, last_body_error=0)
    hardware.body_failures = 0
    hardware.busy_until = math.inf
    hardware.mb = SimpleNamespace(GetStatus=lambda: dict(state))
    hardware.rcb = MagicMock()
    hardware.rcb.moveRamToComCmdSynchronize.return_value = True, struct.pack("<4h", 0, 0, 0, 16384)
    assert not hardware.telemetry_ready()
    with pytest.raises(Fault):
        hardware.body_quaternion()
    hardware.rcb.moveRamToComCmdSynchronize.assert_not_called()
    state.update(body_queue_size=0, body_busy=True)
    assert not hardware.telemetry_ready()
    state["body_busy"] = False
    assert hardware.telemetry_ready()
    assert hardware.body_quaternion() == (0, 0, 0, 1)


def test_lost_poll_reply_is_resolved_before_new_movement():
    hardware = object.__new__(RokiHardware)
    state = dict(body_failures=0, body_queue_size=0, body_busy=False, last_body_error=0)
    hardware.body_failures = 0
    hardware.mb = SimpleNamespace(GetStatus=lambda: dict(state), GetLastStatus=lambda: 255)
    def lost_reply(*args):
        state["body_busy"] = True
        return False, b""
    hardware.rcb = SimpleNamespace(moveRamToComCmdSynchronize=lost_reply, GetError=lambda: "ACM timeout")
    with pytest.raises(Fault):
        hardware.body_quaternion()
    hardware.check_queue()
    with pytest.raises(Fault) as error:
        hardware.send([], 1, 0)
    assert error.value.code == "body_busy"
    state.update(body_busy=False, body_failures=1, last_body_error=8)
    hardware.check_queue()
    assert not hardware.forward_pending and hardware.body_failures == 1
    state["body_failures"] = 2
    with pytest.raises(BodyTransmissionError) as error:
        hardware.check_queue()
    assert error.value.count == 1


def test_cancelled_game_walk_is_not_completed_motion(rig):
    body, _, _, _ = rig
    body.pose = "base_stand"
    job = body.command("game.walk", {"kind": "omni", "distance_m": .2, "course": math.pi/2})["job_id"]
    body.command("motion.stop_graceful", {})
    for _ in range(10):
        body.tick()
    assert body.jobs[job]["status"] == "cancelled"
    assert body.hardware.sent == 0
