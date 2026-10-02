import asyncio
import json
import math
import struct
from unittest.mock import MagicMock

import pytest

from roki_ng.body import Body, RokiHardware
from roki_ng.calibration import (ROTATION_KEYS, TITLES, TestPlan as Plan, describe,
                                 quaternion_yaw, rotation_results, validate_args)
from roki_ng.client import Client
from roki_ng.parameters import Parameters
from roki_ng.supervisor import Supervisor
from roki_ng.wire import Fault, envelope, pack


def make_body(tmp_path):
    events = []
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda op, value: events.append((op, value)), lambda *a: None)
    body.command("control.acquire", {})
    body.hardware.drained = lambda: True
    return body, events


def finish(body):
    for _ in range(10000):
        if not body.active:
            return
        body.next_at = 0
        body.tick()
    raise AssertionError("Test plan did not terminate")


def test_catalog_and_descriptions_fit_udp():
    assert set(TITLES) == {"run_test", "jump_test", "rotation_test", "kick_test", "get_up_test"}
    for name in TITLES:
        pack(envelope("response", "test.describe", {"result": describe(name)}))
    assert describe("run_test")["parameters"]["mode"]["choices"] == [
        "short", "long", "spot", "backwards", "side_left", "side_right", "custom"]
    with pytest.raises(Fault, match="slot 31"):
        validate_args({"name": "kick_test", "mode": "new_kick"})
    with pytest.raises(Fault, match="mode=custom"):
        validate_args({"name": "run_test", "mode": "short", "cycles": 7})
    with pytest.raises(Fault):
        validate_args({"name": "jump_test", "count": 0})


@pytest.mark.parametrize("mode,cycles,step_sign,side_sign,right_first", [
    ("short", 11, 1, 0, True), ("long", 21, 1, 0, True),
    ("spot", 21, 0, 0, True), ("backwards", 21, -1, 0, True),
    ("side_left", 20, 0, 1, False), ("side_right", 20, 0, 1, True),
    ("custom", 3, -1, 1, False)])
def test_run_variants_calculate_and_send_requested_motion(
        tmp_path, monkeypatch, mode, cycles, step_sign, side_sign, right_first):
    pytest.importorskip("starkit")
    from roki_ng.motion.engine import Engine

    body, _ = make_body(tmp_path)
    body.simulated = False
    trajectories, sent = [], []
    calculate = Engine.walk_Cycle
    send = body.hardware.send

    def walk(engine, step, side, yaw, *args):
        trajectories.append((step, side, engine.first_Leg_Is_Right_Leg))
        commands = list(calculate(engine, step, side, yaw, *args))
        assert any(command[0] == "servo" for command in commands), "Cycle has no servo commands"
        yield from commands

    def record(values, frames, pause):
        sent.append({(v.Id, v.Sio): v.Data for v in values})
        send(values, frames, pause)

    monkeypatch.setattr(Engine, "walk_Cycle", walk)
    body.hardware.send = record
    args = {"name": "run_test", "mode": mode}
    if mode == "custom":
        args.update(cycles=3, step_mm=-12, side_mm=5, right_leg=False)
    result = body.command("test.start", args)
    finish(body)
    job = body.jobs[result["job_id"]]
    assert job["status"] == "completed"
    assert job["progress"] == cycles
    assert len(trajectories) == cycles
    sign = lambda value: (value > 0) - (value < 0)
    assert all((sign(step), sign(side), right) == (step_sign, side_sign, right_first)
               for step, side, right in trajectories)
    # Real IK output reaches the hardware boundary, not just a progress counter.
    knees = [frame[8, 1] for frame in sent if (8, 1) in frame]
    assert len(set(knees)) > 1
    assert all(0 <= value <= 16383 for frame in sent for value in frame.values())
    assert body.pose == "stand"


@pytest.mark.parametrize("direction,joint,sign", [
    # Kondo wire directions for the right ankle/hip, after mounting inversion.
    ("forward", 9, -1), ("backward", 9, 1),
    ("left", 6, 1), ("right", 6, -1), ("on_spot", 9, 0)])
def test_jump_variants_send_directional_targets(tmp_path, direction, joint, sign):
    body, _ = make_body(tmp_path)
    sent = []
    body.hardware.send = lambda values, *_: sent.append({(v.Id, v.Sio): v.Data for v in values})
    result = body.command("test.start", {"name": "jump_test", "direction": direction, "count": 2})
    finish(body)
    assert body.jobs[result["job_id"]]["status"] == "completed"
    assert body.jobs[result["job_id"]]["progress"] == 2
    leg_frames = [frame for frame in sent if (joint, 1) in frame]
    assert leg_frames, "No leg commands were sent"
    offset = leg_frames[0][joint, 1] - 7500
    assert (offset > 0) - (offset < 0) == sign
    assert any(frame.get((10, 1), 7500) != 7500 for frame in leg_frames), "No jump impulse"
    assert all(value == 7500 for (servo, _), value in leg_frames[-1].items()
               if servo in (5, 6, 7, 8, 9, 10, 13))


def test_cancel_does_not_save(tmp_path):
    body, events = make_body(tmp_path)
    result = body.command("test.start", {"name": "rotation_test"})
    body.tick()
    body.command("job.cancel", {"job_id": result["job_id"]})
    finish(body)
    assert body.jobs[result["job_id"]]["status"] == "cancelled"
    assert not any(op == "calibration.ready" for op, _ in events)


def test_real_gait_plan_uses_ramp_and_imu_without_hardware(tmp_path):
    body, _ = make_body(tmp_path)
    body.simulated = False
    plan = Plan(body, {"name": "run_test", "mode": "short"})
    plan.origin = 0
    plan.yaw = lambda: 0.1
    calls = []

    class Engine:
        def walk_Initial_Pose(self, **args):
            calls.append(("initial", args))
            yield "drain",

        def walk_Cycle(self, *args):
            calls.append(("cycle", args))
            yield "drain",

        def walk_Final_Pose(self):
            calls.append(("final", {}))
            yield "drain",

    plan.engine = Engine()
    result = body._start("test.start", plan.gait(11, step=64, ramp=True))
    finish(body)
    assert body.jobs[result["job_id"]]["status"] == "completed"
    cycles = [args for kind, args in calls if kind == "cycle"]
    assert len(cycles) == 11
    assert [row[0] for row in cycles[:3]] == pytest.approx([64/3, 128/3, 64])
    assert all(row[2] == pytest.approx(-0.11) for row in cycles)
    assert calls[0] == ("initial", {"start_mixing": False})
    assert calls[-1][0] == "final"


def test_jump_correction_uses_measured_coefficient(tmp_path, monkeypatch):
    body, _ = make_body(tmp_path)
    plan = Plan(body, {"name": "jump_test"})
    plan.origin = 0
    plan.parameters["motion.jump_yaw_ccw"] = 0.4
    values = iter([-0.2, 0])
    plan.yaw = lambda: next(values)
    calls = []

    def jump(direction, fraction=1):
        calls.append((direction, fraction))
        yield "drain",

    monkeypatch.setattr(plan, "jump", jump)
    list(plan.correct_course())
    assert calls[0][0] == "turn_left"
    assert calls[0][1] == pytest.approx(0.5)


def test_bad_imu_fails_before_movement(tmp_path):
    body, events = make_body(tmp_path)
    body.hardware.body_quaternion = lambda: (0, 0, 0, 0)
    result = body.command("test.start", {"name": "run_test"})
    finish(body)
    assert body.jobs[result["job_id"]]["status"] == "failed"
    assert body.hardware.sent == 0
    assert not any(op == "calibration.ready" for op, _ in events)


def test_quaternion_protocol_and_wrap():
    hardware = object.__new__(RokiHardware)
    hardware.rcb = MagicMock()
    hardware.rcb.moveRamToComCmdSynchronize.return_value = (True, list(struct.pack("<hhhh", 0, 0, 0, 16384)))
    assert hardware.body_quaternion() == (0, 0, 0, 1)
    hardware.rcb.moveRamToComCmdSynchronize.assert_called_once_with(0x60, 8)
    assert quaternion_yaw((0, 0, 0, 1)) == 0
    for bad in [(0, 0, 0, 0), (math.nan, 0, 0, 1)]:
        with pytest.raises(Fault):
            quaternion_yaw(bad)
    angles = iter([3.1, -3.1, -3])
    fake_body = MagicMock(parameters={})
    fake_body.read_body_quaternion.side_effect = lambda: (
        lambda a: (0, 0, math.sin(a/2), math.cos(a/2)))(next(angles))
    plan = Plan(fake_body, {"name": "run_test"})
    assert plan.yaw() == pytest.approx(3.1)
    assert plan.yaw() == pytest.approx(2*math.pi-3.1)
    assert plan.yaw() == pytest.approx(2*math.pi-3)


def test_rotation_values_are_atomic(tmp_path):
    result = rotation_results(-1.2, 1.5, -2.3, 2.1)
    assert result == dict(zip(ROTATION_KEYS, [-0.4, 0.5, 0.23, 0.21]))
    for values in [(0, 1.5, -2.3, 2.1), (1.2, 1.5, -2.3, 2.1), (-1.2, 1.5, -2.3, -2.1)]:
        with pytest.raises(Fault):
            rotation_results(*values)
    params = Parameters(tmp_path)
    params.set_many(result)
    before = params.path.read_bytes()
    with pytest.raises(Fault):
        params.set_many(result | {"motion.rotation_yield_right": 0})
    assert params.path.read_bytes() == before
    assert Parameters(tmp_path).values["motion.jump_yaw_cw"] == -0.4


def test_rotation_plan_and_commit_barrier(tmp_path, monkeypatch):
    body, events = make_body(tmp_path)
    yaw = [0.0]
    body.hardware.body_quaternion = lambda: tuple(v * math.sqrt(0.5) for v in (
        math.cos(yaw[0]/2), math.sin(yaw[0]/2), math.sin(yaw[0]/2), math.cos(yaw[0]/2)))
    original_jump = Plan.jump

    def jump(self, direction, *args, **kwargs):
        yield from original_jump(self, direction, *args, **kwargs)
        yaw[0] += -0.4 if direction == "turn_right" else 0.5

    gait_calls = []

    def gait(self, cycles, **kwargs):
        assert cycles == 10
        gait_calls.append(kwargs)
        yaw[0] += -2.3 if kwargs["right_leg"] else 2.1
        yield "drain",

    targets = []
    baselines = []
    original_init = Plan.__init__

    def init(self, *args):
        original_init(self, *args)
        self.engine = MagicMock()
        self.engine.configure.side_effect = lambda values: baselines.append(dict(values))

    def turn(self, target):
        targets.append(target)
        yaw[0] = target
        yield "drain",

    monkeypatch.setattr(Plan, "jump", jump)
    monkeypatch.setattr(Plan, "__init__", init)
    monkeypatch.setattr(Plan, "gait", gait)
    monkeypatch.setattr(Plan, "turn_to_course", turn)
    result = body.command("test.start", {"name": "rotation_test"})
    finish(body)
    job = body.jobs[result["job_id"]]
    assert job["status"] == "saving"
    assert targets == pytest.approx([2*math.pi/3, 0])
    assert baselines[0]["motion.rotation_yield_right"] == 0.23
    assert baselines[0]["motion.rotation_yield_left"] == 0.23
    assert baselines[-1]["motion.rotation_yield_left"] == 0.21
    assert gait_calls == [
        {"right_leg": True, "rotation": -0.23},
        {"right_leg": False, "rotation": -0.23},
    ]
    assert job["calibration"] == dict(zip(ROTATION_KEYS, [-0.4, 0.5, 0.23, 0.21]))
    with pytest.raises(Fault, match="saved"):
        body.command("test.start", {"name": "run_test"})
    body.command("calibration.resolve", {"job_id": job["job_id"], "error": None})
    assert job["status"] == "completed" and job["saved"]
    assert body.parameters["motion.jump_yaw_cw"] == -0.4


@pytest.mark.parametrize("disk_failure", [False, True])
def test_supervisor_automatic_calibration_save(tmp_path, monkeypatch, disk_failure):
    async def exercise():
        from unittest.mock import AsyncMock
        supervisor = Supervisor({"simulate": True, "state_dir": str(tmp_path)})
        body, _ = make_body(tmp_path / "worker-fixture")
        values = rotation_results(-1.2, 1.5, -2.3, 2.1)
        body.jobs["cal"] = {"job_id": "cal", "operation": "test.start", "status": "saving", "calibration": values}
        body.calibration_pending = "cal"
        worker = MagicMock()
        worker.call = AsyncMock(side_effect=lambda op, args: body.command(op, args))
        supervisor.workers["motherboard"] = worker
        before = supervisor.params.path.read_bytes()
        if disk_failure:
            def fail(_):
                raise OSError("disk full")
            monkeypatch.setattr(supervisor.params, "save", fail)
        await supervisor._save_calibration({"job_id": "cal", "values": values})
        job = body.jobs["cal"]
        assert job["saved"] is not disk_failure
        assert body.calibration_pending is None
        if disk_failure:
            assert job["status"] == "failed"
            assert supervisor.params.path.read_bytes() == before
            assert body.parameters["motion.jump_yaw_cw"] != values["motion.jump_yaw_cw"]
        else:
            assert job["status"] == "completed"
            assert all(Parameters(tmp_path).values[k] == v for k, v in values.items())
            assert all(body.parameters[k] == v for k, v in values.items())
    asyncio.run(exercise())


def test_hard_stop_cannot_save_partial_test(tmp_path):
    body, events = make_body(tmp_path)
    result = body.command("test.start", {"name": "rotation_test"})
    body.jobs[result["job_id"]]["calibration"] = rotation_results(-1.2, 1.5, -2.3, 2.1)
    body.command("motion.stop_hard", {})
    job = body.jobs[result["job_id"]]
    assert job["status"] == "cancelled"
    assert "calibration" not in job
    assert not any(op == "calibration.ready" for op, _ in events)


def test_operator_measurement_round_trip(tmp_path):
    async def exercise():
        supervisor = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await supervisor.start()
        client = Client("127.0.0.1", supervisor.sock.getsockname()[1])
        try:
            await client.connect()
            assert (await client.request("test.list"))["items"] == list(TITLES)
            for name in TITLES:
                assert (await client.request("test.describe", {"name": name}))["name"] == name
            await client.request("control.acquire")
            await client.request("mode.set", {"mode": "MANUAL"})
            job = await client.request("test.start", {"name": "run_test", "mode": "short"})
            for _ in range(100):
                status = await client.request("job.status", job)
                if status["status"] != "running":
                    break
                await asyncio.sleep(0.03)
            assert status["status"] == "completed"
            await client.request("test.measure", {"job_id": job["job_id"], "values": {"motion.run_10_mm": 987}})
            assert json.loads((tmp_path / "parameters.json").read_text())["motion.run_10_mm"] == 987
            with pytest.raises(Fault):
                await client.request("test.measure", {"job_id": job["job_id"], "values": {"walk.body_tilt_forward": 0.1}})
        finally:
            await client.close()
            await supervisor.close()
    asyncio.run(exercise())
