import asyncio
import math
from types import SimpleNamespace

import pytest

from roki_ng.body import Body
from roki_ng.client import Client
from roki_ng.body_imu import BodyImu, attitude
from roki_ng.parameters import Parameters
from roki_ng.stabilization import CrouchStabilizer
from roki_ng.supervisor import Supervisor, Session
from roki_ng.wire import Fault, envelope, pack


def quaternion(pitch):
    angle = (math.pi / 2 + math.radians(pitch)) / 2
    return math.sin(angle), 0, 0, math.cos(angle)


def test_correction_limit_accepts_ten_degrees_without_changing_default(tmp_path):
    params = Parameters(tmp_path)
    key = "stabilization.max_correction_deg"
    assert params.values[key] == 2.0
    assert params.describe(key)["max"] == 10
    params.set(key, 10.0)
    assert Parameters(tmp_path).values[key] == 10.0
    for value in (-0.1, 10.1):
        with pytest.raises(Fault):
            params.set(key, value)


@pytest.fixture
def rig(tmp_path, monkeypatch):
    now = [100.0]
    monkeypatch.setattr("time.monotonic", lambda: now[0])
    monkeypatch.setattr("time.monotonic_ns", lambda: int(now[0] * 1e9))
    events, sent = [], []
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *args: events.append(args), lambda *args: None)
    body.reconnect_at = float("inf")
    body.pose = "crouch"
    body.servo_targets = {(servo, bus): 7500 for servo, bus, *_ in body.model.ACTIVESERVOS}
    body.hardware.drained = lambda: True
    body.hardware.body_quaternion = lambda: quaternion(-5)
    body.hardware.send = lambda values, frames, pause: sent.append(
        ({(v.Id, v.Sio): v.Data for v in values}, frames, pause))
    body.command("control.acquire", {})
    return body, now, sent, events


def tick(rig, count=1):
    body, now, *_ = rig
    for _ in range(count):
        now[0] += 0.021
        body.tick()


@pytest.mark.parametrize("pitch", [-10, -5, 0, 5, 10])
def test_mounted_pitch_sign(pitch):
    assert attitude(quaternion(pitch)) == pytest.approx((pitch, 0))
    assert attitude(tuple(-v for v in quaternion(pitch))) == pytest.approx((pitch, 0))


def test_no_poll_or_movement_by_default(rig):
    body, _, sent, _ = rig
    tick(rig, 10)
    assert body.body_imu.sequence == 0 and not sent
    body.command("body.telemetry.watch", {"enabled": True})
    tick(rig, 10)
    assert body.body_imu.sequence == 10 and not sent
    body.command("body.telemetry.watch", {"enabled": False})
    tick(rig, 10)
    assert body.body_imu.sequence == 10


def test_invalid_and_busy_reads_are_not_disconnects(rig):
    body, _, sent, events = rig
    body.parameters["stabilization.enabled"] = True
    body.hardware.body_quaternion = lambda: (0, 0, 0, 0)
    tick(rig)
    assert body.body_connected and body.body_imu.invalid == 1
    assert body.stabilizer.reason == "imu_stale"

    def busy():
        raise Fault("body_busy", "busy")

    body.hardware.body_quaternion = busy
    tick(rig)
    assert body.body_connected and body.body_imu.busy == 1 and not sent
    assert not any(op == "body.connection" for op, _ in events)


def test_pitch_only_hips_nominal_targets_do_not_drift(rig):
    body, _, sent, _ = rig
    baseline = dict(body.servo_targets)
    body.parameters["stabilization.enabled"] = True
    tick(rig, 500)
    assert sent and body.servo_targets == baseline
    assert 0 < body.stabilizer.offset_deg < 1
    for targets, frames, pause in sent:
        assert set(targets) == {(7, 1), (7, 2)}
        assert targets[7, 1] - 7500 == -(targets[7, 2] - 7500)
        assert 1 <= frames <= 2 and pause == frames - 1
    assert body.sent_targets[7, 1] == 7500 + body.stabilizer.ticks(body.stabilizer.offset_deg)[7, 1]


def test_deadband_slew_saturation_and_no_integral(rig):
    body, now, _, _ = rig
    params = body.parameters | {"stabilization.pitch_kp": 1.0}
    imu, reg = BodyImu(), CrouchStabilizer()
    body.hardware.body_quaternion = lambda: quaternion(-10)
    for _ in range(300):
        now[0] += 0.02
        imu.read(body.hardware)
        proposed = reg.propose(imu, params, now[0])
        assert 0 <= proposed - reg.offset_deg <= 0.020001
        reg.commit(proposed)
    assert reg.offset_deg == 2 and reg.saturated
    assert reg.propose(imu, params, now[0]) is None  # No repeated stale feedback.
    body.hardware.body_quaternion = lambda: quaternion(0.2)
    for _ in range(300):
        now[0] += 0.02
        imu.read(body.hardware)
        reg.commit(reg.propose(imu, params, now[0]))
    assert reg.offset_deg == 0 and not reg.saturated


@pytest.mark.parametrize("reason", ["disabled", "stale", "tilted", "busy", "base_stand"])
def test_freeze_never_sends_zero_pose(rig, reason):
    body, _, sent, _ = rig
    body.parameters["stabilization.enabled"] = True
    tick(rig, 50)
    before = body.stabilizer.offset_deg
    sent.clear()
    if reason == "disabled":
        body.parameters["stabilization.enabled"] = False
    elif reason == "stale":
        body.body_imu.invalidate("test")
        body.body_imu.next_at = float("inf")
    elif reason == "tilted":
        body.hardware.body_quaternion = lambda: quaternion(-20)
    elif reason == "busy":
        body.hardware.drained = lambda: False
    else:
        body.pose = "base_stand"
    tick(rig, 100)
    assert not sent and body.stabilizer.offset_deg == before


def test_no_servo_command_after_link_loss_or_hard_stop(rig):
    body, _, sent, _ = rig
    body.parameters["stabilization.enabled"] = True
    tick(rig, 20)
    body.command("motion.stop_hard", {})
    sent.clear()
    tick(rig, 20)
    assert not sent and body.stabilizer.offset_deg == 0
    body._link_lost(Fault("hardware_error", "timeout"))
    body.reconnect_at = float("inf")
    tick(rig, 20)
    assert not sent and body.body_imu.quaternion is None


@pytest.mark.parametrize("direction", ["forward", "backward", "left", "right", "turn_left", "turn_right"])
def test_relative_jump_keeps_correction_and_hips_untouched(rig, direction):
    body, _, sent, _ = rig
    body.parameters["stabilization.enabled"] = True
    tick(rig, 40)
    before = body.stabilizer.offset_deg
    baseline = dict(body.servo_targets)
    sent.clear()
    body.command("motion.jump", {"direction": direction, "crouch": "on"})
    for _ in range(100):
        if not body.active:
            break
        tick(rig)
        assert body.stabilizer.offset_deg == before
    assert body.active is None and body.pose == "crouch"
    assert body.servo_targets == baseline
    assert sent and all((7, 1) not in targets and (7, 2) not in targets for targets, _, _ in sent)


def test_sending_nominal_gait_targets_applies_frozen_offset_once(rig):
    body, _, sent, _ = rig
    body.parameters["stabilization.enabled"] = True
    tick(rig, 40)
    offset = body.stabilizer.ticks(body.stabilizer.offset_deg)[7, 1]
    target = SimpleNamespace(Id=7, Sio=1, Data=7700)
    body._start("motion.drive", iter([("servo", [target], 2, 1)] * 10 + [("drain",)]))
    sent.clear()
    while body.active:
        tick(rig)
    assert len(sent) == 10
    assert all(values[7, 1] == 7700 + offset for values, _, _ in sent)
    assert target.Data == 7700 and body.servo_targets[7, 1] == 7700


def test_base_stand_handoff_has_no_extra_zero_correction_frame(rig):
    body, _, sent, _ = rig
    body.parameters["stabilization.enabled"] = True
    tick(rig, 40)
    body.command("motion.pose", {"name": "base_stand"})
    sent.clear()
    while body.active:
        tick(rig)
    assert body.pose == "base_stand" and body.stabilizer.offset_deg == 0
    assert len(sent) == 2  # Initial_Pose and head, no intermediate neutral hips.
    assert len(sent[0][0]) > 2
    tick(rig, 20)
    assert len(sent) == 2


def test_model_hip_limit_freezes_without_clamping_other_joints(rig):
    body, _, sent, _ = rig
    body.servo_targets[7, 1] = 7500 + 3851
    body.parameters["stabilization.enabled"] = True
    tick(rig, 10)
    assert not sent and body.stabilizer.reason == "joint_limit"


def test_waiting_for_interpolation_does_not_reset_filter(rig):
    body, now, _, _ = rig
    body.parameters["stabilization.enabled"] = True
    tick(rig)
    assert body.stabilizer.filtered_pitch == pytest.approx(-5)
    body.hardware.drained = lambda: False
    tick(rig)
    body.hardware.drained = lambda: True
    body.hardware.body_quaternion = lambda: quaternion(5)
    tick(rig)
    assert -5 < body.stabilizer.filtered_pitch < 0


def test_frozen_offset_cannot_push_motion_beyond_hip_limit(rig):
    body, _, sent, _ = rig
    body.stabilizer.offset_deg = 2
    value = SimpleNamespace(Id=7, Sio=1, Data=7500 + 3851)
    with pytest.raises(Fault, match="hip limit"):
        body._send_targets([value], 2, 1)
    assert not sent


def test_session_expiry_stops_control_before_disabling_telemetry(tmp_path):
    async def run():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path)})
        session = Session(1, 1, (), "owner")
        server.sessions[1] = session
        server.owner = session.id
        server.body_watch = True
        calls = []

        async def call(op, *args, **kwargs):
            calls.append(op)

        server.workers["motherboard"] = SimpleNamespace(call=call)
        await server._expire(session)
        assert calls == ["control.release", "body.telemetry.watch"]
        assert server.owner is None and not server.body_watch
    asyncio.run(run())


def test_native_ik_rigid_pitch_rotation_mainly_changes_hip(tmp_path):
    pytest.importorskip("starkit")
    from roki_ng.motion.engine import Engine
    params = Parameters(tmp_path).values
    for centered in (False, True):
        engine = Engine(params)
        baseline = engine.crouch_target(centered)
        old = {(v.Id, v.Sio): v.Data for v in baseline}
        sine, cosine = math.sin(math.radians(2)), math.cos(math.radians(2))
        for suffix in ("tr", "tl"):
            x, z = getattr(engine, "x" + suffix), getattr(engine, "z" + suffix)
            setattr(engine, "x" + suffix, cosine*x - sine*z)
            setattr(engine, "z" + suffix, sine*x + cosine*z)
        engine.xr = engine.xl = sine
        engine.zr = engine.zl = -cosine
        angles = engine.computeAlphaForWalk()
        assert angles
        changes = {}
        for angle, (servo, bus, sign, *_) in zip(angles, engine.ACTIVESERVOS):
            value = int(7500 + angle * 1698 * sign / (2 if servo == 8 else 1))
            changes[servo, bus] = value - old[servo, bus]
        expected = CrouchStabilizer.ticks(2)
        for key, delta in expected.items():
            assert changes[key] == pytest.approx(delta, abs=3)
        assert all(abs(changes[servo, bus]) <= 3 for servo in (8, 9) for bus in (1, 2))


def test_datastreams_are_requested_shared_fresh_and_fit_udp(tmp_path):
    async def run():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path)})
        calls = []

        async def call(op, body):
            calls.append((op, body))

        server.workers["motherboard"] = SimpleNamespace(alive=True, call=call)
        first = Session(1, 1, (), "one")
        second = Session(2, 2, (), "two")
        server.sessions = {1: first, 2: second}
        for session in (first, second):
            await server._data(session, "data.subscribe", {"topic": "body.imu"})
        assert calls == [("body.telemetry.watch", {"enabled": True, "imu": True, "power": False})]
        await server._data(first, "data.unsubscribe", {"subscription_id": "body.imu"})
        assert len(calls) == 1
        second.closed = True
        await server._sync_body_watch()
        assert calls[-1] == ("body.telemetry.watch", {"enabled": False, "imu": False, "power": False})
        assert not server._sample("body.imu")["valid"]
        imu = BodyImu()
        imu.read(SimpleNamespace(body_quaternion=lambda: quaternion(0)))
        import time
        data = {"body.imu": imu.state(time.monotonic())}
        server.worker_event("motherboard", "body.telemetry", data)
        sample = server._sample("body.imu")
        assert sample["valid"] and sample["source_mono_ns"] is not None
        pack(envelope("sample", "data.sample", sample))
        data["body.imu"]["source_mono_ns"] -= 1_000_000_000
        assert not server._sample("body.imu")["valid"]
        assert not server._sample("body.imu")["data"]["valid"]
    asyncio.run(run())


def test_body_telemetry_through_real_worker_and_udp(tmp_path):
    async def run():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path),
                             "host": "127.0.0.1", "port": 0})
        await server.start()
        client = Client("127.0.0.1", server.sock.getsockname()[1])
        try:
            await client.connect()
            assert {"body.imu", "body.stabilization"} <= {
                item["name"] for item in (await client.request("data.list"))["items"]}
            await client.request("data.subscribe", {"topic": "body.imu", "rate_hz": 10})
            await asyncio.sleep(0.25)
            assert server.body_telemetry["body.imu"]["sequence"] >= 2
            assert server._sample("body.imu")["valid"]
            assert server.owner is None  # Observing does not take control.
            sample = await client.request("data.snapshot", {"topic": "body.stabilization"})
            assert sample["valid"] and not sample["data"]["enabled"]
            await client.request("control.acquire")
            await client.request("params.set", {"key": "stabilization.enabled", "value": True})
            sample = await client.request("data.snapshot", {"topic": "body.stabilization"})
            assert sample["data"]["enabled"] and sample["data"]["offset_deg"] == 0
            await client.request("data.unsubscribe", {"subscription_id": "body.imu"})
            assert not server.body_watch
        finally:
            await client.close()
            await server.close()
    asyncio.run(run())
