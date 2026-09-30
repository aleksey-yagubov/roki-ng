import asyncio
import struct
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from roki_ng.body import Body, RokiHardware
from roki_ng.body_servos import target_zubr
from roki_ng.client import Client
from roki_ng.parameters import Parameters
from roki_ng.supervisor import Supervisor
from roki_ng.wire import Fault, envelope, pack


@pytest.fixture
def body(tmp_path):
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *a: None, lambda *a: None)
    body.reconnect_at = float("inf")
    return body


def test_native_reads_all_thirty_units_including_lower_knees():
    hardware = object.__new__(RokiHardware)
    hardware.rcb = MagicMock()
    expected = tuple(-16000 + i*1000 for i in range(30))
    hardware.rcb.moveRamToComCmdSynchronize.return_value = (True, struct.pack("<30h", *expected))
    assert hardware.body_positions() == expected
    hardware.rcb.moveRamToComCmdSynchronize.assert_called_once_with(0x70, 60)
    hardware.rcb.moveRamToComCmdSynchronize.return_value = (True, b"\0" * 58)
    with pytest.raises(Fault, match="60 bytes"):
        hardware.body_positions()


@pytest.mark.parametrize("wire,expected", [(7500, 0), (7501, 1), (7499, -1), (11500, 6144), (3500, -6144)])
def test_conversion_matches_controller_signed_division(wire, expected):
    assert target_zubr(wire, False) == expected
    assert target_zubr(wire, True) == -expected


def test_catalog_maps_logical_ids_not_physical_motor_ids(body):
    assert len(body.body_servos.catalog) == 30
    expected = {14: "right_hip", 15: "left_hip", 16: "right_knee", 17: "left_knee",
                18: "right_foot_front", 19: "left_foot_front", 25: "head_tilt",
                26: "right_knee_bot", 27: "left_knee_bot"}
    for unit, name in expected.items():
        assert body.body_servos.catalog[unit]["name"] == name
    assert body.body_servos.catalog[24]["name"] is None
    assert body.command("motion.joints", {"offset": 24})["items"][2]["id"] == 13


def test_snapshot_keeps_nominal_corrected_and_measured_separate(body):
    body.stabilizer.offset_deg = 2
    values = [SimpleNamespace(Id=7, Sio=bus, Data=7500) for bus in (1, 2)]
    body._send_targets(values, 2, 1)
    body.body_servos.read(body)
    snapshot = body.body_servos.state(time.monotonic())
    assert snapshot["nominal"][14:16] == [0, 0]
    assert snapshot["sent"][14] == snapshot["sent"][15] > 0
    assert snapshot["measured"] == [0] * 30
    assert snapshot["sent"][0] is None
    assert snapshot["target_age_ms"][0] is None
    assert snapshot["simulated"] and snapshot["servo_freshness"] == "unknown"
    # Goals must be from the same host observation as the measured snapshot.
    body.sent_targets[7, 1] = 8000
    assert body.body_servos.state(time.monotonic())["sent"] == snapshot["sent"]


def test_direct_head_command_is_in_diagnostics(body):
    body.command("control.acquire", {})
    body.command("motion.head", {"pan": 10, "tilt": -10})
    data = body.command("body.telemetry.read", {"servos": True})["body.servos"]
    assert data["sent"][0] == data["sent"][25] == 15


def test_no_poll_without_servo_subscription_or_when_motion_due(body):
    body.hardware.body_positions = MagicMock(return_value=(0,) * 30)
    body.command("body.telemetry.watch", {"enabled": True})
    body.tick()
    body.hardware.body_positions.assert_not_called()
    body.command("body.telemetry.watch", {"enabled": True, "servos": True})
    body.tick()
    body.hardware.body_positions.assert_called_once()
    body.tick()
    body.hardware.body_positions.assert_called_once()
    body.body_servos.next_at = 0
    body.plan = iter([("sleep", 1)])
    body.next_at = 0
    body.tick()
    body.hardware.body_positions.assert_called_once()


def test_busy_retains_sample_until_stale_and_stop_invalidates(body):
    body.body_servos.read(body)

    def busy():
        raise Fault("body_busy", "busy")

    body.hardware.body_positions = busy
    data = body.command("body.telemetry.read", {"servos": True})["body.servos"]
    assert data["busy_reads"] == 1 and data["valid"]
    assert not body.body_servos.state(time.monotonic() + 1)["valid"]
    body.command("motion.stop_hard", {})
    data = body.body_servos.state(time.monotonic())
    assert not data["valid"] and "sent" not in data


def test_wire_budget_with_worst_case_values_and_timestamps(body):
    body.hardware.body_positions = lambda: (-32768,) * 30
    for unit in range(30):
        key = unit // 2, unit % 2 + 1
        body.servo_targets[key] = body.sent_targets[key] = 16383
        body.target_times[key] = 0
    body.body_servos.read(body)
    sample = {"topic": "body.servos", "subscription": "body.servos", "sequence": 2**63,
              "source_mono_ns": time.monotonic_ns(), "valid": True, "age_ms": 500,
              "data": body.body_servos.state(time.monotonic())}
    pack(envelope("sample", "data.sample", sample, session=2**63, token=2**63, sequence=2**63))


def test_new_topic_through_worker_and_udp(tmp_path):
    async def run():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await server.start()
        client = Client("127.0.0.1", server.sock.getsockname()[1])
        try:
            await client.connect()
            joints = await client.request("motion.joints", {"offset": 24})
            assert joints["items"][2]["name"] == "right_knee_bot"
            await client.request("data.subscribe", {"topic": "body.imu"})
            await client.request("data.subscribe", {"topic": "body.servos", "rate_hz": 5})
            await asyncio.sleep(0.3)
            data = (await client.request("data.snapshot", {"topic": "body.servos"}))["data"]
            assert data["valid"] and len(data["measured"]) == 30
            assert all(v is None for v in data["sent"])
            await client.request("data.unsubscribe", {"subscription_id": "body.servos"})
            assert not server.body_servo_watch and server.body_watch
            assert server.owner is None
        finally:
            await client.close()
            await server.close()
    asyncio.run(run())
