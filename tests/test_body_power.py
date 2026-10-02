import asyncio
import time
from unittest.mock import MagicMock

import pytest

from roki_ng.body import Body, RokiHardware
from roki_ng.client import Client
from roki_ng.parameters import Parameters
from roki_ng.supervisor import Supervisor
from roki_ng.wire import Fault, envelope, pack


@pytest.fixture
def body(tmp_path):
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *args: None, lambda *args: None)
    body.reconnect_at = float("inf")
    return body


def test_native_unsigned_adc_and_errors():
    hardware = object.__new__(RokiHardware)
    hardware.rcb = MagicMock()
    hardware.mb = MagicMock()
    hardware.rcb.moveRamToComCmdSynchronize.return_value = (True, b"\xfe\xff")
    assert hardware.body_power_adc() == 65534
    hardware.rcb.moveRamToComCmdSynchronize.assert_called_once_with(0xCC, 2)
    hardware.rcb.moveRamToComCmdSynchronize.return_value = (True, b"\0")
    with pytest.raises(Fault, match="2 bytes"):
        hardware.body_power_adc()
    hardware.rcb.moveRamToComCmdSynchronize.return_value = (False, b"\0\0")
    hardware.mb.GetLastStatus.return_value = 5
    hardware.rcb.GetError.return_value = "Busy"
    with pytest.raises(Fault) as error:
        hardware.body_power_adc()
    assert error.value.code == "body_busy"
    hardware.mb.GetLastStatus.return_value = 6
    with pytest.raises(Fault) as error:
        hardware.body_power_adc()
    assert error.value.code == "hardware_error"


def test_snapshot_conversion_cache_and_freshness(body):
    empty = body._body_telemetry()["body.power"]
    assert not empty["valid"] and empty["voltage_v"] is None
    body.hardware.body_power_adc = MagicMock(return_value=2702)
    for _ in range(3):
        sample = body.command("body.telemetry.read", {"power": True})["body.power"]
        assert sample["valid"] and sample["voltage_v"] == 10.0
        assert sample["adc_raw"] == 2702 and sample["sequence"] == 1
        assert sample["timestamp_kind"] == "host_receive" and sample["simulated"]
    body.hardware.body_power_adc.assert_called_once()
    assert body.body_imu.sequence == 0
    assert not body.body_power.state(time.monotonic() + 4)["valid"]
    pack(envelope("sample", "data.sample", {"topic": "body.power", "data": sample}))


def test_per_robot_calibration_changes_only_volts_and_persists(body, tmp_path):
    body.hardware.body_power_adc = MagicMock(return_value=3660)
    original = body.command("body.telemetry.read", {"power": True})["body.power"]
    scale = 0.9072613518386974
    params = Parameters(tmp_path)
    params.set("power.voltage_scale", scale)
    assert Parameters(tmp_path).values["power.voltage_scale"] == scale
    body.command("params.apply", {"power.voltage_scale": scale})
    calibrated = body.command("body.telemetry.read", {"power": True})["body.power"]
    assert calibrated["voltage_v"] == pytest.approx(12.29, abs=0.001)
    assert calibrated["voltage_scale"] == scale
    for key in ("adc_raw", "source_mono_ns", "sequence", "valid"):
        assert calibrated[key] == original[key]
    body.hardware.body_power_adc.assert_called_once()
    for invalid in (0, -1, 2, float("nan"), float("inf")):
        with pytest.raises(Fault):
            params.set("power.voltage_scale", invalid)


def test_poll_is_opt_in_rate_limited_and_yields_to_motion(body):
    body.hardware.body_power_adc = MagicMock(return_value=3242)
    body.tick()
    body.hardware.body_power_adc.assert_not_called()
    body.command("body.telemetry.watch", {"enabled": True, "power": True, "imu": False})
    body.tick()
    body.tick()
    body.hardware.body_power_adc.assert_called_once()
    assert body.body_imu.sequence == 0 and body.body_servos.received_at is None
    body.body_power.next_at = 0
    body.body_power.received_at -= 2
    body.plan = iter([("sleep", 1)])
    body.next_at = 0
    body.tick()
    body.hardware.body_power_adc.assert_called_once()
    body.plan = None
    body.tick()
    assert body.hardware.body_power_adc.call_count == 2
    body.command("body.telemetry.watch", {"enabled": False})
    body.body_power.next_at = 0
    body.body_power.received_at -= 2
    body.tick()
    assert body.hardware.body_power_adc.call_count == 2


def test_busy_expires_and_link_loss_clears_values(body):
    body.body_power.read(body.hardware)
    body.body_power.received_at -= 2
    body.hardware.body_power_adc = MagicMock(side_effect=Fault("body_busy", "busy"))
    data = body.command("body.telemetry.read", {"power": True})["body.power"]
    assert data["valid"] and data["busy_reads"] == 1 and body.body_connected
    assert not body.body_power.state(time.monotonic() + 2)["valid"]
    body.hardware.body_power_adc.side_effect = Fault("hardware_error", "Timeout")
    with pytest.raises(Fault):
        body.command("body.telemetry.read", {"power": True})
    data = body._body_telemetry()["body.power"]
    assert not body.body_connected and not data["valid"]
    assert data["adc_raw"] is data["voltage_v"] is data["source_mono_ns"] is None
    assert "Timeout" in data["error"]


def test_malformed_reply_does_not_disconnect_body(body):
    body.hardware.body_power_adc = MagicMock(side_effect=Fault("power_data_invalid", "short reply"))
    data = body.command("body.telemetry.read", {"power": True})["body.power"]
    assert body.body_connected and not data["valid"] and data["invalid_reads"] == 1


def test_power_subscription_through_worker_and_udp(tmp_path):
    async def run():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await server.start()
        client = Client("127.0.0.1", server.sock.getsockname()[1])
        second = Client("127.0.0.1", server.sock.getsockname()[1])
        second_closed = False
        try:
            await client.connect()
            await second.connect()
            catalog = (await client.request("data.list"))["items"]
            assert next(t for t in catalog if t["name"] == "body.power")["max_rate_hz"] == 1
            result = await client.request("data.subscribe", {"topic": "body.power"})
            assert result == {"subscription_id": "body.power", "rate_hz": 1}
            await second.request("data.subscribe", {"topic": "body.power", "rate_hz": 0.5})
            with pytest.raises(Fault):
                await client.request("data.update", {"topic": "body.power", "rate_hz": 2})
            assert server.body_power_watch and not server.body_imu_watch
            assert not server.body_servo_watch and server.owner is None

            async def valid_sample():
                while True:
                    message = await client.events.get()
                    if (message["op"] == "data.sample" and message["body"]["topic"] == "body.power"
                            and message["body"]["valid"]):
                        return message["body"]

            sample = await asyncio.wait_for(valid_sample(), 3)
            assert sample["data"]["voltage_v"] == pytest.approx(3242 / 270.2)
            assert sample["age_ms"] < 3000
            snapshot = await client.request("data.snapshot", {"topic": "body.power"})
            assert snapshot["valid"] and snapshot["data"]["adc_raw"] == 3242
            assert server.body_telemetry["body.imu"]["sequence"] == 0
            server.body_telemetry["body.power"]["source_mono_ns"] -= 4_000_000_000
            assert not server._sample("body.power")["valid"]
            assert not server._sample("body.power")["data"]["valid"]
            await client.request("data.subscribe", {"topic": "body.imu"})
            await client.request("data.unsubscribe", {"subscription_id": "body.power"})
            assert server.body_power_watch and server.body_imu_watch
            await second.close()
            second_closed = True
            # session.close is acknowledged before asynchronous subscription cleanup.
            async def session_removed():
                while second.session in server.sessions:
                    await asyncio.sleep(0.01)

            await asyncio.wait_for(session_removed(), 3)
            assert not server.body_power_watch and server.body_imu_watch
            await client.request("data.unsubscribe", {"subscription_id": "body.imu"})
            assert not server.body_watch
        finally:
            if not second_closed:
                await second.close()
            await client.close()
            await server.close()
    asyncio.run(run())
