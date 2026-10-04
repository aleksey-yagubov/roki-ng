import asyncio
from unittest.mock import MagicMock, AsyncMock

import pytest

from roki_ng.body import Body, RokiHardware
from roki_ng.parameters import Parameters
from roki_ng.supervisor import Supervisor
from roki_ng.wire import Fault


def test_acm_loss_requests_one_hardware_reset_without_replaying_motion(tmp_path):
    events = []
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values},
                lambda op, value: events.append((op, value)), lambda *a: None)
    body.command('control.acquire', {})
    ident = body.command('motion.jump', {})['job_id']
    error = Fault('hardware_error', 'ACM write timeout; reset session before reuse')
    for i in range(3):
        body._important_failure(error, reset=False)
        assert body.motherboard_reset_pending == (i == 2)
    assert body.jobs[ident]['status'] == 'failed' and body.plan is None
    sent = body.hardware.sent
    for _ in range(10):
        body.tick()
    assert body.hardware.sent == sent
    assert len([op for op, _ in events if op == 'motherboard.reset_required']) == 1
    body.hardware.reconnect_motherboard = MagicMock()
    body.command('motion.stop_hard', {})
    body.command('motherboard.reconnect', {})
    assert not body.body_connected and not body.motherboard_reset_pending
    for _ in range(3):
        body.reconnect_at = 0
        body.tick()
    assert body.body_connected and body.active is None and body.hardware.sent == sent


def test_body_timeout_does_not_reset_stm(tmp_path):
    events = []
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values},
                lambda op, value: events.append(op), lambda *a: None)
    for _ in range(3):
        body._important_failure(Fault('hardware_error', 'Rcb4: BodyTimeout'), reset=False)
    assert not body.body_connected
    assert not body.motherboard_reset_pending and 'motherboard.reset_required' not in events


def test_reopen_keeps_same_client_and_stops_unknown_controller_kick():
    hw = RokiHardware.__new__(RokiHardware)
    hw.mb = MagicMock()
    mb = hw.mb
    hw.mb.GetStatus.return_value = {'body_failures': 0}
    hw.frame_s = .02
    hw.forward_pending = hw.controller_pending = hw.mixing_started = True
    hw.reconnect_motherboard()
    assert hw.mb is mb
    mb.ConfigureACM.assert_called_once_with(timeout_ms=1000)
    mb.SetBodyQueuePeriod.assert_called_once_with(20)
    mb.ResetBodyQueue.assert_called_once()
    assert hw.controller_pending  # connect_body must still stop the Zubr slot.
    assert not hw.forward_pending and not hw.mixing_started


def test_reset_coordinator_stops_capture_then_boots_then_reopens(tmp_path):
    async def run():
        server = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        steps = []
        async def stop(): steps.append('stop_capture')
        async def reconnect(*a, **kw): steps.append('reconnect')
        server._stop_capture = stop
        server.bootstrap = MagicMock()
        server.bootstrap.start.side_effect = lambda: steps.append('gpio_boot')
        server.workers['motherboard'] = MagicMock(call=AsyncMock(side_effect=reconnect))
        server.game.paused = server.game.pickup_requested = True
        await server._reset_motherboard()
        assert steps == ['stop_capture', 'gpio_boot', 'reconnect']
        assert server.game.paused and server.game.pickup_requested
    asyncio.run(run())


def test_reset_retries_failed_boot_and_coalesces_requests(tmp_path, monkeypatch):
    async def run():
        server = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        server._stop_capture = AsyncMock(side_effect=Fault('hardware_error', 'ACM disconnected'))
        server.bootstrap = MagicMock()
        server.bootstrap.start.side_effect = [RuntimeError('boot not ready'), None]
        server.workers['motherboard'] = MagicMock(call=AsyncMock())
        delays = []
        async def delay(seconds): delays.append(seconds)
        monkeypatch.setattr('roki_ng.supervisor.asyncio.sleep', delay)
        server.worker_event('motherboard', 'motherboard.reset_required', {})
        task = server.motherboard_reset_task
        server.worker_event('motherboard', 'motherboard.reset_required', {})
        assert server.motherboard_reset_task is task
        await task
        assert server.bootstrap.start.call_count == 2
        assert delays == [2]
        server.workers['motherboard'].call.assert_awaited_once_with('motherboard.reconnect', timeout=10)
    asyncio.run(run())
