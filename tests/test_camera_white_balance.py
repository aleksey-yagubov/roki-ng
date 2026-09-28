from types import SimpleNamespace as NS
from unittest.mock import Mock
import asyncio
import pytest
from roki_ng.camera import Camera
from roki_ng.parameters import Parameters
from roki_ng.supervisor import Supervisor
from roki_ng.wire import Fault

RED = 'camera.white_balance.red_gain'
BLUE = 'camera.white_balance.blue_gain'


def camera(values):
    c = Camera({'parameters': values}, Mock(), Mock())
    c.lc = NS(controls=NS(AeEnable='ae', AwbEnable='awb', ExposureTime='exposure',
                          AnalogueGain='gain', FrameDurationLimits='duration', ColourGains='wb'),
              Request=NS(Status=NS(Complete='complete')),
              FrameMetadata=NS(Status=NS(Success='success')))
    c.camera = Mock()
    c.duration, c.exposure, c.gain = 16667, 8000, 1.0
    c.requests = []
    return c


def test_manual_wb_loaded_from_robot_profile_at_start():
    c = camera({RED: 1.19, BLUE: 2.61})
    c.command('camera.start', {})
    controls = c.camera.start.call_args.args[0]
    assert controls['awb'] is False
    assert controls['ae'] is False
    assert controls['wb'] == (1.19, 2.61)


def test_live_wb_uses_next_recycled_request_without_restart():
    c = camera({RED: 1.0, BLUE: 1.0})
    c.command('camera.start', {})
    c.camera.controls = {'wb': NS(min=0.0, max=32.0)}
    stream = object()
    c.stream = stream
    req = Mock(status='complete', buffers={stream: NS(metadata=NS(status='success'))})
    c.manager = NS(get_ready_requests=lambda: [req])
    c._publish = Mock()
    alignment = c.alignment = object()
    c.command('params.apply', {RED: 1.19, BLUE: 2.61})
    c.ready(0)
    req.set_control.assert_called_once_with('wb', (1.19, 2.61))
    c.camera.stop.assert_not_called()
    assert c.camera.start.call_count == 1
    assert c.alignment is alignment
    c.ready(0)
    assert req.set_control.call_count == 1  # no stale controls on next reuse


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), 0, 33, True, '1.2'])
def test_bad_wb_does_not_replace_valid_pair(bad):
    c = camera({RED: 1.19, BLUE: 2.61})
    with pytest.raises(Fault):
        c.command('params.apply', {RED: bad})
    c.command('camera.start', {})
    assert c.camera.start.call_args.args[0]['wb'] == (1.19, 2.61)


def test_wb_parameters_are_persistent_and_routed(tmp_path):
    async def run():
        s = Supervisor({'state_dir': str(tmp_path)})
        from unittest.mock import AsyncMock
        worker = NS(call=AsyncMock(return_value={}))
        s.workers = {'camera': worker}
        await s._set_parameters({RED: 1.19, BLUE: 2.61})
        worker.call.assert_called_once_with('params.apply', {RED: 1.19, BLUE: 2.61})
        loaded = Parameters(tmp_path)
        assert loaded.values[BLUE] == 2.61
    asyncio.run(run())


def test_sensor_range_rejection_keeps_previous_pair():
    c = camera({RED: 1.19, BLUE: 2.61})
    c.camera.controls = {'wb': NS(min=0.1, max=4.0)}
    with pytest.raises(Fault, match='range'):
        c.command('params.apply', {RED: 5.0})
    assert c.white_balance == (1.19, 2.61)
    assert not c.wb_pending


def test_persistence_failure_rolls_worker_back(tmp_path):
    async def run():
        from unittest.mock import AsyncMock
        s = Supervisor({'state_dir': str(tmp_path)})
        worker = NS(call=AsyncMock(return_value={}))
        s.workers = {'camera': worker}
        s.params.set_many = Mock(side_effect=OSError('disk full'))
        with pytest.raises(OSError, match='disk full'):
            await s._set_parameters({RED: 1.19, BLUE: 2.61})
        assert worker.call.call_args_list[-1].args == ('params.apply', {RED: 1.0, BLUE: 1.0})
        assert Parameters(tmp_path).values[RED] == 1.0
    asyncio.run(run())


def test_exposure_and_auto_controls_use_recycled_request():
    c=camera({})
    c.camera.controls={name:NS(min=low,max=high) for name,low,high in
                       [('wb',.01,32),('exposure',1,20000),('gain',1,16),('ae',False,True),('awb',False,True)]}
    c.command('camera.start',{})
    stream=object();c.stream=stream
    req=Mock(status='complete',buffers={stream:NS(metadata=NS(status='success'))})
    c.manager=NS(get_ready_requests=lambda:[req]);c._publish=Mock()
    c.command('params.apply',{'camera.exposure_us':9000,'camera.analogue_gain':2.,'camera.awb_enabled':True})
    c.ready(0)
    assert dict(call.args for call in req.set_control.call_args_list)=={'exposure':9000,'gain':2.,'awb':True}
    assert c.camera.start.call_count==1
    c.camera.stop.assert_not_called()
    before=req.set_control.call_count;c.ready(0)
    assert req.set_control.call_count==before


def test_exposure_longer_than_frame_is_rejected_atomically():
    c=camera({});c.camera.controls={'wb':NS(min=.01,max=32)}
    with pytest.raises(Fault):c.command('params.apply',{'camera.exposure_us':20000,RED:2.})
    assert c.exposure==8000 and c.white_balance==(1.,1.) and not c.pending_controls
