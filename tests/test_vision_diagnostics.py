import asyncio
from contextlib import contextmanager
from types import SimpleNamespace
from unittest.mock import AsyncMock

import cv2
import numpy as np
import pytest

from roki_ng.ball_observation import BallObservation, ball_candidates
from roki_ng.dataplane import FRAME_HEADER, FRAME_BYTES
from roki_ng.detection import Detection
from roki_ng.parameters import Parameters
from roki_ng.supervisor import Supervisor
from roki_ng.vision_diagnostics import DiagnosticVideo, render
from roki_ng.wire import Fault, envelope, pack


def detector(tmp_path):
    return Detection({'state_dir': str(tmp_path), 'parameters': Parameters(tmp_path).values},
                     lambda *a: None, lambda *a: None)


class Publisher:
    def __init__(self, *args, **kwargs):
        self.frames = []
        self.closed = False

    @contextmanager
    def loan(self, size):
        data = bytearray(size)
        yield memoryview(data)
        self.frames.append(bytes(data))

    def close(self):
        self.closed = True


def test_video_is_on_demand_bounded_and_discards_invalidated_frame(monkeypatch):
    monkeypatch.setattr('roki_ng.vision_diagnostics.Channel', Publisher)
    video = DiagnosticVideo()
    image = np.zeros((650, 800, 3), np.uint8)
    header = (42, 123456789, 800, 650, 2400)
    assert video.ticket() is None and video.channel is None
    video.start()
    ticket = video.ticket()
    video.invalidate()
    video.submit(ticket, header, image)
    video.tick()
    assert not video.channel.frames
    video.last_render = 0
    video.submit(video.ticket(), header, image)
    video.tick()
    video.tick()
    assert len(video.channel.frames) == 1
    assert len(video.channel.frames[0]) == FRAME_BYTES
    assert FRAME_HEADER.unpack_from(video.channel.frames[0]) == header
    channel = video.channel
    video.stop()
    assert channel.closed and video.latest is None and not video.state()['publishing']


def test_mask_and_metadata_use_same_frame_and_parameter_revision(tmp_path, monkeypatch):
    monkeypatch.setattr('roki_ng.vision_diagnostics.Channel', Publisher)
    d = detector(tmp_path)
    d.video.start()
    frame = FRAME_HEADER.pack(53, 123456789, 800, 650, 2400) + b'\x00\x8c\xff'*(800*650)
    d._consume(memoryview(frame))
    d.video.tick()
    encoded = d.video.channel.frames[-1]
    assert FRAME_HEADER.unpack_from(encoded)[0] == d.result['frame_sequence'] == 53
    image = np.frombuffer(encoded, np.uint8, offset=FRAME_HEADER.size).reshape(650, 800, 3)
    assert (image[100, 600] == 255).all()  # True robot LAB mask, not decoded preview.
    revision = d.result['parameter_revision']
    d.command('params.apply', {'vision.orange_ball.l_max': 20})
    assert d.result is None and d.video.latest is None
    frame = FRAME_HEADER.pack(54, 123456790, 800, 650, 2400) + frame[FRAME_HEADER.size:]
    d._consume(memoryview(frame))
    assert d.result['parameter_revision'] > revision and not d.result['blobs']
    d.close()


def test_inflight_old_threshold_result_is_discarded(tmp_path, monkeypatch):
    from roki_ng.detection import colour_blobs
    d = detector(tmp_path)
    def change_during_processing(*args, **kwargs):
        result = colour_blobs(*args, **kwargs)
        d.command('params.apply', {'vision.orange_ball.pixels_min': 100})
        return result
    monkeypatch.setattr('roki_ng.detection.colour_blobs', change_during_processing)
    frame = FRAME_HEADER.pack(1, 1, 800, 650, 2400) + bytes(800*650*3)
    d._consume(memoryview(frame))
    assert d.result is None and d.video.latest is None
    d.close()


@pytest.mark.parametrize('turf,point,reason', [
    (True, (1., .1), 'selected'), (False, (1., .1), 'no_field'),
    (True, (10., .1), 'range'), (True, None, 'projection'),
])
def test_diagnostic_reasons_are_actual_game_decisions(tmp_path, turf, point, reason):
    image = np.zeros((650, 800, 3), np.uint8)
    if turf:
        image[:] = (0, 130, 0)
    cv2.circle(image, (400, 400), 20, (0, 140, 255), -1)
    def project(*args):
        if point is None:
            raise ValueError('above horizon')
        return point
    args = image, Parameters(tmp_path).values, SimpleNamespace(ground_point=project), [0, 0, 0, 1]
    debug = {}
    result = ball_candidates(*args, debug=debug)
    assert result == ball_candidates(*args)
    assert debug['objects'][0]['reason'] == reason
    if reason == 'selected':
        measurement = debug['objects'][0]
        assert measurement['x_m'] == pytest.approx(result[0]['x_m'])
        assert measurement['y_m'] == pytest.approx(result[0]['y_m'])
        assert measurement['distance_m'] == pytest.approx(np.hypot(result[0]['x_m'], result[0]['y_m']))
    assert debug['counts'][reason] == 1
    assert debug['counts']['examined'] == 1
    output = render(image, debug['masks'], debug['objects'], 'frame=42')
    assert output.shape == image.shape and output.dtype == image.dtype


def test_diagnostic_lifecycle_does_not_stop_game_owned_observer(tmp_path, monkeypatch):
    monkeypatch.setattr(BallObservation, '_run', lambda self, offset: self.stop_event.wait(5))
    d = detector(tmp_path)
    d.command('detection.start', {'mode': 'ball', 'unicam_minus_stm': 7})
    assert d.state()['mode'] == 'ball' and d.state()['running']
    d.command('detection.stop', {})
    assert not d.ball.state()['running']
    d.command('ball.start', {'parameters': d.parameters, 'unicam_minus_stm': 7})
    with pytest.raises(Fault, match='Game owns'):
        d.command('detection.start', {'mode': 'ball', 'unicam_minus_stm': 7})
    d.command('detection.stop', {})
    assert d.ball.state()['running']
    d.close()


def test_ball_diagnostics_dispatch_requires_sync_and_idle_game_without_body_calls(tmp_path):
    async def run():
        s = Supervisor({'state_dir': str(tmp_path)})
        s.mode = 'MANUAL'
        s.require_control = lambda *args: None
        s.workers = {name: SimpleNamespace(call=AsyncMock(return_value={}))
                     for name in ('camera', 'detection', 'motherboard', 'stream')}
        camera = {'running': True, 'imu_sync': {'state': 'aligning'}}
        s.workers['camera'].call.return_value = camera
        with pytest.raises(Fault, match='synchronized'):
            await s.dispatch(None, 'detection.start', {'mode': 'ball'})
        s.workers['detection'].call.assert_not_called()
        camera['imu_sync'] = {'state': 'synced', 'unicam_minus_stm': 7}
        await s.dispatch(None, 'detection.start', {'mode': 'ball'})
        s.workers['detection'].call.assert_awaited_once_with('detection.start', {'mode': 'ball', 'unicam_minus_stm': 7})
        s.workers['motherboard'].call.assert_not_called()
        s.game.info['running'] = True
        with pytest.raises(Fault, match='Stop'):
            await s.dispatch(None, 'detection.start', {'mode': 'ball'})
        await s.dispatch(None, 'detection.stop', {})
        s.workers['stream'].call.assert_awaited_once_with('videostream.stop_source', {'name': 'detection'})
        s.workers['motherboard'].call.assert_not_called()
    asyncio.run(run())


def test_ball_diagnostic_status_fits_udp(tmp_path):
    d = detector(tmp_path)
    d.mode = 'ball'
    d.ball.diagnostic = True
    d.ball.diagnostics = {'parameter_revision': 12345, 'processing_us': 123456,
                          'counts': dict(components=123456, area_rejected=123450,
                                         eligible=10, examined=10, no_field=4, projection=4, range=1, selected=1)}
    state = d.state()
    assert len(pack(envelope('response', 'detection.status', {'result': state}))) < 1400
    d.close()


def test_football_diagnostic_start_requires_stand_and_never_moves_body(tmp_path):
    async def run():
        s = Supervisor({'state_dir': str(tmp_path)})
        s.mode = 'MANUAL'
        s.require_control = lambda *args: None
        s.workers = {name: SimpleNamespace(call=AsyncMock(return_value={}))
                     for name in ('camera', 'detection', 'motherboard', 'stream')}
        s.workers['camera'].call.return_value = {'running': True, 'imu_sync': {'state': 'synced', 'unicam_minus_stm': 7}}
        s.workers['motherboard'].call.return_value = {'pose': 'unknown', 'body_connected': True}
        with pytest.raises(Fault, match='base_stand'):
            await s.dispatch(None, 'detection.start', {'mode': 'football'})
        s.workers['detection'].call.assert_not_called()
        s.workers['motherboard'].call.return_value = {'pose': 'base_stand', 'body_connected': True,
                                                     'active_job': None, 'head': {'pan': 0, 'tilt': -1000}}
        await s.dispatch(None, 'detection.start', {'mode': 'football'})
        assert s.football_diagnostic
        call = s.workers['detection'].call.await_args
        assert call.args[1]['pose']['tilt'] == -1000
        assert all(c.args == ('state',) for c in s.workers['motherboard'].call.await_args_list)
        s.worker_event('motherboard', 'job.progress', {'operation': 'motion.head'})
        await asyncio.sleep(0)
        assert not s.football_diagnostic
        assert s.workers['detection'].call.await_args.args[0] == 'football.pose'
        assert not s.workers['detection'].call.await_args.args[1]['active']
    asyncio.run(run())


def test_football_status_fits_udp_and_invalidates_pose(tmp_path):
    import time
    d = detector(tmp_path)
    d.ball.football = True
    d.mode = 'football'
    d.ball.scene_tracker.current = dict(pose_id=1, sequence=2, ball=[.123, -.045],
                                       parts=[], received=time.monotonic())
    state = d.state()
    assert state['result']['valid']
    assert len(pack(envelope('response', 'detection.status', {'result': state}))) < 1400
    d.command('football.pose', {'active': False})
    assert not d.state()['result']['valid']
    d.close()


def test_video_failure_does_not_stop_observer_and_parameter_edit_clears_debug(tmp_path, monkeypatch):
    monkeypatch.setattr(BallObservation, '_run', lambda self, offset: self.stop_event.wait(5))
    class BrokenPublisher(Publisher):
        def loan(self, size):
            raise RuntimeError('video unavailable')
    monkeypatch.setattr('roki_ng.vision_diagnostics.Channel', BrokenPublisher)
    d = detector(tmp_path)
    logs = []
    d.log = lambda *args: logs.append(args)
    d.command('detection.start', {'mode': 'ball', 'unicam_minus_stm': 7})
    d.command('source.start', {})
    d.video.submit(d.video.ticket(), (1, 1, 800, 650, 2400), np.zeros((650, 800, 3), np.uint8))
    d.tick()
    assert d.ball.state()['running'] and d.video.error == 'video unavailable'
    assert logs[-1][0] == 'ERROR' and not d.video.requested
    d.ball.diagnostics = {'counts': {'selected': 1}}
    d.command('params.apply', {'vision.orange_ball.pixels_min': 100})
    assert d.ball.state()['diagnostics'] is None
    assert d.ball.state()['result']['reason'] == 'parameters_changed'
    d.close()
