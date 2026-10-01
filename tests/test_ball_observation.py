import threading
import time

import pytest
import numpy as np
import cv2

from roki_ng.ball_observation import BallObservation, BallTracker, ball_candidates, capture_time
from roki_ng.parameters import SCHEMA
from roki_ng.detection import Detection
from roki_ng.wire import Fault


def parameters():
    return {k: v[1] for k, v in SCHEMA.items()}


class Projector:
    def __init__(self, point=(1., .1)):
        self.point = point
        self.pixels = []

    def ground_point(self, pixel, quaternion):
        self.pixels.append(pixel)
        return self.point


def scene(centres=((400, 400),), turf=True):
    image = np.zeros((650, 800, 3), np.uint8)
    if turf:
        image[:] = (0, 130, 0)
    for centre in centres:
        cv2.circle(image, centre, 20, (0, 140, 255), -1)
    return image


def test_shape_turf_and_foot_projection():
    projector = Projector()
    candidates = ball_candidates(scene(), parameters(), projector, [0, 0, 0, 1])
    assert candidates == [dict(x_m=1., y_m=.1, rect=[380, 380, 421, 421])]
    assert projector.pixels == [(400., 420.)]
    assert not ball_candidates(scene(turf=False), parameters(), projector, [0, 0, 0, 1])
    image = scene(())
    cv2.rectangle(image, (300, 380), (450, 400), (0, 140, 255), -1)
    assert not ball_candidates(image, parameters(), projector, [0, 0, 0, 1])


def test_ambiguity_distance_and_size():
    assert len(ball_candidates(scene(((300, 400), (500, 400))), parameters(), Projector(), [0, 0, 0, 1])) == 2
    for point in ((4., 0.), (-1., 0.), (float('nan'), 0.)):
        assert not ball_candidates(scene(), parameters(), Projector(point), [0, 0, 0, 1])
    assert not ball_candidates(scene(((0, 400),)), parameters(), Projector(), [0, 0, 0, 1])
    with pytest.raises(ValueError, match='800x650'):
        ball_candidates(np.zeros((50, 50, 3), np.uint8), parameters(), Projector(), [0, 0, 0, 1])


def candidate(x=1.):
    return dict(x_m=x, y_m=.1, rect=[380, 380, 421, 421])


def test_temporal_freshness_and_rejection():
    tracker = BallTracker()
    assert tracker.state(0)[1]['reason'] == 'no_sync'
    for sequence in range(1, 4):
        result = tracker.update([candidate()], sequence, sequence*100, sequence*.1, sequence*.1)
        assert result['valid'] == (sequence == 3)
    assert tracker.state(.81)[1]['reason'] == 'stale'
    assert not tracker.update([candidate()], 4, 400, .9, .9)['valid']
    assert tracker.update([candidate()], 4, 400, 1., 1.)['reason'] == 'out_of_order'
    assert tracker.update([candidate()], 5, 399, 1., 1.)['reason'] == 'out_of_order'
    assert tracker.update([], 6, 600, 1.1, 1.1)['reason'] == 'no_ball'
    assert tracker.update([candidate(), candidate()], 7, 700, 1.2, 1.2)['reason'] == 'ambiguous'
    assert tracker.update([candidate()], 8, 800, 1.3, 1.9)['reason'] == 'stale'


def test_jump_resets_temporal_gate():
    tracker = BallTracker()
    for sequence in range(1, 4):
        tracker.update([candidate()], sequence, sequence, sequence*.1, sequence*.1)
    assert not tracker.update([candidate(2.)], 4, 4, .4, .4)['valid']


def test_start_returns_before_calibration_and_close_invalidates(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    def delayed(self, offset):
        entered.set()
        release.wait(2)
    monkeypatch.setattr(BallObservation, '_run', delayed)
    detector = Detection(dict(parameters=parameters(), state_dir=str(tmp_path)), lambda *a: None, lambda *a: None)
    began = time.monotonic()
    state = detector.command('ball.start', dict(parameters=parameters(), unicam_minus_stm=4))
    assert time.monotonic()-began < .5
    assert entered.wait(.5)
    assert state['running'] and not state['result']['valid']
    detector.command('detection.stop', {})
    assert detector.command('ball.status', {})['running']
    with pytest.raises(Fault, match='already running'):
        detector.command('ball.start', dict(parameters=parameters(), unicam_minus_stm=4))
    release.set()
    assert detector.command('ball.stop', {})['result']['reason'] == 'stopped'
    detector.close()


def test_calibration_failure_is_worker_error(tmp_path):
    observer = BallObservation(dict(parameters=parameters(), state_dir=str(tmp_path)))
    observer.start(dict(parameters=parameters(), unicam_minus_stm=0))
    observer.thread.join(2)
    state = observer.state()
    assert not state['running'] and state['error'] and not state['result']['valid']
    observer.close()


def test_exposure_age_includes_camera_queue_delay():
    tracker = BallTracker()
    exposed = capture_time(1_000_000_000, 2_000_000_000, 10.)
    assert exposed == 9.
    assert tracker.update([candidate()], 1, 1_000_000_000, exposed, 10.)['reason'] == 'stale'
    future = capture_time(3_000_000_000, 2_000_000_000, 10.)
    assert tracker.update([candidate()], 2, 3_000_000_000, future, 10.)['reason'] == 'stale'


def test_threshold_edits_reach_ball_and_invalidate_old_result(tmp_path):
    detector = Detection(dict(parameters=parameters(),state_dir=str(tmp_path)),lambda *a:None,lambda *a:None)
    old = detector.ball.parameters
    for seq in range(1,4):
        detector.ball.tracker.update([candidate()],seq,seq,seq*.1,seq*.1)
    assert detector.ball.tracker.result['valid']
    detector.command('params.apply',{'vision.orange_ball.pixels_min':80})
    assert detector.ball.parameters['vision.orange_ball.pixels_min']==80
    assert old['vision.orange_ball.pixels_min']==50
    assert detector.ball.parameter_revision==1
    assert detector.ball.tracker.result['reason']=='parameters_changed'
    assert not detector.ball.tracker.result['valid']
    detector.close()
