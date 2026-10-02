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


def test_field_support_and_unchanged_foot_projection():
    projector = Projector()
    candidates = ball_candidates(scene(), parameters(), projector, [0, 0, 0, 1])
    assert candidates == [dict(x_m=1., y_m=.1, rect=[380, 380, 421, 421])]
    assert projector.pixels == [(400., 420.)]
    assert not ball_candidates(scene(turf=False), parameters(), projector, [0, 0, 0, 1])


def test_projection_limits_and_clipped_ball():
    for point in ((4., 0.), (-1., 0.), (float('nan'), 0.)):
        assert not ball_candidates(scene(), parameters(), Projector(point), [0, 0, 0, 1])
    for centre in ((0, 400), (799, 400), (400, 0), (400, 649)):
        assert ball_candidates(scene((centre,)), parameters(), Projector(), [0, 0, 0, 1])
    with pytest.raises(ValueError, match='800x650'):
        ball_candidates(np.zeros((50, 50, 3), np.uint8), parameters(), Projector(), [0, 0, 0, 1])


def test_ball_on_white_line_with_highlight_and_incomplete_colour_mask():
    image = scene(())
    image[350:470, :] = 255  # No green in any of the ball's support regions.
    cv2.circle(image, (400, 400), 20, (0, 140, 255), -1)
    image[380:395, 380:421] = 255  # The bright cap is outside orange thresholds.
    assert ball_candidates(image, parameters(), Projector(), [0, 0, 0, 1])
    # Only a thin crescent remains: neither roundness nor bbox fill is meaningful.
    cv2.circle(image, (400, 392), 23, (255, 255, 255), -1)
    assert ball_candidates(image, parameters(), Projector(), [0, 0, 0, 1])


@pytest.mark.parametrize('colour,origin', [
    ((0, 130, 0), (370, 400)),  # Left only.
    ((255, 255, 255), (425, 400)),  # Right only.
    ((255, 255, 255), (395, 430)),  # Below only.
])
def test_field_support_requires_a_connected_patch_not_scattered_pixels(colour, origin):
    image = scene(turf=False)
    x, y = origin
    image[y, x:x+6] = colour
    assert not ball_candidates(image, parameters(), Projector(), [0, 0, 0, 1])
    image[y+3, x] = colour
    assert not ball_candidates(image, parameters(), Projector(), [0, 0, 0, 1])
    image[y, x+6] = colour
    assert ball_candidates(image, parameters(), Projector(), [0, 0, 0, 1])


def test_thin_mask_uses_pixel_count_and_configurable_size_thresholds():
    image = scene(())
    cv2.line(image, (300, 400), (550, 400), (0, 140, 255), 1)
    p = parameters()
    assert ball_candidates(image, p, Projector(), [0, 0, 0, 1])
    p['vision.orange_ball.pixels_min'] = 252
    assert not ball_candidates(image, p, Projector(), [0, 0, 0, 1])
    p['vision.orange_ball.pixels_min'] = 50
    p['vision.orange_ball.box_area_min'] = 252
    assert not ball_candidates(image, p, Projector(), [0, 0, 0, 1])


def candidate(x=1.):
    return dict(x_m=x, y_m=.1, rect=[380, 380, 421, 421])


def test_temporal_freshness_and_rejection():
    tracker = BallTracker()
    assert tracker.state(0)[1]['reason'] == 'no_sync'
    for sequence in range(1, 4):
        result = tracker.update([candidate()], sequence, sequence*100, sequence*.1, sequence*.1)
        assert result['valid']
    assert tracker.state(.81)[1]['reason'] == 'stale'
    assert tracker.update([candidate()], 4, 400, .9, .9)['valid']
    assert tracker.update([candidate()], 4, 400, 1., 1.)['reason'] == 'out_of_order'
    assert tracker.update([candidate()], 5, 399, 1., 1.)['reason'] == 'out_of_order'
    assert tracker.update([], 6, 600, 1.1, 1.1)['reason'] == 'no_ball'
    assert tracker.update([candidate(), candidate()], 7, 700, 1.2, 1.2)['valid']
    assert tracker.update([candidate()], 8, 800, 1.3, 1.9)['reason'] == 'stale'


def test_nearest_ball_is_available_immediately_and_can_move_quickly():
    class DistanceProjector:
        def ground_point(self, pixel, quaternion):
            return (800-pixel[0])/200, .1

    image = scene(((300, 400), (500, 400), (700, 400)))
    # A nearer orange distractor without field support must not win selection.
    image[350:480, 630:780] = 0
    cv2.circle(image, (700, 400), 20, (0, 140, 255), -1)
    candidates = ball_candidates(image, parameters(), DistanceProjector(), [0, 0, 0, 1])
    tracker = BallTracker()
    first = tracker.update(candidates, 1, 1, .1, .1)
    assert first['valid'] and first['rect'] == [480, 380, 521, 421]
    for order in (candidates, list(reversed(candidates))):
        assert BallTracker().update(order, 1, 1, .1, .1)['rect'] == first['rect']
    moved = tracker.update([candidate(2.5)], 2, 2, .2, .2)
    assert moved['valid'] and moved['x_m'] == 2.5


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
    detector.ball.tracker.update([candidate()],1,1,.1,.1)
    assert detector.ball.tracker.result['valid']
    detector.command('params.apply',{'vision.orange_ball.pixels_min':80})
    assert detector.ball.parameters['vision.orange_ball.pixels_min']==80
    assert old['vision.orange_ball.pixels_min']==50
    assert detector.ball.parameter_revision==1
    assert detector.ball.tracker.result['reason']=='parameters_changed'
    assert not detector.ball.tracker.result['valid']
    detector.close()
