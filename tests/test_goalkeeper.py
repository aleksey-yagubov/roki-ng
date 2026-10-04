import math
import asyncio
import time

import pytest

from roki_ng.goalkeeper import HEAD_TICK_RAD, ball_in_body
from roki_ng.goalkeeper import keeper_target, KeeperMotionGuard
from roki_ng.odometry import GameOdometry
from roki_ng.parameters import Parameters
from test_game import setup, observation


@pytest.mark.parametrize('pan,sign', [(-1333, 1), (1333, -1)])
def test_scanned_ball_uses_legacy_servo_sign(pan, sign):
    raw = dict(valid=True, x_m=1., y_m=0., frame_sequence=21)
    body = ball_in_body(raw, pan)
    assert body['x_m'] == pytest.approx(math.cos(pan * HEAD_TICK_RAD))
    assert body['y_m'] * sign > .7
    assert body['frame_sequence'] == 21 and raw['y_m'] == 0  # No mutation of worker data.


@pytest.mark.parametrize('own', [0, 1])
@pytest.mark.parametrize('side', [-1, 1])
def test_keeper_target_stays_inside_posts_and_never_follows_ball_depth(tmp_path, own, side):
    p = Parameters(tmp_path).values | {'match.own_goal': own}
    pose = GameOdometry(p, 'goalkeeper', 1.3)
    goal = p[f'field.goal.{own}']
    pose.y += side*.1
    target = keeper_target(p, pose, {'x_m': 3., 'y_m': side*3.})
    assert target[0] == pose.home_x
    assert abs(target[1]-goal['y']) <= goal['width']/2-.1+1e-9
    pose.x += .3
    assert keeper_target(p, pose, None) == (pose.home_x, pose.home_y)


def test_keeper_live_guard_compensates_own_motion_and_needs_distinct_frames(tmp_path):
    p = Parameters(tmp_path).values
    pose = GameOdometry(p, 'goalkeeper', 0.)
    initial = observation(.3)['result']
    guard = KeeperMotionGuard(pose, initial, (pose.home_x, .3))
    pose.y = .08
    assert guard.reason(observation(.22, seq=2), pose) is None  # Stationary field ball.
    moved = observation(-.2, seq=3)
    assert guard.reason(moved, pose) is None
    assert guard.reason(moved, pose) is None  # Polling the same frame is not confirmation.
    assert guard.reason(observation(-.2, seq=4), pose) == 'ball_shifted'


def test_keeper_live_guard_expires_repeated_or_missing_frames(tmp_path):
    p = Parameters(tmp_path).values
    pose = GameOdometry(p, 'goalkeeper', 0.)
    first = observation(.3)
    guard = KeeperMotionGuard(pose, first['result'], (pose.home_x, .3))
    guard.last_seen = time.monotonic()-1.
    assert guard.reason(first, pose) == 'ball_not_fresh'


def test_keeper_at_post_does_not_keep_chasing_ball_outside(tmp_path):
    p = Parameters(tmp_path).values
    pose = GameOdometry(p, 'goalkeeper', 0.)
    goal = p[f"field.goal.{p['match.own_goal']}"]
    edge = goal['y']+goal['width']/2-.1
    pose.y = edge
    assert keeper_target(p, pose, {'x_m': 2., 'y_m': 1.}) == (pose.home_x, edge)
    pose.y += .06
    # Even a <10 cm return must not be swallowed by the tracking deadband.
    assert keeper_target(p, pose, {'x_m': 2., 'y_m': 1.}) == (pose.home_x, edge)


def test_keeper_never_starts_walk_from_expired_ball(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        assert await game._prepare(sup.params.values, 'goalkeeper')
        ball = observation(.3)['result']
        ball['sensor_timestamp_ns'] -= 1_000_000_000
        game.keeper_watch = KeeperMotionGuard(game.odometry, ball, (game.odometry.home_x, .3))
        assert not await game._original_walk(sup.params.values, 'goalkeeper', 'omni', ball,
                                            distance_m=.1, course=math.pi/2, heading=0.)
        assert not any(op == 'game.walk' for op, _ in sup.workers['motherboard'].calls)
    asyncio.run(run())


def test_keeper_shift_cancels_once_gracefully_not_hard(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.odometry = GameOdometry(sup.params.values, 'goalkeeper', 0.)
        # The fake IMU uses a zero heading; guard field coordinates follow it.
        game.keeper_watch = KeeperMotionGuard(game.odometry, observation(.3)['result'],
                                               (game.odometry.home_x, .3))
        worker = sup.workers['motherboard']
        original = worker.call
        polls = 0
        async def call(op, args=None, **kwargs):
            nonlocal polls
            if op == 'job.status':
                polls += 1
                return {'status': 'running' if polls < 4 else 'cancelled'}
            return await original(op, args, **kwargs)
        worker.call = call
        frames = iter([observation(-.3, seq=2), observation(-.3, seq=3), observation(-.3, seq=4)])
        async def detection(op, args=None, **kwargs):
            return next(frames) if op == 'ball.status' else {}
        sup.workers['detection'].call = detection
        await game._wait_job({'job_id': 'keeper'}, walking=True)
        assert [op for op, _ in worker.calls].count('motion.stop_graceful') == 1
        assert not any(op == 'motion.stop_hard' for op, _ in worker.calls)
    asyncio.run(run())
