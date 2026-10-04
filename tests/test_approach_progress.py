"""Regression coverage for Simple's transfer to the ball-test approach."""
import asyncio
import math
import time
from unittest.mock import AsyncMock

import pytest

from test_simple_football import harness, commands
from test_goal_memory import sample


def simple(tmp_path, ball=(.46, 0.), goals=None):
    s, g = harness(tmp_path, ball=ball, goals=goals)
    if goals is None:
        s.observe.return_value[1][0]['shootable'] = True
    g.s.body_telemetry = {'body.imu': sample()}
    g._settle = AsyncMock(return_value=True)
    s.last_scene = {'received': time.monotonic()}
    s.shot_deadline = time.monotonic()+.5
    return s, g


@pytest.mark.parametrize('distance', [.351, .40, .451, .461, .499, .625])
def test_far_ball_uses_test_streaming_approach_not_tiny_omni(tmp_path, distance):
    s, g = simple(tmp_path, (distance, 0.))
    asyncio.run(s.step())
    op, args = commands(g)[0]
    assert op == 'game.walk' and args['kind'] == 'approach'
    assert args['distance_m'] == distance
    assert args['x_m'] == distance and 'stamp' in args
    assert s.observe.await_args.kwargs == {'goals': False}


def test_35cm_zone_uses_shared_alignment_and_fixed_leg(tmp_path):
    s, g = simple(tmp_path, (.30, .02), [])
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'game.jump'
    assert commands(g)[0][1]['direction'] == 'right'
    assert s.leg_sign == 1
    s.observe.return_value = ((.28, -.01), [])
    asyncio.run(s.step())
    assert s.leg_sign == 1
    assert all(op == 'game.jump' for op, _ in commands(g))


def test_ready_ball_without_goal_orbits_then_faces_fresh_ball(tmp_path):
    s, g = simple(tmp_path, (.099, -.052), [])
    asyncio.run(s.step())
    assert commands(g)[0][1]['direction'] == 'right'
    assert s.orbit_face_ball
    s.observe.return_value = ((.10, .04), [])
    asyncio.run(s.step())
    assert commands(g)[1][1]['direction'] == 'turn_left'
    assert not s.observe.await_args.kwargs['goals']


def test_ready_ball_with_clear_enemy_opening_kicks(tmp_path):
    s, g = simple(tmp_path, (.099, -.052))
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'motion.kick'


def test_own_goal_still_vetoes_ready_kick(tmp_path):
    s, g = simple(tmp_path, (.099, -.052))
    s.observe.return_value[1].append(dict(colour=s.own, shootable=True,
        ends=[[1., -.5], [1., .5]], bearings=[[-.2, .2]]))
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'game.jump'
    assert all(op != 'motion.kick' for op, _ in commands(g))


def test_fifteen_jumps_no_backwards_reposition_or_sixteenth_jump(tmp_path):
    s, g = simple(tmp_path, (.3, .052), [])
    s.near, s.leg_sign, s.jumps = True, 1, 15
    asyncio.run(s.step())
    assert not commands(g)
    assert g.info['phase'] == 'simple_new_attempt'


def test_ball_leaving_alignment_zone_ends_attempt_before_walk(tmp_path):
    s, g = simple(tmp_path, (.4, .0))
    s.near, s.leg_sign, s.jumps = True, 1, 3
    asyncio.run(s.step())
    assert not commands(g) and not s.near


def test_expired_ball_never_starts_walk(tmp_path):
    s, g = simple(tmp_path)
    s.shot_deadline = time.monotonic()-1
    asyncio.run(s.step())
    assert not commands(g)


def test_kickoff_can_finish_heading_after_two_turns(tmp_path):
    s, g = simple(tmp_path, (.099, -.052), [])
    s.deadline = time.monotonic()+10
    s.kickoff_heading = math.radians(30)
    s.near, s.jumps, s.fine_heading_attempts = True, 7, 2
    asyncio.run(s.step())
    assert commands(g)[0][1]['direction'] == 'turn_left'
    assert s.jumps == 8


def test_ball_lost_uses_one_head_look_then_body_not_forward(tmp_path):
    s, g = simple(tmp_path)
    s.observe.return_value = (None, [])
    s.look = AsyncMock(return_value=None)
    asyncio.run(s.step())
    assert s.look.await_count == 1
    assert commands(g)
    assert all(op == 'game.jump' and args['direction'].startswith('turn_')
               for op, args in commands(g))
