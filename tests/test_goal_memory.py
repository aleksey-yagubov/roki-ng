import asyncio
import math
import time

import pytest

from roki_ng.body_imu import quaternion_yaw
from test_simple_football import harness, commands


def sample(yaw=0.):
    # Mounted body IMU: upright is +Y, yaw is a rotation about world Z.
    a = math.sqrt(.5)
    return dict(valid=True, source_mono_ns=time.monotonic_ns(),
                quaternion_xyzw=[a*math.cos(yaw/2), a*math.sin(yaw/2),
                                 a*math.sin(yaw/2), a*math.cos(yaw/2)])


def prepared(tmp_path):
    s, g = harness(tmp_path, ball=(.13, .052))
    imu = sample()
    g.s.body_telemetry = {'body.imu': imu}
    g._imu.return_value = (quaternion_yaw(imu['quaternion_xyzw']), False)
    s.observe.return_value[1][0]['shootable'] = True
    asyncio.run(s.step())  # Visible opening, one forward alignment jump.
    assert s.goal_memory is not None
    g._move.reset_mock()
    s.observe.return_value = ((.09, .052), [])
    return s, g


def test_close_ball_kicks_after_goal_leaves_frame(tmp_path):
    s, g = prepared(tmp_path)
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'motion.kick'
    assert s.goal_memory is None


@pytest.mark.parametrize('cause', ['turn_return', 'fall', 'gap', 'expiry', 'pause', 'own_goal'])
def test_invalidated_memory_never_allows_blind_kick(tmp_path, cause):
    s, g = prepared(tmp_path)
    if cause == 'turn_return':
        s.check_goal_memory(sample(math.radians(11)))
        s.check_goal_memory(sample())
    elif cause == 'fall':
        s.check_goal_memory(sample() | {'quaternion_xyzw': [0, 0, 0, 1]})
    elif cause == 'gap':
        s.goal_memory['stamp'] -= 300_000_000
        s.check_goal_memory(sample())
    elif cause == 'expiry':
        s.goal_memory['until'] = time.monotonic()-1
    elif cause == 'pause':
        s.cancel_special('pause')
    else:
        s.observe.return_value = ((.09, .052), [dict(colour=s.own,
            ends=[[1., -.5], [1., .5]], shootable=True)])
    asyncio.run(s.step())
    assert not any(op == 'motion.kick' for op, _ in commands(g))
    assert s.goal_memory is None


def test_lateral_budget_counts_both_directions(tmp_path):
    s, g = prepared(tmp_path)
    s.p['motion.jump_left_mm'] = s.p['motion.jump_right_mm'] = 31
    asyncio.run(s.jump('left'))
    assert s.goal_memory is not None
    asyncio.run(s.jump('right'))
    assert s.goal_memory is None


def test_imu_invalidated_during_scene_close_prevents_kick(tmp_path):
    s, g = prepared(tmp_path)
    async def invalidate(*args, **kwargs):
        s.check_goal_memory(sample(math.radians(20)))
    g._call.side_effect = invalidate
    asyncio.run(s.step())
    assert not any(op == 'motion.kick' for op, _ in commands(g))
