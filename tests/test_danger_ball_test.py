import asyncio
import time
import math
import pytest
from unittest.mock import AsyncMock

from test_simple_football import harness, commands
from roki_ng.legacy_strategy import OriginalStrategy


def danger(tmp_path, x=.5, y=0.):
    s, g = harness(tmp_path)
    s.danger = True
    g._capture_now = time.monotonic_ns
    g._usable = OriginalStrategy._usable
    g._settle = AsyncMock(return_value=True)
    def observed(*args, **kw):
        return dict(running=True, error=None, age_ms=0, result=dict(valid=True,
            x_m=x, y_m=y, sensor_timestamp_ns=time.monotonic_ns(), frame_sequence=1))
    g._original_observe = AsyncMock(side_effect=observed)
    return s, g


def test_far_ball_uses_original_streaming_approach(tmp_path):
    s, g = danger(tmp_path, .46)
    asyncio.run(s.step())
    op, args = commands(g)[0]
    assert op == 'game.walk' and args['kind'] == 'approach'
    assert args['distance_m'] == .46 and args['x_m'] == .46
    assert 'stamp' in args


def test_350mm_zone_uses_only_jumps_and_keeps_leg(tmp_path):
    s, g = danger(tmp_path, .30, .02)
    asyncio.run(s.step())
    assert s.near and s.leg_sign == 1
    assert commands(g)[0][0] == 'game.jump'
    assert commands(g)[0][1]['direction'] == 'right'
    g._move.reset_mock()
    g._original_observe.side_effect = lambda *a, **k: dict(running=True, error=None, age_ms=0,
        result=dict(valid=True,x_m=.29,y_m=-.01,sensor_timestamp_ns=time.monotonic_ns()))
    asyncio.run(s.step())
    assert s.leg_sign == 1
    assert all(op == 'game.jump' for op, _ in commands(g))


def test_ready_ball_kicks_even_after_fifteenth_jump(tmp_path):
    s, g = danger(tmp_path, .099, -.052)
    s.near, s.leg_sign, s.jumps = True, -1, 15
    asyncio.run(s.step())
    assert commands(g)[0] == ('motion.kick', dict(leg='right',power=100,offset=-10))
    assert not s.near and s.jumps == 0


def test_no_sixteenth_alignment_jump(tmp_path):
    s, g = danger(tmp_path, .30, .052)
    s.near, s.leg_sign, s.jumps = True, 1, 15
    asyncio.run(s.step())
    assert not commands(g)
    assert g.info['phase'] == 'danger_new_attempt'


def test_pause_during_observation_blocks_motion(tmp_path):
    s, g = danger(tmp_path)
    g._same_plan = lambda rev: rev == g.revision
    async def paused(*a, **k):
        g.revision += 1
    g._original_observe.side_effect = paused
    asyncio.run(s.step())
    assert not commands(g)


def test_ball_leaving_zone_ends_attempt_before_any_walk(tmp_path):
    s, g = danger(tmp_path, .4)
    s.near, s.leg_sign, s.jumps = True, 1, 4
    asyncio.run(s.step())
    assert not commands(g) and not s.near


@pytest.mark.parametrize('side,sign,turn', [('left',1,'turn_left'), ('right',-1,'turn_right')])
def test_selected_angle_turns_then_reobserves_before_strike(tmp_path, side, sign, turn):
    s, g = danger(tmp_path, .099, -.052)
    s.kick_direction = side
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'game.jump'
    assert commands(g)[0][1]['direction'] == turn
    assert s.jumps == 1
    g._imu.return_value = (sign*math.radians(30), False)
    asyncio.run(s.step())
    assert commands(g)[1][0] == 'motion.kick'
    assert g._original_observe.await_count == 2
    assert s.angle_target is None


def test_angle_preference_cannot_delay_strike_forever(tmp_path):
    s, g = danger(tmp_path, .099, -.052)
    s.kick_direction = 'left'
    for _ in range(3):
        asyncio.run(s.step())
    assert [op for op, _ in commands(g)] == ['game.jump', 'game.jump', 'motion.kick']


def test_angle_respects_total_fifteen_jump_budget(tmp_path):
    s, g = danger(tmp_path, .099, -.052)
    s.kick_direction = 'right'
    s.near, s.leg_sign, s.jumps = True, -1, 15
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'motion.kick'


def test_kickoff_left_is_one_soft_strike_then_normal_game(tmp_path):
    s, g = danger(tmp_path, .099, -.052)
    s.kick_direction = 'kickoff_left'
    asyncio.run(s.step())
    assert math.isclose(s.angle_target, math.radians(60))
    assert commands(g)[0][1]['direction'] == 'turn_left'
    g._imu.return_value = (math.radians(60), False)
    asyncio.run(s.step())
    assert commands(g)[1][1]['power'] == 60
    assert s.kick_direction == 'straight'
    asyncio.run(s.step())
    assert commands(g)[2][0] == 'motion.kick'
    assert commands(g)[2][1]['power'] == 100


def test_pickup_cancels_kickoff_without_repeating(tmp_path):
    s, _ = danger(tmp_path)
    s.kick_direction = 'kickoff_left'
    s.cancel_special('pickup')
    assert s.kick_direction == 'straight'


def test_menu_angle_selection_preserves_entry_and_delay():
    from roki_ng.head_menu import menu_tree
    game = next(i for i in menu_tree().children if i.label == 'Game')
    danger_menu = next(i for i in game.children if i.label == 'Danger football')
    for position in danger_menu.children:
        for side in ('left', 'right'):
            option = next(i for i in position.children if i.label == 'Kick '+side)
            assert [i.args['delay_seconds'] for i in option.children] == [0, 10]
            assert all(i.args['kick_direction'] == side and i.args['entry'] == position.label.lower()
                       for i in option.children)
