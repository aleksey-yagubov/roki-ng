import asyncio
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from roki_ng.parameters import Parameters
from roki_ng.simple_goalkeeper import SimpleGoalkeeper
from roki_ng.head_menu import menu_tree
from test_game import observation, setup


def keeper(tmp_path, ball=(.4, .2)):
    g = SimpleNamespace(revision=1, info={}, _same_plan=lambda r: r == g.revision,
        _prepare=AsyncMock(return_value=True), _imu=AsyncMock(return_value=(0., False)),
        _call=AsyncMock(return_value={}), _move=AsyncMock(return_value=True),
        s=SimpleNamespace(log=lambda *a: None))
    k = SimpleGoalkeeper(g, Parameters(tmp_path).values, 'goalkeeper', 0)
    k.look = AsyncMock(return_value={'ball': ball, 'parts': [], 'received': time.monotonic()})
    return k, g


@pytest.mark.parametrize('ball,direction', [((.4,.2),'left'), ((.4,-.2),'right'), ((.4,.05),None), (None,None)])
def test_only_one_bounded_lateral_jump(tmp_path, ball, direction):
    k, g = keeper(tmp_path, ball)
    asyncio.run(k.step())
    calls = g._move.await_args_list
    if direction:
        assert len(calls) == 1 and calls[0].args[0] == 'game.jump'
        assert calls[0].args[1]['direction'] == direction
        assert calls[0].args[1]['fraction']*k.p[f'motion.jump_{direction}_mm'] <= 20.01
    else:
        assert not calls


def test_close_ball_never_raises_head_for_goal(tmp_path):
    k, _ = keeper(tmp_path)
    asyncio.run(k.step())
    assert k.look.await_args_list[0].args == (0, k.p['game.head_tilt'])
    assert k.look.await_count == 1


def test_scan_uses_new_ball_not_pre_scan_position(tmp_path):
    k, g = keeper(tmp_path, (1.5,.3))
    k.look.side_effect = [{'ball': (1.5,.3), 'parts': []},
        {'ball': None, 'parts': [dict(kind='colour', colour=k.enemy, points=[[2.,0.]], area=100)]},
        {'ball': (.4,-.2), 'parts': [], 'received': time.monotonic()}]
    asyncio.run(k.step())
    assert k.look.await_args_list[1].args == (0, 0)
    assert g._move.await_args.args[1]['direction'] == 'right'


def test_pause_during_scan_never_moves(tmp_path):
    k, g = keeper(tmp_path)
    async def paused(*args):
        g.revision += 1
        return {'ball': (.4,.2), 'parts': []}
    k.look.side_effect = paused
    asyncio.run(k.step())
    assert not g._move.await_count


def test_ball_direction_changes_next_jump(tmp_path):
    k, g = keeper(tmp_path)
    asyncio.run(k.step())
    k.observe_motion_ball(observation(-.2, 2))
    k.look.return_value = {'ball': (.4,-.2), 'parts': [], 'received': time.monotonic()}
    asyncio.run(k.step())
    assert [c.args[1]['direction'] for c in g._move.await_args_list] == ['left','right']


def test_live_observation_is_consumed_during_body_job(tmp_path):
    async def run():
        g, s, _ = setup(tmp_path)
        k = SimpleGoalkeeper(g, s.params.values, 'goalkeeper', 0)
        g.simple = k
        k.move_ball = (.4,.2)
        worker = s.workers['motherboard']
        original = worker.call
        count = 0
        async def call(op, args=None, **kw):
            nonlocal count
            if op == 'job.status':
                count += 1
                return {'status': 'running' if count < 3 else 'completed', 'pose': 'base_stand'}
            return await original(op, args, **kw)
        worker.call = call
        await g._wait_job({'job_id': '1'})
        assert k.live_sequence >= 2
        assert any(op == 'ball.status' for op, _ in s.workers['detection'].calls)
        assert g.odometry is None
    asyncio.run(run())


def test_voice_menu_exposes_separate_keeper():
    game = next(i for i in menu_tree().children if i.label == 'Game')
    simple = next(i for i in game.children if i.label == 'Simple football')
    for colour in simple.children:
        keeper_item = next(i for i in colour.children if i.label == 'Goalkeeper')
        assert keeper_item.children[0].args == {'strategy': 'simple_goalkeeper'}


def test_stale_ball_never_dispatches_jump(tmp_path):
    k, g = keeper(tmp_path)
    k.look.return_value['received'] -= 1
    asyncio.run(k.step())
    assert not g._move.await_count


def test_recovery_requires_new_goal_direction(tmp_path):
    k, g = keeper(tmp_path)
    k.cancel_special('fall')
    asyncio.run(k.step())
    assert k.reacquire_heading
    assert not g._move.await_count
    assert k.look.await_args_list[1].args == (0, 0)


def test_keeper_colour_hint_does_not_require_posts_or_opening(monkeypatch, tmp_path):
    np = pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    from roki_ng.football_vision import goal_parts
    binary = np.zeros((650, 800), np.uint8)
    binary[300:305, 400:410] = 255
    monkeypatch.setattr('roki_ng.colour_masks.mask', lambda *a: binary)
    projector = SimpleNamespace(_pixel_ray=lambda *a: [1., 0., -.2])
    args = (np.zeros((650,800,3), np.uint8), Parameters(tmp_path).values,
            projector, [0,0,0,1], {'pan':0})
    assert not goal_parts(*args)
    relaxed = goal_parts(*args, bearing_only=True)
    assert len(relaxed) == 2
    assert all(part['kind'] == 'colour' for part in relaxed)


def test_new_role_has_no_odometry_or_entry(tmp_path):
    async def run():
        g, s, session = setup(tmp_path)
        status = await g.start(session, {'strategy': 'simple_goalkeeper', 'delay_seconds': 30})
        assert status['role'] == 'simple_goalkeeper'
        assert isinstance(g.simple, SimpleGoalkeeper) and g.odometry is None
        assert not g.simple.kickoff_pending and not g.simple.entry_pending
        await g.stop()
    asyncio.run(run())
