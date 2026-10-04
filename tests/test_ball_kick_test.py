import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from roki_ng.ball_kick_test import run_ball_kick_test
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault
from test_game import setup, until


def test_head_menu_exposes_explicit_one_shot_start():
    from roki_ng.head_menu import menu_tree
    tests = next(item for item in menu_tree().children if item.label == 'Tests')
    item = next(item for item in tests.children if item.args.get('strategy') == 'ball_kick_test')
    assert item.op == 'game.start'
    assert 'entry' not in item.args


def fixture(tmp_path, points):
    observations = [dict(x_m=x, y_m=y, frame_sequence=i) if x is not None else None
                    for i, (x, y) in enumerate(points)]
    game = SimpleNamespace(revision=1, info={}, odometry=SimpleNamespace(yaw=0., home_heading=0.),
        s=SimpleNamespace(log=lambda *a: None), _same_plan=lambda r: True,
        _original_observe=AsyncMock(side_effect=observations), _usable=lambda b: b,
        _original_turn=AsyncMock(return_value=True), _original_walk=AsyncMock(return_value=True),
        _move=AsyncMock(return_value=True), _imu=AsyncMock(), _original_kick=AsyncMock(return_value=True))
    return game, Parameters(tmp_path).values


@pytest.mark.parametrize('sign', [-1, 1])
def test_far_approach_then_jumps_then_one_kick(tmp_path, sign):
    game, p = fixture(tmp_path, [(1., sign*.1), (.9, sign*.02), (.16, sign*.08),
                                 (.16, sign*.052), (.13, sign*.052), (.10, sign*.052)])
    assert asyncio.run(run_ball_kick_test(game, p, 'center'))
    assert [c.args[2] for c in game._original_walk.await_args_list] == ['approach']
    assert [c.args[1]['direction'] for c in game._move.await_args_list] == [
        'left' if sign > 0 else 'right', 'forward', 'forward']
    assert game._original_kick.await_count == 1
    assert game._original_kick.await_args.args[5] == pytest.approx(sign*52)


def test_close_centered_ball_aligns_to_foot_without_walking(tmp_path):
    game, p = fixture(tmp_path, [(.09, 0.), (.09, .03), (.09, .052)])
    assert asyncio.run(run_ball_kick_test(game, p, 'center'))
    assert game._original_walk.await_count == game._original_turn.await_count == 0
    assert [c.args[1]['direction'] for c in game._move.await_args_list] == ['right', 'right']
    assert game._original_kick.await_count == 1


def test_five_jumps_without_progress_never_restart_walking_or_kick(tmp_path):
    game, p = fixture(tmp_path, [(.2, .052)]*6)
    with pytest.raises(Fault, match='Five jumps'):
        asyncio.run(run_ball_kick_test(game, p, 'center'))
    assert game._move.await_count == 5
    assert game._original_walk.await_count == game._original_kick.await_count == 0


@pytest.mark.parametrize('after', [(None, None), (.6, .052)])
def test_lost_or_distant_ball_after_jump_cannot_kick(tmp_path, after):
    game, p = fixture(tmp_path, [(.2, .052), after])
    with pytest.raises(Fault):
        asyncio.run(run_ball_kick_test(game, p, 'center'))
    assert game._move.await_count == 1
    assert game._original_kick.await_count == game._original_walk.await_count == 0


def test_failed_jump_and_cancel_do_not_continue(tmp_path):
    game, p = fixture(tmp_path, [(.2, .052)])
    game._move.return_value = False
    assert not asyncio.run(run_ball_kick_test(game, p, 'center'))
    assert game._original_kick.await_count == 0
    game, p = fixture(tmp_path, [(.09, .052)])
    game._same_plan = lambda r: False
    with pytest.raises(Fault, match='interrupted'):
        asyncio.run(run_ball_kick_test(game, p, 'center'))
    assert game._move.await_count == game._original_kick.await_count == 0


@pytest.mark.parametrize('fails', [False, True])
def test_lifecycle_finishes_once_without_kickoff_localisation_or_recovery(tmp_path, monkeypatch, fails):
    async def run():
        game, s, session = setup(tmp_path)
        async def test(g, p, entry):
            assert p['game.use_visual_localisation'] is False
            assert entry == 'center'
            if fails:
                raise Fault('fallen', 'Test fall')
            return True
        runner = AsyncMock(side_effect=test)
        monkeypatch.setattr('roki_ng.game.run_ball_kick_test', runner)
        await game.start(session, {'strategy': 'ball_kick_test'})
        await until(lambda: not game.info['running'])
        assert runner.await_count == 1
        assert game.info['state'] == ('failed' if fails else 'completed')
        assert s.mode == 'MANUAL'
        assert s.params.values['game.use_visual_localisation'] is True
        assert not any(op in ('game.walk', 'motion.get_up') for op, _ in s.workers['motherboard'].calls)
        assert any(op == 'ball.stop' for op, _ in s.workers['detection'].calls)
    asyncio.run(run())


def test_fallen_at_start_does_not_attempt_get_up(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        game._imu = AsyncMock(return_value=(0., True))
        await game.start(session, {'strategy': 'ball_kick_test'})
        await until(lambda: not game.info['running'])
        assert game.info['state'] == 'failed'
        assert not any(op in ('motion.get_up', 'game.walk', 'game.jump', 'motion.kick')
                       for op, _ in s.workers['motherboard'].calls)
    asyncio.run(run())
