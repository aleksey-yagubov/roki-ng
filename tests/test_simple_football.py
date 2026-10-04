import asyncio
import math
import time
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from roki_ng.simple_football import SimpleFootball
from roki_ng.football_vision import assemble_goals, shot_clear, camera_geometry, body_point, SceneTracker
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault
from test_game import setup, until


def harness(tmp_path, ball=(.09, .052), goals=None, entry='center', delay=10):
    p = Parameters(tmp_path).values
    g = SimpleNamespace(revision=1, info={}, _allowed=lambda: True, _same_plan=lambda r: True,
                        s=SimpleNamespace(log=lambda *a: None),
                        _imu=AsyncMock(return_value=(0., False)), _call=AsyncMock(return_value={}),
                        _settle=AsyncMock(return_value=True),
                        _move=AsyncMock(return_value=True))
    strategy = SimpleFootball(g, p, entry, delay)
    goal = dict(colour=strategy.enemy, ends=[[1., -.5], [1., .5]], elements='posts')
    strategy.observe = AsyncMock(return_value=(ball, [goal] if goals is None else goals))
    return strategy, g


def commands(g):
    return [(c.args[0], c.args[1]) for c in g._move.await_args_list]


def test_ready_ball_kicks_immediately_without_map_or_further_alignment(tmp_path):
    s, g = harness(tmp_path)
    asyncio.run(s.step())
    assert commands(g) == [('motion.kick', {'leg': 'left', 'power': 100, 'offset': -10})]


def test_small_heading_error_without_clear_opening_orbits_instead_of_turning(tmp_path):
    s, g = harness(tmp_path, goals=[dict(colour='blue', ends=[[3., .1], [3., 1.1]])])
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'game.jump'
    assert commands(g)[0][1]['direction'] in ('left', 'right')
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_crossing_heading_target_does_not_reverse_turn_jump(tmp_path):
    s, g = harness(tmp_path, ball=(.2, 0.), goals=[dict(colour='blue', ends=[[1., .2], [1., .4]])])
    asyncio.run(s.step())
    assert commands(g)[0][1]['direction'] == 'turn_left'
    s.observe.return_value = ((.2, 0.), [dict(colour='blue', ends=[[1., -.4], [1., -.2]])])
    asyncio.run(s.step())
    assert commands(g)[1][1]['direction'] in ('left', 'right')


def test_lost_ball_at_jump_limit_repositions_instead_of_stalling(tmp_path):
    s, g = harness(tmp_path, ball=None)
    s.near, s.jumps = True, 15
    asyncio.run(s.step())
    assert commands(g) == [('game.jump', {'direction': 'backward', 'fraction': 1.})]
    assert s.jumps == 0


def test_pause_resume_during_scene_close_never_dispatches_old_action(tmp_path):
    async def run():
        s, g = harness(tmp_path)
        g._same_plan = lambda r: r == g.revision
        async def changed(*a, **kw):
            g.revision += 2
        g._call.side_effect = changed
        assert not await s.move('motion.kick', {'leg': 'left'})
        assert not commands(g)
    asyncio.run(run())


def test_cancelled_goal_scan_does_not_start_another_head_job(tmp_path):
    async def run():
        s, g = harness(tmp_path)
        g._same_plan = lambda r: r == g.revision
        g._prepare = AsyncMock(return_value=True)
        async def changed(*a, **kw):
            g.revision += 2
            return None
        s.look = AsyncMock(side_effect=changed)
        assert await SimpleFootball.observe(s, goals=True) == (None, [])
        assert s.look.await_count == 1
        assert not commands(g)
    asyncio.run(run())


def test_cancelled_imu_read_does_not_retarget_kickoff(tmp_path):
    async def run():
        s, g = harness(tmp_path, delay=0)
        g._same_plan = lambda r: r == g.revision
        async def changed():
            g.revision += 2
            s.cancel_special('pause')
            return 0., False
        g._imu.side_effect = changed
        await s.step()
        assert s.deadline is None and not s.kickoff_pending
        assert not commands(g)
    asyncio.run(run())


def test_shot_log_distinguishes_own_goal_veto_and_foot_alignment(tmp_path):
    goals = [dict(colour=c, ends=[[1., -.5], [1., .5]]) for c in ('yellow', 'blue')]
    s, g = harness(tmp_path, ball=(.09, 0.), goals=goals)
    lines = []
    g.s.log = lambda source, level, message: lines.append(message)
    asyncio.run(s.step())
    assert any('own_goal' in line and 'foot_alignment' in line for line in lines)
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_near_ball_uses_jumps_only(tmp_path):
    s, g = harness(tmp_path, ball=(.24, .052))
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'game.jump'
    assert s.jumps == 1
    s.observe.return_value = ((.26, .052), s.observe.return_value[1])
    asyncio.run(s.step())
    assert all(op != 'game.walk' for op, _ in commands(g))


def test_fifteenth_jump_checks_strike_before_reposition(tmp_path):
    s, g = harness(tmp_path)
    s.near, s.jumps = True, 15
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'motion.kick'
    s, g = harness(tmp_path, ball=(.2, .052))
    s.near, s.jumps = True, 15
    asyncio.run(s.step())
    assert commands(g)[0] == ('game.jump', {'direction': 'backward', 'fraction': 1.})
    assert s.jumps == 0


def test_failed_reposition_does_not_reset_attempt_in_place(tmp_path):
    s, g = harness(tmp_path, ball=(.2, .052))
    s.near, s.jumps = True, 15
    # A cancelled movement invalidates the whole plan, not another fine pass.
    g._move.return_value = False
    asyncio.run(s.step())
    assert len(commands(g)) == 1
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_no_goal_or_own_goal_in_corridor_never_kicks(tmp_path):
    for goals in ([], [dict(colour='yellow', ends=[[1., -.5], [1., .5]])]):
        s, g = harness(tmp_path, goals=goals)
        asyncio.run(s.step())
        assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_own_goal_veto_applies_even_with_opponent_goal(tmp_path):
    goals = [dict(colour=colour, ends=[[1., -.5], [1., .5]]) for colour in ('blue', 'yellow')]
    s, g = harness(tmp_path, goals=goals)
    asyncio.run(s.step())
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_map_coordinates_and_absolute_yaw_do_not_choose_action(tmp_path):
    outcomes = []
    for yaw in (-2.4, 0., 1.8):
        s, g = harness(tmp_path, ball=(.2, .052))
        for spec in s.specs:
            spec['x'], spec['y'] = yaw*100, -yaw*100
        g._imu.return_value = (yaw, False)
        asyncio.run(s.step())
        outcomes.append(commands(g))
    assert outcomes[0] == outcomes[1] == outcomes[2]


def test_delayed_or_disabled_center_has_no_kickoff(tmp_path):
    s, _ = harness(tmp_path, delay=10)
    assert not s.kickoff_pending
    s.p['game.forward.kick_off_ride'] = False
    assert not SimpleFootball(s.g, s.p, 'center', 0).kickoff_pending


@pytest.mark.parametrize('entry', ['left', 'right'])
def test_entry_issued_once_even_if_execution_unknown(tmp_path, entry):
    s, g = harness(tmp_path, entry=entry, delay=0)
    g._move.side_effect = Fault('body_unavailable', 'unknown outcome')
    with pytest.raises(Fault):
        asyncio.run(s.step())
    assert not s.entry_pending and not s.kickoff_pending
    assert commands(g)[0][1]['distance_m'] == .2


@pytest.mark.parametrize('sign,side', [(1, 'left'), (-1, 'right')])
def test_kickoff_relative_heading_and_no_goal_exception(tmp_path, sign, side):
    s, g = harness(tmp_path, goals=[], delay=0)
    s.p['game.simple_football.kickoff_side'] = side
    g._imu.side_effect = [(1.4, False), (1.4+sign*math.pi/3, False)]
    asyncio.run(s.step())
    assert commands(g)[0][0] == 'motion.kick'
    assert not s.kickoff_pending


def test_expired_kickoff_cannot_strike_without_goal(tmp_path):
    s, g = harness(tmp_path, goals=[])
    s.deadline, s.kickoff_heading = time.monotonic()-1, 0.
    asyncio.run(s.step())
    assert not any(op == 'motion.kick' for op, _ in commands(g))


def test_scene_requires_distinct_consistent_frames_and_pose(tmp_path):
    tracker = SceneTracker()
    frame = dict(pose_id=1, sequence=1, ball=[.2, 0.], parts=[])
    assert tracker.update(frame)['ball'] is None
    assert tracker.update(frame)['ball'] is None
    assert tracker.update(frame | {'sequence': 2})['ball'] == [.2, 0.]
    assert tracker.update(frame | {'pose_id': 2, 'sequence': 3})['ball'] is None


@pytest.mark.parametrize('combination', ['posts', 'left_bar', 'right_bar'])
def test_any_two_goal_elements_without_map(combination):
    posts = [dict(kind='post', colour='blue', points=[point]) for point in ([1., -.5], [1., .5])]
    bar = dict(kind='bar', colour='blue', points=[[1., -.5], [1., .5]])
    parts = posts if combination == 'posts' else [posts[0 if combination == 'left_bar' else 1], bar]
    goal = assemble_goals(parts, [dict(colour='blue', width=1., x=999, y=-999)])[0]
    assert shot_clear([.1, 0.], goal, .095)
    assert not shot_clear([.1, .49], goal, .095)
    assert not shot_clear([2., 0.], goal, .095)


def test_single_post_and_ambiguous_pairs_not_goal():
    post = lambda x, y: dict(kind='post', colour='blue', points=[[x, y]])
    specs = [dict(colour='blue', width=1.)]
    assert assemble_goals([post(1, 0)], specs) == []
    assert assemble_goals([post(1, -.5), post(1, .5), post(3, -.5), post(3, .5)], specs) == []


def test_camera_reference_and_pan_are_body_relative(tmp_path):
    p = Parameters(tmp_path).values
    p.update({'game.camera_height_m': .43, 'game.camera_reference_tilt': -1000, 'game.camera_forward_m': .04})
    geometry = camera_geometry(p, 0, -1000)
    assert geometry['height'] == .43
    assert body_point([1., 0.], geometry) == pytest.approx([1.04, 0.])
    assert camera_geometry(p, 0, -1500)['height'] < .43
    assert body_point([1., 0.], camera_geometry(p, -1778, -1000))[1] > .85


def test_lifecycle_never_constructs_odometry_or_starts_localisation(tmp_path, monkeypatch):
    async def run():
        game, sup, session = setup(tmp_path)
        monkeypatch.setattr('roki_ng.game.GameOdometry', lambda *a: pytest.fail('Odometry forbidden'))
        calls = []
        async def step(self):
            assert self.g.odometry is None
            assert not self.p['game.use_visual_localisation']
            calls.append(1)
            await asyncio.sleep(.01)
        monkeypatch.setattr(SimpleFootball, 'step', step)
        await game.start(session, {'strategy': 'simple_football', 'entry': 'center'})
        await until(lambda: calls)
        await game.stop()
        assert game.odometry is None
        start = next(args for op, args in sup.workers['detection'].calls if op == 'ball.start')
        assert start['football']
    asyncio.run(run())


def test_kickoff_expiry_during_pre_dispatch_await_cannot_send_kick(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.simple = SimpleFootball(game, sup.params.values, 'center', 0)
        game.simple.kickoff_pending = False
        game.simple.deadline = time.monotonic()+30
        async def invalidate(op, args=None, **kwargs):
            game.simple.deadline = time.monotonic()-1
            return {}
        sup.workers['detection'].call = invalidate
        assert not await game.simple.move('motion.kick', {'leg': 'left'})
        assert not any(op == 'motion.kick' for op, _ in sup.workers['motherboard'].calls)
        assert game.simple.deadline is None
    asyncio.run(run())


def test_unknown_kick_consumes_special_permission_before_acceptance(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.simple = SimpleFootball(game, sup.params.values, 'center', 0)
        game.simple.kickoff_pending = False
        game.simple.deadline = time.monotonic()+30
        game.simple.shot_deadline = time.monotonic()+.5
        original = sup.workers['motherboard'].call
        async def lost(op, args=None, **kwargs):
            if op == 'motion.kick':
                raise Fault('timeout', 'Unknown execution')
            return await original(op, args, **kwargs)
        sup.workers['motherboard'].call = lost
        with pytest.raises(Fault):
            await game.simple.move('motion.kick', {'leg': 'left'})
        assert game.simple.deadline is None and not game.simple.kickoff_pending
        assert game.motion_inflight  # Coordinator must resolve/cancel, not blindly repeat.
    asyncio.run(run())


def test_expiry_during_walk_requests_graceful_stop(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.simple = SimpleFootball(game, sup.params.values, 'center', 0)
        game.simple.kickoff_pending = False
        game.simple.deadline = time.monotonic()-1
        original = sup.workers['motherboard'].call
        count = 0
        async def status(op, args=None, **kwargs):
            nonlocal count
            if op == 'job.status':
                count += 1
                return {'status': 'running' if count == 1 else 'cancelled'}
            return await original(op, args, **kwargs)
        sup.workers['motherboard'].call = status
        result = await game._wait_job({'job_id': 'test'})
        assert result['status'] == 'cancelled'
        assert any(op == 'motion.stop_graceful' for op, _ in sup.workers['motherboard'].calls)
        assert game.simple.deadline is None
    asyncio.run(run())


def test_pause_drops_kickoff_reference(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.info['running'] = True
        game.simple = SimpleFootball(game, sup.params.values, 'center', 0)
        game.simple.deadline = time.monotonic()+30
        game.simple.kickoff_heading = 1.
        await game.pause()
        assert game.simple.deadline is None and game.simple.kickoff_heading is None
        assert not game.simple.kickoff_pending
    asyncio.run(run())


def test_expired_scene_never_dispatches_kick(tmp_path):
    async def run():
        game, sup, _ = setup(tmp_path)
        game.simple = SimpleFootball(game, sup.params.values, 'center', 10)
        game.simple.shot_deadline = time.monotonic()-1
        assert not await game.simple.move('motion.kick', {'leg': 'left'})
        assert not any(op == 'motion.kick' for op, _ in sup.workers['motherboard'].calls)
    asyncio.run(run())


def test_simple_menu_goal_selection_before_entry_does_not_start_motion():
    from roki_ng.head_menu import HeadMenu
    async def run():
        calls = []
        async def command(op, args):
            calls.append((op, args))
            return {'colour': args['colour']}
        menu = HeadMenu(command, AsyncMock(), lambda *a: None, lambda *a: None)
        menu.press('ok')  # Game
        assert menu.selected.label == 'Football'
        assert 'Simple football' not in [i.label for i in menu.selected.children]
        menu.press('right')
        assert menu.selected.label == 'Simple football'
        menu.press('ok')
        assert menu.selected.label == 'Own yellow goal'
        menu.press('back')
        assert menu.selected.label == 'Simple football'
        assert not calls
        menu.press('ok')
        menu.press('ok')
        await menu.action
        assert calls == [('menu.own_goal', {'colour': 'yellow'})]
        assert menu.selected.label == 'Left'
        menu.press('right')
        menu.press('ok')
        assert menu.selected.args['strategy'] == 'simple_football'
        assert menu.selected.args['entry'] == 'center'
        assert len(calls) == 1

        menu.game_active, menu.job = True, 'game1'
        await menu.game_finished(dict(job_id='game1', running=True, pickup=True,
                                      pickup_ready=True, pickup_confirmation_required=False))
        for _ in range(3):
            menu.press('right')
        assert menu.selected.label == 'Simple football'
        menu.press('ok')
        assert menu.selected.label == 'Own yellow goal'
        assert len(calls) == 1
    asyncio.run(run())
