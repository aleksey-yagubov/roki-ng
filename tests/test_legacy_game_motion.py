"""Compare manoeuvre outputs against executable original functions offline."""
import ast
import math
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from roki_ng import legacy_game_motion as legacy
from roki_ng.body import Body
from roki_ng.parameters import Parameters
from roki_ng.game_geometry import camera_height
from roki_ng.legacy_strategy import TABLE, kick_target
from roki_ng.odometry import GameOdometry


def original_method(name):
    path = Path(__file__).resolve().parents[2]/'Roki_2_Soccer/Soccer/Motion/class_Motion_real.py'
    if not path.exists():
        pytest.skip('Original source checkout required for differential test')
    tree = ast.parse(path.read_text())
    method = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == name)
    scope = {'math': math, 'print': lambda *a: None, 'random': SimpleNamespace(random=lambda: 1.)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
    return scope[name]


def legacy_fixture(p):
    first, stride, right, left = legacy.yields(p)
    frames = []
    obj = SimpleNamespace(first_step_yield=first, cycle_step_yield=stride,
                          side_step_right_yield=right, side_step_left_yield=left,
                          first_Leg_Is_Right_Leg=True, neck_play_pose=-1500,
                          refresh_Orientation=lambda: None, imu_body_yaw=lambda: 0.,
                          normalize_rotation=lambda v: legacy.clip(v, .3),
                          walk_Initial_Pose=lambda: None, walk_Final_Pose=lambda: None,
                          head_Return=lambda *a: None,
                          local=SimpleNamespace(coord_odometry=[0., 0., 0.]),
                          glob=SimpleNamespace(hardcode_walking=False, robot_see_ball=-1))
    obj.walk_Cycle = lambda step, side, rotation, i, total, **kw: frames.append(
        (step, side, rotation, i, total, obj.first_Leg_Is_Right_Leg, kw.get('half', False)))
    return obj, frames


@pytest.mark.parametrize('distance,direction,one_off', [
    (100, math.pi/2, True), (340, -math.pi/2, True), (200, 0., True),
    (700, 0., True), (-40, 0., False), (180, 2.55, True), (20, -2.55, True)])
def test_omni_matches_original_cycle_orders(tmp_path, distance, direction, one_off):
    p = Parameters(tmp_path).values
    obj, expected = legacy_fixture(p)
    original_method('near_distance_omni_motion')(obj, distance, direction, one_off)
    actual = [(s.step, s.side, legacy.clip(s.heading, .3)*(1 if s.right else -1),
               s.index, s.total, s.right, s.half)
              for s in legacy.omni(p, distance, direction, 0., one_off=one_off)]
    assert len(actual) == len(expected)
    for left, right in zip(actual, expected):
        assert left == pytest.approx(right)


def test_kickoff_leads_in_from_centre_mark_then_matches_original_turn_arc_and_tail(tmp_path):
    p = Parameters(tmp_path).values
    obj, expected = legacy_fixture(p)
    original_method('kick_off_ride')(obj)
    lead_in = list(legacy.omni(p, 200., 0., 0.))
    recipe = list(legacy.kickoff(p, 1, lambda: None))
    assert recipe[:len(lead_in)] == lead_in
    actual = [(s.step, s.side, legacy.clip(s.heading, .3), s.index, s.total, s.right, s.half)
              for s in recipe[len(lead_in):]]
    assert len(actual) == len(expected)
    for left, right in zip(actual, expected):
        assert left == pytest.approx(right)
    assert any(s[0] > 0 for s in actual)  # Not the old single forward jump.


@pytest.mark.parametrize('goal', [0, 1])
def test_every_strategy_cell_preserves_original_power_and_quantized_angle(tmp_path, goal):
    p = Parameters(tmp_path).values
    p['match.own_goal'] = goal
    pos = GameOdometry(p, 'goalkeeper', .8)
    sign = 1 if goal == 0 else -1
    length, width = p['field.geometry']['length'], p['field.geometry']['width']
    for col, row, power, angle in TABLE:
        pos.x = sign*((col+.5)*length/20-length/2)
        pos.y = sign*(width/2-(row+.5)*width/15)
        heading, strength = kick_target(p, pos, {'x_m': 0., 'y_m': 0.})
        assert strength == {1: 100, 2: 60, 3: 20}[power]
        assert math.cos(heading-pos.home_heading-int(angle*40)/40) == pytest.approx(1.)


def test_streaming_approach_decelerates_and_stops_when_ball_is_close(tmp_path):
    p = Parameters(tmp_path).values
    ball = [(.8, -.1)]
    recipe = legacy.approach(p, .8, .2, lambda: ball[0])
    first = next(recipe)
    assert first.step == pytest.approx(30*.33) and first.right
    ball[0] = (.2, .04)
    rest = list(recipe)
    assert any(s.half for s in rest)  # Leading-leg change, not an abrupt swap.
    assert rest[-1].index == rest[-1].total-1
    assert len(rest) < 10
    assert list(legacy.approach(p, .8, 0., lambda: None)) == []


@pytest.mark.parametrize('point', [(.8, -.1), (.2, .03)])
def test_approach_matches_original_with_live_stationary_ball(tmp_path, point):
    p = Parameters(tmp_path).values
    obj, expected = legacy_fixture(p)
    obj.params = {'KICK_ADJUSTMENT_DISTANCE_1': p['game.kick_approach_mm']}
    obj.local.ball_odometry = list(point)
    obj.body_euler_angle = {'yaw': 0.}
    obj.jump_turn = lambda *a: None
    obj.glob.robot_see_ball = 5
    obj.glob.ball_distance = math.hypot(*point)
    obj.glob.ball_course = math.atan2(point[1], point[0])
    original_method('far_distance_straight_approach_streaming')(obj)
    orders = legacy.approach(p, math.hypot(*point), obj.glob.ball_course, lambda: point)
    actual = [(s.step, s.side, legacy.clip(s.heading, .3)*(1 if s.right else -1),
               s.index, s.total, s.right, s.half) for s in orders]
    assert len(actual) == len(expected)
    for left, right in zip(actual, expected):
        assert left == pytest.approx(right)


@pytest.mark.parametrize('point', [(.4, -.1), (.075, .052)])
def test_verify_matches_original_orders_for_fixed_ball(tmp_path, point):
    p = Parameters(tmp_path).values
    obj, expected = legacy_fixture(p)
    obj.params = {'KICK_ADJUSTMENT_DISTANCE_1': p['game.kick_approach_mm'],
                  'KICK_OFFSET_OF_BALL': p['game.kick_offset_mm']}
    obj.glob.params = obj.params
    obj.glob.side_step_left_yield = obj.side_step_left_yield
    obj.glob.side_step_right_yield = obj.side_step_right_yield
    obj.local.ball_odometry = list(point)
    obj.body_euler_angle = {'yaw': 0.}
    obj.glob.robot_see_ball = 5
    obj.glob.ball_distance = math.hypot(*point)
    obj.glob.ball_course = math.atan2(point[1], point[0])
    method = original_method('verify_ball_position')
    method.__globals__.update(cos=math.cos, sin=math.sin)
    method(obj, point[1] < 0, .15)
    actual = [(s.step, s.side, legacy.clip(s.heading, .3)*(1 if s.right else -1),
               s.index, s.total, s.right, s.half) for s in legacy.verify(p, .15, lambda: point)]
    assert len(actual) == len(expected)
    for left, right in zip(actual, expected):
        assert left == pytest.approx(right)


@pytest.mark.parametrize('start_y', [-.001, .001])
def test_verify_retains_kicking_leg_despite_centre_line_noise(tmp_path, start_y):
    p = Parameters(tmp_path).values
    obj, expected = legacy_fixture(p)
    obj.params = {'KICK_ADJUSTMENT_DISTANCE_1': p['game.kick_approach_mm'],
                  'KICK_OFFSET_OF_BALL': p['game.kick_offset_mm']}
    obj.glob.params = obj.params
    obj.glob.side_step_left_yield, obj.glob.side_step_right_yield = obj.side_step_left_yield, obj.side_step_right_yield
    obj.local.ball_odometry = [.1, start_y]
    obj.body_euler_angle = {'yaw': 0.}
    obj.glob.robot_see_ball, obj.glob.ball_distance = 5, math.hypot(.1, start_y)
    cycle = obj.walk_Cycle
    def noisy_cycle(*args, **kwargs):
        cycle(*args, **kwargs)
        if not kwargs.get('half'):
            obj.local.ball_odometry[1] *= -1
    obj.walk_Cycle = noisy_cycle
    method = original_method('verify_ball_position')
    method.__globals__.update(cos=math.cos, sin=math.sin)
    method(obj, start_y < 0, 0.)
    points = iter([(.1, start_y), (.1, -start_y)] * 25)
    actual = list(legacy.verify(p, 0., lambda: next(points)))
    assert not any(s.half for s in actual)
    assert len(actual) == len(expected)
    for order, original in zip(actual, expected):
        assert (order.step, order.side, 0., order.index, order.total, order.right, order.half) == pytest.approx(original)


def test_verify_switches_kicking_leg_only_beyond_original_deadband(tmp_path):
    p = Parameters(tmp_path).values
    points = iter([(.1, 0.), (.1, .020), (.1, .021), (.1, -.020), (.1, -.021)])
    orders = list(legacy.verify(p, 0., lambda: next(points, None)))
    full = [order for order in orders if not order.half]
    assert [order.right for order in full] == [False, False, True, True, False]
    assert sum(order.half for order in orders) == 2


@pytest.mark.parametrize('direction', [-math.pi/2, math.pi/2])
def test_full_side_series_real_ik_and_confirmed_exit(tmp_path, body_clock, direction):
    pytest.importorskip('starkit')
    logs = []
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values}, lambda *a: None, lambda *a: logs.append(a))
    body.simulated = False
    body.command('control.acquire', {})
    body.pose = 'base_stand'
    ident = body.command('game.walk', dict(kind='omni', distance_m=.2, course=direction))['job_id']
    for _ in range(15000):
        body.tick()
        if not body.active:
            break
    job = body.command('job.status', {'job_id': ident})
    assert job['status'] == 'completed', job
    assert len(job['segments']) > 2 and job['pose'] == 'base_stand'
    assert not job['execution_uncertain']
    assert not any('IK frame skipped' in str(log) for log in logs)
    assert math.copysign(1, sum(s['dy_m'] for s in job['segments'])) == math.copysign(1, direction)


@pytest.mark.parametrize('hard', [False, True])
def test_original_series_cancellation_keeps_only_confirmed_cycles(tmp_path, body_clock, hard):
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values}, lambda *a: None, lambda *a: None)
    body.command('control.acquire', {})
    body.pose = 'base_stand'
    ident = body.command('game.walk', dict(kind='omni', distance_m=.5, course=math.pi/2))['job_id']
    for _ in range(1000):
        body.tick()
        if body.jobs[ident]['segments'] and body.segment_active:
            break
    assert len(body.jobs[ident]['segments']) == 1
    body.command('motion.stop_hard' if hard else 'motion.stop_graceful', {})
    for _ in range(1000):
        body.tick()
        if not body.active:
            break
    job = body.jobs[ident]
    assert job['status'] == 'cancelled'
    assert len(job['segments']) == (1 if hard else 2)
    assert job['pose'] == ('unknown' if hard else 'base_stand')
    assert job['execution_uncertain'] == hard
    assert body.command('game.observe', dict(job_id=ident)) == {'accepted': False}


def test_camera_height_tracks_original_leg_and_neck_geometry(tmp_path):
    from roki_ng.motion.model import Robot
    p, model = Parameters(tmp_path).values, Robot()
    nominal = camera_height(p, model, None, 0., 0., 0, p['game.head_tilt'])
    assert nominal == pytest.approx(p['game.camera_height_m'])
    engine = SimpleNamespace(xtr=0, xtl=0, ytr=-model.d10, ytl=model.d10, ztr=-180, ztl=-180)
    lowered = camera_height(p, model, engine, 0., 0., 0, p['game.head_tilt'])
    assert lowered == pytest.approx(nominal-(-model.ztr0-180)/1000)
    assert camera_height(p, model, None, 0., 0., 0, 0) > nominal


@pytest.mark.parametrize('options', [
    {'kind': 'kickoff', 'invert': 1}, {'kind': 'kickoff', 'invert': -1},
    {'kind': 'approach', 'distance_m': .8, 'x_m': .2, 'y_m': -.03},
    {'kind': 'around', 'distance_m': .3, 'course': .2, 'kick_heading': -.5, 'x_m': .1, 'y_m': .02},
    {'kind': 'verify', 'x_m': .07, 'y_m': -.06},
])
def test_forward_manoeuvres_finish_with_real_ik_and_simulated_transport(tmp_path, body_clock, options):
    pytest.importorskip('starkit')
    logs = []
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values}, lambda *a: None, lambda *a: logs.append(a))
    body.simulated = False
    body.command('control.acquire', {})
    body.pose = 'base_stand'
    ident = body.command('game.walk', options | {'stamp': body_clock[0]})['job_id']
    for _ in range(20000):
        if options['kind'] != 'kickoff':
            body.command('game.observe', dict(job_id=ident, x_m=options['x_m'], y_m=options['y_m'], stamp=body_clock[0]))
        body.tick()
        if not body.active:
            break
    job = body.jobs[ident]
    assert job['status'] == 'completed', job
    assert job['pose'] == 'base_stand' and not job['execution_uncertain']
    assert not any('IK frame skipped' in str(log) for log in logs), logs[:5]
