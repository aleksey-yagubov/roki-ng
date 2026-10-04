import math

import pytest

from roki_ng.odometry import GameOdometry, jump_delta, walk_delta
from roki_ng.parameters import Parameters


@pytest.mark.parametrize('entry,x,y,heading', [
    ('center', -.45, 0, 0),
    ('left', -1.675 + .87, 1.175, -math.pi / 2),
    ('right', -1.675 + .87, -1.175, math.pi / 2),
    ('goalkeeper', -1.675 + .18, 0, 0),
])
def test_entries_bind_arbitrary_imu_yaw_to_field(tmp_path, entry, x, y, heading):
    parameters = Parameters(tmp_path).values
    odometry = GameOdometry(parameters, entry, 2.6)
    assert (odometry.x, odometry.y, odometry.yaw) == pytest.approx((x, y, heading))
    odometry.heading(2.7)
    assert odometry.yaw == pytest.approx(heading + .1)
    parameters['match.own_goal'] = 1
    opposite = GameOdometry(parameters, entry, -1.9)
    assert (opposite.x, opposite.y) == pytest.approx((-x, -y))
    assert math.cos(opposite.yaw - heading) == pytest.approx(-1)


def test_yields_are_physical_calibrations_not_command_amplitudes(tmp_path):
    p = Parameters(tmp_path).values | {
        'motion.run_10_mm': 500, 'motion.run_20_mm': 1040,
        'motion.side_left_20_mm': 800, 'motion.side_right_20_mm': 1000,
        'motion.jump_forward_mm': 70, 'motion.jump_right_mm': 90,
    }
    assert walk_delta(p, step_mm=64, first=True)[0] == pytest.approx(.014)
    assert walk_delta(p, step_mm=32)[0] == pytest.approx(.027)
    assert walk_delta(p, side_mm=10, right_first=False)[1] == pytest.approx(.02)
    assert walk_delta(p, side_mm=10, right_first=True)[1] == pytest.approx(-.025)
    assert jump_delta(p, 'forward', .5) == pytest.approx((.035, 0))
    assert jump_delta(p, 'right', .5) == pytest.approx((0, -.045))


def test_partial_cancelled_job_counts_confirmed_segments_once(tmp_path):
    odometry = GameOdometry(Parameters(tmp_path).values, 'left', 1.3)
    x, y = odometry.x, odometry.y
    job = {'status': 'running', 'segments': [dict(index=1, dx_m=.06, dy_m=0., imu_yaw_rad=1.3)]}
    odometry.apply('job', job)
    job.update(status='cancelled', execution_uncertain=True, reason='link_lost')
    odometry.apply('job', job)
    assert (odometry.x, odometry.y) == pytest.approx((x, y - .06))
    assert odometry.path == pytest.approx(.06)
    assert odometry.state()['confidence'] == 'degraded'
    assert odometry.state()['reason'] == 'link_lost'


def test_worker_counts_only_drained_segments_and_retains_partial_result(tmp_path, body_clock):
    from roki_ng.body import Body
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values}, lambda *a: None, lambda *a: None)
    body.command('control.acquire', {})
    body.pose = 'base_stand'
    drained = [False]
    body.hardware.drained = lambda: drained[0]
    ident = body.command('game.walk', {'kind': 'omni', 'distance_m': .2, 'course': math.pi/2})['job_id']
    for _ in range(35):
        body.tick()
    assert body.jobs[ident]['segments'] == []  # Commands accepted, completion still unknown.
    drained[0] = True
    for _ in range(100):
        body.tick()
        if body.jobs[ident]['segments'] and body.segment_active:
            break
    assert len(body.jobs[ident]['segments']) == 1
    body.command('motion.stop_hard', {})
    job = body.command('job.status', {'job_id': ident})
    assert job['status'] == 'cancelled' and job['execution_uncertain']
    assert len(job['segments']) == 1
    from roki_ng.legacy_game_motion import omni
    order = next(omni(body.parameters, 200, math.pi/2, 0.))
    expected = walk_delta(body.parameters, side_mm=order.side, right_first=False)[1]
    assert job['segments'][0]['dy_m'] == pytest.approx(expected)
