import asyncio
import math
import time

import pytest

from roki_ng.football_vision import assemble_goals, own_goal_blocks, own_colour_ahead, shot_clear, render_scene
from test_approach_progress import simple


def test_one_leg_change_keeps_jump_budget(tmp_path):
    s, _ = simple(tmp_path)
    s.leg_sign, s.jumps = -1, 7
    s.consider_leg_switch(80, 10)
    assert s.leg_sign == -1
    s.consider_leg_switch(75, 11)
    assert s.leg_sign == 1 and s.jumps == 7 and s.leg_switched
    s.consider_leg_switch(-80, 12)
    s.consider_leg_switch(-80, 13)
    assert s.leg_sign == 1 and s.jumps == 7


def test_same_frame_and_centre_noise_do_not_switch(tmp_path):
    s, _ = simple(tmp_path)
    s.leg_sign = -1
    s.consider_leg_switch(80, 1)
    s.consider_leg_switch(80, 1)
    assert not s.leg_switched
    s.consider_leg_switch(10, 2)
    s.consider_leg_switch(80, 3)
    assert not s.leg_switched
    s.consider_leg_switch(81, 4)
    assert s.leg_switched


def test_new_attempt_and_pause_reset_switch(tmp_path):
    s, _ = simple(tmp_path)
    s.leg_switched = True
    s.cancel_special('pause')
    assert not s.leg_switched and s.leg_candidate is None


def test_simple_changes_leg_after_two_real_observations(tmp_path):
    s, _ = simple(tmp_path, (.2, .08), [])
    s.near, s.leg_sign, s.jumps = True, -1, 5
    s.last_scene['ball_sequence'] = 100
    asyncio.run(s.step())
    assert s.leg_sign == -1
    s.last_scene = {'ball_sequence': 101, 'received': time.monotonic()}
    asyncio.run(s.step())
    assert s.leg_sign == 1 and s.jumps == 7


def parts():
    source = dict(colour='yellow', rect=[100,100,140,210], area=600,
                  frame_sequence=123, sensor_timestamp_ns=987, pan=-1778, tilt=-1000, parameter_revision=4)
    return [source | dict(kind='colour', points=[[2.,0.]], bearings=[-.1,.1]),
            source | dict(kind='opening', points=[[1.,-.5],[1.,.5]])]


def test_veto_reasons_retain_exact_source_frame_and_head():
    goal = assemble_goals(parts(), [{'colour':'yellow'}])[0]
    blocks = own_goal_blocks((.1, .05), goal, .045)
    assert {b['reason'] for b in blocks} == {'colour_ahead','shot_intersects_opening'}
    assert all(b['frame_sequence'] == 123 and b['pan'] == -1778 and b['area'] == 600 for b in blocks)
    assert blocks[0]['rect'] == [100,100,140,210]


@pytest.mark.parametrize('angle', [-3., -1., -.1, 0., .1, 1., 3.])
def test_diagnostics_do_not_weaken_existing_veto(angle):
    goal = dict(colour='yellow', bearings=[[angle-.05,angle+.05]], shootable=True,
                ends=[[math.cos(angle)-.1, math.sin(angle)-.4],
                      [math.cos(angle)+.1, math.sin(angle)+.4]])
    ball = (.1, .05)
    assert bool(own_goal_blocks(ball, goal, .045)) == bool(own_colour_ahead(goal) or shot_clear(ball, goal, -.045))


def test_debug_highlight_does_not_modify_source_image():
    np = pytest.importorskip('numpy')
    pytest.importorskip('cv2')
    image = np.zeros((650,800,3), np.uint8)
    scene = dict(parts=parts(), sequence=123, height_m=.43, ball=(.1,.05), ball_radius=.045)
    out = render_scene(image, scene, '', [{'colour':'yellow'}], 'yellow')
    assert not image.any()
    assert tuple(out[100,100]) == (0,0,255)


def test_real_veto_sends_sources_and_never_kicks(tmp_path):
    own = assemble_goals(parts(), [{'colour':'yellow'}])
    s, g = simple(tmp_path, (.099,-.052), own)
    s.own, s.enemy = 'yellow','blue'
    events = []
    g.s.log = lambda *args: events.append(args[-1])
    asyncio.run(s.step())
    assert all(call.args[0] != 'motion.kick' for call in g._move.await_args_list)
    call = next(c for c in g._call.await_args_list if c.args[1] == 'football.decision')
    assert call.args[2]['blocks'][0]['frame_sequence'] == 123
    assert any('Own goal veto:' in e and 'colour_ahead' in e and '123' in e for e in events)
