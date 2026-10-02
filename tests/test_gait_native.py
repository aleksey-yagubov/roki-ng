"""Offline trajectory regression; requires the real native starkit extension."""
import pytest

from roki_ng.motion.engine import Engine
from roki_ng.parameters import Parameters


# Representative trajectories and parameter boundaries, not a Cartesian sweep.
@pytest.mark.parametrize("step,side,yaw,right_first,height,tilt,skew,respect_body_tilt", [
    pytest.param(0, 0, 0, True, 40, 0, 0, True, id="spot"),
    pytest.param(12, 0, 0, False, 32, 0, 0, True, id="short-left-first"),
    pytest.param(24, 0, 0, True, 40, 0, 0, False, id="forward"),
    pytest.param(-24, 0, 0, False, 40, 0, 0, True, id="backward"),
    pytest.param(0, 12, 0, True, 32, 0, 0, True, id="side-left"),
    pytest.param(0, -12, 0, False, 40, 0, 0, False, id="side-right"),
    pytest.param(0, 0, .15, True, 40, 0, 0, True, id="turn-left"),
    pytest.param(0, 0, -.15, False, 32, 0, 0, True, id="turn-right"),
    pytest.param(24, 12, .15, False, 40, 0, 0, True, id="combined"),
    pytest.param(64, 0, 0, True, 40, 0, 0, True, id="long"),
    pytest.param(24, 0, 0, True, 40, .1, .05, True, id="forward-tilted"),
    pytest.param(24, 0, 0, False, 32, -.1, -.05, True, id="forward-countertilted"),
    pytest.param(-24, 0, 0, True, 32, .1, .05, True, id="backward-tilted"),
    pytest.param(-24, 0, 0, False, 40, -.1, -.05, True, id="backward-countertilted"),
    pytest.param(24, 12, .15, True, 40, .1, .05, False, id="finish-without-positive-tilt"),
    pytest.param(24, 12, .15, False, 32, -.1, -.05, False, id="finish-without-negative-tilt"),
])
def test_walk_finishes_all_frames(tmp_path, step, side, yaw, right_first, height, tilt, skew,
                                 respect_body_tilt):
    pytest.importorskip("starkit")
    params = Parameters(tmp_path).values
    params.update({"walk.step_height_mm": height, "walk.body_tilt_forward": tilt,
                   "walk.body_tilt_backward": tilt, "walk.sole_skew": skew})
    logs = []
    engine = Engine(params, log=lambda *a: logs.append(a))
    engine.first_Leg_Is_Right_Leg = right_first
    commands = list(engine.walk_Initial_Pose(start_mixing=False))
    for cycle in range(10):
        commands.extend(engine.walk_Cycle(step, side, yaw, cycle, 10))
    commands.extend(engine.walk_Cycle(0, 0, 0, 0, 1))
    assert engine.ztr == engine.ztl == -params["walk.gait_height_mm"]
    commands.extend(engine.walk_Final_Pose(respect_body_tilt=respect_body_tilt))
    frames = [cmd for cmd in commands if cmd[0] == "servo"]
    assert engine.exitFlag == 0, logs[:2]
    assert frames
    assert all(0 <= s.Data <= 16383 for cmd in frames for s in cmd[1])


@pytest.mark.parametrize("right", [True, False])
@pytest.mark.parametrize("power", [30, 80, 100])
def test_kick_on_fresh_engine(tmp_path, right, power):
    pytest.importorskip("starkit")
    params = Parameters(tmp_path).values
    logs = []
    engine = Engine(params, log=lambda *args: logs.append(args))
    engine.kick_power = power
    commands = list(engine.kick(right))
    frames = [cmd for cmd in commands if cmd[0] == "servo"]
    assert frames
    assert engine.exitFlag == 0, logs
    assert engine.gaitHeight == params["walk.gait_height_mm"]
    assert all(0 <= servo.Data <= 16383 for cmd in frames for servo in cmd[1])


def test_kick_does_not_depend_on_previous_stride(tmp_path):
    pytest.importorskip("starkit")
    params = Parameters(tmp_path).values
    fresh, stale = Engine(params), Engine(params)
    stale.stepLength = -64
    assert list(fresh.kick(True)) == list(stale.kick(True))
