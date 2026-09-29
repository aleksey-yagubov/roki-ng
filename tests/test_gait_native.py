"""Offline trajectory regression; requires the real native starkit extension."""
import pytest

from roki_ng.motion.engine import Engine
from roki_ng.parameters import Parameters


@pytest.mark.parametrize("step,side,yaw", [
    (0, 0, 0), (12, 0, 0), (24, 0, 0), (-24, 0, 0),
    (0, 12, 0), (0, 0, .15), (0, 0, -.15), (24, 12, .15), (64, 0, 0),
])
@pytest.mark.parametrize("right_first", [True, False])
@pytest.mark.parametrize("height", [32, 40])
@pytest.mark.parametrize("tilt,skew", [(0, 0), (.1, .05), (-.1, -.05)])
@pytest.mark.parametrize("respect_body_tilt", [True, False])
def test_walk_finishes_all_frames(tmp_path, step, side, yaw, right_first, height, tilt, skew,
                                 respect_body_tilt):
    native = pytest.importorskip("starkit")
    assert getattr(native, "__file__", None), "This test must not use a fake IK solver"
    params = Parameters(tmp_path).values
    params.update({"walk.step_height_mm": height, "walk.body_tilt_forward": tilt,
                   "walk.body_tilt_backward": tilt, "walk.sole_skew": skew})
    logs = []
    engine = Engine(params, log=lambda *a: logs.append(a))
    engine.first_Leg_Is_Right_Leg = right_first
    list(engine.walk_Initial_Pose(start_mixing=False))
    for cycle in range(10):
        list(engine.walk_Cycle(step, side, yaw, cycle, 10))
    list(engine.walk_Cycle(0, 0, 0, 0, 1))
    commands = list(engine.walk_Final_Pose(respect_body_tilt=respect_body_tilt))
    frames = [cmd for cmd in commands if cmd[0] == "servo"]
    assert engine.exitFlag == 0, logs[:2]
    assert not logs
    assert len(frames) == engine.initPoses
    assert engine.ztr == engine.ztl == -215
    assert all(0 <= s.Data <= 16383 for cmd in frames for s in cmd[1])
    elbows = [s.Data for s in frames[-1][1] if s.Id == 4]
    assert elbows == [7500, 7500]
