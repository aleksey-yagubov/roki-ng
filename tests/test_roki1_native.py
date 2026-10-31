"""Original Roki1 trajectory and reachable interpolated final stance."""
import pytest

from roki_ng.body.motion.backends import load
from roki_ng.body.motion.engine import Engine
from roki_ng.parameters import Parameters


def test_original_walk_keeps_lift_and_reports_skipped_ik(tmp_path):
    pytest.importorskip('starkit')
    p = Parameters(tmp_path, 'roki1').values
    logs = []
    e = Engine(p, backend=load('roki1'), log=lambda *args: logs.append(args))
    list(e.walk_Initial_Pose(start_mixing=False))
    heights = []
    for cycle in range(5):
        for command in e.walk_Cycle(24, 0, 0, cycle, 5):
            if command[0] == 'servo':
                heights.extend((e.ztr, e.ztl))
    assert max(heights) == -190 + 32
    assert e.stepHeight == 32
    assert e.exitFlag > 0
    assert any('IK frame skipped' in message for _, message in logs)



@pytest.mark.parametrize('step,side,yaw,right', [
    (24, 0, 0, True), (64, 0, 0, True), (64, 0, 0, False),
    (-24, 0, 0, False), (0, 20, 0, True), (24, 12, .15, False),
])
def test_final_stance_preserves_xy_and_completes_all_frames(tmp_path, step, side, yaw, right):
    pytest.importorskip('starkit')
    e = Engine(Parameters(tmp_path, 'roki1').values, backend=load('roki1'), log=lambda *a: None)
    e.first_Leg_Is_Right_Leg = right
    list(e.walk_Initial_Pose(start_mixing=False))
    for cycle in range(5):
        list(e.walk_Cycle(step, side, yaw, cycle, 5))
    list(e.walk_Cycle(0, 0, 0, 0, 1))
    xy = e.xtr, e.xtl
    failures = e.exitFlag
    frames = []
    for cmd in e.walk_Final_Pose():
        if cmd[0] == 'servo':
            frames.append(cmd)
            assert (e.xtr, e.xtl) == xy
    assert e.exitFlag == failures
    assert len(frames) == e.initPoses
    assert e.ztr == e.ztl == -230.
    hands = {(row[0], row[1]) for i, row in enumerate(e.ACTIVESERVOS) if i in e.hand_joints}
    assert all(s.Data == 7500 for s in frames[-1][1] if (s.Id, s.Sio) in hands)
    assert any(s.Data != 7500 for s in frames[-1][1] if s.Id in (7, 8, 9))


@pytest.mark.parametrize('right,power', [(True, 30), (False, 30), (True, 100), (False, 100)])
def test_kick_exit_reaches_last_frame(tmp_path, right, power):
    pytest.importorskip('starkit')
    e = Engine(Parameters(tmp_path, 'roki1').values, backend=load('roki1'), log=lambda *a: None)
    e.kick_power = power
    original = e.walk_Final_Pose_After_Kick
    checked = []
    def verify_exit():
        before = e.exitFlag
        xy = e.xtr, e.xtl
        yield from original()
        assert e.exitFlag == before
        assert (e.xtr, e.xtl) == xy
        assert e.ztr == e.ztl == -230.
        checked.append(True)
    e.walk_Final_Pose_After_Kick = verify_exit
    list(e.kick(right))
    assert checked == [True]
