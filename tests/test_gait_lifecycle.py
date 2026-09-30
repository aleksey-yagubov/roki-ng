import pytest

from roki_ng.body import Body
from roki_ng.parameters import Parameters


@pytest.fixture
def body(tmp_path, monkeypatch):
    from roki_ng.motion import engine

    class FakeEngine:
        def __init__(self, *args, **kwargs):
            self.initial = 0
            self.cycles = []

        def walk_Initial_Pose(self, **kwargs):
            self.initial += 1
            yield "drain",

        def walk_Cycle(self, *args):
            self.cycles.append(args)
            yield "drain",

        def walk_Final_Pose(self):
            yield "drain",

        def kick(self, *args, **kwargs):
            yield "drain",

    monkeypatch.setattr(engine, "Engine", FakeEngine)
    instance = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                    lambda *args: None, lambda *args: None)
    instance.command("control.acquire", {})
    # Execute gait branches, but keep all hardware simulated.
    instance.simulated = False
    return instance


def complete(body):
    list(body.plan)
    body._finish("completed")


def walk(body, hold=True):
    body._start("motion.drive", body._walk(cycles=2, fixed={
        "x": 1, "y": 0, "yaw": 0, "speed": 0.5, "crouch": "on" if hold else "off"}))
    complete(body)


def test_hold_crouch_preserves_engine_and_terminal_cycle(body):
    walk(body)
    engine = body.engine
    assert engine.initial == 1
    assert engine.cycles[-1] == (0, 0, 0, 0, 1)
    assert body.pose == "crouch"
    body.command("motion.drive", {"x": 1, "crouch": "on"})
    body.command("motion.drive", {"x": 0, "crouch": "on"})
    complete(body)
    walk(body)
    assert body.engine is engine
    assert engine.initial == 1


@pytest.mark.parametrize("pose", ["unknown", "stand", "base_stand"])
def test_walk_prepares_from_other_poses_on_new_command(body, pose):
    body.pose = pose
    body.command("motion.drive", {"x": 0, "crouch": "on"})
    assert body.active is None
    assert body.pose == pose
    body.command("motion.drive", {"x": 1, "crouch": "on"})
    # Stop before the first cycle, but execute preparation and drain.
    body.command("motion.drive", {"x": 0, "crouch": "on"})
    complete(body)
    assert body.pose == "crouch"
    assert body.engine.initial == 1


@pytest.mark.parametrize("operation,args", [
    ("motion.pose", {"name": "base_stand"}),
    ("motion.pose", {"name": "stand"}),
    ("motion.jump", {"direction": "turn_left"}),
    ("motion.kick", {}),
    ("motion.slot", {"name": "Initial_Pose"}),
])
def test_other_leg_motion_discards_engine(body, operation, args):
    walk(body)
    old = body.engine
    body.command(operation, args)
    complete(body)
    assert body.engine is None
    walk(body)
    assert body.engine is not old
    assert body.engine.initial == 1


def test_head_does_not_reset_gait(body):
    walk(body)
    old = body.engine
    body.command("motion.head", {"pan": 100})
    body.command("motion.pose", {"name": "head_field"})
    complete(body)
    assert body.engine is old
    assert body.pose == "crouch"


def test_stand_after_walk_discards_engine(body):
    walk(body, hold=False)
    assert body.pose == "stand"
    assert body.engine is None


def test_hard_stop_discards_engine_and_does_not_resume(body):
    walk(body)
    body.command("motion.drive", {"x": 1, "crouch": "on"})
    body.command("motion.stop_hard", {})
    assert body.engine is None
    assert body.pose == "unknown"
    assert body.drive is body.active is None
    body.command("motion.drive", {"x": 1, "crouch": "on"})
    body.command("motion.drive", {"x": 0, "crouch": "on"})
    complete(body)
    assert body.engine.initial == 1
