import importlib
import sys
from types import SimpleNamespace

import pytest

from roki_ng.parameters import Parameters


def test_gait_yields_commands_without_legacy_imports(monkeypatch, tmp_path):
    # Fake only the native IK solver: exercise real geometry/state/control-flow.
    monkeypatch.setitem(sys.modules, "starkit", SimpleNamespace(alpha_calculation=lambda *a: [[0.0] * 6]))
    engine_module = importlib.import_module("roki_ng.motion.engine")
    engine = engine_module.Engine(Parameters(tmp_path).values)
    for generator in (engine.walk_Initial_Pose(start_mixing=False),
                      engine.walk_Cycle(12, 0, 0, 0, 2),
                      engine.walk_Cycle(0, 0, 0, 0, 1), engine.walk_Final_Pose(),
                      engine.kick(True, kick_offset=0)):
        steps = list(generator)
        assert steps
        servos = [step for step in steps if step[0] == "servo"]
        assert servos
        for _, values, frames, pause in servos:
            assert isinstance(frames, int)
            assert all(0 <= value.Data <= 16383 for value in values)
    assert not any(name.startswith("Soccer.") for name in sys.modules)


def test_terminal_cycle_lowers_both_feet(monkeypatch, tmp_path):
    monkeypatch.setitem(sys.modules, "starkit", SimpleNamespace(alpha_calculation=lambda *a: [[0.0] * 6]))
    from roki_ng.motion.engine import Engine
    engine = Engine(Parameters(tmp_path).values)
    list(engine.walk_Initial_Pose(start_mixing=False))
    list(engine.walk_Cycle(12, 0, 0, 0, 1000000))
    assert engine.ztl > -engine.gaitHeight
    list(engine.walk_Cycle(0, 0, 0, 0, 1))
    assert engine.ztl == engine.ztr == -engine.gaitHeight


@pytest.mark.parametrize("failed_legs", [("right",), ("left",), ("right", "left")])
def test_bad_ik_skips_frame_and_completes_cycle(monkeypatch, tmp_path, failed_legs):
    from roki_ng.body import Body

    native = SimpleNamespace(alpha_calculation=lambda *a: [[0.0] * 6])
    monkeypatch.setitem(sys.modules, "starkit", native)
    logs = []
    body = Body({"simulate": True, "parameters": Parameters(tmp_path).values},
                lambda *args: None, lambda *args: logs.append(args))
    body.command("control.acquire", {})
    engine = body._engine()
    list(engine.walk_Initial_Pose(start_mixing=False))
    body.hardware.drained = lambda: True
    calls = 0

    def solve(*args):
        nonlocal calls
        frame, leg = calls // 2, ("right", "left")[calls % 2]
        calls += 1
        return [] if frame == 2 and leg in failed_legs else [[0.0] * 6]

    native.alpha_calculation = solve
    job = body._start("test.start", engine.walk_Cycle(12, 0, 0, 0, 1))
    sent_at_skip = None
    for _ in range(1000):
        if not body.active:
            break
        body.next_at = 0
        body.tick()
        if sent_at_skip is None and engine.exitFlag:
            sent_at_skip = body.hardware.sent
    assert body.jobs[job["job_id"]]["status"] == "completed"
    assert body.error is None
    assert body.hardware.resets == 0
    assert engine.exitFlag == 1
    assert calls > 6
    assert body.hardware.sent > sent_at_skip
    assert engine.ztr == engine.ztl == -engine.gaitHeight
    warnings = [message for level, message in logs if level == "WARNING"]
    assert len(warnings) == len(failed_legs)
    for leg in failed_legs:
        assert any(f"{leg} leg has no valid solution" in msg
                   and "position_mm=" in msg and "limits=" in msg for msg in warnings)
    assert not any(level == "ERROR" for level, _ in logs)
