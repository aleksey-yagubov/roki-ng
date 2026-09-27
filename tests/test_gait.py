import importlib
import sys
from types import SimpleNamespace

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
