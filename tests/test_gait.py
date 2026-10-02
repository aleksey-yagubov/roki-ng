import sys
from types import SimpleNamespace

import pytest

from roki_ng.parameters import Parameters


def test_step_height_parameter(tmp_path):
    from roki_ng.body import Body
    from roki_ng.wire import Fault

    params = Parameters(tmp_path)
    key = "walk.step_height_mm"
    assert params.describe(key)["apply"] == "next_job"
    assert params.values[key] == 40
    body = Body({"simulate": True, "parameters": dict(params.values)},
                lambda *a: None, lambda *a: None)
    engine = body._engine()
    assert engine.stepHeight == 40
    body.command("params.apply", {key: 32})
    assert engine.stepHeight == 32
    body.active = "running-test"
    with pytest.raises(Fault, match="Stop motion"):
        body.command("params.apply", {key: 20})
    assert engine.stepHeight == body.parameters[key] == 32
    params.set(key, 32)
    assert Parameters(tmp_path).values[key] == 32
    for value in (-1, 61, float("nan")):
        with pytest.raises(Fault):
            params.set(key, value)


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
