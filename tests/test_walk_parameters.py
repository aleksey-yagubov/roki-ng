import pytest

from roki_ng.body import Body
from roki_ng.motion.engine import Engine
from roki_ng.parameters import Parameters


@pytest.mark.parametrize("key,value", [("walk.gait_height_mm", 175),
                                       ("walk.sway_amplitude_mm", 24)])
def test_geometry_update_requires_new_preparation(tmp_path, key, value):
    params = Parameters(tmp_path)
    body = Body({"simulate": True, "parameters": dict(params.values)},
                lambda *a: None, lambda *a: None)
    body.engine = Engine(body.parameters)
    body.pose = "crouch"
    body.command("params.apply", {key: value})
    assert body.pose == "unknown" and body.engine is None
    assert body.hardware.sent == 0
    engine = body._engine()
    assert getattr(engine, "gaitHeight" if "height" in key else "amplitude") == value


def test_walk_and_kick_skew_are_independent_and_persistent(tmp_path):
    params = Parameters(tmp_path)
    params.set_many({"walk.sole_skew": 0.1, "kick.sole_skew": -0.1})
    engine = Engine(params.values)
    assert engine.params["SOLE_LANDING_SKEW"] == 0.1
    assert engine.params["KICK_SOLE_LANDING_SKEW"] == -0.1
    assert Parameters(tmp_path).values["kick.sole_skew"] == -0.1
