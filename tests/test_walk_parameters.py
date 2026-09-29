import json

import pytest

from roki_ng.body import Body
from roki_ng.motion.engine import Engine
from roki_ng.parameters import Parameters, SCHEMA
from roki_ng.wire import Fault


def test_walk_defaults_and_no_old_aliases(tmp_path):
    (tmp_path / "parameters.json").write_text(json.dumps({"motion.step_height_mm": 12}))
    params = Parameters(tmp_path)
    assert params.values["walk.step_height_mm"] == 40
    assert "motion.step_height_mm" not in SCHEMA
    with pytest.raises(Fault):
        params.set("motion.step_height_mm", 20)
    engine = Engine(params.values)
    assert (engine.gaitHeight, engine.amplitude, engine.frames_per_cycle) == (180, 32, 2)
    assert params.values["walk.heading_kp"] == 1.1
    assert params.values["walk.heading_max_correction_rad"] == 0.3


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


def test_independent_kick_skew_and_descriptions(tmp_path):
    params = Parameters(tmp_path)
    params.set_many({"walk.sole_skew": 0.1, "kick.sole_skew": -0.1})
    engine = Engine(params.values)
    assert engine.params["SOLE_LANDING_SKEW"] == 0.1
    assert engine.params["KICK_SOLE_LANDING_SKEW"] == -0.1
    for key in SCHEMA:
        if key.startswith("walk."):
            assert params.describe(key)["apply"] == "next_job"
            assert len(params.describe(key)["description"]) > 60
    assert Parameters(tmp_path).values["kick.sole_skew"] == -0.1
