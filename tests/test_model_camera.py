import pytest

from roki_ng.parameters import Parameters
from roki_ng.body.motion.backends import load
from roki_ng.body.motion.game_geometry import camera_height
from roki_ng.detection.football import camera_geometry
from roki_ng.vision.camera_geometry import height
from roki_ng.wire import Fault


def test_shared_defaults_and_model_camera(tmp_path):
    first = Parameters(tmp_path/'one', 'roki1')
    second = Parameters(tmp_path/'two', 'roki2')
    for key in first.values:
        if key not in first.backend.MOTION_DEFAULTS:
            assert first.values[key] == second.values[key]
    assert first.values['camera.custom_height_m'] == 0.
    assert load('roki1').Robot.camera != load('roki2').Robot.camera
    for key in ('game.camera_height_m', 'localisation.camera_height_m',
                'game.neck_height_m', 'game.camera_reference_tilt', 'game.camera_forward_m'):
        with pytest.raises(Fault):
            first.describe(key)


@pytest.mark.parametrize('name', ['roki1', 'roki2'])
def test_override_is_absolute_and_reset_restores_model(tmp_path, name):
    params = Parameters(tmp_path, name)
    robot = load(name).Robot()
    params.set('camera.custom_height_m', .65)
    assert Parameters(tmp_path, name).values['camera.custom_height_m'] == .65
    assert height(params.values, robot.camera) == .65
    for tilt in (-1500, -500):
        assert camera_geometry(params.values, 0, tilt, robot.camera)['height'] == .65
        assert camera_height(params.values, robot, None, 10, 5, 0, tilt) == .65
    params.set('camera.custom_height_m', 0.)
    assert height(params.values, robot.camera) == robot.camera.height_m
    assert camera_geometry(params.values, 0, robot.camera.reference_tilt, robot.camera)['height'] == robot.camera.height_m


@pytest.mark.parametrize('value', [None, .01, -1, 4, True, float('nan')])
def test_invalid_override_rejected(tmp_path, value):
    with pytest.raises(Fault):
        Parameters(tmp_path).set('camera.custom_height_m', value)
