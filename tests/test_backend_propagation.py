"""Explicit Roki1 selection must win when both backends are installed."""
import pytest

from roki_ng.body.calibration import TestPlan as Plan
from roki_ng.body.motion.backends import available, load
from roki_ng.body.motion.engine import Engine
from roki_ng.body.runtime import Body
from roki_ng.detection.ball import BallObservation
from roki_ng.localisation.worker import Localisation
from roki_ng.supervisor.supervisor import Supervisor


@pytest.mark.parametrize('name', ['roki1', 'roki2'])
def test_selected_model_reaches_workers_and_calibration(tmp_path, name):
    assert set(available()) == {'roki1', 'roki2'}
    supervisor = Supervisor({'state_dir': str(tmp_path), 'simulate': True, 'robot_model': name})
    config = supervisor.config | {'parameters': dict(supervisor.params.values)}
    logs = []
    body = Body(config, lambda *args: None, lambda *args: logs.append(args))
    plan = Plan(body, {'name': 'run_test', 'mode': 'short'})
    calibration = plan.get_engine()
    manual = body._engine()
    assert calibration.backend is body.backend is manual.backend
    assert calibration.model == manual.model == load(name).Robot().model
    assert calibration.SIZES == body.model.SIZES
    assert calibration.mapping.split_knee == (name == 'roki2')
    assert BallObservation(config).camera_model == body.model.camera
    assert Localisation(config, lambda *a: None, lambda *a: None).camera_model == body.model.camera
    assert any(name in message for _, message in logs)


def test_short_run_preparation_uses_single_full_angle_knee(tmp_path):
    pytest.importorskip('starkit')
    s = Supervisor({'state_dir': str(tmp_path), 'simulate': True, 'robot_model': 'roki1'})
    body = Body(s.config | {'parameters': dict(s.params.values)}, lambda *a: None, lambda *a: None)
    body.simulated = False  # Real IK, never send commands to hardware.
    plan = Plan(body, {'name': 'run_test', 'mode': 'short'})
    frames = []
    generator = plan.gait(11, step=64, ramp=True)
    for command in generator:
        if command[0] == 'drain':
            break  # Only preparation, before the first gait cycle/IMU wait.
        if command[0] == 'servo':
            frames.append(command)
    generator.close()
    expected = Engine(body.parameters, backend=body.backend)
    targets = [c for c in expected.walk_Initial_Pose(start_mixing=False) if c[0] == 'servo']
    assert frames == targets
    assert len(frames) == expected.initPoses
    assert all(s.Id != 13 for frame in frames for s in frame[1])
    for bus in (1, 2):
        assert sum(s.Id == 8 and s.Sio == bus for s in frames[-1][1]) == 1


def test_engine_cannot_silently_select_another_model():
    with pytest.raises(TypeError, match='backend'):
        Engine({})
