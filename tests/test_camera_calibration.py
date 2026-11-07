import shutil

import pytest

from roki_ng.vision.calibration import DEFAULT_DIRECTORY, FILES, calibration_directory


def test_default_is_loaded_without_creating_individual_profile(tmp_path):
    individual = tmp_path / 'localisation'
    assert calibration_directory(individual) == DEFAULT_DIRECTORY
    assert not individual.exists()
    from roki_ng.vision.ground_projection import GroundProjection
    projection = GroundProjection(DEFAULT_DIRECTORY, .43)
    assert projection.P.shape == (3, 3)


def test_individual_profile_has_priority(tmp_path):
    for name in FILES:
        shutil.copyfile(DEFAULT_DIRECTORY / name, tmp_path / name)
    assert calibration_directory(tmp_path) == tmp_path


def test_partial_individual_profile_is_not_mixed_with_default(tmp_path):
    shutil.copyfile(DEFAULT_DIRECTORY / 'profile.json', tmp_path / 'profile.json')
    with pytest.raises(ValueError, match='Incomplete camera calibration'):
        calibration_directory(tmp_path)


def test_incompatible_individual_profile_is_not_silently_replaced(tmp_path):
    for name in FILES:
        shutil.copyfile(DEFAULT_DIRECTORY / name, tmp_path / name)
    (tmp_path / 'profile.json').write_text('{"schema":1,"capture_size":[640,480]}')
    with pytest.raises(ValueError, match='Incompatible camera calibration'):
        calibration_directory(tmp_path)
