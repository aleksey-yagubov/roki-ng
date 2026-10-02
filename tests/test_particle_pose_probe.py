import numpy as np
import pytest
import cv2

pytest.importorskip("scipy", reason="Offline localisation prototype requires SciPy")

from tools.particle_pose_probe import field_model, update, detect_circle


def local_lines(pose):
    c, s = np.cos(pose[2]), np.sin(pose[2])
    return (field_model()-pose[:2])@np.array([[c, -s], [s, c]])


def test_recovers_known_pose_with_unrelated_outlier():
    true = np.array([-1.2, -.85, .62])
    rng = np.random.default_rng(31)
    particles = np.vstack([true, rng.normal(true, [.35, .35, .3], (5000, 3))])
    lines = local_lines(true)+rng.normal(0, .005, (5, 2, 2))
    lines = np.concatenate([lines, [[[5., 5.], [6., 6.]]]])
    weights, _ = update(particles, lines, field_model())
    estimate = np.average(particles, axis=0, weights=weights)
    assert np.linalg.norm(estimate[:2]-true[:2]) < .08
    assert abs(estimate[2]-true[2]) < .06
    assert weights.argmax() == 0
    assert weights.sum() == pytest.approx(1)


def test_rejects_wrong_line_orientation_despite_matching_midpoint():
    model = np.array([[[-1., 0.], [1., 0.]]])
    lines = np.array([[[-.1, 0.], [.1, 0.]]])
    weights, _ = update(np.array([[0, 0, 0], [0, 0, np.pi/2]]), lines, model)
    assert weights[0] > weights[1]*1.3


def test_empty_observations_do_not_invent_information():
    weights, errors = update(np.zeros((10, 3)), [], field_model())
    assert np.allclose(weights, .1)
    assert errors.shape == (10, 0)


def test_extreme_outliers_do_not_zero_all_weights():
    weights, _ = update(np.array([[0, 0, 0], [1, 1, .2]]),
                        [[[1e6, 1e6], [1e6+1, 1e6+1]]], field_model())
    assert np.isfinite(weights).all()
    assert np.allclose(weights, .5)


def test_invalid_input_rejected():
    with pytest.raises(ValueError, match='Nonfinite'):
        update(np.array([[np.nan, 0, 0]]), [], field_model())
    with pytest.raises(ValueError, match='Degenerate observation'):
        update(np.array([[0, 0, 0]]), [[[0, 0], [0, 0]]], field_model())


def test_circle_and_halfway_line_found_without_manual_pixel_input():
    mask = np.zeros((720, 720), np.uint8)
    cv2.circle(mask, (370, 410), 50, 255, 7)
    cv2.line(mask, (100, 410), (650, 410), 255, 5)
    found = detect_circle(mask)
    assert found is not None
    assert np.linalg.norm(np.array(found['pixel_circle'][:2])-[370, 410])<5
    assert found['angular_coverage'] >= .85
    assert detect_circle(np.zeros_like(mask)) is None


def test_circle_breaks_parallel_line_translation_ambiguity():
    pose = np.array([-1.2, -.85, .62])
    c, s = np.cos(pose[2]), np.sin(pose[2])
    center = -pose[:2]@np.array([[c, -s], [s, c]])
    particles = np.array([pose, pose+[.6, 0, 0]])
    weights, _ = update(particles, local_lines(pose)[[3]], field_model(),
                        {'center_robot_m': center})
    assert weights[0]>.95

@pytest.mark.parametrize('flat', [False, True])
def test_hough_layout_preserves_segment_endpoints(monkeypatch, flat):
    from tools.particle_pose_probe import observations
    segments = np.array([[100, 100, 100, 600], [200, 100, 650, 100],
                         [600, 200, 600, 650], [100, 650, 500, 650]], np.int32)
    monkeypatch.setattr(cv2, 'HoughLinesP', lambda *a, **kw: segments if flat else segments[:, None, :])
    result = observations(np.zeros((720, 720), np.uint8))
    expected = segments.reshape(-1, 2, 2)
    expected = np.stack((4-expected[..., 1]/180, 2-expected[..., 0]/180), axis=-1)
    assert len(result) == 4
    for line in expected:
        assert any(np.allclose(line, observed) for observed in result)
