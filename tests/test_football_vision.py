import math

import cv2
import numpy as np
import pytest

from roki_ng.football_vision import goal_parts, assemble_goals, render_scene
from roki_ng.parameters import Parameters


class Projector:
    def _pixel_ray(self, pixel, quaternion):
        u, v = pixel
        return np.array([1., (400-u)/700, (216.6667-v)/700])


def picture(tmp_path, left=True, right=True, bar=True, cropped=False):
    p = Parameters(tmp_path).values
    for i in (0, 1):
        p[f'field.goal.{i}']['width'] = .7
    colours = {'blue_posts': (255, 0, 0), 'green_field': (0, 150, 0),
               'white_marking': (255, 255, 255), 'white_posts': (255, 255, 255),
               'yellow_posts': (0, 255, 255)}
    for name, bgr in colours.items():
        l, a, b = map(float, cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2LAB)[0, 0])
        for key, value in dict(l_min=max(0, l/2.55-2), l_max=min(100, l/2.55+2),
                               a_min=a-128-2, a_max=a-128+2, b_min=b-128-2, b_max=b-128+2).items():
            p[f'vision.{name}.{key}'] = value
    image = np.zeros((650, 800, 3), np.uint8)
    image[450:] = colours['green_field']
    if left:
        cv2.rectangle(image, (200, 100), (220, 449), colours['blue_posts'], -1)
    if right:
        cv2.rectangle(image, (600, 100), (620, 449), colours['blue_posts'], -1)
    if bar:
        cv2.rectangle(image, (0 if cropped else 200, 100), (620, 112), colours['blue_posts'], -1)
    return image, p


@pytest.mark.parametrize('left,right,bar', [(True, True, False), (True, False, True), (False, True, True)])
def test_goal_detector_accepts_actual_two_element_images(tmp_path, left, right, bar):
    image, p = picture(tmp_path, left, right, bar)
    parts = goal_parts(image, p, Projector(), (), dict(height=.4, x=0., y=0., pan=0.))
    goals = assemble_goals(parts, [p['field.goal.1']])
    assert len(goals) == 1
    assert goals[0]['elements'] == ('posts' if not bar else 'post+bar')


def test_cropped_bar_cannot_replace_missing_post(tmp_path):
    image, p = picture(tmp_path, left=False, right=True, bar=True, cropped=True)
    parts = goal_parts(image, p, Projector(), (), dict(height=.4, x=0., y=0., pan=0.))
    assert not assemble_goals(parts, [p['field.goal.1']])


def test_diagnostic_does_not_modify_raw_frame(tmp_path):
    image, _ = picture(tmp_path)
    before = image.copy()
    out = render_scene(image, None, 'kick forbidden: no confirmed goal')
    assert np.array_equal(before, image)
    assert out.shape == image.shape and not np.array_equal(out, image)
