"""Offline single-frame particle measurement update, never robot control.

Uses deduplicated line segments, finite map segments, an outlier mixture and
one measurement update only. Posterior spread is conditional on assumed
camera geometry and is not a calibrated physical accuracy estimate.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from roki_ng.field_observations import segment, field_model, observations, detect_circle, likelihood, update
from tools.bird_view_probe import bird_view, head_angles












def run(capture, calibration):
    metadata = json.loads((capture/'capture.json').read_text())
    if metadata['frame_sequence']-metadata['imu_sequence'] != metadata['unicam_minus_stm']:
        raise ValueError('Unmatched frame/IMU')
    image = cv2.imread(str(capture/'frame.png'))
    pitch, roll = head_angles(metadata['quaternion_xyzw'])
    _, roi, white, _ = segment(image)
    paint, _ = bird_view(cv2.cvtColor(white, cv2.COLOR_GRAY2BGR), calibration, pitch, roll, .4068)
    paint_mask = cv2.cvtColor(paint, cv2.COLOR_BGR2GRAY)
    circle = detect_circle(paint_mask)
    line_mask = paint_mask.copy()
    if circle:
        x, y, radius = circle['pixel_circle']
        yy, xx = np.indices(line_mask.shape)
        # Remove circle rim before straight-line detection to reject chords.
        line_mask[np.abs(np.hypot(xx-x, yy-y)-radius)<8] = 0
    lines = observations(line_mask)
    rng = np.random.default_rng(27)
    prior = np.array([-1.675, -1.175, np.arctan2(1.175, 1.675)])
    particles = rng.normal(prior, [.55, .55, np.deg2rad(25)], size=(60000, 3))
    # Keep apron positions possible; no artificial clipping to playing rectangle.
    weights, errors = update(particles, lines, field_model(), circle)
    best = int(np.argmax(weights)); pose = particles[best]
    quantiles = []
    for dim in range(3):
        order = np.argsort(particles[:, dim])
        quantiles.append(np.interp([.05, .5, .95], np.cumsum(weights[order]), particles[order, dim]).tolist())
    # Diagnostic wider position search uses a different prior, not a second
    # application of the same frame's likelihood to the first posterior.
    broad = rng.uniform([-2, -1.5, -np.pi/2], [2, 1.5, np.pi/2], size=(90000, 3))
    bw, be = update(broad, lines, field_model(), circle)
    bi = int(bw.argmax())
    ew, _ = update(particles, lines, field_model(extra=True), circle)
    report = dict(frame_sequence=metadata['frame_sequence'], particle_count=len(particles), circle=circle,
                  distinct_segments=len(lines), candidate_pose_m_rad=pose.tolist(),
                  distance_from_own_end_m=float(pose[0]+1.675),
                  distance_from_right_side_m=float(pose[1]+1.175),
                  heading_degrees=float(np.rad2deg(pose[2])),
                  effective_sample_size=float(1/(weights@weights)),
                  conditional_quantiles_05_50_95=quantiles,
                  segment_inlier_fraction_10cm=float(np.mean(errors[best]<.1)),
                  median_segment_error_m=float(np.median(errors[best])),
                  broad_prior_best_pose_m_rad=broad[bi].tolist(),
                  legacy_extra_lines_best_pose_m_rad=particles[ew.argmax()].tolist(),
                  measurement_updates=1, pose_valid=False,
                  status='single_frame_prior_conditioned_particle_candidate',
                  limits=['unknown height/extrinsics error', 'line identities and outliers unresolved',
                          'posterior quantiles are not physical accuracy', 'not temporal tracking'])
    canvas = np.full((780, 660, 3), (34, 55, 35), np.uint8)
    def pixel(p):
        return (int(330-p[1]*170), int(390-p[0]*170))
    for line in field_model():
        cv2.line(canvas, pixel(line[0]), pixel(line[1]), (240, 240, 240), 2)
    if circle:
        # Dashed/reference radius is observed, not a confirmed map dimension.
        cv2.circle(canvas, pixel([0, 0]), round(circle['radius_observed_m']*170), (190, 190, 190), 1)
    # Weighted samples illustrate conditional posterior (resampling for display only).
    for p in particles[rng.choice(len(particles), 1800, p=weights)]:
        cv2.circle(canvas, pixel(p), 1, (100, 140, 230), -1)
    for line in lines:
        c, s = np.cos(pose[2]), np.sin(pose[2])
        world = line@np.array([[c, s], [-s, c]])+pose[:2]
        cv2.line(canvas, pixel(world[0]), pixel(world[1]), (230, 80, 220), 2)
    if circle:
        center = np.array(circle['center_robot_m'])@np.array([[c, s], [-s, c]])+pose[:2]
        cv2.circle(canvas, pixel(center), round(circle['radius_observed_m']*170), (255, 220, 0), 2)
    cv2.circle(canvas, pixel(pose), 7, (0, 230, 255), -1)
    end = pose[:2]+.35*np.array([np.cos(pose[2]), np.sin(pose[2])])
    cv2.arrowedLine(canvas, pixel(pose), pixel(end), (0, 230, 255), 3)
    for i, text in enumerate(['SINGLE FRAME - UNVALIDATED', 'White: map | Magenta: observations',
                              'Orange: particles | Yellow: candidate']):
        cv2.putText(canvas, text, (15, 25+i*24), cv2.FONT_HERSHEY_SIMPLEX, .51, (240,240,240), 1)
    cv2.imwrite(str(capture/'particle-pose.png'), canvas)
    np.savez_compressed(capture/'particle-pose.npz', particles=particles, weights=weights, observations=lines)
    (capture/'particle-pose.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('calibration', type=Path)
    args = parser.parse_args()
    run(args.capture, args.calibration)
