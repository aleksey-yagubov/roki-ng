"""Independent field detectors: missing paint must not hide visible goals."""
import time
import numpy as np


def process_frame(image, quaternion, projector, engine, parameters, sequence, timestamp):
    from . import field_observations as field, goal_observations as goal
    from . import field_marks
    from .field_config import disk_model,short_line_lengths
    timings, errors = {}, []
    def stage(name, operation, default):
        began = time.monotonic()
        try:
            return operation()
        except ValueError:
            errors.append(name)
            return default
        finally:
            timings[name] = round((time.monotonic()-began)*1000, 2)

    white = stage('segmentation', lambda: field.runtime_paint_mask(image, parameters), None)
    paint = stage('projection', lambda: projector.project(white, quaternion), None) if white is not None else None
    circle, lines, spots = None, [], []
    if paint is not None:
        spots=stage('marks',lambda:field_marks.disks(paint,disk_model(parameters)),[])
        circle = stage('circle', lambda: field.detect_circle(paint, scale=.5), None)
        if circle:
            x,y,r = circle['pixel_circle']
            yy,xx = np.indices(paint.shape)
            paint = paint.copy()
            paint[np.abs(np.hypot(xx-x,yy-y)-r)<8] = 0
        lines = stage('lines', lambda: field.observations(paint, scale=.5)[:32], [])
        began=time.monotonic()
        short=stage('strokes',lambda:field_marks.short_lines(paint,lines,short_line_lengths(parameters),
                                      paint_width=parameters['field.geometry']['paint_width']),[])
        timings['marks']+=round((time.monotonic()-began)*1000,2)
        timings.pop('strokes')
        if short:lines=list(lines)+short
    raw = stage('posts', lambda: goal.goal_candidates(image, parameters), [])
    posts = stage('uprights', lambda: goal.upright_candidates(raw, projector, quaternion, engine.goals,
                                      parameters['localisation.goal_height_tolerance_ratio']), [])
    bearings = stage('bearings', lambda: goal.paired_bearings(posts, projector, quaternion), [])
    result = stage('filter', lambda: engine.update(sequence, lines, circle, bearings, timestamp=timestamp,spots=spots),
                   dict(valid=False,candidate=None,reason='invalid_geometry',frame_sequence=sequence))
    matched = goal.mapped_candidates(posts, projector, quaternion, result, engine.goals, engine.goal_foot_tolerance)
    # Discovery and map association are separate. Unmatched single posts remain
    # visible to the operator instead of disappearing with an uncertain pose.
    display = [dict(post, metric_valid=post in matched) for post in posts[:4]]
    result.pop('proposal_spread', None)
    result.pop('ess', None)
    result.pop('mode_mass', None)
    timings['posts'] += timings.pop('uprights') + timings.pop('bearings')
    result.update(goal_rejected=len(raw)-len(matched), goal_candidates=display,
                  detector_errors=errors[:4], timings_ms=timings,spots=len(spots),_spot_observations=spots)
    return result, lines, circle, display
