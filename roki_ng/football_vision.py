"""Robot-relative football observations; no map or accumulated robot pose."""
import math
import time
from itertools import combinations

from .calibration import wrap

TICK = .00058909


def camera_geometry(p, pan, tilt):
    neck = p['game.neck_height_m']
    ref = p['game.camera_reference_tilt']*TICK
    angle, pitch = -pan*TICK, tilt*TICK
    radius = neck*math.sin(-pitch)
    return dict(height=p['game.camera_height_m']+neck*(math.cos(pitch)-math.cos(ref)),
                x=p['game.camera_forward_m']-neck*math.sin(-ref)+radius*math.cos(angle),
                y=radius*math.sin(angle), pan=angle)


def body_point(point, geometry):
    x, y = point
    c, s = math.cos(geometry['pan']), math.sin(geometry['pan'])
    return [geometry['x']+c*x-s*y, geometry['y']+s*x+c*y]


def plane_point(projector, pixel, q, geometry, z=0.):
    ray = projector._pixel_ray(pixel, q)
    if abs(float(ray[2])) < .04:
        raise ValueError('Ray near horizon')
    scale = (z-geometry['height'])/float(ray[2])
    if scale <= 0:
        raise ValueError('Plane behind camera')
    point = body_point((float(ray[0])*scale, float(ray[1])*scale), geometry)
    if math.hypot(*point) > 6:
        raise ValueError('Object beyond usable range')
    return point


def goal_parts(image, p, projector, q, geometry, *, bearing_only=False):
    """Colour regions for steering; two image-side boundaries for a strike.

    No mapped goal dimensions, turf support or top-height projection required.
    A colour blob alone never becomes a shootable opening.
    """
    import cv2
    import numpy as np
    from .colour_masks import mask
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    parts = []
    for colour in ('blue', 'yellow'):
        binary = cv2.morphologyEx(mask(lab, p, colour+'_posts'), cv2.MORPH_CLOSE,
                                 np.ones((9, 9), np.uint8))
        n, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
        posts = []

        def opening(left, right, rect):
            if rect[0] <= 1 or rect[2] >= 799 or max(left[1], right[1]) >= 648:
                return  # Clipped side/base cannot establish the opening.
            try:
                ends = [plane_point(projector, pixel, q, geometry) for pixel in (left, right)]
            except ValueError:
                return
            parts.append(dict(kind='opening', colour=colour, points=ends, rect=rect,
                              area=int(np.count_nonzero(binary[rect[1]:rect[3], rect[0]:rect[2]]))))

        for i in sorted(range(1, n), key=lambda j: int(stats[j, 4]), reverse=True)[:6]:
            x, y, w, h, area = map(int, stats[i])
            if area < (30 if bearing_only else 120) or h < (4 if bearing_only else 12) or w < (4 if bearing_only else 6):
                continue
            rect = [x, y, x+w, y+h]
            try:
                ray = projector._pixel_ray((x+w/2, y+h/2), q)
                bearing = wrap(math.atan2(float(ray[1]), float(ray[0]))+geometry['pan'])
                edge_angles = []
                for u in ((x+w/2, x+w/2) if bearing_only else (x, x+w-1)):
                    ray = projector._pixel_ray((u, y+h/2), q)
                    edge_angles.append(wrap(math.atan2(float(ray[1]), float(ray[0]))+geometry['pan']))
            except ValueError:
                # Fisheye frames include pixels outside the calibrated view.
                # Reject this region, not the complete ball/goal worker.
                continue
            parts.append(dict(kind='colour', colour=colour, points=[[2*math.cos(bearing), 2*math.sin(bearing)]],
                              rect=rect, bearings=edge_angles, area=area))
            if bearing_only:
                continue  # Keeper steering only: never declare a shootable opening.
            component = labels[y:y+h, x:x+w] == i
            # Follow the two outside edges row-by-row, even on curved/leaning
            # tubes. Middle rows exclude both upper AND lower crossbars.
            rows = []
            for row in range(int(h*.2), max(int(h*.2)+1, int(h*.8))):
                xs = np.flatnonzero(component[row])
                if len(xs) < 2:
                    continue
                lo, hi = int(xs[0]), int(xs[-1])
                if hi-lo >= max(24, w*.4) and component[row, lo:hi+1].mean() < .55:
                    rows.append((row, lo, hi))
            if h >= 30 and len(rows) >= max(10, int(h*.25)):
                rows = np.asarray(rows, float)
                # Extrapolate only within this component, to the lower base.
                left = float(np.polyval(np.polyfit(rows[:, 0], rows[:, 1], 1), h-1))
                right = float(np.polyval(np.polyfit(rows[:, 0], rows[:, 2], 1), h-1))
                left, right = max(0., min(w-1., left)), max(0., min(w-1., right))
                if right-left >= 24:
                    opening((x+left, y+h-1), (x+right, y+h-1), rect)
                continue
            if h >= 30 and h >= 1.3*w:
                xs = np.flatnonzero(np.any(component[max(0, h-5):], axis=0))
                if len(xs):
                    posts.append((rect, (x+float(np.median(xs)), y+h-1)))
        # Separate colour components can be the two uprights without a bar.
        for (a, af), (b, bf) in combinations(posts, 2):
            if a[0] > b[0]:
                a, af, b, bf = b, bf, a, af
            overlap = min(a[3], b[3])-max(a[1], b[1])
            if b[0]-a[2] >= 20 and overlap >= .4*min(a[3]-a[1], b[3]-b[1]):
                opening(af, bf, [a[0], min(a[1], b[1]), b[2], max(a[3], b[3])])
    return parts


def assemble_goals(parts, specs):
    """Select an opening, or a non-shootable colour hint for steering."""
    result = []
    for spec in specs:
        candidates = [part for part in parts if part['colour'] == spec['colour'] and part['kind'] == 'opening']
        hints = [part for part in parts if part['colour'] == spec['colour'] and part['kind'] == 'colour']
        bearings = [part['bearings'] for part in hints]
        if candidates:
            ends = candidates[0]['points']
            centre = [(a+b)/2 for a, b in zip(*ends)]
            ambiguous = any(math.dist(centre, [(a+b)/2 for a, b in zip(*other['points'])]) > .5
                            for other in candidates[1:])
            if not ambiguous:
                result.append(dict(colour=spec['colour'], ends=ends, elements='opening',
                                   shootable=True, bearings=bearings, colour_sources=hints,
                                   opening_source=candidates[0]))
                continue
        if hints:
            hint = max(hints, key=lambda part: part['area'])
            result.append(dict(colour=spec['colour'], ends=hint['points']*2, elements='colour only',
                               shootable=False, bearings=bearings, colour_sources=hints))
    return result


def own_colour_ahead(goal):
    """A visible own-colour region can veto a kick without a complete goal."""
    for left, right in goal.get('bearings', ()):
        half = abs(wrap(right-left))/2
        centre = wrap(left+wrap(right-left)/2)
        if abs(centre) <= half+math.radians(10):
            return True
    return False


def own_goal_blocks(ball, goal, radius):
    """Explain the existing veto, preserving its exact colour/opening rules."""
    if not goal:
        return []
    blocks = []
    def detail(reason, source, **extra):
        return dict(reason=reason, colour=goal['colour'],
                    **{key: source.get(key) for key in ('frame_sequence', 'sensor_timestamp_ns',
                       'pan', 'tilt', 'rect', 'area', 'parameter_revision')}, **extra)
    for index, (left, right) in enumerate(goal.get('bearings', ())):
        centre = wrap(left+wrap(right-left)/2)
        if abs(centre) <= abs(wrap(right-left))/2+math.radians(10):
            sources = goal.get('colour_sources', [])
            source = sources[index] if index < len(sources) else {}
            blocks.append(detail('colour_ahead', source, bearing_deg=math.degrees(centre),
                                 edges_deg=[math.degrees(left), math.degrees(right)]))
    if ball is not None and shot_clear(ball, goal, -radius):
        blocks.append(detail('shot_intersects_opening', goal.get('opening_source', {}),
                             ends=goal['ends']))
    return blocks


def shot_clear(ball, goal, margin, heading=0.):
    """Ray from ball through the interior of a goal segment (metres)."""
    if not goal.get('shootable', False):
        return False
    a, b = goal['ends']
    d = [math.cos(heading), math.sin(heading)]
    v = [b[0]-a[0], b[1]-a[1]]
    w = [a[0]-ball[0], a[1]-ball[1]]
    cross = lambda x, y: x[0]*y[1]-x[1]*y[0]
    det = cross(d, v)
    length = math.hypot(*v)
    if abs(det) < 1e-6 or length <= 2*margin:
        return False
    along, side = cross(w, v)/det, cross(w, d)/det
    return along > 0 and margin/length <= side <= 1-margin/length


class SceneTracker:
    def __init__(self):
        self.previous = None
        self.current = None

    def update(self, scene):
        previous = self.previous
        self.previous = scene
        valid = previous and previous['pose_id'] == scene['pose_id'] and scene['sequence'] > previous['sequence']
        old_ball = previous.get('ball') if valid else None
        ball = scene.get('ball')
        confirmed_ball = ball if ball and old_ball and math.dist(ball, old_ball) <= .06 else None
        parts = []
        for part in scene['parts']:
            if valid and any(old['kind'] == part['kind'] and old['colour'] == part['colour']
                             and len(old['points']) == len(part['points'])
                             and max(math.dist(a, b) for a, b in zip(old['points'], part['points'])) <= .25
                             for old in previous['parts']):
                parts.append(part)
        self.current = scene | {'ball': confirmed_ball, 'parts': parts, 'confirmed': bool(valid)}
        return self.current


def render_scene(image, scene, note, specs=(), own_colour=None, decision=None):
    import cv2
    import numpy as np
    output = image.copy()
    if scene:
        for part in scene['parts']:
            x, y, x2, y2 = part['rect']
            colour = (0, 255, 0) if part['kind'] == 'opening' else (0, 180, 255)
            cv2.rectangle(output, (x, y), (x2, y2), colour, 2)
            cv2.putText(output, part['colour']+' '+part['kind'], (x, max(20, y-5)),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, colour, 2)
    if scene:
        inset = np.zeros((260, 240, 3), np.uint8)
        point = lambda xy: (int(120-xy[1]*70), int(240-xy[0]*70))
        cv2.circle(inset, point([0, 0]), 5, (255, 255, 255), -1)
        for goal in assemble_goals(scene['parts'], specs):
            if goal['shootable']:
                cv2.line(inset, point(goal['ends'][0]), point(goal['ends'][1]), (255, 255, 0), 4)
            cv2.putText(inset, goal['colour']+' '+goal['elements'], (4, 20 if goal['colour'] == 'blue' else 40),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
        if scene.get('ball'):
            bx, by = scene['ball']
            cv2.circle(inset, point([bx, by]), 4, (0, 140, 255), -1)
            cv2.arrowedLine(inset, point([bx, by]), point([bx+2, by]), (0, 255, 0), 2)
        output[20:280, 550:790] = inset
    cv2.rectangle(output, (0, 540), (799, 649), (0, 0, 0), -1)
    ball = scene.get('ball') if scene else None
    text = f'BALL X={ball[0]*1000:.0f} Y={ball[1]*1000:+.0f} D={math.hypot(*ball)*1000:.0f} mm' if ball else 'BALL: no confirmed support-pose observation'
    cv2.putText(output, text, (8, 570), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 0), 2)
    height = f"H={scene['height_m']*1000:.0f} mm" if scene else 'geometry inactive'
    cv2.putText(output, height+' | '+note[:75], (8, 601), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    cv2.putText(output, note[75:150], (8, 629), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    if scene and own_colour:
        own = next((goal for goal in assemble_goals(scene['parts'], specs)
                    if goal['colour'] == own_colour), None)
        for block in own_goal_blocks(scene.get('ball'), own, scene.get('ball_radius', .045)):
            rect = block.get('rect')
            if rect:
                x, y, x2, y2 = map(int, rect)
                cv2.rectangle(output, (x, y), (x2, y2), (0, 0, 255), 3)
                cv2.putText(output, 'OWN GOAL BLOCK: '+block['reason'], (max(0,x), max(22,y-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
        cv2.putText(output, f"Frame check #{scene['sequence']}; actual decision sources in log",
                    (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
    if decision and decision.get('blocks') and 0 <= time.monotonic()-decision['received'] <= 3:
        block = decision['blocks'][0]
        cv2.rectangle(output, (0, 606), (799, 649), (0, 0, 0), -1)
        label = (f"LAST OWN GOAL BLOCK: {block['reason']} source #{block.get('frame_sequence')} "
                 f"pan={block.get('pan')} tilt={block.get('tilt')}")
        cv2.putText(output, label, (8, 631), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1)
    return output
