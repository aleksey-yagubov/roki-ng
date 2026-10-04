"""Colour ball observations with legacy field support and synchronized projection.

The original blob centre is projected to the plane at ball-radius height.
The paired head IMU supplies pitch/roll; results
are head-level coordinates. The coordinator rotates them by the acknowledged
stationary pan and rejects frames captured before that head job completed.
"""
import json
import math
from pathlib import Path
import threading
import time
from collections import deque

from .wire import Fault, number

FRESH_SECONDS = .5


def invalid(reason, sequence=None, stamp=None):
    return dict(valid=False, reason=reason, frame_sequence=sequence,
                sensor_timestamp_ns=stamp, x_m=None, y_m=None, rect=None)


def capture_time(stamp, boot_ns, monotonic_now):
    """Map libcamera CLOCK_BOOTTIME exposure timestamp to monotonic age."""
    return monotonic_now - (boot_ns-stamp)/1e9


def field_near_ball(rect, green, white):
    """Accept a green or white component beside/below the ball, as in the original."""
    import cv2
    x, y, w, h = rect
    height, width = green.shape
    regions = ((x+w, y, x+2*w, y+2*h),
               (x-w, y, x, y+2*h),
               (x, y+h-1, x+w, y+2*h))
    for left, top, right, bottom in regions:
        left, top = max(0, left), max(0, top)
        right, bottom = min(width, right), min(height, bottom)
        if left >= right or top >= bottom:
            continue
        for mask in (green, white):
            _, _, stats, _ = cv2.connectedComponentsWithStats(
                mask[top:bottom, left:right], connectivity=8, ltype=cv2.CV_32S)
            if (stats[1:, cv2.CC_STAT_AREA] >= 7).any():
                return True
    return False


def ball_candidates(image, parameters, projector, quaternion, camera_height=None, *, debug=None):
    import cv2
    import numpy as np
    if image.shape != (650, 800, 3) or image.dtype != np.uint8:
        raise ValueError('Expected packed 800x650 BGR image')
    from .vision_diagnostics import lab_mask
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    orange, turf, white = (lab_mask(lab, parameters, profile) for profile in
                           ('orange_ball', 'green_field', 'white_marking'))
    _, labels, stats, centers = cv2.connectedComponentsWithStats(
        orange, connectivity=8, ltype=cv2.CV_32S)
    candidates = [i for i in range(1, len(stats))
                  if stats[i, cv2.CC_STAT_AREA] >= parameters['vision.orange_ball.pixels_min']
                  and stats[i, cv2.CC_STAT_WIDTH]*stats[i, cv2.CC_STAT_HEIGHT]
                  >= parameters['vision.orange_ball.box_area_min']]
    candidates.sort(key=lambda i: centers[i][1],
                    reverse=True)
    if debug is not None:
        debug.update(masks={'Orange LAB': orange, 'Green support': turf, 'White support': white},
                     objects=[], counts={'components': len(stats)-1,
                                         'area_rejected': len(stats)-1-len(candidates),
                                         'eligible': len(candidates), 'examined': 0})
    def decision(rect, reason, **measurement):
        if debug is not None:
            debug['objects'].append(dict(rect=rect, reason=reason, **measurement))
            counts = debug['counts']
            counts['examined'] += 1
            counts[reason] = counts.get(reason, 0)+1
    found = []
    for i in candidates[:10]:
        x, y, w, h, _ = map(int, stats[i])
        rect = [x, y, x+w, y+h]
        if not field_near_ball((x, y, w, h), turf, white):
            decision(rect, 'no_field')
            continue
        # Same ray as the original blob.cx()/cy(), not silhouette-floor contact.
        center = tuple(map(float, centers[i]))
        try:
            px, py = map(float, projector.ground_point(center, quaternion))
        except ValueError:
            decision(rect, 'projection')
            continue
        height = parameters['game.camera_height_m'] if camera_height is None else camera_height
        scale = (height-parameters['game.ball_diameter_m']/2)/parameters['game.camera_height_m']
        px, py = px*scale, py*scale
        if not math.isfinite(px) or not math.isfinite(py) or px <= 0 or math.hypot(px, py) > parameters.get('game.ball_max_distance_m', 3.):
            decision(rect, 'range')
            continue
        decision(rect, 'selected', x_m=px, y_m=py, distance_m=math.hypot(px, py))
        found.append(dict(x_m=px, y_m=py, rect=[x, y, x+w, y+h], center_px=list(center)))
        break  # Original selection: the lowest valid orange blob, not nearest range.
    return found


class BallTracker:
    """Select the nearest candidate and reject stale or out-of-order observations."""
    def __init__(self):
        self.sequence = self.stamp = -1
        self.received = None
        self.result = invalid('no_sync')

    def invalidate(self, reason, sequence=None, stamp=None):
        self.result = invalid(reason, sequence, stamp)
        return self.result

    def update(self, candidates, sequence, stamp, received, now):
        if sequence <= self.sequence or stamp <= self.stamp:
            return self.invalidate('out_of_order', sequence, stamp)
        self.sequence, self.stamp = sequence, stamp
        if now-received > FRESH_SECONDS or received > now:
            return self.invalidate('stale', sequence, stamp)
        self.received = received
        if not candidates:
            return self.invalidate('no_ball', sequence, stamp)
        candidate = min(candidates, key=lambda c: math.hypot(c['x_m'], c['y_m']))
        self.result = dict(valid=True, reason='ok',
                           frame_sequence=sequence, sensor_timestamp_ns=stamp,
                           **{k: v for k, v in candidate.items() if k != 'center_px'})
        return self.result

    def state(self, now):
        age = None if self.received is None else max(0, round((now-self.received)*1000))
        result = self.result
        if self.received is not None and now-self.received > FRESH_SECONDS:
            result = invalid('stale', self.sequence, self.stamp)
        return age, dict(result)


class BallObservation:
    def __init__(self, config):
        self.directory = Path(config.get('state_dir', '.'))/'localisation'
        self.parameters = config['parameters']
        self.parameter_revision = 0
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.tracker = BallTracker()
        self.error = None
        self.geometry_history = deque(maxlen=128)
        from .football_vision import SceneTracker
        self.scene_tracker = SceneTracker()
        self.football = False
        self.scene_pose = None
        self.scene_note = ''
        self.own_decision = None
        self.diagnostic = False
        self.diagnostics = None
        self.video = None

    def geometry(self, args):
        height = number(args, 'height_m', None, .1, .8)
        stamp = number(args, 'stamp', None, 0, 1e12)
        with self.lock:
            if not self.geometry_history or stamp > self.geometry_history[-1][0]:
                self.geometry_history.append((stamp, height))
        return {'accepted': True}

    def football_pose(self, args):
        from .football_vision import SceneTracker
        with self.lock:
            self.scene_pose = dict(args) if args.get('active') else None
            self.scene_tracker = SceneTracker()
            self.scene_note = str(args.get('note', ''))[:110]
        return {'accepted': True}

    def football_decision(self, args):
        with self.lock:
            self.own_decision = dict(blocks=args.get('blocks', [])[:4], received=time.monotonic())
        return {'accepted': True}

    def football_state(self):
        with self.lock:
            scene = self.scene_tracker.current
            if not self.football or self.error or self.stop_event.is_set() or scene is None:
                return {'valid': False, 'reason': self.error or 'no_support_scene'}
            age = time.monotonic()-scene['received']
            return dict(scene, valid=0 <= age <= .5, age_ms=round(age*1000))

    def state(self):
        with self.lock:
            age, result = self.tracker.state(time.monotonic())
            state = dict(running=bool(self.thread and self.thread.is_alive() and not self.stop_event.is_set() and not self.error),
                         error=self.error, age_ms=age, result=result)
            if self.diagnostic:
                state['diagnostics'] = self.diagnostics
            return state

    def apply_parameters(self, values):
        with self.lock:
            updated = self.parameters | values
            if updated != self.parameters:
                self.parameters = updated
                self.parameter_revision += 1
                self.tracker.invalidate('parameters_changed')
                self.diagnostics = None
                self.scene_tracker.previous = self.scene_tracker.current = None
                if self.diagnostic and self.video:
                    self.video.invalidate()

    def start(self, args):
        if self.thread and self.thread.is_alive():
            raise Fault('busy', 'Ball observation already running or stopping')
        offset = args.get('unicam_minus_stm')
        if type(offset) is not int or abs(offset) > 2**32:
            raise Fault('invalid_argument', 'Invalid capture alignment')
        parameters = args.get('parameters')
        if not isinstance(parameters, dict):
            raise Fault('invalid_argument', 'parameters must be an object')
        self.parameters = dict(parameters)
        self.football = bool(args.get('football', False))
        self.own_decision = None
        self.football_pose({'active': False})
        self.stop_event = threading.Event()
        self.tracker = BallTracker()
        self.error = None
        self.diagnostics = None
        self.geometry_history.clear()
        self.thread = threading.Thread(target=self._run, args=(offset,), daemon=True)
        self.thread.start()
        return self.state()

    def _run(self, offset):
        channels, guards, waitset = [], [], None
        try:
            import cv2
            import numpy as np
            from .ground_projection import GroundProjection
            from .dataplane import Channel, FRAME_TOPIC, IMU_TOPIC, FRAME_HEADER, IMU_RECORD
            from .synchronization import SequenceJoiner
            cv2.setNumThreads(1)
            profile = json.loads((self.directory/'profile.json').read_text())
            if profile.get('schema') != 1 or profile.get('capture_size') != [800, 650]:
                raise ValueError('Incompatible calibration profile')
            projector = GroundProjection(self.directory, self.parameters.get('game.camera_height_m', .4068))
            if self.stop_event.is_set():
                return
            # Construct and release every iceoryx object on its owning thread.
            for topic in (FRAME_TOPIC, IMU_TOPIC):
                channels.append(Channel(topic))
            iox = channels[0].iox
            waitset = iox.WaitSetBuilder.new().create(iox.ServiceType.Ipc)
            guards = [waitset.attach_deadline(c.listener, iox.Duration.from_secs(1)) for c in channels]
            joiner = SequenceJoiner(capacity=128)
            joiner.set_alignment(offset)
            last_frame = -1
            while not self.stop_event.is_set():
                waitset.wait_and_process()
                pair = None
                bad_frame = False
                for index, channel in enumerate(channels):
                    channel.listener.try_wait()
                    for _ in range(4 if index == 0 else 128):
                        sample = channel.receive()
                        if sample is None:
                            break
                        payload = sample.payload()
                        view = payload.as_memory_view().cast('B')
                        try:
                            if index:
                                rec = IMU_RECORD.unpack(view)
                                matched = joiner.measurement(rec[0], rec)
                            else:
                                rec = FRAME_HEADER.unpack_from(view)
                                if rec[2:] != (800, 650, 2400) or len(view) != FRAME_HEADER.size+650*2400:
                                    raise ValueError('Invalid frame dimensions')
                                if rec[0] <= last_frame:
                                    bad_frame = True
                                    joiner.clear()
                                    continue
                                last_frame = rec[0]
                                # Keep at most two owned frame buffers, including a selected pair.
                                while len(joiner.frames) >= (1 if pair else 2):
                                    joiner.frames.popitem(last=False)
                                owned = bytes(view[FRAME_HEADER.size:])
                                captured = capture_time(rec[1], time.clock_gettime_ns(time.CLOCK_BOOTTIME), time.monotonic())
                                matched = joiner.frame(rec[0], (rec, owned, captured))
                                del owned
                            if matched is not None and (pair is None or matched[0][0][0] > pair[0][0][0]):
                                pair = matched
                                while len(joiner.frames) > 1:
                                    joiner.frames.popitem(last=False)
                            del matched
                        finally:
                            view.release()
                            del view, payload, sample
                if bad_frame:
                    with self.lock:
                        self.tracker.invalidate('out_of_order')
                    continue
                if pair is None:
                    continue
                (header, data, received), imu = pair
                image = np.frombuffer(data, np.uint8).reshape(650, 800, 3)
                with self.lock:
                    parameters, revision = self.parameters, self.parameter_revision
                    height = next((h for stamp, h in reversed(self.geometry_history) if stamp <= received), None)
                    pose = self.scene_pose
                began = time.monotonic_ns()
                debug = {} if self.diagnostic else None
                ticket = self.video.ticket() if self.diagnostic and self.video else None
                candidates = ball_candidates(image, parameters, projector, imu[2:6], height, debug=debug)
                scene = None
                if self.football and pose and received >= pose['after']:
                    from .football_vision import camera_geometry, plane_point, goal_parts
                    geometry = camera_geometry(parameters, pose['pan'], pose['tilt'])
                    # Use exactly the same blob centre as the normal detector.
                    # A separate projection avoids transferring its old floor-height scale.
                    selected = candidates[0] if candidates else None
                    point = None
                    if selected:
                        pixel = selected['center_px']
                        try:
                            point = plane_point(projector, pixel, imu[2:6], geometry, parameters['game.ball_diameter_m']/2)
                        except ValueError:
                            pass
                    parts = goal_parts(image, parameters, projector, imu[2:6], geometry,
                                       bearing_only=pose.get('goal_bearing_only', False))
                    for part in parts:
                        part.update(frame_sequence=header[0], sensor_timestamp_ns=header[1],
                                    pan=pose['pan'], tilt=pose['tilt'], parameter_revision=revision)
                    scene = dict(pose_id=pose['pose_id'], sequence=header[0], stamp=header[1], parameter_revision=revision,
                                 received=received, ball=point, parts=parts, height_m=geometry['height'],
                                 ball_radius=parameters['game.ball_diameter_m']/2)
                processing_us = (time.monotonic_ns()-began)//1000
                with self.lock:
                    if not self.stop_event.is_set() and revision == self.parameter_revision:
                        result = self.tracker.update(candidates, header[0], header[1], received, time.monotonic())
                        if scene is not None and pose is self.scene_pose:
                            self.scene_tracker.update(scene)
                        if debug is not None:
                            self.diagnostics = dict(counts=debug['counts'], parameter_revision=revision,
                                                    processing_us=processing_us)
                            if ticket is not None:
                                from .vision_diagnostics import render
                                caption = f"Ball frame={header[0]} rev={revision} {result['reason']} {processing_us/1000:.1f}ms"
                                if self.football:
                                    from .football_vision import render_scene
                                    output = render_scene(image, self.scene_tracker.current if pose is self.scene_pose else None,
                                                          self.scene_note, [parameters[f'field.goal.{i}'] for i in (0, 1)],
                                                          (parameters[f"field.goal.{parameters['match.own_goal']}"]['colour']
                                                           if pose and pose.get('goal_veto_enabled', True) else None),
                                                          self.own_decision)
                                else:
                                    output = render(image, debug['masks'], debug['objects'], caption)
                                self.video.submit(ticket, header, output)
                del image, data, pair
        except Exception as exc:
            with self.lock:
                if not self.stop_event.is_set():
                    self.error = str(exc)[:200]
                    self.tracker.invalidate('error')
                    self.diagnostics = None
                    if self.diagnostic and self.video:
                        self.video.invalidate()
        finally:
            guards.clear()
            waitset = None
            for channel in channels:
                channel.close()

    def close(self):
        self.stop_event.set()
        with self.lock:
            self.tracker.invalidate('stopped')
            self.diagnostics = None
            if self.diagnostic and self.video:
                self.video.invalidate()
        if self.thread:
            self.thread.join(2)
            if self.thread.is_alive():
                raise Fault('timeout', 'Ball observation is stopping')
            self.thread = None
