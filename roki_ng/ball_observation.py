"""Colour ball observations with legacy field support and synchronized projection.

The bottom of the orange silhouette approximates floor contact. A sphere's visible
silhouette is not its contact point: range has a radius/perspective bias. This is
not radius-corrected metrology. Pan must be held centred by the coordinator; the
paired head IMU supplies pitch/roll and the coordinator handles body heading.
"""
import json
import math
from pathlib import Path
import threading
import time

from .wire import Fault

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


def ball_candidates(image, parameters, projector, quaternion):
    import cv2
    import numpy as np
    if image.shape != (650, 800, 3) or image.dtype != np.uint8:
        raise ValueError('Expected packed 800x650 BGR image')
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    def mask(profile):
        p = [parameters[f'vision.{profile}.{k}'] for k in
             ('l_min', 'l_max', 'a_min', 'a_max', 'b_min', 'b_max')]
        return cv2.inRange(lab, (int(p[0]*2.55), p[2]+128, p[4]+128),
                           (math.ceil(p[1]*2.55), p[3]+128, p[5]+128))
    orange, turf, white = mask('orange_ball'), mask('green_field'), mask('white_marking')
    _, labels, stats, _ = cv2.connectedComponentsWithStats(
        orange, connectivity=8, ltype=cv2.CV_32S)
    candidates = [i for i in range(1, len(stats))
                  if stats[i, cv2.CC_STAT_AREA] >= parameters['vision.orange_ball.pixels_min']
                  and stats[i, cv2.CC_STAT_WIDTH]*stats[i, cv2.CC_STAT_HEIGHT]
                  >= parameters['vision.orange_ball.box_area_min']]
    candidates.sort(key=lambda i: stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT]/2,
                    reverse=True)
    found = []
    for i in candidates[:10]:
        x, y, w, h, _ = map(int, stats[i])
        if not field_near_ball((x, y, w, h), turf, white):
            continue
        # Keep the existing silhouette-foot projection; do not change geometry
        # together with candidate selection. Isolate this component from neighbours.
        component = (labels[y:y+h, x:x+w] == i).astype(np.uint8)
        contours, _ = cv2.findContours(component, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour = np.concatenate(contours) + (x, y)
        points = contour.reshape(-1, 2)
        bottom = points[:, 1].max()
        foot_x = float(np.mean(points[points[:, 1] >= bottom-1, 0]))
        try:
            px, py = map(float, projector.ground_point((foot_x, float(bottom)), quaternion))
        except ValueError:
            continue
        if not math.isfinite(px) or not math.isfinite(py) or px <= 0 or math.hypot(px, py) > parameters.get('game.ball_max_distance_m', 3.):
            continue
        found.append(dict(x_m=px, y_m=py, rect=[x, y, x+w, y+h]))
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
                           frame_sequence=sequence, sensor_timestamp_ns=stamp, **candidate)
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

    def state(self):
        with self.lock:
            age, result = self.tracker.state(time.monotonic())
            return dict(running=bool(self.thread and self.thread.is_alive() and not self.stop_event.is_set() and not self.error),
                        error=self.error, age_ms=age, result=result)

    def apply_parameters(self, values):
        with self.lock:
            updated = self.parameters | values
            if updated != self.parameters:
                self.parameters = updated
                self.parameter_revision += 1
                self.tracker.invalidate('parameters_changed')

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
        self.stop_event = threading.Event()
        self.tracker = BallTracker()
        self.error = None
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
                candidates = ball_candidates(image, parameters, projector, imu[2:6])
                with self.lock:
                    if not self.stop_event.is_set() and revision == self.parameter_revision:
                        self.tracker.update(candidates, header[0], header[1], received, time.monotonic())
                del image, data, pair
        except Exception as exc:
            with self.lock:
                if not self.stop_event.is_set():
                    self.error = str(exc)[:200]
                    self.tracker.invalidate('error')
        finally:
            guards.clear()
            waitset = None
            for channel in channels:
                channel.close()

    def close(self):
        self.stop_event.set()
        with self.lock:
            self.tracker.invalidate('stopped')
        if self.thread:
            self.thread.join(2)
            if self.thread.is_alive():
                raise Fault('timeout', 'Ball observation is stopping')
            self.thread = None
