"""LAB tuning plus an independent synchronized ball observation worker."""

import threading
import time

from .dataplane import Channel, FRAME_HEADER, DETECTIONS_TOPIC
from .frame_reader import FrameReader
from .parameters import COLOUR_DEFAULTS, validate_colour_ranges
from .wire import Fault, choice, pack
from .vision_diagnostics import DiagnosticVideo, lab_mask, render


def colour_blobs(image, parameters, profile, *, debug=None):
    import cv2
    p = {key: parameters[f"vision.{profile}.{key}"] for key in
         ("l_min", "l_max", "a_min", "a_max", "b_min", "b_max", "pixels_min", "box_area_min")}
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    mask = lab_mask(lab, parameters, profile)
    _, _, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8, ltype=cv2.CV_32S)
    valid = [i for i in range(1, len(stats)) if stats[i, cv2.CC_STAT_AREA] >= p["pixels_min"]
             and stats[i, cv2.CC_STAT_WIDTH] * stats[i, cv2.CC_STAT_HEIGHT] >= p["box_area_min"]]
    valid.sort(key=lambda i: int(stats[i, cv2.CC_STAT_AREA]), reverse=True)
    blobs = []
    for i in valid[:4]:
        x, y, w, h, pixels = map(int, stats[i])
        blobs.append({"rect": [x, y, x + w, y + h], "pixels": pixels,
                      "center": [float(v) for v in centers[i]]})
    if debug is not None:
        debug.update(mask=mask, components=len(stats)-1, area_rejected=len(stats)-1-len(valid))
    return blobs, len(valid)


class Detection:
    def __init__(self, config, emit, log):
        from .ball_observation import BallObservation
        self.ball = BallObservation(config)
        self.video = DiagnosticVideo()
        self.ball.video = self.video
        self.lock = threading.Lock()
        self.parameter_revision = 0
        self.mode = 'colour'
        self.emit, self.log = emit, log
        self.parameters = config["parameters"]
        self.reader = self.publisher = None
        self.profile = "orange_ball"
        self.result = self.published = None
        self.frames = 0
        self.error = None
        self.last_frame_at = 0
        self.last_sequence = -1
        self.reported_ball_error = None

    def state(self):
        if self.mode in ('ball', 'football'):
            ball = self.ball.state()
            if self.ball.football:
                scene = self.ball.football_state()
                point = scene.get('ball') if scene.get('valid') else None
                ball['result'] = dict(valid=point is not None, reason='ok' if point else 'no_support_scene',
                                     frame_sequence=scene.get('sequence'), x_m=point[0] if point else None,
                                     y_m=point[1] if point else None)
            return ball | {'mode': 'football' if self.ball.football else 'ball', 'state': 'fault' if ball['error'] else 'ready',
                           'video': self.video.state()}
        return {"state": "fault" if self.error else "ready", "running": self.reader is not None,
                "mode": "colour", "video": self.video.state(),
                "profile": self.profile, "frames": self.frames, "error": self.error,
                "topic": DETECTIONS_TOPIC, "result": self.result,
                "age_ms": round((time.monotonic() - self.last_frame_at) * 1000) if self.result else None}

    def video_source(self):
        from .dataplane import DETECTION_FRAME_TOPIC
        from .video_sources import frame_source
        state = self.state()
        return frame_source('Диагностика LAB / мяча', DETECTION_FRAME_TOPIC, 800, 650,
                            available=state['running'] and not state['error'],
                            reason=None if state['running'] and not state['error'] else 'detector_not_running',
                            requested=self.video.requested, publishing=self.video.state()['publishing'],
                            on_demand=True, dependencies=('camera',), fps=5)

    def command(self, op, args):
        if op == "ball.start":
            result = self.ball.start(args)
            if args.get('football'):
                self.mode = 'ball'
                self.ball.diagnostic = True
            return result
        if op == 'football.pose':
            return self.ball.football_pose(args)
        if op == 'football.status':
            return self.ball.football_state()
        if op == 'football.decision':
            return self.ball.football_decision(args)
        if op == "ball.stop":
            self.ball.close()
            if self.ball.diagnostic:
                self.ball.diagnostic = False
                self.mode = 'colour'
                self.video.stop()
            return self.ball.state()
        if op == "ball.status":
            return self.ball.state()
        if op == 'ball.geometry':
            return self.ball.geometry(args)
        if op in ("state", "detection.status"):
            return self.state()
        if op in ('source.start', 'source.stop', 'source.status'):
            if args:
                raise Fault('invalid_argument', 'Diagnostic video has no source parameters')
            if op == 'source.start':
                if not self.state()['running']:
                    raise Fault('not_ready', 'Start detection first')
                self.video.start()
            elif op == 'source.stop':
                self.video.stop()
            return self.video.state()
        if op == "params.apply":
            updated = self.parameters | args
            validate_colour_ranges(updated)
            with self.lock:
                self.parameters = updated
                self.parameter_revision += 1
                self.result = self.published = None
                self.video.invalidate()
            self.ball.apply_parameters(args)
            return {}
        if op == "detection.stop":
            self._stop_diagnostic()
            return self.state()
        if op == "detection.start":
            if self.reader or self.ball.diagnostic:
                raise Fault("busy", "Detector already running")
            mode = choice(args, 'mode', 'colour', ('colour', 'ball', 'football'))
            if mode in ('ball', 'football'):
                if self.ball.state()['running']:
                    raise Fault('busy', 'Game owns ball observation')
                self.mode = mode
                self.ball.diagnostic = True
                try:
                    self.ball.start({'parameters': self.parameters,
                                     'unicam_minus_stm': args.get('unicam_minus_stm'), 'football': mode == 'football'})
                    if mode == 'football':
                        self.ball.football_pose(args['pose'])
                except Exception:
                    self.ball.diagnostic = False
                    self.mode = 'colour'
                    raise
                self.log('INFO', 'Ball diagnostics started; no motion requested')
                self.reported_ball_error = None
                return self.state()
            self.mode = mode
            self.profile = choice(args, "profile", "orange_ball", tuple(COLOUR_DEFAULTS))
            try:
                import cv2
                cv2.setNumThreads(1)
                self.error = self.result = self.published = None
                self.frames = 0
                self.last_sequence = -1
                self.publisher = Channel(DETECTIONS_TOPIC, publisher=True)
                self.last_frame_at = time.monotonic()
                self.reader = FrameReader(self._consume)
            except Exception:
                self._close_tuning()
                raise
            self.log("INFO", f"LAB detector started: {self.profile}")
            return self.state()
        raise Fault("not_supported", op)

    def _consume(self, view):
        import numpy as np
        seq, stamp, width, height, stride = FRAME_HEADER.unpack_from(view)
        if (width, height, stride) != (800, 650, 2400) or len(view) != FRAME_HEADER.size + stride * height:
            raise ValueError("Expected packed 800x650 BGR frame")
        if seq <= self.last_sequence:
            raise ValueError("Camera restarted without stopping detector")
        self.last_sequence = seq
        began = time.monotonic_ns()
        image = np.ndarray((height, width, 3), dtype=np.uint8, buffer=view, offset=FRAME_HEADER.size)
        with self.lock:
            parameters, revision = self.parameters, self.parameter_revision
        ticket = self.video.ticket()
        debug = {}
        blobs, count = colour_blobs(image, parameters, self.profile, debug=debug)
        processing_us = (time.monotonic_ns() - began) // 1000
        rendered = None
        if ticket is not None:
            caption = f"{self.profile} frame={seq} rev={revision} blobs={count} area_rejected={debug['area_rejected']} {processing_us/1000:.1f}ms"
            rendered = render(image, {self.profile: debug['mask']}, blobs, caption)
        with self.lock:
            if revision != self.parameter_revision:
                return
            self.last_frame_at = time.monotonic()
            self.result = {"frame_sequence": seq, "sensor_timestamp_ns": stamp,
                       "size": [width, height], "profile": self.profile,
                       "blobs": blobs, "total_blobs": count,
                       "area_rejected": debug['area_rejected'], "parameter_revision": revision,
                       "processing_us": processing_us}
            if rendered is not None:
                self.video.submit(ticket, (seq, stamp, width, height, stride), rendered)
            self.frames += 1

    def tick(self):
        if error := self.video.tick():
            self.log('ERROR', f'Diagnostic video: {error}; detector continues')
        if self.ball.diagnostic:
            error = self.ball.state()['error']
            if error and error != self.reported_ball_error:
                self.reported_ball_error = error
                self.log('ERROR', f'Ball diagnostics: {error}')
                self.emit('detection.fault', self.state())
        if not self.reader:
            return
        error = self.reader.error
        if time.monotonic() - self.last_frame_at > 5:
            error = error or "No detector frames for 5 seconds"
        result = self.result
        if not error and result is not None and result is not self.published:
            try:
                data = pack(result, 4096)
                with self.publisher.loan(len(data)) as view:
                    view[:] = data
                self.published = result
            except Exception as exc:
                error = str(exc)
        if error:
            self.error = error[:240]
            self._close_tuning()
            self.log("ERROR", self.error)
            self.emit("detection.fault", self.state())

    def close(self):
        try:
            self.ball.close()
        finally:
            self._close_tuning()

    def _stop_diagnostic(self):
        if self.ball.diagnostic:
            self.ball.close()
            self.ball.diagnostic = False
        self.mode = 'colour'
        self._close_tuning()

    def _close_tuning(self):
        self.video.stop()
        if self.reader:
            self.reader.close()
            self.reader = None
        if self.publisher:
            self.publisher.close()
            self.publisher = None
        self.result = self.published = None
