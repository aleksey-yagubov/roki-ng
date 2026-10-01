"""LAB tuning plus an independent synchronized ball observation worker."""

import math
import time

from .dataplane import Channel, FRAME_HEADER, DETECTIONS_TOPIC
from .frame_reader import FrameReader
from .parameters import COLOUR_DEFAULTS, validate_colour_ranges
from .wire import Fault, choice, pack


def colour_blobs(image, parameters, profile):
    import cv2
    p = {key: parameters[f"vision.{profile}.{key}"] for key in
         ("l_min", "l_max", "a_min", "a_max", "b_min", "b_max", "pixels_min", "box_area_min")}
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    low = (int(p["l_min"] * 2.55), p["a_min"] + 128, p["b_min"] + 128)
    high = (math.ceil(p["l_max"] * 2.55), p["a_max"] + 128, p["b_max"] + 128)
    mask = cv2.inRange(lab, low, high)
    _, _, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8, ltype=cv2.CV_32S)
    valid = [i for i in range(1, len(stats)) if stats[i, cv2.CC_STAT_AREA] >= p["pixels_min"]
             and stats[i, cv2.CC_STAT_WIDTH] * stats[i, cv2.CC_STAT_HEIGHT] >= p["box_area_min"]]
    valid.sort(key=lambda i: int(stats[i, cv2.CC_STAT_AREA]), reverse=True)
    blobs = []
    for i in valid[:4]:
        x, y, w, h, pixels = map(int, stats[i])
        blobs.append({"rect": [x, y, x + w, y + h], "pixels": pixels,
                      "center": [float(v) for v in centers[i]]})
    return blobs, len(valid)


class Detection:
    def __init__(self, config, emit, log):
        from .ball_observation import BallObservation
        self.ball = BallObservation(config)
        self.emit, self.log = emit, log
        self.parameters = config["parameters"]
        self.reader = self.publisher = None
        self.profile = "orange_ball"
        self.result = self.published = None
        self.frames = 0
        self.error = None
        self.last_frame_at = 0

    def state(self):
        return {"state": "fault" if self.error else "ready", "running": self.reader is not None,
                "profile": self.profile, "frames": self.frames, "error": self.error,
                "topic": DETECTIONS_TOPIC, "result": self.result,
                "age_ms": round((time.monotonic() - self.last_frame_at) * 1000) if self.result else None}

    def command(self, op, args):
        if op == "ball.start":
            return self.ball.start(args)
        if op == "ball.stop":
            self.ball.close()
            return self.ball.state()
        if op == "ball.status":
            return self.ball.state()
        if op in ("state", "detection.status"):
            return self.state()
        if op == "params.apply":
            updated = self.parameters | args
            validate_colour_ranges(updated)
            self.parameters = updated
            return {}
        if op == "detection.stop":
            self._close_tuning()
            return self.state()
        if op == "detection.start":
            if self.reader:
                raise Fault("busy", "Detector already running")
            self.profile = choice(args, "profile", "orange_ball", tuple(COLOUR_DEFAULTS))
            try:
                import cv2
                cv2.setNumThreads(1)
                self.error = self.result = self.published = None
                self.frames = 0
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
        if self.result and seq <= self.result["frame_sequence"]:
            raise ValueError("Camera restarted without stopping detector")
        began = time.monotonic_ns()
        image = np.ndarray((height, width, 3), dtype=np.uint8, buffer=view, offset=FRAME_HEADER.size)
        blobs, count = colour_blobs(image, self.parameters, self.profile)
        del image
        self.last_frame_at = time.monotonic()
        self.result = {"frame_sequence": seq, "sensor_timestamp_ns": stamp,
                       "size": [width, height], "profile": self.profile,
                       "blobs": blobs, "total_blobs": count,
                       "processing_us": (time.monotonic_ns() - began) // 1000}
        self.frames += 1

    def tick(self):
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

    def _close_tuning(self):
        if self.reader:
            self.reader.close()
            self.reader = None
        if self.publisher:
            self.publisher.close()
            self.publisher = None
        self.result = self.published = None
