"""On-demand, bounded diagnostic video. IPC objects belong to the worker thread."""
import math
import threading
import time

from .dataplane import Channel, DETECTION_FRAME_TOPIC, FRAME_HEADER


def lab_mask(lab, parameters, profile):
    import cv2
    p = [parameters[f'vision.{profile}.{k}'] for k in
         ('l_min', 'l_max', 'a_min', 'a_max', 'b_min', 'b_max')]
    return cv2.inRange(lab, (int(p[0]*2.55), p[2]+128, p[4]+128),
                       (math.ceil(p[1]*2.55), p[3]+128, p[5]+128))


def render(image, masks, objects, caption):
    """All panels come from one raw camera frame, before video compression."""
    import cv2
    import numpy as np
    annotated = image.copy()
    for item in objects:
        x, y, right, bottom = item['rect']
        reason = item.get('reason', 'blob')
        colour = (0, 255, 0) if reason in ('selected', 'blob') else (0, 0, 255)
        cv2.rectangle(annotated, (x, y), (right, bottom), colour, 2)
        label = reason
        # These fields are produced only by the ball observer's diagnostic
        # path.  The raw camera stream remains untouched.
        if reason == 'selected' and all(isinstance(item.get(key), (int, float))
                                        for key in ('x_m', 'y_m', 'distance_m')):
            label = reason
        cv2.putText(annotated, label, (x, max(18, y-4)), cv2.FONT_HERSHEY_SIMPLEX, .5, colour, 1)
    canvas = np.zeros((650, 800, 3), np.uint8)
    panels = [('Raw + decisions', annotated)] + list(masks.items())
    for index, (label, panel) in enumerate(panels[:4]):
        tile = cv2.resize(panel, (400, 325), interpolation=cv2.INTER_NEAREST)
        if tile.ndim == 2:
            tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        x, y = index % 2 * 400, index // 2 * 325
        canvas[y:y+325, x:x+400] = tile
        cv2.rectangle(canvas, (x, y), (x+399, y+22), (0, 0, 0), -1)
        cv2.putText(canvas, label, (x+5, y+16), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
    if len(panels) == 2:
        for index, text in enumerate((
                'White mask pixels pass the saved LAB thresholds.',
                'Boxes: up to 4 largest blobs passing both area filters.',
                'No box? Check pixels_min / box_area_min and area_rejected.',
                'For the eyedropper select the raw camera stream.')):
            cv2.putText(canvas, text, (15, 370+30*index), cv2.FONT_HERSHEY_SIMPLEX, .5, (220, 220, 220), 1)
    measured = next((item for item in objects if item.get('reason') == 'selected'
                     and all(key in item for key in ('x_m', 'y_m', 'distance_m'))), None)
    if measured is not None:
        # Draw after tiling, at full output resolution, so values stay legible.
        cv2.rectangle(canvas, (0, 548), (799, 623), (0, 0, 0), -1)
        text = (f"BALL  X={measured['x_m']*1000:.0f}  Y={measured['y_m']*1000:+.0f}"
                f"  D={measured['distance_m']*1000:.0f} mm")
        cv2.putText(canvas, text, (12, 580), cv2.FONT_HERSHEY_SIMPLEX, .95, (0, 255, 0), 2)
        cv2.putText(canvas, 'X forward | Y +left / -right | D ground range',
                    (12, 611), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 1)
    cv2.rectangle(canvas, (0, 624), (799, 649), (0, 0, 0), -1)
    cv2.putText(canvas, caption, (5, 642), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1)
    return canvas


class DiagnosticVideo:
    """One owned latest frame; no rendering unless a receiver requests output."""
    def __init__(self):
        self.lock = threading.Lock()
        self.requested = False
        self.generation = 0
        self.latest = self.sent = None
        self.channel = None
        self.last_render = self.last_sent = 0
        self.error = None

    def invalidate(self):
        with self.lock:
            self.generation += 1
            self.latest = self.sent = None
            self.last_sent = 0
            self.last_render = 0

    def start(self):
        if self.channel is None:
            self.channel = Channel(DETECTION_FRAME_TOPIC, publisher=True)
        with self.lock:
            self.error = None
            self.requested = True

    def ticket(self):
        with self.lock:
            now = time.monotonic()
            if not self.requested or now-self.last_render < .2:
                return None
            self.last_render = now
            return self.generation

    def submit(self, ticket, header, image):
        data = FRAME_HEADER.pack(*header) + image.tobytes()
        with self.lock:
            if self.requested and ticket == self.generation:
                self.latest = (time.monotonic(), data)

    def tick(self):
        try:
            with self.lock:
                latest = self.latest
                if not self.channel or latest is None or latest is self.sent or time.monotonic()-latest[0] > .5:
                    return
                with self.channel.loan(len(latest[1])) as view:
                    view[:] = latest[1]
                self.sent = latest
                self.last_sent = time.monotonic()
        except Exception as exc:
            self.stop()
            self.error = str(exc)[:200]
            return self.error

    def state(self):
        return dict(requested=self.requested, publishing=bool(self.last_sent and time.monotonic()-self.last_sent < 2),
                    error=self.error)

    def stop(self):
        with self.lock:
            self.requested = False
        self.invalidate()
        if self.channel:
            self.channel.close()
            self.channel = None
