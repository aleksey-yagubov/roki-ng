"""Runtime BGR frames -> appsrc; camera and IMU lifetimes remain independent."""

from .dataplane import FRAME_HEADER,FRAME_TOPIC
from .frame_reader import FrameReader


class RuntimeVideo:
    def __init__(self, appsrc, gst, fps, topic=FRAME_TOPIC):
        self.appsrc, self.gst = appsrc, gst
        self.period_ns = round(1e9 / fps)
        self.submitted = self.rate_skipped = 0
        self.sequence = None
        self.first_stamp = self.next_stamp = None
        self.reader = FrameReader(self.push) if topic==FRAME_TOPIC else FrameReader(self.push,topic=topic)

    @property
    def error(self):
        return self.reader.error

    @property
    def skipped(self):
        return self.rate_skipped + self.reader.skipped

    def push(self, view):
        seq, stamp, width, height, stride = FRAME_HEADER.unpack_from(view)
        if (width, height, stride) != (800, 650, 2400) or len(view) != FRAME_HEADER.size + stride * height:
            raise ValueError("Expected packed 800x650 BGR frame")
        if self.sequence is not None and seq <= self.sequence:
            raise ValueError("Camera sequence restarted; stop and restart runtime video")
        self.sequence = seq
        if self.first_stamp is None:
            self.first_stamp = self.next_stamp = stamp
        if stamp + self.period_ns // 20 < self.next_stamp:
            self.rate_skipped += 1
            return
        # Drop excess input frames, never synthesize copies to increase FPS.
        self.next_stamp += max(1, (stamp - self.next_stamp) // self.period_ns + 1) * self.period_ns
        buffer = self.gst.Buffer.new_allocate(None, stride * height, None)
        # GI marshals memoryview byte-by-byte; bytes uses its bulk-copy path.
        buffer.fill(0, bytes(view[FRAME_HEADER.size:]))
        buffer.pts = stamp - self.first_stamp
        buffer.duration = self.period_ns
        result = self.appsrc.emit("push-buffer", buffer)
        if result != self.gst.FlowReturn.OK:
            raise RuntimeError(f"appsrc push-buffer: {result.value_nick}")
        self.submitted += 1

    def close(self):
        self.reader.close()
