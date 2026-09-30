"""Sequence association after capture-specific counter alignment.

An equal number alone does not prove physical synchronization: hardware tests
found different Unicam/strobe origins at 30 and 60 FPS. The offset must be
established for the capture session before using this helper for vision.
Never substitute a latest or nearest IMU sample on the Linux side.
"""

from collections import OrderedDict
from collections import deque


class CaptureAlignment:
    """Match independent counters by clock-bracketed falling-edge timestamps."""
    def __init__(self, clock, duration_us, required=8):
        self.clock, self.required = clock, required
        self.tolerance_ns = min(duration_us*1000//4, 3000000)
        if not 0 <= clock["uncertainty_ns"] <= 1000000:
            raise ValueError("Invalid clock uncertainty")
        self.frames, self.edges = OrderedDict(), OrderedDict()
        self.matches = deque(maxlen=required)
        self.used_frames = set()
        self.offset = None
        self.residual_ns = None

    def frame(self, sequence, stamp):
        self.frames[sequence] = stamp
        while len(self.frames) > 128:
            old, _ = self.frames.popitem(last=False)
            self.used_frames.discard(old)
        self._match()

    def records(self, records):
        for r in records:
            if r["flags"] & 7 != 7:
                continue
            delta = ((r["fall_us"]-self.clock["stm_us"]+(1 << 31)) & 0xffffffff)-(1 << 31)
            self.edges[r["sequence"]] = self.clock["host_ns"]+delta*1000
        while len(self.edges) > 128:
            self.edges.popitem(last=False)
        self._match()

    def _match(self):
        if self.offset is not None or not self.frames:
            return
        for seq, stamp in list(self.edges.items()):
            choices = [(abs(ns-stamp), n, ns-stamp) for n, ns in self.frames.items()
                       if n not in self.used_frames]
            if not choices:
                break
            error, frame, residual = min(choices)
            if error > self.tolerance_ns:
                continue
            del self.edges[seq]
            self.used_frames.add(frame)
            self.matches.append((frame-seq, residual))
            if len(self.matches) == self.required and len({m[0] for m in self.matches}) == 1:
                self.offset = self.matches[0][0]  # Unicam minus STM
                self.residual_ns = max(abs(m[1]) for m in self.matches)
                break

    def state(self):
        return {"state": "matched" if self.offset is not None else "aligning",
                "unicam_minus_stm": self.offset, "pairs": len(self.matches),
                "max_residual_ns": self.residual_ns,
                "clock_uncertainty_ns": self.clock["uncertainty_ns"]}


class SequenceJoiner:
    def __init__(self, capacity=128, offset=None):
        self.capacity, self.offset = capacity, offset
        self.frames, self.imu = OrderedDict(), OrderedDict()
        self.dropped = 0

    def clear(self):
        self.frames.clear()
        self.imu.clear()

    def _trim(self, values):
        while len(values) > self.capacity:
            values.popitem(last=False)
            self.dropped += 1

    def frame(self, sequence, value):
        if self.offset is None:
            return None
        key = sequence + self.offset
        if not 0 <= key <= 0xffffffff:
            return None
        if key in self.imu:
            return value, self.imu.pop(key)
        self.frames[key] = value
        self._trim(self.frames)
        return None

    def measurement(self, sequence, value):
        if self.offset is None:
            return None
        if sequence in self.frames:
            return self.frames.pop(sequence), value
        self.imu[sequence] = value
        self._trim(self.imu)
        return None

    def set_alignment(self, unicam_minus_stm):
        self.clear()
        self.offset = None if unicam_minus_stm is None else -unicam_minus_stm
