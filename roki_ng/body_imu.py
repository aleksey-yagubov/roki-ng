"""One body quaternion cache. Timestamps are host receive times, not IMU times."""

import math
import time

from .wire import Fault


def attitude(quaternion):
    x, y, z, w = quaternion
    norm = x*x + y*y + z*z + w*w
    if not all(math.isfinite(v) for v in quaternion) or not 0.9 < norm < 1.1:
        raise Fault("imu_invalid", "Invalid body quaternion")
    gx = 2 * (x*z - w*y) / norm
    gy = 2 * (y*z + w*x) / norm
    gz = 1 - 2 * (x*x + y*y) / norm
    # Mounted PCB: upright gravity is +Y, stomach-down is positive pitch.
    return math.degrees(math.atan2(-gz, gy)), math.degrees(math.atan2(gx, gy))


class BodyImu:
    MAX_AGE_S = 0.15
    PERIOD_S = 0.02

    def __init__(self):
        self.quaternion = None
        self.received_at = None
        self.pitch = self.roll = None
        self.next_at = 0
        self.sequence = self.busy = self.invalid = 0
        self.error = None

    def read(self, hardware, max_age=0.0):
        now = time.monotonic()
        if self.fresh(now, max_age):
            return self.quaternion
        self.next_at = now + self.PERIOD_S
        try:
            quaternion = hardware.body_quaternion()
            pitch, roll = attitude(quaternion)
        except Fault as exc:
            if exc.code == "body_busy":
                self.busy += 1
            else:
                self.invalidate(str(exc))
                self.invalid += 1
            raise
        self.quaternion, self.pitch, self.roll = tuple(quaternion), pitch, roll
        self.received_at = time.monotonic()
        self.sequence += 1
        self.error = None
        return self.quaternion

    def invalidate(self, reason):
        self.quaternion = self.received_at = None
        self.pitch = self.roll = None
        self.error = reason

    def fresh(self, now, max_age=MAX_AGE_S):
        return self.received_at is not None and 0 <= now - self.received_at < max_age

    def state(self, now):
        return {"valid": self.fresh(now), "sequence": self.sequence,
                "source_mono_ns": int(self.received_at * 1e9) if self.received_at is not None else None,
                "timestamp_kind": "host_receive", "quaternion_xyzw": self.quaternion,
                "pitch_deg": self.pitch, "roll_deg": self.roll,
                "busy_reads": self.busy, "invalid_reads": self.invalid, "error": self.error}
