"""Drain the native ACM reader; never query each IMU frame over RPC."""

import math
import time

from .dataplane import Channel, IMU_TOPIC, IMU_RECORD
from .wire import Fault


class ImuPublisher:
    def __init__(self, hardware, emit=lambda *args: None):
        self.hardware, self.emit = hardware, emit
        self.channel = Channel(IMU_TOPIC, publisher=True)
        self.published = self.lost = self.total_errors = 0
        self.alignment = True
        self.seen = set()
        self.highest = -1
        self.initial_drops = hardware.mb.GetStreamDrops()

    def start(self, duration_us):
        mb = self.hardware.mb
        self.hardware.check(mb.StopStrobeCapture(), mb)
        self.hardware.check(mb.ConfigureStrobeFilterUs(duration_us, 4000), mb)
        probes = []
        for _ in range(12):
            before = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            stm = mb.GetClock()
            after = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
            probes.append({"stm_us": stm, "host_ns": (before+after)//2,
                           "uncertainty_ns": (after-before)//2})
        clock = min(probes, key=lambda p: p["uncertainty_ns"])
        if clock["uncertainty_ns"] > 1000000:
            raise Fault("imu_clock", "ACM clock bracket exceeds 1 ms uncertainty")
        self.hardware.check(mb.StartStrobeCapture(), mb)
        mb.StartIMUStream(2)
        return clock

    def normal(self):
        mode, sequence = self.hardware.mb.SetIMUStreamMode(1)
        self.alignment = False
        return {"mode": mode, "first_sequence": sequence}

    def tick(self):
        mb = self.hardware.mb
        if not mb.IsConnected():
            self.total_errors += 1
            raise Fault("hardware_error", "Motherboard ACM disconnected")
        edges = []
        for record in mb.ReadIMUStream(timeout_ms=0, limit=128):
            seq = record["sequence"]
            # Records can complete out of order at the mode-switch boundary.
            if seq in self.seen:
                continue
            if self.highest > 65000 and seq < 100:
                raise Fault("imu_counter_limit", "Restart capture before STM counter wraps")
            self.highest = max(self.highest, seq)
            self.seen.add(seq)
            if len(self.seen) > 512:
                self.seen = {n for n in self.seen if n >= self.highest-256}
            if self.alignment and record["mode"] == 2:
                edges.append({key: record[key] for key in
                              ("sequence", "flags", "rise_us", "fall_us")})
            if not record["flags"] & 1:
                self.lost += 1
                continue
            frame = record["imu"]
            q = frame.Orientation
            values = (q.X, q.Y, q.Z, q.W)
            if not all(math.isfinite(x) for x in values):
                raise Fault("imu_invalid", "Non-finite quaternion")
            stamp = frame.Timestamp.TimeS*1000000000 + frame.Timestamp.TimeNS
            with self.channel.loan(IMU_RECORD.size) as view:
                IMU_RECORD.pack_into(view, 0, seq, stamp, *values, int(frame.SensorID))
            self.published += 1
        if edges:
            self.emit("imu.alignment", {"records": edges})
        if mb.GetStreamDrops() != self.initial_drops:
            raise Fault("imu_overflow", "Native IMU receive queue overflow")

    def close(self):
        try:
            self.hardware.check(self.hardware.mb.StopStrobeCapture(), self.hardware.mb)
        finally:
            self.channel.close()
