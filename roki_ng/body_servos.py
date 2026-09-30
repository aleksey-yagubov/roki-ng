"""Read-only Kondo RAM diagnostics in native Zubr units, not a feedback controller."""

import time

from .motion.slots import joints, MIRRORED
from .wire import Fault


def catalog(model):
    mapping = {servo * 2 + bus - 1: (name, servo, bus)
               for name, (servo, bus) in joints(model).items()}
    return [{"unit": unit, "name": mapping[unit][0] if unit in mapping else None,
             "id": unit // 2, "sio": unit % 2 + 1}
            for unit in range(30)]


def target_zubr(value, mirrored):
    # Match C++ signed integer division (truncate towards zero).
    scaled = (value - 7500) * 1536
    scaled = (abs(scaled) // 1000) * (-1 if scaled < 0 else 1)
    return -scaled if mirrored else scaled


class BodyServos:
    PERIOD_S = 0.2
    MAX_AGE_S = 0.6

    def __init__(self, model):
        self.catalog = catalog(model)
        self.next_at = 0
        self.received_at = None
        self.sequence = self.busy = self.invalid = 0
        self.error = None
        self.sample = {}

    def invalidate(self, reason):
        self.received_at = None
        self.sample = {}
        self.error = reason

    def read(self, body):
        now = time.monotonic()
        self.next_at = now + self.PERIOD_S
        try:
            measured = body.hardware.body_positions()
            if len(measured) != 30:
                raise Fault("servo_data_invalid", "Body positions must contain 30 values")
        except Fault as exc:
            if exc.code == "body_busy":
                self.busy += 1
            else:
                self.invalid += 1
                self.invalidate(str(exc))
            raise
        self.received_at = time.monotonic()
        nominal, sent, ages = [], [], []
        for joint in self.catalog:
            key = joint["id"], joint["sio"]
            mirror = joint["name"] in MIRRORED
            nominal.append(target_zubr(body.servo_targets[key], mirror)
                           if key in body.servo_targets else None)
            sent.append(target_zubr(body.sent_targets[key], mirror)
                        if key in body.sent_targets else None)
            at = body.target_times.get(key)
            ages.append(min(65535, max(0, round((self.received_at - at) * 1000)))
                        if at is not None else None)
        self.sequence += 1
        self.error = None
        self.sample = {"measured": list(measured), "nominal": nominal, "sent": sent,
                       "target_age_ms": ages, "simulated": body.simulated}

    def state(self, now):
        return {"valid": self.received_at is not None and 0 <= now - self.received_at < self.MAX_AGE_S,
                "source_mono_ns": int(self.received_at * 1e9) if self.received_at is not None else None,
                "sequence": self.sequence, "units": "zubr", "unit_count": 30,
                "servo_freshness": "unknown", "timestamp_kind": "host_receive",
                "busy_reads": self.busy, "invalid_reads": self.invalid, "error": self.error,
                **self.sample}
