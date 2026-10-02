"""Battery ADC cache; timestamps refer to receiving the controller reply."""

import time

from .wire import Fault


class BodyPower:
    PERIOD_S = 1.0
    MAX_AGE_S = 3.0

    def __init__(self, simulated=False):
        self.simulated = simulated
        self.adc = self.received_at = None
        self.next_at = 0
        self.sequence = self.busy = self.invalid = 0
        self.error = None

    def read(self, hardware):
        now = time.monotonic()
        if self.received_at is not None and now - self.received_at < self.PERIOD_S:
            return
        self.next_at = now + self.PERIOD_S
        try:
            adc = hardware.body_power_adc()
        except (Fault, OSError) as exc:
            if isinstance(exc, Fault) and exc.code == "body_busy":
                self.busy += 1
            else:
                self.invalid += 1
                self.invalidate(str(exc)[:240])
            raise
        self.adc = adc
        self.received_at = time.monotonic()
        self.sequence += 1
        self.error = None

    def invalidate(self, reason):
        self.adc = self.received_at = None
        self.error = reason

    def state(self, now, voltage_scale=1.0):
        return {"valid": self.received_at is not None and 0 <= now - self.received_at < self.MAX_AGE_S,
                "source_mono_ns": int(self.received_at * 1e9) if self.received_at is not None else None,
                "timestamp_kind": "host_receive", "sequence": self.sequence,
                # Zubr CS_VOLTAGE_CONV: 2702 ADC counts correspond to 10 V.
                "voltage_v": self.adc * 10.0 / 2702.0 * voltage_scale if self.adc is not None else None,
                "voltage_scale": voltage_scale,
                "adc_raw": self.adc, "simulated": self.simulated,
                "busy_reads": self.busy, "invalid_reads": self.invalid, "error": self.error}
