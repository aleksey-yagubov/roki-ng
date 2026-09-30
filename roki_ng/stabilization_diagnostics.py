"""Observational ID7 diagnostics; never changes control or declares a motor fault."""

from .body_servos import target_zubr


class StabilizationDiagnostics:
    def __init__(self):
        self.next_log = 0
        self.sequence = None
        self.targets = None
        self.counts = [0, 0]
        self.result = {"enabled": False}

    def tick(self, body, now):
        if not body.parameters["stabilization.diagnostics_enabled"]:
            self.result = {"enabled": False}
            self.sequence = self.targets = None
            self.counts = [0, 0]
            self.next_log = 0
            return
        sample = body.body_servos.state(now)
        status = "stale"
        sent = measured = differences = None
        if sample["valid"] and body.body_connected:
            sent = [sample["sent"][u] for u in (14, 15)]
            measured = [sample["measured"][u] for u in (14, 15)]
            if any(v is None for v in sent):
                status = "no_targets"
            else:
                differences = [(s - m) * 360 / 16384 for s, m in zip(sent, measured)]
                current = [target_zubr(body.sent_targets[7, bus], bus == 2)
                           if (7, bus) in body.sent_targets else None for bus in (1, 2)]
                ages = [sample["target_age_ms"][u] for u in (14, 15)]
                if body.active or body.pose not in ("crouch", "crouch_centered"):
                    status = "motion_or_other_pose"
                elif current != sent or any(age is None or age < 300 for age in ages):
                    status = "changing_target"
                else:
                    status = "comparing"
        if status != "comparing" or self.targets != sent:
            self.counts = [0, 0]
        if status == "comparing":
            if self.sequence != sample["sequence"]:
                self.counts = [count + 1 if abs(delta) > 2 else 0
                               for count, delta in zip(self.counts, differences)]
            status = "persistent_difference" if max(self.counts) >= 3 else (
                "difference" if any(abs(d) > 2 for d in differences) else "within_tolerance")
        self.sequence, self.targets = sample["sequence"], sent
        self.result = {"enabled": True, "status": status, "servo_sequence": sample["sequence"],
                       "source_mono_ns": sample["source_mono_ns"],
                       "hip_error_deg": differences, "consecutive": list(self.counts)}
        if now < self.next_log:
            return
        self.next_log = now + 1
        pitch = f"{body.body_imu.pitch:.2f}" if body.body_imu.fresh(now) else "stale"
        reg = body.stabilizer
        body.log("INFO", f"Stab diag: pitch={pitch} correction={reg.offset_deg:.2f}deg "
                 f"reason={reg.reason} saturated={reg.saturated} check={status}")
        if sent is not None:
            for index, (unit, side) in enumerate(((14, "R"), (15, "L"))):
                def degrees(value):
                    return f"{value * 360 / 16384:.2f}" if value is not None else "unknown"
                level = "WARNING" if self.counts[index] >= 3 else "INFO"
                body.log(level, f"Stab diag ID7 {side}: nominal={degrees(sample['nominal'][unit])} "
                         f"sent={degrees(sent[index])} measured={degrees(measured[index])}deg "
                         f"age={sample['target_age_ms'][unit]}ms; servo freshness unknown")
