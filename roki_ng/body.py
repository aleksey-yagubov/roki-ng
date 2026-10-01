"""Single ACM owner and cancellable, bounded manual motion jobs."""

import collections
import copy
import json
import math
import struct
from pathlib import Path
from types import SimpleNamespace
import time
import uuid

from .motion.model import Robot
from .wire import Fault, boolean, choice, number, page
from .calibration import TestPlan, TITLES, MEASUREMENTS, describe
from .calibration import quaternion_yaw, wrap
from .motion import slots
from .body_imu import BodyImu
from .body_servos import BodyServos
from .stabilization import CrouchStabilizer
from .stabilization_diagnostics import StabilizationDiagnostics

ASSETS = Path(__file__).with_name("assets")


class RokiHardware:
    def __init__(self, config):
        import Roki
        self.Roki = Roki
        self.mb = Roki.Motherboard()
        self.check(self.mb.ConfigureACM(timeout_ms=1000), self.mb)
        self.check(self.mb.StopStrobeCapture(), self.mb)
        self.check(self.mb.ConfigureBody(200, 1), self.mb)
        self.rcb = Roki.Rcb4(self.mb)
        self.config = config
        self.mixing_started = False
        self.check(self.mb.SetBodyQueuePeriod(config["parameters"]["motion.frame_ms"]), self.mb)
        self.check(self.mb.ResetBodyQueue(), self.mb)
        self.busy_until = 0
        self.frame_s = config["parameters"]["motion.frame_ms"] / 1000
        self.body_failures = self.mb.GetStatus()["body_failures"]

    def probe(self):
        self.check(self.rcb.checkAcknowledge(), self.rcb)

    def connect_body(self):
        # Flush pending host->body traffic before checking a reattached cable.
        self.reset()
        self.probe()
        self.body_failures = self.mb.GetStatus()["body_failures"]

    def prepare_motion(self):
        if not self.mixing_started and not self.config.get("skip_mixing", False):
            self.check(self.rcb.motionPlay(self.config.get("mixing_slot", 3)), self.rcb)
            self.mixing_started = True

    def check(self, ok, source):
        if not ok:
            # STM rejects a forward request while a queued transaction owns UART8.
            # No command was sent, and this does not mean the cable disconnected.
            if self.mb.GetLastStatus() == 5:
                raise Fault("body_busy", source.GetError(), True)
            raise Fault("hardware_error", source.GetError())

    def send(self, values, frames, pause):
        servos = []
        for value in values:
            servo = self.Roki.Rcb4.ServoData()
            servo.Id, servo.Sio, servo.Data = value.Id, value.Sio, value.Data
            servos.append(servo)
        self.check(self.rcb.setServoPosAsync(servos, int(frames), int(pause)), self.rcb)
        self.busy_until = time.monotonic() + frames * self.frame_s

    def drained(self):
        status = self.check_queue()
        return (status["body_queue_size"] == 0 and not status["body_busy"]
                and time.monotonic() >= self.busy_until)

    def check_queue(self):
        try:
            status = self.mb.GetStatus()
        except RuntimeError as exc:
            raise Fault("hardware_error", f"Reading STM body status: {exc}") from exc
        previous = self.body_failures
        self.body_failures = status["body_failures"]
        if self.body_failures != previous:
            # A later successful packet clears last_body_error, not this counter.
            raise Fault("hardware_error", f"STM body transmission failures: {previous} -> "
                        f"{self.body_failures}; last status {status['last_body_error']}")
        return status

    def reset(self):
        self.check(self.mb.ResetBodyQueue(), self.mb)
        self.busy_until = time.monotonic()

    def head(self, values, frames):
        servos = []
        for value in values:
            servo = self.Roki.Rcb4.ServoData()
            servo.Id, servo.Sio, servo.Data = value.Id, value.Sio, value.Data
            servos.append(servo)
        self.check(self.rcb.setServoPos(servos, frames), self.rcb)

    def body_quaternion(self):
        ok, raw = self.rcb.moveRamToComCmdSynchronize(0x0060, 8)
        self.check(ok, self.rcb)
        if len(raw) != 8:
            raise Fault("imu_invalid", "Body IMU response must contain 8 bytes")
        return tuple(value / 16384 for value in struct.unpack("<hhhh", bytes(raw)))

    def body_positions(self):
        ok, raw = self.rcb.moveRamToComCmdSynchronize(0x0070, 60)
        self.check(ok, self.rcb)
        if len(raw) != 60:
            raise Fault("servo_data_invalid", "Body positions response must contain 60 bytes")
        return struct.unpack("<30h", bytes(raw))


class SimHardware:
    def body_positions(self):
        # Synthetic neutral feedback; never pretend it follows commanded targets.
        return (0,) * 30

    def check_queue(self):
        pass

    def prepare_motion(self):
        pass

    def probe(self):
        pass

    def connect_body(self):
        self.reset()

    def __init__(self, config):
        self.sent = 0
        self.resets = 0
        self.busy_until = 0
        self.frame_s = config["parameters"]["motion.frame_ms"] / 1000

    def send(self, values, frames, pause):
        self.sent += 1
        self.busy_until = time.monotonic() + frames * self.frame_s

    def drained(self):
        return time.monotonic() >= self.busy_until

    def reset(self):
        self.resets += 1
        self.busy_until = 0

    def head(self, values, frames):
        self.sent += 1

    def body_quaternion(self):
        return (math.sqrt(0.5), 0, 0, math.sqrt(0.5))


class Body:
    def __init__(self, config, emit, log):
        self.emit, self.log = emit, log
        self.simulated = config.get("simulate", False)
        self.parameters = config["parameters"]
        self.hardware = SimHardware(config) if self.simulated else RokiHardware(config)
        self.body_connected = self.simulated
        self.body_disabled = config.get("body_disabled", False)
        if self.body_disabled:
            self.body_connected = False
        self.imu = None
        self.imu_error = None
        self.imu_published = 0
        self.imu_rpc_errors = 0
        self.reconnect_at = 0
        self.retry_delay = 1.0
        self.probe_successes = 0
        self.recovery_active = False
        self.model = Robot()
        self.hip_limits = {(servo, bus): sorted((7500 + low * sign, 7500 + high * sign))
                           for servo, bus, sign, _, low, high, *_ in self.model.ACTIVESERVOS
                           if servo == 7}
        self.engine = None
        self.servo_targets = {}
        self.sent_targets = {}
        self.target_times = {}
        self.body_servos = BodyServos(self.model)
        self.servo_watch = False
        self.body_imu = BodyImu()
        self.stabilizer = CrouchStabilizer()
        self.stabilization_diagnostics = StabilizationDiagnostics()
        self.telemetry_watch = False
        self.telemetry_at = 0
        self.stabilization_warning = None
        self.stabilization_log_at = 0
        self.pose = "unknown"
        self.control = False
        self.error = None
        self.active = None
        self.plan = None
        self.jobs = collections.OrderedDict()
        self.calibration_pending = None
        self.drive = None
        self.drive_deadline = 0
        self.stop_requested = False
        self.next_at = 0
        self.instruction = None
        self.instruction_started = 0
        self.head = {"pan": 0, "tilt": 0}
        self.slots = sorted(p.stem for p in (ASSETS / "slots").glob("*.json"))
        self.jumps = json.loads((ASSETS / "jumps.json").read_text())
        self.log("INFO", "Motherboard ready; body connection checked separately; camera remains off")

    def state(self):
        return {"state": "degraded" if not self.body_connected else "fault" if self.error else "ready", "pose": self.pose,
                "body_connected": self.body_connected,
                "recovering": self.recovery_active,
                "body_disabled": self.body_disabled, "imu_running": self.imu is not None,
                "imu_published": self.imu.published if self.imu else self.imu_published,
                "imu_invalid": self.imu.lost if self.imu else 0,
                "imu_error": self.imu_error,
                "imu_rpc_errors": self.imu.total_errors if self.imu else self.imu_rpc_errors,
                "active_job": self.active, "head": self.head, "error": self.error,
                "simulated": self.simulated}

    def _engine(self):
        if self.engine is None:
            from .motion.engine import Engine
            self.engine = Engine(self.parameters, log=self.log)
        return self.engine

    def command(self, op, args):
        try:
            return self._command(op, args)
        except (Fault, OSError) as exc:
            if op.startswith("imu."):
                self.imu_error = str(exc)[:240]
            elif isinstance(exc, OSError) or exc.code == "hardware_error":
                self._link_lost(exc)
            raise

    def _command(self, op, args):
        if op == "body.telemetry.watch":
            self.telemetry_watch = boolean(args, "enabled", False)
            self.servo_watch = self.telemetry_watch and boolean(args, "servos", False)
            self.telemetry_at = 0
            return {}
        if op == "body.telemetry.read":
            if self.body_connected:
                try:
                    if boolean(args, "servos", False):
                        self.body_servos.read(self)
                    else:
                        self.read_body_quaternion(max_age=0.02)
                except Fault as exc:
                    if exc.code not in ("body_busy", "imu_invalid", "servo_data_invalid"):
                        raise
            return self._body_telemetry()
        if op == "control.takeover":
            self.control = False
            self.command("motion.stop_hard", {})
            self.control = True
            return {"stop_confirmed": True}
        if op == "imu.stop":
            if self.imu:
                self.imu_published = self.imu.published
                self.imu_rpc_errors = self.imu.total_errors
                current, self.imu = self.imu, None
                current.close()
            elif not self.simulated:
                self.hardware.check(self.hardware.mb.StopStrobeCapture(), self.hardware.mb)
            return self.state()
        if op == "imu.normal":
            if not self.imu:
                raise Fault("not_ready", "IMU stream not running")
            return self.imu.normal()
        if op == "imu.start":
            if self.imu:
                raise Fault("busy", "IMU already running")
            if self.simulated:
                raise Fault("not_supported", "IMU requires STM")
            from .imu import ImuPublisher
            duration = number(args, "frame_duration_us", 16667, 8333, 100000, True)
            candidate = ImuPublisher(self.hardware, self.emit)
            try:
                clock = candidate.start(duration)
            except Exception:
                candidate.close()
                raise
            self.imu, self.imu_error = candidate, None
            return self.state() | {"clock": clock}
        if op == "calibration.resolve":
            ident = args.get("job_id")
            if ident != self.calibration_pending or ident not in self.jobs:
                raise Fault("not_found", "No matching calibration pending")
            job = self.jobs[ident]
            error = args.get("error")
            if error is None:
                self.parameters.update(job["calibration"])
                self.engine = None
            job.update(status="failed" if error else "completed", reason=error, saved=error is None)
            self.calibration_pending = None
            self.emit("job.failed" if error else "job.completed", dict(job))
            return dict(job)
        if self.calibration_pending and op in (
                "control.acquire", "params.apply", "test.start", "motion.pose", "motion.drive",
                "motion.jump", "motion.slot", "motion.kick", "motion.head",
                "motion.get_up", "motion.splits"):
            raise Fault("busy", "Calibration parameters are being saved", True)
        if op == "control.acquire":
            if self.active:
                raise Fault("busy", "Previous motion is still stopping", True)
            self.control = True
            return {}
        if op == "control.release":
            self.control = False
            self.drive = None
            if self.recovery_active:
                self.command("motion.stop_hard", {})
            elif self.active and self.jobs[self.active]["operation"] in ("motion.drive", "test.start"):
                self.stop_requested = True
            else:
                self.command("motion.stop_hard", {})
            return {}
        if op == "state":
            return self.state()
        if op == "params.apply":
            if self.active:
                raise Fault("busy", "Stop motion before applying motion parameters", True)
            geometry_changed = any(key in args and args[key] != self.parameters[key]
                                   for key in ("walk.gait_height_mm", "walk.sway_amplitude_mm"))
            self.parameters.update(args)
            if self.engine:
                self.engine.configure(self.parameters)
            if geometry_changed:
                self.engine = None
                if self.pose in ("crouch", "crouch_centered"):
                    self.pose = "unknown"
            return {}
        if op == "motion.slots":
            return page(self.slots, args)
        if op == "motion.joints":
            return page(self.body_servos.catalog, args)
        if op == "job.status":
            job = self.jobs.get(args.get("job_id"))
            if job is None:
                raise Fault("not_found", "Job no longer retained")
            return job
        if op == "motion.stop_hard":
            self.drive = None
            self.plan = self.instruction = None
            self.pose = "unknown"
            self.engine = None
            self.servo_targets.clear()
            self.sent_targets.clear()
            self.target_times.clear()
            self.body_servos.invalidate("hard_stop")
            self.stabilizer.reset("hard_stop")
            try:
                self.hardware.reset()
            except Exception as exc:
                self.error = str(exc)
                self._finish("failed", self.error)
                raise
            self._finish("cancelled", "hard_stop")
            if self.body_connected:
                self.error = None
            return {"stopped": True, "pose": "unknown", "scope": "host_body_queue",
                    "body_connected": self.body_connected}
        if op in ("motion.stop_graceful", "job.cancel"):
            if op == "job.cancel" and args.get("job_id") != self.active:
                raise Fault("not_found", "Job is not active")
            if self.recovery_active:
                return self.command("motion.stop_hard", {})
            if self.active and self.jobs[self.active]["operation"] not in ("motion.drive", "test.start", "game.step"):
                raise Fault("not_supported", "This motion requires hard stop or completion")
            self.stop_requested = True
            self.drive = None
            return {"accepted": True, "job_id": self.active}
        if op == "test.list":
            return {"items": list(TITLES)}
        if op == "test.describe":
            return describe(args.get("name"))
        if not self.control:
            raise Fault("not_owner", "Motion control has been released")
        if not self.body_connected:
            raise Fault("body_unavailable", "Body disconnected; reconnect pending", True)
        if op == "motion.head":
            if self.recovery_active:
                raise Fault("busy", "Get-up sequence owns the head", True)
            if self.active and self.jobs[self.active]["operation"] not in ("motion.drive", "test.start"):
                raise Fault("busy", "Software slot/pose owns the head", True)
            pan = number(args, "pan", self.head["pan"], -2666, 2666, True)
            tilt = number(args, "tilt", self.head["tilt"], -2600, 950, True)
            frames = number(args, "frames", 10, 1, 100, True)
            self.hardware.prepare_motion()
            values = [SimpleNamespace(Id=0, Sio=1, Data=7500 + pan),
                      SimpleNamespace(Id=12, Sio=2, Data=7500 + tilt)]
            self.hardware.head(values, frames)
            self.servo_targets.update(((v.Id, v.Sio), v.Data) for v in values)
            self.sent_targets.update(((v.Id, v.Sio), v.Data) for v in values)
            self.target_times.update(((v.Id, v.Sio), time.monotonic()) for v in values)
            self.head = {"pan": pan, "tilt": tilt}
            return {"accepted": True, "target": dict(self.head)}
        if op == "motion.drive":
            drive = {axis: number(args, axis, 0.0, -1, 1) for axis in ("x", "y", "yaw")}
            drive["speed"] = number(args, "speed", 0.5, 0.1, 1)
            drive["crouch"] = choice(args, "crouch", "off", ("off", "on", "centered"))
            drive["heading_hold"] = boolean(args, "heading_hold", False)
            active = any(abs(drive[a]) > 0.05 for a in ("x", "y", "yaw"))
            if self.active and self.jobs[self.active]["operation"] != "motion.drive":
                raise Fault("busy", "One-shot motion running", True)
            if self.stop_requested and self.active:
                raise Fault("busy", "Finishing current walk; wait for completion", True)
            self.drive, self.drive_deadline = drive, time.monotonic() + 0.35
            if active and not self.active:
                return self._start(op, self._walk())
            return {"accepted": True, "job_id": self.active}
        if self.active:
            raise Fault("busy", "Motion already running; command discarded", True)
        if self.error:
            raise Fault("motion_fault", self.error)
        if op == "game.step":
            direction = choice(args, "direction", None, ("left", "right"))
            heading = number(args, "heading", None, -math.pi, math.pi)
            side = number(args, "side_mm", 10., 1., 10.)
            cycles = number(args, "cycles", 1, 1, 2, True)
            if self.pose != "crouch":
                raise Fault("invalid_state", "Goalkeeper step requires prepared crouch")
            return self._start(op, self._game_step(direction, heading, side, cycles))
        if op == "motion.pose":
            name = choice(args, "name", None, ("base_stand", "crouch", "stand", "head_field"))
            if name == "crouch" and choice(args, "crouch", "on", ("on", "centered")) == "centered":
                name = "crouch_centered"
            return self._start(op, self._pose(name))
        if op == "motion.slot":
            name = args.get("name")
            if name not in self.slots:
                raise Fault("not_found", "Unknown software slot")
            factor = number(args, "speed_factor", 1.0, 0.25, 2)
            document = json.loads((ASSETS / "slots" / f"{name}.json").read_text())
            if "frames" in document:
                steps = slots.decode(document, self.model, factor)
                return self._start(op, self._individual_slot(steps))
            if document.get("units", "kondo") != "kondo":
                raise Fault("invalid_motion", "Legacy combined-knee rows require Kondo units")
            rows = document[name]
            self._validate_rows(rows, factor)
            return self._start(op, self._slot(rows, factor))
        if op == "motion.jump":
            direction = choice(args, "direction", "forward", tuple(self.jumps) + ("turn_left", "turn_right"))
            fraction = number(args, "fraction", 1.0, 0.1, 1)
            crouch = choice(args, "crouch", "off", ("off", "on", "centered"))
            if crouch != "off":
                target = "crouch_centered" if crouch == "centered" else "crouch"
                if self.pose == target or self.pose == "base_stand":
                    steps = self._relative_jump_steps(direction, fraction)
                    return self._start(op, self._relative_jump(steps))
                return self._start(op, self._prepare_jump(direction, fraction, target))
            rows = self._jump_rows(direction, fraction)
            return self._start(op, self._slot(rows, legs_only=direction.startswith("turn_")))
        if op == "motion.get_up":
            crouch = choice(args, "crouch", "off", ("off", "on", "centered"))
            return self._start(op, self._get_up(crouch))
        if op == "motion.splits":
            kind = choice(args, "kind", "small", ("small", "big"))
            crouch = choice(args, "crouch", "off", ("off", "on", "centered"))
            return self._start(op, self._splits(kind, crouch))
        if op == "motion.kick":
            leg = choice(args, "leg", "right", ("right", "left"))
            power = number(args, "power", 80, 1, 100, True)
            offset = number(args, "offset", 0, -60, 80, True)
            return self._start(op, self._kick(leg, power, offset))
        if op == "test.start":
            plan = TestPlan(self, args)
            return self._start(op, plan.run(), {"test": plan.args,
                               "manual_parameters": list(MEASUREMENTS.get(plan.name, ()))})
        raise Fault("not_supported", op)

    def _start(self, op, plan, details=None):
        self.hardware.prepare_motion()
        if op == "test.start":
            self.engine = None
            self.pose = "unknown"
        ident = uuid.uuid4().hex
        self.active, self.plan = ident, iter(plan)
        self.stop_requested = False
        self.next_at, self.instruction = 0, None
        self.jobs[ident] = {"job_id": ident, "operation": op, "status": "running", "progress": 0}
        self.jobs[ident].update(details or {})
        while len(self.jobs) > 64:
            self.jobs.popitem(last=False)
        self.emit("job.progress", dict(self.jobs[ident]))
        return {"accepted": True, "job_id": ident}

    def _finish(self, status, reason=None):
        if self.active:
            job = self.jobs[self.active]
            if job["operation"] == "test.start" and self.stop_requested and status == "completed":
                status, reason = "cancelled", "graceful_stop"
            job.update(status=status, reason=reason, pose=self.pose)
            if status == "completed" and job.get("calibration"):
                job["status"] = "saving"
                self.calibration_pending = self.active
                self.emit("calibration.ready", {"job_id": self.active, "values": job["calibration"]})
            else:
                job.pop("calibration", None)
                self.emit("job.failed" if status == "failed" else "job.completed", dict(job))
        self.active = self.plan = self.instruction = None
        self.stop_requested = False

    def tick(self):
        if self.imu:
            try:
                self.imu.tick()
            except Exception as exc:
                self.imu_error = str(exc)[:240]
                try:
                    self._command("imu.stop", {})
                except Exception as stop_error:
                    self.log("WARNING", f"IMU stop failed: {stop_error}")
                self.log("ERROR", f"IMU stopped: {self.imu_error}")
                self.emit("imu.fault", self.state())
        now = time.monotonic()
        if not self.body_disabled and now >= self.reconnect_at:
            self.reconnect_at = now + (1.0 if self.body_connected else self.retry_delay)
            try:
                if self.body_connected:
                    self.hardware.probe()
                else:
                    self.hardware.connect_body()
                    self.probe_successes += 1
                    if self.probe_successes < 3:
                        self.reconnect_at = time.monotonic() + 1.0
                        return
                    self.body_connected = True
                    self.error = None
                    self.retry_delay = 1.0
                    self.log("INFO", "Body connected; no previous motion resumed")
                    self.emit("body.connection", self.state())
            except (Fault, OSError) as exc:
                if isinstance(exc, Fault) and exc.code == "body_busy":
                    self.reconnect_at = time.monotonic() + 0.05
                    return
                self._link_lost(exc)
                return
        if not self.body_connected:
            self._publish_body_telemetry()
            return
        self._tick_body_imu()
        self._tick_body_servos()
        if not self.body_connected:
            return
        if not self.plan:
            self._tick_stabilization()
            self._publish_body_telemetry()
            return
        self.stabilizer.freeze("motion")
        try:
            self.hardware.check_queue()
            if time.monotonic() < self.next_at:
                return
            if self.instruction is None:
                self.instruction = next(self.plan)
                self.instruction_started = time.monotonic()
            kind, *args = self.instruction
            if kind == "drain":
                if not self.hardware.drained():
                    if time.monotonic() - self.instruction_started > 3:
                        raise Fault("hardware_error", "Body queue did not drain within 3 seconds")
                    self.next_at = time.monotonic() + 0.02
                    return
            elif kind == "servo":
                values, frames, pause = args
                if not values or not 1 <= frames <= 255 or not 0 <= pause <= 255:
                    raise Fault("invalid_motion", "Invalid servo frame")
                if any(not 0 <= v.Data <= 16383 for v in values):
                    raise Fault("invalid_motion", "Servo position outside protocol range")
                self._send_targets(values, frames, pause)
                self.next_at = time.monotonic() + self.hardware.frame_s * (pause + 1)
            elif kind == "sleep":
                self.next_at = time.monotonic() + max(0, args[0])
            self.instruction = None
        except StopIteration:
            self._finish("completed")
        except Exception as exc:
            if isinstance(exc, OSError) or isinstance(exc, Fault) and exc.code == "hardware_error":
                self._link_lost(exc)
                return
            self.pose = "unknown"
            self.engine = None
            self.servo_targets.clear()
            self.sent_targets.clear()
            self.target_times.clear()
            self.body_servos.invalidate("motion_fault")
            self.stabilizer.reset("motion_fault")
            self.error = str(exc)
            try:
                self.hardware.reset()
            except Exception as reset_error:
                self.error += f"; reset failed: {reset_error}"
            self.log("ERROR", self.error)
            self._finish("failed", self.error)
        finally:
            self._publish_body_telemetry()

    def read_body_quaternion(self, max_age=0.0):
        return self.body_imu.read(self.hardware, max_age=max_age)

    def _body_telemetry(self):
        now = time.monotonic()
        return {"body.imu": self.body_imu.state(now),
                "body.servos": self.body_servos.state(now),
                "body.stabilization": self.stabilizer.state(self.parameters) | {
                    "diagnostics": self.stabilization_diagnostics.result,
                    "source_mono_ns": int(now * 1e9), "valid": self.body_connected}}

    def _publish_body_telemetry(self):
        now = time.monotonic()
        self.stabilization_diagnostics.tick(self, now)
        warning = self.stabilizer.reason if self.stabilizer.reason in (
            "imu_stale", "tilt_outside_range", "joint_limit") else (
                "saturated" if self.stabilizer.saturated and self.stabilizer.reason == "regulating" else None)
        if (self.parameters["stabilization.enabled"] and warning
                and warning != self.stabilization_warning and now >= self.stabilization_log_at):
            self.log("WARNING", f"Crouch stabilization: {warning}; "
                     f"correction={self.stabilizer.offset_deg:.3f} deg")
            self.stabilization_log_at = now + 1
        self.stabilization_warning = warning
        if self.telemetry_watch and now >= self.telemetry_at:
            self.telemetry_at = now + 0.1
            self.emit("body.telemetry", self._body_telemetry())

    def _tick_body_imu(self):
        if not (self.telemetry_watch or self.parameters["body_imu.poll_enabled"]
                or self.parameters["stabilization.diagnostics_enabled"]
                or self.parameters["stabilization.enabled"]):
            return
        now = time.monotonic()
        # Servo deadlines take precedence. No backlog of missed polling periods.
        if now < self.body_imu.next_at or (self.plan and now >= self.next_at):
            return
        try:
            self.read_body_quaternion()
        except Fault as exc:
            if exc.code == "imu_invalid":
                if self.body_imu.invalid == 1 or self.body_imu.invalid % 50 == 0:
                    self.log("WARNING", f"Body IMU rejected: {exc}")
            elif exc.code != "body_busy":
                self._link_lost(exc)
        except OSError as exc:
            self._link_lost(exc)

    def _send_targets(self, values, frames, pause):
        offsets = self.stabilizer.ticks(self.stabilizer.offset_deg)
        corrected = [SimpleNamespace(Id=v.Id, Sio=v.Sio,
                     Data=v.Data + offsets.get((v.Id, v.Sio), 0)) for v in values]
        if any(not 0 <= v.Data <= 16383 for v in corrected):
            raise Fault("invalid_motion", "Corrected servo position outside protocol range")
        for value in corrected:
            key = value.Id, value.Sio
            if offsets.get(key, 0):
                low, high = self.hip_limits[key]
                if not low <= value.Data <= high:
                    raise Fault("invalid_motion", "Frozen stabilization correction exceeds hip limit")
        self.hardware.send(corrected, frames, pause)
        # Relative trajectories start from nominal targets, never corrected ones.
        self.servo_targets.update(((v.Id, v.Sio), v.Data) for v in values)
        self.sent_targets.update(((v.Id, v.Sio), v.Data) for v in corrected)
        self.target_times.update(((v.Id, v.Sio), time.monotonic()) for v in corrected)

    def _tick_body_servos(self):
        now = time.monotonic()
        if (not self.body_connected
                or not (self.servo_watch or self.parameters["stabilization.diagnostics_enabled"])
                or now < self.body_servos.next_at
                or (self.plan and now >= self.next_at)):
            return
        try:
            self.body_servos.read(self)
        except Fault as exc:
            if exc.code == "servo_data_invalid":
                if self.body_servos.invalid == 1 or self.body_servos.invalid % 25 == 0:
                    self.log("WARNING", f"Body positions rejected: {exc}")
            elif exc.code != "body_busy":
                self._link_lost(exc)
        except OSError as exc:
            self._link_lost(exc)

    def _release_stabilization(self):
        # An absolute trajectory owns its next interpolated target. Do not send a
        # separate zero-correction pose before it. No physical motion happens here.
        self.servo_targets.update(self.sent_targets)
        self.stabilizer.reset("absolute_motion")

    def _tick_stabilization(self):
        regulator = self.stabilizer
        if not self.parameters["stabilization.enabled"]:
            regulator.freeze("disabled")
            return
        if self.pose not in ("crouch", "crouch_centered") or self.error:
            regulator.freeze("pose_not_supported")
            return
        hips = ((7, 1), (7, 2))
        if any(key not in self.servo_targets for key in hips):
            regulator.freeze("missing_targets")
            return
        try:
            if not self.hardware.drained():
                regulator.freeze("queue_busy")
                return
            proposed = regulator.propose(self.body_imu, self.parameters, time.monotonic())
            if proposed is None:
                return
            offsets = regulator.ticks(proposed)
            # Respect the model's hip bounds as well as the wire range. The model
            # stores angles before FACTOR, so convert its limits to wire targets.
            targets = []
            for servo, bus in hips:
                data = self.servo_targets[servo, bus] + offsets[servo, bus]
                lo, hi = self.hip_limits[servo, bus]
                if not max(0, lo) <= data <= min(16383, hi):
                    regulator.freeze("joint_limit")
                    return
                targets.append(SimpleNamespace(Id=servo, Sio=bus, Data=data))
            if offsets == regulator.ticks(regulator.offset_deg):
                regulator.offset_deg = proposed  # Accumulate only sub-tick slew.
                return
            frames = max(1, math.ceil(BodyImu.PERIOD_S / self.hardware.frame_s))
            self.hardware.send(targets, frames, frames - 1)
            self.sent_targets.update(((v.Id, v.Sio), v.Data) for v in targets)
            self.target_times.update(((v.Id, v.Sio), time.monotonic()) for v in targets)
            regulator.commit(proposed)
        except (Fault, OSError) as exc:
            if isinstance(exc, Fault) and exc.code == "body_busy":
                regulator.freeze("queue_busy")
            else:
                self._link_lost(exc)

    def _link_lost(self, exc):
        changed = self.body_connected or self.error is None
        self.body_connected = False
        self.probe_successes = 0
        self.pose = "unknown"
        self.drive = None
        self.engine = None
        self.servo_targets.clear()
        self.sent_targets.clear()
        self.target_times.clear()
        self.body_servos.invalidate("link_lost")
        self.body_imu.invalidate(str(exc)[:240])
        self.stabilizer.reset("link_lost")
        self.error = str(exc)[:240]
        self._finish("failed", self.error)
        try:
            self.hardware.reset()
        except (Fault, OSError):
            pass
        self.reconnect_at = time.monotonic() + self.retry_delay
        self.retry_delay = min(5.0, self.retry_delay * 2)
        if changed:
            self.log("ERROR", f"Body link unavailable: {self.error}; retrying, motion cancelled")
            self.emit("body.connection", self.state())

    def _head(self, pan, tilt, frames=10):
        values = [SimpleNamespace(Id=0, Sio=1, Data=7500 + pan),
                  SimpleNamespace(Id=12, Sio=2, Data=7500 + tilt)]
        yield "servo", values, frames, frames - 1
        yield "drain",
        self.head = {"pan": pan, "tilt": tilt}

    def _pose(self, name):
        if name == "head_field":
            yield from self._head(0, self.parameters["head.field_tilt"])
            return
        if self.pose == name or (name == "stand" and self.pose == "base_stand"):
            return
        if name == "stand" and self.pose == "unknown":
            yield from self._pose("base_stand")
            return
        if name == "stand":
            self._release_stabilization()
        if name == "crouch_centered" or (name == "crouch" and self.pose in (
                "crouch_centered", "splits_small", "splits_big")):
            yield from self._static_crouch(name == "crouch_centered")
            return
        if name == "stand" and self.pose in ("splits_small", "splits_big", "crouch_centered"):
            _, values = self._crouch_target(True, height=215)
            # Match walk_Final_Pose's neutral arms at a reachable leg extension.
            for value in values:
                if value.Id in (1, 2, 3, 4):
                    value.Data = 7500
            frames = self.parameters["walk.crouch_transition_frames"]
            self.pose, self.engine = "unknown", None
            yield "servo", values, frames, frames - 1
            yield "drain",
            self.pose = "stand"
            return
        if name == "crouch":
            # Entering gait from another pose must not reuse its foot geometry.
            self.engine = None
        if name == "base_stand":
            rows = json.loads((ASSETS / "slots/Initial_Pose.json").read_text())["Initial_Pose"]
            yield from self._slot(rows)
            # Re-enable the head at its last commanded target after a hard stop.
            yield from self._head(self.head["pan"], self.head["tilt"])
        elif self.simulated:
            if name == "crouch":
                _, values = self._crouch_target(False)
                yield "servo", values, 4, 3
                yield "drain",
            else:
                yield "sleep", 0.08
        elif name == "crouch":
            yield from self._engine().walk_Initial_Pose(start_mixing=False)
            yield "drain",
        elif name == "stand":
            if self.pose != "crouch":
                raise Fault("invalid_state", "Walk final pose requires crouch")
            yield from self._engine().walk_Final_Pose()
            yield "drain",
        self.pose = name
        if name in ("stand", "base_stand"):
            self.engine = None

    def _game_step(self, direction, heading, side, cycles):
        # Finite autonomous primitive: do not inherit manual drive scaling.
        right_first = direction == "right"
        for cycle in range(cycles):
            if self.stop_requested:
                break
            measured = quaternion_yaw(self.read_body_quaternion())
            error = wrap(measured - heading)
            if abs(error) > .35:
                raise Fault("imu_invalid", "Goalkeeper heading changed")
            rotation = max(-.15, min(.15, error * (-1 if right_first else 1)))
            if self.simulated:
                yield "sleep", .08
            else:
                engine = self._engine()
                engine.first_Leg_Is_Right_Leg = right_first
                yield from engine.walk_Cycle(0, side, rotation, cycle, 1000000)
            yield "drain",
        if not self.simulated:
            yield from self._engine().walk_Cycle(0, 0, 0, 0, 1)
        yield "drain",
        self.pose = "crouch"

    def _walk(self, cycles=None, fixed=None):
        if self.pose != "crouch":
            yield from self._pose("crouch")
        cycle = 0
        crouch = (fixed or self.drive or {}).get("crouch", "off")
        heading = None
        while True:
            drive = fixed if fixed is not None else self.drive
            active = fixed is not None or (drive and time.monotonic() < self.drive_deadline
                       and any(abs(drive[a]) > 0.05 for a in ("x", "y", "yaw")))
            if self.stop_requested or not active or (cycles is not None and cycle >= cycles):
                break
            crouch = drive["crouch"]
            right_first = drive["y"] < 0
            rotation = drive["yaw"] * self.parameters["walk.max_yaw_rad"] * drive["speed"]
            if drive.get("heading_hold", False) and abs(drive["yaw"]) <= 0.05:
                measured = quaternion_yaw(self.read_body_quaternion())
                if heading is None:
                    heading = measured
                limit = self.parameters["walk.heading_max_correction_rad"]
                rotation = max(-limit, min(limit, wrap(measured - heading)
                    * self.parameters["walk.heading_kp"] * (-1 if right_first else 1)))
            else:
                heading = None
            if self.simulated:
                yield "sleep", 0.08
            else:
                engine = self._engine()
                engine.first_Leg_Is_Right_Leg = right_first
                yield from engine.walk_Cycle(
                    drive["x"] * self.parameters["walk.max_step_mm"] * drive["speed"],
                    abs(drive["y"]) * self.parameters["walk.max_side_mm"] * drive["speed"],
                    rotation,
                    cycle, 1000000)
            if drive.get("heading_hold", False):
                yield "drain",
            cycle += 1
            self.jobs[self.active]["progress"] = cycle
            self.emit("job.progress", dict(self.jobs[self.active]))
        # The ordinary last gait frame may have a raised foot. Explicit terminal cycle.
        if cycle and not self.simulated:
            yield from self._engine().walk_Cycle(0, 0, 0, 0, 1)
        yield "drain",
        self.pose = "crouch"
        if crouch == "centered":
            yield from self._static_crouch(True)
        elif crouch == "off":
            yield from self._pose("stand")

    def _crouch_target(self, centered, height=None):
        if self.simulated:
            # Protocol simulation has no IK or physical geometry. These neutral
            # targets exist only to exercise selective-joint command bookkeeping.
            addresses = {(s, b) for s, b, *_ in self.model.ACTIVESERVOS[:21]}
            addresses.update(((13, 1), (13, 2)))
            return None, [SimpleNamespace(Id=s, Sio=b, Data=7500)
                          for s, b in sorted(addresses)]
        from .motion.engine import Engine
        engine = Engine(self.parameters, log=self.log)
        if height is not None:
            engine.gaitHeight = height
        return engine, engine.crouch_target(centered)

    def _static_crouch(self, centered, frames=None):
        engine, values = self._crouch_target(centered)
        frames = frames or self.parameters["walk.crouch_transition_frames"]
        self.pose, self.engine = "unknown", None
        yield "servo", values, frames, frames - 1
        yield "drain",
        self.pose = "crouch_centered" if centered else "crouch"
        self.engine = engine

    def _prepare_jump(self, direction, fraction, target):
        yield from self._static_crouch(target == "crouch_centered")
        yield from self._relative_jump(self._relative_jump_steps(direction, fraction))

    def _individual_slot(self, steps):
        self._release_stabilization()
        self.pose, self.engine = "unknown", None
        yield from steps
        yield "drain",

    def _splits(self, kind, crouch):
        steps = slots.decode(slots.splits(kind == "big"), self.model)
        # Always prepare before the deep entry, including a request from stand.
        target = "crouch_centered" if crouch == "centered" else "crouch"
        if self.pose != target:
            yield from self._static_crouch(target == "crouch_centered")
        yield from self._individual_slot(steps)
        self.pose = f"splits_{kind}"

    def _get_up(self, crouch):
        from .recovery import SLOTS, stable_position
        position = yield from stable_position(self)
        if position is None:
            return
        job = self.jobs[self.active]
        job.update(initial_posture=position, stage="getting_up")
        self.emit("job.progress", dict(job))
        target = "crouch_centered" if crouch == "centered" else "crouch"
        if position == "upright":
            yield from self._pose("base_stand" if crouch == "off" else target)
            return
        name = SLOTS[position]
        rows = json.loads((ASSETS / "slots" / f"{name}.json").read_text())[name]
        self.recovery_active = True
        try:
            if crouch == "off":
                yield from self._slot(rows)
                self.pose = "stand"
            else:
                engine, values = self._crouch_target(crouch == "centered")
                steps = list(self._slot(rows))
                # Side recovery has straight legs in every row: fold them from
                # its first frame, retaining the original arm support sequence.
                if position in ("left", "right"):
                    legs = [v for v in values if 5 <= v.Id <= 10 or v.Id == 13]
                    for index, step in enumerate(steps[:-1]):
                        _, original, frames, pause = step
                        steps[index] = ("servo", [v for v in original
                            if not (5 <= v.Id <= 10 or v.Id == 13)] + legs, frames, pause)
                # Back/stomach recovery ends directly in the calculated crouch,
                # not in the old straight-leg last frame followed by sitting down.
                frames = int(rows[-1][0])
                steps[-2] = ("servo", values, frames, frames - 1)
                yield from steps
                self.engine, self.pose = engine, target
            verified = yield from stable_position(self)
            if verified != "upright":
                raise Fault("get_up_failed", f"Body is still {verified}; no automatic retry")
            job.update(stage="upright", posture_verified=True)
            self.emit("job.progress", dict(job))
        finally:
            self.recovery_active = False

    def _kick(self, leg, power, offset):
        self._release_stabilization()
        self.engine = None
        self.pose = "unknown"
        if self.simulated:
            yield "sleep", 0.15
        else:
            engine = self._engine()
            engine.kick_power = power
            yield from engine.kick(leg == "right", kick_offset=offset)
            yield "drain",
        self.pose = "stand"
        self.engine = None

    def _validate_rows(self, rows, factor=1):
        if not isinstance(rows, list) or not 1 <= len(rows) <= 2048:
            raise Fault("invalid_motion", "Invalid slot rows")
        for row in rows:
            if not isinstance(row, list) or not 2 <= len(row) <= len(self.model.ACTIVESERVOS) + 1:
                raise Fault("invalid_motion", "Invalid slot row")
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in row):
                raise Fault("invalid_motion", "Non-numeric slot value")
            if not 1 <= round(row[0] / factor) <= 255:
                raise Fault("invalid_motion", "Slot duration outside 1..255 frames")

    def _slot(self, rows, factor=1, *, legs_only=False):
        self._validate_rows(rows, factor)
        self._release_stabilization()
        self.pose = "unknown"
        self.engine = None
        for row in rows:
            values = []
            for index, angle in enumerate(row[1:]):
                servo, bus, sign, *_ = self.model.ACTIVESERVOS[index]
                if legs_only and not 5 <= servo <= 10:
                    continue
                data = int(angle * sign / (2 if servo == 8 else 1) + 7500)
                if servo == 8:
                    values.append(SimpleNamespace(Id=13, Sio=bus, Data=data))
                values.append(SimpleNamespace(Id=servo, Sio=bus, Data=data))
            frames = round(row[0] / factor)
            yield "servo", values, frames, frames - 1
        yield "drain",

    def _relative_jump_steps(self, direction, fraction):
        if self.pose not in ("crouch", "crouch_centered", "base_stand"):
            raise Fault("invalid_state", "Relative jump requires crouch or base_stand")
        ids = {5, 10} if direction.startswith("turn_") else (
            {9, 10} if direction in ("forward", "backward") else {6, 10})
        selected = [(i, servo, bus, sign) for i, (servo, bus, sign, *_) in
                    enumerate(self.model.ACTIVESERVOS) if servo in ids]
        if any((servo, bus) not in self.servo_targets for _, servo, bus, _ in selected):
            raise Fault("invalid_state", "Missing commanded joint positions; select crouch first")
        baseline = dict(self.servo_targets)
        rows = self._jump_rows(direction, fraction)
        if direction.startswith("turn_"):
            # No neutral-pose preparation before a relative turn.
            rows = rows[1:]
        elif direction in ("forward", "backward"):
            # Legacy first row puts the left ankle value in the knee column.
            # For the relative variant mirror the right ankle, never move knees.
            for row in rows:
                row[13] = -row[2]
        steps = []
        for row in rows:
            values = []
            for index, servo, bus, sign in selected:
                target = baseline[servo, bus] + round(row[index + 1] * sign)
                if not 0 <= target <= 16383:
                    raise Fault("invalid_motion", "Relative jump exceeds servo protocol range")
                values.append(SimpleNamespace(Id=servo, Sio=bus, Data=target))
            frames = round(row[0])
            steps.append(("servo", values, frames, frames - 1))
        return steps

    def _relative_jump(self, steps):
        engine = self.engine
        pose = self.pose
        self.pose = "unknown"
        self.engine = None
        yield from steps
        yield "drain",
        # Only a completed return to the baseline may reuse the gait state.
        self.pose = pose
        self.engine = engine

    def _jump_rows(self, direction, fraction):
        if direction.startswith("turn_"):
            rows = [[10] + [0] * 21, [2] + [0] * 21, [2] + [0] * 21]
            rows[1][1], rows[1][12] = -700, 700
            rows[1][6] = rows[1][17] = round((1000 if direction == "turn_right" else -1000) * fraction)
            return rows
        rows = copy.deepcopy(self.jumps[direction])
        if direction in ("forward", "backward"):
            for row in rows[:2]:
                for index in (2, 13):
                    row[index] = round(row[index] * fraction)
        else:
            value = round((200 if direction == "right" else -200) * fraction)
            for index in (1, 5, 12, 16):
                rows[0][index] = value
            rows[1][5] = rows[1][16] = value
        return rows

    def close(self):
        try:
            self._command("imu.stop", {})
        except Exception as exc:
            self.log("WARNING", f"Could not stop capture: {exc}")
        try:
            self.command("motion.stop_hard", {})
        except (Fault, OSError) as exc:
            self.log("WARNING", f"Could not confirm body queue reset during shutdown: {exc}")
        if not self.simulated and hasattr(self.hardware, "mb"):
            self.hardware.mb.Close()
