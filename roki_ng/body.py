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


class SimHardware:
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
        self.engine = None
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
                "motion.jump", "motion.slot", "motion.kick", "motion.head"):
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
            self.parameters.update(args)
            if self.engine:
                self.engine.configure(self.parameters)
            return {}
        if op == "motion.slots":
            return page(self.slots, args)
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
            if self.active and self.jobs[self.active]["operation"] not in ("motion.drive", "test.start"):
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
            self.hardware.head([SimpleNamespace(Id=0, Sio=1, Data=7500 + pan),
                                SimpleNamespace(Id=12, Sio=2, Data=7500 + tilt)], frames)
            self.head = {"pan": pan, "tilt": tilt}
            return {"accepted": True, "target": dict(self.head)}
        if op == "motion.drive":
            drive = {axis: number(args, axis, 0.0, -1, 1) for axis in ("x", "y", "yaw")}
            drive["speed"] = number(args, "speed", 0.5, 0.1, 1)
            drive["hold_crouch"] = boolean(args, "hold_crouch", True)
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
        if op == "motion.pose":
            name = choice(args, "name", None, ("base_stand", "crouch", "stand", "head_field"))
            return self._start(op, self._pose(name))
        if op == "motion.slot":
            name = args.get("name")
            if name not in self.slots:
                raise Fault("not_found", "Unknown software slot")
            factor = number(args, "speed_factor", 1.0, 0.25, 2)
            rows = json.loads((ASSETS / "slots" / f"{name}.json").read_text())[name]
            self._validate_rows(rows, factor)
            return self._start(op, self._slot(rows, factor))
        if op == "motion.jump":
            direction = choice(args, "direction", "forward", tuple(self.jumps) + ("turn_left", "turn_right"))
            fraction = number(args, "fraction", 1.0, 0.1, 1)
            rows = self._jump_rows(direction, fraction)
            return self._start(op, self._slot(rows, legs_only=direction.startswith("turn_")))
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
            return
        if not self.plan:
            return
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
                self.hardware.send(values, frames, pause)
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
            self.error = str(exc)
            try:
                self.hardware.reset()
            except Exception as reset_error:
                self.error += f"; reset failed: {reset_error}"
            self.log("ERROR", self.error)
            self._finish("failed", self.error)

    def _link_lost(self, exc):
        changed = self.body_connected or self.error is None
        self.body_connected = False
        self.probe_successes = 0
        self.pose = "unknown"
        self.drive = None
        self.engine = None
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
        if name == "crouch":
            # Entering gait from another pose must not reuse its foot geometry.
            self.engine = None
        if name == "base_stand":
            rows = json.loads((ASSETS / "slots/Initial_Pose.json").read_text())["Initial_Pose"]
            yield from self._slot(rows)
            yield from self._head(0, 0)
        elif self.simulated:
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

    def _walk(self, cycles=None, fixed=None):
        if self.pose != "crouch":
            yield from self._pose("crouch")
        cycle = 0
        hold = True
        while True:
            drive = fixed if fixed is not None else self.drive
            active = fixed is not None or (drive and time.monotonic() < self.drive_deadline
                       and any(abs(drive[a]) > 0.05 for a in ("x", "y", "yaw")))
            if self.stop_requested or not active or (cycles is not None and cycle >= cycles):
                break
            hold = drive["hold_crouch"]
            if self.simulated:
                yield "sleep", 0.08
            else:
                engine = self._engine()
                engine.first_Leg_Is_Right_Leg = drive["y"] < 0
                yield from engine.walk_Cycle(
                    drive["x"] * self.parameters["motion.max_step_mm"] * drive["speed"],
                    abs(drive["y"]) * self.parameters["motion.max_side_mm"] * drive["speed"],
                    drive["yaw"] * self.parameters["motion.max_yaw_rad"] * drive["speed"],
                    cycle, 1000000)
            cycle += 1
            self.jobs[self.active]["progress"] = cycle
            self.emit("job.progress", dict(self.jobs[self.active]))
        # The ordinary last gait frame may have a raised foot. Explicit terminal cycle.
        if cycle and not self.simulated:
            yield from self._engine().walk_Cycle(0, 0, 0, 0, 1)
        yield "drain",
        self.pose = "crouch"
        if not hold:
            yield from self._pose("stand")

    def _kick(self, leg, power, offset):
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
