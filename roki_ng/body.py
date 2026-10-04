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
from .calibration import wrap
from .motion import slots
from .body_imu import BodyImu, quaternion_yaw
from .body_servos import BodyServos
from .body_power import BodyPower
from .stabilization import CrouchStabilizer
from .stabilization_diagnostics import StabilizationDiagnostics
from .controller_kick import KICK_SLOT, STOP_SLOT, KICK_PARAMETERS, KickWitness

ASSETS = Path(__file__).with_name("assets")


class BodyTransmissionError(Fault):
    def __init__(self, message, count=1):
        super().__init__("hardware_error", message)
        self.count = count


class RokiHardware:
    forward_pending = False
    controller_pending = False

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
        self._direct_available()
        self.check(self.rcb.checkAcknowledge(), self.rcb, forwarded=True)

    def connect_body(self):
        # Flush pending host->body traffic before checking a reattached cable.
        self.reset()
        self.check_queue()
        self.probe()
        self.body_failures = self.mb.GetStatus()["body_failures"]

    def reconnect_motherboard(self):
        # Reuse the client: its retired RPC IDs protect against late responses.
        # Keep controller_pending until its STOP has actually been acknowledged.
        self.check(self.mb.ConfigureACM(timeout_ms=1000), self.mb)
        self.check(self.mb.StopStrobeCapture(), self.mb)
        self.check(self.mb.ConfigureBody(200, 1), self.mb)
        self.check(self.mb.SetBodyQueuePeriod(round(self.frame_s * 1000)), self.mb)
        self.forward_pending = False
        self.body_failures = self.mb.GetStatus()['body_failures']
        self.mixing_started = False
        self.check(self.mb.ResetBodyQueue(), self.mb)
        self.busy_until = time.monotonic()
        # connect_body() will stop a possibly running controller slot before
        # acknowledging the body. A missing Zubr must not reboot healthy STM.

    def prepare_motion(self):
        self._forward_available()
        if not self.mixing_started and not self.config.get("skip_mixing", False):
            self._direct_available()
            self.check(self.rcb.motionPlay(self.config.get("mixing_slot", 3)), self.rcb, forwarded=True)
            self.mixing_started = True

    def check(self, ok, source, forwarded=False):
        if not ok:
            # STM rejects a forward request while a queued transaction owns UART8.
            # No command was sent, and this does not mean the cable disconnected.
            status = self.mb.GetLastStatus()
            if status == 5:
                raise Fault("body_busy", source.GetError(), True)
            if forwarded and status in (8, 9, 10):
                # Firmware counts failed synchronous reads and queued movement
                # in the same counter. Account for THIS transaction exactly
                # once; never swallow other queued failures by copying GetStatus.
                self.body_failures = (self.body_failures + 1) % (2**32)
            elif forwarded and status == 255:
                # Host lost the response; STM may still be executing this
                # read/command. Resolve it before accepting another movement.
                self.forward_pending = True
            raise Fault("hardware_error", source.GetError())

    def _forward_available(self):
        if self.forward_pending:
            raise Fault("body_busy", "Previous direct transaction outcome pending", True)
        if self.controller_pending:
            raise Fault('body_busy', 'Controller kick completion/stop not confirmed', True)

    def _direct_available(self):
        if not self.telemetry_ready():
            raise Fault("body_busy", "Direct transaction deferred while queue owns UART", True)

    def send(self, values, frames, pause):
        self._forward_available()
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
        if self.forward_pending:
            if status["body_busy"]:
                return status
            self.forward_pending = False
            # With no newly submitted movements, at most one increment belongs
            # to the direct transaction already reported to its caller.
            if status["body_failures"] != previous:
                previous = self.body_failures = (previous + 1) % (2**32)
        self.body_failures = status["body_failures"]
        if self.body_failures != previous:
            # A later successful packet clears last_body_error, not this counter.
            delta = (self.body_failures - previous) % (2**32)
            count = delta if delta < 2**31 else 1  # Firmware reboot, not billions of failures.
            raise BodyTransmissionError(f"STM body transmission failures: {previous} -> "
                                        f"{self.body_failures}; last status {status['last_body_error']}", count)
        return status

    def telemetry_ready(self):
        try:
            status = self.mb.GetStatus()
        except RuntimeError as exc:
            raise Fault("hardware_error", f"Reading telemetry readiness: {exc}") from exc
        # No direct read may overlap queued execution. Defer also when queue
        # failures still need to be collected by the important-operation path.
        return (not self.forward_pending and status["body_failures"] == self.body_failures
                and status["body_queue_size"] == 0 and not status["body_busy"])

    def _read_ram(self, address, size):
        self._direct_available()
        try:
            ok, raw = self.rcb.moveRamToComCmdSynchronize(address, size)
        except (RuntimeError, OSError) as exc:
            self.forward_pending = True
            raise Fault("hardware_error", f"Body RAM read failed: {exc}") from exc
        self.check(ok, self.rcb, forwarded=True)
        return raw

    def reset(self):
        self.check(self.mb.ResetBodyQueue(), self.mb)
        self.busy_until = time.monotonic()
        if self.controller_pending:
            # Clearing STM's queue cannot stop a script already running in Zubr.
            # Keep the latch until STOP is acknowledged, including reconnects.
            self.check_queue()  # Resolve an outstanding/lost direct response first.
            self._direct_available()
            self._controller_call(self.rcb, self.rcb.motionPlay, STOP_SLOT)
            self.controller_pending = False
            self.mixing_started = False

    def _controller_call(self, source, method, *args):
        try:
            result = method(*args)
        except (RuntimeError, OSError) as exc:
            self.forward_pending = True
            raise Fault('hardware_error', f'Controller transaction outcome unknown: {exc}') from exc
        self.check(result[0] if isinstance(result, tuple) else result, source, forwarded=True)
        return result

    def controller_write(self, address, value):
        self._direct_available()
        if not hasattr(self, 'zubr'):
            self.zubr = self.Roki.Zubr(self.mb)
        self._controller_call(self.zubr, self.zubr.memISet, address, value)

    def controller_read(self, address):
        self._direct_available()
        ok, value = self._controller_call(self.zubr, self.zubr.memIGet, address)
        return value

    def controller_start(self):
        self._direct_available()
        # A lost ACK may mean the kick DID start. Never replay it.
        self.controller_pending = True
        try:
            self._controller_call(self.rcb, self.rcb.motionPlay, KICK_SLOT)
        except Fault as exc:
            if exc.code == 'body_busy':
                # STM explicitly rejected forwarding; no slot was started.
                self.controller_pending = False
            raise

    def controller_completed(self):
        self.controller_pending = False
        self.mixing_started = self.config.get('mixing_slot', 3) == 3

    def head(self, values, frames):
        self._direct_available()
        servos = []
        for value in values:
            servo = self.Roki.Rcb4.ServoData()
            servo.Id, servo.Sio, servo.Data = value.Id, value.Sio, value.Data
            servos.append(servo)
        self.check(self.rcb.setServoPos(servos, frames), self.rcb, forwarded=True)

    def body_quaternion(self):
        raw = self._read_ram(0x0060, 8)
        if len(raw) != 8:
            raise Fault("imu_invalid", "Body IMU response must contain 8 bytes")
        return tuple(value / 16384 for value in struct.unpack("<hhhh", bytes(raw)))

    def body_positions(self):
        raw = self._read_ram(0x0070, 60)
        if len(raw) != 60:
            raise Fault("servo_data_invalid", "Body positions response must contain 60 bytes")
        return struct.unpack("<30h", bytes(raw))

    def body_power_adc(self):
        raw = self._read_ram(0x00CC, 2)
        if len(raw) != 2:
            raise Fault("power_data_invalid", "Body power response must contain 2 bytes")
        return struct.unpack("<H", bytes(raw))[0]


class SimHardware:
    controller_pending = False

    def controller_write(self, address, value):
        pass

    def controller_start(self):
        self.controller_pending = True
        self.controller_reads = 0

    def controller_read(self, address):
        self.controller_reads += 1
        values = ({18: 2, 16: KickWitness.ROTATION_BITS, 2: 0, 3: 0}
                  if self.controller_reads <= 5 else {18: 0, 16: 0, 2: 100, 3: 100})
        return values[address]

    def controller_completed(self):
        self.controller_pending = False

    def telemetry_ready(self):
        return True

    def body_power_adc(self):
        return 3242

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
        self.controller_pending = False

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
        self.motherboard_reset_pending = False
        self.retry_delay = 1.0
        self.probe_successes = 0
        self.important_errors = collections.deque(maxlen=3)
        self.last_important_error = None
        self.queue_reset_pending = False
        self.queue_reset_at = 0
        self.fall_sequence = 0
        self.fall_samples = 0
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
        self.body_power = BodyPower(self.simulated)
        self.power_watch = False
        self.imu_watch = False
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
        self.drive_rearm_required = False
        self.drive_deadline = 0
        self.stop_requested = False
        self.next_at = 0
        self.instruction = None
        self.segment_active = False
        self.instruction_started = 0
        self.head = {"pan": 0, "tilt": 0}
        self.slots = sorted(p.stem for p in (ASSETS / "slots").glob("*.json"))
        self.jumps = json.loads((ASSETS / "jumps.json").read_text())
        self.log("INFO", "Motherboard ready; body connection checked separately; camera remains off")

    def state(self):
        body_imu = self.body_imu.state(time.monotonic())
        return {"state": "degraded" if not self.body_connected else "fault" if self.error else "ready", "pose": self.pose,
                "body_connected": self.body_connected,
                "recovering": self.recovery_active,
                "body_disabled": self.body_disabled, "imu_running": self.imu is not None,
                "imu_published": self.imu.published if self.imu else self.imu_published,
                "imu_invalid": self.imu.lost if self.imu else 0,
                "imu_error": self.imu_error,
                "imu_rpc_errors": self.imu.total_errors if self.imu else self.imu_rpc_errors,
                "active_job": self.active, "head": self.head, "error": self.error,
                "important_errors_5s": self._important_error_count(),
                "last_important_error": self.last_important_error,
                "queue_reset_pending": self.queue_reset_pending,
                "body_imu": {key: body_imu[key] for key in ("valid", "sequence", "source_mono_ns")},
                "simulated": self.simulated}

    def _engine(self):
        if self.engine is None:
            from .motion.engine import Engine
            self.engine = Engine(self.parameters, log=self.log)
        return self.engine

    def command(self, op, args):
        # Nested commands (takeover -> stop) must not count the same failed
        # transaction twice. Each top-level call is a distinct attempt.
        nested = getattr(self, "in_command", False)
        self.in_command = True
        try:
            return self._command(op, args)
        except (Fault, OSError) as exc:
            if op.startswith("imu."):
                self.imu_error = str(exc)[:240]
                self.log("WARNING", f"{op} failed: {exc}")
            elif not nested and (isinstance(exc, OSError) or exc.code == "hardware_error"):
                self._important_failure(exc)
            elif not nested:
                self.log("WARNING", f"{op} rejected: {exc}")
            raise
        finally:
            self.in_command = nested

    def _command(self, op, args):
        if op == 'motherboard.reconnect':
            if not self.motherboard_reset_pending:
                raise Fault('invalid_state', 'No motherboard reset pending')
            self.hardware.reconnect_motherboard()
            self.motherboard_reset_pending = False
            self.probe_successes = 0
            self.reconnect_at = 0
            return self.state()
        if op == "body.telemetry.watch":
            self.telemetry_watch = boolean(args, "enabled", False)
            self.servo_watch = self.telemetry_watch and boolean(args, "servos", False)
            self.imu_watch = self.telemetry_watch and boolean(args, "imu", True)
            self.power_watch = self.telemetry_watch and boolean(args, "power", False)
            self.telemetry_at = 0
            return {}
        if op == "body.telemetry.read":
            power, servos = boolean(args, "power", False), boolean(args, "servos", False)
            if self.body_connected:
                try:
                    if power:
                        self.body_power.read(self.hardware)
                    elif servos:
                        self.body_servos.read(self)
                    # IMU snapshots never initiate a hardware transaction.
                except (Fault, OSError) as exc:
                    self.log("WARNING", f"Body telemetry read failed: {exc}")
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
            job['saved'] = error is None
            self.calibration_pending = None
            if self.active == ident:
                if error:
                    self._finish('failed', error)
                else:
                    job.update(status='running', stage='calibration_saved', reason=None)
                    self.emit('job.progress', dict(job))
            else:
                # Cancellation/fall during disk I/O must never restart the plan.
                if job['status'] == 'saving':
                    job.update(status='failed' if error else 'completed', reason=error)
                self.emit('job.failed' if job['status'] == 'failed' else 'job.completed', dict(job))
            return dict(job)
        if self.calibration_pending and op in (
                "control.acquire", "params.apply", "test.start", "motion.pose", "motion.drive",
                "motion.jump", "motion.slot", "motion.kick", "motion.head",
                "motion.get_up", "motion.splits", "game.walk", "game.jump", "game.head"):
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
            if job['operation'] == 'game.walk':
                from .game_geometry import camera_height
                height = self.parameters['game.camera_height_m']
                if self.active == job['job_id'] and self.body_imu.fresh(time.monotonic()):
                    height = camera_height(self.parameters, self.model, self.engine,
                                           self.body_imu.pitch, self.body_imu.roll,
                                           self.head['pan'], self.head['tilt'])
                return job | {'camera_geometry': {'height_m': height, 'stamp': time.monotonic()}}
            return job
        if op == "motion.stop_hard":
            controller_stop = self.hardware.controller_pending
            if controller_stop and self.active:
                self.jobs[self.active]['execution_uncertain'] = True
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
                self.error = None if isinstance(exc, Fault) and exc.code == 'body_busy' else str(exc)
                self.queue_reset_pending = True
                self.queue_reset_at = time.monotonic() + .05
                self._finish("failed", str(exc))
                raise
            self._finish("cancelled", "hard_stop")
            if self.body_connected:
                self.error = None
            return {"stopped": True, "pose": "unknown", "scope": "host_body_queue_and_controller" if controller_stop else "host_body_queue",
                    "body_connected": self.body_connected}
        if op in ("motion.stop_graceful", "job.cancel"):
            if op == "job.cancel" and args.get("job_id") != self.active:
                raise Fault("not_found", "Job is not active")
            if self.recovery_active:
                return self.command("motion.stop_hard", {})
            if self.active and self.jobs[self.active].get('kick_type') == 'controller':
                self.stop_requested = True
                return {"accepted": True, "job_id": self.active}
            if self.active and self.jobs[self.active]["operation"] not in ("motion.drive", "test.start", "game.walk", "game.jump"):
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
        if self.queue_reset_pending:
            raise Fault("body_unavailable", "Body queue reset not confirmed", True)
        if self.error:
            raise Fault("motion_fault", self.error)
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
            if not active:
                self.drive_rearm_required = False
            elif self.drive_rearm_required:
                raise Fault('not_ready', 'Release walking controls before restarting after a fall or lost IMU')
            if self.active and self.jobs[self.active]["operation"] != "motion.drive":
                raise Fault("busy", "One-shot motion running", True)
            if self.stop_requested and self.active:
                raise Fault("busy", "Finishing current walk; wait for completion", True)
            self.drive, self.drive_deadline = drive, time.monotonic() + 0.35
            if active and not self.active:
                return self._start(op, self._walk())
            return {"accepted": True, "job_id": self.active}
        if op == 'game.observe':
            if not self.active or args.get('job_id') != self.active or self.jobs[self.active]['operation'] != 'game.walk':
                return {'accepted': False}  # Feedback may race job completion.
            stamp = number(args, 'stamp', None, 0, 1e12)
            if not 0 <= time.monotonic()-stamp <= .5:
                return {'accepted': False}
            ball = (number(args, 'x_m', None, -4, 4), number(args, 'y_m', None, -4, 4))
            if stamp > getattr(self, 'game_ball_stamp', 0):
                self.game_ball, self.game_ball_stamp = ball, stamp
            return {'accepted': True}
        if self.active:
            raise Fault("busy", "Motion already running; command discarded", True)
        if op == 'game.walk':
            if self.pose != 'base_stand':
                raise Fault('invalid_state', 'Game walking requires base_stand')
            kind = choice(args, 'kind', None, ('omni', 'approach', 'around', 'verify', 'kickoff'))
            options = {key: number(args, key, default, low, high) for key, default, low, high in (
                ('distance_m', 0., -4., 4.), ('course', 0., -math.pi, math.pi),
                ('heading', 0., -math.pi, math.pi), ('initial_heading', 0., -math.pi, math.pi),
                ('yaw_offset', 0., -math.pi, math.pi), ('kick_heading', 0., -math.pi, math.pi),
                ('x_m', 0., -4., 4.), ('y_m', 0., -4., 4.))}
            options['invert'] = number(args, 'invert', 1, -1, 1, True)
            if options['invert'] not in (-1, 1):
                raise Fault('invalid_argument', 'Kickoff direction must be -1 or 1')
            self.game_ball = (options['x_m'], options['y_m'])
            self.game_ball_stamp = number(args, 'stamp', 0., 0., 1e12)
            return self._start(op, self._game_walk(kind, options), {'segments': [], 'execution_uncertain': False})
        if op == "game.jump":
            direction = choice(args, "direction", None, tuple(self.jumps) + ("turn_left", "turn_right"))
            fraction = number(args, "fraction", 1., .1, 1.)
            return self._start(op, self._game_jump(direction, fraction), {"segments": [], "execution_uncertain": False})
        if op == "game.head":
            pan = number(args, 'pan', 0, -2666, 2666, True)
            tilt = number(args, 'tilt', None, -2600, 950, True)
            return self._start(op, self._head(pan, tilt))
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
            kick_type = self.parameters['motion.kick_type']
            power = number(args, "power", 80, 20 if kick_type == 'controller' else 1, 100, True)
            offset = number(args, "offset", 0, -60, 80, True)
            return self._start(op, self._kick(leg, power, offset, kick_type),
                               {'kick_type': kick_type, 'execution_uncertain': False})
        if op == "test.start":
            plan = TestPlan(self, args)
            return self._start(op, plan.run(), {"test": plan.args,
                               "manual_parameters": list(MEASUREMENTS.get(plan.name, ())),
                               "measurement_baseline": {key: plan.parameters[key]
                                   for key in MEASUREMENTS["spot_run"]} if plan.name == "spot_run" else {}})
        raise Fault("not_supported", op)

    def _start(self, op, plan, details=None):
        self.hardware.prepare_motion()
        if op == "test.start":
            self.engine = None
            self.pose = "unknown"
        if op in ('test.start', 'motion.drive', 'motion.kick') or op.startswith('game.'):
            self.fall_samples = 0
            self.fall_sequence = self.body_imu.sequence
        ident = uuid.uuid4().hex
        self.active, self.plan = ident, iter(plan)
        self.segment_active = False
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
            if job["operation"] in ("test.start", "game.walk", "game.jump") and self.stop_requested and status == "completed":
                status, reason = "cancelled", "graceful_stop"
            if "segments" in job and self.segment_active:
                job["execution_uncertain"] = True
            if job.get('kick_type') == 'controller' and status != 'completed' and self.hardware.controller_pending:
                job['execution_uncertain'] = True
            if job.get('kick_type') == 'controller' and self.stop_requested and status == 'completed':
                status, reason = 'cancelled', 'graceful_stop'
            job.update(status=status, reason=reason, pose=self.pose)
            if status == "completed" and job.get("calibration") and not job.get('saved') and self.calibration_pending != self.active:
                job["status"] = "saving"
                self.calibration_pending = self.active
                self.emit("calibration.ready", {"job_id": self.active, "values": job["calibration"]})
            else:
                if not job.get('saved') and self.calibration_pending != self.active:
                    job.pop("calibration", None)
                self.emit("job.failed" if status == "failed" else "job.completed", dict(job))
        self.active = self.plan = self.instruction = None
        self.segment_active = False
        self.stop_requested = False

    def tick(self):
        if self.motherboard_reset_pending:
            self._publish_body_telemetry()
            return
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
                    self.important_errors.clear()
                    self.queue_reset_pending = False
                    self.retry_delay = 1.0
                    self.log("INFO", "Body connected; no previous motion resumed")
                    self.emit("body.connection", self.state())
            except (Fault, OSError) as exc:
                if isinstance(exc, Fault) and exc.code == "body_busy":
                    self.log("WARNING", f"Body probe deferred: {exc}")
                    self.reconnect_at = time.monotonic() + 0.05
                else:
                    self._important_failure(exc)
                    return
        if not self.body_connected:
            self._publish_body_telemetry()
            return
        if self.queue_reset_pending:
            if now >= self.queue_reset_at:
                try:
                    self.hardware.reset()
                    self.queue_reset_pending = False
                except (Fault, OSError) as exc:
                    self.queue_reset_at = time.monotonic() + 0.05
                    if isinstance(exc, Fault) and exc.code == 'body_busy':
                        self.log('WARNING', f'Body stop deferred until direct transaction resolves: {exc}')
                    else:
                        self._important_failure(exc, reset=False)
            return
        try:
            # Read queued execution failures even when the last job has ended.
            self.hardware.check_queue()
        except (Fault, OSError) as exc:
            self._important_failure(exc)
            self._publish_body_telemetry()
            return
        self._tick_body_imu()
        self._tick_body_servos()
        self._tick_body_power()
        if not self.body_connected:
            return
        if not self.plan:
            self._tick_stabilization()
            self._publish_body_telemetry()
            return
        self.stabilizer.freeze("motion")
        try:
            self._monitor_motion_posture()
            if time.monotonic() < self.next_at:
                return
            if self.instruction is None:
                self.instruction = next(self.plan)
                self.instruction_started = time.monotonic()
            kind, *args = self.instruction
            if kind == "imu":
                after = args[0]
                ready = (self.body_imu.fresh(time.monotonic())
                         and (after is None or self.body_imu.sequence > after))
                if not ready and not self.stop_requested:
                    if time.monotonic() - self.instruction_started > 1:
                        raise Fault("imu_stale", "No fresh polled body IMU within 1 second")
                    self.next_at = time.monotonic() + 0.005
                    return
            elif kind == "drain":
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
            elif kind == 'controller_write':
                self.hardware.controller_write(*args)
            elif kind == 'controller_start':
                if not self.stop_requested:
                    self.hardware.controller_start()
            elif kind == 'controller_wait':
                witness = args[0]
                if time.monotonic() - self.instruction_started > 10:
                    raise Fault('hardware_error', 'Controller kick completion not confirmed within 10 seconds')
                try:
                    complete = witness.poll(self.hardware)
                except (Fault, OSError) as exc:
                    if isinstance(exc, Fault) and exc.code not in ('body_busy', 'hardware_error'):
                        raise
                    # Read-only telemetry is not an important motion failure.
                    self.log('WARNING', f'Controller kick status unavailable: {exc}')
                    witness.values = []
                    complete = False
                if not complete:
                    self.next_at = time.monotonic() + .02
                    return
                self.hardware.controller_completed()
            elif kind == "sleep":
                self.next_at = time.monotonic() + max(0, args[0])
            self.instruction = None
        except StopIteration:
            self._finish("completed")
        except Exception as exc:
            if isinstance(exc, Fault) and exc.code == "body_busy" and self.instruction is not None:
                self.log("WARNING", f"Body command deferred before execution: {exc}")
                self.next_at = time.monotonic() + 0.02
                return
            if isinstance(exc, OSError) or isinstance(exc, Fault) and exc.code == "hardware_error":
                self._important_failure(exc)
                return
            self.pose = "unknown"
            if self.active and self.jobs[self.active]['operation'] == 'motion.drive':
                self.drive = None
                self.drive_rearm_required = True
            self.engine = None
            self.servo_targets.clear()
            self.sent_targets.clear()
            self.target_times.clear()
            self.body_servos.invalidate("motion_fault")
            self.stabilizer.reset("motion_fault")
            # Sensor/calibration/recovery failures fail the job, not the body
            # connection. In particular a failed get-up must remain retryable.
            retryable = isinstance(exc, Fault) and exc.code in (
                "imu_stale", "imu_discontinuous", "imu_invalid", "calibration_invalid",
                "course_not_reached", "posture_uncertain", "posture_unstable", "get_up_failed", "fallen")
            self.error = None if retryable else str(exc)
            self.log("ERROR", str(exc))
            self._finish("failed", str(exc))
            try:
                self.hardware.reset()
            except Exception as reset_error:
                if isinstance(reset_error, Fault) and reset_error.code == 'body_busy':
                    self.log('WARNING', f'Body stop deferred until direct transaction resolves: {reset_error}')
                    self.queue_reset_pending = True
                    self.queue_reset_at = time.monotonic() + .05
                else:
                    self._important_failure(reset_error, reset=False)
        finally:
            self._publish_body_telemetry()

    def read_body_quaternion(self, max_age=BodyImu.MAX_AGE_S):
        return self.body_imu.sample(max_age)

    def wait_for_imu(self, after_sequence=None):
        if self.stop_requested:
            return False
        yield "imu", self.body_imu.sequence if after_sequence is None else after_sequence
        return not self.stop_requested

    def _monitor_motion_posture(self):
        from .recovery import body_position
        job = self.jobs[self.active]
        manual = job['operation'] == 'motion.drive'
        if (job.get('kick_type') == 'controller' and self.hardware.controller_pending
                and not self.body_imu.fresh(time.monotonic())):
            raise Fault('imu_stale', 'Body IMU lost during controller kick')
        if manual and job.get('posture_verified') and not self.body_imu.fresh(time.monotonic()):
            raise Fault('imu_stale', 'Body IMU lost during manual walking; release controls to restart')
        if (not (manual or job.get('kick_type') == 'controller' or job["operation"].startswith('game.') or
                 job["operation"] == "test.start" and job.get("posture_verified"))
                or self.recovery_active or self.body_imu.sequence == self.fall_sequence
                or not self.body_imu.fresh(time.monotonic())):
            return
        self.fall_sequence = self.body_imu.sequence
        try:
            position = body_position(self.body_imu.sample())
        except Fault:
            self.fall_samples = 0
            return
        self.fall_samples = self.fall_samples + 1 if position != "upright" else 0
        if self.fall_samples >= 2:
            raise Fault("fallen", f"Body fell {position}; motion cancelled")

    def _body_telemetry(self):
        now = time.monotonic()
        return {"body.imu": self.body_imu.state(now),
                "body.power": self.body_power.state(now, self.parameters["power.voltage_scale"]),
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
        # Supervisor/game consumes the same cache even with no GUI subscriber.
        if now >= self.telemetry_at:
            self.telemetry_at = now + 0.1
            self.emit("body.telemetry", self._body_telemetry())

    def _tick_body_imu(self):
        now = time.monotonic()
        # Servo deadlines take precedence. No backlog of missed polling periods.
        if now < self.body_imu.next_at or (self.instruction is not None
                and self.instruction[0] == "servo" and now >= self.next_at):
            return
        try:
            self.body_imu.next_at = now + BodyImu.PERIOD_S
            if self.hardware.telemetry_ready():
                self.body_imu.poll(self.hardware)
        except (Fault, OSError) as exc:
            self.log("WARNING", f"Body IMU polling failed: {exc}")

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
            self.body_servos.next_at = now + BodyServos.PERIOD_S
            if self.hardware.telemetry_ready():
                self.body_servos.read(self)
        except (Fault, OSError) as exc:
            self.log("WARNING", f"Body positions polling failed: {exc}")

    def _tick_body_power(self):
        now = time.monotonic()
        if (not self.body_connected or not self.power_watch
                or now < self.body_power.next_at
                or (self.plan and now >= self.next_at)):
            return
        try:
            self.body_power.next_at = now + BodyPower.PERIOD_S
            if self.hardware.telemetry_ready():
                self.body_power.read(self.hardware)
        except (Fault, OSError) as exc:
            self.log("WARNING", f"Body power polling failed: {exc}")

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
                self.log("WARNING", f"Stabilization deferred: {exc}")
                regulator.freeze("queue_busy")
            else:
                self._important_failure(exc)

    def _important_error_count(self):
        cutoff = time.monotonic() - 5
        while self.important_errors and self.important_errors[0] <= cutoff:
            self.important_errors.popleft()
        return len(self.important_errors)

    def _important_failure(self, exc, *, reset=True):
        count = min(3, getattr(exc, "count", 1))
        self._important_error_count()
        self.important_errors.extend([time.monotonic()] * count)
        self.last_important_error = str(exc)[:240]
        self.log("ERROR", f"Important body operation failed ({len(self.important_errors)}/3 in 5s): {exc}")
        if not self.body_connected or len(self.important_errors) >= 3:
            self._link_lost(exc)
            return
        # An uncertain command must never be retried. End this job and discard
        # its queued tail, but keep the worker/link usable below the threshold.
        self.pose, self.drive, self.engine = "unknown", None, None
        self.servo_targets.clear()
        self.sent_targets.clear()
        self.target_times.clear()
        self.body_servos.invalidate("uncertain_motion")
        self.stabilizer.reset("uncertain_motion")
        self._finish("failed", str(exc))
        if reset:
            try:
                self.hardware.reset()
                self.queue_reset_pending = False
            except (Fault, OSError) as reset_error:
                if isinstance(reset_error, Fault) and reset_error.code == 'body_busy':
                    self.log('WARNING', f'Body stop deferred until direct transaction resolves: {reset_error}')
                    self.queue_reset_pending = True
                    self.queue_reset_at = time.monotonic() + .05
                else:
                    self._important_failure(reset_error, reset=False)
        else:
            self.queue_reset_pending = True
            self.queue_reset_at = time.monotonic() + 0.05
        self.emit("body.connection", self.state())

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
        self.body_power.invalidate(str(exc)[:240])
        self.stabilizer.reset("link_lost")
        self.error = str(exc)[:240]
        self._finish("failed", self.error)
        transport_error = str(exc).lower()
        if not self.motherboard_reset_pending and (
                'acm' in transport_error or 'host rpc response timeout' in transport_error):
            self.motherboard_reset_pending = True
            # The supervisor stops camera/IMU, pulses GPIO, boots STM, then
            # asks this sole ACM owner to reopen. Never replay the old plan.
            self.emit('motherboard.reset_required', {'reason': self.error})
        try:
            self.hardware.reset()
        except (Fault, OSError) as reset_error:
            self.log("ERROR", f"Body queue reset during reconnect failed: {reset_error}")
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

    def _pose(self, name, *, right_first=True):
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
            engine = self._engine()
            engine.first_Leg_Is_Right_Leg = right_first
            yield from engine.walk_Initial_Pose(start_mixing=False)
            yield "drain",
        elif name == "stand":
            if self.pose != "crouch":
                raise Fault("invalid_state", "Walk final pose requires crouch")
            yield from self._engine().walk_Final_Pose()
            yield "drain",
        self.pose = name
        if name in ("stand", "base_stand"):
            self.engine = None

    def _complete_segment(self, delta, yaw):
        job = self.jobs[self.active]
        job['segments'].append(dict(index=len(job['segments']) + 1,
                                    dx_m=delta[0], dy_m=delta[1], imu_yaw_rad=yaw))
        job['progress'] = len(job['segments'])
        self.segment_active = False
        self.emit('job.progress', copy.deepcopy(job))

    def _game_walk(self, kind, options):
        from . import legacy_game_motion as legacy
        from .odometry import walk_delta
        from .recovery import body_position
        def ball():
            if 0 <= time.monotonic()-self.game_ball_stamp <= .5:
                return self.game_ball
            return None
        p, a = self.parameters, options
        recipes = {
            'omni': lambda: legacy.omni(p, a['distance_m']*1000, a['course'], a['heading']),
            'approach': lambda: legacy.approach(p, a['distance_m'], a['heading'], ball),
            'verify': lambda: legacy.verify(p, a['heading'], ball),
            'around': lambda: legacy.around(p, a['distance_m'], a['course'], a['initial_heading'], a['kick_heading'], ball),
            'kickoff': lambda: legacy.kickoff(p, a['invert'], ball),
        }
        prepared, completed, last = False, 0, None
        for order in recipes[kind]():
            if not (yield from self.wait_for_imu()):
                break
            quaternion = self.read_body_quaternion()
            if body_position(quaternion) != 'upright':
                raise Fault('fallen', 'Fall during original walking manoeuvre')
            if not prepared:
                self.segment_active = True
                yield from self._pose('crouch', right_first=order.right)
                self.segment_active = False
                prepared = True
                if not (yield from self.wait_for_imu()):
                    break
                quaternion = self.read_body_quaternion()
            yaw = quaternion_yaw(quaternion)
            field_yaw = wrap(yaw+a['yaw_offset'])
            rotation = legacy.clip(wrap(order.heading-field_yaw*order.yaw_gain), .3)
            self.segment_active = True
            if self.simulated:
                yield 'sleep', .08
            else:
                engine = self._engine()
                engine.first_Leg_Is_Right_Leg = order.right
                yield from engine.walk_Cycle(order.step, order.side, rotation*(1 if order.right else -1),
                                             order.index, order.total, half=order.half)
            yield 'drain',
            delta = walk_delta(p, order.step, order.side, first=completed == 0, right_first=order.right)
            if order.half:
                delta = tuple(value*.5 for value in delta)
            self._complete_segment(delta, yaw)
            completed += 1
            last = order
        if prepared:
            self.segment_active = True
            # Normal recipes carry the original terminal cycle number. If
            # interrupted mid-series, settle both feet before leaving the gait.
            if last and (last.half or last.index != last.total-1) and not self.simulated:
                yield from self._engine().walk_Cycle(0, 0, 0, 0, 1)
                yield 'drain',
            self.pose = 'crouch'
            yield from self._pose('stand')
            yield from self._pose('base_stand')
            self.segment_active = False

    def _game_jump(self, direction, fraction):
        from .odometry import jump_delta
        from .recovery import body_position
        if not (yield from self.wait_for_imu()):
            return
        if body_position(self.read_body_quaternion()) != 'upright':
            raise Fault('fallen', 'Game jump requires upright body')
        yaw = quaternion_yaw(self.read_body_quaternion())
        self.segment_active = True
        rows = self._jump_rows(direction, fraction)
        if direction.startswith('turn_'):
            rows.append([10] + [0] * 21)  # Same settled finish as rotation calibration.
        yield from self._slot(rows, legs_only=direction.startswith('turn_'))
        self._complete_segment(jump_delta(self.parameters, direction, fraction), yaw)
        # Absolute calibrated rows finish in the neutral stance, not manual crouch.
        self.pose = 'stand'

    def _walk(self, cycles=None, fixed=None):
        # Manual mode never gets up implicitly. No preparation is sent until
        # three distinct polling samples agree that the body is upright.
        from .recovery import stable_position
        position = yield from stable_position(self)
        if position is None:
            return
        if position != 'upright':
            raise Fault('fallen', f'Manual walking requires upright body, got {position}')
        self.jobs[self.active]['posture_verified'] = True
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
                if not (yield from self.wait_for_imu()):
                    break
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

    def _kick(self, leg, power, offset, kick_type=None):
        kick_type = kick_type or self.parameters['motion.kick_type']
        if self.active:
            self.jobs[self.active].update(kick_type=kick_type, execution_uncertain=False)
            self.emit('job.progress', dict(self.jobs[self.active]))
        self._release_stabilization()
        self.engine = None
        if kick_type == 'controller':
            from .recovery import body_position
            yield 'drain',
            yield 'imu', self.body_imu.sequence
            if self.stop_requested:
                return
            if body_position(self.body_imu.sample()) != 'upright':
                raise Fault('posture_uncertain', 'Controller kick requires fresh upright IMU')
            if self.pose != 'base_stand':
                yield from self._pose('base_stand')
                yield 'imu', self.body_imu.sequence
                if body_position(self.body_imu.sample()) != 'upright':
                    raise Fault('posture_uncertain', 'Controller kick preparation lost upright posture')
            self.pose = 'unknown'
            # Zubr owns these targets now; do not retain a pre-kick pose as feedback.
            self.servo_targets.clear()
            self.sent_targets.clear()
            self.target_times.clear()
            self.body_servos.invalidate('controller_kick')
            for address, key in KICK_PARAMETERS:
                if self.stop_requested:
                    return
                yield 'controller_write', address, {'right': int(leg == 'right'), 'power': power, 'offset': offset}[key]
            if self.stop_requested:
                return
            yield 'controller_start',
            if not self.hardware.controller_pending:
                return
            yield 'controller_wait', KickWitness()
            yield 'imu', self.body_imu.sequence
            if body_position(self.body_imu.sample()) != 'upright':
                raise Fault('posture_uncertain', 'Controller kick ended without upright posture')
        elif self.simulated:
            self.pose = 'unknown'
            yield "sleep", 0.15
        else:
            self.pose = 'unknown'
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
