"""Existing Roki library adapter: body commands through motherboard ACM."""
import math
import struct
import time
from types import SimpleNamespace
from roki_ng.wire import Fault
from roki_ng.body.controller_kick import KICK_SLOT, STOP_SLOT, KICK_PARAMETERS, KickWitness


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
