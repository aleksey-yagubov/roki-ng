"""Operator UDP endpoint, leases, logs and fixed worker lifecycle."""

import asyncio
import collections
from dataclasses import dataclass, field
import secrets
import socket
import time
import uuid

from .ipc import WorkerPeer
from .parameters import Parameters, SCHEMA, COLOUR_DEFAULTS, validate_colour_ranges
from .calibration import ROTATION_KEYS
from .platform import Bootstrap
from .wire import Fault, UDP_LIMIT, envelope, number, boolean, pack, page, udp_socket, unpack

OPS = (
    "game.start", "game.stop", "game.status",
    "localisation.start", "localisation.stop", "localisation.status",
    "camera.start", "camera.stop", "camera.status", "camera.controls.freeze",
    "camera.controls.list", "camera.controls.set", "camera.controls.save",
    "detection.list", "detection.start", "detection.stop", "detection.status",
    "session.heartbeat", "session.close", "system.status", "system.capabilities", "system.operations",
    "system.restart_stream_worker", "control.acquire", "control.release", "mode.set",
    "motion.drive", "motion.head", "motion.pose", "motion.jump", "motion.kick", "motion.slot",
    "motion.get_up", "motion.splits",
    "motion.slots", "motion.joints", "motion.stop_graceful", "motion.stop_hard", "job.status", "job.cancel",
    "test.list", "test.describe", "test.start", "test.measure", "camera.capabilities", "videostream.capabilities", "videostream.create",
    "videostream.start", "videostream.status", "videostream.update", "videostream.stop", "videostream.destroy",
    "videostream.sources", "videostream.list", "videostream.attach", "videostream.detach",
    "params.keys", "params.describe", "params.get", "params.set",
    "data.list", "data.snapshot", "data.subscribe", "data.update", "data.unsubscribe",
    "log.sources", "log.snapshot", "log.subscribe", "log.update", "log.unsubscribe",
)
READ_ONLY = {
    "game.status",
    "localisation.status",
    "camera.status",
    "camera.controls.list", "videostream.sources", "videostream.list", "videostream.status",
    "videostream.attach", "videostream.detach",
    "detection.list", "detection.status",
    "motion.slots", "motion.joints", "test.list", "test.describe", "job.status", "camera.capabilities", "videostream.capabilities",
    "params.keys", "params.describe", "params.get", "system.status", "system.capabilities",
    "system.operations", "session.heartbeat", "session.close",
}
BODY_TOPICS = ("body.imu", "body.stabilization", "body.servos")
TOPICS = ("game.state", "system.workers", "motion.state", "camera.state", "videostream.state", "detection.state", "localisation.state", *BODY_TOPICS)
LEVELS = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40, "CRITICAL": 50}
REQUEST_WINDOW = 128


@dataclass
class Session:
    id: int
    token: int
    address: tuple
    instance: str
    seen: float = field(default_factory=time.monotonic)
    high_id: int = 0
    drive_sequence: int = 0
    sequence: int = 0
    cache: dict = field(default_factory=collections.OrderedDict)
    pending: dict = field(default_factory=dict)
    data: dict = field(default_factory=dict)
    logs: dict | None = None
    closed: bool = False


class Supervisor:
    def __init__(self, config):
        self.config = config
        self.params = Parameters(config["state_dir"])
        self.boot_id = uuid.uuid4().hex
        self.sessions = {}
        self.owner = None
        self.motion_ready = True
        self.lease_epoch = 0
        self.mode = "BOOTSTRAP"
        self.workers = {}
        self.history = collections.deque(maxlen=1024)
        self.log_id = 0
        self.counters = collections.Counter()
        self.tasks = set()
        self.sock = None
        self.bootstrap = None
        self.closing = False
        self.control_lock = asyncio.Lock()
        self.parameter_lock = asyncio.Lock()
        self.capture_lock = asyncio.Lock()
        self.capture_session = None
        self.alignment_pending = collections.deque(maxlen=16)
        self.alignment_task = None
        self.head_menu = self.head_buttons = self.voice = None
        self.button_task = None
        self.body_telemetry = {}
        self.body_watch = False
        self.body_servo_watch = False
        self.body_watch_lock = asyncio.Lock()
        self.local_session = Session(secrets.randbits(63) + 1, 0, (), "head-buttons")
        from .game import Goalkeeper
        self.game = Goalkeeper(self)

    def spawn(self, coroutine):
        task = asyncio.create_task(coroutine)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)
        task.add_done_callback(self._task_done)
        return task

    def _task_done(self, task):
        if not task.cancelled() and (error := task.exception()):
            self.log("supervisor", "ERROR", str(error))

    def log(self, source, level, message):
        message = str(message)
        raw = message.encode("utf-8", errors="replace")
        if len(raw) > 200:
            # Preserve long native diagnostics as consecutive bounded records.
            while raw:
                chunk = raw[:200].decode("utf-8", errors="ignore")
                self.log(source, level, chunk)
                raw = raw[len(chunk.encode("utf-8")):]
            return
        self.log_id += 1
        record = {"record_sequence": self.log_id, "monotonic_ns": time.monotonic_ns(),
                  "source": source[:64], "level": level,
                  "message": message}
        self.history.append(record)
        if self.params.values["logging.stdout_enabled"]:
            print(f"[{level}] {source}: {message}", flush=True)

    async def start(self):
        self.sock = udp_socket()
        self.sock.bind((self.config.get("host", "0.0.0.0"), self.config.get("port", 8093)))
        self.spawn(self._receive())
        self.spawn(self._maintenance())
        self.log("supervisor", "INFO", f"UDP control: {self.sock.getsockname()}")
        try:
            if not self.config.get("simulate") and not self.config.get("skip_bootstrap"):
                self.bootstrap = Bootstrap(self.config)
                await asyncio.to_thread(self.bootstrap.start)
            for role in ("motherboard", "stream", "camera", "detection", "localisation"):
                worker = WorkerPeer(role, self.config | {"parameters": dict(self.params.values)}, self.worker_event, self.log)
                self.workers[role] = worker
                await worker.start()
            if not all(worker.alive for worker in self.workers.values()):
                raise RuntimeError("Worker failed during startup")
            self.mode = "IDLE"
            self.log("supervisor", "INFO", "Manual runtime ready; no camera pipeline started")
        except Exception as exc:
            self.mode = "FAULT"
            self.log("supervisor", "ERROR", f"Startup failed: {exc}")
        if self.mode == "IDLE" and not self.config.get("simulate") and not self.config.get("no_head_menu"):
            await self._start_head_menu()

    async def _start_head_menu(self):
        from .head_menu import HeadButtons, HeadMenu, Voice

        log = lambda level, text: self.log("head-menu", level, text)
        self.voice = Voice(log)
        self.head_menu = HeadMenu(self._head_command, self._head_release, self.voice.say, log)
        self.head_buttons = HeadButtons(self.head_menu, log)
        try:
            self.head_buttons.open()
        except (ImportError, OSError) as exc:
            log("ERROR", f"Menu unavailable: {exc}")
            await self.voice.close()
            self.head_menu = self.head_buttons = self.voice = None
            return
        self.button_task = self.spawn(self.head_buttons.run())
        self.voice.say("Ready")

    async def _head_command(self, op, body):
        if op in ("test.start", "game.start"):
            await self.dispatch(self.local_session, "control.acquire", {})
            try:
                await self.dispatch(self.local_session, "mode.set", {
                    "mode": "MANUAL", "lease_epoch": self.lease_epoch})
                return await self.dispatch(self.local_session, op, body | {"lease_epoch": self.lease_epoch})
            except Exception:
                await self._head_release()
                raise
        return await self.dispatch(self.local_session, op, body | {"lease_epoch": self.lease_epoch})

    async def _head_release(self):
        await self._release(self.local_session)

    def worker_event(self, role, op, body):
        if not self.closing and ((role == 'stream' and op in ('videostream.failed', 'videostream.stopped', 'worker.fault'))
                                 or (role == 'localisation' and op in ('localisation.fault', 'worker.fault'))):
            self.spawn(self._video_fault(role, op))
        if role == "motherboard" and op == "body.telemetry":
            self.body_telemetry = body
            return  # Only requested datastreams, never unsolicited operator events.
        if self.head_menu and role == "motherboard" and not self.closing:
            if op in ("job.completed", "job.cancelled", "job.failed"):
                self.spawn(self.head_menu._finish(body))
            elif op == "worker.fault":
                self.spawn(self.head_menu.worker_failed())
        if role == "motherboard" and op == "imu.alignment":
            if self.capture_session is not None and not self.closing:
                # Startup needs recent timestamp pairs, not every historical edge.
                # Keep receiving while a slow media start holds capture_lock.
                self.alignment_pending.append((self.capture_session, body))
                if self.alignment_task is None or self.alignment_task.done():
                    self.alignment_task = self.spawn(self._forward_alignment())
            return  # Internal startup metadata, never an operator datastream.
        if op in ("camera.fault", "imu.fault") and not self.closing:
            self.spawn(self._capture_fault(self.capture_session))
        if role == "motherboard" and op == "calibration.ready":
            self.spawn(self._save_calibration(body))
            return
        if op == "worker.fault" and not self.closing:
            if role == "localisation":
                self.log(role, "ERROR", body.get("error", "Localisation worker failed"))
                return  # Diagnostic failure must not revoke manual control.
            self.mode = "FAULT"
            self.log(role, "ERROR", body.get("error", "worker failed"))
            if role in ("camera", "motherboard"):
                self.spawn(self._capture_fault(self.capture_session))
            if self.owner and self.motion_ready:
                owner = self.local_session if self.owner == self.local_session.id else self.sessions.get(self.owner)
                if owner:
                    self.spawn(self._release(owner))
        for session in list(self.sessions.values()):
            if op.startswith("job.") and session.id != self.owner:
                continue
            self.send(session, "event", op, body)

    def send(self, session, kind, op, body, ident=0):
        if session.closed:
            return None
        if kind in ("event", "sample"):
            session.sequence += 1
        message = envelope(kind, op, body, session=session.id, token=session.token,
                           id=ident, sequence=session.sequence if kind in ("event", "sample") else 0)
        try:
            data = pack(message)
        except Fault:
            self.counters["oversize"] += 1
            if kind != "response":
                return None
            message["body"] = {"error": Fault("too_large", "Response exceeds budget; reduce page size").as_dict()}
            data = pack(message)
        self._send_raw(data, session.address)
        return data

    def _send_raw(self, data, address):
        try:
            self.sock.sendto(data, address)
        except OSError:
            self.counters["send_errors"] += 1

    async def _receive(self):
        loop = asyncio.get_running_loop()
        while True:
            raw, address = await loop.sock_recvfrom(self.sock, UDP_LIMIT + 1)
            try:
                message = unpack(raw)
                if message.get("v") != 1:
                    raise Fault("bad_version", "v must be 1")
                kind = message.get("kind")
                if kind == "hello":
                    self._hello(message, address)
                    continue
                session = self.sessions.get(message.get("session"))
                if (session is None or session.closed or session.address != address
                        or session.token != message.get("token")):
                    self.counters["invalid_session"] += 1
                    continue
                if not isinstance(message.get("body"), dict) or not isinstance(message.get("op"), str):
                    raise Fault("bad_message", "Expected op string and body map")
                session.seen = time.monotonic()
                if kind == "sample" and message["op"] == "motion.drive":
                    sequence = number(message, "sequence", 0, 1, 2**64 - 1, True)
                    if sequence <= session.drive_sequence:
                        continue
                    session.drive_sequence = sequence
                    # At most one forwarded drive in flight; new packets replace it.
                    session.latest_drive = message
                    if not getattr(session, "drive_task", None) or session.drive_task.done():
                        session.drive_task = self.spawn(self._drive(session))
                elif kind == "request":
                    ident = number(message, "id", 0, 1, 2**64 - 1, True)
                    if ident in session.cache:
                        old_op, old_body, data = session.cache[ident]
                        if (old_op, old_body) == (message["op"], message["body"]):
                            self._send_raw(data, address)
                        else:
                            self.send(session, "response", message["op"], {"error": Fault("id_conflict", "ID reused with different request").as_dict()}, ident)
                        continue
                    if ident in session.pending:
                        original = session.pending[ident]
                        if (original["op"], original["body"]) != (message["op"], message["body"]):
                            self.send(session, "response", message["op"], {"error": Fault("id_conflict", "ID reused with different request").as_dict()}, ident)
                        continue
                    if ident <= session.high_id - REQUEST_WINDOW:
                        self.send(session, "response", message["op"], {"error": Fault("stale_request", "ID is older than retained request window").as_dict()}, ident)
                        continue
                    session.high_id = max(session.high_id, ident)
                    # Evict by ID, not completion order: every accepted ID within
                    # the window must remain either pending or cached.
                    for old_id in list(session.cache):
                        if old_id <= session.high_id - REQUEST_WINDOW:
                            del session.cache[old_id]
                    if len(session.pending) >= 8 or len(self.tasks) >= 64:
                        # Cache busy as well: retries must not execute it later.
                        data = self.send(session, "response", message["op"], {"error": Fault("busy", "Too many requests", True).as_dict()}, ident)
                        self._cache(session, message, data)
                        continue
                    session.pending[ident] = message
                    self.spawn(self._request(session, message))
                else:
                    raise Fault("bad_message", "Unsupported message kind")
            except (Fault, TypeError, ValueError):
                self.counters["bad_packets"] += 1

    def _hello(self, message, address):
        body = message.get("body", {})
        if not isinstance(body, dict) or body.get("versions") != [1]:
            raise Fault("bad_version", "hello versions must be [1]")
        instance = body.get("client_instance")
        if not isinstance(instance, str) or not 1 <= len(instance) <= 64:
            raise Fault("bad_message", "Invalid client_instance")
        ident = number(message, "id", 0, 1, 2**64 - 1, True)
        session = next((s for s in self.sessions.values() if s.address == address and s.instance == instance), None)
        if session is None:
            if len(self.sessions) >= 4:
                return
            session = Session(secrets.randbits(63) + 1, secrets.randbits(63) + 1, address, instance)
            self.sessions[session.id] = session
            self.log("supervisor", "INFO", f"Operator connected: {address[0]}:{address[1]}")
        session.seen = time.monotonic()
        self.send(session, "welcome", "hello", {
            "robot_id": self.config.get("robot_id", socket.gethostname()), "boot_id": self.boot_id,
            "heartbeat_ms": 500, "session_timeout_ms": 2000, "drive_timeout_ms": 350,
            "state": self.mode, "capabilities_revision": "manual-1", "max_datagram": UDP_LIMIT,
        }, ident)

    def _cache(self, session, message, data):
        if data and message["id"] > session.high_id - REQUEST_WINDOW:
            session.cache[message["id"]] = (message["op"], message["body"], data)

    async def _request(self, session, message):
        try:
            result = await self.dispatch(session, message["op"], message["body"])
            body = {"result": result}
        except Fault as exc:
            body = {"error": exc.as_dict()}
        except Exception as exc:
            self.log("supervisor", "ERROR", f"{message['op']}: {exc}")
            body = {"error": Fault("internal_error", str(exc)).as_dict()}
        data = self.send(session, "response", message["op"], body, message["id"])
        self._cache(session, message, data)
        session.pending.pop(message["id"], None)
        if message["op"] == "session.close":
            await self._expire(session)

    def require_control(self, session, body):
        if session.closed or self.owner != session.id or body.get("lease_epoch") != self.lease_epoch:
            raise Fault("not_owner", "A valid control lease is required")

    def require_motion_ready(self):
        if not self.motion_ready:
            raise Fault("stop_unconfirmed", "Retry forced control acquisition to confirm the body queue reset", True)

    def require_game_idle(self):
        if self.game.info['running'] or self.game.starting:
            raise Fault('busy', 'Stop goalkeeper before changing motion or capture configuration')

    async def _take_control(self, session):
        """Called under control_lock. Revoke first, then reset via urgent IPC."""
        await self.game.stop('control_takeover', hard=True)
        previous = self.owner
        old = self.local_session if previous == self.local_session.id else self.sessions.get(previous)
        self.owner = session.id
        self.lease_epoch += 1
        self.motion_ready = False
        if old:
            old.latest_drive = None
            if old is self.local_session and self.head_menu:
                self.head_menu.taken_over()
            elif old.id != session.id:
                self.send(old, "event", "control.revoked", {"reason": "operator_takeover"})
        self.log("supervisor", "WARNING", f"Forced control takeover: {previous} -> {session.id}")
        error = None
        worker = self.workers.get("motherboard")
        try:
            if not worker or not worker.alive:
                raise Fault("worker_unavailable", "motherboard")
            await worker.call("control.takeover", urgent=True)
            self.motion_ready = True
        except Fault as exc:
            error = exc.as_dict()
            self.log("supervisor", "ERROR", f"Takeover stop unconfirmed: {exc}")
        if session.closed:
            self.owner = None
            if worker and worker.alive:
                try:
                    await worker.call("control.release", urgent=True)
                except Fault:
                    self.motion_ready = False
            raise Fault("session_expired", "Session closed during acquisition")
        return {"lease_epoch": self.lease_epoch, "motion_ready": self.motion_ready,
                "stop_confirmed": self.motion_ready, "stop_error": error}

    async def _drive(self, session):
        while getattr(session, "latest_drive", None):
            message, session.latest_drive = session.latest_drive, None
            try:
                self.require_control(session, message["body"])
                self.require_motion_ready()
                if self.mode != "MANUAL":
                    raise Fault("invalid_state", "Select MANUAL mode")
                if self.parameter_lock.locked():
                    raise Fault("busy", "Parameters are being saved", True)
                async with self.parameter_lock:
                    await self.workers["motherboard"].call("motion.drive", message["body"])
            except Fault as exc:
                # Repeated drive failures are rate-limited, not ACKed at driving frequency.
                if time.monotonic() - getattr(session, "last_drive_error", 0) > 1:
                    session.last_drive_error = time.monotonic()
                    self.send(session, "event", "motion.rejected", {"error": exc.as_dict()})

    async def dispatch(self, session, op, body, *, _from_game=False):
        if op == "game.status":
            return self.game.state()
        if op in ("game.start", "game.stop"):
            self.require_control(session, body)
            if op == "game.stop":
                return await self.game.stop()
            generation = self.game.generation
            async with self.parameter_lock:
                self.require_control(session, body)
                if generation != self.game.generation:
                    raise Fault('cancelled', 'Game start discarded at stop barrier')
                return await self.game.start(session, body)
        if op in ('motion.stop_hard', 'motion.stop_graceful') or (op == 'mode.set' and body.get('mode') == 'IDLE'):
            self.require_control(session, body)
            await self.game.stop('operator_stop', hard=op == 'motion.stop_hard')
        elif (self.game.info['running'] or self.game.starting) and not _from_game:
            if (op.startswith(('motion.', 'test.', 'camera.', 'detection.', 'localisation.', 'params.'))
                  and op not in READ_ONLY) or op in ('mode.set', 'system.restart_stream_worker', 'job.cancel'):
                raise Fault('busy', 'Stop goalkeeper before changing motion or capture configuration')
        if op not in OPS:
            raise Fault("not_supported", op)
        if op == "session.heartbeat":
            return {"state": self.mode}
        if op == "session.close":
            return {"closing": True}
        if op == "system.operations":
            return page(list(OPS), body, 12)
        if op == "system.capabilities":
            return {"revision": "runtime-1", "modes": ["IDLE", "MANUAL"],
                    "body": ["software_slots", "walk", "jump", "kick", "head", "tests"],
                    "video_backends": ["direct-gst", "runtime", "localisation"], "data_topics": list(TOPICS),
                    "capture_backends": ["libcamera-iceoryx2"],
                    "detectors": ["colour_blobs"],
                    "localisation": {"mode": "diagnostic_only", "motion_pose": False},
                    "game": {"strategies": ["FIRA_penalty_Goalkeeper"], "observe_only_default": True},
                    "future": ["osd", "servo-parameters"],
                    "hardware_slots": False, "simulated": bool(self.config.get("simulate"))}
        if op == "system.status":
            return {"state": self.mode, "owner": self.owner, "boot_id": self.boot_id,
                    "motion_ready": self.motion_ready,
                    "workers": {k: {"alive": w.alive, "state": w.state.get("state")} for k, w in self.workers.items()},
                    "counters": dict(self.counters)}
        if op == "control.acquire":
            force = boolean(body, "force", False)
            async with self.control_lock:
                if session.closed:
                    raise Fault("session_expired", "Session closed")
                if force:
                    return await self._take_control(session)
                body_worker = self.workers.get("motherboard")
                if self.mode == "BOOTSTRAP" or not body_worker or not body_worker.alive:
                    raise Fault("not_ready", self.mode)
                if self.owner and self.owner != session.id:
                    raise Fault("busy", "Another operator owns control")
                if self.owner is None:
                    self.require_motion_ready()
                    self.lease_epoch += 1
                    await self.workers["motherboard"].call("control.acquire", urgent=True)
                    if session.closed:
                        await self.workers["motherboard"].call("control.release", urgent=True)
                        raise Fault("session_expired", "Session closed during acquisition")
                    self.owner = session.id
                return {"lease_epoch": self.lease_epoch, "motion_ready": self.motion_ready}
        if op == "control.release":
            self.require_control(session, body)
            await self._release(session)
            return {"released": True}
        if op.startswith("data."):
            return await self._data(session, op, body)
        if op.startswith("log."):
            return self._logs(session, op, body)
        if op == "params.keys":
            prefix = body.get("prefix", "")
            if not isinstance(prefix, str):
                raise Fault("invalid_argument", "prefix must be a string")
            return page(sorted(k for k in SCHEMA if k.startswith(prefix)), body)
        if op == "params.get" and "keys" in body:
            keys = body["keys"]
            if ("key" in body or not isinstance(keys, list) or not 1 <= len(keys) <= 16
                    or any(not isinstance(k, str) for k in keys) or len(set(keys)) != len(keys)):
                raise Fault("invalid_argument", "Provide 1..16 distinct keys, without key")
            async with self.parameter_lock:
                for key in keys:
                    self.params.describe(key)
                return {"values": {key: self.params.values[key] for key in keys}}
        if op in ("params.get", "params.describe"):
            key = body.get("key")
            meta = self.params.describe(key)
            return meta if op.endswith("describe") else {"key": key, "value": self.params.values[key]}
        if op not in READ_ONLY:
            self.require_control(session, body)
        if op == 'camera.controls.list':
            return await self.workers['camera'].call(op, body)
        if op in ('camera.controls.set', 'camera.controls.save'):
            from .camera import CONTROL_KEYS, WB_KEYS
            values = body.get('values')
            if not isinstance(values, dict) or not values or set(values) - (set(CONTROL_KEYS) | set(WB_KEYS)):
                raise Fault('invalid_argument', 'Provide camera control values using camera.* keys')
            if op.endswith('.save'):
                return {'values': await self._set_parameters(values), 'saved': True}
            async with self.parameter_lock:
                self.require_game_idle()
                values = {key:self.params.validate(key,value) for key,value in values.items()}
                self.require_control(session, body)
                await self.workers['camera'].call('params.apply', values)
                return {'values': values, 'saved': False}
        if op == "params.set":
            if "values" in body:
                values, expected = body["values"], body.get("expected_values")
                if any(k in body for k in ("key", "value", "expected_value")):
                    raise Fault("invalid_argument", "Do not mix single and batch formats")
                if not isinstance(values, dict) or not 1 <= len(values) <= 16:
                    raise Fault("invalid_argument", "Provide 1..16 scalar parameters")
                if any(k not in SCHEMA or SCHEMA[k][0] == "object" for k in values):
                    raise Fault("invalid_argument", "Batch supports known scalar parameters only")
                if expected is not None and (not isinstance(expected, dict) or set(expected) != set(values)):
                    raise Fault("invalid_argument", "expected_values must cover exactly values")
                return {"values": await self._set_parameters(values, expected=expected)}
            if "expected_values" in body:
                raise Fault("invalid_argument", "expected_values requires values")
            key, value = body.get("key"), body.get("value")
            expected = {key: body['expected_value']} if 'expected_value' in body else None
            values = await self._set_parameters({key: value}, expected=expected)
            return {"key": key, "value": values[key], "apply": SCHEMA[key][4]}
        if op == 'camera.controls.freeze':
            from .wire import choice
            group=choice(body,'group','all',('all','exposure','white_balance'))
            async with self.capture_lock:
                self.require_game_idle()
                self.require_control(session,body)
                state=await self.workers['camera'].call('camera.status')
                measured=state.get('measured_controls');age=state.get('measured_age_ms')
                if not state.get('running') or not measured or age is None or age>1000:
                    raise Fault('not_ready','Fresh runtime camera metadata required')
                values={}
                if group in ('all','exposure'):
                    values.update({'camera.ae_enabled':False,'camera.exposure_us':measured.get('exposure_us'),
                                   'camera.analogue_gain':measured.get('gain')})
                if group in ('all','white_balance'):
                    gains=measured.get('colour_gains')
                    if not isinstance(gains,(list,tuple)) or len(gains)!=2:
                        raise Fault('not_ready','ColourGains metadata unavailable')
                    values.update({'camera.awb_enabled':False,'camera.white_balance.red_gain':gains[0],
                                   'camera.white_balance.blue_gain':gains[1]})
                async with self.parameter_lock:
                    self.require_game_idle()
                    values = {key:self.params.validate(key,value) for key,value in values.items()}
                    await self.workers['camera'].call('params.apply', values)
                return {'values':values,'saved':False,'source_sequence':measured['sequence']}
        if op == "test.measure":
            job = await self.workers["motherboard"].call("job.status", {"job_id": body.get("job_id")})
            if job["operation"] != "test.start" or job["status"] != "completed":
                raise Fault("invalid_state", "Measurements require a successfully completed test")
            values = body.get("values")
            expected = set(job.get("manual_parameters", ()))
            if not expected or not isinstance(values, dict) or set(values) != expected:
                raise Fault("invalid_argument", "Provide exactly the job's manual_parameters")
            return {"job_id": job["job_id"], "saved": await self._set_parameters(values)}
        if op == "mode.set":
            if body.get("mode") not in ("IDLE", "MANUAL"):
                raise Fault("not_supported", "Only IDLE and MANUAL are implemented")
            if self.mode == "FAULT":
                raise Fault("invalid_state", "Resolve worker fault first")
            if body["mode"] == "IDLE":
                await self.workers["motherboard"].call("motion.stop_hard", urgent=True)
            self.mode = body["mode"]
            return {"state": self.mode}
        if op == "system.restart_stream_worker":
            old_worker = self.workers["stream"]
            # An intentional exit is not a loss-of-control fault.
            old_worker.event = lambda *_: None
            await old_worker.close()
            async with self.capture_lock:
                await self._sync_video_sources(force_off=True)
            worker = WorkerPeer("stream", self.config, self.worker_event, self.log)
            self.workers["stream"] = worker
            await worker.start()
            if self.workers["motherboard"].alive:
                self.mode = "IDLE"
            return {"restarted": "stream"}
        if op.startswith(("motion.", "test.", "job.")):
            if op == "motion.drive":
                raise Fault("bad_message", "Use kind=sample for motion.drive")
            if op not in READ_ONLY and op not in ("motion.stop_hard", "motion.stop_graceful", "job.cancel") and self.mode != "MANUAL":
                raise Fault("invalid_state", "Select MANUAL mode")
            urgent = op in ("motion.stop_hard", "motion.stop_graceful", "job.cancel")
            if urgent or op in READ_ONLY:
                return await self.workers["motherboard"].call(op, body, urgent=urgent)
            self.require_motion_ready()
            async with self.parameter_lock:
                self.require_control(session, body)
                self.require_motion_ready()
                if self.mode != "MANUAL":
                    raise Fault("invalid_state", "Select MANUAL mode")
                return await self.workers["motherboard"].call(op, body)
        if op == "camera.status":
            return await self.workers["camera"].call(op)
        if op == "localisation.status":
            return await self.workers["localisation"].call(op, body)
        if op in ("localisation.start", "localisation.stop"):
            async with self.capture_lock:
                self.require_game_idle()
                self.require_control(session, body)
                if op == "localisation.stop":
                    await self.workers["stream"].call("videostream.stop_localisation")
                    return await self.workers["localisation"].call(op)
                if self.mode != "MANUAL":
                    raise Fault("invalid_state", "Select MANUAL mode")
                camera = await self.workers["camera"].call("camera.status")
                sync = camera.get("imu_sync", {})
                if not camera.get("running") or sync.get("state") != "synced":
                    raise Fault("not_ready", "A synchronized camera is required")
                return await self.workers["localisation"].call(op, {
                    "prior": body.get("prior"), "unicam_minus_stm": sync["unicam_minus_stm"],
                    "capture_id": uuid.uuid4().hex,
                    "parameters": dict(self.params.values)}, timeout=5)
        if op == "detection.list":
            return {"detector": "colour_blobs", "profiles": list(COLOUR_DEFAULTS),
                    "max_blobs": 4, "coordinates": "image_pixels", "classifies_ball": False}
        if op == "detection.status":
            return await self.workers["detection"].call(op)
        if op in ("detection.start", "detection.stop"):
            async with self.capture_lock:
                self.require_game_idle()
                self.require_control(session, body)
                if op == "detection.start":
                    if self.mode != "MANUAL":
                        raise Fault("invalid_state", "Select MANUAL mode")
                    if not (await self.workers["camera"].call("camera.status"))["running"]:
                        raise Fault("not_ready", "Start runtime camera before detection")
                return await self.workers["detection"].call(op, body)
        if op in ("camera.start", "camera.stop"):
            self.require_control(session, body)
            if 'with_imu' in body:
                raise Fault('invalid_argument', 'Runtime capture always requires IMU; use direct-gst for video without IMU')
            if set(body) - {'frame_duration_us', 'exposure_us', 'gain', 'lease_epoch'}:
                raise Fault('invalid_argument', 'Unknown camera start setting; geometry is fixed')
            async with self.capture_lock:
                if not _from_game:
                    self.require_game_idle()
                self.require_control(session, body)
                if op == "camera.stop":
                    return await self._stop_capture()
                if self.mode != "MANUAL" and not (_from_game and self.mode == 'GAME'):
                    raise Fault("invalid_state", "Select MANUAL mode")
                stream = await self.workers["stream"].call("state")
                if any(s["source"]=="direct-gst" for s in stream.get("active_streams",[])):
                    raise Fault("busy", "Stop direct-gst before starting capture")
                camera=await self.workers["camera"].call("camera.status")
                if camera.get('running') and camera.get('imu_sync',{}).get('state') in ('aligning','matched','synced'):
                    duration = number(body, 'frame_duration_us', camera['frame_duration_us'], 8333, 100000, True)
                    controls = camera.get('requested_controls', {})
                    requested = {'frame_duration_us': duration}
                    current = {'frame_duration_us': camera['frame_duration_us']}
                    for key, control, low, high, integer in (
                            ('exposure_us', 'exposure_us', 1, duration, True),
                            ('gain', 'gain', 1.0, 16.0, False)):
                        if key in body:
                            requested[key] = number(body, key, None, low, high, integer)
                            current[key] = controls.get(control)
                    if requested != current:
                        raise Fault('restart_required', 'Camera settings differ; stop capture explicitly before reconfiguration')
                    return camera
                if camera["prepared"]:
                    raise Fault("busy", "Camera already prepared without requested IMU; stop capture explicitly to reconfigure")
                try:
                    result = await self.workers["camera"].call("camera.prepare", body, timeout=10)
                    self.capture_session = object()
                    self.alignment_pending.clear()
                    imu = await self.workers["motherboard"].call("imu.start", {
                        "frame_duration_us": result["frame_duration_us"]})
                    return await self.workers["camera"].call("camera.start", {
                        "clock": imu["clock"]}, timeout=5)
                except Exception:
                    await self._stop_capture()
                    raise
        if op == 'camera.capabilities':
            return await self.workers['camera'].call(op)
        if op == 'videostream.sources':
            return await self._video_sources(body)
        if op.startswith("videostream."):
            async with self.capture_lock:
                if session.closed:
                    raise Fault('session_expired', 'Session closed')
                if op not in READ_ONLY:
                    self.require_control(session, body)
                args = body | {'session_id': session.id}
                if op == 'videostream.create':
                    args = body
                if op in ('videostream.start', 'videostream.attach'):
                    if set(body) - {'stream_id', 'rtp_port', 'lease_epoch'}:
                        raise Fault('invalid_argument', 'Provide stream_id and rtp_port')
                    number(body, 'rtp_port', None, 1024, 65535, True)
                    args |= {'host': session.address[0]}
                if op == "videostream.start":
                    camera = await self.workers["camera"].call("camera.status")
                    video = await self.workers["stream"].call("videostream.status", body)
                    runtime = video["spec"]["source"] in ("runtime", "localisation")
                    if camera["prepared"] and not runtime:
                        raise Fault("busy", "Stop runtime capture before direct-gst")
                    if video["spec"]["source"] == "localisation":
                        state=await self.workers["localisation"].call('localisation.status')
                        if not state['running']:raise Fault('not_ready','Start localisation before diagnostic video')
                    if runtime:
                        if not camera["running"]:
                            raise Fault("not_ready", "Start runtime camera before video")
                        args |= {"source_frame_duration_us": camera["frame_duration_us"]}
                try:
                    if op not in READ_ONLY:
                        self.require_control(session, body)
                    if op == 'videostream.start' and video['spec']['source'] == 'localisation':
                        await self.workers['localisation'].call('source.start', {
                            'source': 'localisation', 'parameters': video['spec']['parameters']})
                        self.require_control(session, body)
                    result = await self.workers["stream"].call(op, args)
                    if session.closed:
                        await self.workers['stream'].call('videostream.detach_session', {'session_id':session.id})
                        raise Fault('session_expired', 'Session closed during stream operation')
                    return result
                finally:
                    if op not in ('videostream.list', 'videostream.status', 'videostream.capabilities'):
                        await self._sync_video_sources()
        raise Fault("not_supported", op)

    async def _sync_video_sources(self, force_off=False):
        worker = self.workers.get('localisation')
        if worker is None:
            return
        try:
            state = {} if force_off else await self.workers['stream'].call('state')
            wanted = any(s['source'] == 'localisation' for s in state.get('active_streams', []))
            if not wanted:
                await worker.call('source.stop', {'source':'localisation'})
        except Fault as exc:
            self.log('supervisor', 'WARNING', f'Video output cleanup: {exc}')

    async def _video_fault(self, role, op):
        async with self.capture_lock:
            if role == 'localisation':
                await self.workers['stream'].call('videostream.stop_localisation')
            await self._sync_video_sources(force_off=role == 'stream' and op == 'worker.fault')

    async def _video_sources(self, args):
        from .video_sources import SOURCES
        items = []
        stream = self.workers.get('stream')
        try:
            stream_state = await stream.call('state') if stream else {}
        except Fault:
            stream_state = {}
        camera_worker = self.workers.get('camera')
        try:
            camera = await camera_worker.call('camera.status') if camera_worker else {}
        except Fault:
            camera = {}
        for ident, fallback in SOURCES.items():
            item = dict(fallback)
            worker = self.workers.get(item['worker'])
            item.update(available=False, reason='worker_unavailable', output=None)
            if worker is not None and getattr(worker, 'alive', True):
                try:
                    declared = await worker.call('source.list')
                    item.update(next(value for value in declared['items'] if value['id'] == ident))
                    if ident == 'direct-gst':
                        active = stream_state.get('active_streams', [])
                        busy = camera.get('prepared', False) or bool(active)
                        item.update(available=bool(camera) and not busy,
                                    reason='camera_busy' if busy else (None if camera else 'camera_state_unknown'),
                                    output={'publishing':any(s['source']=='direct-gst' and s['state']=='running' for s in active)})
                    elif ident == 'runtime':
                        item.update(available=bool(camera.get('running')), reason=None if camera.get('running') else 'camera_stopped',
                                    output={'publishing':bool(camera.get('running') and camera.get('frames')), 'frames':camera.get('frames',0)})
                    else:
                        state = await worker.call('localisation.status')
                        output = await worker.call('source.status', {'source':ident})
                        available = bool(camera.get('running') and state.get('running') and not state.get('error') and not output.get('error'))
                        item.update(available=available, reason=None if available else ('source_fault' if output.get('error') else 'localisation_not_ready'), output=output)
                except (Fault, KeyError, StopIteration):
                    item.update(available=False, reason='worker_unavailable')
            item['stream_settings'] = {'codecs':['h264','jpeg'], 'max_fps':[1,120],
                'max_size':[1600,1300] if ident == 'direct-gst' else [800,650],
                'jpeg_alignment':8, 'live_update':[] if ident == 'direct-gst' else ['max_fps']}
            items.append(item)
        return page(items, args, 1)

    async def _stop_capture(self):
        self.capture_session = None
        self.alignment_pending.clear()
        if "localisation" in self.workers:
            try:
                await self.workers["localisation"].call("localisation.stop")
            except Fault as exc:
                self.log("supervisor", "WARNING", f"Localisation stop: {exc}")
        try:
            await self.workers["detection"].call("detection.stop")
        except Fault as exc:
            self.log("supervisor", "WARNING", f"Detection stop: {exc}")
        try:
            await self.workers["stream"].call("videostream.stop_runtime")
        except Fault as exc:
            self.log("supervisor", "WARNING", f"Runtime video stop: {exc}")
        try:
            return await self.workers["camera"].call("camera.stop", timeout=5)
        finally:
            await self.workers["motherboard"].call("imu.stop")

    async def _forward_alignment(self):
        while self.alignment_pending:
            session, body = self.alignment_pending.popleft()
            async with self.capture_lock:
                if session is not self.capture_session:
                    continue
                try:
                    state = await self.workers["camera"].call("camera.alignment", body)
                    if state["state"] == "matched":
                        await self.workers["motherboard"].call("imu.normal")
                        state = await self.workers["camera"].call("camera.sync_confirm")
                        self.alignment_pending.clear()
                        self.worker_event("camera", "camera.synchronized", state)
                except Exception as exc:
                    self.log("supervisor", "ERROR", f"Camera alignment failed: {exc}")
                    await self._stop_capture()

    async def _capture_fault(self, session):
        async with self.capture_lock:
            if session is not None and session is self.capture_session:
                await self._stop_capture()

    async def _set_parameters(self, values, expected=None):
        async with self.parameter_lock:
            self.require_game_idle()
            if expected is not None and any(self.params.values.get(k)!=v for k,v in expected.items()):
                raise Fault('conflict','Parameter changed; reload before saving')
            values = {k: self.params.validate(k, v) for k, v in values.items()}
            validate_colour_ranges(self.params.values | values)
            applied = []
            try:
                for role, mode in (("motherboard", "next_job"), ("detection", "next_frame"),
                                   ("localisation", "next_frame"),
                                   ("camera", "next_request")):
                    update = {k: v for k, v in values.items() if SCHEMA[k][4] == mode}
                    if role=='localisation' and role not in self.workers:continue
                    if update:
                        previous = {k: self.params.values[k] for k in update}
                        try:
                            response=await self.workers[role].call("params.apply", update)
                        except Fault as exc:
                            if role != 'localisation' or exc.code != 'worker_unavailable':
                                raise
                            # Localisation receives the complete parameter snapshot on start.
                            self.log('supervisor', 'WARNING', 'Localisation unavailable; parameters will apply on its next start')
                            continue
                        # Camera.start may override persistent manual values for
                        # this capture. Roll back the actual controls, not defaults.
                        if role=='camera' and isinstance(response,dict) and set(response.get('previous',{}))==set(update):
                            previous=response['previous']
                        applied.append((role, previous))
                return await asyncio.to_thread(self.params.set_many, values)
            except Exception:
                for role, previous in reversed(applied):
                    await self.workers[role].call("params.apply", previous)
                raise

    async def _save_calibration(self, result):
        worker = self.workers["motherboard"]
        ident = result["job_id"]
        error = None
        async with self.parameter_lock:
            try:
                job = await worker.call("job.status", {"job_id": ident})
                values = result["values"]
                if (job["status"] != "saving" or job.get("calibration") != values
                        or set(values) != set(ROTATION_KEYS)):
                    raise Fault("calibration_invalid", "Unexpected calibration result")
                await asyncio.to_thread(self.params.set_many, values)
                self.log("supervisor", "INFO", f"Calibration {ident} saved: {values}")
            except Exception as exc:
                error = str(exc)
                self.log("supervisor", "ERROR", f"Calibration {ident} not saved: {error}")
            await worker.call("calibration.resolve", {"job_id": ident, "error": error})

    def _snapshot(self, topic):
        if topic == 'game.state':
            return self.game.state()
        if topic in BODY_TOPICS:
            return dict(self.body_telemetry.get(topic, {}))
        if topic == "system.workers":
            return {k: {"alive": w.alive, "state": w.state.get("state")} for k, w in self.workers.items()}
        role = {"motion.state": "motherboard", "camera.state": "camera", "videostream.state": "stream", "detection.state": "detection", "localisation.state": "localisation"}.get(topic)
        if role is None:
            raise Fault("not_found", "Unknown topic")
        worker = self.workers.get(role)
        return dict(worker.state) if worker else {"state": "starting"}

    def _sample(self, topic):
        if topic == 'game.state':
            return {'topic': topic, 'valid': True, 'source_mono_ns': time.monotonic_ns(),
                    'age_ms': 0, 'data': self.game.state()}
        if topic in BODY_TOPICS:
            data = self._snapshot(topic)
            source = data.get("source_mono_ns")
            age = (time.monotonic_ns() - source) / 1e6 if source is not None else None
            worker = self.workers.get("motherboard")
            limit = {"body.imu": 150, "body.stabilization": 500, "body.servos": 600}[topic]
            valid = bool(worker and worker.alive and data.get("valid")
                         and age is not None and 0 <= age < limit)
            return {"topic": topic, "valid": valid, "source_mono_ns": source,
                    "age_ms": round(age) if age is not None else None,
                    "data": data | {"valid": valid}}
        role = {"motion.state": "motherboard", "camera.state": "camera", "videostream.state": "stream", "detection.state": "detection", "localisation.state": "localisation"}.get(topic)
        worker = self.workers.get(role)
        observed = worker.last_heartbeat if worker else time.monotonic()
        if topic == "localisation.state":
            return {"topic": topic, "valid": False,
                    "source_mono_ns": int(observed * 1e9),
                    "age_ms": round((time.monotonic() - observed) * 1000),
                    "data": self._snapshot(topic)}
        return {"topic": topic, "valid": bool(worker.alive) if worker else topic == "system.workers",
                "source_mono_ns": int(observed * 1e9),
                "age_ms": round((time.monotonic() - observed) * 1000), "data": self._snapshot(topic)}

    async def _sync_body_watch(self):
        async with self.body_watch_lock:
            wanted = any(not s.closed and any(t in s.data for t in BODY_TOPICS)
                         for s in self.sessions.values())
            servos = any(not s.closed and "body.servos" in s.data for s in self.sessions.values())
            if wanted != self.body_watch or servos != self.body_servo_watch:
                args = {"enabled": wanted}
                if servos or self.body_servo_watch:
                    args["servos"] = servos
                await self.workers["motherboard"].call("body.telemetry.watch", args)
                self.body_watch = wanted
                self.body_servo_watch = servos

    async def _data(self, session, op, body):
        if op == "data.list":
            return {"items": [{"name": t, "kind": "state", "max_rate_hz": 5 if t == "body.servos" else 10,
                               "schema": 1} for t in TOPICS]}
        if op == "data.snapshot":
            if body.get("topic") in BODY_TOPICS:
                args = {"servos": True} if body["topic"] == "body.servos" else {}
                self.body_telemetry = await self.workers["motherboard"].call("body.telemetry.read", args)
            return self._sample(body.get("topic"))
        if op in ("data.subscribe", "data.update"):
            topic = body.get("topic")
            self._snapshot(topic)
            rate = number(body, "rate_hz", 2, 0.2, 5 if topic == "body.servos" else 10)
            session.data[topic] = {"rate": rate, "next": 0, "sequence": 0}
            if topic in BODY_TOPICS:
                try:
                    await self._sync_body_watch()
                except Exception:
                    session.data.pop(topic, None)
                    raise
            return {"subscription_id": topic, "rate_hz": rate}
        if op == "data.unsubscribe":
            session.data.pop(body.get("subscription_id"), None)
            await self._sync_body_watch()
            return {}
        raise Fault("not_supported", op)

    def _logs(self, session, op, body):
        if op == "log.sources":
            return page(sorted({r["source"] for r in self.history}), body)
        if op == "log.unsubscribe":
            session.logs = None
            return {}
        level = body.get("level", "INFO")
        sources = body.get("sources", [])
        if level not in LEVELS or not isinstance(sources, list) or not all(isinstance(s, str) for s in sources):
            raise Fault("invalid_argument", "Invalid log filter")
        after = number(body, "after", self.log_id if op != "log.snapshot" else 0, 0, 2**64 - 1, True)
        if op in ("log.subscribe", "log.update"):
            session.logs = {"level": level, "sources": sources, "after": after}
            return {"subscription_id": "logs", "after": after}
        records = [r for r in self.history if r["record_sequence"] > after and self._log_matches(r, level, sources)]
        limit = number(body, "limit", 2, 1, 2, True)
        return {"records": records[:limit], "next_after": records[min(limit, len(records))-1]["record_sequence"] if records else after,
                "oldest": self.history[0]["record_sequence"] if self.history else 0}

    @staticmethod
    def _log_matches(record, level, sources):
        return LEVELS.get(record["level"], 20) >= LEVELS[level] and (not sources or record["source"] in sources)

    async def _release(self, session):
        async with self.control_lock:
            if self.owner != session.id:
                return
            await self.game.stop('control_released')
            self.owner = None
            session.latest_drive = None
            try:
                await self.workers["motherboard"].call("control.release", urgent=True)
            except Fault as exc:
                self.log("supervisor", "ERROR", f"Control release: {exc}")
            if self.mode != "FAULT":
                self.mode = "IDLE"

    async def _expire(self, session):
        if session.closed:
            return
        session.closed = True
        await self._release(session)
        try:
            await self._sync_body_watch()
        except Fault as exc:
            self.log("supervisor", "WARNING", f"Body telemetry release: {exc}")
        async with self.capture_lock:
            try:
                if 'stream' in self.workers:
                    await self.workers["stream"].call("videostream.detach_session", {"session_id": session.id})
            except Fault as exc:
                self.log('stream', 'WARNING', f'Detach expired session: {exc}')
            await self._sync_video_sources()
        self.sessions.pop(session.id, None)

    async def _maintenance(self):
        while True:
            await asyncio.sleep(0.05)
            now = time.monotonic()
            for session in list(self.sessions.values()):
                if now - session.seen > 2 and not session.closed:
                    self.spawn(self._expire(session))
                    continue
                for topic, sub in session.data.items():
                    if now >= sub["next"]:
                        sub["next"] = now + 1 / sub["rate"]
                        sub["sequence"] += 1
                        self.send(session, "sample", "data.sample", {"subscription": topic,
                                  "sequence": sub["sequence"], **self._sample(topic)})
                if session.logs:
                    sub = session.logs
                    records = [r for r in self.history if r["record_sequence"] > sub["after"]]
                    dropped = max(0, self.history[0]["record_sequence"] - sub["after"] - 1) if self.history else 0
                    selected = []
                    for record in records:
                        sub["after"] = record["record_sequence"]
                        if self._log_matches(record, sub["level"], sub["sources"]):
                            selected.append(record)
                            if len(selected) == 2:
                                break
                    if selected or dropped:
                        self.send(session, "sample", "log.sample", {"subscription": "logs", "records": selected, "dropped": dropped})
            for worker in list(self.workers.values()):
                if worker.alive and now - worker.last_heartbeat > 5:
                    worker._failed("Heartbeat timeout; process will be terminated")
                    self.spawn(worker.close())

    async def close(self):
        self.closing = True
        await self.game.stop('shutdown', hard=True)
        if self.button_task:
            self.button_task.cancel()
            await asyncio.gather(self.button_task, return_exceptions=True)
        if self.head_buttons:
            self.head_buttons.close()
        if self.head_menu:
            await self.head_menu.close()
        if self.voice:
            await self.voice.close()
        for worker in reversed(list(self.workers.values())):
            await worker.close()
        for task in list(self.tasks):
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        if self.sock:
            self.sock.close()
        if self.bootstrap:
            self.bootstrap.close()
