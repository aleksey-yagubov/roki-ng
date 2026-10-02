"""Speech-only head menu. Navigation never starts hardware implicitly."""

import asyncio
from contextlib import suppress
from dataclasses import dataclass, field

from .wire import Fault


@dataclass
class Item:
    label: str
    children: tuple = ()
    op: str | None = None
    args: dict = field(default_factory=dict)


def test(label, name, **args):
    return Item(label, op="test.start", args={"name": name, **args})


def menu_tree():
    def forward(position, delay=0):
        return Item("Start later" if delay else "Start", op="game.start",
                    args={"strategy": "forward", "entry": position, "delay_seconds": delay})

    other = (
        test("Run backwards", "run_test", mode="backwards"),
        *(test(f"Jump {direction.replace('_', ' ')}", "jump_test", direction=direction)
          for direction in ("forward", "backward", "left", "right", "on_spot")),
    )
    return Item("Main menu", (
        Item("Game", (Item("Football", (
            Item("Goalkeeper", (Item("Start", op="game.start",
                 args={"strategy": "FIRA_penalty_Goalkeeper", "delay_seconds": 0, "observe_only": False}),
                 Item("Observe only", op="game.start", args={"strategy": "FIRA_penalty_Goalkeeper", "observe_only": True}))),
            Item("Forward", (
                Item("Left", (forward("left"),)),
                Item("Center", (forward("center"), forward("center", 10))),
                Item("Right", (forward("right"),)),
            )),
        )),)),
        Item("Tests", (
            test("Rotation right", "rotation_test"),
            test("Short run", "run_test", mode="short"),
            test("Long run", "run_test", mode="long"),
            test("Spot run", "run_test", mode="spot"),
            test("Kick test", "kick_test", mode="regular"),
            test("Side step left", "run_test", mode="side_left"),
            test("Side step right", "run_test", mode="side_right"),
            test("Get up test", "get_up_test"),
            Item("Other tests", other),
        )),
    ))


class HeadMenu:
    def __init__(self, command, release, say, log):
        self.command, self.release, self.say, self.log = command, release, say, log
        self.stack = [[menu_tree(), 0]]
        self.job = None
        self.action = None
        self.cancel_requested = False
        self.stopping = False
        self.finishing = False
        self.game_active = False

    @property
    def selected(self):
        node, index = self.stack[-1]
        return node.children[index]

    def press(self, key):
        if key not in ("back", "ok", "left", "right"):
            return
        if self.finishing:
            return
        if self.job or self.action and not self.action.done():
            if key == "back":
                self.cancel_requested = True
                if self.job and (self.action is None or self.action.done()):
                    self.action = asyncio.create_task(self._cancel())
            return
        if key == "back":
            if len(self.stack) > 1:
                self.stack.pop()
            self.say(self.selected.label)
        elif key in ("left", "right"):
            node, index = self.stack[-1]
            self.stack[-1][1] = (index + (1 if key == "right" else -1)) % len(node.children)
            self.say(self.selected.label)
        elif self.selected.children:
            self.stack.append([self.selected, 0])
            self.say(self.selected.label)
        else:
            self.cancel_requested = False
            self.action = asyncio.create_task(self._start(self.selected))

    def hold(self, key):
        """A hold is distinct from menu navigation; only an active game uses it."""
        if key == 'back' and self.game_active and not self.finishing:
            self.action = asyncio.create_task(self._pickup())

    async def _pickup(self):
        try:
            await self.command('game.pickup', {})
            self.say('Pick up')
        except Exception as exc:
            self._error(exc)

    def _error(self, exc):
        self.log("WARNING", str(exc))
        text = {
            "body_unavailable": "Body unavailable",
            "busy": "Control busy",
            "worker_unavailable": "Worker unavailable",
            "not_ready": "Not ready",
            "not_supported": "Not available yet",
            "football_unavailable": "Football not available yet",
        }.get(getattr(exc, "code", None), "Command failed")
        self.say(text)

    async def _start(self, item):
        try:
            self.game_active = item.op == 'game.start'
            result = await self.command(item.op, dict(item.args))
            self.job = result["job_id"]
            ident = self.job
            if self.game_active:
                self.say('Goalkeeper observing' if result['observe_only'] else 'Goalkeeper started')
                if self.cancel_requested:
                    await self._cancel()
                await self.game_finished(await self.command('game.status', {}))
                return
            self.say("Test started")
            if self.cancel_requested:
                await self._cancel()
            # A very short job can finish before test.start's reply arrives.
            status = await self.command("job.status", {"job_id": ident})
            await self._finish(status)
        except Exception as exc:
            self.game_active = False
            self._error(exc)
            if self.job is None:
                await self.release()

    async def _cancel(self):
        if self.game_active:
            self.stopping = True
            await self.game_finished(await self.command('game.stop', {}))
            return
        try:
            await self.command("job.cancel", {"job_id": self.job})
            self.stopping = True
            self.say("Stopping")
        except Fault as exc:
            if exc.code != "not_found":
                self._error(exc)
            # Completion may have raced with cancellation.
        try:
            if self.job:
                await self._finish(await self.command("job.status", {"job_id": self.job}))
        except Exception as exc:
            self._error(exc)

    async def _finish(self, status):
        if self.game_active:
            return
        if status.get("job_id") != self.job or not self.job:
            return
        if status.get("status") not in ("completed", "cancelled", "failed"):
            return
        self.job = None
        self.finishing = True
        try:
            await self.release()
        finally:
            self.finishing = False
        if status["status"] == "failed":
            self.log("ERROR", status.get("reason") or "Test failed")
            self.say("Test failed")
        else:
            self.say("Stopped" if self.stopping or status["status"] == "cancelled" else "Test completed")
        self.stopping = self.cancel_requested = False

    async def worker_failed(self):
        self.job = None
        await self.release()
        self.say("Worker unavailable")

    def taken_over(self):
        if self.action and not self.action.done():
            self.action.cancel()
        self.job = None
        self.game_active = False
        self.stopping = self.cancel_requested = False
        self.say("Operator control")

    async def game_finished(self, status):
        if not self.game_active or status.get('running') or status.get('job_id') != self.job:
            return
        self.game_active = False
        self.job = None
        self.finishing = True
        try:
            await self.release()
        finally:
            self.finishing = False
        self.say('Goalkeeper failed' if status.get('state') == 'failed' else 'Goalkeeper stopped')
        self.stopping = self.cancel_requested = False

    async def close(self):
        if self.action:
            self.action.cancel()
            await asyncio.gather(self.action, return_exceptions=True)
        await self.release()


class Voice:
    """One espeak process, with at most one pending (latest) announcement."""

    def __init__(self, log):
        self.log = log
        self.pending = None
        self.wake = asyncio.Event()
        self.process = None
        self.task = asyncio.create_task(self._run())

    def say(self, text):
        self.log("INFO", text)
        self.pending = text
        self.wake.set()
        if self.process and self.process.returncode is None:
            with suppress(ProcessLookupError):
                self.process.terminate()

    async def _run(self):
        try:
            while True:
                await self.wake.wait()
                self.wake.clear()
                text, self.pending = self.pending, None
                try:
                    self.process = await asyncio.create_subprocess_exec(
                        "espeak", "-ven-m1", "-a50", text,
                        stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
                    if self.pending is not None:
                        with suppress(ProcessLookupError):
                            self.process.terminate()
                    code = await self.process.wait()
                    if code > 0:
                        self.log("WARNING", f"espeak exited with code {code}")
                except OSError as exc:
                    self.log("ERROR", f"Speech unavailable: {exc}")
                finally:
                    if self.process and self.process.returncode is None:
                        with suppress(ProcessLookupError):
                            self.process.kill()
                        await self.process.wait()
                    self.process = None
        finally:
            if self.process and self.process.returncode is None:
                with suppress(ProcessLookupError):
                    self.process.kill()
                await self.process.wait()

    async def close(self):
        self.task.cancel()
        await asyncio.gather(self.task, return_exceptions=True)


class HeadButtons:
    def __init__(self, menu, log):
        self.menu, self.log = menu, log
        self.device = None
        self.held = set()
        self.hold_tasks = {}

    def event(self, event):
        # BTN2 was process reload. BTN3 is OK; BTN1/4 are left/right.
        keys = {184: "back", 185: "ok", 183: "left", 186: "right"}
        if event.type != 1 or event.code not in keys:
            return
        if event.value == 0:
            self.held.discard(event.code)
            task = self.hold_tasks.pop(event.code, None)
            if task: task.cancel()
        elif event.value == 1 and event.code not in self.held:
            self.held.add(event.code)
            self.menu.press(keys[event.code])
            if keys[event.code] == 'back':
                try:
                    loop = asyncio.get_running_loop()
                except RuntimeError:
                    # Unit-level navigation is synchronous.  The real input loop
                    # always runs in asyncio, where the hold timer is installed.
                    pass
                else:
                    async def held_back(code=event.code):
                        await asyncio.sleep(.7)
                        if code in self.held:
                            self.menu.hold('back')
                    self.hold_tasks[event.code] = loop.create_task(held_back())

    def open(self):
        from evdev import InputDevice, list_devices

        for path in list_devices():
            device = InputDevice(path)
            if device.name == "roki-head-buttons":
                self.device = device
                self.held = set(device.active_keys())
                return
            device.close()
        raise FileNotFoundError("roki-head-buttons input device not found")

    async def run(self):
        from evdev import ecodes

        while True:
            try:
                if self.device is None:
                    self.open()
                dropped = False
                async for event in self.device.async_read_loop():
                    if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_DROPPED:
                        dropped = True
                    elif dropped:
                        if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_REPORT:
                            self.held = set(self.device.active_keys())
                            dropped = False
                    else:
                        self.event(event)
            except OSError as exc:
                self.log("WARNING", f"Head buttons disconnected: {exc}")
                self.close()
                await asyncio.sleep(1)

    def close(self):
        for task in self.hold_tasks.values(): task.cancel()
        self.hold_tasks.clear()
        if self.device:
            self.device.close()
            self.device = None
