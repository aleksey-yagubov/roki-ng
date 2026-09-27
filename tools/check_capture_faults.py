"""Head-only worker-exit test: stop related capture, keep supervisor responsive."""

import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from roki_ng.supervisor import Supervisor, Session


async def check(role):
    supervisor = Supervisor(dict(host="0.0.0.0", port=8099,
                                 state_dir="/tmp/roki-ng-capture-check",
                                 uart="/dev/ttyAMA5", body_disabled=True,
                                 skip_mixing=True))
    supervisor.params.set_many({"logging.stdout_enabled": True})
    session = Session(1, 1, ("172.30.0.1", 8099), "capture-fault-check")
    try:
        await supervisor.start()
        if supervisor.mode != "IDLE":
            raise RuntimeError("Supervisor startup failed")
        lease = await supervisor.dispatch(session, "control.acquire", {})
        await supervisor.dispatch(session, "mode.set", lease | {"mode": "MANUAL"})
        await supervisor.dispatch(session, "camera.start", lease)
        deadline = time.monotonic() + 6
        while time.monotonic() < deadline:
            state = await supervisor.workers["camera"].call("camera.status")
            if state["imu_sync"]["state"] == "synced":
                break
            if state["error"]:
                raise RuntimeError(state["error"])
            await asyncio.sleep(0.05)
        else:
            raise RuntimeError("No camera/IMU alignment")

        print("KILL", role, flush=True)
        failed = supervisor.workers[role]
        failed.process.kill()
        await failed.process.wait()
        survivor = supervisor.workers["motherboard" if role == "camera" else "camera"]
        operation = "state" if role == "camera" else "camera.status"
        running_key = "imu_running" if role == "camera" else "running"
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            state = await survivor.call(operation)
            if not state[running_key] and supervisor.capture_session is None:
                break
            await asyncio.sleep(0.05)
        else:
            raise RuntimeError("Related capture did not stop after worker exit")
        status = await supervisor.dispatch(session, "system.status", {})
        assert status["state"] == "FAULT", status
        assert survivor.alive
        print("FAULT HANDLED", role, state, flush=True)
    finally:
        await supervisor.close()
        # SIGKILL cannot run motherboard cleanup. Firmware has no host lease yet;
        # clear its capture explicitly after all runtime owners have exited.
        import Roki
        mb = Roki.Motherboard()
        try:
            assert mb.ConfigureACM(timeout_ms=1000), mb.GetError()
            before = mb.GetStatus()
            print("STM BEFORE CLEANUP", before, flush=True)
            if role == "camera":
                assert not before["capture_active"] and before["stream_mode"] == 0, before
            assert mb.StopStrobeCapture(), mb.GetError()
            after = mb.GetStatus()
            assert not after["capture_active"] and after["stream_mode"] == 0, after
        finally:
            mb.Close()


async def main():
    for role in ("camera", "motherboard"):
        await check(role)


if __name__ == "__main__":
    asyncio.run(main())
