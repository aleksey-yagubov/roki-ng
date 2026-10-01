"""Head-only end-to-end check with per-start timestamp alignment; no movement."""

import argparse
import asyncio
import json
import multiprocessing as mp
import sys
import time
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from roki_ng.dataplane import Channel, FRAME_TOPIC, IMU_TOPIC, FRAME_HEADER, IMU_RECORD
from roki_ng.synchronization import SequenceJoiner


def observe(pipe, timeout):
    import iceoryx2 as iox
    channels = [Channel(FRAME_TOPIC), Channel(IMU_TOPIC)]
    waitset = iox.WaitSetBuilder.new().create(iox.ServiceType.Ipc)
    guards = [waitset.attach_deadline(c.listener, iox.Duration.from_secs(1)) for c in channels]
    joiner = SequenceJoiner()
    counts = [0, 0]
    first = [None, None]
    last = [None, None]
    matches = 0
    crc = None
    pipe.send("ready")
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if pipe.poll():
                command = pipe.recv()
                if isinstance(command, dict):
                    for c in channels:
                        c.drain()
                    joiner.set_alignment(command["alignment"])
                    counts, first, last, matches = [0, 0], [None, None], [None, None], 0
                    pipe.send("aligned")
                    continue
                report = dict(frames=counts[0], imu=counts[1], matches=matches,
                              first=first, last=last, frame_crc=crc, unmatched_evicted=joiner.dropped)
                if command == "stop":
                    pipe.send(report)
                    break
                for c in channels:
                    c.drain()
                joiner.set_alignment(None)
                counts, first, last, matches = [0, 0], [None, None], [None, None], 0
                pipe.send(report)
            waitset.wait_and_process()
            for index, channel in enumerate(channels):
                channel.listener.try_wait()
                while (sample := channel.receive()) is not None:
                    payload = sample.payload()
                    view = payload.as_memory_view().cast("B")
                    record = (FRAME_HEADER if index == 0 else IMU_RECORD).unpack_from(view)
                    seq = record[0]
                    if first[index] is None:
                        first[index] = seq
                    last[index] = seq
                    counts[index] += 1
                    if index == 0:
                        assert len(view) == FRAME_HEADER.size + record[3] * record[4]
                        crc = zlib.crc32(view[FRAME_HEADER.size:])
                        pair = joiner.frame(seq, record)
                    else:
                        pair = joiner.measurement(seq, record)
                    if pair:
                        assert pair[0][0] + joiner.offset == pair[1][0]
                        matches += 1
                    view.release()
                    del view, payload, sample
    finally:
        del guards, waitset
        for channel in channels:
            channel.close()
        pipe.close()


async def check(args):
    from roki_ng.supervisor import Supervisor, Session
    cfg = dict(host="0.0.0.0", port=8099, state_dir="/tmp/roki-ng-capture-check",
               uart="/dev/ttyAMA5", body_disabled=not args.probe_body, skip_mixing=True)
    supervisor = Supervisor(cfg)
    supervisor.params.set_many({"logging.stdout_enabled": True})
    # Dispatch locally: the target image currently has no configured loopback.
    session = Session(1, 1, ("172.30.0.1", 8099), "local-capture-check")
    ctx = mp.get_context("spawn")
    pipe, child = ctx.Pipe()
    process = ctx.Process(target=observe, args=(child, args.cycles * (args.seconds + 15) + 30))
    try:
        await supervisor.start()
        if supervisor.mode != "IDLE":
            raise RuntimeError("Supervisor startup failed")
        process.start()
        child.close()
        if not await asyncio.to_thread(pipe.poll, 10) or pipe.recv() != "ready":
            raise RuntimeError("Observer failed to start")
        lease = await supervisor.dispatch(session, "control.acquire", {})
        body = {"lease_epoch": lease["lease_epoch"]}
        if args.fps30:
            body["frame_duration_us"] = 33333
        await supervisor.dispatch(session, "mode.set", body | {"mode": "MANUAL"})
        for cycle in range(args.cycles):
            await supervisor.dispatch(session, "camera.start", body)
            deadline = time.monotonic()+6
            while time.monotonic() < deadline:
                state = await supervisor.workers["camera"].call("camera.status")
                if state["imu_sync"]["state"] == "synced":
                    break
                if state["error"]:
                    raise RuntimeError(state["error"])
                await asyncio.sleep(0.05)
            else:
                raise RuntimeError("Camera failed to synchronize IMU")
            print("SYNC", state["imu_sync"], flush=True)
            pipe.send({"alignment":state["imu_sync"]["unicam_minus_stm"]})
            if not await asyncio.to_thread(pipe.poll, 3) or pipe.recv() != "aligned":
                raise RuntimeError("Observer did not apply alignment")
            await asyncio.sleep(args.seconds)
            print("CAMERA", await supervisor.workers["camera"].call("camera.status"), flush=True)
            print("MOTHERBOARD", await supervisor.workers["motherboard"].call("state"), flush=True)
            await supervisor.dispatch(session, "camera.stop", body)
            pipe.send("stop" if cycle == args.cycles - 1 else "reset")
            if not await asyncio.to_thread(pipe.poll, 5):
                raise RuntimeError("Observer did not report")
            result = pipe.recv()
            print("RESULT", cycle, json.dumps(result), flush=True)
            if not result["frames"] or not result["imu"] or not result["matches"]:
                raise RuntimeError("Missing frames/IMU/exact sequence matches")
            if result["matches"] < 0.9 * min(result["frames"], result["imu"]):
                raise RuntimeError("Too few exact camera/IMU matches")
    finally:
        await supervisor.close()
        if process.pid:
            await asyncio.to_thread(process.join, 3)
            if process.is_alive():
                process.kill()
                await asyncio.to_thread(process.join)
        pipe.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--30fps", dest="fps30", action="store_true")
    parser.add_argument("--probe-body", action="store_true")
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--cycles", type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 600 or not 1 <= args.cycles <= 10:
        parser.error("Expected seconds in 1..600 and cycles in 1..10")
    asyncio.run(check(args))
