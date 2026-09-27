"""Hardware-independent, bounded SPMC smoke test on the target Python runtime."""

import ctypes
import json
import multiprocessing as mp
import struct
import time
import uuid
import zlib

import iceoryx2 as iox

HEADER = struct.Struct("<QQ")


def service(node, name):
    return (node.service_builder(iox.ServiceName.new(name))
            .publish_subscribe(iox.Slice[ctypes.c_uint8])
            .max_publishers(1).max_subscribers(3).max_nodes(4)
            .history_size(0).subscriber_max_buffer_size(2)
            .subscriber_max_borrowed_samples(1).enable_safe_overflow(True)
            .open_or_create())


def consume(name, size, delay, pipe):
    try:
        node = iox.NodeBuilder.new().create(iox.ServiceType.Ipc)
        svc = service(node, name)
        sub = svc.subscriber_builder().create()
        pipe.send("ready")
        count = skipped = 0
        last = -1
        max_age_ms = 0.0
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if pipe.poll():
                pipe.recv()
                break
            sample = sub.receive()
            if sample is None:
                time.sleep(0.001)
                continue
            payload = sample.payload()
            view = payload.as_memory_view().cast("B")
            seq, sent = HEADER.unpack_from(view)
            checksum = zlib.crc32(view)
            # Hold a borrowed sample across publisher writes to other slots.
            time.sleep(delay)
            if len(view) != size or checksum != zlib.crc32(view):
                raise RuntimeError("borrowed sample changed or wrong size")
            if seq <= last or view[-1] != seq % 256:
                raise RuntimeError("invalid sequence or payload")
            if last >= 0:
                skipped += seq - last - 1
            last = seq
            count += 1
            max_age_ms = max(max_age_ms, (time.monotonic_ns() - sent) / 1e6)
            del view, payload, sample
        pipe.send(dict(count=count, skipped=skipped, last=last,
                       max_age_ms=round(max_age_ms, 2)))
    except Exception as exc:
        pipe.send(dict(error=repr(exc)))
        raise
    finally:
        pipe.close()


def check(size, frames=180):
    ctx = mp.get_context("spawn")
    name = "roki/check/" + uuid.uuid4().hex
    node = iox.NodeBuilder.new().create(iox.ServiceType.Ipc)
    svc = service(node, name)
    pub = (svc.publisher_builder().initial_max_slice_len(size)
           .max_loaned_samples(1)
           .backpressure_strategy(iox.BackpressureStrategy.DiscardData).create())
    children = []
    try:
        for delay in (0.0, 0.08):
            parent, child = ctx.Pipe()
            process = ctx.Process(target=consume, args=(name, size, delay, child))
            process.start()
            child.close()
            children.append((process, parent))
            if not parent.poll(10) or parent.recv() != "ready":
                raise RuntimeError("subscriber startup failed")
        pub.update_connections()
        start = time.monotonic()
        max_send_ms = 0.0
        for seq in range(frames):
            sample = pub.loan_slice_uninit(size)
            payload = sample.payload()
            ctypes.memset(payload.as_ptr(), seq % 256, size)
            header = HEADER.pack(seq, time.monotonic_ns())
            ctypes.memmove(payload.as_ptr(), header, len(header))
            del payload
            sample = sample.assume_init()
            before = time.monotonic()
            sample.send()
            max_send_ms = max(max_send_ms, (time.monotonic() - before) * 1000)
            del sample
            time.sleep(max(0, start + (seq + 1) / 60 - time.monotonic()))
        elapsed = time.monotonic() - start
        results = []
        for process, pipe in children:
            pipe.send("stop")
            if not pipe.poll(5):
                raise RuntimeError("subscriber did not stop")
            result = pipe.recv()
            process.join(5)
            if process.exitcode != 0 or result.get("error") or not result.get("count"):
                raise RuntimeError(str(result))
            results.append(result)
        if not results[1]["skipped"]:
            raise RuntimeError("slow reader did not exercise overflow")
        print(json.dumps(dict(bytes=size, published=frames, seconds=round(elapsed, 3),
                              max_send_ms=round(max_send_ms, 3), readers=results)), flush=True)
    finally:
        for process, pipe in children:
            if process.is_alive():
                process.terminate()
                process.join(3)
            if process.is_alive():
                process.kill()
                process.join()
            pipe.close()


if __name__ == "__main__":
    check(64)
    check(800 * 650 * 3 + HEADER.size)
