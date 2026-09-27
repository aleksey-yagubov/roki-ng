#!/usr/bin/env python3
"""Exclusive, non-motion ACM v2 stress test. Stop roki-head before running.

Only ACK and RAM reads are forwarded to Zubr. The STM queue contains ACKs,
never servo positions, slots, relaxation or writes to controller memory.
"""
import argparse
from collections import Counter
import json
import statistics
import struct
import time

import Roki
import serial

HEADER = struct.Struct("<2sBBHBBH")
ACK = bytes((4, 254, 6, 8))


def memory_read(address, size):
    packet = bytes((10, 0, 0x20, 0, 0, 0, address & 255, address >> 8, size))
    return packet + bytes((sum(packet) & 255,))


class Peer:
    def __init__(self):
        self.port = serial.Serial(Roki.FindMotherboard(), timeout=2, write_timeout=2,
                                  exclusive=True)
        self.ident = 0

    def read(self, size):
        data = self.port.read(size)
        if len(data) != size:
            raise RuntimeError(f"Incomplete ACM reply: {len(data)}/{size}")
        return data

    def rpc(self, code, payload=b""):
        self.ident = (self.ident + 1) & 65535
        packet = HEADER.pack(b"RK", 2, 1, self.ident, code, 0, len(payload)) + payload
        if self.port.write(packet) != len(packet):
            raise RuntimeError("Incomplete ACM write")
        magic, version, kind, ident, reply_code, status, size = HEADER.unpack(self.read(10))
        if (magic, version, kind, ident, reply_code) != (b"RK", 2, 2, self.ident, code):
            raise RuntimeError("Unexpected reply: another process or stream owns ACM")
        return status, self.read(size)

    def required(self, code, payload=b""):
        status, data = self.rpc(code, payload)
        if status:
            raise RuntimeError(f"RPC {code:#x}: status {status}")
        return data


def phase(peer, seconds, queued):
    counts, errors, times = Counter(), Counter(), []
    before = peer.required(2).hex()
    start = time.monotonic()
    next_queue = start
    while time.monotonic() - start < seconds:
        if queued and time.monotonic() >= next_queue:
            status, _ = peer.rpc(0x21, bytes((0, 4, 4)) + ACK)
            counts["queued_ack"] += 1
            if status:
                errors[f"enqueue:{status}"] += 1
            next_queue = time.monotonic() + 0.02
        for name, command, expected in (("ack", ACK, 4),
                                        ("imu8", memory_read(0x60, 8), 11),
                                        ("ram64", memory_read(0, 64), 67)):
            t = time.monotonic()
            status, reply = peer.rpc(0x20, bytes((len(command), expected)) + command)
            times.append((time.monotonic() - t) * 1000)
            counts[name] += 1
            if status:
                errors[f"{name}:{status}"] += 1
            elif len(reply) != expected or reply[0] != expected or sum(reply[:-1]) & 255 != reply[-1]:
                errors[f"{name}:invalid_packet"] += 1
            elif name == "ack" and reply != ACK:
                errors["ack:unexpected_data"] += 1
    times.sort()
    print(json.dumps({"queued": queued, "seconds": time.monotonic() - start,
                      "counts": counts, "errors": errors,
                      "median_ms": statistics.median(times),
                      "p99_ms": times[int(len(times) * .99)], "max_ms": max(times),
                      "status_before_hex": before, "status_after_hex": peer.required(2).hex()}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("seconds must be positive")
    peer = Peer()
    try:
        peer.required(0x12)  # Stop capture/stream: this test has exclusive ownership.
        peer.required(0x25, struct.pack("<HB", 200, 1))
        peer.required(0x24)
        peer.required(0x23, bytes((20,)))
        phase(peer, args.seconds, False)
        phase(peer, args.seconds, True)
    finally:
        try:
            peer.required(0x24)
        finally:
            peer.port.close()


if __name__ == "__main__":
    main()
