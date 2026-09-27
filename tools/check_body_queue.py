#!/usr/bin/env python3
"""Physical BodyQueue test: small head/ID4 arm movements, never leg commands.

Stop roki-head first, secure the robot, and verify the selected servos can safely
move around 7500. No mixing slot, automatic retries or recovery poses are used.
"""
import argparse
import json
import math
import time

import Roki


def require(ok, source):
    if not ok:
        raise RuntimeError(source.GetError())


def batch(mb, rcb, label, addresses, count, amplitude):
    require(mb.ResetBodyQueue(), mb)
    before = mb.GetStatus()
    start = time.monotonic()
    max_size = 0
    for index in range(count):
        offset = round(amplitude * math.sin(2 * math.pi * index / 40)) if index < count-1 else 0
        servos = []
        for ident, bus in addresses:
            servo = Roki.Rcb4.ServoData()
            servo.Id, servo.Sio, servo.Data = ident, bus, 7500 + offset
            servos.append(servo)
        require(rcb.setServoPosAsync(servos, 2, 0), rcb)
        if index % 20 == 0:
            ok, info = mb.GetBodyQueueInfo()
            require(ok, mb)
            max_size = max(max_size, info.Size)
    enqueue_ms = (time.monotonic() - start) * 1000
    deadline = time.monotonic() + count * .25 + 3
    next_report = 0
    while True:
        status = mb.GetStatus()
        max_size = max(max_size, status["body_queue_size"])
        if time.monotonic() >= next_report:
            print(json.dumps({"phase": label, "queued": status["body_queue_size"],
                              "busy": status["body_busy"],
                              "new_failures": status["body_failures"] - before["body_failures"],
                              "last_error": status["last_body_error"]}), flush=True)
            next_report = time.monotonic() + 1
        if not status["body_queue_size"] and not status["body_busy"]:
            break
        if time.monotonic() > deadline:
            raise RuntimeError("BodyQueue did not drain")
        time.sleep(.02)
    time.sleep(.1)
    print(json.dumps({"phase": label, "commands": count, "addresses": addresses,
                      "enqueue_ms": enqueue_ms, "max_queue_size": max_size,
                      "total_seconds": time.monotonic()-start,
                      "new_failures": status["body_failures"]-before["body_failures"],
                      "final_status": status}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument("--amplitude", type=int, default=40)
    args = parser.parse_args()
    if not args.execute or not 2 <= args.count <= 400 or not 1 <= args.amplitude <= 75:
        parser.error("requires --execute, count 2..400 and amplitude 1..75")
    mb = Roki.Motherboard()
    require(mb.ConfigureACM(timeout_ms=1000), mb)
    try:
        require(mb.ConfigureBody(200, 1), mb)
        require(mb.SetBodyQueuePeriod(20), mb)
        rcb = Roki.Rcb4(mb)
        for label, addresses in (("head", ((0, 1), (12, 2))),
                                 ("arms", ((4, 1), (4, 2))),
                                 ("head_arms", ((0, 1), (12, 2), (4, 1), (4, 2)))):
            batch(mb, rcb, label, addresses, args.count, args.amplitude)
    finally:
        try:
            require(mb.ResetBodyQueue(), mb)
        finally:
            mb.Close()


if __name__ == "__main__":
    main()
