"""Bounded test preflight using the body's mounted IMU, not the head IMU."""

import json
import math
from pathlib import Path

from .wire import Fault


SLOTS = {
    "stomach": "Roki_2_Get_UP_Stomach",
    "back": "Roki_2_Get_UP_Face_Up",
    "right": "Get_Up_Right",
    "left": "Get_Up_Left",
}


def body_position(quaternion):
    x, y, z, w = quaternion
    norm = x*x + y*y + z*z + w*w
    if not all(math.isfinite(v) for v in quaternion) or not 0.9 < norm < 1.1:
        raise Fault("imu_invalid", "Invalid body quaternion")
    # World-up in sensor coordinates. Upright is +Y: the legacy PCB mounting
    # subtracts pi/2 from Euler X and negates Euler Y. Gravity avoids gimbal lock.
    gx = 2 * (x*z - w*y) / norm
    gy = 2 * (y*z + w*x) / norm
    gz = 1 - 2 * (x*x + y*y) / norm
    if gy >= math.cos(0.6):
        return "upright"
    axis, value = ("pitch", -gz) if abs(gz) >= abs(gx) else ("roll", gx)
    if abs(value) < math.sin(1.0):
        raise Fault("posture_uncertain", "Body is tilted, inverted or between recovery positions")
    if axis == "pitch":
        return "stomach" if value > 0 else "back"
    return "right" if value > 0 else "left"


def stable_position(body):
    positions = []
    for index in range(3):
        if body.stop_requested:
            return None
        positions.append(body_position(body.read_body_quaternion()))
        if index < 2:
            yield "sleep", 0.05
    if len(set(positions)) != 1:
        raise Fault("posture_unstable", "Body position changed during preflight")
    return positions[0]


def ensure_upright(body):
    job = body.jobs[body.active]
    job.update(stage="checking_posture")
    body.emit("job.progress", dict(job))
    position = yield from stable_position(body)
    if position is None:
        return
    job["initial_posture"] = position
    if position != "upright":
        name = SLOTS[position]
        rows = json.loads((Path(__file__).with_name("assets") / "slots" / f"{name}.json").read_text())[name]
        body._validate_rows(rows)
        job.update(stage="getting_up", recovery_slot=name)
        body.emit("job.progress", dict(job))
        body.log("INFO", f"Getting up from {position}: {name}")
        body.recovery_active = True
        try:
            body.hardware.reset()
            yield from body._slot(rows)
            # Wait for the queue and final interpolation, then let the body settle.
            for _ in range(10):
                if body.stop_requested:
                    return
                yield "sleep", 0.05
            position = yield from stable_position(body)
            if position is None:
                return
            if position != "upright":
                raise Fault("get_up_failed", f"Body is still {position}; no automatic retry")
            body.pose = "stand"
            body.engine = None
        finally:
            body.recovery_active = False
    if not body.stop_requested:
        job.update(stage="upright", posture_verified=True)
        body.emit("job.progress", dict(job))
