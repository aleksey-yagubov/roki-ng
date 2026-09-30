"""Slot decoding for the fixed Kondo body backend (ACM v2 is its transport)."""

import math
from types import SimpleNamespace

from ..wire import Fault


def joints(model):
    result = {row[6].removeprefix("MASK_").lower(): (row[0], row[1])
              for row in model.ACTIVESERVOS}
    result.update(right_knee_bot=(13, 1), left_knee_bot=(13, 2))
    return result


MIRRORED = frozenset((
    "left_clavicle", "left_shoulder", "left_elbow_side", "right_elbow",
    "torso_rotate", "left_pelvic", "left_hip_side", "left_hip", "left_knee",
    "left_knee_bot", "right_foot_front", "right_foot_side", "head_tilt",
))


def position(angle, units, joint):
    if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle):
        raise Fault("invalid_motion", "Non-finite joint angle")
    if units == "zubr":
        angle = angle * (-1 if joint in MIRRORED else 1) * 1000 / 1536
    elif units != "kondo":
        raise Fault("invalid_motion", "Unknown slot units")
    value = 7500 + round(angle)
    if not 0 <= value <= 16383:
        raise Fault("invalid_motion", "Joint target outside Kondo range")
    return value


def decode(document, model, factor=1):
    """Explicit individual joints; Kondo angles are signed wire offsets from 7500."""
    units = document.get("units", "kondo")
    if units not in ("kondo", "zubr") or document.get("joint_space") != "individual":
        raise Fault("invalid_motion", "Expected individual joints and kondo/zubr units")
    rows = document.get("frames")
    if not isinstance(rows, list) or not 1 <= len(rows) <= 2048:
        raise Fault("invalid_motion", "Invalid slot frames")
    mapping = joints(model)
    steps = []
    for row in rows:
        if not isinstance(row, dict):
            raise Fault("invalid_motion", "Invalid slot frame")
        frames = row.get("duration")
        if (isinstance(frames, bool) or not isinstance(frames, (int, float))
                or not math.isfinite(frames) or not 1 <= round(frames / factor) <= 255):
            raise Fault("invalid_motion", "Invalid slot duration")
        frames = round(frames / factor)
        targets = row.get("targets")
        if not isinstance(targets, dict) or not targets or targets.keys() - mapping.keys():
            raise Fault("invalid_motion", "Invalid joint names")
        values = [SimpleNamespace(Id=mapping[name][0], Sio=mapping[name][1],
                                  Data=position(value, units, name))
                  for name, value in targets.items()]
        steps.append(("servo", values, frames, frames - 1))
    return steps


def splits(big):
    """ROKI2 splits entry, retaining individual lower knees; no automatic exit."""
    def frame(duration, **targets):
        return {"duration": duration, "targets": {
            f"{side}_{joint}": value for joint, value in targets.items()
            for side in ("right", "left")}}
    rows = [frame(80, pelvic=0, hip_side=0, hip=3000, knee=3600,
                  knee_bot=3600, foot_front=4100, foot_side=0),
            frame(9, foot_side=3000),
            frame(10, pelvic=0, hip_side=1400, hip=3000, knee=3600,
                  knee_bot=3600, foot_front=4100, foot_side=1400)]
    if big:
        rows.extend([frame(9, foot_side=3000),
                     frame(40, hip_side=4200, foot_side=1500, clavicle=2800, shoulder=0),
                     frame(40, hip=0, knee=0, knee_bot=0, foot_front=0)])
    return {"units": "zubr", "joint_space": "individual", "frames": rows}
