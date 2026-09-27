"""Typed, atomic per-robot settings. Defaults are not robot calibration."""

import json
import os
from pathlib import Path
import tempfile

from .wire import Fault, number, boolean

# Starting colour ranges, not the calibration of any particular robot/field.
COLOUR_DEFAULTS = {
    "orange_ball": (20, 100, 5, 65, 15, 100),
    "green_field": (10, 90, -100, -5, -10, 100),
    "white_marking": (60, 100, -15, 15, -15, 15),
    "blue_posts": (5, 80, -10, 80, -128, -5),
    "yellow_posts": (40, 100, -40, 20, 20, 127),
    "white_posts": (60, 100, -15, 15, -15, 15),
}

SCHEMA = {
    **{f"camera.white_balance.{colour}_gain":
       ("float", 1.0, 0.01, 32.0, "next_request",
        f"Manual white balance {colour} gain; calibrate for venue lighting")
       for colour in ("red", "blue")},
    "logging.stdout_enabled": ("bool", False, None, None, "live", "Mirror runtime logs to supervisor stdout"),
    "motion.max_step_mm": ("float", 24.0, 1, 64, "next_job", "Forward stride scale"),
    "motion.max_side_mm": ("float", 12.0, 1, 20, "next_job", "Side stride scale"),
    "motion.max_yaw_rad": ("float", 0.15, 0, 0.3, "next_job", "Rotation per walking cycle"),
    "motion.frame_ms": ("int", 20, 10, 40, "restart", "Motherboard body queue period"),
    "motion.frames_per_cycle": ("int", 2, 1, 10, "restart", "Servo interpolation frames per gait frame"),
    "motion.body_tilt": ("float", 0.0, -0.3, 0.3, "next_job", "Forward walking body tilt"),
    "motion.body_tilt_back": ("float", 0.0, -0.3, 0.3, "next_job", "Backward walking body tilt"),
    "motion.body_tilt_kick": ("float", 0.0, -0.3, 0.3, "next_job", "Kicking body tilt"),
    "motion.sole_skew": ("float", 0.0, -0.2, 0.2, "next_job", "Sole landing skew"),
    "motion.shift_x_mm": ("float", 0.0, -2000, 2000, "next_job", "Displacement over spot-walk test, X mm"),
    "motion.shift_y_mm": ("float", 0.0, -2000, 2000, "next_job", "Displacement over spot-walk test, Y mm"),
    "motion.rotation_yield_right": ("float", 0.23, 0.01, 1, "next_job", "Measured clockwise yield"),
    "motion.rotation_yield_left": ("float", 0.23, 0.01, 1, "next_job", "Measured counterclockwise yield"),
    "motion.jump_yaw_cw": ("float", -0.44, -1.5, -0.01, "next_job", "Clockwise full-jump yaw, radians"),
    "motion.jump_yaw_ccw": ("float", 0.41, 0.01, 1.5, "next_job", "Counterclockwise full-jump yaw, radians"),
    "motion.run_10_mm": ("float", 1000.0, 1, 10000, "next_job", "Measured short_run distance, mm"),
    "motion.run_20_mm": ("float", 2000.0, 1, 20000, "next_job", "Measured long_run distance, mm"),
    "motion.side_right_20_mm": ("float", 400.0, 1, 10000, "next_job", "Measured 20 right cycles, mm"),
    "motion.side_left_20_mm": ("float", 400.0, 1, 10000, "next_job", "Measured 20 left cycles, mm"),
    **{f"motion.jump_{direction}_mm": ("float", 30.0, 1, 500, "next_job", "Measured displacement per jump, mm")
       for direction in ("forward", "backward", "left", "right")},
    "head.field_tilt": ("int", -1000, -2600, 950, "next_job", "Head field pose, servo ticks"),
    **{f"vision.{profile}.{key}": ("int", value, 0 if key.startswith("l_") else -128,
                                   100 if key.startswith("l_") else 127, "next_frame",
                                   f"LAB threshold {profile}: {key}; L 0..100, a/b -128..127")
       for profile, values in COLOUR_DEFAULTS.items()
       for key, value in zip(("l_min", "l_max", "a_min", "a_max", "b_min", "b_max"), values)},
    **{f"vision.{profile}.{key}": ("int", 50, 1, 800 * 650, "next_frame", description)
       for profile in COLOUR_DEFAULTS
       for key, description in (("pixels_min", "Minimum foreground pixels in a colour blob"),
                                ("box_area_min", "Minimum bounding rectangle area, pixels"))},
}


def validate_colour_ranges(values):
    for profile in COLOUR_DEFAULTS:
        for axis in ("l", "a", "b"):
            prefix = f"vision.{profile}.{axis}"
            if values[prefix + "_min"] > values[prefix + "_max"]:
                raise Fault("invalid_argument", f"{prefix}_min must not exceed {prefix}_max")


class Parameters:
    def __init__(self, directory):
        self.path = Path(directory) / "parameters.json"
        self.values = {k: v[1] for k, v in SCHEMA.items()}
        self.extra = {}
        if self.path.exists():
            stored = json.loads(self.path.read_text())
            if not isinstance(stored, dict):
                raise Fault("configuration", "parameters.json must contain an object")
            for key, value in stored.items():
                if key in SCHEMA:
                    self.values[key] = self.validate(key, value)
                else:
                    self.extra[key] = value
        validate_colour_ranges(self.values)
        self.save(self.values)

    def describe(self, key):
        if key not in SCHEMA:
            raise Fault("not_found", f"Unknown parameter: {key}")
        kind, default, low, high, apply, description = SCHEMA[key]
        return dict(key=key, type=kind, default=default, min=low, max=high,
                    apply=apply, description=description)

    def validate(self, key, value):
        meta = self.describe(key)
        if meta["type"] == "bool":
            return boolean({key: value}, key)
        return number({key: value}, key, None, meta["min"], meta["max"], meta["type"] == "int")

    def set(self, key, value):
        values = self.set_many({key: value})
        return {"key": key, "value": values[key], "apply": SCHEMA[key][4]}

    def set_many(self, values):
        values = {key: self.validate(key, value) for key, value in values.items()}
        updated = self.values | values
        validate_colour_ranges(updated)
        self.save(updated)
        self.values = updated
        return values

    def save(self, values):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(dir=self.path.parent, prefix=".params-")
        try:
            with os.fdopen(fd, "w") as handle:
                json.dump(self.extra | values, handle, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(name, self.path)
            fd = os.open(self.path.parent, os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        finally:
            if os.path.exists(name):
                os.unlink(name)
