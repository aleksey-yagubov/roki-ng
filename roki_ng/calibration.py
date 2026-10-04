"""Cancellable test plans with bounded posture recovery before each test."""

import math
import time

from .wire import Fault, boolean, choice, number
from .recovery import ensure_upright


ROTATION_KEYS = ("motion.jump_yaw_cw", "motion.jump_yaw_ccw",
                 "motion.rotation_yield_right", "motion.rotation_yield_left")
TITLES = {"run_test": "Ходьба", "jump_test": "Прыжки",
          "rotation_test": "Калибровка поворотов", "kick_test": "Удары",
          "get_up_test": "Вставание"}
RUN_MODES = {"short": "short_run", "long": "long_run", "spot": "spot_run",
             "backwards": "run_backwards", "side_left": "side_step_left",
             "side_right": "side_step_right", "custom": "custom"}
MEASUREMENTS = {
    "short_run": ("motion.run_10_mm",),
    "long_run": ("motion.run_20_mm",),
    "spot_run": ("motion.shift_x_mm", "motion.shift_y_mm"),
    "side_step_left": ("motion.side_left_20_mm",),
    "side_step_right": ("motion.side_right_20_mm",),
    **{f"jump_{direction}": (f"motion.jump_{direction}_mm",)
       for direction in ("forward", "backward", "left", "right")},
}
UNAVAILABLE = {"new_kick": "Ball-guided calibration scenario not implemented; use regular with motion.kick_type"}


def describe(name):
    if not isinstance(name, str) or name not in TITLES:
        raise Fault("not_found", "Unknown legacy test")
    parameters = {
        "run_test": {"mode": {"choices": list(RUN_MODES), "default": "short"},
                     "cycles": {"type": "int", "min": 1, "max": 100, "default": 10, "when": "custom"},
                     "step_mm": {"type": "float", "min": -64, "max": 64, "default": 24, "when": "custom"},
                     "side_mm": {"type": "float", "min": 0, "max": 20, "default": 0, "when": "custom"},
                     "right_leg": {"type": "bool", "default": True, "when": "custom"}},
        "jump_test": {"direction": {"choices": ["forward", "backward", "left", "right", "on_spot"],
                                    "default": "forward"},
                      "count": {"type": "int", "min": 1, "max": 100, "default": 10}},
        "rotation_test": {},
        "get_up_test": {},
        "kick_test": {"mode": {"choices": ["regular", "new_kick"], "default": "regular"}},
    }[name]
    return {"name": name, "title": TITLES[name], "parameters": parameters,
            "unavailable_modes": UNAVAILABLE if name == "kick_test" else {},
            "body_imu": True,
            "automatic_parameters": list(ROTATION_KEYS) if name == "rotation_test" else []}


def validate_args(args):
    name = choice(args, "name", None, tuple(TITLES))
    allowed = {"name", "lease_epoch"} | set(describe(name)["parameters"])
    if set(args) - allowed:
        raise Fault("invalid_argument", "Unknown test parameters")
    if name == "run_test":
        mode = choice(args, "mode", "short", tuple(RUN_MODES))
        result = {"name": name, "mode": mode}
        if mode == "custom":
            result.update(cycles=number(args, "cycles", 10, 1, 100, True),
                          step_mm=number(args, "step_mm", 24, -64, 64),
                          side_mm=number(args, "side_mm", 0, 0, 20),
                          right_leg=boolean(args, "right_leg", True))
        elif set(args) & {"cycles", "step_mm", "side_mm", "right_leg"}:
            raise Fault("invalid_argument", "Gait overrides require mode=custom")
        return result, RUN_MODES[mode]
    if name == "jump_test":
        direction = choice(args, "direction", "forward", ("forward", "backward", "left", "right", "on_spot"))
        return {"name": name, "direction": direction,
                "count": number(args, "count", 10, 1, 100, True)}, f"jump_{direction}"
    if name == "kick_test":
        mode = choice(args, "mode", "regular", ("regular", "new_kick"))
        if mode in UNAVAILABLE:
            raise Fault("not_supported", UNAVAILABLE[mode])
        return {"name": name, "mode": mode}, mode
    return {"name": name}, name


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def rotation_results(cw, ccw, right, left):
    # Do not turn noise or reversed wiring into a near-full-circle calibration.
    if not -4.5 <= cw <= -0.03 or not 0.03 <= ccw <= 4.5:
        raise Fault("calibration_invalid", "Jump yaw has wrong sign or insufficient displacement")
    if not 0.1 <= abs(right) <= 10 or not 0.1 <= abs(left) <= 10 or right * left >= 0:
        raise Fault("calibration_invalid", "Walking turns must have nonzero opposite directions")
    return dict(zip(ROTATION_KEYS, (round(cw / 3, 3), round(ccw / 3, 3),
                                    round(abs(right) / 10, 3), round(abs(left) / 10, 3))))


class TestPlan:
    def __init__(self, body, args):
        self.args, self.name = validate_args(args)
        self.body = body
        self.parameters = dict(body.parameters)
        self.engine = None
        self.yaw_epoch = None
        self.origin = None
        self.progress = 0

    def yaw(self):
        imu = self.body.body_imu
        imu.sample()
        if self.yaw_epoch is not None and self.yaw_epoch != imu.yaw_epoch:
            raise Fault("imu_discontinuous", "Body IMU continuity lost; repeat calibration")
        self.yaw_epoch = imu.yaw_epoch
        return imu.unwrapped_yaw

    def mark(self, stage):
        self.progress += 1
        job = self.body.jobs[self.body.active]
        job.update(progress=self.progress, stage=stage)
        self.body.emit("job.progress", dict(job))

    def pause(self, seconds):
        # Yield small waits so release/cancel does not wait for a long sleep.
        for _ in range(round(seconds / 0.05)):
            if self.body.stop_requested:
                return
            yield "sleep", 0.05

    def get_engine(self):
        if self.engine is None:
            from .motion.engine import Engine
            self.engine = Engine(self.parameters)
        return self.engine

    def gait(self, cycles, step=0, side=0, right_leg=True, rotation=None, ramp=False):
        if self.body.stop_requested:
            return
        engine = None if self.body.simulated else self.get_engine()
        if engine:
            engine.first_Leg_Is_Right_Leg = right_leg
            yield from engine.walk_Initial_Pose(start_mixing=False)
            yield "drain",
        else:
            yield "sleep", 0.01
        self.body.pose = "crouch"
        completed = 0
        for cycle in range(cycles):
            if self.body.stop_requested:
                break
            correction = rotation
            if not (yield from self.body.wait_for_imu()):
                break
            if correction is None:
                # All headings come from the worker's continuously unwrapped
                # polling stream, never from a synchronous request in a plan.
                measured = self.yaw()
                limit = self.parameters["walk.heading_max_correction_rad"]
                correction = max(-limit, min(limit, wrap(measured - self.origin) * self.parameters["walk.heading_kp"]
                                           * (-1 if right_leg else 1)))
            stride = step * ((cycle + 1) / 3 if ramp and cycle < 2 else 1)
            if engine:
                yield from engine.walk_Cycle(stride, side, correction, cycle, cycles)
            else:
                yield "sleep", 0.01
            yield "drain",
            completed += 1
            self.mark("walking")
        if engine:
            if completed and self.body.stop_requested:
                yield from engine.walk_Cycle(0, 0, 0, 0, 1)
            yield from engine.walk_Final_Pose()
        yield "drain",
        self.body.pose = "stand"
        # This private engine cannot leak calibration baselines into normal gait.
        self.body.engine = None

    def jump(self, direction, fraction=1, calibration=False):
        if self.body.stop_requested:
            return
        rows = self.body._jump_rows(direction, fraction)
        if calibration:
            rows.append([10] + [0] * 21)
        yield from self.body._slot(rows, legs_only=direction.startswith("turn_"))
        self.mark(direction)

    def correct_course(self):
        for _ in range(20):
            if self.body.stop_requested:
                return
            if not (yield from self.body.wait_for_imu()):
                return
            error = wrap(self.origin - self.yaw())
            if abs(error) < 0.09:
                return
            direction = "turn_left" if error > 0 else "turn_right"
            full = abs(self.parameters["motion.jump_yaw_ccw" if error > 0 else "motion.jump_yaw_cw"])
            yield from self.jump(direction, min(1, abs(error) / full))
            yield from self.pause(0.5)
        raise Fault("course_not_reached", "Jump correction exceeded 20 attempts")

    def run(self):
        name = self.name
        yield from ensure_upright(self.body)
        if self.body.stop_requested or name == "get_up_test":
            return
        if name == "regular":
            yield from self.body._kick("right", 100, 0)
            return
        # Validate the body IMU before issuing even a head or pose command.
        if not (yield from self.body.wait_for_imu()):
            return
        self.origin = self.yaw()
        if not self.body.simulated and not name.startswith("jump_"):
            self.get_engine()
        tilt = -2000 if name.startswith("jump_") else self.parameters["head.field_tilt"]
        yield from self.body._head(0, tilt)
        if self.body.stop_requested:
            return
        if name.startswith("rotation_"):
            yield from self.rotation()
        elif name.startswith("jump_"):
            direction = name[5:] if name != "jump_on_spot" else "forward"
            for _ in range(self.args["count"]):
                if self.body.stop_requested:
                    break
                yield from self.jump(direction, 0 if name == "jump_on_spot" else 1)
                yield from self.pause(0.5)
                yield from self.correct_course()
                yield from self.pause(0.5)
        elif name.startswith("side_step_"):
            yield from self.gait(20, side=20, right_leg=name.endswith("right"))
        elif name == "custom":
            yield from self.gait(self.args["cycles"], step=self.args["step_mm"],
                                side=self.args["side_mm"], right_leg=self.args["right_leg"], ramp=True)
        else:
            cycles = 11 if name == "short_run" else 21
            step = {"spot_run": 0, "run_backwards": -50}.get(name, 64)
            yield from self.gait(cycles, step=step, ramp=True)

    def rotation(self):
        yield from self.pause(1)
        deltas = []
        for direction in ("turn_right", "turn_left"):
            if not (yield from self.body.wait_for_imu()):
                return
            initial = self.yaw()
            for _ in range(3):
                if self.body.stop_requested:
                    return
                yield from self.jump(direction, calibration=True)
                yield from self.pause(1)
            yield from self.pause(2)
            if not (yield from self.body.wait_for_imu()):
                return
            deltas.append(self.yaw() - initial)
        self.parameters.update({"motion.rotation_yield_right": 0.23,
                                "motion.rotation_yield_left": 0.23})
        if self.engine:
            self.engine.configure(self.parameters)
        for right in (True, False):
            if self.body.stop_requested:
                return
            if not (yield from self.body.wait_for_imu()):
                return
            initial = self.yaw()
            # This is deliberately -0.23 in both passes.  The gait engine
            # reverses the physical turn when its first support leg changes;
            # this matches the original rotation_test.  Reversing the value
            # here as well makes both measured turns have the same yaw sign.
            yield from self.gait(10, right_leg=right, rotation=-0.23)
            yield from self.pause(2)
            if not (yield from self.body.wait_for_imu()):
                return
            deltas.append(self.yaw() - initial)
        if not self.body.stop_requested:
            result = rotation_results(*deltas)
            self.parameters.update(result)
            if self.engine:
                self.engine.configure(self.parameters)
            # Original persists the complete measurements BEFORE the demo turns.
            # Wait for supervisor's durable-save acknowledgement before moving.
            ident = self.body.active
            job = self.body.jobs[ident]
            job.update(calibration=result, status='saving', stage='saving_calibration')
            self.body.calibration_pending = ident
            self.body.emit('job.progress', dict(job))
            self.body.emit('calibration.ready', {'job_id': ident, 'values': result})
            deadline = time.monotonic() + 10
            while self.body.calibration_pending == ident:
                if self.body.stop_requested:
                    return
                if time.monotonic() > deadline:
                    raise Fault('calibration_invalid', 'Calibration save acknowledgement timed out')
                yield 'sleep', .05
            if self.body.stop_requested:
                return
            yield from self.turn_to_course(self.origin + 2 * math.pi / 3)
            yield from self.turn_to_course(self.origin)

    def turn_to_course(self, target):
        if self.body.stop_requested:
            return
        if not (yield from self.body.wait_for_imu()):
            return
        error = wrap(target - self.yaw())
        if abs(error) < 0.035:
            return
        cycles = math.floor(abs(error) / 0.23) + 1
        right = error <= 0
        engine = None if self.body.simulated else self.get_engine()
        if engine:
            engine.first_Leg_Is_Right_Leg = right
            yield from engine.walk_Initial_Pose(start_mixing=False)
            yield "drain",
        completed = 0
        for cycle in range(cycles):
            if self.body.stop_requested:
                break
            if not (yield from self.body.wait_for_imu()):
                break
            error = wrap(target - self.yaw())
            if abs(error) < 0.035:
                break
            rotation = max(-0.3, min(0.3, error / (cycles - cycle))) * (1 if right else -1)
            if engine:
                yield from engine.walk_Cycle(0, 0, rotation, cycle, cycles)
            else:
                yield "sleep", 0.01
            yield "drain",
            completed += 1
            self.mark("verify_course")
        if engine:
            if self.body.stop_requested and completed < cycles and completed:
                yield from engine.walk_Cycle(0, 0, 0, 0, 1)
            yield from engine.walk_Final_Pose()
            engine.first_Leg_Is_Right_Leg = True
        yield "drain",
        self.body.pose = "stand"
        self.body.engine = None
        if not (yield from self.body.wait_for_imu()):
            return
        if not self.body.stop_requested:
            actual = self.yaw()
            error = wrap(target - actual)
            job = self.body.jobs[self.body.active]
            job['verification'] = dict(target_rad=target, actual_rad=actual, error_rad=error)
            if abs(error) >= .09:
                self.body.log('WARNING', f'Calibration saved; verification turn residual {math.degrees(error):.1f} deg')
            self.body.emit('job.progress', dict(job))
