"""Cooperative gait generator; no camera, filesystem calibration or global app."""

from .gait import GaitAlgorithms
from .model import Robot
from ..wire import Fault


class Engine(Robot, GaitAlgorithms):
    def __init__(self, parameters):
        Robot.__init__(self)
        self.configure(parameters)
        self.simThreadCycleInMs = 20
        self.initPoses = 20
        self.first_Leg_Is_Right_Leg = True
        self.keep_hands_up = False
        self.falling_Flag = self.exitFlag = 0
        self.kick_power = 80
        self.xtr = self.xtl = self.xr = self.xl = self.yr = self.yl = 0
        self.wr = self.wl = 0
        self.zr = self.zl = -1
        self.ytr, self.ytl = -self.d10, self.d10
        self.ztr = self.ztl = -self.gaitHeight

    def configure(self, values):
        self.frames_per_cycle = values["motion.frames_per_cycle"]
        self.motion_shift_correction_x = -values["motion.shift_x_mm"] / 21
        self.motion_shift_correction_y = -values["motion.shift_y_mm"] / 21
        self.params = {
            "BODY_TILT_AT_WALK": values["motion.body_tilt"],
            "BODY_TILT_AT_WALK_BACKWARDS": values["motion.body_tilt_back"],
            "BODY_TILT_AT_KICK": values.get("motion.body_tilt_kick", 0),
            "SOLE_LANDING_SKEW": values["motion.sole_skew"],
            "ROTATION_YIELD_RIGHT": values["motion.rotation_yield_right"],
            "ROTATION_YIELD_LEFT": values["motion.rotation_yield_left"],
        }

    def falling_Test(self):
        # Recovery is explicit in the worker. Never launch a get-up slot implicitly.
        return 0

    def computeAlphaForWalk(self, hands_on=True):
        result = super().computeAlphaForWalk(hands_on)
        if not result:
            raise Fault("inverse_kinematics", "No valid leg solution; movement stopped")
        return result
