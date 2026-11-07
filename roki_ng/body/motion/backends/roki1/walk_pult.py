"""Experimental normal walk from walk_pult.cpp v3.0 (04.11.2026).

Only the walking phases are ported. No controller slots, kicks, jumps or IMU
driver changes. 10 ms Zubr frames: interpolate 8, submit every 2; the transport
must have a 20 ms queue period. Cartesian trajectories mirror the source;
native Starkit IK and the existing Roki1 servo mapping remain in use.
"""
import math

from roki_ng.body.motion.engine import Engine
from roki_ng.wire import Fault


class PultWalk(Engine):
    pult_walk = True

    def configure(self, values):
        super().configure(values)
        self.gaitHeight, self.stepHeight, self.amplitude = 165., 20., 32.

    def __init__(self, parameters, log=None, *, backend):
        if backend.NAME != 'roki1':
            raise Fault('not_supported', 'Experimental walk_pult requires Roki1')
        if parameters['motion.frame_ms'] != 20:
            raise Fault('invalid_argument', 'Experimental walk_pult requires motion.frame_ms=20')
        super().__init__(parameters, log, backend=backend)
        self.gaitHeight, self.stepHeight, self.amplitude = 165., 20., 32.
        self.fr1, self.fr2 = 8, 12
        self.ztr0 = self.ztl0 = -231.
        # Source sfIk limits, converted to Starkit's 1698 ticks/radian units.
        self.LIMALPHA = [[math.radians(lo)*1698, math.radians(hi)*1698]
                        for lo, hi in ((-150, 150), (-90, 17), (-97, 103),
                                       (-158, 32), (-122, 83), (-83, 20))]
        self.last_order = (0., 0., 0.)
        self.arms_up = True

    def solve_leg(self, leg, *args):
        import starkit
        result = starkit.alpha_calculation(*args)
        if not result:
            self.log('WARNING', f'Walk_pult IK skip {leg}: xyz={tuple(round(v, 2) for v in args[:3])}; '
                     'previous leg target retained, walking continues')
        return result

    def frame(self, frames=8, pause=0):
        values = []
        for leg, xyz, orientation, yaw, logical_right in (
                ('right', (self.xtr, self.ytr, self.ztr), (self.xr, self.yr, self.zr), self.wr, True),
                ('left', (self.xtl, -self.ytl, self.ztl), (self.xl, -self.yl, self.zl), self.wl, False)):
            solutions = self.solve_leg(leg, *xyz, *orientation, yaw, self.SIZES, self.LIMALPHA)
            if not solutions:
                self.exitFlag += 1
                if frames == 40:
                    raise Fault('invalid_motion', 'Experimental walk_pult: unreachable transition pose')
                continue  # Source keeps the previous target of this leg only.
            angles = list(min(solutions, key=lambda a: a[2]))
            if not self.first_Leg_Is_Right_Leg:
                angles[5] = -angles[5]  # Source changes both pelvic signs.
            physical_right = logical_right == self.first_Leg_Is_Right_Leg
            pose = [0.]*21
            start = 0 if physical_right else 11
            pose[start:start+6] = [a if physical_right else -a for a in angles]
            targets = self.mapping.targets(self, pose, scale=1698, legs_only=True)
            values.extend(v for v in targets if v.Sio == (1 if physical_right else 2))
        pose = [0.]*21
        if self.arms_up or self.keep_hands_up:
            pose[6], pose[17] = 1.973, -1.973
            right_x, left_x = (self.xtr, self.xtl) if self.first_Leg_Is_Right_Leg else (self.xtl, self.xtr)
            pose[9], pose[20] = -.147-left_x/114, .147+right_x/114
        values.extend(v for v in self.mapping.targets(self, pose, scale=1698)
                      if v.Id in (1, 2, 3, 4))
        if values:
            # Kondo wire frames are 20 ms; source sfPoseGroupLin frames are
            # 10 ms. Zubr multiplies Kondo frames by CS_STEP_PER_SECOND/50.
            yield 'servo', values, frames//2, pause
        else:
            yield 'sleep', .02

    def initial(self):
        self.arms_up = True
        self.xtr = self.xtl = 0.
        self.ztr = self.ztl = -165.
        self.ytr, self.ytl = -self.d10-16., self.d10-16.
        yield from self.frame(40, 19)
        yield 'drain',

    def final(self):
        self.arms_up = False
        self.xtr = self.xtl = 0.
        self.ztr = self.ztl = -231.
        self.ytr, self.ytl = -self.d10, self.d10
        self.xr = self.xl = 0.
        yield from self.frame(40, 19)
        yield 'drain',

    def centered_crouch(self):
        self.arms_up = True
        self.xtr = self.xtl = self.xr = self.xl = self.wr = self.wl = 0.
        self.yr, self.yl = -.01, .01
        self.ztr = self.ztl = -165.
        self.ytr, self.ytl = -self.d10, self.d10
        yield from self.frame(40, 19)
        yield 'drain',

    def walk_Initial_Pose(self, amplitude=24, start_mixing=True):
        yield from self.initial()

    def walk_Final_Pose(self, respect_body_tilt=False):
        yield from self.final()

    def walk_Cycle(self, stepLength, sideLength, rotation, cycle, number_Of_Cycles, half=False):
        # Preserve the nominal 64/20 command and calibration contract used by
        # recipes/tests, but execute the shorter pult stride at full command.
        step = max(-64., min(64., stepLength)) / 64 * (25. if stepLength >= 0 else 20.)
        side = max(0., min(20., sideLength)) / 20 * (18. if self.first_Leg_Is_Right_Leg else 24.)
        kind = 'first' if cycle == 0 else 'last' if cycle == number_Of_Cycles-1 else 'other'
        if cycle == 0 and number_Of_Cycles == 1:
            # Existing callers use this as a terminal settle cycle.
            kind = 'last'
        yield from self.cycle(step, side, max(-.7,min(.7,rotation)), kind, half)

    def cycle(self, step, side, rotation, kind='other', half=False):
        self.last_order = (step, side, rotation)
        step -= 50./21.
        side -= 70./21.
        half_side = side/2
        turn = -rotation*.25  # Source yields are both .23.
        xtl0 = step*(1-24/32)*1.5
        xtr0 = step*(.5-24/32)
        dx0_typical, dy0 = step/32*2, side/24*2
        old = self.xr, self.xl, self.yr, self.yl
        self.xr = self.xl = 0.
        self.yr, self.yl = -.01, .01
        alpha = 2*math.pi/8
        self.ztr = self.ztl = -165.
        try:
            # Phase 1: both feet down; the first cycle skips its output.
            for j in range(0, 8, 2):
                s = (16+half_side)*math.cos(alpha*(j/2+.7*2))
                self.ytr, self.ytl = s-self.d10+half_side, s+self.d10+half_side
                if kind == 'first':
                    continue
                self.xtl = xtl0-dx0_typical-dx0_typical*j/2
                self.xtr = xtr0-dx0_typical-dx0_typical*j/2
                yield from self.frame()
            # Phase 2: right (logical leading) foot in the air.
            self.ztr = -145.
            dy, turn_step = side*2/8, turn/8*2
            dx = step/8*2/(2 if kind == 'first' else 1)
            for j in range(0, 12, 2):
                if j == 0:
                    self.xtr -= dx0_typical
                    self.ytr = s-self.d10+dy0
                elif j == 10:
                    self.xtr -= dx0_typical
                    self.ytr = s-self.d10+2*dy0-side
                else:
                    self.xtr += dx
                    self.ytr = s-64+dy0-dy*(j/2)
                    self.wr = self.wl = turn-j*turn_step
                self.xtl -= dx0_typical
                self.ytl += dy0
                yield from self.frame()
            if half:
                return
            # Phase 3: both feet down.
            self.ztr = self.ztl = -165.
            for j in range(0, 8, 2):
                s = (16+half_side)*math.cos(alpha*((j-12)/2+.7*2))
                self.ytr, self.ytl = s-self.d10-half_side, s+self.d10+half_side
                self.xtl -= dx0_typical
                self.xtr -= dx0_typical
                yield from self.frame()
            # Phase 4: left foot in the air; special terminal stride.
            self.ztl = -145.
            if kind == 'last':
                dx0 = dx0_typical*4/12
                dx = (step*20/32+2*dx0)/8*2/1.23076941
            else:
                dx, dx0 = step/8*2, dx0_typical
            for j in range(0, 12, 2):
                if kind == 'last' and j == 10:
                    self.ztl = -165.
                    self.ytl = s+self.d10
                if j == 0:
                    self.xtl -= dx0
                    self.ytl = s+64+half_side
                elif j == 10:
                    self.xtl -= dx0
                    self.ytl = s+self.d10+half_side
                else:
                    self.xtl += dx
                    self.ytl = s+64+half_side
                    self.wr = self.wl = j*turn_step-turn
                self.xtr -= dx0
                self.ytr += dy0
                yield from self.frame()
        finally:
            self.xr, self.xl, self.yr, self.yl = old


def drive_order(drive):
    """Source fixed joystick directions, with normalized GUI inputs."""
    x, y = drive['x'], drive['y']  # GUI positive Y is left.
    step = (25. if x > 0 else -20.) if abs(x) >= .1 and abs(x) > abs(y) else 0.
    side = (24. if y > 0 else 18.) if abs(y) >= .1 and abs(y) > abs(x) else 0.
    right = None if not side else y < 0
    turn = math.copysign(.15, drive['yaw']) if abs(drive['yaw']) > .1 else 0.
    return step, side, right, turn
