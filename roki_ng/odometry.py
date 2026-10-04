"""Calibrated estimates, applied only to acknowledged motion segments."""
import math

from .calibration import wrap


def walk_delta(parameters, step_mm=0, side_mm=0, *, first=False, right_first=True):
    short, long = parameters['motion.run_10_mm'], parameters['motion.run_20_mm']
    forward = (19 * short - 9 * long) / 10 if first else (long - short) / 10
    side = parameters['motion.side_right_20_mm' if right_first else 'motion.side_left_20_mm'] / 20
    return forward * step_mm / 64 / 1000, side * side_mm / 20 / 1000 * (-1 if right_first else 1)


def jump_delta(parameters, direction, fraction):
    if direction.startswith('turn_'):
        return 0., 0.
    distance = parameters[f'motion.jump_{direction}_mm'] * fraction / 1000
    return {'forward': (distance, 0.), 'backward': (-distance, 0.),
            'left': (0., distance), 'right': (0., -distance)}[direction]


class GameOdometry:
    def __init__(self, parameters, entry, imu_yaw):
        own = parameters[f"field.goal.{parameters['match.own_goal']}"]
        other = parameters[f"field.goal.{1 - parameters['match.own_goal']}"]
        self.home_heading = 0. if own['x'] < other['x'] else math.pi
        sign = 1 if self.home_heading == 0 else -1
        self.home_x, self.home_y = own['x'] + sign * .18, own['y']
        geometry = parameters['field.geometry']
        if entry == 'goalkeeper':
            self.x, self.y, self.yaw = self.home_x, self.home_y, self.home_heading
        else:
            x, y, heading = -.45, 0., 0.
            if entry in ('left', 'right'):
                x = -geometry['length'] / 2 + .87
                y = geometry['width'] / 2 * (1 if entry == 'left' else -1)
                heading = -math.pi / 2 if entry == 'left' else math.pi / 2
            self.x, self.y, self.yaw = sign * x, sign * y, wrap(heading + self.home_heading)
        self.yaw_offset = wrap(self.yaw - imu_yaw)
        self.path = 0.
        self.confidence = 'odometry'
        self.reason = None
        self.applied = {}

    def heading(self, imu_yaw):
        self.yaw = wrap(imu_yaw + self.yaw_offset)

    def uncertain(self, reason):
        self.confidence, self.reason = 'degraded', reason

    def correct(self, pose, imu_yaw):
        self.x, self.y, self.yaw = pose
        self.yaw_offset = wrap(self.yaw-imu_yaw)
        self.confidence, self.reason = 'visual', None

    def apply(self, job_id, job):
        previous = self.applied.get(job_id, 0)
        for segment in job.get('segments', ()):
            if segment['index'] <= previous:
                continue
            angle = wrap(segment['imu_yaw_rad'] + self.yaw_offset)
            dx, dy = segment['dx_m'], segment['dy_m']
            self.x += math.cos(angle) * dx - math.sin(angle) * dy
            self.y += math.sin(angle) * dx + math.cos(angle) * dy
            self.path += math.hypot(dx, dy)
            previous = segment['index']
        self.applied[job_id] = previous
        # Jobs are serial. Retain only the latest few replay barriers.
        if len(self.applied) > 16:
            del self.applied[next(iter(self.applied))]
        if job.get('execution_uncertain'):
            self.uncertain(job.get('reason') or 'unfinished_motion')

    def state(self):
        return dict(x_m=self.x, y_m=self.y, yaw_rad=self.yaw,
                    confidence=self.confidence, reason=self.reason)
