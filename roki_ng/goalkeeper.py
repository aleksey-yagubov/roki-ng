"""Finite FIRA corrections in field coordinates, using calibrated yields."""
import math
import time


HEAD_TICK_RAD = .00058909

# Nominal motion limits, not a guarantee against foot slip or pose error.
KEEPER_STEP_M = .10
KEEPER_POST_MARGIN_M = .10


def keeper_target(parameters, position, ball):
    """Hold goal depth; clip the lateral target to the inside of the posts."""
    own = parameters[f"field.goal.{parameters['match.own_goal']}"]
    half_width = max(0., own['width']/2-KEEPER_POST_MARGIN_M)
    low, high = own['y']-half_width, own['y']+half_width
    target_y = (position.y + math.sin(position.yaw)*ball['x_m']
                + math.cos(position.yaw)*ball['y_m']) if ball else position.home_y
    target_y = max(low, min(high, target_y))
    # Deadband must not prevent returning from outside the allowed zone.
    if low <= position.y <= high and ball and abs(target_y-position.y) <= .1:
        target_y = position.y
    target_x = position.home_x
    if abs(target_x-position.x) <= .05:
        target_x = position.x
    return target_x, target_y


class KeeperMotionGuard:
    """Only stop an in-flight job; a new target always needs a standing look."""
    def __init__(self, position, ball, target):
        self.target = target
        self.direction = (target[0]-position.x, target[1]-position.y)
        self.ball_y = (position.y + math.sin(position.yaw)*ball['x_m']
                       + math.cos(position.yaw)*ball['y_m']) if ball else None
        self.sequence = ball.get('frame_sequence', -1) if ball else -1
        self.last_seen = time.monotonic()
        self.shifted = 0
        self.previous_shift = None

    def reason(self, observed, position):
        dx, dy = self.direction
        if (self.target[0]-position.x)*dx+(self.target[1]-position.y)*dy <= 0:
            return 'target_reached'
        if self.ball_y is None:
            return None  # Returning home does not depend on seeing a ball.
        now = time.monotonic()
        result, age = observed.get('result') or {}, observed.get('age_ms')
        sequence = result.get('frame_sequence', -1)
        valid = (not observed.get('error') and observed.get('running') and result.get('valid')
                 and type(age) in (int, float) and 0 <= age <= 500
                 and type(sequence) is int and sequence > self.sequence
                 and all(type(result.get(k)) in (int, float) and math.isfinite(result[k])
                         for k in ('x_m', 'y_m')))
        if valid:
            self.sequence = sequence
            self.last_seen = now-age/1000
            ball_y = position.y+math.sin(position.yaw)*result['x_m']+math.cos(position.yaw)*result['y_m']
            shift = ball_y-self.ball_y
            agrees = self.previous_shift is not None and abs(shift-self.previous_shift) <= .10
            self.shifted = self.shifted+1 if abs(shift) > .12 and agrees else int(abs(shift) > .12)
            self.previous_shift = shift
            if self.shifted >= 2:
                return 'ball_shifted'
        if now-self.last_seen > .75:
            return 'ball_not_fresh'
        return None


def ball_in_body(result, pan):
    """Rotate the head-level projection using the acknowledged stationary pan."""
    if not result.get('valid'):
        return result
    x, y = result.get('x_m'), result.get('y_m')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)):
        return result | {'valid': False, 'reason': 'invalid_ball_geometry'}
    angle = -pan * HEAD_TICK_RAD
    c, s = math.cos(angle), math.sin(angle)
    return result | {'x_m': c*x-s*y, 'y_m': s*x+c*y}
