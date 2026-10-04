"""Experimental visual keeper: no odometry, no forward pursuit, no kicks."""
import math
import time

from .calibration import wrap
from .simple_football import SimpleFootball


class SimpleGoalkeeper(SimpleFootball):
    goal_bearing_only = True

    def __init__(self, game, parameters, entry, delay):
        super().__init__(game, parameters, entry, delay)
        self.kickoff_pending = self.entry_pending = False
        self.heading = None
        self.reacquire_heading = False
        self.goal_at = self.scan_at = 0.
        self.goal_offset = None
        self.live_sequence = -1
        self.live_ball = None
        self.live_at = 0.
        self.ball_speed = 0.
        self.move_ball = None
        self.motion_changed = False
        self.track_deadline = 0.

    def motion_fresh(self, op, args):
        if op == 'game.jump' and args.get('direction') in ('left', 'right'):
            if time.monotonic() >= self.track_deadline:
                self.describe('keeper_hold', 'ball expired before lateral jump; reobserve')
                return False
        return True

    def cancel_special(self, reason):
        super().cancel_special(reason)
        self.heading = None
        self.reacquire_heading = True
        self.goal_at = self.scan_at = 0.
        self.goal_offset = self.live_ball = None

    def observe_motion_ball(self, observation):
        """Consume detector updates while the finite jump runs; never chain jumps."""
        result = observation.get('result') or {}
        age, seq = observation.get('age_ms'), result.get('frame_sequence')
        valid = (observation.get('running') and not observation.get('error') and result.get('valid')
                 and type(age) in (int, float) and 0 <= age <= 500
                 and type(seq) is int and seq > self.live_sequence
                 and all(type(result.get(k)) in (int, float) and math.isfinite(result[k])
                         for k in ('x_m', 'y_m')))
        if valid:
            point = (result['x_m'], result['y_m'])
            stamp = time.monotonic()-age/1000
            if self.live_ball is not None and stamp > self.live_at:
                self.ball_speed = math.dist(point, self.live_ball)/(stamp-self.live_at)
            self.live_ball, self.live_at, self.live_sequence = point, stamp, seq
            changed = self.move_ball is not None and abs(point[1]-self.move_ball[1]) > .10
        else:
            changed = time.monotonic()-self.live_at > .5
        if changed and self.move_ball is not None and not self.motion_changed:
            self.motion_changed = True
            self.g.s.log('game', 'INFO', 'Simple keeper: ball shifted/lost during jump; finish this jump, reobserve before next')

    async def step(self):
        revision = self.g.revision
        if not await self.g._prepare(self.p, self.entry) or not self.g._same_plan(revision):
            return
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        if self.heading is None:
            self.heading = yaw  # Operator places keeper facing the opponent goal.
        observed = await self.g._call('detection', 'ball.status')
        if not self.g._same_plan(revision):
            return
        self.observe_motion_ball(observed)
        # Look down first: a close or moving ball has priority over a goal scan.
        scene = await self.look(0, self.p['game.head_tilt'])
        if not self.g._same_plan(revision):
            return
        ball = scene.get('ball') if scene else None
        urgent = ball is not None and (math.hypot(*ball) < .7 or self.ball_speed > .15)
        if time.monotonic()-self.scan_at >= 4 and (not urgent or self.reacquire_heading):
            self.scan_at = time.monotonic()
            self.describe('keeper_goal_scan', 'head up; colour bearing only')
            goal_scene = await self.look(0, 0)
            if not self.g._same_plan(revision):
                return
            hints = [p for p in (goal_scene or {}).get('parts', [])
                     if p['colour'] == self.enemy and p['kind'] == 'colour']
            if hints:
                hint = max(hints, key=lambda p: p['area'])
                bearing = math.atan2(hint['points'][0][1], hint['points'][0][0])
                yaw, _ = await self.g._imu()
                if not self.g._same_plan(revision):
                    return
                if self.reacquire_heading:
                    self.heading = wrap(yaw+bearing)
                    self.reacquire_heading = False
                # Compass bearing relative to the initial facing direction.
                # A single post/colour patch is not a metric goal centre.
                self.goal_offset = wrap(yaw+bearing-self.heading)
                self.goal_at = time.monotonic()
                self.g.s.log('game', 'INFO', f'Simple keeper: enemy colour bearing={math.degrees(self.goal_offset):.1f}deg')
            scene = await self.look(0, self.p['game.head_tilt'])
            if not self.g._same_plan(revision):
                return
            ball = scene.get('ball') if scene else None  # Never reuse pre-scan ball.
        if self.reacquire_heading:
            self.describe('keeper_hold', 'recovery: need opponent colour to restore heading')
            return
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        error = wrap(self.heading-yaw)
        if abs(error) > math.radians(15):
            self.describe('keeper_heading', 'restore starting course by IMU')
            await self.turn(max(-.35, min(.35, error)))
            return
        if ball is None:
            self.describe('keeper_hold', 'no fresh ball; no blind lateral pursuit')
            return
        x, y = ball
        self.g.info['ball'] = dict(x_m=x, y_m=y)
        # A distant colour bearing can veto outward movement, but cannot prove
        # that the robot remains between its own posts. No fake field position.
        if (time.monotonic()-self.goal_at < 8 and self.goal_offset is not None
                and abs(self.goal_offset) > math.radians(6) and y*self.goal_offset < 0):
            self.describe('keeper_hold', 'enemy bearing vetoes further outward displacement')
            return
        if x <= 0 or abs(y) <= .10:
            self.describe('keeper_hold', 'ball centred or behind; no forward movement')
            return
        direction = 'left' if y > 0 else 'right'
        fraction = min(1., 20/max(1., self.p[f'motion.jump_{direction}_mm']))
        self.move_ball, self.motion_changed = ball, False
        self.track_deadline = scene['received']+.5
        self.describe('keeper_track', f'ball x={x*1000:.0f} y={y*1000:.0f}mm; one {direction} jump, nominal <=20mm')
        await self.jump(direction, fraction)
        self.move_ball = None
