"""Original FIRA/forward decisions, adapted to supervisor-owned async jobs.

Source: Roki_2_Soccer/Soccer/strategy.py and Motion/class_Motion_real.py.
Keep cancellation/freshness barriers at every physical action. Never preserve a
route across pause, pickup, fall or an uncertain command result.
"""
import asyncio
import json
import math
import random
import time
from pathlib import Path

from .calibration import wrap
from .wire import Fault
from .goalkeeper import keeper_target, KeeperMotionGuard, KEEPER_STEP_M


TABLE = json.loads((Path(__file__).parent/'assets/strategy_data_FIRA.json').read_text())['strategy_data']


def kick_target(parameters, position, ball):
    c, s = math.cos(position.yaw), math.sin(position.yaw)
    x = position.x+c*ball['x_m']-s*ball['y_m']
    y = position.y+s*ball['x_m']+c*ball['y_m']
    sign = 1 if position.home_heading == 0 else -1
    length, width = (parameters['field.geometry'][k] for k in ('length', 'width'))
    col = max(0, min(19, math.floor((sign*x+length/2)/(length/20))))
    row = max(0, min(14, math.floor((-sign*y+width/2)/(width/15))))
    _, _, power, angle = TABLE[col*15+row]
    # Glob.import_strategy_data stores signed integer angle*40, not the float.
    return wrap(int(angle*40)/40+position.home_heading), {1: 100, 2: 60, 3: 20}[power]


class OriginalStrategy:
    def _same_plan(self, revision):
        return self._allowed() and self.revision == revision

    async def _settle(self, seconds):
        revision = self.revision
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if not self._allowed() or self.revision != revision:
                return False
            await self._poll_localisation()
            await asyncio.sleep(min(.05, max(0., deadline-time.monotonic())))
        return self._allowed() and revision == self.revision

    async def _original_turn(self, target, *, for_kick=False, for_search=False, for_approach=False):
        revision = self.revision
        # Search only needs a new viewing sector; precise alignment causes
        # needless direction reversals when a small jump overshoots the target.
        tolerance = math.radians(15) if for_kick or for_search or for_approach else .09
        previous_yaw = None
        stagnant = 0
        self.s.log('game', 'INFO', f'Turn: target={math.degrees(target):.0f} deg, tolerance={math.degrees(tolerance):.1f} deg, search={for_search}')
        for _ in range(20):
            if not self._same_plan(revision):
                return False
            await self._imu()
            if not self._same_plan(revision):
                return False
            error = wrap(target-self.odometry.yaw)
            self.s.log('game', 'INFO', f'Turn: yaw={math.degrees(self.odometry.yaw):.1f} target={math.degrees(target):.1f} error={math.degrees(error):.1f} deg, step={_}')
            if abs(error) <= tolerance:
                return True
            if previous_yaw is not None:
                stagnant = stagnant + 1 if abs(wrap(self.odometry.yaw-previous_yaw)) < math.radians(2) else 0
                if stagnant >= 3:
                    self.s.log('game', 'WARNING', 'Turn has no measured progress for three jumps; reobserve and continue game')
                    return False
            previous_yaw = self.odometry.yaw
            p = self.s.params.values
            gain = abs(p['motion.jump_yaw_ccw' if error > 0 else 'motion.jump_yaw_cw'])
            if not await self._move('game.jump', {'direction': 'turn_left' if error > 0 else 'turn_right',
                                                'fraction': max(.1, min(1., abs(error)/gain))}):
                return False
            if not await self._settle(.5):
                return False
        # Unlike legacy fall-through, a failed turn cannot authorize a kick.
        self.s.log('game', 'WARNING', 'Turn did not converge; reobserve and continue game')
        return False

    async def _original_observe(self, p, entry, *, settle=1.):
        revision = self.revision
        if not await self._prepare(p, entry) or not self._same_plan(revision):
            return None
        if not await self._move('game.head', {'pan': 0, 'tilt': p['game.head_tilt']}):
            return None
        await self._call('detection', 'ball.geometry', {'height_m': p['game.camera_height_m'], 'stamp': time.monotonic()})
        if not self._same_plan(revision):
            return None
        await self._open_localisation(p)
        if not self._same_plan(revision) or not await self._settle(settle) or not self._same_plan(revision):
            return None
        self.observation_after = self._capture_now()  # Ball must be observed after settling.
        deadline = time.monotonic()+1.
        while self._same_plan(revision):
            observed = await self._ball()
            if not self._same_plan(revision):
                return None
            if observed['result'].get('reason') != 'awaiting_support_pose_frame':
                if self._usable(observed) is None:
                    self.s.log('game', 'INFO', f"Observe {self.info.get('phase')}: unusable ball, reason={observed['result'].get('reason')} age_ms={observed.get('age_ms')} error={observed.get('error')}")
                return observed
            if time.monotonic() >= deadline:
                self.s.log('game', 'WARNING', f"Observe {self.info.get('phase')}: no fresh support-pose frame within 1 s")
                return None
            if not await self._settle(.02):
                return None
        return None

    async def _original_walk(self, p, entry, kind, ball=None, **options):
        revision = self.revision
        if not await self._prepare(p, entry) or not self._same_plan(revision):
            return False
        yaw, _ = await self._imu()
        if not self._same_plan(revision):
            return False
        args = dict(kind=kind, yaw_offset=wrap(self.odometry.yaw-yaw-self.odometry.home_heading))
        args.update(options)
        if ball:
            args.update(x_m=ball['x_m'], y_m=ball['y_m'])
            # Capture timestamp, not the receipt of an old observation.
            args['stamp'] = time.monotonic()-(self._capture_now()-ball['sensor_timestamp_ns'])/1e9
            if self.keeper_watch and not 0 <= time.monotonic()-args['stamp'] <= .5:
                self.info['reason'] = 'keeper_ball_expired_before_walk'
                self.s.log('game', 'INFO', 'Goalkeeper: ball expired before walk; reobserve')
                return False
        return await self._move('game.walk', args, monitor_ball=kind in ('approach', 'around', 'verify'))

    def _capture_now(self):
        return time.clock_gettime_ns(time.CLOCK_BOOTTIME)

    @staticmethod
    def _usable(observed):
        if not observed or not observed.get('running') or observed.get('error'):
            return None
        result, age = observed.get('result') or {}, observed.get('age_ms')
        if not result.get('valid') or type(age) not in (int, float) or not 0 <= age <= 500:
            return None
        x, y = result.get('x_m'), result.get('y_m')
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in (x, y)) or x <= 0:
            return None
        return result

    async def _original_kick(self, revision, target, power, ball, x, y, *, reason):
        """Execute the original offset-capable kick from a fresh observation."""
        await self._imu()
        if not self._same_plan(revision) or abs(wrap(target-self.odometry.yaw)) > math.radians(15):
            self.s.log('game', 'INFO', f'Kick withheld: plan_active={self._same_plan(revision)}, yaw={math.degrees(self.odometry.yaw):.1f}, target={math.degrees(target):.1f} deg')
            return False
        self.last_action_sequence = ball['frame_sequence']
        self.info.update(state='kicking', phase='kick')
        offset = max(-60, min(80, int(abs(y))-62))
        leg = 'right' if y < 0 else 'left'
        self.s.log('game', 'INFO',
                   f'Forward: KICK {leg}, power={power}, offset={offset} mm ({reason}); x={x:.0f} y={y:.0f} mm')
        if not await self._move('motion.kick', {'leg': leg, 'power': power, 'offset': offset}):
            return False
        self.s.log('game', 'INFO', 'Kick motion completed')
        await self._settle(1.)
        return True

    async def _original_keeper(self, p, entry):
        revision = self.revision
        self.info.update(state='orienting', phase='restore_heading')
        if not await self._original_turn(self.odometry.home_heading, for_approach=True) or not self._same_plan(revision):
            return
        observed = await self._original_observe(p, entry)
        if not self._same_plan(revision):
            return
        ball = self._usable(observed)
        self.info.update(state='tracking', phase='goalkeeper', decision='hold', reason='no_ball' if ball is None else 'ball_centered')
        target = keeper_target(p, self.odometry, ball)
        dx, dy = target[0]-self.odometry.x, target[1]-self.odometry.y
        distance = math.hypot(dx, dy)
        if distance < .02:
            self.s.log('game', 'INFO', f"Goalkeeper: hold; {self.info['reason']}, target=({target[0]:.2f},{target[1]:.2f})m")
            return
        if ball:
            self.last_action_sequence = ball['frame_sequence']
        course = wrap(math.atan2(dy, dx)-self.odometry.yaw)
        step = min(KEEPER_STEP_M, distance)
        self.info.update(decision='return' if not ball or abs(dx) > .05 else 'left' if math.sin(course) > 0 else 'right',
                         reason='bounded_goal_correction')
        self.s.log('game', 'INFO', f'Goalkeeper: target=({target[0]:.2f},{target[1]:.2f})m step={step*1000:.0f}mm course={math.degrees(course):.0f}deg')
        # The supervisor observes live ball frames during the finite walk.
        # Do not modify its target in place or replay an interrupted section.
        self.keeper_watch = KeeperMotionGuard(self.odometry, ball, target)
        try:
            await self._original_walk(p, entry, 'omni', ball, distance_m=step, course=course,
                                      heading=wrap(self.odometry.yaw-self.odometry.home_heading))
        finally:
            self.keeper_watch = None

    async def _original_forward(self, p, entry):
        revision = self.revision
        observed = await self._original_observe(p, entry)
        if not self._same_plan(revision):
            return
        ball = self._usable(observed)
        if ball is None:
            # No camera/invalid geometry is not seven independent failed looks.
            if observed and observed['result'].get('reason') == 'no_ball':
                self.forward_misses += 1
            self.info.update(state='searching', phase='forward_search')
            if self.forward_misses >= 7:
                self.forward_misses = 0
                self.s.log('game', 'INFO', 'Forward: ball search turn after seven missed looks')
                await self._original_turn(wrap(self.odometry.yaw+2*math.pi/3), for_search=True)
            return
        self.forward_misses = 0
        target, power = kick_target(p, self.odometry, ball)
        x, y = ball['x_m'], ball['y_m']
        distance, course = math.hypot(x, y), math.atan2(y, x)
        player_angle = wrap(self.odometry.yaw+course+math.pi-target)
        fast = abs(player_angle) > 2. and distance <= .35
        # A close ball alone cannot authorize a large in-place turn: that
        # leaves it behind the robot. Use the orbit route for large angles.
        heading_error = abs(wrap(target-self.odometry.yaw))
        close_kick_route = distance <= .2 and abs(course) <= 1. and heading_error <= math.radians(45)
        fast = fast and heading_error <= math.radians(45)
        self.s.log('game', 'INFO',
                   f'Forward: ball x={x*1000:.0f} y={y*1000:.0f} mm, target={math.degrees(target):.0f} deg, power={power}')
        # Five fine jumps without a strike means that this close-in route did
        # not converge.  Do the original approach/orbit route once before a
        # further fine-adjustment pass; otherwise a stationary observation
        # restarts the same five jumps forever.
        if self.forward_replan:
            fast = False
            close_kick_route = False
            self.forward_replan = False
        initial = self.odometry.yaw
        # Match verify's 350 mm acceptance boundary. The fast-route heuristic
        # must never bypass approach for a distance that verify will reject.
        if distance > .35:
            self.info.update(state='approaching', phase='forward_approach')
            self.s.log('game', 'INFO', 'Forward: approach')
            heading = wrap(initial+course)
            if not await self._original_turn(heading, for_approach=True) or not self._same_plan(revision):
                return
            fresh = self._usable(await self._original_observe(p, entry))
            if fresh and self._same_plan(revision):
                await self._original_walk(p, entry, 'approach', fresh,
                                          distance_m=math.hypot(fresh['x_m'], fresh['y_m']),
                                          heading=wrap(heading-self.odometry.home_heading))
            return
        if not close_kick_route and not fast:
            self.info.update(state='approaching', phase='forward_around')
            self.s.log('game', 'INFO', 'Forward: go around ball')
            if distance > .5 or not await self._original_turn(wrap(initial+course), for_approach=True) or not self._same_plan(revision):
                return
            fresh = self._usable(await self._original_observe(p, entry))
            if fresh and self._same_plan(revision):
                await self._original_walk(p, entry, 'around', fresh, distance_m=distance, course=course,
                                          initial_heading=wrap(initial-self.odometry.home_heading),
                                          kick_heading=wrap(target-self.odometry.home_heading))
            return
        self.info.update(state='approaching', phase='forward_verify')
        self.s.log('game', 'INFO', 'Forward: verify kick position')
        if not await self._original_turn(target, for_kick=True) or not self._same_plan(revision):
            return
        ball = self._usable(await self._original_observe(p, entry))
        if ball is None or not self._same_plan(revision):
            return
        x, y = ball['x_m']*1000, ball['y_m']*1000
        if (x <= p['game.kick_distance_mm']+10 and math.hypot(x, y) < 200
                and abs(abs(y)-p['game.kick_offset_mm']) <= 60):
            await self._original_kick(revision, target, power, ball, x, y, reason='close ball; skip further alignment')
            return
        if math.hypot(x, y) > 350:
            self.forward_replan = True
            self.info.update(state='approaching', phase='forward_replan', reason='ball_beyond_verify_range')
            self.s.log('game', 'INFO', 'Verify cancelled: ball beyond 350 mm; replan approach')
            return
        course = math.atan2(ball['y_m'], ball['x_m'])
        if abs(course) > 1:
            self.s.log('game', 'INFO', f'Forward: side recovery, ball course={math.degrees(course):.0f} deg')
            await self._original_walk(p, entry, 'omni', ball, distance_m=.18,
                                      course=math.copysign(2.55, course),
                                      heading=wrap(self.odometry.yaw-self.odometry.home_heading))
            return
        if not await self._original_walk(p, entry, 'verify', ball, heading=wrap(target-self.odometry.home_heading)):
            return
        if not self._same_plan(revision):
            return
        # Original fine_adjustment_before_kick uses KICK_ADJUSTMENT_DISTANCE_2
        # (100 mm by default) only as an upper/front target: once the ball is
        # no farther than that target it must not be jumped backwards.  The
        # original code then strikes a ball within 200 mm when its lateral
        # kick offset is aligned.  Requiring abs(front) here was a regression:
        # it made a ball already under the kicking foot trigger endless
        # backwards jumps instead of a kick.
        #
        # Keep the port's five-jump safety/re-observation limit.  A bounded
        # pass explicitly requests a normal re-approach on the next cycle.
        jumps = 0
        previous_ball = ball
        for attempt in range(6):
            self.info.update(state='approaching', phase='forward_fine_adjustment')
            ball = self._usable(await self._original_observe(p, entry, settle=2. if attempt == 0 else .25))
            if ball is None or not self._same_plan(revision):
                return
            if math.hypot(ball['x_m']-previous_ball['x_m'], ball['y_m']-previous_ball['y_m']) > .2:
                self.s.log('game', 'INFO', 'Ball shifted over 200 mm during alignment; confirming in support pose')
                # No motion between confirmations. Require two independent,
                # agreeing observations; never act on the old coordinates.
                candidate = ball
                for _ in range(2):
                    fresh = self._usable(await self._original_observe(p, entry, settle=.5))
                    if fresh is None or not self._same_plan(revision):
                        self.s.log('game', 'INFO', 'Ball shift unconfirmed: no fresh observation')
                        return
                    agrees = (fresh['frame_sequence'] > candidate['frame_sequence']
                              and math.hypot(fresh['x_m']-candidate['x_m'], fresh['y_m']-candidate['y_m']) <= .1)
                    candidate = fresh
                    if agrees:
                        ball = fresh
                        self.s.log('game', 'INFO', f"Ball shift confirmed: x={ball['x_m']*1000:.0f} y={ball['y_m']*1000:.0f} mm")
                        break
                else:
                    self.s.log('game', 'INFO', 'Ball position unstable; abandon alignment and observe again')
                    return
            previous_ball = ball
            x, y = ball['x_m']*1000, ball['y_m']*1000
            if math.hypot(x, y) > 350:
                self.forward_replan = True
                self.info.update(state='approaching', phase='forward_replan', reason='ball_beyond_verify_range')
                self.s.log('game', 'INFO', f'Fine adjustment cancelled: ball distance={math.hypot(x, y):.0f} mm; replan approach')
                return
            front = x-p['game.kick_distance_mm']
            side = y+p['game.kick_offset_mm'] if y < 0 else y-p['game.kick_offset_mm']
            if front <= 10 and abs(side) <= 60 and math.hypot(x, y) < 200:
                await self._original_kick(revision, target, power, ball, x, y, reason='within offset strike envelope')
                return
            if jumps >= 5:
                self.forward_replan = True
                self.info.update(state='approaching', phase='forward_replan',
                                 reason='fine_adjustment_limit')
                self.s.log('game', 'INFO', 'Forward: five fine jumps reached; starting a new approach')
                return
            directions = []
            if front > 10: directions.append('forward')
            if side > 10: directions.append('left')
            elif side < -10: directions.append('right')
            self.s.log('game', 'INFO',
                       f"Forward: fine x={x:.0f} y={y:.0f} mm, front={front:.0f} side={side:.0f} mm; jump {','.join(directions) or 'none'}")
            for direction in directions:
                if jumps >= 5:
                    break
                if not await self._move('game.jump', {'direction': direction, 'fraction': 1.}):
                    return
                if not self._same_plan(revision):
                    return
                jumps += 1
            if not await self._original_turn(target, for_kick=True) or not self._same_plan(revision):
                return

    async def _original_kickoff(self, p, entry):
        self.info.update(state='kickoff_ride', phase='kickoff_ride')
        invert = random.choice((-1, 1))
        self.s.log('game', 'INFO', f"Forward: kickoff lead-in 200 mm, then original arc {'left' if invert > 0 else 'right'}")
        return await self._original_walk(p, entry, 'kickoff', invert=invert)
