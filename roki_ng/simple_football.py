"""Detection-only state machine. IMU is a local heading reference, never odometry."""
import asyncio
import math
import time
import json

from .calibration import wrap
from .football_vision import assemble_goals, shot_clear, own_goal_blocks
from .wire import Fault
from .body_imu import quaternion_yaw
from .recovery import body_position
from .ball_kick_test import kick_alignment


class SimpleFootball:
    def __init__(self, game, parameters, entry, delay, *, danger=False):
        self.danger = danger
        self.attack_heading = None
        self.attack_attempts = 0
        self.goal_memory = None
        self.own_veto_reported = False
        self.remembered_shot = False
        self.danger_approaches = 0
        self.kick_direction = 'straight'
        self.angle_target = None
        self.angle_attempts = 0
        self.fine_heading_attempts = 0
        self.goal_scan_needed = True
        self.last_goal_scan = -math.inf
        self.g, self.p, self.entry = game, parameters | {'game.use_visual_localisation': False}, entry
        self.entry_pending = entry in ('left', 'right')
        self.kickoff_pending = not danger and entry == 'center' and not delay and parameters['game.forward.kick_off_ride']
        self.deadline = self.kickoff_heading = None
        self.pose_id = 0
        self.scan = 0
        self.near = False
        self.far_frames = 0
        self.jumps = 0
        self.leg_sign = None
        self.reset_leg_switch()
        self.orbit_sign = 1
        self.orbit_face_ball = False
        self.last_ball_side = 1
        self.last_score = None
        self.score_kind = None
        self.last_turn_error = None
        self.stagnant = 0
        self.last_scene = None
        self.shot_deadline = None
        self.note = 'starting'
        self.specs = [parameters[f'field.goal.{i}'] for i in (0, 1)]
        self.own = self.specs[parameters['match.own_goal']]['colour']
        self.enemy = self.specs[1-parameters['match.own_goal']]['colour']
        if not danger and (self.own not in ('yellow', 'blue') or self.enemy not in ('yellow', 'blue') or self.own == self.enemy):
            raise Fault('configuration', 'Simple football needs distinct blue/yellow goal colours')

    def cancel_special(self, reason):
        self.orbit_face_ball = False
        self.reset_leg_switch()
        self.goal_scan_needed = True
        self.last_goal_scan = -math.inf
        self.fine_heading_attempts = 0
        if self.kick_direction == 'kickoff_left':
            self.kick_direction = 'straight'
            self.g.info['kick_direction'] = 'straight'
        self.angle_target = None
        self.angle_attempts = 0
        self.danger_approaches = 0
        self.forget_goal(reason)
        if self.kickoff_pending or self.deadline is not None:
            self.g.s.log('game', 'INFO', f'Simple football kickoff cancelled: {reason}')
        self.kickoff_pending = False
        self.deadline = self.kickoff_heading = None
        self.entry_pending = False
        self.near = False
        self.jumps = self.stagnant = 0
        self.leg_sign = self.last_scene = self.last_score = None
        self.last_turn_error = self.score_kind = None
        self.shot_deadline = None

    def forget_goal(self, reason):
        if self.goal_memory is not None:
            self.g.s.log('game', 'INFO', f'Simple goal memory cancelled: {reason}')
        self.goal_memory = None

    def check_goal_memory(self, imu=None):
        """Check every published IMU sample, including while awaiting body/head jobs."""
        memory = self.goal_memory
        if memory is None:
            return False
        imu = imu if imu is not None else self.g.s.body_telemetry.get('body.imu', {})
        stamp = imu.get('source_mono_ns')
        now = time.monotonic()
        reason = None
        if now >= memory['until']:
            reason = '5 second expiry'
        elif (not imu.get('valid') or type(stamp) is not int
              or not 0 <= (time.monotonic_ns()-stamp)/1e9 < .25
              or stamp < memory['stamp'] or stamp-memory['stamp'] > 250_000_000):
            reason = 'IMU stale, missing or reset'
        else:
            try:
                if body_position(imu['quaternion_xyzw']) != 'upright':
                    reason = 'body not upright'
                elif abs(wrap(quaternion_yaw(imu['quaternion_xyzw'])-memory['yaw'])) > math.radians(10):
                    reason = 'heading departed by more than 10 degrees'
            except (KeyError, ValueError, TypeError, Fault):
                reason = 'invalid body orientation'
        if reason:
            self.forget_goal(reason)
            return False
        memory['stamp'] = stamp
        return True

    def expired(self):
        return self.deadline is not None and time.monotonic() >= self.deadline

    def describe(self, phase, reason):
        left = '' if self.deadline is None else f' kickoff={max(0, self.deadline-time.monotonic()):.1f}s'
        self.note = f'{phase} jump={self.jumps}/15{left} {reason}'
        self.g.info.update(phase=phase, reason=reason, decision=phase)
        self.g.s.log('game', 'INFO', ('Danger football: ' if self.danger else 'Simple football: ')+self.note)

    def log_search_found(self, pan, tilt, scene):
        x, y = scene['ball']
        self.g.s.log('game', 'INFO', f'Ball head search: FOUND pan={pan} tilt={tilt} x={x*1000:.0f} y={y*1000:.0f}mm')

    def reset_leg_switch(self):
        self.leg_switched = False
        self.leg_candidate = None
        self.leg_sample = None

    def consider_leg_switch(self, y_mm, sample):
        """One change per attempt, two distinct agreeing frames, >=60mm benefit."""
        if self.leg_switched or self.leg_sign is None or sample is None:
            return
        if sample == self.leg_sample:
            return
        self.leg_sample = sample
        sign = -1 if y_mm < 0 else 1
        offset = self.p['game.kick_offset_mm']
        benefit = abs(y_mm-self.leg_sign*offset)-abs(y_mm-sign*offset)
        if sign == self.leg_sign or abs(y_mm) < 30 or benefit < 60:
            self.leg_candidate = None
            return
        if self.leg_candidate is not None and sign == self.leg_candidate[0] and abs(y_mm-self.leg_candidate[1]) <= 80:
            old = self.leg_sign
            self.leg_sign, self.leg_switched = sign, True
            self.leg_candidate = None
            self.g.s.log('game', 'INFO', f"Fine leg switched once: {'left' if old > 0 else 'right'} -> {'left' if sign > 0 else 'right'}; y={y_mm:.0f}mm benefit={benefit:.0f}mm jumps={self.jumps}/15 frame={sample}")
        else:
            self.leg_candidate = (sign, y_mm)

    async def move(self, op, args):
        revision = self.g.revision
        if self.expired():
            self.cancel_special('30 second deadline')
            return False
        self.last_scene = None
        await self.g._call('detection', 'football.pose', {'active': False, 'note': self.note})
        if not self.g._same_plan(revision):
            self.cancel_special('plan changed before motion')
            return False
        if op == 'motion.kick' and self.remembered_shot and not self.check_goal_memory():
            return False
        if hasattr(self, 'motion_fresh') and not self.motion_fresh(op, args):
            return False
        result = await self.g._move(op, args, monitor_ball=op == 'game.walk' and self.g.info.get('phase') != 'simple_entry')
        if not result:
            self.cancel_special('motion interrupted')
        if op == 'motion.kick':
            self.forget_goal('strike attempted')
            self.g.s.log('game', 'INFO' if result else 'WARNING',
                         f'Simple kick: {"completed" if result else "not completed"}; {self.g.info.get("reason", "")}')
        return result

    async def walk(self, distance, course=0.):
        self.forget_goal('walking; require a new opening observation')
        revision = self.g.revision
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return False
        return await self.move('game.walk', dict(kind='omni', distance_m=distance,
                                                course=course, heading=0., yaw_offset=wrap(-yaw)))

    async def jump(self, direction, fraction=1., *, alignment=False):
        if self.goal_memory is not None:
            if direction in ('left', 'right'):
                self.goal_memory['side_mm'] += abs(self.p[f'motion.jump_{direction}_mm'])*max(.1, min(1., fraction))
                if self.goal_memory['side_mm'] > 60:
                    self.forget_goal('lateral command budget exceeded 60mm')
            elif direction != 'forward':
                self.forget_goal('turn or backward reposition')
        if alignment:
            self.jumps += 1
        return await self.move('game.jump', dict(direction=direction, fraction=max(.1, min(1., fraction))))

    async def turn(self, angle, *, alignment=False, tolerance=15):
        if abs(angle) <= math.radians(tolerance):
            return True
        key = 'motion.jump_yaw_ccw' if angle > 0 else 'motion.jump_yaw_cw'
        return await self.jump('turn_left' if angle > 0 else 'turn_right', abs(angle)/abs(self.p[key]), alignment=alignment)

    async def search_sector(self):
        revision = self.g.revision
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        target = wrap(yaw+math.pi/3)
        for _ in range(3):
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
            error = wrap(target-yaw)
            if abs(error) <= math.radians(15) or (self.near and self.jumps >= 15):
                return
            if not await self.turn(error, alignment=self.near):
                return

    async def look(self, pan, tilt):
        if self.expired() or not self.g._allowed():
            return None
        revision = self.g.revision
        await self.g._call('detection', 'football.pose', {'active': False, 'note': self.note})
        if not self.g._same_plan(revision):
            return None
        if not await self.g._move('game.head', {'pan': pan, 'tilt': tilt}):
            return None
        if not await self.g._settle(.10):
            return None
        if not self.g._same_plan(revision):
            return None
        self.pose_id += 1
        pose_id = self.pose_id
        await self.g._call('detection', 'football.pose', dict(active=True, pose_id=pose_id,
                         pan=pan, tilt=tilt, after=time.monotonic(), note=self.note,
                         goal_bearing_only=getattr(self, 'goal_bearing_only', False),
                         goal_veto_enabled=not self.danger and not getattr(self, 'goal_bearing_only', False)))
        until = time.monotonic()+.65
        while self.g._same_plan(revision) and time.monotonic() < until and not self.expired():
            scene = await self.g._call('detection', 'football.status')
            if not self.g._same_plan(revision):
                return None
            if scene.get('valid') and scene.get('pose_id') == pose_id:
                # Tracker marks the first frame unconfirmed; wait for the second.
                if scene.get('confirmed'):
                    return scene
            await asyncio.sleep(.025)
        return None

    async def observe(self, goals=False):
        revision = self.g.revision
        if not await self.g._prepare(self.p, self.entry) or not self.g._same_plan(revision):
            return None, []
        began = time.monotonic()
        parts = []
        ball_scene = None
        parameter_revision = None
        if goals:
            # Sweep progresses across passes; always finish with a close ball look.
            poses = [(0, self.p['head.field_tilt']), (-1778, self.p['head.field_tilt']),
                     (1778, self.p['head.field_tilt']), (0, 0), (-2666, 0), (2666, 0)]
            for _ in range(2):
                pan, tilt = poses[self.scan % len(poses)]
                self.scan += 1
                scene = await self.look(pan, tilt)
                if not self.g._same_plan(revision):
                    return None, []
                if scene:
                    if parameter_revision != scene.get('parameter_revision'):
                        parts, ball_scene = [], None
                        parameter_revision = scene.get('parameter_revision')
                    parts.extend(scene['parts'])
                    if scene.get('ball'):
                        ball_scene = scene
                if time.monotonic()-began > 1.8:
                    break
        scene = await self.look(0, self.p['game.head_tilt'])
        if not scene or not self.g._same_plan(revision):
            return None, []
        if parameter_revision != scene.get('parameter_revision'):
            parts, ball_scene = [], None
        ball_received = scene['received']
        ball_sequence = scene.get('sequence')
        if not scene.get('ball') and ball_scene and time.monotonic()-ball_scene['received'] <= .5:
            scene = scene | {'ball': ball_scene['ball']}
            ball_received = ball_scene['received']
            ball_sequence = ball_scene.get('sequence')
        self.last_scene = scene | {'ball_sequence': ball_sequence}
        self.shot_deadline = min(began+3, ball_received+.5)
        self.g.info['ball'] = dict(x_m=scene['ball'][0], y_m=scene['ball'][1]) if scene['ball'] else None
        parts.extend(scene['parts'])
        goals_seen = assemble_goals(parts, self.specs) if time.monotonic()-began <= 3 else []
        return scene['ball'], goals_seen

    async def step(self):
        revision = self.g.revision
        if self.expired():
            self.cancel_special('30 second deadline')
        if self.entry_pending:
            self.entry_pending = False  # Unknown execution can never repeat entry.
            self.describe('simple_entry', 'advance into field')
            distance = self.p['game.simple_football.entry_forward_mm']/1000
            if distance:
                await self.walk(distance)
            return
        if self.danger:
            await self.danger_ball_step()
            return
        if self.kickoff_pending:
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
            sign = 1 if self.p['game.simple_football.kickoff_side'] == 'left' else -1
            self.kickoff_heading = wrap(yaw+sign*math.pi/3)
            self.deadline = time.monotonic()+30
            self.kickoff_pending = False
            self.describe('simple_kickoff', f"side={self.p['game.simple_football.kickoff_side']} relative=60deg")

        revision = self.g.revision
        # Kickoff spends its 30-second budget on the ball, not mandatory goal scans.
        # Goals visible in the ball frame still veto an own-goal corridor.
        memory_valid = self.check_goal_memory()
        scan_goals = (self.near and not self.orbit_face_ball and self.deadline is None and not memory_valid
                      and (self.goal_scan_needed or time.monotonic()-self.last_goal_scan >= 4))
        if scan_goals:
            self.last_goal_scan = time.monotonic()
            self.goal_scan_needed = False
        self.g.s.log('game', 'INFO', 'Simple observation: '+(
            'scan goals' if scan_goals else 'ball only; goal memory valid' if memory_valid else 'ball only; defer goal scan'))
        ball, goals = await self.observe(goals=scan_goals)
        if not self.g._same_plan(revision):
            return
        if self.expired():
            self.cancel_special('30 second deadline')
            return
        if ball is None:
            self.leg_candidate = None
            self.forget_goal('ball lost')
            self.describe('simple_search', 'no fresh ball')
            if self.near and self.jumps >= 15:
                self.describe('simple_new_attempt', '15 jumps; ball lost, restart search without reposition')
                self.near, self.jumps, self.leg_sign = False, 0, None
                self.fine_heading_attempts = 0
                return
            self.orbit_face_ball = False
            # The centre was just inspected. One look towards the last ball,
            # then turn the body instead of repeating six head poses.
            pan, tilt = -1778*self.last_ball_side, self.p['head.field_tilt']
            scene = await self.look(pan, tilt)
            if not self.g._same_plan(revision) or self.expired():
                return
            if scene and scene.get('ball'):
                self.log_search_found(pan, tilt, scene)
                await self.turn(math.atan2(scene['ball'][1], scene['ball'][0]), alignment=self.near)
                return
            self.g.s.log('game', 'INFO', 'Ball head search: no ball in scanned poses; turn body to next sector')
            await self.search_sector()
            return

        x, y = ball
        if abs(y) > .02:
            self.last_ball_side = 1 if y > 0 else -1
        distance, course = math.hypot(x, y), math.atan2(y, x)
        if self.near and distance > .35:
            self.describe('simple_new_attempt', 'ball left 350mm zone; finish alignment attempt')
            self.near, self.jumps, self.leg_sign = False, 0, None
            self.fine_heading_attempts = 0
            return
        if not self.near:
            self.orbit_face_ball = False
            self.leg_sign = None
            if distance <= .35:
                self.near = True
                self.reset_leg_switch()
                self.jumps = self.fine_heading_attempts = 0
        enemy = next((g for g in goals if g['colour'] == self.enemy), None)
        own = next((g for g in goals if g['colour'] == self.own), None)
        margin = self.p['game.ball_diameter_m']/2+.05
        yaw, _ = await self.g._imu()
        if not self.g._same_plan(revision):
            return
        kickoff = self.deadline is not None
        if self.danger:
            error, corridor = 0., True  # Explicit reckless role: no goal-direction veto.
        elif kickoff:
            error = wrap(self.kickoff_heading-yaw)
            corridor = abs(error) <= math.radians(15)
        elif enemy:
            centre = [(a+b)/2 for a, b in zip(*enemy['ends'])]
            error = math.atan2(centre[1]-y, centre[0]-x)
            corridor = enemy.get('shootable', False) and shot_clear(ball, enemy, margin)
        else:
            error, corridor = 0., False
        own_blocks = own_goal_blocks(ball, own, self.p['game.ball_diameter_m']/2)
        own_veto = bool(own_blocks)
        if own_veto:
            self.g.s.log('game', 'INFO', 'Own goal veto: '+json.dumps(dict(
                ball_frame=(self.last_scene or {}).get('ball_sequence'), ball_m=ball,
                blocks=own_blocks), separators=(',', ':')))
        if own_veto or self.own_veto_reported:
            await self.g._call('detection', 'football.decision', {'blocks': own_blocks})
            if not self.g._same_plan(revision):
                return
            self.own_veto_reported = own_veto
        if own_veto:
            corridor = False
        self.remembered_shot = False
        self.check_goal_memory()
        if not self.danger and not kickoff:
            if own_veto or distance > .25:
                self.forget_goal('own goal visible or ball outside close zone')
            elif enemy and enemy.get('shootable', False):
                if corridor:
                    imu = getattr(self.g.s, 'body_telemetry', {}).get('body.imu', {})
                    self.goal_memory = dict(yaw=yaw, until=time.monotonic()+5,
                                            stamp=imu.get('source_mono_ns', 0), side_mm=0.)
                    self.check_goal_memory()
                else:
                    self.forget_goal('visible opening contradicts strike direction')
            elif self.check_goal_memory():
                corridor = self.remembered_shot = True
                error = 0.
                self.g.s.log('game', 'INFO', 'Simple shot: using recent goal opening; IMU heading retained')

        if self.leg_sign is None:
            self.leg_sign = -1 if y < 0 else 1
        if self.near:
            self.consider_leg_switch(y*1000, (self.last_scene or {}).get('ball_sequence'))
        front = x*1000-self.p['game.kick_distance_mm']
        side = y*1000-self.leg_sign*self.p['game.kick_offset_mm']
        blockers = []
        if own_veto: blockers.append('own_goal')
        if not self.danger and not kickoff and not self.remembered_shot and enemy is None: blockers.append('no_opponent_goal')
        elif not self.danger and not kickoff and not self.remembered_shot and not enemy.get('shootable', False): blockers.append('goal_colour_only')
        elif not corridor and not own_veto: blockers.append('heading' if kickoff else 'outside_goal_opening')
        if x <= 0: blockers.append('ball_behind')
        if front > 10: blockers.append('ball_far_forward')
        if abs(side) > 20: blockers.append('foot_alignment')
        if distance >= .2: blockers.append('ball_range')
        age = ((time.monotonic()-self.last_scene['received'])*1000
               if self.last_scene else None)
        self.g.s.log('game', 'INFO',
                    f"Simple ball: x={x*1000:.0f} y={y*1000:.0f} d={distance*1000:.0f}mm "
                    f"age_ms={round(age) if age is not None else 'unknown'} "
                    f"leg={'left' if self.leg_sign > 0 else 'right'} front={front:.0f} side={side:.0f}mm")
        self.g.s.log('game', 'INFO',
                    f"Simple shot: enemy={enemy['colour'] if enemy else 'none'} "
                    f"own={own['colour'] if own else 'none'} yaw_error={math.degrees(error):.1f}deg "
                    f"blocked={','.join(blockers) or 'none'}")

        if not self.near:
            if abs(course) > math.radians(15):
                await self.turn(course)
                return
            # Same finite, live-feedback approach used by Ball approach and kick.
            # No distance-minus-450mm command and no centimetre omni walks.
            if not self.last_scene:
                return
            if self.shot_deadline is None:
                return
            stamp = min(self.last_scene['received'], self.shot_deadline-.5)
            if not 0 <= time.monotonic()-stamp <= .5:
                return
            self.forget_goal('streaming approach')
            self.describe('simple_test_approach', f'original streaming approach; distance={distance*1000:.0f}mm')
            await self.move('game.walk', dict(kind='approach', distance_m=distance,
                heading=course, yaw_offset=wrap(-yaw), x_m=x, y_m=y, stamp=stamp))
            return

        ready, direction, fraction, front, side = kick_alignment(
            self.p, x*1000, y*1000, self.leg_sign*self.p['game.kick_offset_mm'])
        ready = ready and x > 0
        if corridor and ready:
            if self.expired() or not self.g._same_plan(revision):
                return
            power = 60 if kickoff else 100
            self.describe('simple_kick', f'safe corridor; request strike power={power}')
            completed = await self.move('motion.kick', dict(leg='left' if y >= 0 else 'right', power=power,
                offset=max(-60, min(80, int(abs(y)*1000)-62))))
            if completed:
                self.near, self.jumps, self.leg_sign = False, 0, None
                self.fine_heading_attempts = 0
                await self.g._settle(1.)
            return
        if self.jumps >= 15:
            # End this attempt without the old backward/orbit manoeuvre.
            self.describe('simple_new_attempt', '15 jumps; reobserve without blind strike')
            self.near, self.jumps, self.leg_sign = False, 0, None
            self.fine_heading_attempts = 0
            self.orbit_face_ball = False
            return
        if self.orbit_face_ball:
            # Complete a short arc: lateral jump, fresh ball, then face it.
            # Never execute a stored turn against an old ball observation.
            self.orbit_face_ball = False
            if abs(course) > math.radians(5):
                self.describe('simple_orbit_face', 'face freshly observed ball after lateral jump')
                await self.turn(course, alignment=True, tolerance=5)
                return
        if ready:
            self.goal_scan_needed = False  # Periodic scans, not a sweep every iteration.
            # Goal checks still gate the shot. Only a confirmed opening (or
            # explicit kickoff heading) may request a bounded orientation fix.
            if (not own_veto and (kickoff or enemy and enemy.get('shootable', False))
                    and math.radians(15) < abs(error) <= math.pi/2
                    and (kickoff or self.fine_heading_attempts < 2)):
                self.fine_heading_attempts += 1
                self.describe('simple_fine_heading', f'confirmed direction; attempt={self.fine_heading_attempts}; total budget=15 jumps')
                await self.turn(max(-math.pi/6, min(math.pi/6, error)), alignment=True)
            else:
                # A colour hint may guide movement, but never permit a shot.
                # Keep the selected direction without a trustworthy opening.
                if (kickoff or enemy and enemy.get('shootable', False)) and abs(error) > math.radians(15):
                    self.orbit_sign = 1 if error > 0 else -1
                direction = 'right' if self.orbit_sign > 0 else 'left'
                fraction = 30/abs(self.p[f'motion.jump_{direction}_mm'])
                self.describe('simple_orbit', f'{direction} nominal 30mm; '+','.join(blockers))
                if await self.jump(direction, fraction, alignment=True):
                    self.orbit_face_ball = True
            return
        if x <= 0:
            self.describe('simple_search', 'ball behind; reobserve, no blind forward jump')
            return
        self.describe('simple_test_fine', f'jump {self.jumps+1}/15 {direction} front={front:.0f} side={side:.0f}mm')
        await self.jump(direction, fraction, alignment=True)

    async def reposition(self, reason):
        self.describe('simple_reposition', reason)
        if self.deadline is not None:
            self.cancel_special(reason)
        if await self.jump('backward'):
            self.jumps = self.stagnant = 0
            self.leg_sign = self.last_score = self.score_kind = self.last_turn_error = None

    async def danger_ball_step(self):
        """Ball approach/kick test repeated as a game, with a 15-jump budget."""
        revision = self.g.revision
        observed = await self.g._original_observe(self.p, self.entry, settle=.5)
        if not self.g._same_plan(revision):
            return
        ball = self.g._usable(observed)
        if ball is None:
            self.leg_candidate = None
            if self.jumps >= 15:
                self.near, self.leg_sign, self.jumps = False, None, 0
                self.danger_approaches = 0
            self.describe('danger_search', 'no fresh ball; retain fine alignment budget')
            for pan in (0, -1778, 1778):
                scene = await self.look(pan, self.p['game.head_tilt'])
                if not self.g._same_plan(revision):
                    return
                if scene and scene.get('ball'):
                    self.log_search_found(pan, self.p['game.head_tilt'], scene)
                    await self.turn(math.atan2(scene['ball'][1], scene['ball'][0]))
                    return
            self.g.s.log('game', 'INFO', 'Ball head search: no ball in scanned poses; turn body to next sector')
            await self.search_sector()
            return
        x, y = ball['x_m']*1000, ball['y_m']*1000
        distance = math.hypot(x, y)/1000
        self.g.s.log('game', 'INFO', f'Danger ball test: x={x:.0f} y={y:.0f}mm jumps={self.jumps}/15')
        if self.near and distance > .35:
            self.describe('danger_new_attempt', 'ball left 350mm zone; finish this alignment attempt')
            self.near, self.leg_sign, self.jumps = False, None, 0
            self.danger_approaches = 0
            return
        if not self.near and distance > .35:
            course = math.atan2(y, x)
            if abs(course) > math.radians(15):
                await self.turn(course)
                return  # Fresh observation after facing the ball, just like the test.
            if self.danger_approaches >= 3:
                self.describe('danger_new_attempt', 'three approaches completed; reobserve for a new attempt')
                self.danger_approaches = 0
                return
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
            stamp = time.monotonic()-(self.g._capture_now()-ball['sensor_timestamp_ns'])/1e9
            if not 0 <= time.monotonic()-stamp <= .5:
                return
            self.danger_approaches += 1
            self.describe('danger_test_approach', f'original streaming approach; distance={distance*1000:.0f}mm')
            await self.move('game.walk', dict(kind='approach', distance_m=distance,
                heading=course, yaw_offset=wrap(-yaw), x_m=ball['x_m'], y_m=ball['y_m'], stamp=stamp))
            return
        if not self.near:
            self.near = True
            self.reset_leg_switch()
            self.leg_sign = -1 if y < 0 else 1
            self.jumps = 0
            self.describe('danger_test_fine', f"leg={'left' if self.leg_sign > 0 else 'right'}; no more walking")
        self.consider_leg_switch(y, ball.get('sensor_timestamp_ns'))
        ready, direction, fraction, front, side = kick_alignment(self.p, x, y, self.leg_sign*self.p['game.kick_offset_mm'])
        if ready:
            yaw, _ = await self.g._imu()
            if not self.g._same_plan(revision):
                return
            if self.kick_direction != 'straight':
                kickoff = self.kick_direction == 'kickoff_left'
                angle = 60 if kickoff else 30
                attempts = 4 if kickoff else 2
                if self.angle_target is None:
                    self.angle_target = wrap(yaw+math.radians(angle)*(1 if self.kick_direction in ('left', 'kickoff_left') else -1))
                error = wrap(self.angle_target-yaw)
                if abs(error) > math.radians(10) and self.angle_attempts < attempts and self.jumps < 15:
                    self.angle_attempts += 1
                    self.describe('danger_angled_kick', f'{self.kick_direction} {angle}deg; remaining={math.degrees(error):.1f}deg attempt={self.angle_attempts}/{attempts}')
                    await self.turn(error, alignment=True, tolerance=10)
                    return  # Turn moves the ball relative to the feet: always reobserve/re-align.
            stamp = time.monotonic()-(self.g._capture_now()-ball['sensor_timestamp_ns'])/1e9
            self.shot_deadline = stamp+.5
            power = 60 if self.kick_direction == 'kickoff_left' else 100
            self.describe('danger_test_kick', f'power={power} front={front:.0f} side={side:.0f}mm')
            completed = await self.move('motion.kick', dict(leg='right' if y < 0 else 'left', power=power,
                offset=max(-60, min(80, int(abs(y))-62))))
            if completed:
                if self.kick_direction == 'kickoff_left':
                    self.kick_direction = 'straight'
                    self.g.info['kick_direction'] = 'straight'
                self.angle_target = None
                self.angle_attempts = 0
                self.near, self.leg_sign, self.jumps = False, None, 0
                self.danger_approaches = 0
                await self.g._settle(1.)
            return
        if self.jumps >= 15:
            self.describe('danger_new_attempt', '15 alignment jumps; no blind strike, reobserve')
            self.near, self.leg_sign, self.jumps = False, None, 0
            self.danger_approaches = 0
            return
        self.describe('danger_test_fine', f'jump {self.jumps+1}/15 {direction} fraction={fraction:.2f} front={front:.0f} side={side:.0f}mm')
        await self.jump(direction, fraction, alignment=True)
