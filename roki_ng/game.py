"""Supervisor-owned game coordinator; body jobs remain finite and cancellable."""
import asyncio
import copy
import math
import time
import uuid

from .body_imu import quaternion_yaw
from .odometry import GameOdometry
from .goalkeeper import ball_in_body
from .legacy_strategy import OriginalStrategy
from .ball_kick_test import run_ball_kick_test
from .recovery import body_position
from .wire import Fault, boolean, choice, number


def capture_clock():
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


class Goalkeeper(OriginalStrategy):
    def __init__(self, supervisor):
        self.s = supervisor
        self.task = self.session = None
        self.stop_lock = asyncio.Lock()
        self.generation = self.revision = 0
        self.starting = False
        self.stop_requested = self.hard_stop_requested = False
        self.motion_inflight = self.paused = self.pickup_requested = False
        self.motion_job = self.stop_sent = None
        self.pickup_attempted = self.pickup_needs_confirmation = False
        self.odometry = None
        self.simple = None
        self.keeper_watch = None
        self.needs_prepare = True
        self.body_ready_after = 0
        self.last_seen = 0.
        self.last_seen_sequence = self.last_action_sequence = -1
        self.head_pan = None
        self.observation_after = capture_clock()
        self.info = self._info()
        self.forward_misses = 0
        # A fine-adjustment pass is deliberately bounded.  The following pass
        # must use the normal approach/orbit route rather than immediately
        # entering another identical series of jumps.
        self.forward_replan = False
        self.visual_window = None
        self.visual_capture = None
        self.visual_owned = False
        self.visual_retry_at = self.visual_poll_at = 0.

    def _info(self):
        return dict(running=False, state='stopped', role=None, entry=None, phase='idle',
                    recovery='none', recovery_attempt=0, paused=False, pickup=False,
                    pickup_ready=False, pickup_confirmation_required=False,
                    reason='', decision='hold', ball=None, position=None,
                    last_correction=None, travel_m=0., path_m=0., job_id=None)

    def state(self):
        state = copy.deepcopy(self.info)
        # game.status and game.state must fit one UDP datagram even when a
        # hardware error contains multibyte text. Full errors remain in logs.
        for record in (state, state.get('position'), state.get('ball')):
            if record and record.get('reason'):
                record['reason'] = str(record['reason']).encode('utf-8')[:180].decode('utf-8', errors='ignore')
        return state

    async def _call(self, role, op, args=None, **kwargs):
        return await self.s.workers[role].call(op, args or {}, **kwargs)

    def _invalidate_observation(self):
        self.visual_window = None
        self.observation_after = capture_clock()
        self.info.update(ball=None, decision='hold')

    def _position(self):
        if self.odometry:
            self.info.update(position=self.odometry.state(), path_m=self.odometry.path,
                             travel_m=self.odometry.path)

    def _uncertain(self, reason):
        self.forward_misses = 0
        if self.odometry:
            self.odometry.uncertain(reason)
            self._position()

    def _allowed(self, *, pickup=False):
        return not self.stop_requested and (self.pickup_requested if pickup
                                           else not self.paused and not self.pickup_requested)

    async def start(self, session, args):
        if set(args) - {'strategy', 'entry', 'delay_seconds', 'lease_epoch', 'kick_direction'}:
            raise Fault('invalid_argument', 'Unknown game settings')
        role = choice(args, 'strategy', None, ('FIRA_penalty_Goalkeeper', 'forward', 'ball_kick_test', 'simple_football', 'danger_football', 'simple_goalkeeper'))
        if role != 'danger_football' and 'kick_direction' in args:
            raise Fault('invalid_argument', 'Kick direction is only available in Danger football')
        kick_direction = choice(args, 'kick_direction', 'straight', ('straight', 'left', 'right', 'kickoff_left'))
        entry = choice(args, 'entry', 'center', ('center', 'left', 'right')) if role in ('forward', 'simple_football', 'danger_football') else ('center' if role == 'ball_kick_test' else 'goalkeeper')
        if role not in ('forward', 'simple_football', 'danger_football') and 'entry' in args:
            raise Fault('invalid_argument', 'This strategy has no entry')
        delay = number(args, 'delay_seconds', 0, 0, 30, True)
        from .simple_football import SimpleFootball
        from .simple_goalkeeper import SimpleGoalkeeper
        simple = (SimpleFootball(self, copy.deepcopy(self.s.params.values), entry, delay, danger=role == 'danger_football')
                  if role in ('simple_football', 'danger_football') else None)
        if role == 'simple_goalkeeper':
            simple = SimpleGoalkeeper(self, copy.deepcopy(self.s.params.values), entry, delay)
        if role == 'danger_football':
            simple.kick_direction = kick_direction
        if self.starting or self.stop_lock.locked():
            raise Fault('busy', 'Game start is pending')
        if self.info['running']:
            if not self.pickup_requested or not self.info['pickup_ready']:
                raise Fault('busy', 'Stop game or complete Pick up before re-entry')
            reentry_generation = self.generation
            await self.stop('reentry')
            if self.generation != reentry_generation + 1:
                raise Fault('cancelled', 'Re-entry cancelled by stop')
        if self.s.mode != 'MANUAL':
            raise Fault('invalid_state', 'Select MANUAL before starting game')
        self.s.require_motion_ready()
        self.starting = True
        generation = self.generation
        try:
            body = await self._call('motherboard', 'state')
            self.s.require_control(session, args)
            if generation != self.generation or self.s.mode != 'MANUAL':
                raise Fault('cancelled', 'Game start cancelled')
            if body.get('active_job') or not body.get('body_connected') or body.get('error') or body.get('queue_reset_pending'):
                raise Fault('not_ready', 'Body must be connected, idle and healthy')
            self.session = session
            self.stop_requested = self.hard_stop_requested = self.paused = self.pickup_requested = False
            self.motion_inflight = False
            self.motion_job = self.stop_sent = None
            self.pickup_attempted = self.pickup_needs_confirmation = False
            self.odometry = None
            self.simple = simple
            self.visual_window = self.visual_capture = None
            self.visual_owned = False
            self.visual_retry_at = 0.
            self.forward_misses = 0
            self.forward_replan = False
            self.needs_prepare = True
            self.body_ready_after = time.monotonic_ns()
            self.last_seen, self.last_seen_sequence, self.last_action_sequence = 0., -1, -1
            self.head_pan = None
            self.info = self._info() | dict(running=True, state='preparing', role=role, entry=entry,
                                           phase='initialising', job_id=uuid.uuid4().hex)
            if role == 'danger_football':
                self.info['kick_direction'] = kick_direction
            self._invalidate_observation()
            self.s.mode = 'GAME'
            self.task = asyncio.create_task(self._run(copy.deepcopy(self.s.params.values), role, entry, delay))
            return self.state()
        finally:
            self.starting = False

    async def stop(self, reason='operator_stop', *, hard=False):
        self.stop_requested = True
        self.hard_stop_requested |= hard
        self.revision += 1
        self.generation += 1
        async with self.stop_lock:
            if self.task and not self.task.done():
                self.info.update(state='stopping', phase='stopping', reason=reason)
                if hard or not self.motion_inflight:
                    self.task.cancel()
                else:
                    await self._request_graceful()
                await asyncio.gather(self.task, return_exceptions=True)
            if self.motion_inflight:
                await self._hard_stop()
            self.info.update(running=False, state='stopped', phase='stopped', decision='hold', reason=reason)
            if self.s.mode == 'GAME':
                self.s.mode = 'MANUAL' if self.s.owner else 'IDLE'
            return self.state()

    async def pause(self):
        if not self.info['running']:
            raise Fault('invalid_state', 'No active game')
        self.paused = True
        if self.simple:
            self.simple.cancel_special('pause')
        self.revision += 1
        self._invalidate_observation()
        self.info.update(paused=True, state='paused', phase='paused', decision='hold')
        await self._request_graceful()
        return self.state()

    async def resume(self):
        if not self.info['running'] or self.pickup_requested:
            raise Fault('invalid_state', 'Game cannot resume')
        self.paused = False
        self.forward_misses = 0
        self.forward_replan = False
        self.revision += 1
        self.needs_prepare = True
        self._invalidate_observation()
        self.info.update(paused=False, state='searching', phase='searching')
        return self.state()

    async def pickup(self, args=None):
        args = args or {}
        if set(args) - {'confirm', 'lease_epoch'}:
            raise Fault('invalid_argument', 'Unknown Pick up settings')
        confirm = boolean(args, 'confirm', False)
        if not self.info['running']:
            raise Fault('invalid_state', 'No active game')
        if confirm:
            if not self.pickup_requested or not self.pickup_needs_confirmation:
                raise Fault('invalid_state', 'No upright confirmation requested')
            _, fallen = await self._imu(True)
            if fallen:
                raise Fault('not_ready', 'Place body upright before confirmation')
            self.pickup_needs_confirmation = self.pickup_attempted = False
        elif self.pickup_requested:
            return self.state()
        else:
            self.pickup_attempted = False
            if getattr(self.s, 'voice', None):
                self.s.voice.say('Pick up')
        if self.simple:
            self.simple.cancel_special('pickup')
        self.pickup_requested = self.paused = True
        self.revision += 1
        self._invalidate_observation()
        self._uncertain('pickup_requires_reentry')
        self.info.update(pickup=True, paused=True, state='pickup', phase='pickup',
                         pickup_ready=False, pickup_confirmation_required=self.pickup_needs_confirmation,
                         decision='hold', recovery='cancelled')
        await self._request_graceful()
        return self.state()

    async def _imu(self, fallen_ok=False):
        imu = self.s.body_telemetry.get('body.imu', {})
        stamp = imu.get('source_mono_ns')
        age = (time.monotonic_ns() - stamp) / 1e9 if type(stamp) is int else math.inf
        if not imu.get('valid') or not 0 <= age < .25 or stamp <= self.body_ready_after:
            raise Fault('imu_invalid', 'Fresh body IMU required')
        position = body_position(imu['quaternion_xyzw'])
        fallen = position != 'upright'
        if fallen and not fallen_ok:
            raise Fault('fallen', f'Body is {position}')
        yaw = quaternion_yaw(imu['quaternion_xyzw'])
        if self.odometry:
            self.odometry.heading(yaw)
            self._position()
        return yaw, fallen

    async def _request_graceful(self):
        barrier = self.motion_job, self.revision
        if not self.motion_inflight or self.stop_sent == barrier:
            return
        self.stop_sent = barrier
        try:
            await self._call('motherboard', 'motion.stop_graceful', urgent=True)
        except Fault as exc:
            self.s.log('game', 'WARNING', f'Graceful stop: {exc}')

    def _account(self, ident, job):
        if self.odometry:
            self.odometry.apply(ident, job)
            self._position()

    async def _hard_stop(self):
        result = await self._call('motherboard', 'motion.stop_hard', urgent=True)
        if not result.get('stopped'):
            raise Fault('stop_unconfirmed', 'Body queue stop not confirmed')
        if self.motion_job:
            try:
                self._account(self.motion_job, await self._call('motherboard', 'job.status', {'job_id': self.motion_job}))
            except Fault as exc:
                self.s.log('game', 'WARNING', f'Interrupted job result unavailable: {exc}')
        self.queue_stopped()

    def queue_stopped(self):
        """Called only after an acknowledged reset, including operator takeover."""
        self.motion_inflight = False
        self.motion_job = None
        self.forward_misses = 0
        self.forward_replan = False
        self.needs_prepare = True
        self.head_pan = None
        self._invalidate_observation()

    async def _wait_job(self, response, timeout=20, *, recovery=False, monitor_ball=False, pickup=False, walking=False):
        self.motion_job = response['job_id']
        self.motion_inflight = True
        deadline = time.monotonic() + timeout
        while True:
            job = await self._call('motherboard', 'job.status', {'job_id': self.motion_job})
            if self.simple and self.simple.expired():
                self.simple.cancel_special('30 second deadline during motion')
                await self._request_graceful()
            self._account(self.motion_job, job)
            if walking and job.get('camera_geometry'):
                await self._call('detection', 'ball.geometry', job['camera_geometry'])
            if job['status'] in ('completed', 'failed', 'cancelled'):
                self.motion_inflight = False
                self.motion_job = None
                self._invalidate_observation()
                return job
            if not self._allowed(pickup=pickup):
                await self._request_graceful()
            if not recovery:
                try:
                    _, fallen = await self._imu(True)
                except Fault as exc:
                    if self.stop_sent != (self.motion_job, self.revision):
                        self.s.log('game', 'WARNING', f'Body measurement unavailable during motion: {exc}')
                    await self._request_graceful()
                    fallen = False
                if fallen:
                    self._uncertain('fallen')
                    await self._hard_stop()
                    raise Fault('fallen', 'Fall during body job')
            keeper_live = self.simple is not None and hasattr(self.simple, 'observe_motion_ball')
            if monitor_ball or walking or keeper_live:
                observation = await self._call('detection', 'ball.status')
                self._record_ball(observation)
                if keeper_live and self.simple.move_ball is not None:
                    self.simple.observe_motion_ball(observation)
                result = observation.get('result') or {}
                age = observation.get('age_ms')
                if walking and result.get('valid') and type(age) in (int, float) and 0 <= age <= 500:
                    stamp = result.get('sensor_timestamp_ns')
                    if type(stamp) is int:
                        await self._call('motherboard', 'game.observe', dict(job_id=self.motion_job,
                                         x_m=result['x_m'], y_m=result['y_m'],
                                         stamp=time.monotonic()-(capture_clock()-stamp)/1e9))
                if monitor_ball and (observation.get('error') or time.monotonic() - self.last_seen > self.s.params.values['game.ball_loss_timeout_s']):
                    await self._request_graceful()
                if walking and self.keeper_watch and self.odometry:
                    reason = self.keeper_watch.reason(observation, self.odometry)
                    if reason and self.stop_sent != (self.motion_job, self.revision):
                        self.info['reason'] = reason
                        self.s.log('game', 'INFO', f'Goalkeeper: finish current step, reobserve; {reason}')
                        await self._request_graceful()
            if time.monotonic() >= deadline:
                self._uncertain('motion_timeout')
                await self._hard_stop()
                raise Fault('timeout', 'Body job timed out; queue stopped')
            await asyncio.sleep(.05)

    async def _move(self, op, args, *, pickup=False, recovery=False, monitor_ball=False):
        if not self._allowed(pickup=pickup):
            return False
        revision = self.revision
        self.motion_inflight = True  # Before await: acceptance may race with pause/stop.
        self.motion_job = self.stop_sent = None
        if op == 'game.head':
            self.head_pan = None
        self._invalidate_observation()
        try:
            if self.visual_capture:
                # Locally invalidated before the await. A late computation can
                # never correct odometry once a motion request is in flight.
                try:
                    await self._call('localisation', 'localisation.window', {'window': None}, timeout=.3)
                except Fault as exc:
                    self.s.log('game', 'WARNING', f'Localisation window close: {exc}')
                if revision != self.revision or not self._allowed(pickup=pickup):
                    self.motion_inflight = False
                    return False
            if self.simple and self.simple.expired():
                self.simple.cancel_special('30 second deadline before command')
                self.motion_inflight = False
                return False
            if self.simple and op == 'motion.kick':
                if self.simple.remembered_shot and not self.simple.check_goal_memory():
                    self.info['reason'] = 'goal_memory_invalid_before_kick'
                    self.motion_inflight = False
                    return False
                if self.simple.shot_deadline is None or time.monotonic() >= self.simple.shot_deadline:
                    self.info['reason'] = 'scene_expired_before_kick'
                    self.s.log('game', 'INFO', 'Simple kick withheld: scene_expired_before_kick; reobserve')
                    self.motion_inflight = False
                    return False
                self.simple.deadline = self.simple.kickoff_heading = None
            response = await self._call('motherboard', op, args)
            self.motion_job = response['job_id']
            if revision != self.revision or not self._allowed(pickup=pickup):
                await self._request_graceful()
            job = await self._wait_job(response, 180 if op == 'game.walk' else 30 if recovery else 20, recovery=recovery,
                                       monitor_ball=monitor_ball, pickup=pickup, walking=op == 'game.walk')
            if job['status'] != 'completed':
                self.needs_prepare = True
                self.info['reason'] = job.get('reason') or 'motion_interrupted'
                self.s.log('game', 'WARNING', f"Motion {op}: {job['status']}, reason={self.info['reason']}, pose={job.get('pose')}")
                if job.get('pose') == 'unknown':
                    self._uncertain(self.info['reason'])
                return False
            self.needs_prepare = job.get('pose') != 'base_stand'
            if op == 'game.head':
                self.head_pan = args.get('pan', 0)
            return revision == self.revision and self._allowed(pickup=pickup)
        except Fault as exc:
            self.needs_prepare = True
            self._uncertain(str(exc))
            if self.motion_inflight and not self._allowed(pickup=pickup):
                # The stop sent while acceptance was pending may have reached
                # the worker first. With a lost reply there is no job to track.
                await self._hard_stop()
            raise  # RPC failure is not evidence that no motion was accepted.

    async def _recover(self):
        if self.simple:
            self.simple.cancel_special('fall')
        self.info.update(state='recovering', phase='get_up', recovery='checking')
        while self._allowed():
            try:
                _, fallen = await self._imu(True)
                if not fallen:
                    self.needs_prepare = True
                    return True
                self._uncertain('fallen')
                self.info.update(recovery='getting_up', recovery_attempt=self.info['recovery_attempt'] + 1)
                await self._move('motion.get_up', {'crouch': 'off'}, recovery=True)
            except Fault as exc:
                if self._allowed():
                    self.info.update(recovery='waiting_measurement', reason=str(exc)[:180])
                if self.motion_inflight:
                    return False
            await asyncio.sleep(.1)
        return False

    async def _prepare(self, parameters, entry):
        revision = self.revision
        state = await self._call('motherboard', 'state')
        if not self._same_plan(revision):
            return False
        if not state.get('body_connected') or state.get('error') or state.get('queue_reset_pending'):
            self.needs_prepare = True
            self.body_ready_after = time.monotonic_ns()
            raise Fault('body_unavailable', state.get('error') or 'Waiting for body connection')
        if self.motion_inflight or state.get('active_job'):
            await self._hard_stop()
            self.body_ready_after = time.monotonic_ns()
            return False
        # A new run deliberately rejects samples captured before its start.
        # The polling publisher runs asynchronously (10 Hz); wait for its next
        # sample instead of failing a one-shot test in that initial gap.
        imu_deadline = time.monotonic()+3.
        waiting = False
        while self._same_plan(revision):
            try:
                yaw, fallen = await self._imu(True)
                break
            except Fault as exc:
                if exc.code != 'imu_invalid':
                    raise
                if time.monotonic() >= imu_deadline:
                    raise Fault('imu_invalid', 'No fresh polled body IMU within 3 seconds')
                if not waiting:
                    self.s.log('game', 'INFO', 'Waiting for fresh polled body IMU before preparation')
                    waiting = True
                await asyncio.sleep(.025)
        else:
            return False
        if not self._same_plan(revision):
            return False
        if fallen:
            if self.info.get('role') == 'ball_kick_test':
                raise Fault('fallen', 'Ball kick test requires an upright robot; no automatic get up')
            await self._recover()
            return False
        if self.odometry is None and self.simple is None:
            self.odometry = GameOdometry(parameters, entry, yaw)
            self._position()
        if self.needs_prepare or state.get('pose') != 'base_stand':
            # Finish a known walking pose before restoring the original
            # observation slot. Retaining crouch is a manual-only option.
            if state.get('pose') in ('crouch', 'crouch_centered'):
                if not await self._move('motion.pose', {'name': 'stand'}):
                    return False
            if not await self._move('motion.pose', {'name': 'base_stand'}):
                return False
            if not await self._move('game.head', {'tilt': parameters['game.head_tilt']}):
                return False
            self.needs_prepare = False
        self.info['recovery'] = 'none'
        return self._allowed()

    async def _pickup_wait(self):
        if self.pickup_attempted:
            return
        self.pickup_attempted = True
        try:
            if self.motion_inflight:
                await self._hard_stop()
            _, fallen = await self._imu(True)
            if fallen or self.pickup_needs_confirmation:
                raise Fault('not_ready', 'Place upright and explicitly confirm Pick up')
            if not await self._move('motion.pose', {'name': 'base_stand'}, pickup=True):
                raise Fault('not_ready', 'Pick up stance not completed')
            # A cached upright sample from before base_stand is not readiness.
            completed_at = time.monotonic_ns()
            deadline = time.monotonic() + .5
            while True:
                if not self._allowed(pickup=True):
                    return
                imu = self.s.body_telemetry.get('body.imu', {})
                if (imu.get('valid') and type(imu.get('source_mono_ns')) is int
                        and imu['source_mono_ns'] > completed_at):
                    _, fallen = await self._imu(True)
                    break
                if time.monotonic() >= deadline:
                    raise Fault('imu_invalid', 'No fresh body IMU after Pick up stance')
                await asyncio.sleep(.02)
            if fallen:
                raise Fault('not_ready', 'Body is not upright after Pick up stance')
            self.info.update(pickup_ready=True, pickup_confirmation_required=False,
                             state='pickup_ready', phase='pickup_ready', reason='')
            if getattr(self.s, 'voice', None):
                self.s.voice.say('Ready to pick up')
        except Fault as exc:
            self.pickup_needs_confirmation = True
            self.info.update(pickup_ready=False, pickup_confirmation_required=True,
                             state='pickup_waiting', phase='pickup_waiting', reason=str(exc)[:180])

    def _record_ball(self, obs):
        result = obs.get('result') or {}
        sequence, age = result.get('frame_sequence'), obs.get('age_ms')
        if (obs.get('running') and not obs.get('error') and result.get('valid') and type(sequence) is int
                and sequence > self.last_seen_sequence and type(age) in (int, float) and 0 <= age <= 500):
            self.last_seen_sequence = sequence
            self.last_seen = time.monotonic() - age / 1000

    async def _ball(self):
        await self._poll_localisation()
        obs = await self._call('detection', 'ball.status')
        if obs.get('error') or not obs.get('running'):
            raise Fault('detection_fault', str(obs.get('error') or 'Ball observation stopped'))
        self._record_ball(obs)
        result = obs.get('result') or {}
        stamp, sequence = result.get('sensor_timestamp_ns'), result.get('frame_sequence')
        if (type(stamp) is not int or stamp <= self.observation_after
                or type(sequence) is not int or sequence <= self.last_action_sequence or self.head_pan is None):
            result = result | {'valid': False, 'reason': 'awaiting_support_pose_frame'}
        else:
            result = ball_in_body(result, self.head_pan)
        self.info['ball'] = result
        return obs | {'result': result}

    async def _open_localisation(self, parameters):
        if not parameters.get('game.use_visual_localisation', True):
            return
        if ('localisation' not in self.s.workers or not self.odometry or not self._allowed()
                or self.motion_inflight or self.needs_prepare or self.head_pan != 0
                or time.monotonic() < self.visual_retry_at):
            return
        revision = self.revision
        try:
            yaw, _ = await self._imu()
            prior = [self.odometry.x, self.odometry.y, self.odometry.yaw]
            if self.visual_capture is None:
                camera = await self._call('camera', 'camera.status')
                sync = camera.get('imu_sync', {})
                if not camera.get('running') or sync.get('state') != 'synced':
                    return
                await self._call('localisation', 'localisation.stop')
                self.visual_owned = True
                # Same stationary optical height as the game's ball projection.
                effective = parameters | {k:v for k,v in self.s.params.values.items() if k.startswith('vision.')}
                effective['localisation.camera_height_m'] = parameters['game.camera_height_m']
                capture = uuid.uuid4().hex
                await self._call('localisation', 'localisation.start', dict(mode='game', prior=prior,
                                 capture_id=capture, parameters=effective, unicam_minus_stm=sync['unicam_minus_stm']), timeout=5)
                self.visual_capture = capture
            if revision != self.revision or not self._allowed():
                return
            from .visual_correction import CorrectionWindow
            # Existing strategy settling time is reused, no extra mandatory look.
            window = CorrectionWindow(uuid.uuid4().hex, capture_clock()+200_000_000)
            await self._call('localisation', 'localisation.window', {'window': {
                'id': window.ident, 'after_ns': window.after_ns, 'prior': prior}})
            if revision == self.revision and self._allowed() and not self.motion_inflight:
                self.visual_window = window
                self.visual_poll_at = 0.
        except Fault as exc:
            self.visual_window = self.visual_capture = None
            self.visual_retry_at = time.monotonic()+5
            self.s.log('game', 'WARNING', f'Visual correction unavailable; using odometry: {exc}')

    async def _poll_localisation(self):
        window = self.visual_window
        if (window is None or window.accepted or not self._allowed() or self.motion_inflight
                or self.needs_prepare or self.head_pan != 0 or time.monotonic()<self.visual_poll_at):
            return
        self.visual_poll_at = time.monotonic()+.1
        revision = self.revision
        try:
            state = await self._call('localisation', 'localisation.status', timeout=.2)
            if window is not self.visual_window or revision != self.revision or not self._allowed() or self.motion_inflight:
                return
            if state.get('capture_id') != self.visual_capture or state.get('error') or not state.get('running'):
                raise Fault('localisation_unavailable', state.get('error') or 'Localisation stopped or replaced')
            yaw, _ = await self._imu()
            pose = window.observe(state, capture_clock())
            now = time.monotonic()
            if now >= getattr(self, '_localisation_log_at', 0.):
                self._localisation_log_at = now + 2.
                result = state.get('result') or {}
                self.s.log('game', 'INFO', f"Localisation: fit={result.get('fit_state')} valid={result.get('valid')} ambiguous={result.get('ambiguous')} confirms={len(window.poses)}/3 reason={result.get('reason')}")
                self.s.log('game', 'INFO', f"Localisation: frame={result.get('frame_sequence')} window_match={result.get('window_id') == window.ident} candidate={result.get('candidate')}")
            if pose is not None:
                # Reject large relocalisations in a live strategy. Re-entry sets
                # a new prior; it is never inferred from a teleporting candidate.
                if math.hypot(pose[0]-self.odometry.x,pose[1]-self.odometry.y)>.5 or abs((pose[2]-self.odometry.yaw+math.pi)%(2*math.pi)-math.pi)>.35:
                    self.s.log('game', 'WARNING', 'Visual correction rejected: exceeds stationary innovation limit')
                    return
                self.odometry.correct(pose, yaw)
                self.info['last_correction'] = dict(frame_sequence=window.sequence, sensor_timestamp_ns=window.stamp,
                                                    pose=pose, observations=3)
                self.s.log('game', 'INFO',
                           f'Visual correction applied: x={pose[0]:.2f} y={pose[1]:.2f} '
                           f'yaw={math.degrees(pose[2]):.0f} deg, frame={window.sequence}')
                self._position()
        except Fault as exc:
            self.visual_window = self.visual_capture = None
            self.visual_retry_at = time.monotonic()+5
            self.s.log('game', 'WARNING', f'Visual correction skipped: {exc}')

    async def _start_detection(self, parameters):
        if not parameters.get('game.use_visual_localisation', True) and 'localisation' in self.s.workers:
            try:
                await self._call('localisation', 'localisation.stop', timeout=.5)
            except Fault as exc:
                self.s.log('game', 'WARNING', f'Visual localisation disabled; stopping worker: {exc}')
        revision = self.revision
        camera = await self._call('camera', 'camera.status')
        if not self._same_plan(revision):
            return False
        if not camera.get('running') or camera.get('error'):
            await self.s.dispatch(None, 'camera.start', {}, _from_game=True)
        # Capture shutdown also stops the game observer. Do not restart it
        # midway through that teardown, before camera/IMU have actually stopped.
        async with self.s.capture_lock:
            if not self._same_plan(revision):
                return False
            camera = await self._call('camera', 'camera.status')
            if not camera.get('running') or camera.get('error'):
                raise Fault('camera_fault', camera.get('error') or 'Camera stopped')
            sync = camera.get('imu_sync', {})
            if sync.get('state') != 'synced' or type(sync.get('unicam_minus_stm')) is not int:
                raise Fault('camera_not_ready', 'Waiting for confirmed camera/IMU alignment')
            await self._call('detection', 'ball.stop', timeout=5)
            if not self._same_plan(revision):
                return False
            parameters = parameters | {k: v for k, v in self.s.params.values.items() if k.startswith('vision.')}
            await self._call('detection', 'ball.start', {'parameters': parameters,
                             'unicam_minus_stm': sync['unicam_minus_stm'], 'football': self.simple is not None})
        self.last_seen_sequence = self.last_action_sequence = -1
        self._invalidate_observation()
        return self._same_plan(revision)

    async def _run(self, parameters, role, entry, delay):
        detection_started = detection_ready = False
        if role in ('ball_kick_test', 'simple_football', 'danger_football', 'simple_goalkeeper'):
            # Per-run override only. Never modify the persisted game settings.
            parameters = parameters | {'game.use_visual_localisation': False}
        kickoff = role == 'forward' and entry == 'center' and not delay and parameters['game.forward.kick_off_ride']
        delay_until = time.monotonic() + delay
        try:
            while not self.stop_requested:
                if self.pickup_requested:
                    await self._pickup_wait()
                    await asyncio.sleep(.05)
                    continue
                if self.paused:
                    await asyncio.sleep(.05)
                    continue
                if time.monotonic() < delay_until:
                    self.info.update(state='waiting', phase='start_later')
                    await asyncio.sleep(.05)
                    continue
                try:
                    if not await self._prepare(parameters, entry):
                        await asyncio.sleep(.05)
                        continue
                    if not detection_ready:
                        detection_started = True
                        detection_ready = await self._start_detection(parameters)
                        if not detection_ready:
                            continue
                    if not self._allowed():
                        continue
                    if kickoff:
                        kickoff = False
                        await self._original_kickoff(parameters, entry)
                        continue
                    if role in ('simple_football', 'danger_football', 'simple_goalkeeper'):
                        await self.simple.step()
                    elif role == 'ball_kick_test':
                        try:
                            completed = await asyncio.wait_for(run_ball_kick_test(self, parameters, entry), 120.)
                        except TimeoutError:
                            raise Fault('test_timeout', 'Ball kick test exceeded 120 seconds')
                        if completed:
                            self.info.update(state='completed', phase='completed', reason='kick_completed')
                            self.s.log('game', 'INFO', 'Ball kick test completed; no second kick')
                        elif self._allowed():
                            self.info.update(state='failed', phase='failed', reason=self.info.get('reason') or 'test_interrupted')
                        break
                    elif role == 'FIRA_penalty_Goalkeeper':
                        await self._original_keeper(parameters, entry)
                    else:
                        await self._original_forward(parameters, entry)
                except Fault as exc:
                    if self.simple:
                        self.simple.forget_goal(str(exc))
                    if self.simple and (self.simple.deadline is not None or self.motion_inflight):
                        self.simple.cancel_special(str(exc))
                    if role == 'ball_kick_test':
                        self.info.update(state='failed', phase='failed', reason=str(exc)[:180])
                        self.s.log('game', 'WARNING', f'Ball kick test ended: {exc}')
                        break
                    if self._allowed():
                        self.info.update(state='reconnecting', phase='reconnecting', recovery='waiting', reason=str(exc)[:180])
                    self.s.log('game', 'WARNING', str(exc))
                    if exc.code == 'detection_fault':
                        detection_ready = False
                    await asyncio.sleep(.1)
                await asyncio.sleep(.05)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.info.update(state='failed', phase='failed', reason=str(exc)[:180])
            self.s.log('game', 'ERROR', str(exc))
        finally:
            self.visual_window = None
            stop_confirmed = True
            if self.motion_inflight or self.hard_stop_requested:
                try:
                    await self._hard_stop()
                except Exception as exc:
                    stop_confirmed = False
                    self.info.update(state='failed', phase='stop_unconfirmed', reason=str(exc)[:180])
                    self.s.log('game', 'ERROR', f'Stop unconfirmed: {exc}')
            if self.visual_owned:
                try:
                    await self._call('localisation', 'localisation.stop')
                except Exception as exc:
                    self.s.log('game', 'WARNING', f'Localisation cleanup: {exc}')
                self.visual_capture = None
                self.visual_owned = False
            if detection_started:
                try:
                    await self._call('detection', 'ball.stop', timeout=5)
                except Exception as exc:
                    self.s.log('game', 'ERROR', f'Ball cleanup: {exc}')
            self.info['running'] = not stop_confirmed
            if self.info['state'] not in ('failed', 'completed'):
                self.info.update(state='stopped', phase='stopped')
            if stop_confirmed and self.s.mode == 'GAME':
                self.s.mode = 'MANUAL' if self.s.owner else 'IDLE'
