"""Bounded FIRA penalty goalkeeper. The supervisor remains the motion arbiter."""
import asyncio
import math
import time
import uuid

from .calibration import quaternion_yaw, wrap
from .wire import Fault, boolean, choice, number


def decision(observation, deadband, max_distance):
    """Robot-relative lateral correction; never act on untrusted observations."""
    result = observation.get('result') or {}
    age = observation.get('age_ms')
    if (not observation.get('running') or observation.get('error') or
            not result.get('valid') or not isinstance(age, (int, float)) or
            not math.isfinite(age) or not 0 <= age <= 500):
        return 'hold', result.get('reason') or 'no_fresh_ball'
    x, y = result.get('x_m'), result.get('y_m')
    if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or
            not math.isfinite(v) for v in (x, y)) or
            x <= 0 or math.hypot(x, y) > max_distance):
        return 'hold', 'invalid_ball_geometry'
    if abs(y) <= deadband:
        return 'hold', 'ball_centered'
    return ('left' if y > 0 else 'right'), 'lateral_correction'


class Goalkeeper:
    def __init__(self, supervisor):
        self.s = supervisor
        self.task = None
        self.session = None
        self.stop_lock = asyncio.Lock()
        self.generation = 0
        self.cleaning = False
        self.starting = False
        self.stop_requested = False
        self.hard_stop_requested = False
        self.motion_inflight = False
        self.info = dict(running=False, state='stopped', observe_only=True,
                         reason='', decision='hold', ball=None, travel_m=0., job_id=None)

    def state(self):
        return dict(self.info)

    async def start(self, session, args):
        if self.info['running'] or (self.task and not self.task.done()):
            raise Fault('busy', 'Goalkeeper already running or stopping')
        if set(args) - {'strategy', 'observe_only', 'delay_seconds', 'lease_epoch'}:
            raise Fault('invalid_argument', 'Unknown game settings')
        choice(args, 'strategy', None, ('FIRA_penalty_Goalkeeper',))
        observe = boolean(args, 'observe_only', True)
        delay = number(args, 'delay_seconds', 0, 0, 30, True)
        if self.s.mode != 'MANUAL':
            raise Fault('invalid_state', 'Select MANUAL before starting game')
        self.s.require_motion_ready()
        params = dict(self.s.params.values)
        if not observe and not params['game.geometry_verified']:
            raise Fault('not_ready', 'Verify camera/body geometry in goalkeeper crouch first')
        generation = self.generation
        self.starting = True
        try:
            body = await self.s.workers['motherboard'].call('state')
        finally:
            self.starting = False
        self.s.require_control(session, args)
        if self.info['running'] or self.s.mode != 'MANUAL' or generation != self.generation:
            raise Fault('busy', 'Game state changed during start')
        if body.get('active_job') or not body.get('body_connected') or body.get('error'):
            raise Fault('not_ready', 'Body must be connected and idle without errors')
        if observe and body.get('head', {}).get('pan') != 0:
            raise Fault('not_ready', 'Center head pan before observation-only goalkeeper')
        self.session = session
        self.cleaning = False
        self.stop_requested = self.hard_stop_requested = self.motion_inflight = False
        self.info = dict(running=True, state='preparing', observe_only=observe,
                         reason='', decision='hold', ball=None, travel_m=0.,
                         job_id=uuid.uuid4().hex)
        self.s.mode = 'GAME'
        self.task = asyncio.create_task(self._run(params, delay))
        return self.state()

    async def stop(self, reason='operator_stop', *, hard=False):
        # Escalation must not wait behind another caller awaiting graceful stop.
        self.stop_requested = True
        self.hard_stop_requested |= hard
        if hard and self.task and not self.task.done() and not self.cleaning:
            self.task.cancel()
        async with self.stop_lock:
            self.generation += 1
            task = self.task
            if task and not task.done():
                self.info.update(state='stopping', reason=reason, decision='hold')
                if not self.cleaning and (self.hard_stop_requested or not self.motion_inflight):
                    task.cancel()
                await asyncio.gather(task, return_exceptions=True)
            if self.info['state'] != 'failed':
                self.info.update(running=False, state='stopped', reason=reason, decision='hold')
            if self.s.mode == 'GAME':
                self.s.mode = 'MANUAL' if self.s.owner else 'IDLE'
            return self.state()

    async def _call(self, role, op, args=None, **kwargs):
        return await self.s.workers[role].call(op, args or {}, **kwargs)

    async def _job(self, response, timeout=15):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            job = await self._call('motherboard', 'job.status', {'job_id': response['job_id']})
            if job['status'] == 'completed':
                self.motion_inflight = False
                return
            if job['status'] in ('failed', 'cancelled'):
                raise Fault('motion_fault', job.get('reason') or 'Game motion interrupted')
            await asyncio.sleep(.05)
        raise Fault('timeout', 'Game motion did not complete')

    async def _imu(self):
        data = await self._call('motherboard', 'body.telemetry.read')
        imu = data.get('body.imu', {})
        stamp = imu.get('source_mono_ns')
        age = (time.monotonic_ns() - stamp) / 1e9 if isinstance(stamp, int) else float('inf')
        if not imu.get('valid') or not 0 <= age < .25:
            raise Fault('imu_invalid', 'Fresh body IMU required')
        if abs(imu.get('pitch_deg', 90)) > 25 or abs(imu.get('roll_deg', 90)) > 25:
            raise Fault('imu_invalid', 'Body tilted or fallen; goalkeeper stopped')
        return quaternion_yaw(imu['quaternion_xyzw'])

    async def _run(self, p, delay):
        ball_started = False
        observe = self.info['observe_only']
        failed = False
        try:
            if delay:
                self.info['state'] = 'waiting'
                await asyncio.sleep(delay)
            heading = await self._imu()
            # Observation mode never changes head or body pose.
            if not observe:
                self.motion_inflight = True
                await self._job(await self._call('motherboard', 'motion.pose', {'name': 'crouch'}))
                if self.stop_requested:
                    return
                await self._call('motherboard', 'motion.head', {'pan': 0, 'tilt': p['game.head_tilt']})
                await asyncio.sleep(.5)
            await self.s.dispatch(None, 'camera.start', {}, _from_game=True)
            until = time.monotonic() + 15
            while True:
                camera = await self._call('camera', 'camera.status')
                if camera.get('error') or not camera.get('running'):
                    raise Fault('camera_fault', camera.get('error') or 'Camera stopped')
                if camera.get('imu_sync', {}).get('state') == 'synced':
                    break
                if time.monotonic() > until:
                    raise Fault('timeout', 'Camera/IMU alignment timed out')
                await asyncio.sleep(.1)
            ball_started = True  # Stop even if command reply is lost.
            await self._call('detection', 'ball.start', {'parameters': p,
                'unicam_minus_stm': camera['imu_sync']['unicam_minus_stm']}, timeout=5)
            self.info['state'] = 'observing' if observe else 'tracking'
            last_sequence = -1
            minimum_stamp = 0
            last_seen = time.monotonic()
            self.info['path_m'] = 0.
            while not self.stop_requested:
                if self.s.mode != 'GAME':
                    raise Fault('invalid_state', 'Game mode revoked')
                body = await self._call('motherboard', 'state')
                if not body.get('body_connected') or body.get('error'):
                    raise Fault('body_unavailable', 'Body connection lost')
                yaw = await self._imu()
                if abs(wrap(yaw - heading)) > p['game.max_heading_error_rad']:
                    raise Fault('imu_invalid', 'Heading drift exceeded limit')
                observation = await self._call('detection', 'ball.status')
                if observation.get('error'):
                    raise Fault('detection_fault', observation['error'])
                action, reason = decision(observation, p['game.deadband_m'], p['game.ball_max_distance_m'])
                result = observation.get('result') or {}
                seq = result.get('frame_sequence', -1)
                fresh = isinstance(seq, int) and seq > last_sequence
                if (not isinstance(result.get('sensor_timestamp_ns'), int) or
                        result['sensor_timestamp_ns'] <= minimum_stamp):
                    fresh = False
                if fresh:
                    last_sequence = seq
                    if result.get('valid') and observation.get('age_ms', 1000) <= 500:
                        last_seen = time.monotonic()
                else:
                    action, reason = 'hold', 'waiting_for_new_frame'
                self.info.update(ball=result or None, decision=action, reason=reason)
                if not observe and time.monotonic() - last_seen > p['game.ball_loss_timeout_s']:
                    raise Fault('not_ready', 'Ball lost; explicit restart required')
                if action != 'hold' and not observe:
                    # Bound total path as well as signed excursion; do not reset budget on reversal.
                    reserve = p['game.step_budget_m']
                    if self.info['path_m'] + reserve > p['game.max_path_m']:
                        raise Fault('not_ready', 'Movement budget reached; verify position and restart')
                    sign = 1 if action == 'left' else -1
                    if abs(self.info['travel_m'] + sign * reserve) > p['game.max_excursion_m']:
                        raise Fault('not_ready', 'Goalkeeper excursion limit reached')
                    self.info['path_m'] += reserve
                    self.info['travel_m'] += sign * reserve
                    self.info['state'] = 'stepping'
                    self.motion_inflight = True
                    response = await self._call('motherboard', 'game.step', {
                        'direction': action, 'heading': heading,
                        'side_mm': p['game.side_step_mm'], 'cycles': p['game.step_cycles']})
                    await self._monitor_step(response, heading, p)
                    # Wait for an observation acquired after motion completion.
                    minimum_stamp = time.clock_gettime_ns(getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC))
                    self.info['state'] = 'tracking'
                await asyncio.sleep(.1)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            failed = True
            self.info.update(state='failed', reason=str(exc)[:180], decision='hold')
            self.s.log('game', 'ERROR', str(exc))
        finally:
            self.cleaning = True
            # A failed/unknown active motion requires the urgent barrier. A normal
            # exit after the terminal gait cycle must preserve its supported pose.
            if not observe and (self.motion_inflight or self.hard_stop_requested):
                self.s.motion_ready = False
                try:
                    await self._call('motherboard', 'motion.stop_hard', urgent=True)
                    self.s.motion_ready = True
                except Exception as exc:
                    self.s.motion_ready = False
                    failed = True
                    self.info.update(state='failed', reason=f'Stop unconfirmed: {exc}'[:180])
            if ball_started:
                try:
                    await self._call('detection', 'ball.stop', timeout=5)
                except Exception as exc:
                    self.s.log('game', 'ERROR', f'Ball cleanup: {exc}')
            # Keep shared camera running for operator video/calibration.
            self.info.update(running=False, decision='hold')
            self.cleaning = False
            if not failed:
                self.info['state'] = 'stopped'
            if self.s.mode == 'GAME':
                self.s.mode = 'MANUAL' if self.s.owner else 'IDLE'
            if self.s.head_menu and self.session is self.s.local_session:
                self.s.spawn(self.s.head_menu.game_finished(self.state()))

    async def _monitor_step(self, response, heading, p):
        until = time.monotonic() + 12
        finishing = False
        while time.monotonic() < until:
            job = await self._call('motherboard', 'job.status', {'job_id': response['job_id']})
            if job['status'] == 'completed':
                self.motion_inflight = False
                return
            if job['status'] in ('failed', 'cancelled'):
                raise Fault('motion_fault', job.get('reason') or 'Game step interrupted')
            if abs(wrap(await self._imu() - heading)) > p['game.max_heading_error_rad']:
                raise Fault('imu_invalid', 'Heading drift during step')
            status = await self._call('detection', 'ball.status')
            result = status.get('result') or {}
            missing = (status.get('error') or not result.get('valid') or
                       status.get('age_ms') is None or status['age_ms'] > 500)
            if (self.stop_requested or missing) and not finishing:
                await self._call('motherboard', 'motion.stop_graceful', urgent=True)
                finishing = True
                self.info.update(state='finishing_step', decision='hold',
                                 reason='operator_stop' if self.stop_requested else 'ball_observation_lost')
            await asyncio.sleep(.1)
        raise Fault('timeout', 'Goalkeeper step timed out')
