import asyncio
import math
import time
from types import SimpleNamespace

import pytest

from roki_ng.game import Goalkeeper
from roki_ng.legacy_strategy import OriginalStrategy
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


def observation(y=.2, seq=1):
    return dict(running=True, error=None, age_ms=10,
                result=dict(valid=True, frame_sequence=seq,
                            sensor_timestamp_ns=time.clock_gettime_ns(getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC)),
                            x_m=1., y_m=y))


@pytest.mark.parametrize('y,expected', [(.2, 'left'), (-.2, 'right'), (.05, 'hold')])
def test_robot_relative_decision(tmp_path, y, expected):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        assert await game._prepare(s.params.values, 'goalkeeper')
        async def observed(*args, **kwargs): return observation(y)
        game._original_observe = observed
        await game._original_keeper(s.params.values, 'goalkeeper')
        moves = [args for op, args in s.workers['motherboard'].calls if op == 'game.walk']
        if expected == 'hold':
            assert not moves and game.info['decision'] == 'hold'
        else:
            assert moves[0]['distance_m'] == min(.1, abs(y))
            assert moves[0]['course'] == math.copysign(math.pi/2, y)
    asyncio.run(run())


@pytest.mark.parametrize('change', [{'age_ms': 501}, {'running': False}, {'error': 'lost'}, {'age_ms': None}])
def test_invalid_observations_never_move(change):
    assert OriginalStrategy._usable(observation() | change) is None


class Worker:
    def __init__(self, role):
        self.role = role
        self.calls = []
        self.alive = True
        self.seq = 0
        self.bad_ball = False
        self.bad_imu = False
        self.pose = 'unknown'
        self.jobs = {}
        self.publisher = None

    def polled_telemetry(self):
        return {'body.imu': dict(valid=not self.bad_imu, source_mono_ns=time.monotonic_ns(),
                                quaternion_xyzw=[2**-.5, 0, 0, 2**-.5], pitch_deg=0, roll_deg=0)}

    async def call(self, op, args=None, **kwargs):
        self.calls.append((op, args or {}))
        if self.publisher:
            self.publisher(self.polled_telemetry())
        if op == 'state':
            return dict(body_connected=True, active_job=None, error=None, pose=self.pose, head={'pan': 0})
        if op == 'body.telemetry.read':
            pytest.fail('Game must consume published IMU, not request telemetry')
        if op == 'camera.status':
            return dict(running=True, imu_sync={'state': 'synced', 'unicam_minus_stm': 5})
        if op == 'ball.status':
            self.seq += 1
            return observation(seq=self.seq) | ({'error': 'no_camera'} if self.bad_ball else {})
        if op in ('motion.pose', 'game.walk', 'game.head', 'game.jump', 'motion.get_up', 'motion.kick'):
            args = args or {}
            self.pose = args.get('name', 'stand' if op in ('game.jump', 'motion.get_up', 'motion.kick') else self.pose)
            ident = str(len(self.jobs) + 1)
            segments = []
            if op == 'game.walk':
                assert self.pose == 'base_stand'
                self.pose = 'base_stand'
                segments = [dict(index=1, dx_m=0., dy_m=.01, imu_yaw_rad=0.)]
            self.jobs[ident] = dict(status='completed', pose=self.pose, segments=segments)
            return {'job_id': ident}
        if op == 'job.status':
            return self.jobs[args['job_id']]
        if op == 'motion.stop_hard':
            self.pose = 'unknown'
            return {'stopped': True}
        if op == 'control.takeover':
            self.pose = 'unknown'
            return {'stop_confirmed': True}
        return {}


def setup(tmp_path):
    workers = {role: Worker(role) for role in ('motherboard', 'camera', 'detection')}
    session = SimpleNamespace(id=1, closed=False)
    async def dispatch(*args, **kwargs):
        return {}
    class SupervisorState(SimpleNamespace):
        @property
        def body_telemetry(self):
            return workers['motherboard'].polled_telemetry()

    s = SupervisorState(mode='MANUAL', owner=1, lease_epoch=1, workers=workers,
                        parameter_lock=asyncio.Lock(), capture_lock=asyncio.Lock(),
                        motion_ready=True, head_menu=None, local_session=None,
                        params=Parameters(tmp_path), dispatch=dispatch,
                        require_motion_ready=lambda: None,
                        require_control=lambda *args: None, log=lambda *args: None)
    game = Goalkeeper(s)
    async def settled(seconds):
        await asyncio.sleep(0)
        return game._allowed()
    game._settle = settled
    return game, s, session


async def until(predicate):
    for _ in range(200):
        if predicate(): return
        await asyncio.sleep(.01)
    assert predicate()


def test_goalkeeper_start_prepares_body_and_stops_detection(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await until(lambda: any(op == 'game.walk' for op, _ in s.workers['motherboard'].calls))
        assert s.mode == 'GAME'
        await game.stop()
        assert not game.state()['running'] and s.mode == 'MANUAL'
        assert ('motion.pose', {'name': 'base_stand'}) in s.workers['motherboard'].calls
        assert ('motion.pose', {'name': 'crouch'}) not in s.workers['motherboard'].calls
        assert any(op == 'ball.stop' for op, _ in s.workers['detection'].calls)
    asyncio.run(run())


def test_goalkeeper_does_not_require_manual_geometry_flag(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await until(lambda: game.state()['state'] == 'tracking')
        assert s.mode == 'GAME'
        await game.stop()
    asyncio.run(run())


def test_cancel_delay_prevents_later_start(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper', 'delay_seconds': 10})
        await asyncio.sleep(.01)
        await game.stop()
        assert not s.workers['detection'].calls
        assert not game.state()['running']
    asyncio.run(run())


def test_detection_fault_enters_reconnect_and_resumes(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        s.workers['detection'].bad_ball = True
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await until(lambda: game.state()['state'] == 'reconnecting')
        s.workers['detection'].bad_ball = False
        await until(lambda: game.state()['state'] == 'tracking')
        assert game.state()['running']
        await game.stop()
    asyncio.run(run())


def test_immediate_stop_clears_game_mode(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await game.stop()
        assert s.mode == 'MANUAL' and not game.state()['running']
        assert not s.workers['detection'].calls
    asyncio.run(run())


def test_stop_cancels_start_still_checking_body(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        original = s.workers['motherboard'].call
        gate = asyncio.Event()
        async def delayed(op, *args, **kwargs):
            await gate.wait()
            return await original(op, *args, **kwargs)
        s.workers['motherboard'].call = delayed
        task = asyncio.create_task(game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'}))
        await asyncio.sleep(0)
        await game.stop()
        gate.set()
        with pytest.raises(Fault): await task
        assert s.mode == 'MANUAL' and not game.state()['running']
    asyncio.run(run())


def test_supervisor_game_protocol_ownership_and_barriers(tmp_path):
    from roki_ng.supervisor import Supervisor, Session
    async def run():
        server = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        server.workers = {r: Worker(r) for r in ('motherboard', 'camera', 'detection')}
        server.mode = 'MANUAL'
        server.owner, server.lease_epoch = 1, 3
        owner = Session(1, 2, (), 'test')
        foreign = Session(2, 3, (), 'other')
        server.game.info['running'] = True
        server.mode = 'GAME'
        assert (await server.dispatch(foreign, 'game.status', {}))['running']
        with pytest.raises(Fault): await server.dispatch(foreign, 'game.stop', {'lease_epoch': 3})
        for op, args in [('motion.head', {'tilt': -1000}),
                         ('params.set', {'key':'walk.gait_height_mm','value':180}), ('camera.stop', {})]:
            with pytest.raises(Fault, match='Stop goalkeeper'):
                await server.dispatch(owner, op, args | {'lease_epoch': 3})
        result = await server.dispatch(owner, 'game.stop', {'lease_epoch': 3})
        assert not result['running'] and server.mode == 'MANUAL'
        assert server.workers['motherboard'].calls == []
    asyncio.run(run())


def test_head_menu_game_uses_game_stop_and_status():
    from roki_ng.head_menu import HeadMenu, Item
    async def run():
        calls, voices = [], []
        async def command(op, args):
            calls.append(op)
            return {'job_id': 'game1', 'running': op != 'game.stop', 'role': 'FIRA_penalty_Goalkeeper', 'state': 'stopped'}
        async def release(): calls.append('release')
        menu = HeadMenu(command, release, voices.append, lambda *a: None)
        await menu._start(Item('Start', op='game.start', args={}))
        assert menu.game_active and 'Goalkeeper started' in voices
        await menu._cancel()
        assert calls == ['game.start', 'game.status', 'game.stop', 'release']
        assert not menu.game_active and menu.job is None
    asyncio.run(run())


def test_emergency_stop_cancels_pending_supervisor_start(tmp_path):
    from roki_ng.supervisor import Supervisor, Session
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        s.workers = {r: Worker(r) for r in ('motherboard', 'camera', 'detection')}
        s.mode, s.owner, s.lease_epoch = 'MANUAL', 1, 3
        owner = Session(1, 2, (), 'test')
        gate = asyncio.Event()
        original = s.workers['motherboard'].call
        async def delayed(op, *args, **kwargs):
            if op == 'state': await gate.wait()
            return await original(op, *args, **kwargs)
        s.workers['motherboard'].call = delayed
        task = asyncio.create_task(s.dispatch(owner, 'game.start', {
            'strategy': 'FIRA_penalty_Goalkeeper', 'lease_epoch': 3}))
        await asyncio.sleep(0)
        await s.dispatch(owner, 'motion.stop_hard', {'lease_epoch': 3})
        gate.set()
        with pytest.raises(Fault): await task
        assert not s.game.state()['running'] and s.mode == 'MANUAL'
    asyncio.run(run())


def test_queued_parameter_write_rechecks_game_after_lock(tmp_path):
    from roki_ng.supervisor import Supervisor
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        await s.parameter_lock.acquire()
        task = asyncio.create_task(s._set_parameters({'game.kick_distance_mm': 120.}))
        await asyncio.sleep(0)
        s.game.info['running'] = True
        s.parameter_lock.release()
        with pytest.raises(Fault, match='Stop goalkeeper'): await task
        assert s.params.values['game.kick_distance_mm'] == 100.
    asyncio.run(run())


def test_stop_discards_game_start_waiting_for_parameter_lock(tmp_path):
    from roki_ng.supervisor import Supervisor, Session
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        s.workers = {r: Worker(r) for r in ('motherboard', 'camera', 'detection')}
        s.mode, s.owner, s.lease_epoch = 'MANUAL', 1, 3
        owner = Session(1, 2, (), 'test')
        await s.parameter_lock.acquire()
        task = asyncio.create_task(s.dispatch(owner, 'game.start', {
            'strategy': 'FIRA_penalty_Goalkeeper', 'lease_epoch': 3}))
        await asyncio.sleep(0)
        await s.dispatch(owner, 'game.stop', {'lease_epoch': 3})
        s.parameter_lock.release()
        with pytest.raises(Fault, match='stop barrier'): await task
        assert s.workers['motherboard'].calls == []
        assert not s.game.state()['running']
    asyncio.run(run())


def test_full_parameter_snapshot_fits_worker_ipc(tmp_path):
    from roki_ng.wire import IPC_LIMIT, pack, unpack
    message = {'body': {'parameters': Parameters(tmp_path).values}}
    assert unpack(pack(message, IPC_LIMIT), IPC_LIMIT) == message
    # The larger internal map limit must not relax untrusted operator messages.
    public = pack({'x': {str(i): 0 for i in range(129)}})
    with pytest.raises(Fault, match='MessagePack'): unpack(public)


def test_lost_ball_finishes_step_without_hard_stop(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        worker = s.workers['motherboard']
        original = worker.call
        polls = 0
        async def call(op, args=None, **kwargs):
            nonlocal polls
            if op == 'job.status':
                polls += 1
                return {'status':'completed' if polls == 3 else 'running'}
            return await original(op, args, **kwargs)
        worker.call = call
        s.workers['detection'].bad_ball = True
        game.motion_inflight = True
        await game._wait_job({'job_id':'step'}, monitor_ball=True)
        assert not game.motion_inflight
        assert [op for op, _ in worker.calls].count('motion.stop_graceful') == 1
        assert not any(op == 'motion.stop_hard' for op, _ in worker.calls)
    asyncio.run(run())


def test_graceful_game_stop_waits_for_support_pose(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        game.session = session
        game.info.update(running=True)
        game.motion_inflight = True
        entered, completed = asyncio.Event(), asyncio.Event()
        async def step():
            entered.set()
            await completed.wait()
            game.motion_inflight = False
        game.task = asyncio.create_task(step())
        await entered.wait()
        stopping = asyncio.create_task(game.stop())
        await asyncio.sleep(0)
        assert game.stop_requested and not stopping.done() and not game.task.cancelled()
        completed.set()
        await stopping
        assert not game.task.cancelled()
    asyncio.run(run())


@pytest.mark.parametrize('interruption', ['hard_stop', 'link_error'])
def test_interrupted_exit_keeps_completed_steps_but_not_confirmed_stance(tmp_path, body_clock, interruption):
    from roki_ng.body import Body
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values}, lambda *a: None, lambda *a: None)
    body.command('control.acquire', {})
    body.pose = 'base_stand'
    original_pose = body._pose
    leaving = False
    def pose(name, **kwargs):
        nonlocal leaving
        if name == 'stand':
            leaving = True
            if interruption == 'link_error':
                raise Fault('hardware_error', 'Exit transmission failed')
            yield 'sleep', 10
        yield from original_pose(name, **kwargs)
    body._pose = pose
    ident = body.command('game.walk', {'kind': 'omni', 'distance_m': .1, 'course': -math.pi/2})['job_id']
    for _ in range(500):
        body.tick()
        if leaving:
            break
    assert leaving
    if interruption == 'hard_stop':
        body.command('motion.stop_hard', {})
    job = body.jobs[ident]
    assert job['status'] in ('cancelled', 'failed') and job['execution_uncertain']
    assert len(job['segments']) > 2 and job['pose'] == 'unknown'
    assert body.plan is None


@pytest.mark.parametrize('role,entry', [('FIRA_penalty_Goalkeeper', 'goalkeeper'), ('forward', 'left')])
@pytest.mark.parametrize('previous_pose', ['unknown', 'crouch', 'crouch_centered', 'stand'])
def test_autonomous_preparation_restores_base_stand_not_held_crouch(tmp_path, role, entry, previous_pose):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info.update(running=True, role=role)
        worker = s.workers['motherboard']
        worker.pose = previous_pose
        assert await game._prepare(s.params.values, entry)
        poses = [args['name'] for op, args in worker.calls if op == 'motion.pose']
        assert poses == (['stand'] if previous_pose.startswith('crouch') else []) + ['base_stand']
        assert worker.pose == 'base_stand' and not game.needs_prepare
        assert game.head_pan == 0
    asyncio.run(run())


@pytest.mark.parametrize('op,args', [('game.jump', {'direction': 'forward', 'fraction': 1.}),
                                    ('motion.kick', {'leg': 'right', 'power': 80, 'offset': 0}),
                                    ('motion.get_up', {'crouch': 'off'})])
def test_autonomous_action_requires_new_base_stand_and_fresh_observation(tmp_path, op, args):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info.update(running=True)
        assert await game._prepare(s.params.values, 'goalkeeper')
        old_frame = observation()
        assert await game._move(op, args)
        assert game.needs_prepare
        assert await game._prepare(s.params.values, 'goalkeeper')
        worker = s.workers['motherboard']
        action = next(i for i, (name, _) in enumerate(worker.calls) if name == op)
        assert ('motion.pose', {'name': 'base_stand'}) in worker.calls[action + 1:]
        assert worker.pose == 'base_stand'
        async def stale(*a, **kw): return old_frame
        s.workers['detection'].call = stale
        assert not (await game._ball())['result']['valid']
    asyncio.run(run())


@pytest.mark.parametrize('disconnect', [False, True])
def test_game_survives_operator_release_and_reacquisition(tmp_path, disconnect):
    from roki_ng.supervisor import Supervisor, Session
    async def run():
        s = Supervisor({'simulate':True,'state_dir':str(tmp_path)})
        s.workers = {r:Worker(r) for r in ('motherboard','camera','detection','stream')}
        s.worker_event('motherboard', 'body.telemetry', s.workers['motherboard'].polled_telemetry())
        s.workers['motherboard'].publisher = lambda data: s.worker_event('motherboard', 'body.telemetry', data)
        s.mode, s.owner, s.lease_epoch = 'MANUAL', 1, 3
        owner, viewer = Session(1,2,(),'owner'), Session(2,3,(),'viewer')
        original = s.workers['camera'].call
        async def camera(op, args=None, **kwargs):
            if op == 'camera.status':
                return {'running':True,'prepared':True,'frame_duration_us':16667,
                        'imu_sync':{'state':'synced','unicam_minus_stm':5}}
            return await original(op,args,**kwargs)
        s.workers['camera'].call = camera
        s.sessions[1] = owner
        await s.dispatch(owner,'game.start',{'strategy':'FIRA_penalty_Goalkeeper','lease_epoch':3})
        # Disconnect before the background task even starts its camera request.
        if disconnect:
            await s._expire(owner)
        else:
            await s.dispatch(owner,'control.release',{'lease_epoch':3})
        await until(lambda:any(op=='game.walk' for op,_ in s.workers['motherboard'].calls))
        assert s.owner is None and s.mode=='GAME'
        assert not any(op=='control.release' for op,_ in s.workers['motherboard'].calls)
        lease = await s.dispatch(viewer,'control.acquire',{})
        assert s.game.state()['running'] and s.mode=='GAME'
        assert not any(op=='control.acquire' for op,_ in s.workers['motherboard'].calls)
        await s.dispatch(viewer,'game.stop',lease)
        assert not s.game.state()['running'] and s.mode=='MANUAL'
    asyncio.run(run())


def test_ball_loss_clock_counts_observations_during_step(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        game.last_seen = time.monotonic() - 5.
        worker = s.workers['motherboard']
        original = worker.call
        polls = 0
        async def call(op, args=None, **kwargs):
            nonlocal polls
            if op == 'job.status':
                polls += 1
                return {'status':'completed' if polls == 2 else 'running'}
            return await original(op,args,**kwargs)
        worker.call = call
        await game._wait_job({'job_id':'step'}, monitor_ball=True)
        assert time.monotonic() - game.last_seen < .5
        seen = game.last_seen
        game._record_ball(observation(seq=1))
        assert game.last_seen == seen  # Repeated frame cannot refresh it.
        game._record_ball(observation(seq=2) | {'age_ms':501})
        assert game.last_seen == seen
    asyncio.run(run())


def test_game_allows_live_vision_isp_logging_and_diagnostic_settings(tmp_path):
    from roki_ng.supervisor import Supervisor, Session
    async def run():
        s = Supervisor({'simulate':True,'state_dir':str(tmp_path)})
        s.workers = {r:Worker(r) for r in ('motherboard','camera','detection','localisation')}
        s.owner, s.lease_epoch, s.mode = 1, 3, 'GAME'
        s.game.info['running'] = True
        owner = Session(1,2,(),'owner')
        values = {'vision.orange_ball.pixels_min':80,'camera.analogue_gain':2.,
                  'logging.stdout_enabled':True,'localisation.max_speed_m_s':.4}
        result = await s.dispatch(owner,'params.set',{'values':values,'lease_epoch':3})
        assert result['values'] == values
        assert not s.workers['motherboard'].calls
        assert ('params.apply',{'vision.orange_ball.pixels_min':80}) in s.workers['detection'].calls
        assert s.params.values['localisation.max_speed_m_s'] == .4
        await s.dispatch(owner,'camera.controls.set',{'values':{'camera.analogue_gain':3.},'lease_epoch':3})
        assert s.params.values['camera.analogue_gain'] == 2.  # Volatile ISP edit.
        for key, value in (('walk.gait_height_mm',190), ('game.camera_height_m',.5), ('stabilization.enabled',True)):
            with pytest.raises(Fault, match='Stop goalkeeper'):
                await s.dispatch(owner,'params.set',{'key':key,'value':value,'lease_epoch':3})
        assert s.game.info['running'] and s.mode=='GAME'
    asyncio.run(run())


def test_ball_start_uses_thresholds_changed_during_preparation(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        await game.start(session,{'strategy':'FIRA_penalty_Goalkeeper'})
        s.params.values['vision.orange_ball.pixels_min'] = 123
        await until(lambda:game.state()['state']=='tracking')
        args = next(args for op,args in s.workers['detection'].calls if op=='ball.start')
        assert args['parameters']['vision.orange_ball.pixels_min']==123
        await game.stop()
    asyncio.run(run())


def test_colour_diagnostics_available_but_game_owns_localisation(tmp_path):
    from roki_ng.supervisor import Supervisor, Session
    async def run():
        s = Supervisor({'simulate':True,'state_dir':str(tmp_path)})
        s.workers = {r:Worker(r) for r in ('motherboard','camera','detection','localisation','stream')}
        s.owner, s.lease_epoch, s.mode = 1, 3, 'GAME'
        s.game.info['running'] = True
        owner = Session(1,2,(),'owner')
        for op in ('detection.start','detection.stop'):
            await s.dispatch(owner,op,{'lease_epoch':3})
        for op in ('localisation.start','localisation.stop'):
            with pytest.raises(Fault, match='Stop'):
                await s.dispatch(owner,op,{'lease_epoch':3})
        assert not s.workers['motherboard'].calls
        assert s.game.info['running'] and s.mode=='GAME'
    asyncio.run(run())


def test_pause_during_delayed_start_prevents_preparation(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        await game.start(session, {'strategy': 'forward', 'delay_seconds': 1})
        await game.pause()
        await asyncio.sleep(1.05)
        assert game.state()['state'] == 'paused'
        assert s.workers['motherboard'].calls == [('state', {})]
        assert not s.workers['detection'].calls
        await game.resume()
        await until(lambda: any(op == 'game.head' for op, _ in s.workers['motherboard'].calls))
        await game.stop()
    asyncio.run(run())


@pytest.mark.parametrize('barrier', ['pause', 'pickup'])
def test_barrier_during_acceptance_stops_job_and_forbids_successor(tmp_path, barrier):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        worker = s.workers['motherboard']
        original = worker.call
        accepted, reply = asyncio.Event(), asyncio.Event()
        async def delayed(op, args=None, **kwargs):
            result = await original(op, args, **kwargs)
            if op == 'game.jump':
                accepted.set()
                await reply.wait()
            return result
        worker.call = delayed
        task = asyncio.create_task(game._move('game.jump', {'direction': 'forward', 'fraction': 1.}))
        await accepted.wait()
        await getattr(game, barrier)()
        assert game.motion_inflight
        reply.set()
        assert not await task
        assert not await game._move('motion.kick', {'leg': 'right', 'power': 80})
        assert any(op == 'motion.stop_graceful' for op, _ in worker.calls)
        assert not any(op == 'motion.kick' for op, _ in worker.calls)
    asyncio.run(run())


def test_pickup_lying_requires_confirmation_and_completed_stance(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        spoken = []
        s.voice = SimpleNamespace(say=spoken.append)
        game.info['running'] = True
        worker = s.workers['motherboard']
        upright = worker.polled_telemetry
        worker.polled_telemetry = lambda: {'body.imu': upright()['body.imu'] | {'quaternion_xyzw': [0, 0, 0, 1]}}
        await game.pickup()
        await game.pickup()  # Holding the button cannot repeat speech or a stance.
        await game._pickup_wait()
        assert game.state()['pickup_confirmation_required'] and not game.state()['pickup_ready']
        assert not worker.calls and spoken == ['Pick up']
        with pytest.raises(Fault):
            await game.pickup({'confirm': True})
        worker.polled_telemetry = upright
        await game._pickup_wait()
        assert not worker.calls  # Becoming upright alone never resumes movement.
        await game.pickup({'confirm': True})
        await game._pickup_wait()
        assert game.state()['pickup_ready']
        assert spoken == ['Pick up', 'Ready to pick up']
        assert ('motion.pose', {'name': 'base_stand'}) in worker.calls
        assert not any(op == 'motion.get_up' for op, _ in worker.calls)
    asyncio.run(run())


def test_failed_pickup_stance_never_announces_readiness(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        spoken = []
        s.voice = SimpleNamespace(say=spoken.append)
        game.info['running'] = True
        worker = s.workers['motherboard']
        original = worker.call
        async def failed(op, args=None, **kwargs):
            result = await original(op, args, **kwargs)
            if op == 'job.status':
                return result | {'status': 'failed', 'pose': 'unknown', 'reason': 'send uncertain'}
            return result
        worker.call = failed
        await game.pickup()
        await game._pickup_wait()
        assert not game.state()['pickup_ready'] and game.state()['pickup_confirmation_required']
        assert spoken == ['Pick up']
    asyncio.run(run())


def test_motion_timeout_requires_confirmed_queue_stop(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        stopped = [False]
        async def call(op, args=None, **kwargs):
            if op == 'job.status':
                return {'status': 'running'}
            if op == 'motion.stop_hard':
                return {'stopped': stopped[0]}
            raise AssertionError(op)
        s.workers['motherboard'].call = call
        with pytest.raises(Fault) as error:
            await game._wait_job({'job_id': 'unknown'}, timeout=0)
        assert error.value.code == 'stop_unconfirmed' and game.motion_inflight
        stopped[0] = True
        with pytest.raises(Fault) as error:
            await game._wait_job({'job_id': 'unknown'}, timeout=0)
        assert error.value.code == 'timeout' and not game.motion_inflight
    asyncio.run(run())


def test_frames_before_support_pose_or_already_used_do_not_authorize_action(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        obs = observation()
        async def call(*args, **kwargs): return obs
        s.workers['detection'].call = call
        game.head_pan = 0  # Acknowledged head pose; frame age alone is insufficient.
        game._invalidate_observation()
        assert not (await game._ball())['result']['valid']
        obs = observation(seq=2)
        assert (await game._ball())['result']['valid']
        game.last_action_sequence = 2
        assert not (await game._ball())['result']['valid']
    asyncio.run(run())


def test_repeated_failed_getups_are_cancelled_by_pickup(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        worker = s.workers['motherboard']
        original, upright = worker.call, worker.polled_telemetry
        worker.polled_telemetry = lambda: {'body.imu': upright()['body.imu'] | {'quaternion_xyzw': [0, 0, 0, 1]}}
        async def failed_getup(op, args=None, **kwargs):
            result = await original(op, args, **kwargs)
            if op == 'job.status':
                return result | {'status': 'failed', 'pose': 'unknown', 'reason': 'still lying'}
            return result
        worker.call = failed_getup
        task = asyncio.create_task(game._recover())
        await until(lambda: game.state()['recovery_attempt'] >= 3)
        await game.pickup()
        assert not await task
        attempts = game.state()['recovery_attempt']
        await game._pickup_wait()
        assert game.state()['recovery_attempt'] == attempts
        assert not game.state()['pickup_ready']
    asyncio.run(run())


def test_reentry_discards_old_odometry_and_ball_without_closing_camera(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await until(lambda: game.odometry is not None)
        old_job, old_odometry = game.state()['job_id'], game.odometry
        game.odometry.y = .4
        await game.pickup()
        await until(lambda: game.state()['pickup_ready'])
        await game.start(session, {'strategy': 'forward', 'entry': 'right', 'delay_seconds': 10})
        assert game.odometry is None and game.state()['ball'] is None
        assert game.last_action_sequence == -1 and game.state()['job_id'] != old_job
        assert game.state()['entry'] == 'right'
        assert not any(op in ('camera.stop', 'close') for w in s.workers.values() for op, _ in w.calls)
        assert game.odometry is not old_odometry
        await game.stop()
    asyncio.run(run())


def test_unknown_movement_is_stopped_and_prepared_not_replayed(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        worker = s.workers['motherboard']
        original = worker.call
        async def lost_reply(op, args=None, **kwargs):
            result = await original(op, args, **kwargs)
            if op == 'game.jump':
                raise Fault('worker_timeout', 'Movement may have executed')
            return result
        worker.call = lost_reply
        with pytest.raises(Fault):
            await game._move('game.jump', {'direction': 'forward', 'fraction': 1.})
        assert game.motion_inflight
        assert not await game._prepare(s.params.values, 'center')
        assert await game._prepare(s.params.values, 'center')
        operations = [op for op, _ in worker.calls]
        assert operations.count('game.jump') == 1
        assert operations.index('motion.stop_hard') < operations.index('motion.pose') < operations.index('game.head')
    asyncio.run(run())


@pytest.mark.parametrize('entry,enabled,delay,expected', [
    ('center', True, 0, 1), ('center', False, 0, 0), ('center', True, 10, 0),
    ('left', True, 0, 0), ('right', True, 0, 0),
])
def test_only_immediate_enabled_center_uses_kickoff(tmp_path, monkeypatch, entry, enabled, delay, expected):
    from roki_ng import game as module
    now = [time.monotonic()]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: now[0],
        monotonic_ns=time.monotonic_ns, clock_gettime_ns=time.clock_gettime_ns, CLOCK_BOOTTIME=time.CLOCK_BOOTTIME))
    async def run():
        game, s, session = setup(tmp_path)
        s.params.values['game.forward.kick_off_ride'] = enabled
        original = s.workers['detection'].call
        async def no_ball(op, args=None, **kwargs):
            result = await original(op, args, **kwargs)
            if op == 'ball.status':
                return result | {'result': result['result'] | {'valid': False, 'reason': 'no_ball'}}
            return result
        s.workers['detection'].call = no_ball
        await game.start(session, {'strategy': 'forward', 'entry': entry, 'delay_seconds': delay})
        if delay:
            await until(lambda: game.state()['state'] == 'waiting')
            now[0] += delay + .1
        await until(lambda: any(op == 'ball.status' for op, _ in s.workers['detection'].calls))
        await game.stop()
        jumps = [args for op, args in s.workers['motherboard'].calls if op == 'game.walk' and args['kind'] == 'kickoff']
        assert len(jumps) == expected
    asyncio.run(run())


def test_keeper_heading_is_rebased_for_arbitrary_imu_and_own_goal(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        s.params.values['match.own_goal'] = 1
        yaw = 1.6
        c, z = math.cos(yaw / 2) / math.sqrt(2), math.sin(yaw / 2) / math.sqrt(2)
        worker = s.workers['motherboard']
        original = worker.polled_telemetry
        worker.polled_telemetry = lambda: {'body.imu': original()['body.imu'] | {'quaternion_xyzw': [c, z, z, c]}}
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await until(lambda: any(op == 'game.walk' for op, _ in worker.calls))
        await game.pause()
        args = next(args for op, args in worker.calls if op == 'game.walk')
        assert args['heading'] == pytest.approx(0.)
        assert args['yaw_offset'] == pytest.approx(-yaw)
        assert args['distance_m'] == pytest.approx(.1)
        assert args['course'] == pytest.approx(math.pi/2)
        await game.stop()
    asyncio.run(run())


def test_takeover_after_unconfirmed_game_stop_grants_lease_but_not_motion(tmp_path):
    from roki_ng.supervisor import Supervisor, Session
    async def run():
        server = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        server.mode = 'GAME'
        server.game.info['running'] = server.game.motion_inflight = True
        confirmed = [False]
        calls = []
        async def call(op, *args, **kwargs):
            calls.append(op)
            if op == 'motion.stop_hard':
                raise Fault('worker_timeout', 'No stop acknowledgement')
            assert op == 'control.takeover'
            return {'stop_confirmed': confirmed[0]}
        server.workers['motherboard'] = SimpleNamespace(alive=True, call=call)
        session = Session(1, 2, (), 'operator')
        first = await server.dispatch(session, 'control.acquire', {'force': True})
        assert server.owner == 1 and not first['motion_ready']
        assert first['stop_error']['code'] == 'stop_unconfirmed'
        assert server.game.info['running'] and server.game.motion_inflight
        confirmed[0] = True
        second = await server.dispatch(session, 'control.acquire', {'force': True})
        assert second['stop_confirmed'] and second['motion_ready']
        assert not server.game.info['running'] and not server.game.motion_inflight
        assert server.mode == 'MANUAL'
        assert calls.count('control.takeover') == 2
    asyncio.run(run())


def test_lost_acceptance_reply_after_pause_confirms_stop(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        original = s.workers['motherboard'].call
        entered, reply = asyncio.Event(), asyncio.Event()
        async def call(op, args=None, **kwargs):
            if op == 'game.jump':
                entered.set()
                await reply.wait()
                raise Fault('worker_timeout', 'Acceptance unknown')
            return await original(op, args, **kwargs)
        s.workers['motherboard'].call = call
        task = asyncio.create_task(game._move('game.jump', {'direction': 'forward'}))
        await entered.wait()
        await game.pause()
        reply.set()
        with pytest.raises(Fault): await task
        assert not game.motion_inflight
        assert ('motion.stop_hard', {}) in s.workers['motherboard'].calls
    asyncio.run(run())


def test_physical_pickup_overrides_gui_lease_without_releasing_it(tmp_path):
    from roki_ng.supervisor import Supervisor, Session
    async def run():
        server = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        server.mode, server.owner, server.lease_epoch = 'GAME', 42, 7
        server.game.info.update(running=True, job_id='gui-game')
        result = await server._head_command('game.pickup', {})
        assert result['pickup'] and server.owner == 42
        with pytest.raises(Fault) as error:
            await server.dispatch(Session(41, 2, (), 'stranger'), 'game.pickup', {'lease_epoch': 7})
        assert error.value.code == 'not_owner'
    asyncio.run(run())


def test_head_reentry_after_gui_pickup_takes_over_then_releases_menu_control(tmp_path):
    from roki_ng.supervisor import Supervisor
    async def run():
        server = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        server.workers = {r: Worker(r) for r in ('motherboard', 'camera', 'detection')}
        server.mode, server.owner, server.lease_epoch = 'GAME', 42, 7
        server.game.pickup_requested = True
        server.game.info.update(running=True, pickup=True, pickup_ready=True, job_id='gui-game')
        result = await server._head_command('game.start', dict(strategy='forward', entry='left', delay_seconds=10))
        assert result['running'] and result['role'] == 'forward' and result['entry'] == 'left'
        assert result['job_id'] != 'gui-game' and server.owner is None
        assert ('control.takeover', {}) in server.workers['motherboard'].calls
        await server.game.stop()
    asyncio.run(run())


def test_pan_is_applied_only_to_frames_after_completed_head_job(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        s.workers['motherboard'].pose = 'base_stand'
        obs = observation(y=0.)
        async def call(*args, **kwargs): return obs
        s.workers['detection'].call = call
        await game._move('game.head', {'pan': -1333, 'tilt': -1500})
        assert not (await game._ball())['result']['valid']
        obs = observation(y=0., seq=2)
        transformed = (await game._ball())['result']
        assert transformed['valid'] and transformed['y_m'] > .7
        assert s.workers['motherboard'].pose == 'base_stand'
        game.head_pan = None
        assert not (await game._ball())['result']['valid']
    asyncio.run(run())


def test_keeper_without_ball_returns_to_goal_in_short_sections(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        detection = s.workers['detection']
        original = detection.call
        async def no_ball(op, args=None, **kwargs):
            result = await original(op, args, **kwargs)
            if op == 'ball.status':
                return result | {'result': result['result'] | {'valid': False, 'reason': 'no_ball'}}
            return result
        detection.call = no_ball
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await until(lambda: game.state()['reason'] == 'no_ball')
        game.odometry.x += .3
        game.odometry.y += 2.
        await asyncio.sleep(.1)
        await game.stop()
        calls = s.workers['motherboard'].calls
        moves = [args for op, args in calls if op == 'game.walk']
        assert moves and all(args['distance_m'] <= .1 for args in moves)
        assert math.sin(moves[0]['course']) < 0  # Return from outside the right post.
        assert all(args.get('pan', 0) == 0 for op, args in calls if op == 'game.head')
    asyncio.run(run())


@pytest.mark.parametrize('ball_xy,target,expected', [((.8, 0.), 0., 'approach'),
                                                   ((.25, 0.), math.pi, 'around'),
                                                   ((.1, -.052), 0., 'kick'),
                                                   # Original fine adjustment does not back away
                                                   # from a ball already inside its 100 mm target.
                                                   ((.05, -.052), 0., 'kick'),
                                                   ((.19, .052), 0., 'five_jumps')])
def test_original_forward_selects_approach_orbit_and_calibrated_kick(tmp_path, monkeypatch, ball_xy, target, expected):
    from roki_ng import legacy_strategy
    monkeypatch.setattr(legacy_strategy, 'kick_target', lambda *args: (target, 60))
    async def run():
        game, s, _ = setup(tmp_path)
        game.info.update(running=True, role='forward')
        detector = s.workers['detection']
        original = detector.call
        async def observed(op, args=None, **kwargs):
            result = await original(op, args, **kwargs)
            if op == 'ball.status':
                result['result'].update(x_m=ball_xy[0], y_m=ball_xy[1])
            return result
        detector.call = observed
        assert await game._prepare(s.params.values, 'center')
        await game._original_forward(s.params.values, 'center')
        calls = s.workers['motherboard'].calls
        walks = [args['kind'] for op, args in calls if op == 'game.walk']
        kicks = [args for op, args in calls if op == 'motion.kick']
        if expected in ('approach', 'around'):
            assert walks == [expected] and not kicks
        elif expected == 'kick':
            assert not walks  # Already kickable: do not push the ball with a verify walk.
            assert kicks == [{'leg': 'right', 'power': 60, 'offset': -10}]
        else:
            assert walks == ['verify'] and not kicks
            jumps = [args for op, args in calls if op == 'game.jump']
            assert len(jumps) == 5 and all(args['direction'] == 'forward' for args in jumps)
        assert not any(op == 'motion.pose' and args['name'] == 'crouch' for op, args in calls)
    asyncio.run(run())


def test_original_forward_replans_after_five_fine_jumps(tmp_path, monkeypatch):
    """A non-converging close observation cannot restart fine jumps forever."""
    from roki_ng import legacy_strategy
    monkeypatch.setattr(legacy_strategy, 'kick_target', lambda *args: (0., 60))

    async def run():
        game, s, _ = setup(tmp_path)
        game.info.update(running=True, role='forward')
        detector = s.workers['detection']
        original = detector.call

        async def observed(op, args=None, **kwargs):
            result = await original(op, args, **kwargs)
            if op == 'ball.status':
                # Far enough in front to require a jump, while the target
                # direction causes the first pass to use fine adjustment.
                result['result'].update(x_m=.19, y_m=.052)
            return result

        detector.call = observed
        async def turn(_, **kwargs):
            return True
        game._original_turn = turn
        assert await game._prepare(s.params.values, 'center')
        await game._original_forward(s.params.values, 'center')
        assert game.forward_replan
        await game._original_forward(s.params.values, 'center')
        calls = s.workers['motherboard'].calls
        jumps = [args for op, args in calls if op == 'game.jump']
        walks = [args['kind'] for op, args in calls if op == 'game.walk']
        assert len(jumps) == 5
        assert walks == ['verify', 'around']
        assert not game.forward_replan

    asyncio.run(run())


def test_close_kickable_ball_bypasses_orbit_when_robot_already_faces_goal(tmp_path, monkeypatch):
    """Regression for the field log: x=110, y=-50 was sent to orbit."""
    from roki_ng import legacy_strategy
    monkeypatch.setattr(legacy_strategy, 'kick_target', lambda *args: (0., 100))

    async def run():
        game, s, _ = setup(tmp_path)
        game.info.update(running=True, role='forward')
        assert await game._prepare(s.params.values, 'center')
        # A robot already facing the goal must kick a ready ball without orbiting.
        game.odometry.yaw = 0.
        async def observation_at_ball(*_args, **_kwargs):
            result = observation(-.05)
            result['result']['x_m'] = .11
            return result
        async def turn(_, **kwargs):
            return True
        async def walk(*_args, **_kwargs):
            return True
        game._original_observe = observation_at_ball
        game._original_turn = turn
        game._original_walk = walk
        await game._original_forward(s.params.values, 'center')
        kicks = [args for op, args in s.workers['motherboard'].calls if op == 'motion.kick']
        assert kicks == [{'leg': 'right', 'power': 100, 'offset': -12}]

    asyncio.run(run())


def test_five_fine_jumps_use_original_offset_kick_for_safe_residual(tmp_path, monkeypatch):
    """Field regression: x=86, y=-74 must kick with +12 mm offset."""
    from roki_ng import legacy_strategy
    monkeypatch.setattr(legacy_strategy, 'kick_target', lambda *args: (0., 100))

    async def run():
        game, s, _ = setup(tmp_path)
        game.info.update(running=True, role='forward')
        assert await game._prepare(s.params.values, 'center')
        looks = 0
        async def observation_at_ball(*_args, **_kwargs):
            nonlocal looks
            looks += 1
            # Initial look, post-turn look, then six fine-adjustment looks.
            result = observation(-.074 if looks >= 8 else -.052)
            result['result']['x_m'] = .086 if looks >= 8 else .19
            return result
        async def turn(_, **kwargs):
            return True
        async def walk(*_args, **_kwargs):
            return True
        game._original_observe = observation_at_ball
        game._original_turn = turn
        game._original_walk = walk
        await game._original_forward(s.params.values, 'center')
        calls = s.workers['motherboard'].calls
        jumps = [args for op, args in calls if op == 'game.jump']
        kicks = [args for op, args in calls if op == 'motion.kick']
        assert len(jumps) == 5
        assert kicks == [{'leg': 'right', 'power': 100, 'offset': 12}]
        assert not game.forward_replan

    asyncio.run(run())


def test_original_forward_search_turns_after_seven_distinct_failed_looks(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        assert await game._prepare(s.params.values, 'center')
        async def empty(*args, **kw):
            return observation() | {'result': {'valid': False, 'reason': 'no_ball'}}
        game._original_observe = empty
        turns = []
        async def turn(target, **kwargs): turns.append(target); return True
        game._original_turn = turn
        for _ in range(6):
            await game._original_forward(s.params.values, 'center')
        assert not turns
        await game._original_forward(s.params.values, 'center')
        assert turns == [pytest.approx(2*math.pi/3)]
    asyncio.run(run())


def test_original_observation_settling_is_interruptible_by_pickup(tmp_path):
    from roki_ng.legacy_strategy import OriginalStrategy
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        task = asyncio.create_task(OriginalStrategy._settle(game, 1.))
        await asyncio.sleep(.01)
        await game.pickup()
        assert await asyncio.wait_for(task, .2) is False
        assert not any(op.startswith('motion.') for op, _ in s.workers['motherboard'].calls)
    asyncio.run(run())


@pytest.mark.parametrize('role', ['keeper', 'forward'])
def test_pause_resume_during_walk_preparation_discards_entire_decision(tmp_path, monkeypatch, role):
    monkeypatch.setattr('roki_ng.legacy_strategy.kick_target', lambda *args: (0., 60))
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        entry = 'goalkeeper' if role == 'keeper' else 'center'
        assert await game._prepare(s.params.values, entry)
        observations = []
        async def observed(*args, **kwargs):
            observations.append(game.revision)
            return observation(.25 if role == 'keeper' else 0.)
        game._original_observe = observed
        worker = s.workers['motherboard']
        original = worker.call
        interrupted = False
        async def interleaved(op, args=None, **kwargs):
            nonlocal interrupted
            if op == 'state' and not interrupted:
                interrupted = True
                await game.pause()
                await game.resume()
            return await original(op, args, **kwargs)
        worker.call = interleaved
        worker.calls.clear()
        decision = game._original_keeper if role == 'keeper' else game._original_forward
        await decision(s.params.values, entry)
        assert interrupted and observations and all(rev < game.revision for rev in observations)
        assert not any(op in ('game.walk', 'game.jump', 'motion.kick') for op, _ in worker.calls)
        # Resume is not a permanent latch: a new decision may use a new look.
        await decision(s.params.values, entry)
        assert observations[-1] == game.revision
        assert any(op == 'game.walk' for op, _ in worker.calls)
    asyncio.run(run())


def test_pause_resume_before_kick_requires_new_decision_and_ball(tmp_path, monkeypatch):
    monkeypatch.setattr('roki_ng.legacy_strategy.kick_target', lambda *args: (0., 60))
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        assert await game._prepare(s.params.values, 'center')
        looks = 0
        async def observed(*args, **kwargs):
            nonlocal looks
            looks += 1
            result = observation(-.052)
            result['result']['x_m'] = .1
            return result
        game._original_observe = observed
        original = game._imu
        interrupted = False
        async def interleaved(*args, **kwargs):
            nonlocal interrupted
            if looks >= 2 and not interrupted:
                interrupted = True
                await game.pause()
                await game.resume()
            return await original(*args, **kwargs)
        game._imu = interleaved
        await game._original_forward(s.params.values, 'center')
        calls = s.workers['motherboard'].calls
        assert interrupted and not any(op == 'motion.kick' for op, _ in calls)
        await game._original_forward(s.params.values, 'center')
        assert any(op == 'motion.kick' for op, _ in calls)
    asyncio.run(run())


@pytest.mark.parametrize('interruption', ['capture_stop', 'pause_resume'])
def test_observer_start_rechecks_capture_and_plan_after_capture_lock(tmp_path, interruption):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        camera = s.workers['camera']
        original = camera.call
        read = asyncio.Event()
        running = True
        async def status(op, *args, **kwargs):
            result = await original(op, *args, **kwargs)
            if op == 'camera.status':
                read.set()
                return result | {'running': running}
            return result
        camera.call = status
        async with s.capture_lock:
            task = asyncio.create_task(game._start_detection(s.params.values))
            await asyncio.wait_for(read.wait(), .5)
            assert not s.workers['detection'].calls
            if interruption == 'capture_stop':
                running = False
            else:
                await game.pause()
                await game.resume()
        if interruption == 'capture_stop':
            with pytest.raises(Fault, match='Camera stopped'):
                await asyncio.wait_for(task, .5)
        else:
            assert await asyncio.wait_for(task, .5) is False
        assert not any(op == 'ball.start' for op, _ in s.workers['detection'].calls)
    asyncio.run(run())


def test_game_observer_waits_for_confirmed_capture_alignment(tmp_path):
    async def run():
        game, s, _ = setup(tmp_path)
        game.info['running'] = True
        camera = s.workers['camera']
        original = camera.call
        sync = {'state': 'aligning', 'unicam_minus_stm': None}
        async def status(op, *args, **kwargs):
            result = await original(op, *args, **kwargs)
            return result | {'imu_sync': dict(sync)} if op == 'camera.status' else result
        camera.call = status
        for state, offset in [('aligning', None), ('matched', 7)]:
            sync.update(state=state, unicam_minus_stm=offset)
            with pytest.raises(Fault, match='alignment'):
                await game._start_detection(s.params.values)
        assert not s.workers['detection'].calls
        sync['state'] = 'synced'
        assert await game._start_detection(s.params.values)
        started = [args for op, args in s.workers['detection'].calls if op == 'ball.start']
        assert len(started) == 1 and started[0]['unicam_minus_stm'] == 7
    asyncio.run(run())


@pytest.mark.parametrize('barrier', [None, 'pause', 'pickup'])
def test_capture_loss_recovers_game_observer_but_respects_operator_barriers(tmp_path, monkeypatch, barrier):
    from collections import deque
    from roki_ng.ball_observation import BallObservation
    from roki_ng.detection import Detection
    from roki_ng.supervisor import Supervisor

    async def run():
        game, s, session = setup(tmp_path)
        camera_running = True
        def frames(observer, offset):
            sequence = 0
            while not observer.stop_event.wait(.005):
                if camera_running:
                    sequence += 1
                    now = time.monotonic()
                    with observer.lock:
                        observer.tracker.update([dict(x_m=.5, y_m=0., rect=[0, 0, 10, 10])],
                                                sequence, game._capture_now(), now, now)
        monkeypatch.setattr(BallObservation, '_run', frames)
        detector = Detection({'state_dir': str(tmp_path), 'parameters': s.params.values},
                             lambda *a: None, lambda *a: None)
        worker = s.workers['detection']
        calls = worker.calls
        async def detection(op, args=None, **kwargs):
            calls.append((op, args or {}))
            return detector.command(op, args or {})
        worker.call = detection
        camera = s.workers['camera']
        original = camera.call
        async def capture(op, args=None, **kwargs):
            nonlocal camera_running
            result = await original(op, args, **kwargs)
            if op == 'camera.stop':
                camera_running = False
            if op == 'camera.status':
                return result | {'running': camera_running}
            return result
        camera.call = capture
        restarts = []
        async def dispatch(session, op, args, **kwargs):
            nonlocal camera_running
            assert op == 'camera.start' and kwargs['_from_game']
            async with s.capture_lock:
                restarts.append(op)
                camera_running = True
                s.capture_session = object()
            return {}
        s.dispatch = dispatch
        s.capture_session, s.alignment_pending = object(), deque()
        s.workers['stream'] = Worker('stream')
        try:
            await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
            await until(lambda: (game.info['ball'] or {}).get('valid'))
            async with s.capture_lock:
                await Supervisor._stop_capture(s)
            stopped_at = game._capture_now()
            assert not detector.ball.state()['running'] and not camera_running
            if barrier:
                await getattr(game, barrier)()
                # Allow the strategy to observe the operator barrier.
                await asyncio.sleep(.2)
                assert not restarts and not detector.ball.state()['running']
                if barrier == 'pickup':
                    return
                await game.resume()
            await until(lambda: restarts and (game.info['ball'] or {}).get('valid')
                        and game.info['ball']['sensor_timestamp_ns'] > stopped_at)
            assert game.state()['running'] and detector.ball.state()['running']
            assert len(restarts) == 1
        finally:
            await game.stop()
            detector.close()
    asyncio.run(run())
