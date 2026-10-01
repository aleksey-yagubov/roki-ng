import asyncio
import time
from types import SimpleNamespace

import pytest

from roki_ng.game import Goalkeeper, decision
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


def observation(y=.2, seq=1):
    return dict(running=True, error=None, age_ms=10,
                result=dict(valid=True, frame_sequence=seq, sensor_timestamp_ns=time.monotonic_ns(), x_m=1., y_m=y))


@pytest.mark.parametrize('y,expected', [(.2, 'left'), (-.2, 'right'), (.05, 'hold')])
def test_robot_relative_decision(y, expected):
    assert decision(observation(y), .1, 3)[0] == expected


@pytest.mark.parametrize('change', [{'age_ms': 501}, {'running': False}, {'error': 'lost'}, {'age_ms': None}])
def test_invalid_observations_never_move(change):
    assert decision(observation() | change, .1, 3)[0] == 'hold'


class Worker:
    def __init__(self, role):
        self.role = role
        self.calls = []
        self.alive = True
        self.seq = 0
        self.bad_ball = False
        self.bad_imu = False

    async def call(self, op, args=None, **kwargs):
        self.calls.append((op, args or {}))
        if op == 'state':
            return dict(body_connected=True, active_job=None, error=None, head={'pan': 0})
        if op == 'body.telemetry.read':
            return {'body.imu': dict(valid=not self.bad_imu, source_mono_ns=time.monotonic_ns(),
                                    quaternion_xyzw=[0, 0, 0, 1], pitch_deg=0, roll_deg=0)}
        if op == 'camera.status':
            return dict(running=True, imu_sync={'state': 'synced', 'unicam_minus_stm': 5})
        if op == 'ball.status':
            self.seq += 1
            return observation(seq=self.seq) | ({'error': 'no_camera'} if self.bad_ball else {})
        if op in ('motion.pose', 'game.step'):
            return {'job_id': 'body-job'}
        if op == 'job.status':
            return {'status': 'completed'}
        return {}


def setup(tmp_path):
    workers = {role: Worker(role) for role in ('motherboard', 'camera', 'detection')}
    session = SimpleNamespace(id=1, closed=False)
    async def dispatch(*args, **kwargs):
        return {}
    s = SimpleNamespace(mode='MANUAL', owner=1, lease_epoch=1, workers=workers,
                        motion_ready=True, head_menu=None, local_session=None,
                        params=Parameters(tmp_path), dispatch=dispatch,
                        require_motion_ready=lambda: None,
                        require_control=lambda *args: None, log=lambda *args: None)
    return Goalkeeper(s), s, session


async def until(predicate):
    for _ in range(200):
        if predicate(): return
        await asyncio.sleep(.01)
    assert predicate()


def test_observe_start_stop_has_no_head_or_body_motion(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await until(lambda: game.state()['decision'] == 'left')
        assert s.mode == 'GAME'
        await game.stop()
        assert not game.state()['running'] and s.mode == 'MANUAL'
        assert not any(op.startswith(('motion.', 'game.')) for op, _ in s.workers['motherboard'].calls)
        assert any(op == 'ball.stop' for op, _ in s.workers['detection'].calls)
    asyncio.run(run())


def test_physical_requires_verified_geometry_before_side_effects(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        with pytest.raises(Fault, match='geometry'):
            await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper', 'observe_only': False})
        assert s.workers['motherboard'].calls == []
        assert s.mode == 'MANUAL'
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


def test_failed_ball_stops_and_does_not_resume(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        s.workers['detection'].bad_ball = True
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper'})
        await until(lambda: not game.state()['running'])
        assert game.state()['state'] == 'failed'
        s.workers['detection'].bad_ball = False
        await asyncio.sleep(.02)
        assert not game.state()['running']
    asyncio.run(run())


def test_physical_path_budget_and_urgent_stop(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        s.params.values.update({'game.geometry_verified': True, 'game.max_path_m': .1})
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper', 'observe_only': False})
        await until(lambda: not game.state()['running'])
        calls = s.workers['motherboard'].calls
        assert sum(op == 'game.step' for op, _ in calls) == 1
        assert calls[-1][0] == 'motion.stop_hard'
        assert 'budget' in game.state()['reason']
    asyncio.run(run())


def test_body_step_is_finite_and_ignores_manual_speed(tmp_path):
    from roki_ng.body import Body
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values}, lambda *a: None, lambda *a: None)
    body.command('control.acquire', {})
    body.pose = 'crouch'
    body.read_body_quaternion = lambda: [0, 0, 0, 1]
    body.command('game.step', {'direction': 'left', 'heading': 0, 'cycles': 2})
    steps = list(body.plan)
    assert len([s for s in steps if s[0] == 'sleep']) == 2
    body.command('motion.stop_hard', {})
    assert body.plan is None


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


def test_stop_does_not_interrupt_fault_cleanup(tmp_path):
    async def run():
        game, s, session = setup(tmp_path)
        s.params.values['game.geometry_verified'] = True
        s.workers['detection'].bad_ball = True
        original = s.workers['motherboard'].call
        entered, release = asyncio.Event(), asyncio.Event()
        async def wait_stop(op, *args, **kwargs):
            if op == 'motion.stop_hard':
                entered.set()
                await release.wait()
            return await original(op, *args, **kwargs)
        s.workers['motherboard'].call = wait_stop
        await game.start(session, {'strategy': 'FIRA_penalty_Goalkeeper', 'observe_only': False})
        await asyncio.wait_for(entered.wait(), 2)
        assert not s.motion_ready
        stop = asyncio.create_task(game.stop())
        await asyncio.sleep(.02)
        assert not stop.done()  # Must await the hardware stop acknowledgement.
        release.set()
        await stop
        assert s.motion_ready and not game.state()['running']
        assert any(op == 'ball.stop' for op, _ in s.workers['detection'].calls)
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
        for op, args in [('motion.head', {'tilt': -1000}), ('params.set', {}), ('camera.stop', {})]:
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
            return {'job_id': 'game1', 'running': op != 'game.stop', 'observe_only': True, 'state': 'stopped'}
        async def release(): calls.append('release')
        menu = HeadMenu(command, release, voices.append, lambda *a: None)
        await menu._start(Item('Observe', op='game.start', args={}))
        assert menu.game_active and 'Goalkeeper observing' in voices
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
        task = asyncio.create_task(s._set_parameters({'game.deadband_m': .2}))
        await asyncio.sleep(0)
        s.game.info['running'] = True
        s.parameter_lock.release()
        with pytest.raises(Fault, match='Stop goalkeeper'): await task
        assert s.params.values['game.deadband_m'] == .1
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
