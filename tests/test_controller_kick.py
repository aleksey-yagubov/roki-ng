from types import SimpleNamespace

import pytest

from roki_ng.body import Body, RokiHardware
from roki_ng.controller_kick import KickWitness
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


def test_kick_parameter_persistence_and_validation(tmp_path):
    params = Parameters(tmp_path)
    assert params.values['motion.kick_type'] == 'software'
    assert params.describe('motion.kick_type')['choices'] == ['software', 'controller']
    params.set('motion.kick_type', 'controller')
    assert Parameters(tmp_path).values['motion.kick_type'] == 'controller'
    for value in ('slot31', 31, True, None):
        with pytest.raises(Fault):
            params.set('motion.kick_type', value)
    assert Parameters(tmp_path).values['motion.kick_type'] == 'controller'


@pytest.fixture
def body(tmp_path, body_clock):
    logs = []
    body = Body({'simulate': True, 'parameters': Parameters(tmp_path).values},
                lambda *args: None, lambda *args: logs.append(args))
    body.command('control.acquire', {})
    body.command('params.apply', {'motion.kick_type': 'controller'})
    body.pose = 'base_stand'
    body.kick_logs = logs
    return body


def until(body, predicate, limit=1500):
    for _ in range(limit):
        body.tick()
        if predicate():
            return
    pytest.fail('Worker did not reach expected condition')


@pytest.mark.parametrize('leg,power,offset', [('right', 20, -60), ('left', 100, 80)])
def test_controller_kick_writes_parameters_once_and_waits_for_mixing(body, leg, power, offset):
    writes, starts = [], []
    body.hardware.controller_write = lambda *args: writes.append(args)
    original_start = body.hardware.controller_start
    body.hardware.controller_start = lambda: (starts.append(31), original_start())
    job = body.command('motion.kick', dict(leg=leg, power=power, offset=offset))['job_id']
    until(body, lambda: body.hardware.controller_pending)
    assert body.active == job and body.pose == 'unknown'
    assert writes == [(1029, int(leg == 'right')), (1028, power), (1030, offset)]
    assert starts == [31]
    until(body, lambda: body.active is None)
    assert body.jobs[job]['status'] == 'completed'
    assert body.jobs[job]['kick_type'] == 'controller'
    assert body.pose == 'stand' and not body.hardware.controller_pending
    assert not body.jobs[job]['execution_uncertain']
    assert not body.hardware.sent  # No Python kick/pose packets overlap the slot.


def test_selection_is_frozen_and_controller_power_is_not_silently_clamped(body):
    with pytest.raises(Fault):
        body.command('motion.kick', {'power': 19})
    job = body.command('motion.kick', {})['job_id']
    with pytest.raises(Fault, match='Stop motion'):
        body.command('params.apply', {'motion.kick_type': 'software'})
    until(body, lambda: body.active is None)
    assert body.jobs[job]['kick_type'] == 'controller'
    body.command('params.apply', {'motion.kick_type': 'software'})
    job = body.command('motion.kick', {'power': 1})['job_id']
    until(body, lambda: body.active is None)
    assert body.jobs[job]['kick_type'] == 'software'
    assert body.jobs[job]['status'] == 'completed'


@pytest.mark.parametrize('after_start', [False, True])
def test_hard_stop_cancels_and_does_not_replay(body, after_start):
    job = body.command('motion.kick', {})['job_id']
    if after_start:
        until(body, lambda: body.hardware.controller_pending)
    result = body.command('motion.stop_hard', {})
    assert result['scope'] == ('host_body_queue_and_controller' if after_start else 'host_body_queue')
    assert not body.hardware.controller_pending
    assert body.jobs[job]['status'] == 'cancelled'
    assert body.jobs[job]['execution_uncertain'] == after_start
    for _ in range(100):
        body.tick()
    assert body.active is None and body.pose == 'unknown'


@pytest.mark.parametrize('after_start', [False, True])
def test_graceful_stop_prevents_launch_or_finishes_atomic_kick(body, after_start):
    job = body.command('motion.kick', {})['job_id']
    if after_start:
        until(body, lambda: body.hardware.controller_pending)
    body.command('motion.stop_graceful', {})
    until(body, lambda: body.active is None)
    assert body.jobs[job]['status'] == 'cancelled'
    assert not body.hardware.controller_pending
    assert not body.jobs[job]['execution_uncertain']
    assert body.pose == ('stand' if after_start else 'base_stand')


def test_unknown_launch_result_fails_job_without_replay(body):
    calls = []
    def uncertain_launch():
        body.hardware.controller_pending = True
        calls.append(31)
        raise Fault('hardware_error', 'lost ACK after execution')
    body.hardware.controller_start = uncertain_launch
    job = body.command('motion.kick', {})['job_id']
    until(body, lambda: body.active is None)
    assert calls == [31]
    assert body.jobs[job]['status'] == 'failed'
    assert body.jobs[job]['execution_uncertain']
    assert body.body_connected and body.state()['important_errors_5s'] == 1
    assert not body.hardware.controller_pending


def test_polling_errors_are_logged_without_counting_as_motion_errors(body):
    original_read = body.hardware.controller_read
    reads = []
    def read(address):
        reads.append(address)
        if len(reads) <= 3:
            raise Fault('hardware_error', 'telemetry timeout')
        return original_read(address)
    body.hardware.controller_read = read
    job = body.command('motion.kick', {})['job_id']
    until(body, lambda: body.active is None)
    assert body.jobs[job]['status'] == 'completed'
    assert body.state()['important_errors_5s'] == 0
    assert sum(level == 'WARNING' and 'telemetry timeout' in msg for level, msg in body.kick_logs) == 3


@pytest.mark.parametrize('memory', [
    {18: 0, 16: 0, 2: 100, 3: 100},  # ACK but kick never started; already mixing.
    {18: 2, 16: KickWitness.ROTATION_BITS, 2: 0, 3: 0},  # Stuck kick.
])
def test_unconfirmed_execution_times_out_and_stops_controller(body, memory):
    body.hardware.controller_read = memory.__getitem__
    job = body.command('motion.kick', {})['job_id']
    until(body, lambda: body.active is None)
    assert body.jobs[job]['status'] == 'failed'
    assert body.jobs[job]['execution_uncertain']
    assert body.pose == 'unknown' and not body.hardware.controller_pending
    assert body.state()['important_errors_5s'] == 1


def test_stale_imu_prevents_controller_launch(body):
    body.hardware.body_quaternion = lambda: (_ for _ in ()).throw(Fault('imu_invalid', 'bad sample'))
    job = body.command('motion.kick', {})['job_id']
    until(body, lambda: body.active is None)
    assert body.jobs[job]['status'] == 'failed'
    assert not hasattr(body.hardware, 'controller_reads')
    assert body.state()['important_errors_5s'] == 0


@pytest.mark.parametrize('failure', ['fall', 'imu_lost', 'ik'])
def test_controller_motion_failure_stops_script_and_discards_pose(body, failure):
    job = body.command('motion.kick', {})['job_id']
    until(body, lambda: body.hardware.controller_pending)
    if failure == 'fall':
        body.hardware.body_quaternion = lambda: (0, 0, 0, 1)
    elif failure == 'imu_lost':
        body.hardware.body_quaternion = lambda: (_ for _ in ()).throw(Fault('imu_invalid', 'IMU unavailable'))
    else:
        body.hardware.controller_read = {18: 2, 16: KickWitness.ROTATION_BITS, 2: 1, 3: 0}.__getitem__
    until(body, lambda: body.active is None)
    assert body.jobs[job]['status'] == 'failed'
    assert body.jobs[job]['execution_uncertain']
    assert not body.hardware.controller_pending and body.pose == 'unknown'
    assert body.state()['important_errors_5s'] == 0


def test_manual_crouch_prepares_standing_before_starting_original_slot(body):
    body.pose = 'crouch'
    starts = []
    original = body.hardware.controller_start
    def start():
        starts.append(body.hardware.sent)
        original()
    body.hardware.controller_start = start
    job = body.command('motion.kick', {})['job_id']
    until(body, lambda: body.active is None)
    assert starts and starts[0] > 0
    assert body.jobs[job]['status'] == 'completed'
    assert not body.sent_targets and not body.servo_targets


def test_kick_calibration_uses_selected_backend(body):
    job = body.command('test.start', {'name': 'kick_test', 'mode': 'regular'})['job_id']
    until(body, lambda: body.active is None)
    assert body.jobs[job]['status'] == 'completed'
    assert body.jobs[job]['kick_type'] == 'controller'
    assert body.hardware.controller_reads >= 10


def hardware_stub():
    hw = object.__new__(RokiHardware)
    state = dict(body_failures=0, body_queue_size=0, body_busy=False, last_body_error=0)
    calls = []
    hw.body_failures = 0
    hw.config = {}
    hw.mb = SimpleNamespace(GetStatus=lambda: dict(state), GetLastStatus=lambda: 0,
                            ResetBodyQueue=lambda: calls.append('reset') or True)
    hw.rcb = SimpleNamespace(motionPlay=lambda slot: calls.append(slot) or True, GetError=lambda: 'failure')
    hw.zubr = SimpleNamespace(memISet=lambda *args: calls.append(args) or True,
                              memIGet=lambda addr: (True, 2), GetError=lambda: 'failure')
    return hw, state, calls


def test_real_adapter_writes_vm_globals_and_stops_zubr_not_just_stm():
    hw, state, calls = hardware_stub()
    hw.controller_write(1030, -60)
    hw.controller_start()
    assert hw.controller_pending and calls == [(1030, -60), 31]
    hw.reset()
    assert calls == [(1030, -60), 31, 'reset', 66]
    assert not hw.controller_pending and not hw.mixing_started


def test_busy_launch_is_not_mistaken_for_an_uncertain_executed_kick():
    hw, state, calls = hardware_stub()
    hw.rcb.motionPlay = lambda slot: False
    hw.mb.GetLastStatus = lambda: 5
    with pytest.raises(Fault) as error:
        hw.controller_start()
    assert error.value.code == 'body_busy'
    assert not hw.controller_pending


def test_unacknowledged_parameter_write_cannot_launch_kick(body):
    writes, starts = [], []
    def write(address, value):
        writes.append(address)
        if address == 1028:
            raise Fault('hardware_error', 'parameter ACK lost')
    body.hardware.controller_write = write
    body.hardware.controller_start = lambda: starts.append(31)
    job = body.command('motion.kick', {})['job_id']
    until(body, lambda: body.active is None)
    assert writes == [1029, 1028] and not starts
    assert body.jobs[job]['status'] == 'failed'
    assert not body.jobs[job]['execution_uncertain']


def test_witness_rejects_torn_transition_snapshot():
    witness = KickWitness()
    samples = iter([2, KickWitness.ROTATION_BITS, 0, 0, 2,
                    2, 0, 100, 100, 0,
                    0, 0, 100, 100, 0])
    hw = SimpleNamespace(controller_read=lambda address: next(samples))
    assert not any(witness.poll(hw) for _ in range(5))
    assert witness.started
    assert not any(witness.poll(hw) for _ in range(5))
    assert not any(witness.poll(hw) for _ in range(4))
    assert witness.poll(hw)


def test_lost_launch_ack_requires_stop_after_forward_resolution():
    hw, state, calls = hardware_stub()
    hw.rcb.motionPlay = lambda slot: calls.append(slot) or slot == 66
    hw.mb.GetLastStatus = lambda: 255
    with pytest.raises(Fault):
        hw.controller_start()
    assert hw.forward_pending and hw.controller_pending
    state['body_busy'] = True
    with pytest.raises(Fault, match='deferred'):
        hw.reset()
    assert hw.controller_pending
    state['body_busy'] = False
    hw.reset()
    assert calls.count(31) == 1 and calls.count(66) == 1
    assert not hw.controller_pending and not hw.forward_pending


def test_failed_stop_keeps_latch_until_acknowledged():
    hw, state, calls = hardware_stub()
    hw.controller_start()
    hw.rcb.motionPlay = lambda slot: False
    with pytest.raises(Fault):
        hw.reset()
    assert hw.controller_pending
    with pytest.raises(Fault, match='completion/stop not confirmed'):
        hw.prepare_motion()
    hw.rcb.motionPlay = lambda slot: calls.append(slot) or True
    hw.connect_body = lambda: hw.reset()
    hw.connect_body()
    assert not hw.controller_pending and calls[-1] == 66


def test_stop_waits_for_busy_transaction_without_counting_busy_as_failure(body):
    resets = []
    original_reset = body.hardware.reset
    def reset():
        resets.append(True)
        if len(resets) <= 3:
            raise Fault('body_busy', 'transaction pending')
        original_reset()
    def start():
        body.hardware.controller_pending = True
        raise Fault('hardware_error', 'lost kick ACK')
    body.hardware.controller_start = start
    body.hardware.reset = reset
    job = body.command('motion.kick', {})['job_id']
    until(body, lambda: body.active is None)
    assert body.jobs[job]['execution_uncertain']
    assert body.queue_reset_pending and body.body_connected
    with pytest.raises(Fault, match='reset not confirmed'):
        body.command('motion.kick', {})
    until(body, lambda: not body.queue_reset_pending)
    assert body.body_connected and not body.hardware.controller_pending
    assert body.state()['important_errors_5s'] == 1
    assert len(resets) == 4


def test_hard_stop_busy_remains_blocked_until_ack_without_entering_fault(body):
    body.command('motion.kick', {})
    until(body, lambda: body.hardware.controller_pending)
    original = body.hardware.reset
    body.hardware.reset = lambda: (_ for _ in ()).throw(Fault('body_busy', 'transaction pending'))
    with pytest.raises(Fault):
        body.command('motion.stop_hard', {})
    assert body.queue_reset_pending and not body.error
    assert body.state()['important_errors_5s'] == 0
    body.hardware.reset = original
    until(body, lambda: not body.queue_reset_pending)
    assert not body.hardware.controller_pending and body.body_connected


def test_fall_followed_by_busy_stop_does_not_increase_motion_error_counter(body):
    body.command('motion.kick', {})
    until(body, lambda: body.hardware.controller_pending)
    body.hardware.body_quaternion = lambda: (0, 0, 0, 1)
    original = body.hardware.reset
    body.hardware.reset = lambda: (_ for _ in ()).throw(Fault('body_busy', 'transaction pending'))
    until(body, lambda: body.active is None)
    assert body.queue_reset_pending and body.state()['important_errors_5s'] == 0
    body.hardware.reset = original
    until(body, lambda: not body.queue_reset_pending)
    assert not body.hardware.controller_pending
