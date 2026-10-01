import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest
from roki_ng.supervisor import Supervisor
from roki_ng.parameters import Parameters,SCHEMA
from roki_ng.wire import Fault,pack


def server(tmp_path,state=None):
    s=Supervisor({'state_dir':str(tmp_path)})
    s.require_control=lambda *args:None
    s.workers={role:SimpleNamespace(call=AsyncMock(return_value=state or {}))
               for role in ('camera','detection','localisation')}
    return s


def test_lab_batch_is_atomic_and_conflict_checked(tmp_path):
    async def run():
        s=server(tmp_path)
        keys=['vision.white_marking.l_min','vision.white_marking.l_max']
        old={k:s.params.values[k] for k in keys}
        values=dict(zip(keys,[10,40])) # Sequential min/max write can otherwise fail.
        await s.dispatch(None,'params.set',{'values':values,'expected_values':old})
        assert all(Parameters(tmp_path).values[k]==v for k,v in values.items())
        for role in ('detection','localisation'):
            s.workers[role].call.assert_awaited_once_with('params.apply',values)
        with pytest.raises(Fault,match='changed'):
            await s.dispatch(None,'params.set',{'values':values,'expected_values':old})
        before=s.params.path.read_bytes()
        with pytest.raises(Fault):
            await s.dispatch(None,'params.set',{'values':dict(zip(keys,[90,40]))})
        assert s.params.path.read_bytes()==before
    asyncio.run(run())


@pytest.mark.parametrize('age,running,good',[(10,True,True),(1001,True,False),(10,False,False),(None,True,False)])
def test_freeze_requires_fresh_metadata_without_implicit_save(tmp_path,age,running,good):
    async def run():
        s=server(tmp_path,{'running':running,'measured_age_ms':age,
            'measured_controls':{'sequence':77,'exposure_us':7000,'gain':2.,'colour_gains':[1.2,2.6]}})
        if good:
            result=await s.dispatch(None,'camera.controls.freeze',{'group':'all'})
            assert result['source_sequence']==77
            assert result['values']['camera.exposure_us']==7000
            assert result['values']['camera.awb_enabled'] is False
            assert result['values']['camera.white_balance.blue_gain']==2.6
            assert result['saved'] is False
            assert Parameters(tmp_path).values['camera.exposure_us']==8000
        else:
            with pytest.raises(Fault,match='Fresh'):
                await s.dispatch(None,'camera.controls.freeze',{'group':'all'})
            assert s.workers['camera'].call.await_count==1
    asyncio.run(run())


def test_missing_auto_metadata_never_uses_defaults(tmp_path):
    async def run():
        s=server(tmp_path,{'running':True,'measured_age_ms':0,'measured_controls':{'sequence':3,'colour_gains':None}})
        with pytest.raises(Fault):await s.dispatch(None,'camera.controls.freeze',{'group':'white_balance'})
        assert s.workers['camera'].call.await_count==1
    asyncio.run(run())


def test_largest_colour_group_with_expected_values_fits_udp():
    values={k:v[1] for k,v in SCHEMA.items() if k.startswith('vision.yellow_posts.')}
    pack({'v':1,'kind':'request','id':2**63,'session':2**63,'token':2**63,
          'op':'params.set','body':{'values':values,'expected_values':values,'lease_epoch':2**63}})


def test_camera_rollback_restores_capture_override_not_persistent_default(tmp_path):
    async def run():
        s=server(tmp_path)
        s.workers['camera'].call=AsyncMock(return_value={'previous':{'camera.exposure_us':5000}})
        def fail(_):raise OSError('disk full')
        s.params.set_many=fail
        with pytest.raises(OSError):await s._set_parameters({'camera.exposure_us':7000})
        assert s.workers['camera'].call.call_args_list[-1].args==('params.apply',{'camera.exposure_us':5000})
        assert Parameters(tmp_path).values['camera.exposure_us']==8000
    asyncio.run(run())


def test_single_format_unchanged_and_batch_get_round_trip(tmp_path):
    async def run():
        s = server(tmp_path)
        key = 'camera.exposure_us'
        before = await s.dispatch(None, 'params.get', {'key': key})
        assert before == {'key': key, 'value': 8000}
        result = await s.dispatch(None, 'params.set', {'key': key, 'value': 7000})
        assert result == {'key': key, 'value': 7000, 'apply': 'next_request'}
        values = {key: 6000, 'camera.analogue_gain': 2.0}
        assert await s.dispatch(None, 'params.set', {'values': values}) == {'values': values}
        assert await s.dispatch(None, 'params.get', {'keys': list(values)}) == {'values': values}
        s.require_control = lambda *_: (_ for _ in ()).throw(AssertionError('read requires lease'))
        assert (await s.dispatch(None, 'params.get', {'keys': [key]}))['values'][key] == 6000
    asyncio.run(run())


@pytest.mark.parametrize('body', [
    {'keys': []}, {'keys': 'camera.exposure_us'}, {'keys': [1]},
    {'keys': ['camera.exposure_us'] * 2},
    {'keys': ['camera.exposure_us'], 'key': 'camera.exposure_us'},
    {'keys': ['camera.exposure_us', 'missing']},
    {'keys': [str(i) for i in range(17)]},
])
def test_invalid_batch_get_is_rejected(tmp_path, body):
    async def run():
        with pytest.raises(Fault):
            await server(tmp_path).dispatch(None, 'params.get', body)
    asyncio.run(run())


@pytest.mark.parametrize('body', [
    {'values': {}}, {'values': []},
    {'values': {'camera.exposure_us': 6000}, 'key': 'camera.exposure_us'},
    {'values': {'camera.exposure_us': 6000}, 'expected_values': {}},
    {'values': {'camera.exposure_us': 6000, 'missing': 1}},
    {'key': 'camera.exposure_us', 'value': 6000, 'expected_values': {}},
])
def test_invalid_batch_set_does_not_apply_anything(tmp_path, body):
    async def run():
        s = server(tmp_path)
        before = dict(s.params.values)
        with pytest.raises(Fault):
            await s.dispatch(None, 'params.set', body)
        assert s.params.values == before
        for worker in s.workers.values():
            worker.call.assert_not_awaited()
    asyncio.run(run())


def test_batch_get_waits_for_in_progress_parameter_transaction(tmp_path):
    async def run():
        s = server(tmp_path)
        await s.parameter_lock.acquire()
        task = asyncio.create_task(s.dispatch(None, 'params.get', {'keys': ['camera.exposure_us']}))
        await asyncio.sleep(0)
        assert not task.done()
        s.params.values['camera.exposure_us'] = 5000
        s.parameter_lock.release()
        assert await task == {'values': {'camera.exposure_us': 5000}}
    asyncio.run(run())


def test_unavailable_localisation_does_not_block_detector_settings(tmp_path):
    async def run():
        s = server(tmp_path)
        s.workers['localisation'].call.side_effect = Fault('worker_unavailable', 'localisation')
        key = 'vision.orange_ball.pixels_min'
        await s.dispatch(None, 'params.set', {'key': key, 'value': 51})
        s.workers['detection'].call.assert_awaited_once_with('params.apply', {key: 51})
        assert Parameters(tmp_path).values[key] == 51
    asyncio.run(run())


@pytest.mark.parametrize('error', ['invalid_argument', 'worker_timeout'])
def test_live_localisation_rejection_still_rolls_back_detector(tmp_path, error):
    async def run():
        s = server(tmp_path)
        s.workers['localisation'].call.side_effect = Fault(error, 'apply failed')
        key = 'vision.orange_ball.pixels_min'
        with pytest.raises(Fault) as caught:
            await s.dispatch(None, 'params.set', {'key': key, 'value': 51})
        assert caught.value.code == error
        assert s.workers['detection'].call.call_args_list[-1].args == ('params.apply', {key: 50})
        assert Parameters(tmp_path).values[key] == 50
    asyncio.run(run())


@pytest.mark.parametrize('settings,code', [
    ({}, None),
    ({'frame_duration_us': 16667, 'exposure_us': 8000, 'gain': 1.0}, None),
    ({'frame_duration_us': 33333}, 'restart_required'),
    ({'exposure_us': 9000}, 'restart_required'),
    ({'gain': 2.0}, 'restart_required'),
    ({'with_imu': False}, 'invalid_argument'),
    ({'frame_duration_us': 0}, 'invalid_argument'),
    ({'exposure_us': 20000}, 'invalid_argument'),
])
def test_running_camera_start_checks_requested_settings(tmp_path, settings, code):
    async def run():
        state = {'prepared': True, 'running': True, 'frame_duration_us': 16667,
                 'imu_sync': {'state': 'synced'},
                 'requested_controls': {'exposure_us': 8000, 'gain': 1.0}}
        s = server(tmp_path, state)
        s.mode = 'MANUAL'
        s.workers['stream'] = SimpleNamespace(call=AsyncMock(return_value={'active_streams': []}))
        if code:
            with pytest.raises(Fault) as caught:
                await s.dispatch(None, 'camera.start', settings)
            assert caught.value.code == code
        else:
            assert await s.dispatch(None, 'camera.start', settings) == state
        if 'with_imu' in settings:
            s.workers['camera'].call.assert_not_awaited()
        else:
            s.workers['camera'].call.assert_awaited_once_with('camera.status')
    asyncio.run(run())
