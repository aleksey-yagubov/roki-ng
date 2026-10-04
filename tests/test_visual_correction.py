import asyncio
import math
import time
from types import SimpleNamespace

import numpy as np
import pytest

from roki_ng.localisation import PoseFilter
from roki_ng.localisation_pipeline import process_frame
from roki_ng.parameters import Parameters
from roki_ng.visual_correction import CorrectionWindow


def goal_observation(p, pose, colour='blue'):
    goal = p['field.goal.1']
    ends = np.array([[goal['x'],goal['y']-goal['width']/2],[goal['x'],goal['y']+goal['width']/2]])
    delta = ends-np.array(pose[:2])
    c,s = math.cos(pose[2]),math.sin(pose[2])
    return dict(colour=colour, ground_feet=(delta@np.array([[c,-s],[s,c]])).tolist(),
                bearings=(np.arctan2(delta[:,1],delta[:,0])-pose[2]).tolist())


def test_goal_only_recovers_pose_without_three_lines_or_manual_flag(tmp_path):
    p = Parameters(tmp_path).values
    pose = [-1.2,-.8,.2]
    engine = PoseFilter([0,0,0], parameters=p)
    for seq in range(1,4):
        result = engine.update(seq, [], None, [goal_observation(p,pose)], timestamp=seq*.3)
        assert result['valid'] and result['reason']=='ok'
        assert np.allclose(result['candidate'], pose, atol=.08)
    assert len(engine.particles)==2048  # Observation proposals do not accumulate.


def test_imu_heading_resolves_side_for_white_lines_but_not_parallel_geometry(tmp_path):
    from roki_ng.field_observations import field_model
    p = Parameters(tmp_path).values
    assert not PoseFilter([0,0,0],parameters=p).update(1,field_model(),None)['valid']
    result = PoseFilter([0,0,0],parameters=p,heading=0).update(1,field_model(),None)
    assert result['valid'] and np.linalg.norm(result['candidate'])<.1
    lines = [[[-1,y],[1,y]] for y in (-1.175,0,1.175)]
    assert not PoseFilter([0,0,0],parameters=p,heading=0).update(1,lines,None)['valid']


def test_two_white_goals_are_alternatives_not_false_side_proof(tmp_path):
    p = Parameters(tmp_path).values
    for i in (0,1):
        p[f'field.goal.{i}'] = p[f'field.goal.{i}'] | {'colour':'white'}
    observation = goal_observation(p,[0,0,0],colour='white')
    assert not PoseFilter([0,0,0],parameters=p).update(1,[],None,[observation])['valid']
    assert PoseFilter([0,0,0],parameters=p,heading=0).update(1,[],None,[observation])['valid']


@pytest.mark.parametrize('failed_stage', ['segmentation','circle','lines'])
def test_failure_in_paint_pipeline_does_not_skip_goals(tmp_path, monkeypatch, failed_stage):
    from roki_ng import field_observations as f, goal_observations as g
    p = Parameters(tmp_path).values
    image = np.zeros((650,800,3),np.uint8)
    def missing(*args,**kw):
        raise ValueError('missing paint')
    monkeypatch.setattr(f,'runtime_paint_mask',missing if failed_stage=='segmentation' else lambda *a: image[:,:,0])
    monkeypatch.setattr(f,'detect_circle',missing if failed_stage=='circle' else lambda *a,**kw: None)
    monkeypatch.setattr(f,'observations',missing if failed_stage=='lines' else lambda *a,**kw: [])
    post=dict(colour='blue',foot_px=[400,500],rect=[395,300,10,200],metric_valid=False)
    monkeypatch.setattr(g,'goal_candidates',lambda *a: [post])
    monkeypatch.setattr(g,'upright_candidates',lambda posts,*a: posts)
    monkeypatch.setattr(g,'paired_bearings',lambda *a: [goal_observation(p,[0,0,0])])
    monkeypatch.setattr(g,'mapped_candidates',lambda posts,*a: posts)
    projector = SimpleNamespace(project=lambda *a: np.zeros((720,720),np.uint8))
    result,_,_,posts = process_frame(image,[],projector,PoseFilter([0,0,0],parameters=p),p,1,.1)
    assert result['valid'] and result['goal_pairs']==1
    assert failed_stage in result['detector_errors']
    assert 'filter' in result['timings_ms'] and 'posts' in result['timings_ms']
    assert posts[0]['metric_valid']


def sample(seq, pose=None, revision=0, **changes):
    result=dict(window_id='w',frame_sequence=seq,sensor_timestamp_ns=seq*100_000_000,
                parameter_revision=revision,valid=True,fit_state='matched',ambiguous=False,
                candidate=pose or [0,0,0]) | changes
    return dict(running=True,error=None,result=result)


def test_three_distinct_exposures_required_and_parameters_break_consensus():
    w=CorrectionWindow('w',0)
    assert w.observe(sample(1),100_000_000) is None
    for _ in range(5):
        assert w.observe(sample(1),200_000_000) is None
    assert w.observe(sample(2),200_000_000) is None
    assert w.observe(sample(3,revision=1),300_000_000) is None
    assert w.observe(sample(4,revision=1),400_000_000) is None
    assert w.observe(sample(5,revision=1),500_000_000)==[0,0,0]
    assert w.observe(sample(6,revision=1),600_000_000) is None


@pytest.mark.parametrize('change', [dict(valid=False),dict(ambiguous=True),dict(fit_state='weak'),
                                  dict(candidate=[float('nan'),0,0]),dict(window_id='previous')])
def test_unusable_result_never_contributes(change):
    w=CorrectionWindow('w',0)
    for seq in (1,2,3):
        assert w.observe(sample(seq,**change),seq*100_000_000) is None
    assert not w.accepted


def test_capture_barrier_freshness_disagreement_and_wrapped_angles():
    w=CorrectionWindow('w',100_000_000)
    assert w.observe(sample(1),100_000_000) is None  # Before support pose.
    assert w.observe(sample(2),800_000_000) is None  # Late frame.
    assert w.observe(sample(3,[0,0,3.13]),300_000_000) is None
    assert w.observe(sample(4,[0,0,-3.13]),400_000_000) is None
    result=w.observe(sample(5,[0,0,math.pi]),500_000_000)
    assert result and abs(abs(result[2])-math.pi)<.01
    w=CorrectionWindow('w',0)
    for seq,x in enumerate((0,.1,.2),1):
        assert w.observe(sample(seq,[x,0,0]),seq*100_000_000) is None
    assert len(w.poses)==1  # Pairwise agreement, not a drifting chain.


def test_game_corrects_only_standing_and_rejects_late_reply_after_pause(tmp_path):
    from test_game import setup
    from roki_ng.game import capture_clock
    async def run():
        game,s,_=setup(tmp_path)
        game.info['running']=True
        assert await game._prepare(s.params.values,'goalkeeper')
        initial=game.odometry.state()
        pose=[initial['x_m']+.05,initial['y_m']+.03,initial['yaw_rad']+.02]
        game.visual_capture='capture'
        game.visual_window=CorrectionWindow('w',capture_clock()-1_000_000_000)
        sequence=0
        async def call(op,args=None,**kwargs):
            nonlocal sequence
            sequence+=1
            state=sample(sequence,pose,sensor_timestamp_ns=capture_clock())
            return state | {'capture_id':'capture'}
        s.workers['localisation']=SimpleNamespace(call=call)
        for _ in range(3):
            game.visual_poll_at=0
            await game._poll_localisation()
        assert game.info['last_correction']['observations']==3
        assert game.odometry.confidence=='visual'
        assert np.allclose([game.odometry.x,game.odometry.y,game.odometry.yaw],pose)
        await game._imu()
        assert game.odometry.yaw==pytest.approx(pose[2])  # Updated IMU field offset persists.
        gate,entered=asyncio.Event(),asyncio.Event()
        async def delayed(*args,**kw):
            entered.set();await gate.wait()
            return await call(*args,**kw)
        s.workers['localisation'].call=delayed
        game.visual_window=CorrectionWindow('w',0)
        game.visual_poll_at=0
        task=asyncio.create_task(game._poll_localisation())
        await entered.wait()
        await game.pause()
        gate.set();await task
        assert game.visual_window is None and game.odometry.x==pytest.approx(pose[0])
    asyncio.run(run())


def test_supervisor_exports_validity_and_capture_age(tmp_path):
    from roki_ng.supervisor import Supervisor
    s=Supervisor({'state_dir':str(tmp_path)})
    state=dict(running=True,age_ms=20,error=None,result={'valid':True,'candidate':[0,0,0]})
    worker=SimpleNamespace(alive=True,last_heartbeat=time.monotonic(),state=state)
    s.workers['localisation']=worker
    assert s._sample('localisation.state')['valid']
    worker.last_heartbeat-=.6
    assert not s._sample('localisation.state')['valid']
    worker.last_heartbeat=time.monotonic();worker.alive=False
    assert not s._sample('localisation.state')['valid']


def test_no_observation_does_not_block_settling_and_motion_closes_window_first(tmp_path):
    from test_game import setup
    from roki_ng.legacy_strategy import OriginalStrategy
    async def run():
        game,s,_=setup(tmp_path)
        game.info['running']=True
        assert await game._prepare(s.params.values,'goalkeeper')
        calls=[]
        async def localisation(op,args=None,**kw):
            calls.append(op)
            return dict(capture_id='c',running=True,error=None,result=None)
        body=s.workers['motherboard'].call
        async def motion(op,args=None,**kw):
            calls.append(op)
            if op=='game.jump':
                assert game.visual_window is None
                assert calls.index('localisation.window')<calls.index('game.jump')
            return await body(op,args,**kw)
        s.workers['motherboard'].call=motion
        s.workers['localisation']=SimpleNamespace(call=localisation)
        game.visual_capture='c'
        game.visual_window=CorrectionWindow('w',0)
        began=time.monotonic()
        assert await OriginalStrategy._settle(game,.03)
        assert time.monotonic()-began<.3
        assert game.info['last_correction'] is None
        assert await game._move('game.jump',{'direction':'left','fraction':.1})
        assert game.visual_window is None
    asyncio.run(run())


def test_pickup_and_stale_imu_discard_pending_correction(tmp_path):
    from test_game import setup
    from roki_ng.game import capture_clock
    async def run():
        game,s,_=setup(tmp_path)
        game.info['running']=True
        assert await game._prepare(s.params.values,'goalkeeper')
        game.visual_capture='c'
        game.visual_window=CorrectionWindow('w',0)
        for seq in (1,2):
            game.visual_window.observe(sample(seq),seq*100_000_000)
        async def response(*a,**kw):
            return sample(3,sensor_timestamp_ns=capture_clock()) | {'capture_id':'c'}
        s.workers['localisation']=SimpleNamespace(call=response)
        s.workers['motherboard'].bad_imu=True
        await game._poll_localisation()
        assert game.info['last_correction'] is None and game.visual_window is None
        s.workers['motherboard'].bad_imu=False
        game.visual_window=CorrectionWindow('w',0)
        await game.pickup()
        assert game.visual_window is None
    asyncio.run(run())
