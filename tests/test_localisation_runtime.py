import numpy as np
import pytest


def test_measurement_is_not_reused_or_accepted_out_of_order():
    from roki_ng.localisation import PoseFilter
    pf = PoseFilter([-1., -.5, .4], count=512)
    assert pf.update(10, [], None)['valid'] is False
    before=pf.particles.copy()
    with pytest.raises(ValueError,match='sequence'):
        pf.update(10, [], None)
    with pytest.raises(ValueError,match='sequence'):
        pf.update(9, [], None)
    assert np.array_equal(before,pf.particles)


def test_empty_frame_loses_pose_without_fabricated_measurement():
    from roki_ng.localisation import PoseFilter
    pf=PoseFilter([-1.,-.5,.4],count=512)
    result=pf.update(1,[],None)
    assert result['valid'] is False
    assert result['candidate'] is None
    assert result['reason']=='insufficient_observations'


def test_candidate_never_valid_without_verified_geometry():
    from roki_ng.localisation import PoseFilter
    from roki_ng.field_observations import field_model
    pose=np.array([-1.,-.5,.4]);c,s=np.cos(pose[2]),np.sin(pose[2])
    lines=(field_model()-pose[:2])@np.array([[c,-s],[s,c]])
    pf=PoseFilter(pose,count=1024)
    result=pf.update(1,lines,None)
    assert result['candidate'] is not None
    assert result['valid'] is False
    assert 'calibration' in result['reason']


def test_particle_count_and_prior_are_bounded():
    from roki_ng.localisation import PoseFilter
    for prior,count in [([0,0,float('nan')],512),([0,0,0],10000000)]:
        with pytest.raises(ValueError):PoseFilter(prior,count=count)


def test_quaternion_projection_matches_reference_without_scipy():
    from roki_ng.ground_projection import head_angles
    from tools.bird_view_probe import head_angles as reference
    for q in ([.8980102539,-.1106567383,-.047668457,.4230957031],
              [.70947265625,-.05584716797,-.040649414,.70129394531]):
        assert np.allclose(head_angles(q),reference(q),atol=1e-8)
    with pytest.raises(ValueError):head_angles([0,0,0,0])


def test_localisation_worker_starts_idle_and_reports_stale_candidate(tmp_path, monkeypatch):
    from roki_ng.localisation_worker import Localisation
    worker=Localisation({'state_dir':str(tmp_path)},lambda *a:None,lambda *a:None)
    assert worker.state()['running'] is False
    worker.result={'candidate':[0.,0.,0.], 'valid':False}
    worker.last_measurement=0.
    assert worker.state()['result']['valid'] is False
    assert worker.state()['result']['reason']=='stale'
    worker.close()
    assert worker.state()['result'] is None


def test_supervisor_rejects_localisation_without_imu_sync(tmp_path):
    import asyncio
    from types import SimpleNamespace
    from unittest.mock import AsyncMock
    from roki_ng.supervisor import Supervisor
    from roki_ng.wire import Fault
    async def run():
        server=Supervisor({'state_dir':str(tmp_path)});server.mode='MANUAL'
        server.require_control=lambda *a:None
        server.workers['camera']=SimpleNamespace(call=AsyncMock(return_value={'running':True,'imu_sync':{'state':'aligning'}}))
        worker=SimpleNamespace(call=AsyncMock());server.workers['localisation']=worker
        with pytest.raises(Fault,match='synchronized'):
            await server.dispatch(None,'localisation.start',{'prior':[0,0,0]})
        worker.call.assert_not_called()
    asyncio.run(run())


def test_capture_stop_stops_localisation_before_camera(tmp_path):
    import asyncio
    from types import SimpleNamespace
    from roki_ng.supervisor import Supervisor
    async def run():
        events=[];server=Supervisor({'state_dir':str(tmp_path)})
        async def call(op,*a,**kw):events.append(op);return {}
        server.workers={role:SimpleNamespace(call=call) for role in ['localisation','detection','stream','camera','motherboard']}
        await server._stop_capture()
        assert events.index('localisation.stop')<events.index('camera.stop')
    asyncio.run(run())


def test_localisation_datastream_does_not_call_candidate_valid(tmp_path):
    import time
    from types import SimpleNamespace
    from roki_ng.supervisor import Supervisor
    server=Supervisor({'state_dir':str(tmp_path)})
    server.workers['localisation']=SimpleNamespace(alive=True,last_heartbeat=time.monotonic(),
        state={'state':'ready','result':{'valid':False,'candidate':[0,0,0]}})
    assert server._sample('localisation.state')['valid'] is False


def test_runtime_segmentation_preserves_capture_coordinates():
    import cv2
    from roki_ng.field_observations import runtime_paint_mask
    im=np.full((650,800,3),(40,140,45),np.uint8)
    cv2.line(im,(80,320),(720,320),(240,240,240),8)
    cv2.rectangle(im,(80,100),(720,590),(240,240,240),8)
    mask=runtime_paint_mask(im)
    assert mask.shape==(650,800)
    assert mask[320,400] and mask[200,80]
    assert not mask[250,400]


def test_fast_colour_classification_matches_euclidean_distance():
    from roki_ng.field_observations import classify
    rng=np.random.default_rng(19)
    values=rng.uniform(0,255,(40,30,3)).astype(np.float32)
    centres=rng.uniform(0,255,(5,3)).astype(np.float32)
    expected=np.argmin(((values[...,None,:]-centres)**2).sum(axis=-1),axis=-1)
    assert np.array_equal(classify(values,centres),expected)


def test_half_resolution_features_keep_full_resolution_coordinates():
    import cv2
    from roki_ng.field_observations import observations,detect_circle
    mask=np.zeros((720,720),np.uint8)
    cv2.circle(mask,(370,410),50,255,7)
    cv2.line(mask,(80,410),(650,410),255,5)
    circle=detect_circle(mask,scale=.5)
    assert circle is not None
    assert np.linalg.norm(np.array(circle['pixel_circle'][:2])-[370,410])<6
    lines=observations(mask,scale=.5)
    assert any(abs(line[:,0].mean()-(4-410/180))<.04 for line in lines)


def test_goal_posts_require_vertical_structure_and_nearby_turf():
    import cv2
    from roki_ng.goal_observations import goal_candidates
    image=np.full((650,800,3),180,np.uint8)
    image[400:]=(40,140,45)
    cv2.rectangle(image,(180,300),(190,425),(200,40,20),-1)
    cv2.rectangle(image,(350,300),(360,425),(200,40,20),-1)
    cv2.rectangle(image,(180,300),(360,310),(200,40,20),-1)
    cv2.rectangle(image,(500,30),(510,100),(0,220,240),-1) # Background object.
    found=goal_candidates(image)
    assert len(found)==2
    assert all(x['colour']=='blue' for x in found)
    assert all(x['metric_valid'] is False for x in found)
    assert np.allclose([x['foot_px'][1] for x in found],425,atol=3)


def test_goal_detection_rejects_clipped_base():
    import cv2
    from roki_ng.goal_observations import goal_candidates
    image=np.full((650,800,3),(40,140,45),np.uint8)
    cv2.rectangle(image,(100,300),(120,649),(0,220,240),-1)
    assert goal_candidates(image)==[]


@pytest.mark.parametrize('colour,bgr', [('yellow',(0,220,240)), ('blue',(200,40,20))])
def test_goal_colours_preserve_identity_and_reject_background(colour,bgr):
    import cv2
    from roki_ng.goal_observations import goal_candidates
    image=np.full((650,800,3),180,np.uint8)
    image[350:]=(40,140,45)
    cv2.rectangle(image,(300,250),(310,400),bgr,-1)
    # A similarly coloured poster with an isolated green patch must not count.
    image[60:95,50:100]=(40,140,45)
    cv2.rectangle(image,(70,20),(80,70),bgr,-1)
    found=goal_candidates(image)
    assert len(found)==1
    assert found[0]['colour']==colour
    assert found[0]['foot_px']==[305.,400]
    assert found[0]['metric_valid'] is False
