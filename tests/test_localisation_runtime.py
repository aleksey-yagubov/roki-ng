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
    # Independent SciPy Rotation reference values, computed on the workstation.
    # Keep this regression runnable on Buildroot without desktop tools or SciPy.
    for q, expected in (
        ([.8980102539,-.1106567383,-.047668457,.4230957031],
         (0.6911682142181879, 0.008023745243401548)),
        ([.70947265625,-.05584716797,-.040649414,.70129394531],
         (0.013008758267155862, 0.020654057744836196)),
    ):
        assert np.allclose(head_angles(q), expected, atol=1e-8, rtol=0)
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


def test_parallel_lines_do_not_claim_constrained_geometry():
    from roki_ng.localisation import PoseFilter
    lines=np.array([[[-1.,y],[1.,y]] for y in [-1.175,0,1.175]])
    pf=PoseFilter([0,0,0],count=512)
    pf.particles[:]=0
    result=pf.update(1,lines,None)
    assert result['fit_state']!='matched'
    assert result['valid'] is False


def test_configuration_id_changes_only_for_metric_configuration():
    from roki_ng.field_config import configuration_id,DEFAULTS
    p=DEFAULTS|{'match.own_goal':0,'localisation.camera_height_m':.4068}
    assert configuration_id(p)==configuration_id(p|{'vision.field_auto':False})
    assert configuration_id(p)!=configuration_id(p|{'localisation.camera_height_m':.5})
    assert configuration_id(p)!=configuration_id(p|{'match.own_goal':1})


def test_proposal_ambiguity_does_not_become_physical_accuracy():
    from roki_ng.localisation import PoseFilter
    from roki_ng.field_observations import field_model
    pf=PoseFilter([0,0,0],count=512)
    pf.particles[:256]=[0,0,0]
    pf.particles[256:]=[0,0,np.pi]
    result=pf.update(1,field_model(),None)
    assert result['ambiguous']
    assert result['fit_state']=='ambiguous'
    assert result['valid'] is False


def test_segmentation_keeps_original_pixel_edges():
    import cv2
    from roki_ng.field_observations import runtime_paint_mask
    image=np.full((650,800,3),(40,140,45),np.uint8)
    cv2.rectangle(image,(80,80),(720,590),(240,240,240),7)
    cv2.line(image,(111,101),(651,551),(240,240,240),3)
    mask=runtime_paint_mask(image)
    # Half-size binary upscaling would make every 2x2 block constant.
    assert np.any(mask[0::2,0::2]!=mask[0::2,1::2])
    assert np.any(mask[0::2,0::2]!=mask[1::2,0::2])
    assert mask[326,381] and not mask[250,500]


def test_bounded_status_with_four_goals_fits_udp():
    import time
    from roki_ng.localisation_worker import Localisation
    from roki_ng.localisation import PoseFilter
    from roki_ng.field_observations import field_model
    from roki_ng.field_config import GEOMETRY
    from roki_ng.wire import envelope,pack
    worker=Localisation({'state_dir':'/tmp'},lambda *a:None,lambda *a:None)
    result=PoseFilter([0,0,0]).update(1,field_model(),None)
    result.pop('proposal_spread',None)
    result.update(sensor_timestamp_ns=2**60,frame_sequence=2**32-1,imu_sequence=2**32-1,
                  processing_ms=99999,parameter_revision=99999,
                  goal_candidates=[{'colour':'yellow','rect':[700,500,100,150],
                                    'foot_px':[799.,649.],'metric_valid':False}]*4)
    worker.result=result;worker.last_measurement=time.monotonic()
    worker.geometry=GEOMETRY;worker.configuration_id='f'*16;worker.capture_id='f'*32
    worker.frames=worker.dropped=2**32-1
    data={'topic':'localisation.state','valid':False,'source_mono_ns':2**60,
          'age_ms':99999,'data':worker.state()}
    packet=pack(envelope('sample','data.sample',data,session=2**63,token=2**63,sequence=2**32-1))
    assert len(packet)<=1200


def test_circle_refinement_rejects_filled_disc():
    import cv2
    from roki_ng.field_observations import detect_circle
    mask=np.zeros((720,720),np.uint8)
    cv2.circle(mask,(350,400),48,255,-1)
    assert detect_circle(mask,.5) is None


def test_halfway_association_uses_configured_map_size():
    from roki_ng.field_observations import likelihood
    from roki_ng.field_config import DEFAULTS,line_model,circle_model
    params=DEFAULTS|{'field.geometry':DEFAULTS['field.geometry']|{'length':6.,'width':4.}}
    model=line_model(params)
    circle={'center_robot_m':[0.,0.]}
    lines=np.array([[[0.,-1.9],[0.,1.9]]])
    particles=np.array([[0,0,0],[3.,0,0]])
    a,_=likelihood(particles,lines,model,circle,circle_model(params))
    b,_=likelihood(particles,lines,model,circle,None)
    assert np.allclose(a,b)
    assert a[0]>a[1]


def test_weak_geometry_does_not_move_particle_proposals():
    from roki_ng.localisation import PoseFilter
    pf=PoseFilter([0,0,0],count=512)
    # Three parallel fragments leave position along the lines unconstrained.
    lines=np.array([[[-1.,y],[1.,y]] for y in (-1.175,0.,1.175)])
    before=pf.particles.copy()
    for sequence in range(1,6):
        result=pf.update(sequence,lines,None)
        assert result['fit_state']!='matched'
    assert np.array_equal(pf.particles,before)


def test_ambiguous_mirrored_field_does_not_collapse_to_one_side():
    from roki_ng.localisation import PoseFilter
    from roki_ng.field_observations import field_model
    pf=PoseFilter([0,0,0],count=512)
    pf.particles[:256]=[0,0,0]
    pf.particles[256:]=[0,0,np.pi]
    before=pf.particles.copy()
    for sequence in range(1,6):
        assert pf.update(sequence,field_model(),None)['fit_state']=='ambiguous'
    assert np.array_equal(pf.particles,before)


def test_consistent_unambiguous_geometry_can_update_proposals():
    from roki_ng.localisation import PoseFilter
    from roki_ng.field_observations import field_model
    pf=PoseFilter([0,0,0],count=512)
    pf.particles[:]=0
    before=pf.particles.copy()
    result=pf.update(1,field_model(),None)
    assert result['fit_state']=='matched'
    assert not np.array_equal(pf.particles,before)
    assert result['valid'] is False


def test_circle_recovers_translation_without_claiming_unique_field_side():
    from roki_ng.localisation import PoseFilter
    from roki_ng.field_observations import field_model
    pose=np.array([-1.3,-.95,.2]);c,s=np.cos(pose[2]),np.sin(pose[2])
    rotation=np.array([[c,-s],[s,c]])
    lines=(field_model()-pose[:2])@rotation
    circle={'center_robot_m':(-pose[:2]@rotation).tolist()}
    pf=PoseFilter([0,0,0], count=2048)
    before=pf.particles.copy()
    result=pf.update(1,lines,circle)
    assert result['inlier_fraction'] >= .8
    assert result['median_residual_m'] < .08
    assert result['ambiguous'] and not result['valid']
    assert np.array_equal(before,pf.particles)


def test_ground_projection_matches_legacy_matrix_rays(monkeypatch):
    import cv2,math
    from roki_ng.ground_projection import GroundProjection,head_angles
    g=GroundProjection.__new__(GroundProjection)
    g.P=np.array([[500.,0,800],[0,500.,650],[0,0,1]])
    row,col=np.indices((720,720),dtype=np.float32)
    g.ground=np.stack((4-(row+.5)/180,2-(col+.5)/180,np.full_like(row,-.4068)),axis=-1)
    g.mx=g.my=np.zeros((1300,1600),np.float32)
    recorded=[]
    def remap(source,u,v,*args,**kwargs):
        recorded.append((u.copy(),v.copy()))
        return np.zeros(u.shape,np.float32)
    monkeypatch.setattr(cv2,'remap',remap)
    q=[.89,-.07,-.03,.45]
    g.project(np.zeros((650,800),np.uint8),q)
    pitch,roll=head_angles(q);cr,sr,cp,sp=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
    rotation=np.array([[cp,0,sp],[sr*sp,cr,-sr*cp],[-cr*sp,sr,cr*cp]])
    ray=g.ground.astype(float)@rotation;denom=np.where(ray[...,0]>.01,ray[...,0],1.)
    u=800-500*ray[...,1]/denom;v=650-500*ray[...,2]/denom
    visible=(u>=0)&(u<1599)&(v>=0)&(v<1299)&(ray[...,0]>.01)
    assert np.allclose(recorded[0][0][visible],u[visible],atol=.001)
    assert np.allclose(recorded[0][1][visible],v[visible],atol=.001)


def test_coloured_goal_pair_breaks_symmetry_but_rejects_teleport():
    from roki_ng.localisation import PoseFilter
    from roki_ng.field_observations import field_model
    from roki_ng.parameters import SCHEMA
    p={k:v[1] for k,v in SCHEMA.items()}
    pose=np.array([-1.2,-.8,.2]);c,s=np.cos(pose[2]),np.sin(pose[2]);rot=np.array([[c,-s],[s,c]])
    lines=(field_model()-pose[:2])@rot;circle={'center_robot_m':(-pose[:2]@rot).tolist()}
    g=p['field.goal.1'];ends=np.array([[g['x'],g['y']-.5],[g['x'],g['y']+.5]])-pose[:2]
    angles=(np.arctan2(ends[:,1],ends[:,0])-pose[2]).tolist()
    pf=PoseFilter([0,0,0],parameters=p)
    result=pf.update(1,lines,circle,[{'colour':'blue','bearings':angles,'ground_feet':(ends@rot).tolist()}],timestamp=10.)
    assert result['fit_state']=='matched'
    assert np.linalg.norm(np.asarray(result['candidate'])[:2]-pose[:2])<.15
    assert result['goal_pairs']==1 and not result['valid']
    before=pf.particles.copy();anchor=pf.anchor.copy()
    result=pf.update(2,lines,circle,[{'colour':'yellow','bearings':angles,'ground_feet':(ends@rot).tolist()}],timestamp=10.1)
    assert result['reason']=='motion_discontinuity'
    assert result['candidate'] is None
    assert np.array_equal(pf.particles,before) and np.array_equal(pf.anchor,anchor)
    later=pf.update(3,lines,circle,[{'colour':'yellow','bearings':angles,'ground_feet':(ends@rot).tolist()}],timestamp=20.)
    assert later['fit_state']=='matched'  # Enough elapsed time for traversal.
    assert later['candidate'] is not None


def test_goal_pair_rejects_single_fragment_and_duplicate_colours():
    from roki_ng.goal_observations import paired_bearings,bearing_log_likelihood
    class Projector:
        def bearing(self,p,q):return p[0]/500
        def ground_point(self,p,q):return np.array(p)/100
    one={'colour':'blue','rect':[20,20,10,50],'foot_px':[25,70]}
    assert paired_bearings([one],Projector(),[])==[]
    assert paired_bearings([one,one|{'foot_px':[30,70]}],Projector(),[])==[]
    goals=[{'colour':'blue','x':-1,'y':0,'width':1},{'colour':'blue','x':1,'y':0,'width':1}]
    score,used=bearing_log_likelihood(np.array([[0,0,0.]]),[{'colour':'blue','bearings':[-.2,.2]}],goals)
    assert used==0 and score[0]==0


def test_matching_goal_bearings_without_field_contact_do_not_reweight_pose():
    from roki_ng.goal_observations import bearing_log_likelihood
    particles=np.array([[0.,0.,0.],[0.,0.,np.pi]])
    goals=[{'colour':'blue','x':1.675,'y':0.,'width':1.}]
    feet=np.array([[1.675,-.5],[1.675,.5]])
    angles=np.arctan2(feet[:,1],feet[:,0]).tolist()
    # Same directions, but the coloured objects are far behind the end line.
    score,used=bearing_log_likelihood(particles,[{'colour':'blue','bearings':angles,'ground_feet':(feet*3).tolist()}],goals)
    assert used==0 and np.array_equal(score,[0.,0.])
    score,used=bearing_log_likelihood(particles,[{'colour':'blue','bearings':angles,'ground_feet':feet.tolist()}],goals)
    assert used==1 and score[0]>score[1]


def test_post_ground_intersection_rejects_horizon():
    from roki_ng.ground_projection import GroundProjection
    g=GroundProjection.__new__(GroundProjection)
    g.ground=np.array([[[0.,0.,-.4]]])
    g._pixel_ray=lambda *a:np.array([1.,0.,.1])
    with pytest.raises(ValueError,match='horizon'):g.ground_point([400,300],[0,0,0,1])
    g._pixel_ray=lambda *a:np.array([1.,.5,-.2])
    assert np.allclose(g.ground_point([400,300],[0,0,0,1]),[2.,1.])


def test_map_gate_rejects_background_and_unmatched_pose():
    from roki_ng.goal_observations import mapped_candidates
    class Projector:
        def ground_point(self,pixel,q):return np.asarray(pixel,float)
    goals=[{'colour':'blue','x':1.675,'y':0.,'width':1.}]
    posts=[{'colour':'blue','foot_px':[1.675,.5]},
           {'colour':'blue','foot_px':[2.4,.5]},
           {'colour':'yellow','foot_px':[1.675,-.5]}]
    matched={'fit_state':'matched','candidate':[0.,0.,0.]}
    assert mapped_candidates(posts,Projector(),[],matched,goals,.35)==posts[:1]
    for state in ('weak','ambiguous','rejected'):
        assert mapped_candidates(posts,Projector(),[],dict(matched,fit_state=state),goals,.35)==[]
    # Frame and world axes can differ: rotate a real observation back to the goal.
    turned=[dict(posts[0],foot_px=[.5,-1.675])]
    assert mapped_candidates(turned,Projector(),[],dict(matched,candidate=[0.,0.,np.pi/2]),goals,.35)==turned


def test_alternative_pairs_of_same_goal_are_not_extra_evidence():
    from roki_ng.goal_observations import bearing_log_likelihood
    particles=np.array([[0.,0.,0.],[0.,0.,np.pi]])
    feet=np.array([[1.675,-.5],[1.675,.5]])
    pair={'colour':'blue','bearings':np.arctan2(feet[:,1],feet[:,0]),'ground_feet':feet}
    goals=[{'colour':'blue','x':1.675,'y':0.,'width':1.}]
    once,count=bearing_log_likelihood(particles,[pair],goals)
    twice,count2=bearing_log_likelihood(particles,[pair,pair],goals)
    np.testing.assert_allclose(once,twice)
    assert count==count2==1


def test_upright_height_filter_rejects_near_ground_colour_fragments():
    from roki_ng.goal_observations import upright_candidates
    class Projector:
        def upright_height(self,foot,top,q):return foot[0]
    goals=[{'colour':'yellow','height':.6}]
    posts=[{'colour':'yellow','rect':[10,10,10,30],'foot_px':[h,0]} for h in (.55,.05,1.3)]
    assert upright_candidates(posts,Projector(),[],goals,.5)==posts[:1]


def test_adaptive_paint_excludes_carpet_outer_edge_but_keeps_inner_boundary():
    import cv2
    from roki_ng.field_observations import runtime_paint_mask
    image=np.full((650,800,3),230,np.uint8)
    image[100:600,60:740]=(40,140,45)
    cv2.rectangle(image,(90,130),(710,570),(245,245,245),6)
    cv2.line(image,(90,350),(710,350),(245,245,245),6)
    mask=runtime_paint_mask(image)
    assert mask[130,300]>0 and mask[350,300]>0
    assert not mask[96:100,200:600].any()
