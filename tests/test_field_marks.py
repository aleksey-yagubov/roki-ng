import math
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from roki_ng.field_config import disk_model,circle_model,line_model,short_line_lengths
from roki_ng.field_marks import disks,disk_scores,short_lines
from roki_ng.localisation import PoseFilter
from roki_ng.parameters import Parameters


@pytest.fixture
def parameters(tmp_path):
    p=Parameters(tmp_path).values
    for i,(x,y) in enumerate((x,y) for x in (-.9,.9) for y in (-.65,0,.65)):
        key=f'field.mark.{i:02d}'
        p[key].update(enabled=True,kind='disk',x=x,y=y,size=.05)
    for i,y in enumerate((-.65,0,.65),6):
        p[f'field.mark.{i:02d}'].update(enabled=True,kind='line',x=0.,y=y,size=.155,angle=0.)
    return p


def test_disks_are_separate_from_centre_ring_and_strokes(parameters):
    assert len(disk_model(parameters))==6
    assert circle_model(parameters)==[{'center':[0.,0.],'radius':None}]
    assert len(line_model(parameters))==8
    assert short_line_lengths(parameters)==[.155]


def test_disk_detection_retains_centres_but_rejects_rings_crosses_and_squares(parameters):
    mask=np.zeros((720,720),np.uint8)
    for x in (180,300,420):
        cv2.circle(mask,(x,300),4,255,-1)
    cv2.circle(mask,(180,400),5,255,1)
    cv2.rectangle(mask,(297,397),(303,403),255,-1)
    cv2.line(mask,(416,400),(424,400),255,2)
    cv2.line(mask,(420,396),(420,404),255,2)
    cv2.line(mask,(50,600),(660,600),255,5)
    cv2.circle(mask,(0,200),4,255,-1)
    found=disks(mask,disk_model(parameters))
    assert len(found)==3,found
    assert sorted(tuple(np.round(o['center_robot_m'],3)) for o in found)==[
        (2.333,-.333),(2.333,.333),(2.333,1.)]
    assert disks(mask,[])==[]


def test_short_ticks_across_long_centre_line_survive_detection():
    mask=np.zeros((720,720),np.uint8)
    cv2.line(mask,(80,360),(640,360),255,5)
    for x in (240,360,480):
        cv2.line(mask,(x,346),(x,374),255,3)
    long_lines=np.array([[[2.,-1.6],[2.,1.6]]])
    found=short_lines(mask,long_lines,[.155])
    assert 3<=len(found)<=8
    centres=np.array([row.mean(axis=0) for row in found])
    for y in (-2/3,0,2/3):
        assert np.min(np.linalg.norm(centres-[2.,y],axis=1))<.06
    for row in found:
        assert abs(row[1,0]-row[0,0])>abs(row[1,1]-row[0,1])


def test_disk_scores_use_position_and_preserve_field_symmetry(parameters):
    model=disk_model(parameters)
    observed=[dict(center_robot_m=m['center'],radius_observed_m=.025) for m in model[:3]]
    particles=np.array([[0.,0.,0.],[.4,.3,0.],[0.,0.,math.pi]])
    scores=disk_scores(particles,observed,model)
    assert scores[0]>scores[1]+1
    assert scores[0]==pytest.approx(scores[2])
    assert disk_scores(particles,observed*3,model)==pytest.approx(scores)
    wrong=[dict(center_robot_m=m['center'],radius_observed_m=.3) for m in model[:3]]
    assert disk_scores(particles,wrong,model)[0]<scores[0]


def test_disks_alone_do_not_certify_position(parameters):
    spots=[dict(center_robot_m=[.9,0.],radius_observed_m=.025)]
    result=PoseFilter([0,0,0],parameters=parameters,heading=0).update(1,[],None,spots=spots)
    assert not result['valid']


def test_short_strokes_alone_cannot_certify_even_with_circle(parameters):
    lines=line_model(parameters)[5:]
    result=PoseFilter([0,0,0],parameters=parameters,heading=0).update(
        1,lines,{'center_robot_m':[0.,0.]})
    assert not result['valid']


def test_pipeline_uses_spots_even_if_long_line_detector_fails(parameters,monkeypatch):
    from roki_ng import field_observations,goal_observations
    from roki_ng.localisation_pipeline import process_frame
    mask=np.zeros((720,720),np.uint8)
    cv2.circle(mask,(360,360),4,255,-1)
    monkeypatch.setattr(field_observations,'runtime_paint_mask',lambda *a:mask)
    monkeypatch.setattr(field_observations,'detect_circle',lambda *a,**kw:None)
    monkeypatch.setattr(field_observations,'observations',lambda *a,**kw:(_ for _ in ()).throw(ValueError('no long lines')))
    monkeypatch.setattr(goal_observations,'goal_candidates',lambda *a:[])
    calls=[]
    def update(*args,**kwargs):
        calls.append(kwargs['spots'])
        return dict(valid=False,candidate=None,frame_sequence=1,reason='insufficient_observations')
    engine=SimpleNamespace(update=update,goals=[],goal_foot_tolerance=.35)
    result,_,_,_=process_frame(np.zeros((650,800,3),np.uint8),[],
                              SimpleNamespace(project=lambda *a:mask),engine,parameters,1,1.)
    assert len(calls[0])==1 and result['spots']==1
    assert 'lines' in result['detector_errors']
    assert 'marks' in result['timings_ms']
