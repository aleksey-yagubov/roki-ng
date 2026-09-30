import copy

import pytest

from roki_ng.parameters import Parameters
from roki_ng.field_config import DEFAULTS, line_model, circle_model
from roki_ng.wire import Fault, pack


def test_defaults_roundtrip_and_wire_size(tmp_path):
    p=Parameters(tmp_path)
    for key,value in DEFAULTS.items():
        assert p.values[key]==value
        pack({'type':'response','session':2**63,'id':2**63,'ok':True,'result':p.describe(key)})
    key='field.mark.00'
    mark=p.values[key]|{'enabled':True,'x':.5,'kind':'ring'}
    p.set(key,mark)
    assert Parameters(tmp_path).values[key]==mark
    assert DEFAULTS[key]['enabled'] is False


@pytest.mark.parametrize('update',[{'length':float('nan')},{'width':True},
    {'length':6.},{'circle_diameter':3.},{'unknown':1}])
def test_bad_geometry_preserves_file(tmp_path,update):
    p=Parameters(tmp_path);before=p.path.read_bytes()
    with pytest.raises(Fault):p.set('field.geometry',p.values['field.geometry']|update)
    assert p.path.read_bytes()==before


def test_custom_marks_and_side_change(tmp_path):
    p=Parameters(tmp_path)
    p.set('field.geometry',p.values['field.geometry']|{'length':3.,'width':2.})
    assert line_model(p.values)[0].tolist()==[[-1.5,-1.],[1.5,-1.]]
    p.set('field.mark.00',p.values['field.mark.00']|{'enabled':True,'x':.7,'y':.4})
    assert len(line_model(p.values))==7
    p.set('field.mark.01',p.values['field.mark.01']|{'enabled':True,'kind':'ring','x':-.7})
    assert circle_model(p.values)[1]['center']==[-.7,0.]
    before=copy.deepcopy(p.values)
    p.set('match.own_goal',1)
    assert all(p.values[k]==v for k,v in before.items() if k.startswith('field.'))


def test_ring_not_forced_to_field_centre(tmp_path):
    import numpy as np
    from roki_ng.field_observations import likelihood
    p=Parameters(tmp_path)
    p.set('field.mark.00',p.values['field.mark.00']|{'enabled':True,'kind':'ring','x':1.,'size':.2})
    p.set('field.geometry',p.values['field.geometry']|{'circle_measured':True})
    particles=np.array([[0.,0.,0.],[-1.,0.,0.]])
    scores,_=likelihood(particles,[],line_model(p.values),
        {'center_robot_m':[1.,0.],'radius_observed_m':.1},circle_model(p.values))
    assert scores[0]>scores[1]


def test_field_compare_and_set_rejects_stale_editor(tmp_path):
    import asyncio
    from roki_ng.supervisor import Supervisor
    async def run():
        server=Supervisor({'state_dir':str(tmp_path)})
        key='field.geometry';old=copy.deepcopy(server.params.values[key]);new=old|{'length':3.}
        await server._set_parameters({key:new},expected={key:old})
        with pytest.raises(Fault,match='changed'):
            await server._set_parameters({key:old},expected={key:old})
        assert Parameters(tmp_path).values[key]==new
    asyncio.run(run())


def test_operator_colour_threshold_changes_goal_observation(tmp_path):
    import cv2
    import numpy as np
    from roki_ng.goal_observations import goal_candidates
    p=Parameters(tmp_path)
    image=np.full((650,800,3),(40,140,45),np.uint8)
    cv2.rectangle(image,(300,200),(310,400),(200,40,20),-1)
    assert any(c['colour']=='blue' for c in goal_candidates(image,p.values))
    p.set_many({'vision.blue_posts.l_min':95,'vision.blue_posts.l_max':100})
    assert not goal_candidates(image,p.values)


def test_goal_outside_turf_cannot_borrow_green_from_the_side(tmp_path):
    import cv2
    import numpy as np
    from roki_ng.goal_observations import goal_candidates
    p=Parameters(tmp_path)
    image=np.full((650,800,3),(100,100,100),np.uint8)
    image[300:620,200:760]=(40,140,45)
    # Bottom is outside the turf, but the old 41 px patch overlaps green at right.
    cv2.rectangle(image,(184,270),(192,410),(200,40,20),-1)
    assert goal_candidates(image,p.values)==[]


def test_true_goal_on_turf_boundary_retains_visible_support(tmp_path):
    import cv2
    import numpy as np
    from roki_ng.goal_observations import goal_candidates
    p=Parameters(tmp_path)
    image=np.full((650,800,3),(100,100,100),np.uint8)
    image[400:640,80:720]=(40,140,45)
    cv2.rectangle(image,(300,250),(312,400),(200,40,20),-1)
    assert any(c['colour']=='blue' for c in goal_candidates(image,p.values))


def test_post_support_can_cross_white_paint_but_not_background(tmp_path):
    import cv2
    import numpy as np
    from roki_ng.goal_observations import goal_candidates
    p=Parameters(tmp_path)
    image=np.full((650,800,3),(100,100,100),np.uint8)
    image[420:640,80:720]=(40,140,45)
    image[400:420,80:720]=255
    cv2.rectangle(image,(300,250),(312,400),(200,40,20),-1)
    assert any(c['colour']=='blue' for c in goal_candidates(image,p.values))
    image[401:420,80:720]=100
    assert not goal_candidates(image,p.values)
