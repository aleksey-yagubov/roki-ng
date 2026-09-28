import cv2
import numpy as np
import pytest
from tools.auto_field_calibration import segment,fit,correct,maps


@pytest.mark.parametrize('brightness',[.65,1.0])
def test_automatic_colour_model_separates_paint_at_different_brightness(brightness):
    im=np.full((650,800,3),(80,65,60),np.uint8)
    cv2.rectangle(im,(100,150),(700,620),(40,140,45),-1)
    cv2.rectangle(im,(100,400),(700,620),(25,100,30),-1)
    cv2.rectangle(im,(125,175),(675,590),(240,240,240),8)
    cv2.line(im,(125,350),(675,350),(240,240,240),8)
    im=(im*brightness).astype(np.uint8)
    _,roi,white,_=segment(im)
    assert roi[500,400] and roi[250,400]
    assert white[350,400] and white[590,400]
    assert not white[250,400] and not white[500,400]
    assert not white[50,400]


def test_empty_scene_rejected():
    with pytest.raises(ValueError,match='turf'):
        segment(np.full((650,800,3),180,np.uint8))


def test_recovers_synthetic_division_distortion_on_unseen_samples():
    lines=[];k=-1.3
    for x0,x1 in [(100,250),(680,540)]:
        points=np.column_stack((np.linspace(x0,x1,100),np.linspace(200,610,100)))
        u=(points-[400,325])/800
        ru=np.linalg.norm(u,axis=1)
        rd=2*ru/(1+np.sqrt(1-4*k*ru**2))
        lines.append(u*(rd/ru)[:,None]*800+[400,325])
    p,quality=fit(lines,(650,800))
    assert abs(p[2]-k)<.01
    assert quality['accepted_for_preview']
    assert max(quality['held_out_rms_after_px'])<.01
    mx,my=maps(p,(650,800))
    ys,xs=np.mgrid[50:601:100,50:751:100]
    src=np.column_stack((mx[ys,xs].ravel(),my[ys,xs].ravel()))
    output=(correct(src,p,(650,800))-[400,325])*.65+[400,325]
    assert np.max(np.abs(output-np.column_stack((xs.ravel(),ys.ravel()))))<.001
