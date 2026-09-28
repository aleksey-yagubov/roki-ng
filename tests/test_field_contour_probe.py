import numpy as np
import cv2
from tools.field_contour_probe import extract


def test_lines_survive_turf_mask_but_white_background_is_excluded():
    im=np.full((650,800,3), 180, np.uint8)
    cv2.rectangle(im,(100,150),(700,600),(30,130,30),-1)
    cv2.rectangle(im,(125,175),(675,575),(240,240,240),8)
    cv2.line(im,(125,375),(675,375),(240,240,240),8)
    roi,lines,result=extract(im)
    assert result['valid'] and not result['pose_valid']
    assert lines[375,400] and lines[175,400] and lines[575,400]
    assert roi[250,400] and roi[450,400]
    assert not np.any(lines[:100])
    assert not np.any(roi[:100])


def test_no_turf_does_not_fall_back_to_whole_image():
    roi,lines,result=extract(np.full((650,800,3),255,np.uint8))
    assert not result['valid']
    assert not np.any(roi) and not np.any(lines)
