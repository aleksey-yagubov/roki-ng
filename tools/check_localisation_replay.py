"""Replay a saved synchronized frame without opening camera or commanding motion."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np

from roki_ng.parameters import SCHEMA
from roki_ng.field_observations import runtime_paint_mask, observations, detect_circle
from roki_ng.ground_projection import GroundProjection
from roki_ng.localisation import PoseFilter
from roki_ng.goal_observations import goal_candidates


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('calibration',type=Path)
    parser.add_argument('--prior',type=float,nargs=3,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    parameters={k:v[1] for k,v in SCHEMA.items()}
    image=cv2.imread(str(args.capture/'frame.png'))
    meta=json.loads((args.capture/'capture.json').read_text())
    if image is None:raise ValueError('Missing frame.png')
    if meta['frame_sequence']-meta['imu_sequence']!=meta['unicam_minus_stm']:
        raise ValueError('Unmatched frame/IMU')
    cv2.setNumThreads(1)
    started=time.monotonic()
    projector=GroundProjection(args.calibration,parameters['localisation.camera_height_m'])
    mask=runtime_paint_mask(image,parameters)
    paint=projector.project(mask,meta['quaternion_xyzw'])
    circle=detect_circle(paint,scale=.5)
    if circle:
        x,y,r=circle['pixel_circle'];yy,xx=np.indices(paint.shape)
        paint[np.abs(np.hypot(xx-x,yy-y)-r)<8]=0
    lines=observations(paint,scale=.5)[:32]
    result=PoseFilter(args.prior,parameters=parameters).update(meta['frame_sequence'],lines,circle)
    result['goal_candidates']=goal_candidates(image,parameters)
    result['replay_processing_ms']=round((time.monotonic()-started)*1000)
    result['source']='saved_frame_single_measurement'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
