"""Experimental planar map fit after calibrated ground projection.

Uses an operator-supplied corner/heading prior to resolve symmetry. Does not
command the robot or claim tracking/independent accuracy from a single frame.
"""
import json
from pathlib import Path
import argparse
import cv2
import numpy as np
from scipy.optimize import least_squares
from tools.auto_field_calibration import segment
from tools.bird_view_probe import bird_view,head_angles


def solve(image,metadata,calibration,height):
    pitch,roll=head_angles(metadata['quaternion_xyzw'])
    _,roi,white,_=segment(image)
    paint,_=bird_view(cv2.cvtColor(white,cv2.COLOR_GRAY2BGR),calibration,pitch,roll,height)
    bgr=image.copy();bgr[roi==0]=0
    bird,visibility=bird_view(bgr,calibration,pitch,roll,height)
    mask=cv2.cvtColor(paint,cv2.COLOR_BGR2GRAY)
    edges=cv2.Canny(mask,70,150)
    segments=cv2.HoughLinesP(edges,1,np.pi/720,70,minLineLength=130,maxLineGap=15)
    if segments is None:raise ValueError('No long line observations')
    if segments.shape[-1]!=4:raise ValueError('Unexpected HoughLinesP layout')
    seg=segments.reshape(-1,2,2).astype(float)
    ground=np.stack((4-seg[...,1]/180,2-seg[...,0]/180),axis=-1)
    delta=ground[:,1]-ground[:,0]
    angle=np.arctan2(delta[:,1],delta[:,0])
    prior=np.array([-1.675,-1.175,np.arctan2(1.175,1.675)])
    # Discard directions inconsistent with either orthogonal map direction.
    err=np.abs((angle+prior[2]+np.pi/4)%(np.pi/2)-np.pi/4)
    ground=ground[err<np.deg2rad(12)]
    if len(ground)<6:raise ValueError('Insufficient orthogonal lines')
    points=np.concatenate([ground[:,0]*f+ground[:,1]*(1-f) for f in (0,.25,.5,.75,1)])
    model=[(-1.675,-1.175,1.675,-1.175),(-1.675,1.175,1.675,1.175),(-1.675,-1.175,-1.675,1.175),(1.675,-1.175,1.675,1.175),(0,-1.175,0,1.175)]
    def world(p):
        c,s=np.cos(p[2]),np.sin(p[2])
        return points@np.array([[c,s],[-s,c]])+p[:2]
    def distances(p):
        q=world(p);d=[]
        for ax,ay,bx,by in model:
            a=np.array([ax,ay]);v=np.array([bx-ax,by-ay])
            t=np.clip((q-a)@v/(v@v),0,1)
            d.append(np.linalg.norm(q-a-t[:,None]*v,axis=1))
        return np.min(d,axis=0)
    def residual(p):return np.r_[distances(p),(p[:2]-prior[:2])*.4,(p[2]-prior[2])*.15]
    fit=least_squares(residual,prior,bounds=(prior-[.55,.55,.35],prior+[.55,.55,.35]),loss='soft_l1',f_scale=.035,max_nfev=150)
    pose=fit.x;d=distances(pose)
    ppm=180;rows,cols=np.indices((round(3.35*ppm),round(2.35*ppm)),dtype=float)
    wx=1.675-(rows+.5)/ppm;wy=1.175-(cols+.5)/ppm
    c,s=np.cos(pose[2]),np.sin(pose[2]);dx=wx-pose[0];dy=wy-pose[1]
    rx=c*dx+s*dy;ry=-s*dx+c*dy
    mx=((2-ry)*ppm).astype(np.float32);my=((4-rx)*ppm).astype(np.float32)
    aligned=cv2.remap(bird,mx,my,cv2.INTER_LINEAR)
    report={'candidate_pose_m_rad':pose.tolist(),'operator_prior_m_rad':prior.tolist(),
            'median_line_residual_m':float(np.median(d)),'inlier_fraction_8cm':float(np.mean(d<.08)),
            'segments':len(ground),'height_assumed_m':height,'pose_valid':False,
            'status':'single_frame_candidate_not_tracking',
            'limits':['prior-dependent symmetry resolution','unverified camera height and IMU extrinsics','line identities inferred, not independently validated','crop is model window, not proof of detected playing boundary']}
    return aligned,report


if __name__=='__main__':
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('capture',type=Path);a.add_argument('calibration',type=Path)
    a.add_argument('--height-m',type=float,required=True)
    x=a.parse_args();m=json.loads((x.capture/'capture.json').read_text())
    if m['frame_sequence']-m['imu_sequence']!=m['unicam_minus_stm']:raise ValueError('Unmatched IMU')
    view,report=solve(cv2.imread(str(x.capture/'frame.png')),m,x.calibration,x.height_m)
    assert cv2.imwrite(str(x.capture/'field-map-candidate.png'),view)
    (x.capture/'pose-candidate.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
