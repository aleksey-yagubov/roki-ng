"""Offline ground-plane preview using legacy calibration and matched head IMU.

Output axes: forward at top, left at left. Height is supplied, not measured.
This is not an estimated global robot pose.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def head_angles(quaternion):
    q=np.asarray(quaternion,dtype=float)
    if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<.1:
        raise ValueError('Invalid head quaternion')
    pitch,roll,_=Rotation.from_quat(q).as_euler('xyz')
    pitch=(pitch-np.pi/2+np.pi)%(2*np.pi)-np.pi
    return float(pitch),float(-roll)


def bird_view(image,calibration,pitch,roll,height_m,forward_m=4.0,half_width_m=2.0,ppm=180):
    if not .1<=height_m<=1.5:raise ValueError('Supply actual/assumed camera height in metres')
    P=np.load(calibration/'Camera_calibration_P.npy')
    m1=np.load(calibration/'Camera_calibration_map1.npy')
    m2=np.load(calibration/'Camera_calibration_map2.npy')
    sx,sy=cv2.convertMaps(m1,m2,cv2.CV_32FC1)
    h,w=round(forward_m*ppm),round(2*half_width_m*ppm)
    row,col=np.indices((h,w),dtype=np.float64)
    # Robot/head reference x forward, y left, z up. Follow legacy IMU convention.
    ground=np.stack((forward_m-(row+.5)/ppm,half_width_m-(col+.5)/ppm,np.full((h,w),-height_m)),axis=-1)
    R=Rotation.from_euler('x',roll).as_matrix()@Rotation.from_euler('y',pitch).as_matrix()
    ray=ground@R
    front=ray[...,0]>.01
    denom=np.where(front,ray[...,0],1.)
    u=(P[0,2]-P[0,0]*ray[...,1]/denom).astype(np.float32)
    v=(P[1,2]-P[1,1]*ray[...,2]/denom).astype(np.float32)
    valid=front&(u>=0)&(v>=0)&(u<sx.shape[1]-1)&(v<sx.shape[0]-1)
    mx=cv2.remap(sx,u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=-100)*image.shape[1]/1600
    my=cv2.remap(sy,u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT,borderValue=-100)*image.shape[0]/1300
    valid&=(mx>=0)&(my>=0)&(mx<image.shape[1]-1)&(my<image.shape[0]-1)
    mx[~valid]=-100;my[~valid]=-100
    return cv2.remap(image,mx,my,cv2.INTER_LINEAR),valid


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('image',type=Path);p.add_argument('metadata',type=Path)
    p.add_argument('calibration',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--height-m',type=float,required=True)
    a=p.parse_args();m=json.loads(a.metadata.read_text())
    if m['frame_sequence']-m['imu_sequence']!=m['unicam_minus_stm']:
        raise ValueError('Frame and head IMU do not match')
    pitch,roll=head_angles(m['quaternion_xyzw'])
    im=cv2.imread(str(a.image))
    if im is None:raise ValueError('Cannot read frame')
    result,valid=bird_view(im,a.calibration,pitch,roll,a.height_m)
    a.output.mkdir(parents=True,exist_ok=True)
    assert cv2.imwrite(str(a.output/'bird-view.png'),result)
    report={'pitch_rad':pitch,'roll_rad':roll,'height_m':a.height_m,'height_source':'supplied assumption, not measured',
            'legacy_imu_mounting_assumed':True,'global_pose_valid':False,'visible_fraction':float(valid.mean()),'capture':m}
    (a.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))

if __name__=='__main__':main()
