"""Cached legacy maps; explicit assumed floor height. No SciPy dependency."""
from pathlib import Path
import math
import cv2
import numpy as np


def head_angles(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q)<.1:
        raise ValueError('Invalid head quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    a = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    b = math.asin(float(np.clip(2*(w*y-z*x), -1, 1)))
    return (a-math.pi/2+math.pi)%(2*math.pi)-math.pi, -b


class GroundProjection:
    def __init__(self, directory, height):
        if not math.isfinite(height) or not .2<=height<=.8:
            raise ValueError('Invalid camera height')
        directory = Path(directory)
        self.P = np.load(directory/'Camera_calibration_P.npy', allow_pickle=False)
        m1 = np.load(directory/'Camera_calibration_map1.npy', allow_pickle=False)
        m2 = np.load(directory/'Camera_calibration_map2.npy', allow_pickle=False)
        if self.P.shape!=(3,3) or not np.isfinite(self.P).all() or min(self.P[0,0],self.P[1,1])<=0:
            raise ValueError('Invalid projection matrix')
        if m1.shape!=(1300,1600,2) or m1.dtype!=np.int16 or m2.shape!=(1300,1600) or m2.dtype!=np.uint16:
            raise ValueError('Incompatible legacy remap tables')
        self.mx,self.my = cv2.convertMaps(m1,m2,cv2.CV_32FC1)
        row,col=np.indices((720,720),dtype=np.float32)
        self.ground=np.stack((4-(row+.5)/180,2-(col+.5)/180,np.full_like(row,-height)),axis=-1)

    def project(self, image, quaternion):
        if image.shape[:2]!=(650,800):
            raise ValueError('Expected 800x650 capture; calibration is mode-specific')
        pitch,roll=head_angles(quaternion)
        cr,sr,cp,sp=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
        x,y,z=np.moveaxis(self.ground,-1,0)
        # Evaluate only the three required ray components; avoid allocating a
        # float64 HxWx3 matrix product on every camera frame.
        forward=cp*x+sr*sp*y-cr*sp*z
        left=cr*y+sr*z
        up=sp*x-sr*cp*y+cr*cp*z
        front=forward>.01
        denom=np.where(front,forward,1.)
        u=(self.P[0,2]-self.P[0,0]*left/denom).astype(np.float32)
        v=(self.P[1,2]-self.P[1,1]*up/denom).astype(np.float32)
        good=front&(u>=0)&(u<1599)&(v>=0)&(v<1299)
        mx=cv2.remap(self.mx,u,v,cv2.INTER_LINEAR,borderValue=-100)*.5
        my=cv2.remap(self.my,u,v,cv2.INTER_LINEAR,borderValue=-100)*.5
        good&=(mx>=0)&(mx<799)&(my>=0)&(my<649)
        mx[~good]=-100;my[~good]=-100
        return cv2.remap(image,mx,my,cv2.INTER_LINEAR)
