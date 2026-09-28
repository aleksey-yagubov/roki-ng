"""Offline automatic colour segmentation and plumb-line distortion candidate.

No clicks, supplied pixel coordinates or robot motion. Candidate only: a single
view does not establish full camera intrinsics or metric robot pose.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares


def clusters(values, count):
    cv2.setRNGSeed(17)
    sample=np.ascontiguousarray(values[::max(1,len(values)//30000)],np.float32)
    _,_,centres=cv2.kmeans(sample,count,None,(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,60,.1),3,cv2.KMEANS_PP_CENTERS)
    return centres


def classify(values, centres):
    return np.argmin(np.sum((values[...,None,:]-centres)**2,axis=-1),axis=-1)


def largest(mask):
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask)
    if n<2:return np.zeros_like(mask)
    return (labels==1+np.argmax(stats[1:,cv2.CC_STAT_AREA])).astype(np.uint8)*255


def segment(image):
    h,w=image.shape[:2]
    lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB).astype(np.float32)
    centres=clusters(lab.reshape(-1,3),5)
    labels=classify(lab,centres)
    populations=np.bincount(labels.ravel(),minlength=5)
    possible=[i for i in range(5) if centres[i,1]<120 and populations[i]>h*w*.05]
    if not possible:raise ValueError('No dominant green turf cluster')
    turf=min(possible,key=lambda i:centres[i,1])
    green=np.isin(labels,possible).astype(np.uint8)*255
    green=cv2.morphologyEx(green,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    interior=largest(green)
    k=max(3,round(w*.04)|1)
    joined=cv2.morphologyEx(green,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(k,k)))
    contour,_=cv2.findContours(largest(joined),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    roi=np.zeros((h,w),np.uint8)
    cv2.fillConvexPoly(roi,cv2.convexHull(max(contour,key=cv2.contourArea)),255)
    roi=cv2.dilate(roi,np.ones((11,11),np.uint8))
    local=clusters(lab[roi!=0],3)
    # Paint is the bright, near-neutral cluster; a green cluster is never white.
    white_id=min(range(3),key=lambda i:float(np.linalg.norm(local[i,1:]-128)-local[i,0]*.25))
    if local[white_id,0] < np.median(lab[green!=0,0])+15:
        raise ValueError('White paint not separated from turf')
    white=((classify(lab,local)==white_id)&(roi!=0)).astype(np.uint8)*255
    return interior,roi,white,{'turf_lab':centres[turf].tolist(),'roi_lab_centres':local.tolist(),'white_cluster':int(white_id),'method':'per-frame deterministic LAB clustering'}


def traces(interior,white):
    h,w=white.shape
    ys,xs=np.nonzero(interior)
    if not len(xs):raise ValueError('No interior')
    result=[]
    # Largest unjoined green interior is bounded by the actual paint; the
    # external green apron is disconnected by the side lines and is not used.
    for side in ('left','right'):
        points=[]
        for y in range(int(ys.min()+.28*(ys.max()-ys.min())),int(ys.max()-.06*h),3):
            row=np.flatnonzero(interior[y])
            if len(row)<w*.2:continue
            edge=row[0] if side=='left' else row[-1]
            lo,hi=max(0,edge-18),min(w,edge+19)
            xx=np.flatnonzero(white[y,lo:hi])+lo
            if len(xx)<2:continue
            points.append((float(np.median(xx)),float(y)))
        if len(points)<25:raise ValueError('Insufficient side-line support')
        result.append(np.array(points))
    return result


def correct(points,p,shape):
    h,w=shape[:2]
    cx,cy,k1,k2=p
    d=(np.asarray(points)-[cx*w,cy*h])/w
    r2=np.sum(d*d,axis=-1)
    denom=1+k1*r2+k2*r2*r2
    return d/denom[...,None]*w+[cx*w,cy*h]


def line_error(points):
    centred=points-points.mean(axis=0)
    _,_,vh=np.linalg.svd(centred,full_matrices=False)
    return centred@vh[-1]


def fit(lines,shape):
    train=[x[::2] for x in lines]
    valid=[x[1::2] for x in lines]
    # Fix optical centre provisionally to the image centre. Two side lines do
    # not constrain a freely moving centre and higher-order coefficients well.
    def parameters(k):return [.5,.5,float(k[0]),0.0]
    def residual(k):
        return np.concatenate([line_error(correct(x,parameters(k),shape)) for x in train])
    r=least_squares(residual,[-1.0],bounds=([-3.0],[0.0]),loss='soft_l1',f_scale=1.0)
    p=parameters(r.x)
    before=[float(np.sqrt(np.mean(line_error(x)**2))) for x in valid]
    after=[float(np.sqrt(np.mean(line_error(correct(x,p,shape))**2))) for x in valid]
    # Compare at the same local scale: correction can expand coordinates.
    ok=all(a<b*.65 for a,b in zip(after,before)) and all(a<3 for a in after)
    return p,{'accepted_for_preview':bool(ok),'held_out_rms_before_px':before,'held_out_rms_after_px':after,
              'validation':'alternating samples on the same two traces; no independent-view validation',
              'centre_assumed':True,'full_calibration_valid':False,'pose_valid':False}


def maps(p,shape,scale=.65):
    h,w=shape[:2]
    yy,xx=np.indices((h,w),dtype=float)
    cu=np.array([p[0]*w,p[1]*h])
    u=(np.stack((xx,yy),axis=-1)-cu)/scale/w
    ru=np.linalg.norm(u,axis=-1)
    lo=np.zeros_like(ru);hi=np.full_like(ru,1.0)
    # Single-parameter negative division model is monotonic before its pole.
    if p[2]<0:hi[:]=min(1.0,np.sqrt(-1/p[2])*.999)
    for _ in range(40):
        mid=(lo+hi)/2
        projected=mid/(1+p[2]*mid**2)
        take=projected<ru
        lo=np.where(take,mid,lo);hi=np.where(take,hi,mid)
    rd=(lo+hi)/2
    d=u*(rd/np.maximum(ru,1e-12))[...,None]
    points=d*w+cu
    return points[...,0].astype(np.float32),points[...,1].astype(np.float32)


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('image',type=Path);a.add_argument('output',type=Path)
    args=a.parse_args();im=cv2.imread(str(args.image))
    if im is None:raise SystemExit('Cannot read image')
    args.output.mkdir(parents=True,exist_ok=True)
    interior,roi,white,colours=segment(im)
    lines=traces(interior,white)
    p,quality=fit(lines,im.shape)
    overlay=im.copy();overlay[roi==0]=(overlay[roi==0]*.15).astype(np.uint8)
    overlay[white!=0]=(200,0,200)
    for line in lines:cv2.polylines(overlay,[np.rint(line).astype(np.int32)],False,(0,255,255),3)
    cv2.imwrite(str(args.output/'automatic-lines.png'),overlay)
    if quality['accepted_for_preview']:
        mx,my=maps(p,im.shape)
        cv2.imwrite(str(args.output/'straightened.png'),cv2.remap(im,mx,my,cv2.INTER_LINEAR))
        cv2.imwrite(str(args.output/'straightened-lines.png'),cv2.remap(overlay,mx,my,cv2.INTER_LINEAR))
        np.save(args.output/'map_x.npy',mx);np.save(args.output/'map_y.npy',my)
    report={'source':str(args.image.resolve()),'colours':colours,'division_model':p,'quality':quality,'trace_points':[len(x) for x in lines]}
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))

if __name__=='__main__':main()
