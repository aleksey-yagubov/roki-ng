"""Small white marks in the calibrated 180 px/m ground projection.

Disk discovery is separate from map association. Short strokes supplement long
lines; they do not independently certify a pose or resolve field-side symmetry.
"""
import cv2
import numpy as np


def disks(mask, model):
    if not model:
        return []
    binary=(mask>128).astype(np.uint8)
    count,labels,stats,centres=cv2.connectedComponentsWithStats(binary)
    radii=np.asarray([m['radius'] for m in model])*180
    candidates=[]
    for i in range(1,count):
        x,y,w,h,area=stats[i]
        # Require a complete, compact component; ignore tiny unresolved noise.
        if area<12 or min(w,h)<4 or min(w,h)/max(w,h)<.75:
            continue
        if x<2 or y<2 or x+w>=mask.shape[1]-2 or y+h>=mask.shape[0]-2:
            continue
        if max(w,h)>2*(max(radii)*1.35+2):
            continue
        component=(labels[y:y+h,x:x+w]==i).astype(np.uint8)
        contours,_=cv2.findContours(component,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        (cx,cy),radius=cv2.minEnclosingCircle(max(contours,key=cv2.contourArea))
        if radius<2 or np.min(np.abs(radii-radius))>max(2.,radius*.3):
            continue
        # Circle fill distinguishes disks from rings, crosses and square paint.
        if cv2.contourArea(max(contours,key=cv2.contourArea))/(np.pi*radius**2)<.70:
            continue
        if area/(np.pi*radius**2)<.8:
            continue
        cx,cy=cx+x,cy+y
        outer=int(np.ceil(radius+4))
        x0,y0=max(0,int(cx)-outer),max(0,int(cy)-outer)
        roi=binary[y0:min(mask.shape[0],int(cy)+outer+1),x0:min(mask.shape[1],int(cx)+outer+1)]
        yy,xx=np.indices(roi.shape)
        distance=np.hypot(xx+x0-cx,yy+y0-cy)
        surround=(distance>radius+1)&(distance<=radius+4)
        if not surround.any() or roi[surround].mean()>.15:
            continue
        candidates.append(dict(center_robot_m=[4-cy/180,2-cx/180],
                               radius_observed_m=radius/180,pixel_circle=[cx,cy,radius]))
    return candidates[:12]


def disk_scores(particles, observations, model):
    """Bounded contribution per map landmark, not per duplicate image fragment."""
    scores=np.zeros(len(particles))
    if not observations or not model:
        return scores
    observations=observations[:12]
    xy=np.asarray([o['center_robot_m'] for o in observations],float)
    radius=np.asarray([o['radius_observed_m'] for o in observations],float)
    if not np.isfinite(xy).all() or not np.isfinite(radius).all() or np.any(radius<=0):
        raise ValueError('Invalid disk observations')
    c,s=np.cos(particles[:,2,None]),np.sin(particles[:,2,None])
    wx=c*xy[:,0]-s*xy[:,1]+particles[:,0,None]
    wy=s*xy[:,0]+c*xy[:,1]+particles[:,1,None]
    centres=np.asarray([m['center'] for m in model])
    distances=(wx[:,:,None]-centres[:,0])**2+(wy[:,:,None]-centres[:,1])**2
    radii=np.asarray([m['radius'] for m in model])
    error=distances/.06**2+((radius[:,None]-radii)/.015)**2
    associations=np.argmin(error,axis=2)
    for index in range(len(model)):
        support=np.where(associations==index,np.exp(-.5*error[:,:,index]),0).max(axis=1)
        scores+=.5*np.log1p(4*support)
    return scores


def short_lines(mask, long_lines, lengths, *, paint_width=.05):
    if not lengths:
        return []
    min_px=max(6,round(min(lengths)*180*.65))
    found=cv2.HoughLinesP(cv2.Canny(mask,70,150),1,np.pi/360,
                          max(6,min_px//2),minLineLength=min_px,
                          maxLineGap=max(2,round(paint_width*180)+3))
    if found is None:
        return []
    uv=found.reshape(-1,2,2)
    rows=np.stack((4-uv[:,:,1]/180,2-uv[:,:,0]/180),axis=-1)
    accepted=[]
    for row in rows:
        v=row[1]-row[0];length=np.linalg.norm(v)
        if length<=0 or not any(.65*k<=length<=1.35*k for k in lengths):
            continue
        direction=v/length;middle=row.mean(axis=0)
        duplicate=False
        for existing in list(long_lines)+accepted:
            d=existing[1]-existing[0];n=np.linalg.norm(d)
            if n<=0:continue
            if abs(direction[0]*d[1]/n-direction[1]*d[0]/n)<.15:
                t=np.clip(np.dot(middle-existing[0],d)/(n*n),0,1)
                if np.linalg.norm(middle-(existing[0]+t*d))<.07:
                    duplicate=True;break
        if not duplicate:accepted.append(row)
        if len(accepted)>=8:break
    return accepted
