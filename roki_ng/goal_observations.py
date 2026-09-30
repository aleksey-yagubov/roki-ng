"""Bounded coloured post candidates, not yet verified metric landmarks.

The bottom of a coloured component is not proof of a post/ground intersection.
Keep it diagnostic until the post model and field profile validate association.
"""
import cv2
import numpy as np


def goal_candidates(image,parameters=None):
    if image.shape!=(650,800,3):raise ValueError('Expected 800x650 BGR')
    from .parameters import SCHEMA
    from .colour_masks import mask as colour_mask
    if parameters is None:parameters={k:v[1] for k,v in SCHEMA.items()}
    lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB)
    green=colour_mask(lab,parameters,'green_field')!=0
    joined=cv2.morphologyEx(green.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((31,31),np.uint8))
    count,field_labels,field_stats,_=cv2.connectedComponentsWithStats(joined)
    if count<2:return []
    field_id=1+np.argmax(field_stats[1:,cv2.CC_STAT_AREA])
    if field_stats[field_id,cv2.CC_STAT_AREA]<image.shape[0]*image.shape[1]*.1:return []
    green&=field_labels==field_id
    results=[]
    for colour in ('blue','yellow'):
        mask=colour_mask(lab,parameters,colour+'_posts')
        vertical=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((17,3),np.uint8))
        n,labels,stats,_=cv2.connectedComponentsWithStats(vertical)
        candidates=[]
        for i in range(1,n):
            x,y,w,h,pixels=map(int,stats[i])
            if pixels<40 or h<20 or h<1.5*w or y+h>=649 or x<=0 or x+w>=800:continue
            foot_y=y+h-1
            xs=np.flatnonzero(np.any(labels[max(y,foot_y-3):foot_y+1]==i,axis=0))
            if not len(xs):continue
            foot_x=float(np.median(xs))
            # Support must be below the footprint, never borrowed from the side.
            # White boundary tape can separate the coloured base from turf.
            half=max(3,min(10,w//3))
            left,right=max(0,int(foot_x)-half),min(800,int(foot_x)+half+1)
            stop=min(650,foot_y+34)
            patch=green[foot_y+2:stop,left:right]
            white=colour_mask(lab[foot_y+2:stop,left:right],parameters,'white_marking')!=0
            if patch.shape[0]<16:continue
            # A gap is permitted only when it is paint, not arbitrary background.
            supported=False
            for offset in (0,8,16):
                turf=patch[offset:offset+16]
                if len(turf)<16:continue
                bridge=(patch|white)[:offset]
                if offset and np.mean(bridge)<.8:continue
                if turf.mean()>=.5 and np.mean(np.mean(turf,axis=0)>=.5)>=.6:
                    supported=True;break
            if not supported:continue
            candidates.append((pixels,{'colour':colour,'foot_px':[foot_x,foot_y],
                                      'rect':[x,y,w,h],'metric_valid':False}))
        candidates.sort(key=lambda item:item[0],reverse=True)
        results.extend(item[1] for item in candidates[:6])
    return results


def upright_candidates(candidates,projector,quaternion,goals,tolerance):
    """Reject small paint fragments using calibrated height, not pixel size."""
    accepted=[]
    for post in candidates:
        matches=[g for g in goals if g['colour']==post['colour']]
        if len(matches)!=1:continue
        x,y,w,h=post['rect']
        try:height=projector.upright_height(post['foot_px'],[x+w/2,y],quaternion)
        except ValueError:continue
        expected=matches[0]['height']
        if abs(height-expected)<=expected*tolerance:accepted.append(post)
    return accepted


def paired_bearings(candidates,projector,quaternion):
    """Require two spatially separate upright colour components, not one blob."""
    from itertools import combinations
    result=[]
    for colour in ('blue','yellow'):
        projected=[]
        for post in [p for p in candidates if p['colour']==colour][:6]:
            try:
                angle=projector.bearing(post['foot_px'],quaternion)
                foot=projector.ground_point(post['foot_px'],quaternion).tolist()
            except ValueError:continue
            projected.append((post,angle,foot))
        for (a,aa,af),(b,ba,bf) in combinations(projected,2):
            if abs(a['foot_px'][0]-b['foot_px'][0])<max(24,2*max(a['rect'][2],b['rect'][2])):continue
            separation=abs((aa-ba+np.pi)%(2*np.pi)-np.pi)
            if .08<=separation<=1.5:
                result.append({'colour':colour,'bearings':[aa,ba],'ground_feet':[af,bf]})
    return result


def bearing_log_likelihood(particles,observations,goals,max_foot_error=.35):
    """Score bearings only where projected feet agree with mapped goal posts."""
    scores_by_colour={}
    for observation in observations:
        matches=[g for g in goals if g['colour']==observation['colour']]
        if len(matches)!=1:continue
        if 'ground_feet' not in observation:continue
        feet=np.asarray(observation['ground_feet'],float)
        if feet.shape!=(2,2) or not np.isfinite(feet).all():continue
        g=matches[0];angles=np.asarray(observation['bearings'],float)
        if angles.shape!=(2,) or not np.isfinite(angles).all():continue
        if abs((angles[0]-angles[1]+np.pi)%(2*np.pi)-np.pi)<.08:continue
        endpoints=np.array([[g['x'],g['y']-g['width']/2],[g['x'],g['y']+g['width']/2]])
        delta=endpoints[None,:,:]-particles[:,None,:2]
        predicted=np.arctan2(delta[:,:,1],delta[:,:,0])-particles[:,2,None]
        c,s=np.cos(particles[:,2]),np.sin(particles[:,2])
        world=np.stack((c[:,None]*feet[:,0]-s[:,None]*feet[:,1]+particles[:,0,None],
                        s[:,None]*feet[:,0]+c[:,None]*feet[:,1]+particles[:,1,None]),axis=-1)
        errors=[]
        for target,endpoint_order in ((angles,endpoints),(angles[::-1],endpoints[::-1])):
            residual=(predicted-target+np.pi)%(2*np.pi)-np.pi
            error=np.mean(residual**2,axis=1)
            foot_error=np.linalg.norm(world-endpoint_order[None,:,:],axis=-1).max(axis=1)
            error=np.where(foot_error<=max_foot_error,error,np.inf)
            errors.append(error)
        best=np.minimum(*errors)
        if not np.isfinite(best).any():continue
        # Allow uncertain width/extrinsics; contradictory pairs retain outliers.
        score=np.log(.005+.995*np.exp(-.5*best/.15**2))
        colour=observation['colour']
        # Alternative associations of the same goal are not independent evidence.
        if colour in scores_by_colour:score=np.maximum(scores_by_colour[colour],score)
        scores_by_colour[colour]=score
    return sum(scores_by_colour.values(),np.zeros(len(particles))),len(scores_by_colour)


def mapped_candidates(candidates,projector,quaternion,result,goals,tolerance):
    """Show goals only after a matched pose supports their ground contact.

    Goal uprights are above the floor and on its boundary: clipping their whole
    image rectangle to the green mask would remove genuine goals.
    """
    if result.get('fit_state')!='matched' or result.get('candidate') is None:return []
    x,y,yaw=result['candidate'];c,s=np.cos(yaw),np.sin(yaw)
    accepted=[]
    for post in candidates:
        matches=[g for g in goals if g['colour']==post['colour']]
        if len(matches)!=1:continue
        try:foot=projector.ground_point(post['foot_px'],quaternion)
        except ValueError:continue
        world=np.array([x+c*foot[0]-s*foot[1],y+s*foot[0]+c*foot[1]])
        g=matches[0]
        ends=np.array([[g['x'],g['y']-g['width']/2],[g['x'],g['y']+g['width']/2]])
        if np.min(np.linalg.norm(ends-world,axis=1))<=tolerance:accepted.append(post)
    # Keep the existing telemetry bound: at most two posts per colour.
    return [p for colour in ('blue','yellow') for p in [p for p in accepted if p['colour']==colour][:2]]
