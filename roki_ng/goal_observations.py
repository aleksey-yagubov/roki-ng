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
            if pixels<40 or h<20 or h<1.5*w or y+h>=649:continue
            foot_y=y+h-1
            xs=np.flatnonzero(np.any(labels[max(y,foot_y-3):foot_y+1]==i,axis=0))
            if not len(xs):continue
            foot_x=float(np.median(xs))
            left,right=max(0,int(foot_x)-20),min(800,int(foot_x)+21)
            patch=green[max(0,foot_y-8):min(650,foot_y+16),left:right]
            if not patch.size or patch.mean()<.15:continue
            candidates.append((pixels,{'colour':colour,'foot_px':[foot_x,foot_y],
                                      'rect':[x,y,w,h],'metric_valid':False}))
        candidates.sort(key=lambda item:item[0],reverse=True)
        results.extend(item[1] for item in candidates[:2])
    return results
