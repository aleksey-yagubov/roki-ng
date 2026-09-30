"""Render frame-matched observations for the existing stream worker."""
import cv2
import numpy as np

def video_frame(image,projector,quaternion,lines,circle,posts,result,model,circles=None):
    """Burn observations onto exactly the frame consumed by localization."""
    from .field_observations import likelihood
    out=image.copy();errors=None
    if result.get('candidate') is not None and len(lines):
        _,residuals=likelihood(np.asarray([result['candidate']]),lines,model,circle,circles)
        errors=residuals[0]
    def curve(points,colour):
        uv,valid=projector.image_points(points,quaternion)
        for i in range(len(uv)-1):
            if valid[i] and valid[i+1]:cv2.line(out,tuple(np.rint(uv[i]).astype(int)),tuple(np.rint(uv[i+1]).astype(int)),colour,3)
    for i,line in enumerate(lines):
        colour=(160,160,160) if errors is None else ((40,230,40) if errors[i]<.1 else (0,150,255))
        curve(np.linspace(line[0],line[1],32),colour)
    if circle:
        angles=np.linspace(0,2*np.pi,100)
        centre=np.asarray(circle['center_robot_m']);r=circle['radius_observed_m']
        curve(centre+r*np.column_stack((np.cos(angles),np.sin(angles))),(255,0,255))
    for p in posts:
        x,y,w,h=p['rect'];colour=(255,140,20) if p['colour']=='blue' else (0,230,255)
        cv2.rectangle(out,(x,y),(x+w,y+h),colour,3)
        cv2.putText(out,p['colour'],(x,max(18,y-5)),cv2.FONT_HERSHEY_SIMPLEX,.6,colour,2)
    cv2.rectangle(out,(0,0),(800,65),(20,20,20),-1)
    cv2.putText(out,f"Frame {result['frame_sequence']} | {result.get('fit_state','no fit')} | lines {len(lines)} | goals {len(posts)} rejected {result.get('goal_rejected',0)}",(8,24),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
    cv2.putText(out,str(result.get('reason',''))[:100],(8,49),cv2.FONT_HERSHEY_SIMPLEX,.46,(230,230,230),1)
    return out
