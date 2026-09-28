"""Pure field observations and robust measurement likelihood; no hardware access."""
import cv2
import numpy as np


def runtime_paint_mask(image, parameters=None):
    if image.shape != (650, 800, 3) or image.dtype != np.uint8:
        raise ValueError('Expected uint8 800x650 BGR capture')
    reduced = cv2.resize(image, (400, 325), interpolation=cv2.INTER_AREA)
    if parameters is not None and not parameters.get('vision.field_auto',True):
        from .colour_masks import mask
        lab=cv2.cvtColor(reduced,cv2.COLOR_BGR2LAB)
        green=mask(lab,parameters,'green_field')
        joined=cv2.morphologyEx(green,cv2.MORPH_CLOSE,np.ones((17,17),np.uint8))
        contours,_=cv2.findContours(largest(joined),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        if not contours:raise ValueError('No turf in configured LAB range')
        roi=np.zeros(green.shape,np.uint8)
        cv2.fillConvexPoly(roi,cv2.convexHull(max(contours,key=cv2.contourArea)),255)
        white=mask(lab,parameters,'white_marking') & roi
    else:
        _, _, white, _ = segment(reduced,parameters)
    return cv2.resize(white, (800, 650), interpolation=cv2.INTER_NEAREST)

def clusters(values, count):
    cv2.setRNGSeed(17)
    sample=np.ascontiguousarray(values[::max(1,len(values)//30000)],np.float32)
    _,_,centres=cv2.kmeans(sample,count,None,(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,60,.1),3,cv2.KMEANS_PP_CENTERS)
    return centres


def classify(values, centres):
    # ||pixel||² is common to every centre; omit it. OpenCV evaluates the
    # remaining affine scores without a large HxWxclustersx3 temporary.
    matrix=np.column_stack((-2*centres,np.sum(centres*centres,axis=1))).astype(np.float32)
    scores=cv2.transform(np.ascontiguousarray(values.reshape(-1,1,3),dtype=np.float32),matrix)
    return np.argmin(scores.reshape(values.shape[:-1]+(len(centres),)),axis=-1)


def largest(mask):
    n,labels,stats,_=cv2.connectedComponentsWithStats(mask)
    if n<2:return np.zeros_like(mask)
    return (labels==1+np.argmax(stats[1:,cv2.CC_STAT_AREA])).astype(np.uint8)*255


def segment(image,parameters=None):
    h,w=image.shape[:2]
    lab=cv2.cvtColor(image,cv2.COLOR_BGR2LAB).astype(np.float32)
    centres=clusters(lab.reshape(-1,3),5)
    labels=classify(lab,centres)
    populations=np.bincount(labels.ravel(),minlength=5)
    if parameters is not None:
        from .colour_masks import bounds
        low,high=bounds(parameters,'green_field')
        possible=[i for i in range(5) if np.all(centres[i]>=low) and np.all(centres[i]<=high)
                  and populations[i]>h*w*.05]
    else:
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
    contrast=parameters.get('vision.field_auto_contrast',15) if parameters is not None else 15
    if local[white_id,0] < np.median(lab[green!=0,0])+contrast:
        raise ValueError('White paint not separated from turf')
    white=((classify(lab,local)==white_id)&(roi!=0)).astype(np.uint8)*255
    return interior,roi,white,{'turf_lab':centres[turf].tolist(),'roi_lab_centres':local.tolist(),'white_cluster':int(white_id),'method':'per-frame deterministic LAB clustering'}


def field_model(extra=False):
    rows = [[[-1.675, y], [1.675, y]] for y in (-1.175, 1.175)]
    rows += [[[x, -1.175], [x, 1.175]] for x in (-1.675, 0, 1.675)]
    if extra:
        rows += [[[x, -1.175], [x, 1.175]] for x in (-1.425, 1.425)]
    return np.asarray(rows, dtype=float)


def observations(mask, scale=1.):
    if scale not in (.5,1.):raise ValueError('Unsupported feature scale')
    source=mask if scale==1 else cv2.resize(mask,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
    found = cv2.HoughLinesP(cv2.Canny(source, 70, 150), 1, np.pi/720,
                            round(70*scale), minLineLength=round(130*scale), maxLineGap=round(15*scale))
    if found is None:
        raise ValueError('No line observations')
    if found.shape[-1] != 4:
        raise ValueError('Unexpected HoughLinesP layout')
    uv = found.reshape(-1, 2, 2).astype(float)/scale
    lines = np.stack((4-uv[..., 1]/180, 2-uv[..., 0]/180), axis=-1)
    lengths = np.linalg.norm(lines[:, 1]-lines[:, 0], axis=1)
    kept, descriptors = [], []
    for i in np.argsort(-lengths):
        line = lines[i]
        d = line[1]-line[0]
        angle = np.arctan2(d[1], d[0]) % np.pi
        normal = np.array([-np.sin(angle), np.cos(angle)])
        midpoint = line.mean(axis=0)
        duplicate = False
        for a, mid in descriptors:
            angle_error = abs((angle-a+np.pi/2) % np.pi-np.pi/2)
            if angle_error < np.deg2rad(6) and abs((midpoint-mid)@normal) < .09:
                duplicate = True
                break
        if not duplicate:
            kept.append(line)
            descriptors.append((angle, midpoint))
    return np.asarray(kept)


def detect_circle(mask, scale=1.):
    """Require near-complete paint coverage; do not assume a field radius."""
    if scale not in (.5,1.):raise ValueError('Unsupported feature scale')
    source=mask if scale==1 else cv2.resize(mask,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
    candidates = cv2.HoughCircles(cv2.GaussianBlur(source, (5, 5), 1),
        cv2.HOUGH_GRADIENT, 1, round(60*scale), param1=100,
        param2=22 if scale==1 else 14, minRadius=round(20*scale), maxRadius=round(110*scale))
    if candidates is None:
        return None
    angles = np.linspace(0, 2*np.pi, 120, endpoint=False)
    scored = []
    for x, y, radius in candidates.reshape(-1,3)/scale:
        coverage = np.zeros(len(angles), bool)
        for dr in (-3, -1, 0, 1, 3):
            xx = np.rint(x+(radius+dr)*np.cos(angles)).astype(int)
            yy = np.rint(y+(radius+dr)*np.sin(angles)).astype(int)
            inside = (xx>=0)&(yy>=0)&(xx<mask.shape[1])&(yy<mask.shape[0])
            coverage[inside] |= mask[yy[inside], xx[inside]]>128
        if coverage.mean() >= .85:
            scored.append((float(coverage.mean()), float(x), float(y), float(radius)))
    if not scored:
        return None
    scored.sort(reverse=True)
    # Ambiguous complete circles cannot silently be treated as the centre mark.
    if len(scored)>1 and scored[0][0]-scored[1][0]<.05:
        return None
    coverage, x, y, radius = scored[0]
    return dict(center_robot_m=[4-y/180, 2-x/180], radius_observed_m=radius/180,
                pixel_circle=[x, y, radius], angular_coverage=coverage)


def likelihood(particles, lines, model, circle=None, circles=None):
    """Each distinct segment supplies one robust likelihood, not 5 repeats."""
    particles = np.asarray(particles, dtype=float)
    lines = np.asarray(lines, dtype=float).reshape(-1, 2, 2)
    model = np.asarray(model, dtype=float).reshape(-1, 2, 2)
    if particles.ndim != 2 or particles.shape[1] != 3 or not len(particles):
        raise ValueError('Expected nonempty Nx3 particles')
    if not all(np.isfinite(a).all() for a in (particles, lines, model)):
        raise ValueError('Nonfinite geometry')
    if not len(model) or np.any(np.linalg.norm(model[:, 1]-model[:, 0], axis=1) < 1e-8):
        raise ValueError('Empty or degenerate map')
    if np.any(np.linalg.norm(lines[:, 1]-lines[:, 0], axis=1) < 1e-8):
        raise ValueError('Degenerate observation')
    c, s = np.cos(particles[:, 2]), np.sin(particles[:, 2])
    scores = np.zeros(len(particles))
    errors = []
    for line in lines:
        points = np.array([line[0]*(1-t)+line[1]*t for t in (0, .5, 1)])
        wx = c[:, None]*points[:, 0]-s[:, None]*points[:, 1]+particles[:, 0, None]
        wy = s[:, None]*points[:, 0]+c[:, None]*points[:, 1]+particles[:, 1, None]
        q = np.stack((wx, wy), axis=-1)
        angle = np.arctan2(*(line[1]-line[0])[::-1])+particles[:, 2]
        best = np.full(len(particles), np.inf)
        best_distance = best.copy()
        associated_model = model
        if circle is not None and circles is None:
            center = np.asarray(circle['center_robot_m'])
            direction = line[1]-line[0]
            normal = np.array([-direction[1], direction[0]])/np.linalg.norm(direction)
            # A long straight passing through the detected circle's centre is
            # the halfway line, not an arbitrary parallel boundary.
            if abs((center-line[0])@normal)<.10 and np.linalg.norm(direction)>.8:
                associated_model = np.array([[[0, -1.175], [0, 1.175]]])
        for a, b in associated_model:
            v = b-a
            t = np.clip(((q-a)*v).sum(axis=-1)/(v@v), 0, 1)
            dist = np.sqrt(((q-a-t[..., None]*v)**2).sum(axis=-1)).mean(axis=1)
            ae = (angle-np.arctan2(v[1], v[0])+np.pi/2) % np.pi-np.pi/2
            e = (dist/.10)**2+(ae/np.deg2rad(8))**2
            improve = e < best
            best_distance[improve] = dist[improve]
            best = np.minimum(best, e)
        # Uniform outlier component prevents one spurious edge annihilating a pose.
        # Short fragments (including possible circle chords) should not outweigh
        # long boundaries simply because Hough produced more of them.
        reliability = np.clip(np.linalg.norm(line[1]-line[0])/1.5, .2, 1.)
        scores += reliability*np.log(.18+.82*np.exp(-.5*best))
        errors.append(best_distance)
    if circle is not None:
        x, y = circle['center_robot_m']
        cx = c*x-s*y+particles[:, 0]
        cy = s*x+c*y+particles[:, 1]
        # Circle centre is one 2D landmark; radius is diagnostic only because
        # the physical radius of this particular field has not been confirmed.
        if circles is None:
            circles=[{'center':[0.,0.],'radius':None}]
        # Marginalize ambiguous ring associations rather than forcing every
        # observed ring onto the centre spot. Radius contributes only if known.
        terms=[]
        for landmark in circles:
            mx,my=landmark['center']
            error=((cx-mx)**2+(cy-my)**2)/.10**2
            if landmark['radius'] is not None and 'radius_observed_m' in circle:
                error=error+((circle['radius_observed_m']-landmark['radius'])/.06)**2
            terms.append(np.exp(-.5*error))
        if terms:scores += np.log(.02+.98*np.mean(terms,axis=0))
    return scores, np.asarray(errors).T if len(lines) else np.empty((len(particles), 0))


def update(particles, lines, model, circle=None, circles=None):
    logw, errors = likelihood(particles, lines, model, circle, circles)
    weights = np.exp(logw-logw.max())
    weights /= weights.sum()
    return weights, errors
