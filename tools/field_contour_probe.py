"""Offline field/white-line candidate extraction; no metric pose estimate."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def extract(image):
    """Return conservative turf ROI and white candidates, including curved lines."""
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    # OpenCV LAB: a/b are offset by 128. Require distinctly green turf.
    green = cv2.inRange(lab, (20, 0, 105), (240, 116, 210))
    green = cv2.morphologyEx(green, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    connected = cv2.morphologyEx(green, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31)))
    contours, _ = cv2.findContours(connected, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi = np.zeros(green.shape, np.uint8)
    lines = roi.copy()
    if not contours:
        return roi, lines, {'valid': False, 'reason': 'no_turf'}
    main = max(contours, key=cv2.contourArea)
    if cv2.contourArea(main) < image.shape[0]*image.shape[1]*0.05:
        return roi, lines, {'valid': False, 'reason': 'insufficient_turf'}
    hull = cv2.convexHull(main)
    cv2.fillConvexPoly(roi, hull, 255)
    # Keep bordering white paint instead of intersecting white and green pixels.
    roi = cv2.dilate(roi, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31)))
    near_green = cv2.dilate(green, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)))
    white = cv2.inRange(lab, (145, 113, 113), (255, 143, 143))
    candidate = cv2.bitwise_and(cv2.bitwise_and(white, roi), near_green)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(candidate)
    components = []
    for i in range(1, count):
        x,y,w,h,area = map(int, stats[i])
        if area < 80 or max(w,h) < 25:
            continue
        lines[labels == i] = 255
        components.append({'box': [x,y,w,h], 'pixels': area})
    return roi, lines, {'valid': bool(components), 'pose_valid': False,
                        'roi_pixels': int(np.count_nonzero(roi)),
                        'line_pixels': int(np.count_nonzero(lines)),
                        'components': components,
                        'limitations': ['uncalibrated image coordinates',
                                        'candidate ROI; not certified playing boundary',
                                        'white objects on turf can be false positives']}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('output', type=Path)
    args=parser.parse_args()
    im=cv2.imread(str(args.image))
    if im is None: raise SystemExit('Cannot read image')
    roi,lines,report=extract(im)
    args.output.mkdir(parents=True,exist_ok=True)
    overlay=im.copy()
    overlay[roi == 0]=(overlay[roi == 0]*0.2).astype(np.uint8)
    overlay[lines != 0]=(255,0,255)
    contours,_=cv2.findContours(roi,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(overlay,contours,-1,(255,255,0),2)
    for name,a in [('overlay.png',overlay),('field-mask.png',roi),('lines-mask.png',lines)]:
        if not cv2.imwrite(str(args.output/name),a):raise RuntimeError(name)
    report['source']=str(args.image.resolve())
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))

if __name__=='__main__':main()
