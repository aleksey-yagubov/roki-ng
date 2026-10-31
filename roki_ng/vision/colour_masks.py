"""Shared LAB units and thresholds for tuning and localization."""
import math


def bounds(parameters,profile):
    prefix=f'vision.{profile}.'
    return ((int(parameters[prefix+'l_min']*2.55),parameters[prefix+'a_min']+128,parameters[prefix+'b_min']+128),
            (math.ceil(parameters[prefix+'l_max']*2.55),parameters[prefix+'a_max']+128,parameters[prefix+'b_max']+128))


def mask(lab,parameters,profile):
    import cv2
    return cv2.inRange(lab,*bounds(parameters,profile))
