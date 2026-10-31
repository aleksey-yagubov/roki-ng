"""Camera model constants; not operator calibration parameters."""
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraGeometry:
    height_m: float
    neck_m: float = .03676
    reference_tilt: int = -1500
    forward_m: float = 0.


def height(parameters, geometry):
    custom = parameters.get('camera.custom_height_m', 0.)
    return geometry.height_m if custom == 0 else custom
