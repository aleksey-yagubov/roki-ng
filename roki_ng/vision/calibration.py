"""Select one complete optical calibration; never copy or mix profiles."""
import json
from pathlib import Path

FILES = ('profile.json', 'Camera_calibration_P.npy',
         'Camera_calibration_map1.npy', 'Camera_calibration_map2.npy')
DEFAULT_DIRECTORY = Path(__file__).parent / 'calibration_default'


def calibration_directory(individual):
    individual = Path(individual)
    directory = individual if any((individual / name).exists() for name in FILES) else DEFAULT_DIRECTORY
    missing = [name for name in FILES if not (directory / name).is_file()]
    if missing:
        raise ValueError(f'Incomplete camera calibration in {directory}: missing {", ".join(missing)}')
    profile = json.loads((directory / 'profile.json').read_text())
    if profile.get('schema') != 1 or profile.get('capture_size') != [800, 650]:
        raise ValueError(f'Incompatible camera calibration profile: {directory}')
    return directory
