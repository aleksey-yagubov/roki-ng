"""Existing Roki 2 geometry and movements, unchanged defaults."""
from pathlib import Path
from .model import Robot
from .defaults import MOTION_DEFAULTS
from .primitives import jump_rows, splits

NAME = 'roki2'
ASSETS = Path(__file__).parent
RECOVERY = {'stomach': 'Roki_2_Get_UP_Stomach', 'back': 'Roki_2_Get_UP_Face_Up',
            'left': 'Get_Up_Left', 'right': 'Get_Up_Right'}
STAND_HEIGHT = 215.
LEAD_STRIDE_SCALE = 1.5
ZERO_FIRST_RIGHT = False
HALF_LIFT = True
COSINE_SWAY = False
KICK_OFFSET = True
CONTROLLER_KICK = True
STABILIZATION = True
