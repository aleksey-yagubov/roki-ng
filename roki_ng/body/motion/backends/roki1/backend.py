"""Roki 1 differences from the common Starkit gait, not a second runtime."""
from pathlib import Path
from .model import Robot
from .defaults import MOTION_DEFAULTS
from .primitives import jump_rows, splits

NAME = 'roki1'
ASSETS = Path(__file__).parent
RECOVERY = {'stomach': 'Roki_1_Get_UP_Stomach', 'back': 'Roki_1_Get_UP_Face_Up',
            'left': 'Get_Up_Left', 'right': 'Get_Up_Right'}
# The original 233 mm target exceeds this model's 232 mm vertical chain.
# Keep 2 mm extension margin: 231 mm fails with the original 16 mm forward
# foot offset after a long stride. Preserve that offset and the original exit.
# This is a reachable nominal pose, not a field stability calibration.
STAND_HEIGHT = 230.
LEAD_STRIDE_SCALE = 2.5
ZERO_FIRST_RIGHT = True
HALF_LIFT = False
COSINE_SWAY = True
KICK_OFFSET = False  # The supplied software kick has no offset argument.
CONTROLLER_KICK = False  # Slot 31 contract is only verified for roki2met.
STABILIZATION = False  # Roki2 ID7 crouch tuning has not been validated on Roki1.
