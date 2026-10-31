"""Model-specific movement assets."""
import json
from pathlib import Path
from functools import partial
from roki_ng.body.motion.primitives import jump_rows as scale_jump

JUMPS = json.loads((Path(__file__).with_name("jumps.json")).read_text())
jump_rows = partial(scale_jump, JUMPS)

def splits(big):
    from roki_ng.wire import Fault
    raise Fault("unsupported", "Splits are not ported for roki1")
