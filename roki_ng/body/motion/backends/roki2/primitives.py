"""Model-specific movement assets."""
import json
from pathlib import Path
from functools import partial
from roki_ng.body.motion.primitives import jump_rows as scale_jump

JUMPS = json.loads((Path(__file__).with_name("jumps.json")).read_text())
jump_rows = partial(scale_jump, JUMPS)

def splits(big):
    """ROKI2 splits entry, retaining individual lower knees; no automatic exit."""
    def frame(duration, **targets):
        return {"duration": duration, "targets": {
            f"{side}_{joint}": value for joint, value in targets.items()
            for side in ("right", "left")}}
    rows = [frame(80, pelvic=0, hip_side=0, hip=3000, knee=3600,
                  knee_bot=3600, foot_front=4100, foot_side=0),
            frame(9, foot_side=3000),
            frame(10, pelvic=0, hip_side=1400, hip=3000, knee=3600,
                  knee_bot=3600, foot_front=4100, foot_side=1400)]
    if big:
        rows.extend([frame(9, foot_side=3000),
                     frame(40, hip_side=4200, foot_side=1500, clavicle=2800, shoulder=0),
                     frame(40, hip=0, knee=0, knee_bot=0, foot_front=0)])
    return {"units": "zubr", "joint_space": "individual", "frames": rows}
