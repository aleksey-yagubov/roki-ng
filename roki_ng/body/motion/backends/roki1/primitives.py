"""Model-specific movement assets."""
import json
from pathlib import Path
from functools import partial
from roki_ng.body.motion.primitives import jump_rows as scale_jump

JUMPS = json.loads((Path(__file__).with_name("jumps.json")).read_text())
jump_rows = partial(scale_jump, JUMPS)


def pult_turn_steps(model, baseline, direction, fraction):
    """walk_pult turn(factor=10): 40 ms impulse, 40 ms return, 60 ms rest.

    Preserve a manual crouch by applying the source impulse around its targets.
    At neutral joint targets this is identical to the controller script.
    """
    from types import SimpleNamespace
    from roki_ng.body.motion import slots
    from roki_ng.wire import Fault
    sign = 1 if direction == 'turn_right' else -1
    amplitude = 2000 * fraction
    offsets = {'right_pelvic': -sign*amplitude, 'left_pelvic': sign*amplitude,
               'right_foot_side': amplitude, 'left_foot_side': amplitude}
    joints = slots.joints(model)
    impulse, restore = [], []
    for name, offset in offsets.items():
        servo, bus = joints[name]
        if (servo, bus) not in baseline:
            raise Fault('invalid_state', 'Missing turn baseline; select standing or crouch pose first')
        start = baseline[servo, bus]
        target = start + slots.position(offset, 'zubr', name) - 7500
        if not 0 <= target <= 16383:
            raise Fault('invalid_motion', 'Pult turn exceeds servo protocol range')
        impulse.append(SimpleNamespace(Id=servo, Sio=bus, Data=target))
        restore.append(SimpleNamespace(Id=servo, Sio=bus, Data=start))
    # The caller confirms completion once, then supplies the source rest time.
    return [('servo', impulse, 2, 1), ('servo', restore, 2, 1)]

def splits(big):
    from roki_ng.wire import Fault
    raise Fault("unsupported", "Splits are not ported for roki1")
