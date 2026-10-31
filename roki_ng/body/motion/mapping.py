"""Logical joint angles -> physical servos; independent of transport and IK."""
from types import SimpleNamespace


class JointMapping:
    def __init__(self, split_knee):
        self.split_knee = split_knee

    def targets(self, model, angles, *, scale=1., keep_hands_up=False,
                legs_only=False, extra_first=True):
        values = []
        for index, angle in enumerate(angles):
            servo, bus, sign, *_ = model.ACTIVESERVOS[index]
            if keep_hands_up and index in model.hand_joints:
                continue
            if legs_only and not 5 <= servo <= 10:
                continue
            split = self.split_knee and servo == 8
            # Preserve legacy multiplication and integer truncation exactly.
            value = int(angle * scale * sign / (2 if split else 1) + 7500)
            ids = ((13, servo) if extra_first else (servo, 13)) if split else (servo,)
            values.extend(SimpleNamespace(Id=i, Sio=bus, Data=value) for i in ids)
        return values

    def joints(self, model):
        result = {row[6].removeprefix('MASK_').lower(): (row[0], row[1])
                  for row in model.ACTIVESERVOS}
        if self.split_knee:
            result.update(right_knee_bot=(13, 1), left_knee_bot=(13, 2))
        return result
