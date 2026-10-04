"""Three independent exposures in one stationary window; never a motion gate."""
import math
from .calibration import wrap


class CorrectionWindow:
    def __init__(self, ident, after_ns):
        self.ident, self.after_ns = ident, after_ns
        self.sequence = self.stamp = -1
        self.revision = None
        self.poses = []
        self.accepted = False

    def observe(self, state, now_ns):
        r = state.get('result') or {}
        if self.accepted or r.get('window_id') != self.ident:
            return None
        seq, stamp = r.get('frame_sequence'), r.get('sensor_timestamp_ns')
        if type(seq) is not int or type(stamp) is not int or seq <= self.sequence or stamp <= self.stamp:
            return None
        if self.stamp >= 0 and stamp-self.stamp > 1_000_000_000:
            self.poses.clear()
        self.sequence, self.stamp = seq, stamp
        revision = r.get('parameter_revision')
        if revision != self.revision:
            self.poses.clear()
            self.revision = revision
        pose = r.get('candidate')
        if (not state.get('running') or state.get('error') or not r.get('valid')
                or r.get('fit_state') != 'matched' or r.get('ambiguous')
                or stamp <= self.after_ns or not 0 <= now_ns-stamp <= 500_000_000
                or not isinstance(pose, list) or len(pose) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose)):
            self.poses.clear()
            return None
        # No chain drift: each of the three must agree with every other one.
        if any(math.hypot(pose[0]-p[0],pose[1]-p[1])>.12 or abs(wrap(pose[2]-p[2]))>.12
               for p in self.poses):
            self.poses.clear()
        self.poses.append(pose)
        if len(self.poses) < 3:
            return None
        self.accepted = True
        return [sum(p[i] for p in self.poses)/3 for i in (0,1)] + [
            math.atan2(sum(math.sin(p[2]) for p in self.poses),sum(math.cos(p[2]) for p in self.poses))]
