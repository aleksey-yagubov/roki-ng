# Roki1: original motion and reachable final stance

By operator decision, keep original walking/kicking trajectories, including
skipping an IK frame with a warning and continuing. No automatic lift reduction,
preflight replay or additional foot centring is applied. Servo limits are not
expanded. Missing IK frames remain a known limitation of the original motion.

Final IK height is 230 mm. At 231 mm, the retained 16 mm forward offset after
a long stride made the last frame unreachable. The original interpolated exit
preserves X coordinates and reaches all frames in the tested walk/kick scenarios.
No additional base_stand slot is appended to stand or kick completion. Existing
explicit autonomous base_stand requests remain unchanged. Roki2 is unchanged.

Actual balance and transition to straight legs require physical verification.
