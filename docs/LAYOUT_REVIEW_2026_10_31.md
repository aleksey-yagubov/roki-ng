# Layout and model camera review

## Final pre-publication check

`PYTHONPATH=../starkit python3 -m pytest -q --tb=no`: **842 passed,
5 skipped, 15 failed**. The failing tests are the same known set documented in
COMMIT_REVIEW_2026_10_05.md. The complete suite is not green.
`git diff --cached --check` is clean after normalising whitespace in imported
Roki1 resources (motion values unchanged). GUI and robot deployment are excluded.

The commits separate mechanical module relocation, backend/model integration,
Roki1 final stance, and documentation. The model selection and camera geometry
changes share a source of model metadata and are committed together.

The figures below describe earlier stages of the refactor, not the final run.

Runtime modules are grouped by workers. Body runtime and motion now live in
`roki_ng/body`, while the existing motherboard process still owns the physical
ACM connection. This does not introduce USB support or a new protocol.
See BODY_ARCHITECTURE.md for remaining process-boundary work.

Both models use the same operator parameter names. Gait defaults differ by backend;
measured displacement calibrations from Real_params are not substituted.
The original measured camera geometry is
a nominal model baseline, not proof of accuracy for a particular assembled robot.
No robot deployment or field calibration was performed.

Validation:

- Full suite: 704 passed, 125 skipped, 15 failed. Same failing tests as recorded
  in COMMIT_REVIEW_2026_10_05.md; not a fully green suite.
- Native Starkit gait/manual posture/calibration and model camera selection:
  148 passed.
- Wheel build succeeded; 55 Roki1 and 57 Roki2 slot resources included.
- GUI kick parameter editor and optional-height parsing passed.

Roki1 physical readiness is NOT established. Earlier native solver checks found
unreachable poses in the imported gait, including its 233 mm final stance.
Subsequent work selected a 230 mm nominal final stance with the original IK exit;
experimental lift adaptation was removed by operator request. See
ROKI1_REACHABILITY.md. Physical readiness still requires supported trials.
