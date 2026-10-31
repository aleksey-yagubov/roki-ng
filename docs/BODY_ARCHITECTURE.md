# Body and motherboard

`body/runtime.py` owns motion jobs, cancellation, recovery and body state.
`body/motion/backends/{roki1,roki2}` contains mechanical models and motion assets.
Operator parameters use shared defaults overlaid by each backend's defaults.py,
then saved values. Backend defaults describe nominal gait, not measured displacement.
Model camera constants are separate from these parameters.

`body/communication/transports/motherboard.py` is the existing adapter through
the Roki library and motherboard ACM. The library still implements the protocol;
`communication/protocols/` reserves its future home, not a new implementation.

The motherboard worker currently runs the body runtime in the same process,
retaining a single ACM owner. This is a code layout boundary, not yet independent
processes. Motherboard connection/polling orchestration is still coupled to the
runtime and must be separated before launching a USB body worker.

## TODO

- Add modified IronArt codec and direct USB transport when their contracts are
  available. Preserve queue completion and uncertain-execution semantics.
- Run a separate body worker for direct USB. Motherboard remains the IMU source;
  pass timestamped measurements to the body worker, with freshness checks.
- Replace the reference camera pose formula with head-axis geometry and servo
  feedback, then include leg pose and body IMU for height above the floor.
  Do not implement this as part of the directory refactor.
- Verify Roki1 gait reachability and physical movement with support before use.
- Roki1: not all original walking frames have valid IK solutions at the current
  geometry, limits and gait settings; software kick also has skipped frames.
  Current behavior deliberately matches the original: warn, skip the frame,
  continue. The 230 mm final-stance fix does NOT resolve these intermediate
  frames. Investigate solver branch selection and trajectories, then validate
  on the actual robot. Do not silently widen servo limits or mark this field-ready.

## Model selection

`--robot-model roki1` or `--robot-model roki2` explicitly selects an installed
backend. Without the argument, the sole installed backend is selected; with both
installed, Roki2 is selected. No installed backend, a missing explicitly selected
backend, or a broken backend is a startup error, never a silent fallback.
For a single-model deployment, omit the unwanted backend directory from the
deployment copy, not from the development repository.

Model selection is not an operator parameter. The state directory is bound to its
model; reusing it for a different model fails. On first startup effective defaults
are saved; subsequent startups preserve operator values. `params.describe.default`
returns the selected model's default for the GUI reset button. Missing keys use
current effective defaults.

Internal motion engines require an explicit backend; they must never repeat
automatic model selection. Calibration TestPlan uses the owning body's backend
and log sink, just like manual/game movement. This prevents an explicitly selected
Roki1 from accidentally using Roki2 geometry when both backend folders are present.
Camera projection helpers likewise require the selected model geometry explicitly.

Roki1 retains original trajectories and IK-frame skipping, as requested. Standing
uses the original interpolated exit at 230 mm; see ROKI1_REACHABILITY.md. Physical stability
and kick contact remain unverified.

## Camera height

The model holds nominal height, neck geometry, reference tilt and forward offset.
Both detection and localisation use the model height. The optional shared
`camera.custom_height_m` overrides optical centre height above ground for a head
mounted on a stand. Default `0` uses the model. Stop motion before changing it
and restart detection/localisation. Set 0 or restore its default to
remove the override. Old game/localisation camera geometry keys are no longer
operator parameters; there are no compatibility aliases.
