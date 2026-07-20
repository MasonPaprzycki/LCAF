# Simple Profile Slicer

`lcaf.toolpathing` is the initial computational-geometry toolpath generator.
It creates the JSONL format consumed by `ForgeBrain.load_jsonl()` without
invoking FEM, MPM, or a surrogate model.

The scope is intentionally narrow: a watertight triangular target mesh is
sliced along its longest axis (or a selected axis).  Every slice is reduced to
its convex radial support envelope.  A cylindrical billet is then constrained
by planar die strikes at 90 degree increments (or a user-selected increment).
For a square bar, those four orientations correspond to the +Z, +Y, -Z, and
-Y faces.  Each orientation makes a serpentine X sweep; a required reduction
larger than the configured maximum is split into multiple strikes.

## Inputs

- **OBJ** and both ASCII and binary **STL** are supported without a third-party
  dependency.  Input coordinates are interpreted in millimetres by default.
  OBJ has no unit metadata, so use `--scale 25.4` for inch-based geometry.
- A native SolidWorks **`.sldprt`** is a proprietary feature-history format and
  is deliberately not guessed or partially parsed. Export it as a watertight
  STL (recommended) or OBJ from SolidWorks/FreeCAD, then import the exported
  mesh. This retains a deterministic, testable geometry boundary.

## Output contract

Each line is a complete `ToolpathOperation` accepted by the existing loader:

```json
{"step": 1, "operation": "STRIKE", "x": -10.0, "y": 0.0, "die_gap": 4.0, "rotation": 90.0, "target_temperature": 1100.0, "metadata": {"generator": "lcaf.profile_slicer"}}
```

`MotionCoordinator` already expands every operation into its safe sequence:
retract Z, retract X/Y, rotate A, move X/Y, then move Z. `die_gap` is therefore
the final **machine Z coordinate**, not an inferred physical gap.

## Run it

From the repository root in PowerShell:

```powershell
$env:PYTHONPATH = (Resolve-Path .\src)
python -m lcaf.toolpathing.ui
```

The UI includes supplied OBJ examples and a command-by-command, animated
die-envelope playback. See [the UI guide](profile_slicer_ui_guide.md) for the
first-run workflow and the meaning of every control.

Or export directly:

```powershell
$env:PYTHONPATH = (Resolve-Path .\src)
python -m lcaf.toolpathing target.stl output\target.jsonl --stock-radius 20 --axial-resolution 5 --rotation-step 90 --max-reduction 2 --die-contact-z 10
```

## Constraints and safety checks

The target must fit inside the starting cylinder at every generated die
orientation. The planner rejects target regions that require adding material,
excess machine X/Y/Z travel, and rotations outside the configured limits in
`configs/forge_parameters.json`.

The current LCAF configuration allows only A = -90 to +90 degrees. A complete
four-face 0/90/180/270 degree plan is consequently rejected by default:
`MotionCoordinator` commands **absolute** A positions and has no re-clamp or
continuous-rotation state. Do not bypass this check unless the rotary hardware
and its LinuxCNC limits have been proven to support those absolute positions.
The UI and CLI expose an explicit override for that setup-only case.

This model is a geometric envelope planner, not a forming simulation. It does
not predict volume flow, flash, springback, temperature effects, die compliance,
or tooling collisions. Its generated Z values depend on the fixture-specific
`die_contact_z_mm` calibration. Review and prove every program off-material
before loading it into the machine controller.
