# Simple Toolpath Slicer

`lcaf.toolpathing` is the initial computational-geometry toolpath generator.
It creates the JSONL format consumed by `ForgeBrain.load_jsonl()` without
invoking FEM, MPM, or a surrogate model.

The scope is intentionally narrow: a watertight triangular target mesh's
longitudinal extent is divided into `radial_segments` axial regions. Each
region's strike depth is not sampled at one point -- it is the *numerical
average* of the target's true cross-section across that whole region,
integrated from several sub-samples along its span, so a region spanning a
taper is struck with something resembling the mean of its two ends, not
either end alone. Each region is struck `strikes_per_segment` times, at
rotations evenly spaced across the full 360 degrees; for a square bar,
`strikes_per_segment=4` corresponds to the +Z, +Y, -Z, and -Y faces. The
entire region-by-strike sweep can be repeated `cycles` times end-to-end --
because every operation just replays, in order, against one running material
state, a later cycle automatically trues up whatever an earlier, rougher
cycle left rough, without any special cycle-aware logic. A required
reduction larger than the configured maximum per strike is split into
multiple retract/re-strike depth passes. This planner only prescribes the
coordinates each strike needs -- it does not choose an execution order or
optimise travel between strikes. Sequencing that set of strikes is entirely
the control system's responsibility, and does not change which strikes are
generated.

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
{"step": 1, "operation": "STRIKE", "x": -10.0, "y": 0.0, "die_gap": 4.0, "rotation": 90.0, "target_temperature": 1100.0, "metadata": {"generator": "lcaf.toolpath_slicer"}}
```

`MotionCoordinator` already expands every operation into its safe sequence:
retract Z, retract X/Y, rotate A, move X/Y, then move Z. `die_gap` is therefore
the final **machine Z coordinate**, not an inferred physical gap.

## Machine axis convention

- **X** is the billet's long axis. Positive X points outward, away from the
  clamped end, toward the free/forged end. `stock_clamped_end` (`"min"` or
  `"max"`, see below) tells the planner which end of the *mesh's* bounding box
  is actually clamped, since a source mesh may be authored either way; every
  generated X is oriented so increasing X always means "away from the clamp."
- **Y** is the radial axis the lower die -- the one whose geometry is
  configured -- lies along.
- **Z** is the radial axis the upper die (a flat-faced circular disc) travels
  along: positive Z drives it into the billet, negative Z retracts it. In
  real life the lower die never moves; it is the billet's X/Y/rotation and
  the upper die's Z that do. The generated toolpath reflects that: it
  commands billet X/Y/rotation and a Z (`die_gap`) for the upper die.
- The billet -- not the dies -- rotates about X between strikes. `rotation`
  selects which local feature of the target is currently presented to the
  machine's fixed +Z strike direction.
- The UI's preview animation is a deliberate exception to this: for clarity,
  it always shows only the lower die -- the one whose rectangular geometry
  is configured in **3. Die geometry** -- moving through a stroke, with the
  upper die fixed as a translucent, sized circular disc always touching
  wherever the material actually currently sits on the opposite side (not a
  fixed pristine position, which could otherwise appear to float clear of
  the billet once an earlier rotation has already reduced that side). That
  matches neither die's real-life motion, but keeps the animation legible:
  the one visible geometry change always corresponds to the one visibly
  moving tool.

## Run it

From the repository root in PowerShell:

```powershell
$env:PYTHONPATH = (Resolve-Path .\src)
python -m lcaf.toolpathing.ui
```

The UI includes supplied OBJ examples and a command-by-command, animated
die-envelope playback. See [the UI guide](toolpath_slicer_ui_guide.md) for the
first-run workflow and the meaning of every control.

Or export directly:

```powershell
$env:PYTHONPATH = (Resolve-Path .\src)
python -m lcaf.toolpathing target.stl output\target.jsonl --stock-radius 20 --radial-segments 4 --strikes-per-segment 4 --max-reduction 2 --die-contact-z 10
```

Add `--cycles N` to repeat the whole region-by-strike sweep `N` times (a
real finishing pass, truing up whatever the first, rougher pass left rough),
or `--auto-cycles` to keep adding cycles automatically until the simulated
geometry matches the target within `--auto-cycles-tolerance` (default 0.5
mm), up to `--auto-cycles-max` (default 20) before giving up with a warning.

## Radial segments, strikes per segment, and cycles

- **`--radial-segments` / `radial_segments`** (default 4): how many axial
  regions the target's length is divided into. Each region's own strike
  depth is computed by numerically integrating (averaging) the target
  mesh's true cross-section across that region's whole span -- not sampled
  at a single point -- so a region spanning a taper is struck with
  something resembling the mean of its two ends. A target that is itself a
  regular polygon along its length (a hexagon needs 6, an octagon 8) needs
  exactly as many `strikes_per_segment` as it has flat faces to reproduce
  it exactly with a flat die; approximating a genuinely curved profile
  within one segment, rather than one flat facet per rotation, would in
  principle need infinitely many strikes per segment -- this planner always
  uses flat rectangular/circular dies, so a curved target is approximated
  by however many strikes you configure.
- **`--strikes-per-segment` / `strikes_per_segment`** (default 4): how many
  rotations each segment is struck at, evenly spaced across the full 360
  degrees (`360 / strikes_per_segment` degrees apart).
- **`--cycles` / `cycles`** (default 1): how many times the entire
  region-by-strike sweep repeats, end-to-end. Total strike count (before any
  depth-pass splitting) is `radial_segments * strikes_per_segment * cycles`.
  Every operation is simply replayed, in order, against one running material
  state, so later cycles automatically true up whatever an earlier, rougher
  cycle left rough or overshot -- there is no separate "cycle-aware" engine
  logic, it is exactly the same replay mechanism already used to converge a
  single cycle's own alternating rotations.
- **`--auto-cycles`**: instead of a fixed `cycles` count, keep adding cycles
  until the simulated (computational-geometry) final geometry matches the
  target within tolerance. Useful when you don't know in advance how many
  cycles a given die/segment/strike configuration needs to converge.
- **`--max-reduction` / `max_reduction_per_strike_mm`**: unrelated to the
  above and unchanged -- still caps how much radial material a single strike
  may remove, splitting a deeper requirement into multiple retract/re-strike
  depth passes at the same segment and rotation.

## Die geometry

Two rigid, finite surfaces act on every strike in the *preview*, matching
the machine: the **lower die** (the fixed anvil, rectangular) and the
**upper die** (the moving striker, a flat-faced circular disc). Leaving any
of the fields below unset does not mean "unconstrained" -- it means a
sensible physical default sized from the stock geometry, so the preview
conserves volume by bulging displaced material sideways by default, without
requiring you to configure die dimensions first. These settings only shape
the *preview's* intermediate deformation, never the strike coordinates
themselves -- the target's final geometry the machine actually cuts is
unaffected by them.

- The **upper die** (machine +Z, the real striking die) is a flat-faced
  circular disc. `--upper-die-radius` / `upper_die_radius_mm` sets its
  radius; left unset, it defaults to `stock_radius_mm`, which is always
  large enough to fully cover the target at the one segment a strike
  targets. Its *rigid* contact footprint never reaches a neighbouring
  segment, regardless of its radius -- widening it changes how far it
  reaches tangentially, never how far along X. (Its bulge margin, like the
  lower die's below, can still nudge an adjacent segment toward -- never
  past -- that segment's own eventual target.) An explicitly smaller radius
  is an honest physical trade-off: like a real round punch smaller than a
  face, it can legitimately leave part of that face unstruck -- **the
  "final shape always matches target exactly" guarantee below requires
  `upper_die_radius_mm >= stock_radius_mm`.**
- The **lower die** (machine -Z, the real anvil) never moves in real life.
  `--die-width` / `die_width_mm` and `--die-length` / `die_length_mm`
  describe *its* finite face -- the width across the strike/tangential
  direction and the length along the billet axis that actually supports the
  billet -- defaulting to `stock_radius_mm` and the striking segment's own
  width respectively. `--die-corner-radius` / `die_corner_radius_mm` blends
  its tangential edges into a radius instead of a sharp corner, and cannot
  exceed half of `die_width_mm`.

Within its footprint each die is an impenetrable displacement boundary: the
lower die holds material at the original stock surface (it never reduces
anything, only prevents bulging past where the billet already rests) across
every axial region its length spans and every tangential angle its width
spans, a genuinely 3D constraint; the upper die presses material down to the
commanded strike depth within its own disc, rigidly confined to the one
segment it targets. A `die_length_mm` spanning more than one segment's width
rigidly holds every segment the lower die reaches, not just the nearest one
-- but never extends the upper die's own rigid single-segment reach, since
widening the anvil's axial hold must never make the striking disc's rigid
footprint touch a segment it wasn't asked to.

Volume either footprint displaces is not deleted: it bulges the material
immediately adjacent to it -- both axially into neighbouring segments and
tangentially around neighbouring angles -- so that volume across every
affected segment is conserved, the same way forge-temperature steel spreads
out from under a die rather than shrinking in volume. That bulge fades out
within a couple of footprint dimensions of each die's edge, is capped so it
never overshoots past the target's own boundary in that exact direction
(never past that segment's own eventual value), and is tied entirely to that
die's own contact interface: material outside a strike's zone of influence
is left completely untouched by it.

At the default `upper_die_radius_mm` (which always fully covers the target
at the segment it strikes), **the final shape, once every rotation has
struck, still converges exactly to the target's own support envelope,
regardless of the configured lower-die size** -- the lower die's
configuration only shapes how the intermediate animation looks, never the
final result. This is checked directly in the test suite by comparing the
fully struck geometry against the target at every segment and orientation,
including a counter-example that deliberately undersizes
`upper_die_radius_mm` (and narrows the anvil, so its own generous default
doesn't independently fill in the gap) and confirms the resulting shortfall.

## Which end is clamped

`--clamped-end` / `stock_clamped_end` (`"min"` or `"max"`, default `"min"`)
tells the planner which end of the target mesh's bounding box, along its
resolved longitudinal axis, is held in the clamp. Every generated X is then
oriented so increasing X always means "away from the clamp," regardless of
which end of the mesh happened to sit at the origin when it was authored.

## Matching stock volume to the target

Every plan reports `target_volume_mm3` and `recommended_stock_length_mm` (also
printed as a warning line). The recommended length is the length a cylinder of
the configured `stock_radius_mm` would need to contain exactly the target's own
volume. Cutting stock to that length keeps the constant-volume model
consistent end to end: it is never asked to conjure material that was never
in the billet, or discard material that has nowhere else to go. Volume is
trapezoidal-integrated between segment *centres*, so with few
`radial_segments` it systematically undercounts the half-segment overhang at
each end of the part -- use more segments for a closer estimate. The UI
shows this figure after **Generate preview**; the CLI prints it as a warning
line.

## Constraints and safety checks

The target must fit inside the starting cylinder: every vertex of the
target's true, un-averaged cross-section -- sampled far more finely than
`radial_segments`, specifically so this check isn't fooled by a segment's
own averaging -- is checked against `stock_radius_mm`, not just the discrete
rotations actually struck. A square's diagonal corner, for example, can
exceed the stock radius even when its 0/90/180/270 faces all fit, since only
those discrete rotations are ever struck. The planner also rejects excess
machine X/Y/Z travel.

The rotary (A) axis is continuous on this machine and is not limited by the
planner or `configs/axis.jsonl`; any commanded rotation is valid.

This model is a geometric envelope planner, not a forming simulation. It does
not predict volume flow, flash, springback, temperature effects, die compliance,
or tooling collisions. Its generated Z values depend on the fixture-specific
`die_contact_z_mm` calibration. Review and prove every program off-material
before loading it into the machine controller.
