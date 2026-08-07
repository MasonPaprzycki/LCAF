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

- **X** is the billet's long axis, zero-based like the machine itself:
  **X=0 is the clamp** -- the same physical point LinuxCNC's own homing
  gives machine X=0 (the negative limit switch end of travel), so every
  generated X is `>= 0`. `stock_clamped_end` (`"min"` or `"max"`, see below)
  tells the planner which end of the *mesh's* bounding box is actually
  clamped, since a source mesh may be authored either way; every generated X
  is measured from that end, increasing outward toward the free/forged end.
  `x_offset_mm` adds on top of this if the clamp itself sits some fixed
  distance from machine X=0 rather than exactly at it.

  This generated X (and Y, and `die_gap`/Z) is deliberately left zero-based
  at the physical switch, matching LinuxCNC's own machine coordinate 0 --
  this module never shifts it to account for `retracted_distance` (the
  joint's post-homing standoff position and real soft-limit floor, see
  `docs/hardware_setup.md` sections 7 and 10). That correction happens once,
  later, when the JSONL is actually loaded
  (`ForgeBrain.load_jsonl()`/`_retracted_offset_mm()`), not here -- so this
  module's own output, and any JSONL file it wrote, never needs editing
  when `retracted_distance` changes.
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

Add `--material` and `--temperature` to make the billet material/temperature
drive the *preview's* deformation mechanics and a separate force/contact-
pressure estimate printed after planning -- see
[Material and temperature](#material-and-temperature) below. Neither flag
changes the planned strike coordinates. `--material` accepts one generic
band per family (`plasticine`, `aluminum`, `steel`; default `steel`) plus
several named grades within each family -- run `python -m lcaf.toolpathing
--help` for the full, current list (`lcaf.toolpathing.material.MATERIALS`).

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

Two rigid, finite surfaces are defined for every strike, matching the
machine: the **lower die** (the fixed anvil, rectangular) and the **upper
die** (the moving striker, a flat-faced circular disc). Leaving any of the
fields below unset does not mean "unconstrained" -- it means a sensible
physical default sized from the stock geometry. These settings never
change the strike coordinates themselves -- the target's final geometry the
machine actually cuts is unaffected by them.

- The **upper die** (machine +Z, the real striking die) is a flat-faced
  circular disc. `--upper-die-radius` / `upper_die_radius_mm` sets its
  radius; left unset, it defaults to `stock_radius_mm`.
- The **lower die** (machine -Z, the real anvil) never moves in real life.
  `--die-width` / `die_width_mm` and `--die-length` / `die_length_mm`
  describe *its* finite face -- the width across the strike/tangential
  direction and the length along the billet axis that actually supports the
  billet -- defaulting to `stock_radius_mm` and the striking segment's own
  width respectively. `--die-corner-radius` / `die_corner_radius_mm` blends
  its tangential edges into a radius instead of a sharp corner, and cannot
  exceed half of `die_width_mm`.

**Only `die_length_mm` reaches the deformation preview.** It is read
directly as the strike's own bite length -- one of the three process
parameters (`alpha0`/`xb`/`eps_h`) the surrogate network
(`lcaf.simulation.surrogate`, see
[docs/surrogate_deformation_model.md](surrogate_deformation_model.md)) was
trained on. `upper_die_radius_mm`, `die_width_mm`, and
`die_corner_radius_mm` now only affect the *rendered* shape of the dies in
the preview -- the network was trained assuming both dies are wide enough
to fully support the workpiece (matching the paper's own implicit saddle
assumption, and this machine's own default configuration), so a
deliberately undersized value here does not visibly restrict the predicted
deformation the way it did under the old geometric preview, and is outside
the network's own trained domain (see
`lcaf.simulation.surrogate.process_params.ProcessParameters.within_trained_domain`).

Unlike the old geometric preview, **there is no guarantee the final shape
converges exactly onto the target**, at any die configuration or cycle
count -- a trained network predicts what a real strike actually does, not
what is needed to hit an arbitrary target. `--auto-cycles` keeps adding
cycles until the surrogate-predicted geometry does converge (within
tolerance), and warns rather than silently giving up if it still has not by
`--auto-cycles-max` -- treat that warning as a signal the plan may ask for
more than this die/checkpoint combination can physically achieve.

## Material and temperature

`--material` and `--temperature` (degrees C) are resolved by
`lcaf.toolpathing.material` into a temperature band with a 0..1
`formability` -- an approximate, order-of-magnitude engineering estimate,
not a sourced alloy datasheet or a constitutive model. Bands are
deliberately simple (cold/warm/hot per material, roughly matched to where
each material is actually forged/worked in practice) rather than a
continuous curve, for practicality.

`lcaf.toolpathing.material.MATERIALS` lists every valid `--material` key.
Each family -- `plasticine`, `aluminum`, `steel` -- keeps its original bare
name as a generic, prototypical mid-range band, and additionally offers
several named grades with their own flow stress/friction/formability
numbers (e.g. `steel_1018` mild steel, `steel_4140` chromoly alloy,
`steel_304_stainless`, `aluminum_1100` pure/dead-soft, `aluminum_6061`,
`aluminum_7075` aerospace alloy, `plasticine_soft`, `plasticine_hard`). These
grades are still hand-picked, order-of-magnitude engineering estimates, not
mill-test-report or datasheet values -- their purpose is to make relative
comparisons between grades believable (a stainless needs more force/pressure
than a mild steel at the same geometry; a soft aluminum spreads more per
strike than a hard one), not to stand in for real material characterization.
`lcaf.toolpathing.material.MATERIAL_LABELS` maps each key to a human-readable
label for the UI's material picker.

**`formability` no longer affects the deformation preview.** The preview is
now driven entirely by a trained neural surrogate (see
[docs/surrogate_deformation_model.md](surrogate_deformation_model.md)) --
`lcaf.simulation.surrogate` -- rather than the geometric heuristic this
section used to describe (a rigid die clip plus a raised-cosine bulge
gated by `reach_scale`/`closure_fraction`, both derived from `formability`).
A surrogate checkpoint is trained for one material/temperature combination,
so `--material`/`--temperature` now only feed the separate force/pressure
estimate below, never the animated shape.

A completely separate slab-method (friction-hill) estimate -- the standard
closed-form flat-die forging calculation -- is computed from the same
material/temperature and the strike's own die geometry. It is reported two
ways:

- **Contact pressure** (`lcaf.toolpathing.material.
  estimate_strike_contact_pressure_mpa` / `estimate_operation_stress_mpa`),
  in MPa: `p = flow_stress * (1 + friction * width / (3 * height))`. This is
  the actual *stress* the die must apply against the material's own contact
  area to induce plastic flow at that geometry -- the number a die/press/
  frame strength check should be sized against, since it does not depend on
  how large the contact patch happens to be.
- **Force** (`lcaf.toolpathing.material.estimate_strike_force_kn` /
  `estimate_operation_force_kn`), in kN: `F = p * contact_area`, the
  contact pressure above times the strike's own contact area. This is what a
  press-tonnage/actuator-capacity check should be sized against.

The two numbers can diverge sharply for the same material: a small die
concentrates a modest total force into very high contact pressure, while a
large die needs much more total force to reach the same pressure. Reading
force alone can therefore understate how much stress the material and
tooling actually see at a strike with a small contact patch. Both are purely
hand-calculation-grade estimates for process planning, entirely separate
from the deformation preview: neither feeds back into planned coordinates or
the animated shape, and dies are treated as rigid and able to supply
whatever force/pressure is reported. `plan_force_report` returns both
figures per operation. The CLI prints the plan's peak estimated force *and*
peak contact pressure; the UI's **Force estimate** tab plots both across the
whole plan (see [the UI guide](toolpath_slicer_ui_guide.md)).

## Deformation preview

How a strike actually displaces material in the preview -- the affected
zone around it, how far it reaches along X, whether it grows toward the
target -- is now entirely up to whichever trained
`lcaf.simulation.surrogate.inference.SurrogateNetwork` checkpoint is
selected in the UI (section 4 of the toolpath UI), not a rule this planner
implements. See [docs/surrogate_deformation_model.md](surrogate_deformation_model.md)
for what the network predicts and how it plugs into
`lcaf.toolpathing.visualization`. There is no built-in fallback -- a
checkpoint must be selected before a preview can be generated at all.

## Volume conservation and the trim allowance

Forging never creates or deletes material -- unlike machining, nothing is
cut away. Whatever the surrogate's own local displacement prediction does
not reabsorb near a strike has to go somewhere: real open-die forging
pushes it out the free end (the clamped end cannot move), extending the
billet's total length beyond the target's own -- exactly the "upset" a bar
undergoes when its cross-section is reduced without also being cut
shorter, which a shop then saws off once forging is complete.

`lcaf.toolpathing.visualization.axial_trim_allowance_mm(plan,
operation_index, operation_progress, network, radial_segments=48)` computes
that length directly from a volume balance -- current total volume (the
current, deformed cross-section trapezoidally integrated over every
station, from the surrogate-driven `material_state`) versus the original
stock cylinder's volume -- rather than modelling axial material flow
directly, so it is exactly conserving by construction and continuous in
`operation_progress` (both the reference volume and the current volume use
the same polygon discretisation and the same station-centre trapezoidal
convention, so the comparison is apples-to-apples and reads exactly zero
before the first strike, not some phantom discretisation error). It returns
0.0 whenever the current state already holds at least as much volume as
the original stock.

This trim allowance is purely a *reporting/preview* quantity: it is never
written into the exported JSONL and never changes a planned strike's own
coordinates. The UI renders it as a distinct amber stub extending past the
target's own free end in both the 2D axial-plan panel and the 3D view, with
a dashed line marking exactly where the target's own length ends and the
allowance begins, and reports the final expected allowance (once all
strikes/cycles are done) in the stock-length info line after **Generate
preview**. Using `recommended_stock_length_mm` (see below) as your actual
starting stock length minimizes this allowance to (ideally) zero; using the
mesh's own, typically longer, axial extent as your starting stock length
(the simpler, more common real-world choice) will show a persistent,
non-zero allowance to saw off -- both are handled by the same volume
balance, honestly.

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

The rotary (A) axis is continuous on this machine and free to rotate either
direction from wherever it was sitting at power-on (there is no cable-wrap
constraint -- see `docs/hardware_setup.md`), so any signed `rotation` value
is valid and not limited by the planner or `configs/axis.json`.

## Limits of the computational-geometry model

This planner is a geometric envelope planner, not a forming simulation, and
that distinction should be read literally, not as a hedge. Concretely:

- **The strike coordinates (x/y/die_gap/rotation) are the only outputs this
  module can vouch for.** They come from an exact, closed-form geometric
  construction (numerical integration of the target's true cross-section,
  the convex-hull/support-function math in this module) with no free
  parameters and no physical modelling involved.
- **The animated bulge is a trained neural network's prediction, not a
  closed-form geometric construction.** See
  [docs/surrogate_deformation_model.md](surrogate_deformation_model.md) for
  what it is (a from-scratch implementation of Jagtap, Reinisch & Bailly,
  ESAFORM 2024, generalised to 3D) and its own explicit scope/limitations
  section -- notably: trained on idealised rectangular billets and
  full-width dies (a deliberately undersized `die_width_mm`/
  `upper_die_radius_mm` now only changes the *rendered* die shape, not the
  predicted deformation), no guarantee of ever converging exactly onto an
  arbitrary target the way the old geometric heuristic did, and no
  self-intersection guard. Two different die/segment configurations that
  look equally plausible in the preview can still differ arbitrarily in how
  a real billet would actually deform between them -- the network was
  trained on FEA data for one material/process window, not validated
  against physical trials on this machine.
- **The volume-conservation trim allowance
  (`axial_trim_allowance_mm`) is a real, exact volume balance of the
  *displayed* geometry** -- current deformed cross-section vs. original
  stock cylinder -- so it is internally consistent with what the preview
  shows. It is not a prediction of real forging upset length, which depends
  on friction, die shape, temperature gradients, and flow behavior this
  model does not represent.
- **Concave and hollow cross-sections are silently reduced to their convex
  outer support envelope** (see the planner's own warning text in `plan()`).
  Any concave feature -- a slot, a groove, an internal cavity -- is not
  planned at all; the strikes generated are for the convex hull around it.
- **What this model never predicts, at all:** true metal flow/grain flow,
  flash formation, springback, die deflection/compliance, friction
  distribution across a contact face (the friction-hill correction is a
  single averaged number, not a spatial distribution), strain hardening
  evolution across strikes, adiabatic heating or any temperature evolution
  during forging (`target_temperature_c` is a fixed, user-supplied constant,
  not simulated), grain structure, or tooling/fixture collisions.
- The separate force/contact-pressure estimate (see [Material and
  temperature](#material-and-temperature)) is a standard hand-calculation
  (slab-method/friction-hill), not a simulation either -- it assumes
  perfectly rigid, indestructible dies and a single averaged contact
  pressure, and is meant for order-of-magnitude process-planning sanity
  checks (is this within the press's tonnage rating), not for a stress
  analysis of the tooling or fixture.
- Its generated Z values depend entirely on the fixture-specific
  `die_contact_z_mm` calibration measured at setup, not on anything CAD or
  computational geometry can supply.

None of the above is a defect to be silently trusted around: review and
prove every generated program off-material, and use the deformation
preview, trim allowance, and force/pressure estimate as planning aids only
-- not as a substitute for a real forming simulation or physical trial
where the process, material, or tooling is unfamiliar.
