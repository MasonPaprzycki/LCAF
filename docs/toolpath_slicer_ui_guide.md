# Toolpath Slicer UI Guide

The Toolpath Slicer UI is a local planning and visualisation tool. It does not
connect to LinuxCNC, enable the forge, or command an axis. It creates a JSONL
file only when you choose **Export JSONL**.

## Start the UI

From the repository root in PowerShell:

```powershell
$env:PYTHONPATH = (Resolve-Path .\src)
python -m lcaf.toolpathing.ui
```

## First animated example

1. In **4. Surrogate deformation model**, pick a trained checkpoint (`.npz`)
   -- either from the quick-pick dropdown (auto-populated from
   `lcaf/simulation/surrogate/trained_network_parameters/`) or **Browse
   .npz…** for any other file. **This is required**: the animated preview
   has no built-in geometric fallback, and **Generate preview** refuses to
   run without a checkpoint selected. See
   [Choose a surrogate model](#choose-a-surrogate-model) below.
2. In **Example target**, select `Square bar — 10 mm`. The form fills in a
   10 mm stock radius, X-axis alignment, and a reasonable demonstration
   resolution.
3. The rotary (A) axis is continuous on this machine, so the four absolute
   orientations 0/90/180/270 degrees a square requires are always planned;
   there is no rotation limit to bypass.
4. Select **Generate preview**. The status line reports the radial segments,
   strikes per segment, and total strike operations.
5. In **Animated toolpath preview**, select **Play**. The gold die moves --
   only the punch itself animates; the X/A repositioning between strikes is
   instant, matching how the machine actually moves. A bold **Step N of M**
   counter tracks progress; use **Pause**, **Step**, **Restart**, and the
   speed slider to inspect a command. The die's animated stroke always starts
   from wherever the material actually is (not the pristine stock surface),
   so a second pass on an already-reduced segment never appears to retract
   back out through material that is no longer there.

Two dies are always shown. The **lower die** -- the rectangular anvil whose
length, width, and corner radius you configure in **3. Die geometry** -- is
drawn gold and is the *only* one that animates through a stroke, moving from
wherever the material currently is to the commanded depth. In real life it
is the billet's X/Y/rotation and the upper die that actually move, not the
lower die, but since the lower die is the one whose rectangular geometry is
prescribed here, that is the one this view animates. The **upper die** is
drawn as a fixed, translucent light-blue circular disc -- sized by its own
**Upper die radius** field -- that never moves: it always sits against the
opposite side of the billet from the lower die, at wherever the material
actually currently is there (not a fixed pristine position, which could
otherwise appear to float clear of the billet once an earlier rotation has
already reduced that side).

The **2D cross-section + axial plan** tab's left panel is the current radial
cross-section, with the dashed circle as the starting cylinder, green as the
target's convex envelope, gold as the animated lower die, and translucent
blue as the fixed upper die. The right panel is an axial view: it highlights
the segment, die position, rotary orientation, and multi-strike pass count.

Select the **3D view** tab to switch to an interactive playback of the same
animation, where the lower die renders as a real, finite rectangular block
and the upper die renders as a real, full-size circular disc, at its
correctly scaled `upper_die_radius_mm` -- not infinitely thin sheets or
unbounded planes, and never flattened, chopped, or shrunk to reflect how far
that one strike's own rigid contact mechanics is confined. The die is
always rendered as the true, physical tool it represents; the rigid contact
computation's own confinement to the one segment a strike targets (see
[toolpath_slicer.md](toolpath_slicer.md#die-geometry)) is a modelling choice
about how far this one strike's commanded depth is trusted to apply, not a
claim about the tool's actual size or shape. Three mode buttons in
the top-left corner control what dragging does:

- **Rotate**: spins the whole view about the fixed axis pointing into the
  screen.
- **Roll**: free 3D orbit, examining the part from any angle -- like
  SolidWorks' rotate.
- **Pan**: drags the view around in X/Y.

Hold the left mouse button down and move the mouse to use whichever mode is
selected; use the **+**/**−** buttons (not the scroll wheel) to zoom, and
**Reset view** to return to the default angle, zoom, and pan. A small
labelled X/Y/Z triad in the bottom-right corner always shows the current
orientation.

A **Show dies** checkbox next to the mesh-quality slider hides both dies in
both tabs, leaving only the red deforming geometry and the green target
visible, if you want an unobstructed view of the deformation itself. The same
slider sets how many radial samples the deformed-geometry preview uses --
higher values render a smoother deformed surface at some extra redraw cost.

How far one strike's own effect reaches -- which neighbouring segments it
visibly nudges, and by how much -- is now entirely up to the trained
surrogate checkpoint you selected in **4. Surrogate deformation model**,
not a rule this UI implements. `die_length_mm` (the lower die's own
**Contact length, X**) still matters: it is read directly as the strike's
bite length, one of the three process parameters the network was trained
on. See [Deformation preview](toolpath_slicer.md#deformation-preview) and
[docs/surrogate_deformation_model.md](surrogate_deformation_model.md) for
what the network predicts.

Both tabs also render an **amber trim allowance**: forging conserves volume
exactly, so whatever the surrogate's own local displacement prediction does
not reabsorb nearby must reappear as extra length at the target's own free
end, exactly the way a real bar upsets (gets longer) as its cross-section is
squeezed down without also being cut shorter. A dashed line marks exactly
where the target's own defined length ends and this allowance begins --
material a saw trims off once forging is complete. Its *total* is a fixed
quantity set entirely by your stock length and the target's own volume. It
grows as strikes progress; see
[Volume conservation and the trim allowance](toolpath_slicer.md#volume-conservation-and-the-trim-allowance)
for the underlying volume balance. It is a preview/reporting quantity only:
it is never written into the exported JSONL and never changes a planned
strike's own coordinates.

Select the **Force estimate** tab to see a separate, independent estimate of
both the forging **force** and the die **contact pressure (stress)** each
strike needs, computed from the chosen material/temperature and the
strike's own die contact geometry -- a standard slab-method (friction-hill)
hand calculation, not a simulation. Force and pressure are not the same
question: the same total force concentrated over a small contact patch
needs far higher stress to induce plastic flow than the same force spread
over a large one, so a die/press capacity check needs the force number while
a "will this actually deform the material" check needs the pressure number.
Neither feeds back into the deformation preview or the planned coordinates:
dies are treated as rigid and able to supply whatever force/pressure is
shown. The tab reports the current step's force and pressure (updating live
during playback), the peak of each across the whole plan, and a separate
line plot for force (kN) and for contact pressure (MPa), each with the
current step marked.

## Choose a target

Use **Choose OBJ / STL** for your own watertight target mesh, or choose an
included target from **Example target**:

| Example | Useful for seeing |
| --- | --- |
| Square bar | Four-face reduction of a cylinder into a constant square profile |
| Hex bar | Six-sided target geometry with a constant axial profile |
| Tapered square bar | Segment-to-segment die-depth changes along X |
| Tapered hex bar | Both changing axial profile and polygonal radial support |

Native `.sldprt` files must be exported to watertight STL or OBJ first. Use the
model's intended length direction for **Billet longitudinal axis**; `auto`
chooses its largest bounding-box dimension.

## Set the planning inputs

- **Stock radius**: radius of the initial cylindrical billet. The target must
  fit inside this cylinder at every planned orientation.
- **Radial segments**: how many axial regions the target's length is divided
  into. Each region's own strike depth is the numerical average of the
  target's true cross-section across that whole region, not a single point
  -- so more segments follow a taper more closely (and create more
  operations), while a target that is itself a constant N-sided polygon
  along its length only ever needs enough segments to resolve where its
  faces actually change.
- **Strikes per segment**: how many rotations each segment is struck at,
  evenly spaced across the full 360°. `4` produces the four sides of a
  square; a hexagon wants `6`, an octagon `8`.
- **Max reduction / strike**: caps radial reduction per hit. Smaller values
  create additional incremental presses at each segment/orientation.
- **Die contact Z**: the measured machine Z coordinate at first contact with
  the unformed stock. This is fixture/tooling calibration, not CAD data.
- **Model scale**: millimetres per OBJ/STL unit; use `25.4` for inch-based
  geometry.
- **Target temperature (°C)**: fixture metadata recorded on every generated
  operation, and -- together with **Billet material** below -- drives the
  separate force/pressure estimate on the **Force estimate** tab. It does
  not affect the deformation preview (a surrogate checkpoint is trained for
  one material/temperature combination -- see
  [Choose a surrogate model](#choose-a-surrogate-model)) or change the
  planned strike coordinates themselves.
- **X offset from clamp (mm) / Y tool position**: fixture coordinates used
  directly in the generated controller operations.
- **Cycles**: how many times the entire radial-segment x strike sweep
  repeats, end-to-end. Every operation just replays, in order, against one
  running material state, so a second cycle automatically trues up whatever
  the first, rougher cycle left rough or overshot -- the same way a real
  finishing pass would. Disabled while **Complete necessary cycles
  automatically** is checked, which instead keeps adding cycles on its own
  until the simulated geometry matches the target closely, reporting how
  many it needed in the status line once **Generate preview** finishes.
- **Clamped end**: which end of the mesh's bounding box, along its resolved
  longitudinal axis, is held in the clamp. Machine +X always points away from
  whichever end you pick here, regardless of how the source mesh was
  authored -- see the axis convention in
  [toolpath_slicer.md](toolpath_slicer.md).
- **Billet material**: a dropdown of named materials grouped by family --
  a generic `Plasticine`/`Aluminum`/`Steel` band per family, plus several
  specific grades within each (e.g. `Steel 1018 (mild/low-carbon)`,
  `Steel 4140 (chromoly alloy)`, `Steel 304 (austenitic stainless)`,
  `Aluminum 1100 (pure, dead-soft)`, `Aluminum 6061 (structural)`,
  `Aluminum 7075 (aerospace, high-strength)`, `Plasticine -- soft grade`,
  `Plasticine -- hard grade`). Together with **Target temperature**, the
  chosen grade drives the separate force/contact-pressure estimate on the
  **Force estimate** tab -- see
  [Material and temperature](toolpath_slicer.md#material-and-temperature)
  for the underlying model, including how each named grade's numbers were
  chosen (order-of-magnitude engineering estimates, not sourced alloy
  datasheets). A hint under the picker shows a realistic temperature range
  for the chosen material's family. Neither setting changes the planned
  strike coordinates, the target's final geometry, or (since a surrogate
  checkpoint is trained for one material/temperature combination) the
  deformation preview.

## Set the die geometry

Two rigid, finite surfaces act on every strike, matching the machine: the
**lower die** (the anvil, rectangular) and the **upper die** (the striker, a
flat-faced circular disc). Leaving any field in **3. Die geometry** blank
does not mean "unconstrained" -- it means a sensible physical default sized
from the stock geometry.

- **Die face shape**: `Full rectangular (sharp edge)` (the previous
  behaviour) or `Radiused edge`, which blends the lower die's tangential
  edges into a corner radius instead of a sharp corner.
- **Contact length, X (mm)** / **Contact width, Y (mm)**: the lower die's
  face. Leave blank for a default sized from the striking segment's own
  width / the stock radius; set a finite value to model a lower die that
  supports further (or less far) along the billet, or only part of its
  cross-section. **Contact length, X** is the one field here that reaches
  the deformation preview directly: it is read as the strike's own bite
  length, one of the surrogate's three trained process-parameter inputs
  (see [Deformation preview](toolpath_slicer.md#deformation-preview)).
- **Corner radius (mm)**: enabled only for a radiused face; must not exceed
  half of the contact width.
- **Upper die radius, Z (mm)**: the upper (striking) die's flat-faced
  circular contact radius.

**Contact width, Y**, **Corner radius**, and **Upper die radius** now only
affect the *rendered* shape of the dies in the preview, not the predicted
deformation -- the surrogate network was trained assuming both dies are
wide enough to fully support the workpiece (the paper's own assumption; see
[docs/surrogate_deformation_model.md](surrogate_deformation_model.md)'s
scope section), which is also this machine's own default configuration.
Setting a deliberately undersized value here is outside the network's
trained domain and will not visibly restrict the animated deformation the
way it used to under the old geometric preview.

## Choose a surrogate model

**4. Surrogate deformation model** picks which trained network drives the
animated preview -- see
[docs/surrogate_deformation_model.md](surrogate_deformation_model.md) for
what it predicts and its own scope/limitations.

- **Checkpoint** (dropdown): every `.npz` file already present in
  `lcaf/simulation/surrogate/trained_network_parameters/`, auto-discovered
  each time the UI starts.
- **Browse .npz…**: pick any other checkpoint file.

Once loaded, the status line next to these controls reports the
checkpoint's own description and, if it is the repository's committed
`dummy_smoke_test.npz` fixture, an explicit warning that it is a
structural test fixture only -- trained on synthetic data, not real FEA
results, and **not physically meaningful**. There is no real, FEA-trained
checkpoint shipped with this repository yet; see
[docs/surrogate_training_guide.md](surrogate_training_guide.md) to generate
training data and train one.

**Generate preview** refuses to run, with an explanatory error, until a
checkpoint is selected -- there is no geometric fallback.

Unlike the old geometric preview, **the final shape is no longer
guaranteed to converge exactly to the target**, at any die configuration:
a trained network predicts what a real strike actually does, which may
never reach an arbitrary target. **Complete necessary cycles
automatically** keeps adding cycles until it does converge (within its own
tolerance), and reports (as a warning) if it still cannot within its own
cycle cap -- treat that warning as a signal the plan may be asking for more
reduction than this die/checkpoint combination can physically achieve, not
as a bug to work around by raising the cycle cap indefinitely.

Below the geometry fields, a line reports the target's own volume and the
**recommended stock length**: how long a cylinder of the chosen stock radius
would need to be to contain exactly that volume. Cut your starting bar to
that length so the constant-volume model is never asked to remove volume
that has nowhere to go, or add volume that was never there. If it comes out
much shorter or longer than the target's own axial extent, that is a sign the
chosen stock radius is a poor match for the target and worth revisiting.
The same line also reports the expected **trim allowance** left over once
forging finishes if you instead start from the mesh's own (typically
longer) axial extent -- the amber stub described above, quantified in mm.

## Read validation errors before exporting

The planner stops rather than emits a program when a target needs material
outside the stock cylinder or an X/Y/Z command exceeds configured travel.
The stock-fit check looks at every corner of the target, not just the
rotations you configured -- a square's diagonal corner can exceed the stock
radius even when its four faces all fit, since only those four discrete
rotations are actually struck. Correct the setup or fixture (usually a
larger stock radius) instead of looking for an override.

Only the exported strike coordinates themselves come from an exact
geometric construction -- the animated bulge is a trained neural network's
prediction (see [docs/surrogate_deformation_model.md](surrogate_deformation_model.md)),
not a verified physics solve, and the force/contact-pressure estimate is a
standard hand-calculation, not a simulation either. Neither predicts true
material flow, flash, springback, die compliance, friction distribution,
strain hardening, in-process temperature change, or tooling/fixture
collisions -- see [Limits of the computational-geometry
model](toolpath_slicer.md#limits-of-the-computational-geometry-model) for
the full list, including the surrogate's own explicit scope gaps. Treat its
animation, and the force/pressure numbers, as toolpath review and
process-planning aids only. Review the JSONL, prove it off-material, and
use normal ForgeBrain/LinuxCNC safety procedures before any physical
motion.

## Export and inspect JSONL

After reviewing the entire animation, select **Export JSONL** and choose a
file. Each line is one ForgeBrain `ToolpathOperation`. The metadata records the
source segment, cycle, target support, strike pass, the die geometry
(lower die width, length, corner radius, shape; upper die radius), and the
chosen material used for that strike -- purely informational: the machine-
facing fields (`x`, `y`, `die_gap`, `rotation`, `target_temperature`) are
unaffected by material and are all `ForgeBrain.load_jsonl()` actually reads.
The separate force estimate is never written into this file; read it from
the **Force estimate** tab, or from the CLI's own printed peak-force line.

For the controller semantics and CLI usage, see
[toolpath_slicer.md](toolpath_slicer.md). For the supplied geometries, see
[examples/README.md](../examples/README.md).
