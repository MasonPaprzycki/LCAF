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

1. In **Example target**, select `Square bar — 10 mm`. The form fills in a
   10 mm stock radius, X-axis alignment, and a reasonable demonstration
   resolution.
2. The rotary (A) axis is continuous on this machine, so the four absolute
   orientations 0/90/180/270 degrees a square requires are always planned;
   there is no rotation limit to bypass.
3. Select **Generate preview**. The status line reports the radial segments,
   strikes per segment, and total strike operations.
4. In **Animated toolpath preview**, select **Play**. The gold die moves --
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
and the upper die renders as a real, sized circular disc -- not infinitely
thin sheets or unbounded planes -- so both dies' true, engine-confined extent
is what you see: the disc's own axial thickness matches exactly the one
segment the engine confines that strike to, not an arbitrarily generous
size. If the disc's true radius is wider than the segment it strikes, it is
drawn genuinely clipped by that segment boundary (an ellipse along X, at
full radius tangentially) rather than visually overshooting into
neighbouring segments the engine never lets it touch. Three mode buttons in
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

An unconfigured lower-die length keeps every strike's rigid effect confined
to exactly the one segment it targets -- nothing deforms in a segment a
visible die isn't touching unless you explicitly set a `die_length_mm` that
reaches further (its bulge margin can still nudge the immediately adjacent
segment toward, but never past, that segment's own eventual target).

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
- **Target temperature (°C) / X centre offset / Y tool position**: fixture
  coordinates and metadata used directly in the generated controller
  operations.
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

## Set the die geometry

Two rigid, finite surfaces act on every strike, matching the machine: the
**lower die** (the anvil, rectangular) and the **upper die** (the striker, a
flat-faced circular disc). Leaving any field in **3. Die geometry** blank
does not mean "unconstrained" -- it means a sensible physical default sized
from the stock geometry, so the preview conserves volume by visibly bulging
displaced material sideways by default, without configuring anything first.

- **Die face shape**: `Full rectangular (sharp edge)` (the previous
  behaviour) or `Radiused edge`, which blends the lower die's tangential
  edges into a corner radius instead of a sharp corner.
- **Contact length, X (mm)** / **Contact width, Y (mm)**: the lower die's
  face. Leave blank for a default sized from the striking segment's own
  width / the stock radius; set a finite value to model a lower die that
  supports further (or less far) along the billet, or only part of its
  cross-section.
- **Corner radius (mm)**: enabled only for a radiused face; must not exceed
  half of the contact width.
- **Upper die radius, Z (mm)**: the upper (striking) die's flat-faced
  circular contact radius. Leave blank to keep it large enough to always
  fully cover the target at the segment it strikes -- **an explicitly
  smaller radius is an honest physical trade-off: like a real round punch
  smaller than a face, it can leave part of that face unstruck, so the
  "final shape always matches target" guarantee below no longer holds.**

Within its footprint each die is an impenetrable displacement boundary: the
lower die holds material at the original stock surface across every segment
its length reaches, not just the one the strike is nominally "at"; the upper
die presses material to the commanded strike depth within its own disc,
rigidly confined to that one segment. Volume either footprint displaces is
not deleted: it bulges the material immediately adjacent to it -- both
axially into neighbouring segments and tangentially around neighbouring
angles -- the way forge-temperature steel spreads out from under a die
rather than shrinking in volume. That response fades out within a couple of
footprint dimensions of each die's edge, and is capped so it never
overshoots past the target's own boundary in that direction (never past a
neighbouring segment's own eventual value). **At the default upper die
radius, whatever the lower die is configured to do, once every rotation has
struck, the final shape always converges exactly to the target** -- the
lower die's own configuration only shapes how the animation looks partway
through, never the end result.

Below the geometry fields, a line reports the target's own volume and the
**recommended stock length**: how long a cylinder of the chosen stock radius
would need to be to contain exactly that volume. Cut your starting bar to
that length so the constant-volume model is never asked to remove volume
that has nowhere to go, or add volume that was never there. If it comes out
much shorter or longer than the target's own axial extent, that is a sign the
chosen stock radius is a poor match for the target and worth revisiting.

## Read validation errors before exporting

The planner stops rather than emits a program when a target needs material
outside the stock cylinder or an X/Y/Z command exceeds configured travel.
The stock-fit check looks at every corner of the target, not just the
rotations you configured -- a square's diagonal corner can exceed the stock
radius even when its four faces all fit, since only those four discrete
rotations are actually struck. Correct the setup or fixture (usually a
larger stock radius) instead of looking for an override.

The current visualisation is a computational-geometry envelope. It does not
predict material flow, flash, springback, thermal effects, die compliance, or
collisions. Treat its animation as a toolpath review aid. Review the JSONL,
prove it off-material, and use normal ForgeBrain/LinuxCNC safety procedures
before any physical motion.

## Export and inspect JSONL

After reviewing the entire animation, select **Export JSONL** and choose a
file. Each line is one ForgeBrain `ToolpathOperation`. The metadata records the
source segment, cycle, target support, strike pass, and the die geometry
(lower die width, length, corner radius, shape; upper die radius) used for
that strike.

For the controller semantics and CLI usage, see
[toolpath_slicer.md](toolpath_slicer.md). For the supplied geometries, see
[examples/README.md](../examples/README.md).
