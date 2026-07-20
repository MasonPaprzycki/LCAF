# Profile Slicer UI Guide

The Profile Slicer UI is a local planning and visualisation tool. It does not
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
2. The checked-in machine configuration has an A-axis range of -90 to +90
   degrees. A square requires the four absolute orientations 0/90/180/270
   degrees, so the planner correctly refuses it by default.
3. To view the demonstration animation only, enable **Override configured
   A-axis limits**. Do this for an actual program only after continuous or
   indexed rotation and LinuxCNC limits have been proven for the machine.
4. Select **Generate preview**. The status line reports the stations,
   orientations, and total strike operations.
5. In **Animated toolpath preview**, select **Play**. The gold die line moves
   from the cylindrical stock boundary toward the requested strike depth. Use
   **Pause**, **Step**, **Restart**, and the speed slider to inspect a command.

The left panel is the current radial cross-section. The dashed circle is the
starting cylinder, green is the target's convex envelope, and gold is the die
at its current animated depth. The right panel is an axial view: it highlights
the station, die position, rotary orientation, and multi-strike pass count.

## Choose a target

Use **Choose OBJ / STL** for your own watertight target mesh, or choose an
included target from **Example target**:

| Example | Useful for seeing |
| --- | --- |
| Square bar | Four-face reduction of a cylinder into a constant square profile |
| Hex bar | Six-sided target geometry with a constant axial profile |
| Tapered square bar | Station-to-station die-depth changes along X |
| Tapered hex bar | Both changing axial profile and polygonal radial support |

Native `.sldprt` files must be exported to watertight STL or OBJ first. Use the
model's intended length direction for **Billet longitudinal axis**; `auto`
chooses its largest bounding-box dimension.

## Set the planning inputs

- **Stock radius**: radius of the initial cylindrical billet. The target must
  fit inside this cylinder at every planned orientation.
- **Axial spacing**: maximum distance between X stations. Smaller values follow
  tapers more closely and create more operations.
- **Rotation step**: angular resolution of radial faces. `90` produces the
  four sides of a square. It must divide 360 exactly.
- **Max reduction / strike**: caps radial reduction per hit. Smaller values
  create additional incremental presses at each station.
- **Die contact Z**: the measured machine Z coordinate at first contact with
  the unformed stock. This is fixture/tooling calibration, not CAD data.
- **Model scale**: millimetres per OBJ/STL unit; use `25.4` for inch-based
  geometry.
- **X centre offset / Y tool position**: fixture coordinates used directly in
  the generated controller operations.

## Read validation errors before exporting

The planner stops rather than emits a program when a target needs material
outside the stock cylinder, an X/Y/Z command exceeds configured travel, or a
rotation is outside the configured A-axis range. Correct the setup, fixture,
or machine configuration instead of treating an override as a general fix.

The current visualisation is a computational-geometry envelope. It does not
predict material flow, flash, springback, thermal effects, die compliance, or
collisions. Treat its animation as a toolpath review aid. Review the JSONL,
prove it off-material, and use normal ForgeBrain/LinuxCNC safety procedures
before any physical motion.

## Export and inspect JSONL

After reviewing the entire animation, select **Export JSONL** and choose a
file. Each line is one ForgeBrain `ToolpathOperation`. The metadata records the
source station, target support, strike pass, and whether a rotary-limit
override was active during planning.

For the controller semantics and CLI usage, see
[toolpath_slicer.md](toolpath_slicer.md). For the supplied geometries, see
[examples/README.md](../examples/README.md).
