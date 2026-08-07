# Surrogate deformation model

`lcaf.simulation.surrogate` is the toolpath slicer's animated-preview
deformation engine: a feed-forward neural network that predicts per-node
displacement for one open-die forging stroke. It replaced the previous
`lcaf.toolpathing.visualization` engine, which was a pure
computational-geometry heuristic (a rigid die clip plus a hand-tuned bulge
relaxation) with no connection to how a real strike actually moves material.

## 1. Source technique and citation

This implements the method in:

> Jagtap, N. V., Reinisch, N., & Bailly, D. (2024). *Fast prediction of the
> material displacement in open die forging using neural networks.*
> Material Forming – ESAFORM 2024, Materials Research Proceedings 41,
> 2299–2308. https://doi.org/10.21741/9781644903131-253 (CC BY 3.0)

The paper trains a feed-forward MLP on FEA data to predict a point's
displacement from three process parameters plus its reference-configuration
coordinates, restricted to a 2D cross-section through the workpiece's core
fibre. See `lcaf/simulation/surrogate/README.md` for the full
adopted-vs-changed breakdown; the short version is: **everything is taken
directly from the paper except one addition -- the spread/width axis is
added as a third network input and output, extending the model from the
paper's 2D core-fibre plane to full 3D.**

## 2. The three process parameters

Following the paper's Eq. 1-3, one forging stroke is described by:

- **`alpha0` (aspect ratio)** = `h0 / w0` -- the pre-stroke workpiece's
  height (press direction) over its width (spread direction).
- **`xb` (bite ratio)** = `bite_length / h0` -- the die's axial contact
  length over the pre-stroke height.
- **`eps_h` (height reduction)** = `reduction / h0` -- how much this one
  stroke reduces the height, as a fraction of the pre-stroke height.

`lcaf/simulation/surrogate/process_params.py` is the single implementation
of these formulas (and the paper's own Table 1 variable-space ranges:
`alpha0` in [0.9, 1.5], `xb` in [0.3, 0.9], `eps_h` in [0.05, 0.26]), shared
by both the runtime inference path and the offline FEA data generator, so
neither can define these quantities two different ways.

## 3. Mapping onto this machine's geometry

`lcaf.toolpathing.toolpath_slicer` already computes everything needed to
derive `alpha0`/`xb`/`eps_h` for a real strike -- nothing new is invented:

| Process parameter | Comes from |
|---|---|
| `h0`, `w0` (pre-stroke height/width) | The *current* material state's own support in the strike's press/spread directions (`lcaf.simulation.surrogate.geometry.support_from_row`, the same support-function definition as `Segment.support_mm`) -- stateful across strikes, exactly like the paper's own multi-stroke handling. |
| `bite_length` | `die_length_mm` in the STRIKE operation's metadata (already resolved to the striking segment's own axial width by `ToolpathSlicer.plan()`). |
| `reduction` | `radial_reduction_mm` in the STRIKE metadata, divided by `strike_pass` to recover this specific pass's own increment (that field is the *cumulative* reduction through this pass). |

### Local coordinate frame

The network is only ever evaluated in one strike's own local frame
(`lcaf.simulation.surrogate.geometry.LocalFrame`):

- **`z0`** (axial, paper's "core fibre" direction) -- offset from the die's
  leading edge along machine X (`segment_x_start_mm`). Paper: "moving the
  origin of the coordinate system to the left edge of the saddle."
- **`y0`** (press direction) -- 0 at the anvil surface, `h0` at the
  pre-stroke free (die-facing) surface.
- **`x0`** (spread direction) -- 0 on the rotation-axis centreline.

This is built directly from the strike's own `rotation_deg` (the billet
rotation this operation commands) and re-derives the same projection
`Segment.support_mm` already uses -- `press_direction() = (sin(rotation),
cos(rotation))` in the machine's (Y, Z) plane, `tangential_direction()`
orthogonal to it.

### 3D generalisation

The paper's `(alpha0, xb, eps_h, y0, z0) -> (uy, uz)` becomes
`(alpha0, xb, eps_h, x0, y0, z0) -> (ux, uy, uz)` here (`ux`/`dx0` is the
new spread-direction displacement). A predicted local `(dx0, dy0)` is
rotated back to global `(dY, dZ)` via `LocalFrame.displacement_to_global`.
The axial component (`dz0`) is computed but **not** applied to a station's
position -- see the scope section below.

## 4. Network architecture

`lcaf/simulation/surrogate/model.py`: a plain-JAX MLP (no Flax/Optax --
see that module's docstring for why), 16 hidden layers of 300 units each
(4800 hidden neurons total, matching the paper's own stated final-model
size), `tanh` activation, linear output layer. Configurable via
`train.py`'s `--hidden-layers`/`--hidden-width`/`--activation` flags.

Inputs and outputs are z-score normalised (`preprocessing.py`) using stats
fit from the training data and saved alongside the weights in the
checkpoint, so a loaded network is never evaluated against a different
normalisation than it was trained with.

## 5. Checkpoints

A trained network is one portable `.npz` file
(`lcaf/simulation/surrogate/checkpoint.py`): weights, architecture config,
normalisation stats, and provenance metadata (paper citation, training data
description, final loss), all in one place. Written with `numpy.savez`,
read with `numpy.load(..., allow_pickle=False)` -- no pickle.
`lcaf/simulation/surrogate/trained_network_parameters/` holds checkpoint
files; see its own README for what's there (currently only a
`--dummy`-trained smoke-test fixture -- there is no real, FEA-trained
checkpoint in this repository yet).

## 6. Integration with the toolpath UI

`lcaf.toolpathing.visualization.material_state` (and everything built on
it -- `material_cross_section`, `axial_trim_allowance_mm`,
`find_sufficient_cycles`) now takes a required
`network: lcaf.simulation.surrogate.inference.SurrogateNetwork` argument.
Per strike, it calls `network.apply_strike(...)`, which:

1. Derives this strike's `(alpha0, xb, eps_h)` and local frame from the
   *current* material-state ring at the strike's own station.
2. Evaluates the network at every grid point within
   `geometry.affected_station_indices`' zone of influence (a generous
   window around the strike, scaled from the bite length -- the network
   itself is trusted to predict near-zero displacement far from where it
   struck, matching the paper's own finding that predictions "outside the
   deformation zone" are accurate).
3. Moves each point by the predicted in-plane `(dx0, dy0)` displacement,
   rotated back to global.

`lcaf.toolpathing.ui.ToolpathApp` has a "4. Surrogate deformation model"
section: a checkpoint picker (quick-pick combobox over
`trained_network_parameters/*.npz`, plus a "Browse .npz…" button for any
other file). **There is no geometric fallback** -- `_generate()` refuses to
plan a preview until a checkpoint is selected.

Consequence: `material`/`target_temperature_c` in `SliceSettings` no longer
affect the deformation preview at all (a checkpoint is trained for one
material/temperature combination, matching the paper's own scope) -- they
still drive the separate, independent slab-method force/pressure estimate
(`lcaf.toolpathing.material`), unchanged.

## 7. Scope and limitations (stated explicitly, not silently dropped)

- **Full-width-die assumption.** Training data (see
  `../JAXFEM/generate_surrogate_training_data.py`) uses a die/anvil that
  fully spans the billet's spread (Y) direction -- only the axial bite
  length is a finite footprint dimension, matching the paper's own implicit
  saddle assumption. This is also this machine's *default* die
  configuration (`die_width_mm`/`upper_die_radius_mm` default to the stock
  radius). A deliberately undersized die footprint is outside this model's
  trained domain; `die_width_mm`/`upper_die_radius_mm`/`die_corner_radius_mm`
  now only affect the *rendered* die/anvil shape in the preview, not the
  predicted deformation.
- **Rectangular-billet training assumption.** Like the paper, training
  samples are idealised rectangular slabs, not the true (possibly already
  irregular, previously-forged) cross-section a later strike actually acts
  on. This is the same "locally flat slab" approximation real forging
  process-parameter theory (Knapp's spread ratios) already makes.
- **Fixed axial station grid / unused `dz0`.** The material-state
  representation keeps a fixed set of axial (X) sample stations (matching
  the old geometric preview's own representation, so the rest of the
  preview/rendering pipeline needed no restructuring). The network's
  predicted axial (core-fibre) displacement is computed but not applied to
  move a station -- `axial_trim_allowance_mm` still estimates the
  free-end's excess length from a volume balance instead, as before.
- **No self-intersection guard.** Points can now move tangentially, not
  just radially (a more physically faithful reading of "predict a point's
  displacement" than the old radius-only heuristic) -- nothing currently
  detects or prevents two points ending up out of order after many strikes.
  Same category of known gap as the earlier `kinematic_forge` branch's own
  documented lack of self-contact detection.
- **No convergence guarantee.** Unlike the old heuristic (which
  mathematically always converges to an arbitrary target given enough
  cycles), a trained network predicts what a real strike actually does.
  `find_sufficient_cycles` keeps its "try more cycles, warn if not within
  tolerance by `max_cycles`" structure, but will legitimately warn more
  often -- which is the point: it can now honestly reveal an unrealistic
  plan instead of geometrically forcing convergence.
- **Domain extrapolation warnings.** `ProcessParameters.within_trained_domain()`
  is checked on every strike; outside the paper's own Table 1 ranges,
  `inference.SurrogateDomainWarning` fires -- the prediction is not wrong
  by definition, just unguaranteed.

## 8. Training data and training

See `docs/surrogate_training_guide.md` for the exact command sequence.
Summary: `../JAXFEM/generate_surrogate_training_data.py` (run in the
JAX-FEM conda/WSL environment) Latin-Hypercube-samples the variable space
and runs one single-strike J2-elastoplastic FEA simulation per sample,
writing `.npz` files `lcaf.simulation.surrogate.dataset` consumes;
`lcaf.simulation.surrogate.train` (`python -m
lcaf.simulation.surrogate.train`) trains a checkpoint from that data (or
`--dummy` synthetic data, for a structural smoke test with no FEA data at
all).
