# Surrogate deformation model

This package is LCAF's toolpath-preview deformation engine: a feed-forward
neural network that predicts per-node displacement for one open-die forging
stroke, replacing the earlier pure-computational-geometry preview in
`lcaf.toolpathing.visualization`.

## Citation

This implements the technique described in:

> Jagtap, N. V., Reinisch, N., & Bailly, D. (2024). *Fast prediction of the
> material displacement in open die forging using neural networks.*
> Material Forming – ESAFORM 2024, Materials Research Proceedings 41,
> 2299–2308. https://doi.org/10.21741/9781644903131-253

Published by Materials Research Forum LLC under a Creative Commons
Attribution 3.0 license (CC BY 3.0) — reproduction/adaptation is permitted
with attribution, which this citation and the docstrings throughout this
package provide. This is a from-scratch reimplementation (plain JAX; the
paper's own reference implementation was MATLAB), not a copy of any of the
paper's code or data.

## What is adopted as-is vs. changed

**Adopted directly from the paper:**

- The core idea: a feed-forward MLP mapping `(process parameters,
  reference-configuration point) -> displacement`, trained on FEA data,
  evaluated per node.
- The three process parameters -- aspect ratio `alpha0`, bite ratio `xb`,
  height reduction `eps_h` -- and their formulas (Eq. 1-3 in the paper).
- The paper's own variable space (Table 1: `alpha0` in [0.9, 1.5], `xb` in
  [0.3, 0.9], `eps_h` in [0.05, 0.26]) as this implementation's default
  training-data sampling range.
- The "reposition the reference configuration to the die's leading edge"
  trick that makes one trained network usable for any stroke in a pass
  schedule, not just the one it was trained on.
- The final architecture's rough size: 16 hidden layers, ~4800 hidden
  neurons total (see `model.py`).
- Latin Hypercube sampling of the variable space, and a train/test split in
  the same spirit as the paper's 200 training / 54 test simulations.

**The one deliberate change: 2D -> 3D.** The paper restricts itself to a 2D
plane through the core fibre (the `yz` cross-section at the workpiece's
mid-width) and predicts 2 displacement components. This package adds the
spread/width coordinate `x0` as a third network input and the corresponding
displacement `dx0` as a third output -- literally the paper's own stated
future work ("an extension of the 2D model to predict 3D material flow
seems possible"). See `docs/surrogate_deformation_model.md` (repository
root) for the full derivation, the mapping onto this machine's actual
geometry, and an explicit list of scope limitations.

## Layout

- `process_params.py` -- process-parameter formulas + Latin Hypercube
  sampling (no JAX/mesh dependency; shared by training-data generation and
  runtime inference).
- `geometry.py` -- global (machine) <-> local (per-strike) coordinate frame.
- `model.py` -- the MLP architecture (plain JAX).
- `preprocessing.py` -- input/output normalisation.
- `checkpoint.py` -- `.npz` save/load.
- `dataset.py` -- load FEA-generated training samples.
- `train.py` -- training CLI (`python -m lcaf.simulation.surrogate.train`).
- `inference.py` -- `SurrogateNetwork`, the runtime prediction/strike API.
- `trained_network_parameters/` -- checkpoint files (see its own README).

See `docs/surrogate_training_guide.md` (repository root `docs/`) for the
end-to-end, copy-pasteable command list to generate training data and train
a real checkpoint on a CUDA machine.
