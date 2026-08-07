# Trained network parameters

Each `.npz` file here is one trained surrogate checkpoint, loadable with
`lcaf.simulation.surrogate.inference.SurrogateNetwork.load(path)` and
selectable from the LCAF toolpathing UI's "Surrogate model" picker.

## Format

Written/read by `../checkpoint.py`. A checkpoint is self-describing:

- `w0, b0, w1, b1, ...` -- per-layer weight/bias arrays.
- `format_version`, `hidden_layers`, `hidden_width`, `activation` --
  architecture (see `../model.py`).
- `input_mean`, `input_std`, `output_mean`, `output_std` -- normalisation
  stats fit from the training data (see `../preprocessing.py`).
- `meta_*` -- provenance metadata (paper citation, training data
  description, final train/val loss, etc.), all stored as strings.

Written with `numpy.savez`/read with `numpy.load(..., allow_pickle=False)`
-- no pickle, safe to load from an untrusted source.

## Files

- **`dummy_smoke_test.npz`** -- produced by `../train.py --dummy`, trained
  on synthetic (not physically meaningful) data purely to exercise the
  full pipeline end to end: data loading, normalisation, training,
  checkpointing, and the UI/inference path. **Do not use this for anything
  but testing the code.** Its `meta_is_dummy_smoke_test` field is `"True"`.

There is no real, FEA-trained checkpoint in this repository yet. Generate
one following `docs/surrogate_training_guide.md` (repository root `docs/`):
run `../../JAXFEM/generate_surrogate_training_data.py` on a machine with
JAX-FEM installed, then `../train.py` on the resulting data (ideally on a
CUDA machine, but any JAX install works).
