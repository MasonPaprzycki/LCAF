# Training the surrogate deformation model

Step-by-step commands to generate FEA training data and train a real
checkpoint for `lcaf.simulation.surrogate`. See
`docs/surrogate_deformation_model.md` for what the model is and how it
plugs into the toolpath UI; this doc is only the command sequence.

Two separate environments are involved:

- **JAX-FEM environment** (WSL/conda, CPU or CUDA) -- runs the FEA data
  generator. Needs `jax-fem`, `petsc4py`, `gmsh`, `basix`, `meshio`.
- **Surrogate training environment** -- runs `lcaf.simulation.surrogate.train`.
  Needs only `jax`/`jaxlib` and `numpy` (already required by this repo) --
  no new dependency.

## 1. Set up the JAX-FEM environment

Follow `src/lcaf/simulation/JAXFEM/InstallProcess.ipynb` and
`SetupJaxFEM.ipynb` (already in this repo) to create the `jax-fem-env`
conda environment inside WSL. Short version:

```bash
conda create -n jax-fem-env python=3.12 -y
conda activate jax-fem-env
pip install numpy scipy matplotlib meshio gmsh pyfiglet
pip install --upgrade "jax[cpu]"
conda install -c conda-forge -y petsc petsc4py mpi4py
pip install fenics-basix
pip install git+https://github.com/deepmodeling/jax-fem.git
```

### CUDA machine: swap the JAX wheel

If the machine generating training data (or training the network) has an
NVIDIA GPU, install the CUDA build of JAX instead of `jax[cpu]` -- match the
CUDA version actually installed on that machine (check `nvidia-smi`):

```bash
pip uninstall -y jax jaxlib
pip install --upgrade "jax[cuda12]"   # or jax[cuda13], etc. -- see jax.readthedocs.io
python -c "import jax; print(jax.devices())"   # should list a CudaDevice, not just Cpu
```

Nothing in `lcaf.simulation.surrogate` or the FEA generator pins a device
explicitly -- JAX places arrays/computation on GPU automatically once the
CUDA wheel is installed.

## 2. Generate training data

From the repo root, inside the JAX-FEM environment:

```bash
cd src/lcaf/simulation/JAXFEM
python generate_surrogate_training_data.py \
    --samples 200 --seed 0 --out data/train \
    --nx 40 --ny 16 --nz 16 --n-increments 8
python generate_surrogate_training_data.py \
    --samples 54 --seed 1 --out data/test \
    --nx 40 --ny 16 --nz 16 --n-increments 8
```

The 200/54 sample counts mirror the paper's own train/test split; `--seed`
differs between the two calls so the test set's own Latin Hypercube sample
is independent of the training set's. Each run writes one `sample_NNNN.npz`
per strike into `--out`.

**Before a full run**, smoke-test with a coarse mesh and a single sample to
confirm the environment and script work end to end on your machine (this
script has been validated by construction against
`lcaf.simulation.surrogate.geometry`'s own coordinate convention, but has
not been executed against a real JAX-FEM install as part of this change --
see the script's own module docstring):

```bash
python generate_surrogate_training_data.py --samples 1 --nx 12 --ny 8 --nz 8 --out data/smoke_test
```

Inspect the resulting `data/smoke_test/sample_0000.npz` (`numpy.load` it,
check `x0`/`y0`/`z0`/`dx0`/`dy0`/`dz0` shapes match and no `nan`/`inf`
values) before committing to a full 200+54-sample run, which can take a
long time depending on mesh resolution and hardware.

## 3. Train a checkpoint

From the surrogate training environment (this repo's normal Python
environment is sufficient -- only `jax`/`numpy` are required):

```bash
python -m lcaf.simulation.surrogate.train \
    --data src/lcaf/simulation/JAXFEM/data/train \
    --val src/lcaf/simulation/JAXFEM/data/test \
    --epochs 20 \
    --out src/lcaf/simulation/surrogate/trained_network_parameters/steel_1100c_v1.npz \
    --description "42CrMo4 hot steel ~1100C, 200 train / 54 test single-strike FEA samples"
```

`--epochs 20` matches the paper's own ~20 training epochs. Increase
`--batch-size`/adjust `--learning-rate` if training loss plateaus early or
oscillates; `--hidden-layers`/`--hidden-width`/`--activation` change the
network architecture (defaults: 16 layers x 300 units, `tanh` -- see
`docs/surrogate_deformation_model.md` section 4).

### Smoke-testing the training pipeline without FEA data

`--dummy` generates a small synthetic dataset in memory and trains on it --
useful for confirming the training/checkpointing pipeline itself works
(this is how `trained_network_parameters/dummy_smoke_test.npz`, the fixture
committed to this repo, was produced) without needing any FEA data at all:

```bash
python -m lcaf.simulation.surrogate.train --dummy --dummy-samples 2000 --epochs 8
```

The resulting checkpoint is **not physically meaningful** -- it is trained
on a hand-picked synthetic "displacement field," not real FEA data. Its
`meta_is_dummy_smoke_test` field (and the UI's status line, once loaded) say
so explicitly.

## 4. Use the trained checkpoint

Launch the toolpath UI (`python -m lcaf.toolpathing`), open section "4.
Surrogate deformation model," and either pick the new checkpoint from the
quick-pick combobox (if it's inside
`lcaf/simulation/surrogate/trained_network_parameters/`) or use "Browse
.npz…" to point at it anywhere else. The animated preview (2D and 3D tabs)
uses it immediately; "Generate preview" is blocked until a checkpoint is
selected.

## 5. Running the automated tests

```bash
python -m pytest debug/tests/test_surrogate.py debug/tests/test_toolpath_slicer.py -q
```

These use small, freshly initialised (not pretrained) networks -- they do
not require the JAX-FEM environment, FEA training data, or GPU hardware,
and run in a couple of seconds.
