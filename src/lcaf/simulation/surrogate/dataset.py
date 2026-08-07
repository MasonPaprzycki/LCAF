"""Load JAX-FEM-generated single-strike training samples into flat arrays.

Each sample ``.npz`` (written by
``../JAXFEM/generate_surrogate_training_data.py``) holds one forging
stroke's worth of FEA ground truth, already expressed in the same local
``(x0, y0, z0)`` frame ``geometry.py``'s runtime path produces (see that
module and ``docs/surrogate_deformation_model.md``) -- no coordinate
transform needed here, only concatenation across samples and the
process-parameter broadcast the paper's own architecture needs (Fig. 3: one
``(alpha0, xb, eps_h)`` input block shared by every node of one stroke).

Expected per-sample ``.npz`` keys::

    alpha0, xb, eps_h   scalars -- this sample's process parameters
    x0, y0, z0          (n_nodes,) reference-configuration local coords, mm
    dx0, dy0, dz0       (n_nodes,) displacement components, mm
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

_REQUIRED_KEYS = ("alpha0", "xb", "eps_h", "x0", "y0", "z0", "dx0", "dy0", "dz0")


@dataclass(frozen=True)
class Dataset:
    """Flat, network-ready training data."""

    inputs: np.ndarray    # (N, 6): alpha0, xb, eps_h, x0, y0, z0
    outputs: np.ndarray   # (N, 3): dx0, dy0, dz0
    sample_id: np.ndarray  # (N,) source file stem per row, for diagnostics

    def __len__(self) -> int:
        return int(self.inputs.shape[0])

    def subset(self, indices: np.ndarray) -> "Dataset":
        return Dataset(self.inputs[indices], self.outputs[indices], self.sample_id[indices])


def load_sample(path: str | Path) -> Dataset:
    """Load one FEA sample ``.npz`` into a single-sample ``Dataset``."""
    path = Path(path)
    with np.load(path) as data:
        missing = [key for key in _REQUIRED_KEYS if key not in data]
        if missing:
            raise ValueError(f"{path} is missing required keys: {missing}")

        n_nodes = int(data["x0"].shape[0])
        for key in ("y0", "z0", "dx0", "dy0", "dz0"):
            if int(data[key].shape[0]) != n_nodes:
                raise ValueError(
                    f"{path}: '{key}' has {data[key].shape[0]} rows, expected {n_nodes} (matching 'x0')."
                )

        process = np.array([float(data["alpha0"]), float(data["xb"]), float(data["eps_h"])])
        process_broadcast = np.tile(process, (n_nodes, 1))
        coords = np.stack([data["x0"], data["y0"], data["z0"]], axis=1)
        inputs = np.concatenate([process_broadcast, coords], axis=1)
        outputs = np.stack([data["dx0"], data["dy0"], data["dz0"]], axis=1)
        sample_id = np.full(n_nodes, path.stem)

    return Dataset(inputs=inputs, outputs=outputs, sample_id=sample_id)


def load_directory(directory: str | Path) -> Dataset:
    """Concatenate every ``*.npz`` sample in ``directory`` into one ``Dataset``."""
    directory = Path(directory)
    paths = sorted(directory.glob("*.npz"))
    if not paths:
        raise FileNotFoundError(f"No .npz training samples found in {directory}.")
    samples = [load_sample(path) for path in paths]
    return Dataset(
        inputs=np.concatenate([sample.inputs for sample in samples], axis=0),
        outputs=np.concatenate([sample.outputs for sample in samples], axis=0),
        sample_id=np.concatenate([sample.sample_id for sample in samples], axis=0),
    )


def train_val_split(dataset: Dataset, val_fraction: float = 0.15, seed: int = 0) -> tuple[Dataset, Dataset]:
    """A random, seeded train/validation split at the *node* level.

    The paper instead splits at the *simulation* level (200 training sims,
    54 held-out test sims, run through completely separate FEA jobs) so
    that no test node's own process parameters were ever seen in training.
    For real training data (produced sample-by-sample by
    ``generate_surrogate_training_data.py``), callers should reproduce that
    by pointing ``load_directory`` at separate ``train/``/``test/``
    directories rather than relying on this function to hold out whole
    simulations. This node-level split exists for quick local validation
    (including the ``--dummy`` smoke test) where a held-out simulation set
    is not the point.
    """
    if not 0.0 < val_fraction < 1.0:
        raise ValueError("val_fraction must be in (0, 1).")
    rng = np.random.default_rng(seed)
    n = len(dataset)
    indices = rng.permutation(n)
    n_val = max(1, int(round(n * val_fraction)))
    val_indices, train_indices = indices[:n_val], indices[n_val:]
    return dataset.subset(train_indices), dataset.subset(val_indices)
