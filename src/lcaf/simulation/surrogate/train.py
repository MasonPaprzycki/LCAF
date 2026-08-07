"""Train the JAGTAP-et-al.-style displacement surrogate.

Plain-JAX training loop (hand-rolled Adam -- see the module docstring in
``model.py`` for why no Flax/Optax dependency is used) over data produced by
``../JAXFEM/generate_surrogate_training_data.py``. Designed to run
unchanged on a CUDA machine: JAX places arrays/computation on whatever
device ``jax.devices()`` reports (GPU automatically, if ``jax[cuda]`` is the
installed wheel -- see ``docs/surrogate_training_guide.md``), nothing here
pins a device explicitly.

``--dummy`` generates a small synthetic dataset in memory instead of reading
FEA ``.npz`` samples, so the whole pipeline (data loading, normalisation,
training, checkpointing, inference) can be smoke-tested end to end without
JAX-FEM installed -- the synthetic "displacement field" is a smooth,
plausible-looking bump, not a physically calibrated one; it exists to prove
the code works, not to produce a usable model. See
``docs/surrogate_training_guide.md`` for the real, FEA-data-driven workflow.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import jax
import jax.numpy as jnp
import numpy as np

from . import checkpoint as checkpoint_module
from . import dataset as dataset_module
from . import preprocessing
from .model import ArchitectureConfig, Params, forward, init_params
from .process_params import PAPER_ALPHA0_RANGE, PAPER_EPS_H_RANGE, PAPER_XB_RANGE, REFERENCE_HEIGHT_MM

_DEFAULT_DUMMY_CHECKPOINT = (
    Path(__file__).resolve().parent / "trained_network_parameters" / "dummy_smoke_test.npz"
)


def _synthetic_dataset(n_samples: int, seed: int) -> dataset_module.Dataset:
    """A smooth, structurally-plausible-but-not-physical synthetic dataset.

    Built directly from the paper's own variable-space ranges
    (``process_params.PAPER_*_RANGE``) and reference height so the
    synthetic inputs at least *look like* real strikes, then passed through
    a hand-picked Gaussian-bump displacement field that decays away from
    the die contact region and scales with reduction -- enough structure
    for a training loop to visibly reduce loss on, nothing more. Never used
    for anything but the ``--dummy`` smoke test.
    """
    rng = np.random.default_rng(seed)
    alpha0 = rng.uniform(*PAPER_ALPHA0_RANGE, n_samples)
    xb = rng.uniform(*PAPER_XB_RANGE, n_samples)
    eps_h = rng.uniform(*PAPER_EPS_H_RANGE, n_samples)

    h0 = REFERENCE_HEIGHT_MM
    bite_mm = xb * h0
    reduction_mm = eps_h * h0

    x0 = rng.uniform(-h0, h0, n_samples)
    y0 = rng.uniform(0.0, h0, n_samples)
    z0 = rng.uniform(-h0, 2.0 * h0, n_samples)

    axial_decay = np.exp(-((z0 - bite_mm / 2.0) ** 2) / (2.0 * bite_mm**2 + 1e-6))
    height_decay = np.exp(-((y0 - h0) ** 2) / (2.0 * h0**2))
    bump = axial_decay * height_decay

    dy0 = -reduction_mm * bump * (y0 / h0)
    dx0 = 0.3 * reduction_mm * bump * np.tanh(x0 / (0.3 * h0))
    dz0 = 0.2 * reduction_mm * bump * np.tanh((z0 - bite_mm / 2.0) / (0.5 * bite_mm + 1e-6))

    inputs = np.stack([alpha0, xb, eps_h, x0, y0, z0], axis=1)
    outputs = np.stack([dx0, dy0, dz0], axis=1)
    sample_id = np.full(n_samples, "dummy")
    return dataset_module.Dataset(inputs=inputs, outputs=outputs, sample_id=sample_id)


def init_adam_state(params: Params) -> dict:
    zeros = jax.tree_util.tree_map(jnp.zeros_like, params)
    return {"m": zeros, "v": jax.tree_util.tree_map(jnp.zeros_like, params), "t": 0}


def _adam_update(params: Params, grads: Params, state: dict, learning_rate: float) -> tuple[Params, dict]:
    beta1, beta2, epsilon = 0.9, 0.999, 1e-8
    t = state["t"] + 1
    m = jax.tree_util.tree_map(lambda moment, g: beta1 * moment + (1.0 - beta1) * g, state["m"], grads)
    v = jax.tree_util.tree_map(lambda moment, g: beta2 * moment + (1.0 - beta2) * (g**2), state["v"], grads)
    m_hat = jax.tree_util.tree_map(lambda moment: moment / (1.0 - beta1**t), m)
    v_hat = jax.tree_util.tree_map(lambda moment: moment / (1.0 - beta2**t), v)
    new_params = jax.tree_util.tree_map(
        lambda p, mh, vh: p - learning_rate * mh / (jnp.sqrt(vh) + epsilon), params, m_hat, v_hat
    )
    return new_params, {"m": m, "v": v, "t": t}


def _make_train_step(activation: str, learning_rate: float):
    def loss_fn(params: Params, x: jnp.ndarray, y: jnp.ndarray) -> jnp.ndarray:
        prediction = forward(params, x, activation=activation)
        return jnp.mean((prediction - y) ** 2)

    grad_fn = jax.value_and_grad(loss_fn)

    @jax.jit
    def train_step(params: Params, opt_state: dict, x: jnp.ndarray, y: jnp.ndarray):
        loss, grads = grad_fn(params, x, y)
        new_params, new_opt_state = _adam_update(params, grads, opt_state, learning_rate)
        return new_params, new_opt_state, loss

    @jax.jit
    def eval_loss(params: Params, x: jnp.ndarray, y: jnp.ndarray) -> jnp.ndarray:
        return loss_fn(params, x, y)

    return train_step, eval_loss


def train(
    train_data: dataset_module.Dataset,
    val_data: dataset_module.Dataset,
    architecture: ArchitectureConfig,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    seed: int,
) -> tuple[Params, preprocessing.NormalizationStats, dict[str, list[float]]]:
    """Run the training loop, returning trained params, fitted normalisation
    stats, and a per-epoch history of (train_loss, val_loss) for logging.
    """
    stats = preprocessing.fit(train_data.inputs, train_data.outputs)
    train_x = jnp.asarray(preprocessing.normalize_inputs(stats, train_data.inputs))
    train_y = jnp.asarray(preprocessing.normalize_outputs(stats, train_data.outputs))
    val_x = jnp.asarray(preprocessing.normalize_inputs(stats, val_data.inputs))
    val_y = jnp.asarray(preprocessing.normalize_outputs(stats, val_data.outputs))

    params = init_params(architecture, seed)
    opt_state = init_adam_state(params)
    train_step, eval_loss = _make_train_step(architecture.activation, learning_rate)

    rng = np.random.default_rng(seed)
    n_train = train_x.shape[0]
    batch_size = max(1, min(batch_size, n_train))
    history: dict[str, list[float]] = {"train_loss": [], "val_loss": []}

    for epoch in range(epochs):
        order = rng.permutation(n_train)
        epoch_losses = []
        for start in range(0, n_train, batch_size):
            batch_indices = order[start : start + batch_size]
            params, opt_state, loss = train_step(params, opt_state, train_x[batch_indices], train_y[batch_indices])
            epoch_losses.append(float(loss))
        train_loss = float(np.mean(epoch_losses))
        val_loss = float(eval_loss(params, val_x, val_y))
        history["train_loss"].append(train_loss)
        history["val_loss"].append(val_loss)
        print(f"  epoch {epoch + 1:>4d}/{epochs}  train_loss={train_loss:.6f}  val_loss={val_loss:.6f}")

    return params, stats, history


def main(argv: list[str] | None = None) -> Path:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=None, help="Directory of training .npz samples.")
    parser.add_argument("--val", type=Path, default=None, help="Directory of held-out validation .npz samples.")
    parser.add_argument("--val-fraction", type=float, default=0.15, help="Used only when --val is omitted.")
    parser.add_argument("--dummy", action="store_true", help="Smoke-test with synthetic in-memory data instead of --data.")
    parser.add_argument("--dummy-samples", type=int, default=2000)
    parser.add_argument("--epochs", type=int, default=20, help="Matches the paper's own ~20 training epochs.")
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--hidden-layers", type=int, default=16)
    parser.add_argument("--hidden-width", type=int, default=300)
    parser.add_argument("--activation", type=str, default="tanh", choices=("tanh", "relu", "gelu"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=Path, default=None, help="Checkpoint output path (.npz).")
    parser.add_argument("--description", type=str, default="", help="Free-text note stored in the checkpoint metadata.")
    args = parser.parse_args(argv)

    if not args.dummy and args.data is None:
        parser.error("--data is required unless --dummy is set.")

    print(f"JAX devices: {jax.devices()}")

    if args.dummy:
        full_dataset = _synthetic_dataset(args.dummy_samples, args.seed)
        train_data, val_data = dataset_module.train_val_split(full_dataset, args.val_fraction, args.seed)
        data_description = f"synthetic --dummy data ({args.dummy_samples} samples), NOT physically meaningful"
        out_path = args.out or _DEFAULT_DUMMY_CHECKPOINT
    else:
        if args.val is not None:
            train_data = dataset_module.load_directory(args.data)
            val_data = dataset_module.load_directory(args.val)
            data_description = f"train={args.data}, val={args.val}"
        else:
            full_dataset = dataset_module.load_directory(args.data)
            train_data, val_data = dataset_module.train_val_split(full_dataset, args.val_fraction, args.seed)
            data_description = f"train/val split of {args.data} (val_fraction={args.val_fraction})"
        if args.out is None:
            parser.error("--out is required unless --dummy is set.")
        out_path = args.out

    architecture = ArchitectureConfig(
        hidden_layers=args.hidden_layers, hidden_width=args.hidden_width, activation=args.activation
    )
    print(
        f"Training on {len(train_data)} nodes ({data_description}), "
        f"validating on {len(val_data)} nodes, architecture={architecture}"
    )

    started = time.time()
    params, stats, history = train(
        train_data, val_data, architecture, args.epochs, args.batch_size, args.learning_rate, args.seed
    )
    elapsed = time.time() - started
    print(f"Training finished in {elapsed:.1f}s.")

    metadata = {
        "description": args.description or data_description,
        "epochs": str(args.epochs),
        "final_train_loss": f"{history['train_loss'][-1]:.6f}",
        "final_val_loss": f"{history['val_loss'][-1]:.6f}",
        "is_dummy_smoke_test": str(args.dummy),
    }
    saved_path = checkpoint_module.save(out_path, params, architecture, stats, metadata)
    print(f"Saved checkpoint: {saved_path}")
    return saved_path


if __name__ == "__main__":
    main()
