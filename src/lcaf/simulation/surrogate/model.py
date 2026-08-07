"""The displacement-prediction network: a plain-JAX MLP.

Jagtap et al.'s final model is "16 layers with a total of 4800 neurons"
mapping ``(process parameters, reference point) -> displacement`` on a 2D
(y, z) cross-section. This module is that same feed-forward architecture --
default 16 hidden layers of 300 units each (4800 hidden neurons, matching
the paper's own final-model size), a single linear output layer -- with one
extra input unit (the spread-direction coordinate ``x0``) and one extra
output unit (the spread-direction displacement ``dx0``), which is the whole
2D -> 3D generalisation (see ``docs/surrogate_deformation_model.md``).

Deliberately plain JAX (a pytree of per-layer weight/bias arrays), not
Flax/Haiku/Equinox: none of those are installed anywhere in this repo (the
JAX-FEM conda environments in ``../JAXFEM/environment.yml`` and the
notebooks only ever install ``jax``/``jaxlib``), and a plain MLP forward
pass is short enough that hand-writing it avoids a new dependency the CUDA
training machine would also need. ``forward`` is written to batch over a
leading ``(N, ...)`` axis via ordinary matrix multiplication, so it needs no
``vmap`` and JIT-compiles (and places on GPU, if ``jax[cuda]`` is installed)
exactly the way any other JAX function does.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import jax
import jax.numpy as jnp

INPUT_DIM = 6   # (alpha0, xb, eps_h, x0, y0, z0) -- see geometry.py
OUTPUT_DIM = 3  # (dx0, dy0, dz0) local displacement

Params = list[dict[str, jnp.ndarray]]


@dataclass(frozen=True)
class ArchitectureConfig:
    """Hyperparameters describing one network's shape -- saved alongside its
    weights in a checkpoint (see ``checkpoint.py``) so a checkpoint is
    self-describing and does not need the caller to already know its shape.
    """

    hidden_layers: int = 16
    hidden_width: int = 300
    activation: str = "tanh"

    def layer_sizes(self) -> tuple[int, ...]:
        return (INPUT_DIM,) + (self.hidden_width,) * self.hidden_layers + (OUTPUT_DIM,)


def _activation_fn(name: str):
    if name == "tanh":
        return jnp.tanh
    if name == "relu":
        return jax.nn.relu
    if name == "gelu":
        return jax.nn.gelu
    raise ValueError(f"Unknown activation '{name}'. Choose one of: tanh, relu, gelu.")


def init_params(config: ArchitectureConfig, seed: int) -> Params:
    """Glorot/Xavier-uniform initialised weights, zero biases.

    Glorot bounds (``sqrt(6 / (fan_in + fan_out))``) are the standard choice
    for a ``tanh``-activated network -- keeps activations from saturating in
    the first forward pass regardless of depth, which matters here at 16
    layers deep.
    """
    key = jax.random.PRNGKey(seed)
    sizes = config.layer_sizes()
    params: Params = []
    for fan_in, fan_out in zip(sizes[:-1], sizes[1:]):
        key, weight_key = jax.random.split(key)
        limit = float(jnp.sqrt(6.0 / (fan_in + fan_out)))
        weight = jax.random.uniform(weight_key, (fan_in, fan_out), minval=-limit, maxval=limit)
        bias = jnp.zeros((fan_out,))
        params.append({"w": weight, "b": bias})
    return params


def forward(params: Params, x: jnp.ndarray, activation: str = "tanh") -> jnp.ndarray:
    """Batched forward pass: ``x`` is ``(N, INPUT_DIM)`` -> ``(N, OUTPUT_DIM)``.

    Every hidden layer applies ``activation``; the final layer is linear
    (a displacement can be any sign/magnitude, so it is never squashed).
    """
    act = _activation_fn(activation)
    hidden = x
    for layer in params[:-1]:
        hidden = act(hidden @ layer["w"] + layer["b"])
    output_layer = params[-1]
    return hidden @ output_layer["w"] + output_layer["b"]


# A JIT-compiled variant of ``forward``, used by
# ``lcaf.simulation.surrogate.inference.SurrogateNetwork`` for interactive use
# (the UI preview re-evaluates the network on every animation frame during
# playback). JAX caches one compiled executable per distinct (params shape,
# x shape, activation) combination it is called with -- since a loaded
# checkpoint's params shape never changes and the same handful of batch
# sizes (a station's full angular ring, most commonly) recur across an
# animation, this makes every call after the first at a given batch size
# roughly two orders of magnitude faster than re-tracing ``forward`` in plain
# Python each time (measured ~140ms first call vs. ~1.5ms uncached-repeat vs.
# sub-millisecond once genuinely cached). ``activation`` is marked static
# since it selects which Python function runs inside the traced graph, not
# an array JAX can trace over.
forward_jit = jax.jit(forward, static_argnames=("activation",))


def flatten_params(params: Params) -> dict[str, jnp.ndarray]:
    """Params -> a flat ``{"w0": ..., "b0": ..., "w1": ..., ...}`` dict, for ``.npz`` storage."""
    flat: dict[str, jnp.ndarray] = {}
    for index, layer in enumerate(params):
        flat[f"w{index}"] = layer["w"]
        flat[f"b{index}"] = layer["b"]
    return flat


def unflatten_params(flat: dict[str, "jnp.ndarray | object"], num_layers: int) -> Params:
    """Inverse of ``flatten_params``."""
    return [{"w": jnp.asarray(flat[f"w{index}"]), "b": jnp.asarray(flat[f"b{index}"])} for index in range(num_layers)]


def count_parameters(params: Params) -> int:
    return int(sum(layer["w"].size + layer["b"].size for layer in params))


def param_leaves(params: Params) -> Sequence[jnp.ndarray]:
    """A flat list of every array in ``params``, in a stable order -- used by
    ``train.py``'s hand-rolled optimiser, which needs to walk the pytree
    without depending on ``jax.tree_util``'s dict key ordering guarantees
    across JAX versions.
    """
    leaves: list[jnp.ndarray] = []
    for layer in params:
        leaves.append(layer["w"])
        leaves.append(layer["b"])
    return leaves
