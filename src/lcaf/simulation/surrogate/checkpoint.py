"""Save/load a trained network to a single portable ``.npz`` file.

A checkpoint is self-describing: architecture shape, normalisation stats,
and provenance metadata travel with the weights, so
``trained_network_parameters/some_model.npz`` is everything
``inference.SurrogateNetwork`` needs, and nothing about how it was trained
has to be remembered separately or hard-coded at the call site. Deliberately
plain ``numpy.savez``/``numpy.load`` with ``allow_pickle=False`` -- no
pickle, so a checkpoint is safe to load from an untrusted source and portable
across the CUDA training machine and this repo's own CPU sandbox without any
extra serialisation dependency.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .model import ArchitectureConfig, Params, flatten_params, unflatten_params
from .preprocessing import NormalizationStats

CHECKPOINT_FORMAT_VERSION = 1

PAPER_CITATION = (
    "Jagtap, N. V., Reinisch, N., & Bailly, D. (2024). Fast prediction of the "
    "material displacement in open die forging using neural networks. "
    "Materials Research Proceedings, 41, 2299-2308. "
    "https://doi.org/10.21741/9781644903131-253"
)


@dataclass(frozen=True)
class Checkpoint:
    params: Params
    architecture: ArchitectureConfig
    stats: NormalizationStats
    metadata: dict[str, str] = field(default_factory=dict)


def save(
    path: str | Path,
    params: Params,
    architecture: ArchitectureConfig,
    stats: NormalizationStats,
    metadata: dict[str, str] | None = None,
) -> Path:
    """Write ``params``/``architecture``/``stats``/``metadata`` to one ``.npz``."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    arrays: dict[str, np.ndarray] = dict(flatten_params(params))
    arrays["format_version"] = np.array(CHECKPOINT_FORMAT_VERSION)
    arrays["hidden_layers"] = np.array(architecture.hidden_layers)
    arrays["hidden_width"] = np.array(architecture.hidden_width)
    arrays["activation"] = np.array(architecture.activation)
    arrays["input_mean"] = np.asarray(stats.input_mean)
    arrays["input_std"] = np.asarray(stats.input_std)
    arrays["output_mean"] = np.asarray(stats.output_mean)
    arrays["output_std"] = np.asarray(stats.output_std)

    full_metadata = {"paper_citation": PAPER_CITATION, **(metadata or {})}
    for key, value in full_metadata.items():
        arrays[f"meta_{key}"] = np.array(str(value))

    np.savez(path, **arrays)
    return path


def load(path: str | Path) -> Checkpoint:
    """Read a checkpoint written by ``save``."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"No surrogate checkpoint at {path}.")

    with np.load(path, allow_pickle=False) as data:
        format_version = int(data["format_version"])
        if format_version != CHECKPOINT_FORMAT_VERSION:
            raise ValueError(
                f"Checkpoint {path} has format_version={format_version}, this code "
                f"expects {CHECKPOINT_FORMAT_VERSION}."
            )

        architecture = ArchitectureConfig(
            hidden_layers=int(data["hidden_layers"]),
            hidden_width=int(data["hidden_width"]),
            activation=str(data["activation"].item()),
        )
        params = unflatten_params(
            {name: data[name] for name in data.files if name.startswith(("w", "b"))},
            num_layers=architecture.hidden_layers + 1,
        )
        stats = NormalizationStats(
            input_mean=np.asarray(data["input_mean"]),
            input_std=np.asarray(data["input_std"]),
            output_mean=np.asarray(data["output_mean"]),
            output_std=np.asarray(data["output_std"]),
        )
        metadata = {
            name[len("meta_"):]: str(data[name].item())
            for name in data.files
            if name.startswith("meta_")
        }

    return Checkpoint(params=params, architecture=architecture, stats=stats, metadata=metadata)
