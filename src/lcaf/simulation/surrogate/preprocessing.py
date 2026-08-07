"""Input/output normalisation, fit once from training data and applied
identically at train and inference time.

The six network inputs span wildly different scales (dimensionless process
parameters around 0-1 vs. millimetre coordinates that can be tens to
hundreds of millimetres), and displacement targets are typically a couple of
orders of magnitude smaller than the coordinates that produce them. Feeding
an MLP unnormalised inputs of such different scales makes the first layer's
weights do double duty as an implicit, untrained rescaling -- standard
practice (and necessary here) is to fit a z-score per input/output channel
from the training data and carry those same stats through to inference,
saved alongside the weights in the checkpoint (see ``checkpoint.py``) so a
loaded network is never evaluated against a different normalisation than it
was trained with.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

_MIN_STD = 1e-8


@dataclass(frozen=True)
class NormalizationStats:
    """Per-channel mean/std for the 6 inputs and 3 outputs (see ``model.py``)."""

    input_mean: np.ndarray
    input_std: np.ndarray
    output_mean: np.ndarray
    output_std: np.ndarray

    def __post_init__(self) -> None:
        for name, array, expected_len in (
            ("input_mean", self.input_mean, 6),
            ("input_std", self.input_std, 6),
            ("output_mean", self.output_mean, 3),
            ("output_std", self.output_std, 3),
        ):
            if array.shape != (expected_len,):
                raise ValueError(f"{name} must have shape ({expected_len},), got {array.shape}.")


def fit(inputs: np.ndarray, outputs: np.ndarray) -> NormalizationStats:
    """Fit per-channel mean/std from raw ``(N, 6)`` inputs and ``(N, 3)`` outputs.

    ``_MIN_STD`` floors every std so a constant (zero-variance) channel --
    for example a dummy dataset that never varies ``eps_h`` -- normalises to
    a finite, if uninformative, value instead of dividing by zero.
    """
    input_mean = inputs.mean(axis=0)
    input_std = np.maximum(inputs.std(axis=0), _MIN_STD)
    output_mean = outputs.mean(axis=0)
    output_std = np.maximum(outputs.std(axis=0), _MIN_STD)
    return NormalizationStats(input_mean, input_std, output_mean, output_std)


def normalize_inputs(stats: NormalizationStats, inputs: np.ndarray) -> np.ndarray:
    return (inputs - stats.input_mean) / stats.input_std


def normalize_outputs(stats: NormalizationStats, outputs: np.ndarray) -> np.ndarray:
    return (outputs - stats.output_mean) / stats.output_std


def denormalize_outputs(stats: NormalizationStats, normalized_outputs: np.ndarray) -> np.ndarray:
    return normalized_outputs * stats.output_std + stats.output_mean
