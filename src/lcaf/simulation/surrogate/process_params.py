"""Pure math for open-die forging process parameters.

Jagtap, Reinisch & Bailly ("Fast prediction of the material displacement in
open die forging using neural networks," ESAFORM 2024, Materials Research
Proceedings 41, 2299-2308, DOI 10.21741/9781644903131-253) parameterise one
forging stroke by three dimensionless scalars -- aspect ratio, bite ratio,
height reduction -- computed from the workpiece's own pre-stroke geometry.
This module is the single source of truth for those three formulas (and
their inverse, sampling a billet geometry from a point in the paper's
variable space), shared by:

- ``geometry.py`` -- the runtime path, deriving process parameters from a
  live ``lcaf.toolpathing`` plan's own segment/operation geometry.
- ``../JAXFEM/generate_surrogate_training_data.py`` -- the offline path,
  building synthetic rectangular-billet FEA training samples from a Latin
  Hypercube sample of the same variable space.

Keeping both paths on one set of formulas means the network is trained on
exactly the quantities it is fed at inference time -- no separate
reimplementation to drift out of sync.

Nothing here touches JAX, a mesh, or any ``lcaf.toolpathing`` type: every
function is plain float/array math so it can be imported from a bare
``numpy`` environment (this repo's default Windows sandbox) as well as the
WSL/conda ``jax-fem-env`` the FEA data generator actually runs in.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# The paper's own variable space (Table 1) -- the domain the reference
# model was actually trained and validated over. Used both as the default
# Latin Hypercube sampling range for the FEA data generator and as the
# "is this strike within the trained domain" check at inference time.
PAPER_ALPHA0_RANGE: tuple[float, float] = (0.9, 1.5)
PAPER_XB_RANGE: tuple[float, float] = (0.3, 0.9)
PAPER_EPS_H_RANGE: tuple[float, float] = (0.05, 0.26)

# The paper fixes workpiece height at 100 mm for its square (alpha0=1)
# samples; this module fixes h0 at that same physical scale for every
# sample, deriving w0 = h0 / alpha0, so absolute displacement magnitudes
# the network learns are anchored to a consistent physical scale rather
# than a dimensionless one the FEA solver would otherwise have no basis to
# distinguish from unit scaling.
REFERENCE_HEIGHT_MM: float = 100.0


@dataclass(frozen=True)
class ProcessParameters:
    """The three dimensionless scalars one forging stroke is parameterised by."""

    alpha0: float
    xb: float
    eps_h: float

    def as_array(self) -> np.ndarray:
        return np.array([self.alpha0, self.xb, self.eps_h], dtype=np.float64)

    def within_trained_domain(
        self,
        alpha0_range: tuple[float, float] = PAPER_ALPHA0_RANGE,
        xb_range: tuple[float, float] = PAPER_XB_RANGE,
        eps_h_range: tuple[float, float] = PAPER_EPS_H_RANGE,
    ) -> bool:
        """Whether this stroke falls inside the variable space training data covers.

        A network extrapolating outside the ranges it was trained on has no
        accuracy guarantee at all (see the paper's own discussion of
        generalisation) -- callers should surface this as a warning rather
        than silently trusting an extrapolated prediction.
        """
        return (
            alpha0_range[0] <= self.alpha0 <= alpha0_range[1]
            and xb_range[0] <= self.xb <= xb_range[1]
            and eps_h_range[0] <= self.eps_h <= eps_h_range[1]
        )


def aspect_ratio(h0_mm: float, w0_mm: float) -> float:
    """alpha0 = h0 / w0 (Eq. 1) -- ratio of press-direction height to spread-direction width."""
    if w0_mm <= 0.0:
        raise ValueError("w0_mm must be positive.")
    return h0_mm / w0_mm


def bite_ratio(bite_length_mm: float, h0_mm: float) -> float:
    """xb = b / h0 (Eq. 2) -- axial die contact length over pre-stroke height."""
    if h0_mm <= 0.0:
        raise ValueError("h0_mm must be positive.")
    return bite_length_mm / h0_mm


def height_reduction(h0_mm: float, reduction_mm: float) -> float:
    """eps_h = (h0 - h1) / h0 (Eq. 3), given the reduction h0 - h1 directly."""
    if h0_mm <= 0.0:
        raise ValueError("h0_mm must be positive.")
    return reduction_mm / h0_mm


def height_reduction_from_heights(h0_mm: float, h1_mm: float) -> float:
    """eps_h = (h0 - h1) / h0 (Eq. 3), given both heights directly."""
    return height_reduction(h0_mm, h0_mm - h1_mm)


def billet_dimensions_mm(alpha0: float, h0_mm: float = REFERENCE_HEIGHT_MM) -> tuple[float, float]:
    """Invert Eq. 1 for a synthetic training billet: fix h0, derive w0 = h0 / alpha0.

    Used only by the offline FEA data generator, which needs a concrete
    rectangular billet to mesh for a sampled ``alpha0`` -- the runtime path
    (``geometry.py``) goes the other direction, computing ``alpha0`` from an
    already-concrete machine geometry.
    """
    if alpha0 <= 0.0:
        raise ValueError("alpha0 must be positive.")
    return h0_mm, h0_mm / alpha0


def bite_length_mm(xb: float, h0_mm: float = REFERENCE_HEIGHT_MM) -> float:
    """Invert Eq. 2: bite length b = xb * h0, for a synthetic training sample."""
    return xb * h0_mm


def reduction_mm(eps_h: float, h0_mm: float = REFERENCE_HEIGHT_MM) -> float:
    """Invert Eq. 3: reduction h0 - h1 = eps_h * h0, for a synthetic training sample."""
    return eps_h * h0_mm


def latin_hypercube_samples(
    ranges: dict[str, tuple[float, float]],
    n_samples: int,
    seed: int,
) -> dict[str, np.ndarray]:
    """A small, dependency-free Latin Hypercube sampler.

    Mirrors the paper's own sampling strategy ("kept random yet ensuring
    sufficient representation of each variable using latin hypercube
    sampling"): each variable's ``[0, 1]`` range is split into ``n_samples``
    equal-probability strata, one uniform draw per stratum, then the strata
    are independently shuffled per variable -- guaranteeing every stratum of
    every variable is represented exactly once, unlike plain uniform random
    sampling which can leave gaps.

    Returns one ``(n_samples,)`` array per key in ``ranges``, each scaled
    into its own ``(low, high)`` range.
    """
    if n_samples <= 0:
        raise ValueError("n_samples must be positive.")
    rng = np.random.default_rng(seed)
    strata_edges = np.linspace(0.0, 1.0, n_samples + 1)
    strata_low, strata_high = strata_edges[:-1], strata_edges[1:]

    result: dict[str, np.ndarray] = {}
    for name, (low, high) in ranges.items():
        if high <= low:
            raise ValueError(f"Invalid range for '{name}': {(low, high)}")
        within_stratum = rng.uniform(strata_low, strata_high)
        rng.shuffle(within_stratum)
        result[name] = low + within_stratum * (high - low)
    return result


def sample_process_parameters(n_samples: int, seed: int) -> tuple[ProcessParameters, ...]:
    """Latin-Hypercube-sample the paper's own variable space (Table 1)."""
    raw = latin_hypercube_samples(
        {"alpha0": PAPER_ALPHA0_RANGE, "xb": PAPER_XB_RANGE, "eps_h": PAPER_EPS_H_RANGE},
        n_samples,
        seed,
    )
    return tuple(
        ProcessParameters(alpha0=float(a), xb=float(b), eps_h=float(e))
        for a, b, e in zip(raw["alpha0"], raw["xb"], raw["eps_h"])
    )


def is_finite_positive(value: float) -> bool:
    return math.isfinite(value) and value > 0.0
