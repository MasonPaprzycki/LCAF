"""Global (machine) <-> local (per-strike, paper-frame) coordinate transform.

The paper's network is only ever evaluated in one strike's own local frame:
origin at the die's leading edge along the billet axis, 0 at the anvil
surface in the press direction, 0 on the rotation-axis centerline in the
tangential (spread) direction (see ``docs/surrogate_deformation_model.md``
section 2 for the full derivation with figures-in-words). This module builds
that local frame directly from the same geometry
``lcaf.toolpathing.toolpath_slicer``/``lcaf.toolpathing.visualization``
already compute for one STRIKE operation -- no new geometric concepts, no
duplicated die/segment logic.

Two things are read from the plan's *static* operation metadata (bite
length, this strike's own reduction); two are read from the *current,
already-struck* material state (pre-stroke height/width) -- process
parameters are stateful across a sequence of strikes exactly the way the
paper's own "any stroke, as long as the reference configuration is
correctly aligned" repositioning trick is (see the paper's Conclusion).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .process_params import ProcessParameters, aspect_ratio, bite_ratio, height_reduction

Point2 = tuple[float, float]


@dataclass(frozen=True)
class LocalFrame:
    """The affine map from global (X, Y, Z) to one strike's local (x0, y0, z0).

    ``rotation_rad`` is the strike's own commanded rotation (billet rotation
    about the machine X axis). ``axial_origin_mm`` is the die's leading edge
    along X (``segment_x_start_mm``) -- paper: "left edge of saddle".
    ``anvil_support_mm`` is the current material's own extent on the anvil
    (-press-direction) side at this strike's station, so that ``y0 = 0``
    lands exactly on the anvil surface and ``y0 = h0`` on the current
    pre-stroke free surface, matching the paper's Fig. 1 convention.
    """

    rotation_rad: float
    axial_origin_mm: float
    anvil_support_mm: float

    def press_direction(self) -> Point2:
        """Unit vector in (Y, Z) pointing from the anvil toward the die.

        Matches ``lcaf.toolpathing.toolpath_slicer.Segment.support_mm``'s own
        projection direction exactly, so a point exactly at this strike's
        target support sits at local ``x0=0, y0=support``.
        """
        return (math.sin(self.rotation_rad), math.cos(self.rotation_rad))

    def tangential_direction(self) -> Point2:
        """Unit vector in (Y, Z) orthogonal to ``press_direction`` (spread axis)."""
        return (math.cos(self.rotation_rad), -math.sin(self.rotation_rad))

    def to_local(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float]:
        """Global machine point -> local (x0, y0, z0), all in millimetres."""
        press_y, press_z = self.press_direction()
        tang_y, tang_z = self.tangential_direction()
        x0 = y_mm * tang_y + z_mm * tang_z
        y0 = y_mm * press_y + z_mm * press_z + self.anvil_support_mm
        z0 = x_mm - self.axial_origin_mm
        return x0, y0, z0

    def displacement_to_global(self, delta_x0_mm: float, delta_y0_mm: float) -> Point2:
        """Local in-plane (spread, press) displacement -> global (dY, dZ).

        Deliberately takes only the in-plane components. The local axial
        (``delta_z0``, core-fibre elongation) component is intentionally not
        converted here -- see ``docs/surrogate_deformation_model.md``'s
        scope section: the material-state grid keeps a fixed axial station
        spacing in this version, so per-point axial displacement has nowhere
        to go; ``visualization.axial_trim_allowance_mm`` still estimates
        excess free-end length from a volume balance instead.
        """
        press_y, press_z = self.press_direction()
        tang_y, tang_z = self.tangential_direction()
        dy = delta_x0_mm * tang_y + delta_y0_mm * press_y
        dz = delta_x0_mm * tang_z + delta_y0_mm * press_z
        return (dy, dz)


def support_from_row(row: Sequence[Point2], direction_rad: float) -> float:
    """The current material's extent in ``direction_rad``, read off the current state grid.

    This is exactly ``lcaf.toolpathing.toolpath_slicer.Segment.support_mm``'s
    own definition (the convex support function: the maximum projection of
    any point onto a direction), applied to one station's *current* ring of
    (y, z) points instead of the static target polygon -- deliberately a
    max over every point rather than a "nearest angle" lookup, since a
    surrogate strike can move a point tangentially off its original sampled
    angle (see the module docstring on ``LocalFrame``), so the point
    supporting a given direction is not reliably the one that started out
    nearest it. Clamped to zero: on a badly distorted ring every projection
    could in principle be negative, which is not a physically meaningful
    "support."
    """
    sine, cosine = math.sin(direction_rad), math.cos(direction_rad)
    return max(0.0, max((y * sine + z * cosine for y, z in row), default=0.0))


def strike_local_frame(
    row: Sequence[Point2],
    rotation_deg: float,
    segment_x_start_mm: float,
) -> LocalFrame:
    """Build one strike's local frame from its current material-state row."""
    rotation_rad = math.radians(rotation_deg)
    anvil_support_mm = support_from_row(row, rotation_rad + math.pi)
    return LocalFrame(
        rotation_rad=rotation_rad,
        axial_origin_mm=segment_x_start_mm,
        anvil_support_mm=anvil_support_mm,
    )


def strike_process_parameters(
    row: Sequence[Point2],
    operation_metadata: dict,
) -> ProcessParameters:
    """Derive (alpha0, xb, eps_h) for one strike from live state + plan metadata.

    ``h0``/``w0`` (pre-stroke height/width) are read from the *current*
    material state at this strike's own station -- the process is stateful
    across strikes, exactly like the paper's own multi-stroke repositioning
    (see the module docstring). Bite length and this strike's own reduction
    come from the plan's static operation metadata
    (``lcaf.toolpathing.toolpath_slicer.ToolpathSlicer.plan``'s STRIKE
    metadata): ``die_length_mm`` is already resolved to the striking
    segment's own axial width there (the paper's bite length ``b``);
    ``radial_reduction_mm`` is the *cumulative* reduction through this pass
    (``applied_reduction = reduction * pass_index / passes``), so this
    strike's own increment is ``radial_reduction_mm / strike_pass``
    (cumulative reduction is linear in pass index, so this recovers the
    constant per-pass increment exactly).
    """
    rotation_rad = math.radians(float(operation_metadata["rotation_deg"]))
    h0_mm = support_from_row(row, rotation_rad) + support_from_row(row, rotation_rad + math.pi)
    w0_mm = support_from_row(row, rotation_rad + math.pi / 2.0) + support_from_row(
        row, rotation_rad - math.pi / 2.0
    )
    if h0_mm <= 0.0 or w0_mm <= 0.0:
        raise ValueError(
            f"Degenerate current cross-section at this strike (h0={h0_mm:.6g} mm, "
            f"w0={w0_mm:.6g} mm) -- cannot derive process parameters."
        )

    strike_pass = float(operation_metadata.get("strike_pass", 1))
    cumulative_reduction_mm = float(operation_metadata["radial_reduction_mm"])
    per_pass_reduction_mm = cumulative_reduction_mm / max(strike_pass, 1.0)

    bite_mm = float(operation_metadata["die_length_mm"])

    return ProcessParameters(
        alpha0=aspect_ratio(h0_mm, w0_mm),
        xb=bite_ratio(bite_mm, h0_mm),
        eps_h=height_reduction(h0_mm, per_pass_reduction_mm),
    )


def affected_station_indices(
    station_x_mm: Sequence[float],
    center_x_mm: float,
    bite_length_mm: float,
    reach_multiple: float = 4.0,
    minimum_reach_mm: float = 1.0,
) -> tuple[int, ...]:
    """Which stations fall within this strike's zone of influence along X.

    A trained network already predicts displacement smoothly decaying to
    ~0 far from where it was struck (the paper's own finding: "predictions
    in the regions outside the deformation zone are predicted with a higher
    accuracy"), so this is a coarse pre-filter for efficiency and sanity
    (skip evaluating the network on stations obviously outside any strike's
    plausible reach), not a hard physical boundary the way the old
    geometric heuristic's margins were.
    """
    reach_mm = max(reach_multiple * bite_length_mm, minimum_reach_mm)
    return tuple(
        index
        for index, station_x in enumerate(station_x_mm)
        if abs(station_x - center_x_mm) <= reach_mm
    )
