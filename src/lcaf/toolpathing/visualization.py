"""Die-shape rendering helpers and the surrogate-driven material-state preview.

This module renders two things for the 2D/3D slicer preview:

1. **Die/anvil shape** (``die_cap``, ``die_contact_profile``,
   ``disc_contact_profile``, ``disc_rim_profile``, ``anvil_side_support``) --
   pure geometry describing the *tooling's* footprint, independent of how
   material actually deforms under it. Unchanged by, and unrelated to, the
   deformation model below.
2. **Material state** (``material_state`` and everything built on it) -- the
   billet's own animated shape as it is struck. This used to be a pure
   computational-geometry relaxation (a rigid die clip plus a hand-tuned
   raised-cosine bulge, guaranteed to converge to the target shape but with
   no connection to how a real strike displaces material). It is now driven
   entirely by ``lcaf.simulation.surrogate.inference.SurrogateNetwork`` -- a
   neural network trained on FEA data following Jagtap, Reinisch & Bailly
   (ESAFORM 2024, DOI 10.21741/9781644903131-253), generalised to 3D. See
   ``docs/surrogate_deformation_model.md`` for the full method writeup and
   ``lcaf/simulation/surrogate/README.md`` for the paper citation.

Every material-state function below now *requires* a loaded
``SurrogateNetwork`` -- there is no geometric fallback. Unlike the old
heuristic, a trained network has no guarantee of converging exactly to an
arbitrary target shape; ``find_sufficient_cycles`` reflects that honestly
(it already tolerated non-convergence, returning a best-effort plan with a
warning rather than raising -- see its own docstring).
"""

from __future__ import annotations

import dataclasses
import math
from typing import Sequence

from lcaf.simulation.surrogate.inference import SurrogateNetwork

from .toolpath_slicer import (
    MachineLimits,
    Point2,
    SliceSettings,
    ToolpathPlan,
    ToolpathSlicer,
    TriangleMesh,
)


def die_cap(
    tangential_offset_mm: float,
    support_mm: float,
    half_width_mm: float,
    corner_radius_mm: float,
) -> float | None:
    """Return the die surface depth at a tangential offset from its centre.

    Flat for ``|tangential_offset_mm| <= half_width_mm - corner_radius_mm``,
    then follows a circular fillet of ``corner_radius_mm`` out to the edge of
    the footprint.  Returns ``None`` outside the footprint (die not present).
    """
    offset = abs(tangential_offset_mm)
    if offset > half_width_mm + 1e-9:
        return None
    flat_half = max(0.0, half_width_mm - corner_radius_mm)
    if offset <= flat_half:
        return support_mm
    remainder = offset - flat_half
    return support_mm + corner_radius_mm - math.sqrt(
        max(0.0, corner_radius_mm**2 - remainder**2)
    )


def die_contact_profile(
    width_mm: float | None,
    corner_radius_mm: float,
    support_mm: float,
    samples: int = 24,
) -> tuple[Point2, ...] | None:
    """Sample (tangential offset, depth) pairs tracing the die's contact face.

    Returns ``None`` for an unconstrained (full-width) die; callers should
    fall back to drawing a plane spanning the whole visible extent.
    """
    if width_mm is None or width_mm <= 0:
        return None
    half_width = width_mm / 2.0
    samples = max(4, samples)
    return tuple(
        (
            offset,
            die_cap(offset, support_mm, half_width, corner_radius_mm) or support_mm,
        )
        for offset in (
            -half_width + width_mm * index / samples for index in range(samples + 1)
        )
    )


def disc_contact_profile(
    radius_mm: float | None,
    axial_offset_mm: float,
    support_mm: float,
    samples: int = 24,
) -> tuple[Point2, ...] | None:
    """Sample (tangential offset, depth) pairs tracing the circular striking
    die's contact face at one axial offset from its own centre.

    The disc's effective tangential half-width at ``axial_offset_mm`` is
    ``sqrt(max(0, radius_mm**2 - axial_offset_mm**2))`` -- zero, and this
    returns ``None``, once ``axial_offset_mm`` exceeds ``radius_mm`` (the die
    does not reach that far along the billet axis at all).
    """
    if radius_mm is None or radius_mm <= 0 or abs(axial_offset_mm) > radius_mm:
        return None
    half_width = math.sqrt(max(0.0, radius_mm**2 - axial_offset_mm**2))
    if half_width <= 0.0:
        return None
    return die_contact_profile(2.0 * half_width, 0.0, support_mm, samples)


def disc_rim_profile(
    radius_mm: float,
    half_length_mm: float,
    sides: int = 24,
) -> tuple[Point2, ...]:
    """Perimeter of the striking disc's true footprint in its own (axial,
    tangential) plane: a circle of ``radius_mm`` truncated to the strip
    ``|axial| <= half_length_mm``.

    Returns the full circle when ``radius_mm <= half_length_mm`` (the disc
    never reaches its own segment boundary).
    """
    sides = max(8, sides)
    if radius_mm <= half_length_mm + 1e-9:
        return tuple(
            (radius_mm * math.cos(2.0 * math.pi * index / sides), radius_mm * math.sin(2.0 * math.pi * index / sides))
            for index in range(sides)
        )

    half_length_mm = max(0.0, half_length_mm)
    theta_c = math.acos(min(1.0, half_length_mm / radius_mm))
    arc_span = math.pi - 2.0 * theta_c
    chord_half = math.sqrt(max(0.0, radius_mm**2 - half_length_mm**2))
    if arc_span <= 1e-9:
        # The clip is so tight the disc barely pokes past it -- a thin
        # rectangle rather than a degenerate zero-area sliver.
        return (
            (half_length_mm, chord_half),
            (-half_length_mm, chord_half),
            (-half_length_mm, -chord_half),
            (half_length_mm, -chord_half),
        )

    arc_points = max(2, sides // 2)
    rim: list[Point2] = []
    for index in range(arc_points + 1):
        theta = theta_c + arc_span * index / arc_points
        rim.append((radius_mm * math.cos(theta), radius_mm * math.sin(theta)))
    for index in range(arc_points + 1):
        theta = math.pi + theta_c + arc_span * index / arc_points
        rim.append((radius_mm * math.cos(theta), radius_mm * math.sin(theta)))
    return tuple(rim)


def anvil_side_support(ring: Sequence[Point2], rotation_deg: float) -> float:
    """The material's actual current boundary on the anvil's (-normal) side.

    The anvil never moves, but an earlier rotation may already have reduced
    this side below the pristine stock radius by the time a later rotation
    treats it as its own anvil -- this is the mirror image of the
    +normal "how far does the target/material extend toward the die"
    query used throughout this module, evaluated in the opposite direction,
    so a caller can render the anvil at wherever material actually is
    instead of a fixed, potentially "levitated" position.
    """
    if not ring:
        return 0.0
    angle = math.radians(rotation_deg)
    sine, cosine = math.sin(angle), math.cos(angle)
    return max(-(y * sine + z * cosine) for y, z in ring)


def material_state(
    plan: ToolpathPlan,
    operation_index: int,
    operation_progress: float,
    network: SurrogateNetwork,
    radial_segments: int = 48,
) -> tuple[tuple[Point2, ...], ...]:
    """Return every station's current material ring for one playback instant.

    ``operation_progress`` is in [0, 1] for the active operation.  Previous
    operations are fully applied, later operations are absent.  Every
    qualifying strike is replayed in order, against a shared station-by-angle
    grid, starting from the original cylindrical stock -- via
    ``network.apply_strike`` (see
    ``lcaf.simulation.surrogate.inference.SurrogateNetwork.apply_strike``),
    not a geometric heuristic. The process is inherently stateful: what one
    strike predicts depends on whatever the previous strikes already left
    behind, exactly like the paper's own multi-stroke repositioning.
    """
    sections = plan.sections
    station_count = len(sections)
    if not plan.operations or station_count == 0:
        return tuple(() for _ in range(station_count))
    if radial_segments < 8:
        raise ValueError("radial_segments must be at least 8")

    current_index = max(0, min(operation_index, len(plan.operations) - 1))
    current_progress = max(0.0, min(operation_progress, 1.0))
    stock_radius = float(plan.operations[0]["metadata"]["stock_radius_mm"])
    angles = tuple(2.0 * math.pi * index / radial_segments for index in range(radial_segments))
    station_x = tuple(section.x_model_mm for section in sections)
    points_grid: list[list[Point2]] = [
        [(stock_radius * math.cos(angle), stock_radius * math.sin(angle)) for angle in angles]
        for _ in range(station_count)
    ]

    for index, operation in enumerate(plan.operations[: current_index + 1]):
        progress = current_progress if index == current_index else 1.0
        points_grid = network.apply_strike(points_grid, station_x, operation["metadata"], stroke_progress=progress)

    return tuple(tuple(row) for row in points_grid)


def material_cross_section(
    plan: ToolpathPlan,
    station_index: int,
    operation_index: int,
    operation_progress: float,
    network: SurrogateNetwork,
    radial_segments: int = 48,
) -> tuple[Point2, ...]:
    """Convenience wrapper returning one station's ring from :func:`material_state`."""
    if not plan.operations:
        return ()
    if not 0 <= station_index < len(plan.sections):
        raise IndexError("station_index is outside the toolpath sections")
    return material_state(plan, operation_index, operation_progress, network, radial_segments)[station_index]


def _ring_area(ring: Sequence[Point2]) -> float:
    """Shoelace area of a closed ring of (y, z) points."""
    count = len(ring)
    if count < 3:
        return 0.0
    total = 0.0
    for index in range(count):
        y1, z1 = ring[index]
        y2, z2 = ring[(index + 1) % count]
        total += y1 * z2 - y2 * z1
    return abs(total) / 2.0


def axial_trim_allowance_mm(
    plan: ToolpathPlan,
    operation_index: int,
    operation_progress: float,
    network: SurrogateNetwork,
    radial_segments: int = 48,
) -> float:
    """How much extra length the free (+X) end currently holds to keep the
    billet's total volume exactly conserved.

    Forging never deletes or creates material. The surrogate's own local
    displacement prediction at each struck station does not, by itself,
    balance the cross-sectional area removed there against the rest of the
    billet -- real open-die forging pushes whatever is not locally
    reabsorbed out the free end (the clamped end cannot move), extending the
    billet's total length beyond the target's own (material a saw trims off
    once forging is complete). This function returns that length directly
    from a volume balance -- current total volume (trapezoidally integrated
    over every station's actual current ring, from ``material_state``)
    versus the original stock cylinder's volume -- rather than modelling
    axial flow explicitly. This is also where the local displacement
    kernel's un-applied axial component (see
    ``lcaf.simulation.surrogate.geometry.LocalFrame.displacement_to_global``)
    is compensated for in aggregate, rather than per station -- see
    ``docs/surrogate_deformation_model.md``'s scope section.

    Returns 0.0 once the current state already holds at least as much
    volume as the original stock (which can happen briefly ahead of the
    local prediction's own gradual catch-up, or once ``stock_radius_mm`` and
    the mesh's own length already happen to match the target's own volume
    with nothing left over -- see ``recommended_stock_length_mm``).
    """
    sections = plan.sections
    if not sections or not plan.operations:
        return 0.0
    station_x = tuple(section.x_model_mm for section in sections)
    if len(station_x) < 2:
        return 0.0
    span_mm = station_x[-1] - station_x[0]
    if span_mm <= 0.0:
        return 0.0

    # The reference "original stock volume" must use the exact same
    # discretisation (this same polygon-sampled ring, trapezoidally
    # integrated between these same station centres) that current_volume_mm3
    # below uses -- otherwise a mismatch as small as a regular n-gon's area
    # versus the true circle's, or trapezoidal-between-centres versus the
    # sections' true edge-to-edge span, would show up as a phantom volume
    # "deficit" (and therefore a phantom trim allowance) even for the
    # pristine, untouched stock. Since the pristine cross-section is
    # constant along X, trapezoidally integrating it is exactly
    # area * span regardless of intermediate station spacing.
    stock_radius_mm = float(plan.operations[0]["metadata"]["stock_radius_mm"])
    angles = tuple(2.0 * math.pi * index / radial_segments for index in range(radial_segments))
    pristine_ring = tuple((stock_radius_mm * math.cos(angle), stock_radius_mm * math.sin(angle)) for angle in angles)
    original_stock_volume_mm3 = _ring_area(pristine_ring) * span_mm

    state = material_state(plan, operation_index, operation_progress, network, radial_segments)
    current_volume_mm3 = sum(
        0.5 * (_ring_area(state[index]) + _ring_area(state[index + 1]))
        * (station_x[index + 1] - station_x[index])
        for index in range(len(state) - 1)
    )

    deficit_mm3 = original_stock_volume_mm3 - current_volume_mm3
    if deficit_mm3 <= 0.0:
        return 0.0
    free_end_area_mm2 = _ring_area(state[-1])
    if free_end_area_mm2 <= 1e-9:
        return 0.0
    return deficit_mm3 / free_end_area_mm2


def find_sufficient_cycles(
    mesh: TriangleMesh,
    settings: SliceSettings,
    limits: MachineLimits,
    network: SurrogateNetwork,
    max_cycles: int = 20,
    tolerance_mm: float = 0.5,
) -> ToolpathPlan:
    """Grow ``settings.cycles`` until the surrogate-predicted geometry matches target.

    Tries ``cycles = 1, 2, 3, ...`` (ignoring whatever ``settings.cycles``
    was already set to), replans each time, and compares the surrogate's
    predicted struck geometry against each segment's own target ring,
    stopping as soon as the worst vertex deviation anywhere is within
    ``tolerance_mm``. Unlike the old geometric heuristic (which was
    mathematically guaranteed to converge given enough cycles), a trained
    network predicts what a real strike actually does -- it may never reach
    an arbitrary target within ``max_cycles``, which is not a bug in this
    function so much as an honest signal that the plan itself may be
    physically unrealistic for the die/material combination the checkpoint
    was trained on. If it never converges by ``max_cycles``, the
    ``max_cycles`` attempt is returned with a warning appended rather than
    looping forever.
    """
    plan: ToolpathPlan | None = None
    for cycles in range(1, max(1, max_cycles) + 1):
        trial_settings = dataclasses.replace(settings, cycles=cycles)
        plan = ToolpathSlicer(mesh, trial_settings, limits).plan()
        if not plan.operations:
            return plan
        last_operation = len(plan.operations) - 1
        # One shared material_state call for every segment's check, not one
        # per segment (material_cross_section on its own would replay the
        # whole operation history from scratch for each segment) -- cheap for
        # the old arithmetic heuristic, but each replayed strike is now a
        # network evaluation, so this redundancy is worth avoiding.
        final_state = material_state(plan, last_operation, 1.0, network, radial_segments=48)
        worst_deviation_mm = 0.0
        for segment_index, segment in enumerate(plan.sections):
            target_ring = radial_resample(segment.polygon_yz_mm, radial_segments=48)
            final_ring = final_state[segment_index]
            for (target_y, target_z), (final_y, final_z) in zip(target_ring, final_ring):
                worst_deviation_mm = max(
                    worst_deviation_mm, math.hypot(final_y - target_y, final_z - target_z)
                )
        if worst_deviation_mm <= tolerance_mm:
            return plan

    assert plan is not None
    warning = (
        f"'Complete necessary cycles' reached max_cycles={max_cycles} without the "
        f"surrogate-predicted geometry converging within {tolerance_mm:.2f} mm of the "
        "target -- inspect the preview before trusting this plan; this can mean the plan "
        "asks for more reduction than this die/material combination can achieve, or simply "
        "that the checkpoint's own training data does not cover this strike well (see "
        "lcaf.simulation.surrogate.process_params.ProcessParameters.within_trained_domain)."
    )
    return dataclasses.replace(plan, warnings=plan.warnings + (warning,))


def radial_resample(polygon: Sequence[Point2], radial_segments: int = 48) -> tuple[Point2, ...]:
    """Resample a convex radial target polygon to a consistently indexed ring."""
    if len(polygon) < 3:
        raise ValueError("A target polygon needs at least three points")
    if radial_segments < 8:
        raise ValueError("radial_segments must be at least 8")

    result: list[Point2] = []
    for index in range(radial_segments):
        angle = 2.0 * math.pi * index / radial_segments
        direction = (math.cos(angle), math.sin(angle))
        radius = _ray_polygon_distance(polygon, direction)
        result.append((radius * direction[0], radius * direction[1]))
    return tuple(result)


def _ray_polygon_distance(polygon: Sequence[Point2], direction: Point2) -> float:
    """Return the first positive ray intersection with a convex polygon."""
    intersections: list[float] = []
    for start, end in zip(polygon, tuple(polygon[1:]) + (polygon[0],)):
        edge = (end[0] - start[0], end[1] - start[1])
        denominator = _cross(direction, edge)
        if abs(denominator) <= 1e-9:
            continue
        ray_distance = _cross(start, edge) / denominator
        edge_fraction = _cross(start, direction) / denominator
        if ray_distance >= -1e-9 and -1e-9 <= edge_fraction <= 1.0 + 1e-9:
            intersections.append(ray_distance)

    if not intersections:
        # This should not occur for normal centred targets accepted by the
        # planner, but preserves a usable preview for an unusual input.
        return 0.0
    return max(0.0, min(intersections))


def _cross(a: Point2, b: Point2) -> float:
    return a[0] * b[1] - a[1] * b[0]
