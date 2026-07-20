"""Geometric material-state helpers shared by the 2D and 3D slicer previews.

These functions replay the generated planar die constraints.  They are an
envelope visualisation only: each station begins as a circular billet section
and is clipped by the strikes that have occurred at that station.  No material
is transported between stations and no constitutive deformation is modelled.
"""

from __future__ import annotations

import math
from typing import Sequence

from .profile_slicer import Point2, ToolpathPlan


def material_cross_section(
    plan: ToolpathPlan,
    station_index: int,
    operation_index: int,
    operation_progress: float,
    radial_segments: int = 48,
) -> tuple[Point2, ...]:
    """Return the red remaining-stock polygon for one station and playback time.

    ``operation_progress`` is in [0, 1] for the active operation.  Previous
    operations are fully applied, later operations are absent.  A sampled ray
    reaches the nearest applied die plane or the original cylindrical radius.
    """
    if not plan.operations:
        return ()
    if not 0 <= station_index < len(plan.sections):
        raise IndexError("station_index is outside the toolpath sections")
    if radial_segments < 8:
        raise ValueError("radial_segments must be at least 8")

    current_index = max(0, min(operation_index, len(plan.operations) - 1))
    current_progress = max(0.0, min(operation_progress, 1.0))
    stock_radius = float(plan.operations[0]["metadata"]["stock_radius_mm"])
    planes: list[tuple[float, float, float]] = []

    for index, operation in enumerate(plan.operations[: current_index + 1]):
        metadata = operation["metadata"]
        if int(metadata["station_index"]) != station_index:
            continue
        progress = current_progress if index == current_index else 1.0
        reduction = float(metadata["radial_reduction_mm"]) * progress
        support = stock_radius - reduction
        rotation = math.radians(float(operation["rotation"]))
        planes.append((math.sin(rotation), math.cos(rotation), support))

    result: list[Point2] = []
    for index in range(radial_segments):
        angle = 2.0 * math.pi * index / radial_segments
        direction_y, direction_z = math.cos(angle), math.sin(angle)
        radius = stock_radius
        for normal_y, normal_z, support in planes:
            alignment = normal_y * direction_y + normal_z * direction_z
            if alignment > 1e-9:
                radius = min(radius, max(0.0, support / alignment))
        result.append((radius * direction_y, radius * direction_z))
    return tuple(result)


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
