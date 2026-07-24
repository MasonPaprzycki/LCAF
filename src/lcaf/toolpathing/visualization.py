"""Geometric material-state helpers shared by the 2D and 3D slicer previews.

These functions replay the generated die strikes against an evolving material
state spanning every axial station at once.  They are an envelope
visualisation only: the billet begins as a circular cylinder and is reshaped,
strike by strike, by the die that struck it.  No constitutive/thermal model
is used -- only computational geometry.

Two rigid surfaces act on every strike, matching the machine: a moving
(striking) die and a fixed anvil it presses the billet against. Both have a
real, finite contact footprint by default -- rather than one side being an
unconstrained infinite plane -- so that material a strike displaces is never
simply deleted: it relaxes smoothly into the immediately adjacent stations
(axially) and angles (tangentially), the way forge-temperature steel
actually spreads out from under a die rather than shrinking.

- The **striking die** (the +normal side, machine +Z) is a flat-faced
  **circular disc** of radius ``upper_die_radius_mm``, confined -- exactly
  like every previous version of this planner -- to the one station the
  operation targets: it never reaches a neighbouring station regardless of
  how large ``upper_die_radius_mm`` is, so widening the *anvil's* own axial
  reach (``die_length_mm``) can never make the striking die touch a station
  it wasn't asked to. Its radius instead controls how far it reaches
  *tangentially* at that one station: a tangential offset beyond
  ``upper_die_radius_mm`` is simply outside the disc, exactly as a
  tangential offset beyond the anvil's own half-width is outside it.
- The **anvil** (the -normal side, machine -Z) never moves. It is the
  rectangular prism described by ``die_width_mm``/``die_length_mm``
  (optionally rounded at its tangential edges by ``die_corner_radius_mm``):
  within that footprint it is an impenetrable displacement boundary holding
  material at the original stock surface (it never reduces anything, only
  prevents bulging past where the billet already rests); outside it, the
  anvil is simply not there.

Bulge growth is a smooth, bounded *relaxation toward the target* -- not a
volume-conservation solve. Each affected ray moves a fraction of the way
from wherever it currently sits toward the target's own known boundary in
that exact direction, whether that means growing (material spilling
sideways from a narrow die) or shrinking (a neighbouring station easing
toward its own smaller eventual value ahead of its own dedicated strike):

    new_radius = radius + weight * closure_fraction * (target_radius - radius)

``weight`` is the product of an axial and a tangential raised-cosine
factor, exactly ``1`` at a die's rigid edge (continuous with the rigid clip
itself) and fading smoothly to exactly ``0`` a margin's width away -- so it,
and therefore ``new_radius``, varies continuously with position: no ray can
jump straight from "completely untouched" to "sitting exactly on its final
target" the way solving one aggregate volume-conservation growth per strike
could (and, in an earlier version of this module, did -- concentrating an
entire strike's displaced volume onto the handful of rays nearest a narrow
die's edge and, for the overwhelmingly common case of a *reducing* target
(target below the ray's current radius), snapping them straight to that
target in a single strike regardless of how small the computed growth was,
while their immediate neighbours stayed completely unmoved: a visible,
"triangulated" discontinuity rather than a smooth bulge). ``weight *
closure_fraction`` is always < 1 for every material this module knows
about, so a single strike can only ever close *part* of the remaining gap
-- the shape only reaches its target *exactly* in the limit, after enough
strikes/cycles, never as an intermediate-frame artifact. This also means
volume is no longer conserved *exactly* per strike, only approximately --
the deliberate trade for continuity; the die's own rigid, commanded depth
is completely unaffected either way.

Each footprint's bulge margin -- the raised-cosine taper's own width -- is
never allowed to shrink below a fraction of ``stock_radius_mm`` (see
``_MIN_MARGIN_FRACTION``), so it always stays wide enough for the angular/
axial sampling grid to actually resolve it as a smooth curve rather than a
handful of samples that alias into a step (the same discontinuity as above,
from a different cause: too few samples inside an overly-narrow margin).
Two further, purely geometric refinements make that bulge behave more like
real forge-temperature material actually spreads, without ever becoming a
constitutive/FEM model:

- **Confinement-weighted anisotropy.** A die's own footprint is rarely
  square: a long, narrow anvil confines material strongly along its length
  but leaves it relatively free tangentially, while a short, wide anvil does
  the opposite. Each footprint's own axial-vs-tangential bulge margins (see
  ``_footprint_bias``) are scaled by that footprint's own aspect ratio, so
  material genuinely spreads preferentially in whichever direction the die
  itself is smallest -- not by the same symmetric distance-based falloff
  regardless of die shape. A square/symmetric footprint reduces exactly to
  the previous, isotropic margins.
- **Material/temperature-gated response.** ``lcaf.toolpathing.material``
  resolves ``material``/``target_temperature_c`` (carried on every
  operation) to a 0..1 ``formability``, which scales ``reach_scale`` (how
  far the bulge margins reach -- cold/stiff material bulges tightly against
  the die; hot/soft material spreads much further) and ``closure_fraction``
  (how much of the remaining gap to target closes per strike, in the
  relaxation formula above). The die's own rigid, commanded depth always
  applies in full and immediately -- only the secondary bulge redistribution
  is gated -- so cold, unworkable material genuinely takes more strikes/
  cycles to fully spread onto the target, the same way real forging
  practice needs more hits for less workable material, while the shape
  still converges exactly given enough cycles (``find_sufficient_cycles``
  warns rather than silently failing if it does not, within
  ``max_cycles``).

Because the default ``upper_die_radius_mm`` (``stock_radius_mm``) always
fully covers the target at the exact station it targets, the final shape
(after enough strikes/cycles have run) still converges exactly to the
target's own support envelope at that default, regardless of the configured
anvil size -- an explicitly undersized ``upper_die_radius_mm`` is an honest
exception to this: like a real round punch smaller than a face, it can
legitimately leave parts of that face unstruck.
"""

from __future__ import annotations

import dataclasses
import math
from typing import Sequence

from .material import formability_response, resolve_material_band
from .toolpath_slicer import (
    MachineLimits,
    Point2,
    SliceSettings,
    ToolpathPlan,
    ToolpathSlicer,
    TriangleMesh,
)

# The minimum width of a bulge margin's raised-cosine taper, as a fraction of
# stock_radius_mm -- see the module docstring's discussion of why a margin
# must never be allowed to shrink below what the angular/axial sampling grid
# can actually resolve as smooth.
_MIN_MARGIN_FRACTION = 0.12


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

    This is the exact shape ``_apply_strike_3d``'s ``disc_half_width_at``
    already enforces per station -- tangential half-width
    ``sqrt(max(0, radius_mm**2 - axial**2))``, zero beyond ``radius_mm`` --
    just expressed as a rim polygon instead of a per-station formula, so a
    3D renderer can draw the die's real footprint (a circle flattened by two
    straight chords where it overhangs the segment, never a uniformly
    shrunk circle that would misrepresent how far it reaches tangentially)
    instead of approximating it.

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
    radial_segments: int = 48,
) -> tuple[tuple[Point2, ...], ...]:
    """Return every station's remaining-material ring for one playback instant.

    ``operation_progress`` is in [0, 1] for the active operation.  Previous
    operations are fully applied, later operations are absent.  Every
    qualifying strike is replayed in order against a shared station-by-angle
    grid of sampled radii, starting from the original cylindrical stock, so
    that a strike's finite axial length can genuinely reach neighbouring
    stations rather than only the one it is centred on.
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
    radii_grid: list[list[float]] = [[stock_radius] * radial_segments for _ in range(station_count)]
    # The target's own boundary is a hard ceiling on bulge growth: displaced
    # material may spread into unclaimed space, but a strike at one rotation
    # must never push material, in some other direction it does not itself
    # cover, past where the target ultimately requires it to end up -- that
    # is never recovered by a later strike and is exactly the "deformed away
    # from the target" failure mode this guards against.
    target_radii_grid = tuple(
        tuple(
            _ray_polygon_distance(section.polygon_yz_mm, (math.cos(angle), math.sin(angle)))
            for angle in angles
        )
        for section in sections
    )

    for index, operation in enumerate(plan.operations[: current_index + 1]):
        metadata = operation["metadata"]
        progress = current_progress if index == current_index else 1.0
        reduction = float(metadata["radial_reduction_mm"]) * progress
        support = stock_radius - reduction
        rotation = math.radians(float(operation["rotation"]))
        width = float(metadata.get("die_width_mm") or stock_radius)
        corner_radius = float(metadata.get("die_corner_radius_mm", 0.0) or 0.0)
        die_length = metadata.get("die_length_mm")
        # The anvil's own axial reach -- always concrete in current plans
        # (plan() resolves an unset die_length_mm to the striking segment's
        # own width), the 1e-6 fallback only guards older/hand-built metadata.
        axial_half_length = (float(die_length) / 2.0) if die_length is not None else 1e-6
        upper_die_radius = float(metadata.get("upper_die_radius_mm") or stock_radius)
        # The striking disc spans its own whole segment by default -- a
        # strike operation now inherently represents that entire axial
        # region (its target support is already the region's own
        # numerically integrated average), not one infinitesimal station.
        segment_x_start = metadata.get("segment_x_start_mm")
        segment_x_end = metadata.get("segment_x_end_mm")
        disc_axial_half_length = (
            abs(float(segment_x_end) - float(segment_x_start)) / 2.0
            if segment_x_start is not None and segment_x_end is not None
            else 1e-6
        )
        center_x = station_x[int(metadata["segment_index"])]
        material = metadata.get("material", "steel")
        temperature_c = float(operation.get("target_temperature", 0.0))
        band = resolve_material_band(material, temperature_c)
        response = formability_response(band.formability)
        radii_grid = _apply_strike_3d(
            radii_grid, angles, station_x, center_x, axial_half_length, disc_axial_half_length,
            rotation, support, stock_radius, width, corner_radius, target_radii_grid, upper_die_radius,
            response.reach_scale, response.closure_fraction,
        )

    return tuple(
        tuple((radius * math.cos(angle), radius * math.sin(angle)) for radius, angle in zip(row, angles))
        for row in radii_grid
    )


def material_cross_section(
    plan: ToolpathPlan,
    station_index: int,
    operation_index: int,
    operation_progress: float,
    radial_segments: int = 48,
) -> tuple[Point2, ...]:
    """Convenience wrapper returning one station's ring from :func:`material_state`."""
    if not plan.operations:
        return ()
    if not 0 <= station_index < len(plan.sections):
        raise IndexError("station_index is outside the toolpath sections")
    return material_state(plan, operation_index, operation_progress, radial_segments)[station_index]


def find_sufficient_cycles(
    mesh: TriangleMesh,
    settings: SliceSettings,
    limits: MachineLimits,
    max_cycles: int = 20,
    tolerance_mm: float = 0.5,
) -> ToolpathPlan:
    """Grow ``settings.cycles`` until the simulated geometry matches target.

    Every operation just replays, in order, against one running material
    state, so a later cycle automatically trues up whatever an earlier
    (rougher) cycle left rough or overshot -- no cycle-aware engine logic is
    needed beyond simply planning with more cycles and re-checking. This
    tries ``cycles = 1, 2, 3, ...`` (ignoring whatever ``settings.cycles``
    was already set to), replans each time, and compares the fully struck
    geometry against each segment's own target ring the same way
    ``test_final_deformed_geometry_matches_the_target_exactly`` already
    does, stopping as soon as the worst vertex deviation anywhere is within
    ``tolerance_mm``. If it never converges by ``max_cycles`` (a too-small
    die footprint or too few strikes per segment can make some directions
    structurally unreachable), the ``max_cycles`` attempt is returned with a
    warning appended rather than looping forever.
    """
    plan: ToolpathPlan | None = None
    for cycles in range(1, max(1, max_cycles) + 1):
        trial_settings = dataclasses.replace(settings, cycles=cycles)
        plan = ToolpathSlicer(mesh, trial_settings, limits).plan()
        if not plan.operations:
            return plan
        last_operation = len(plan.operations) - 1
        worst_deviation_mm = 0.0
        for segment_index, segment in enumerate(plan.sections):
            target_ring = radial_resample(segment.polygon_yz_mm, radial_segments=48)
            final_ring = material_cross_section(
                plan, segment_index, last_operation, 1.0, radial_segments=48
            )
            for (target_y, target_z), (final_y, final_z) in zip(target_ring, final_ring):
                worst_deviation_mm = max(
                    worst_deviation_mm, math.hypot(final_y - target_y, final_z - target_z)
                )
        if worst_deviation_mm <= tolerance_mm:
            return plan

    assert plan is not None
    warning = (
        f"'Complete necessary cycles' reached max_cycles={max_cycles} without the "
        f"simulated geometry converging within {tolerance_mm:.2f} mm of the target -- "
        "inspect the preview before trusting this plan; a too-small die footprint or "
        "too few strikes per segment can make some directions structurally unreachable "
        "no matter how many cycles are run."
    )
    return dataclasses.replace(plan, warnings=plan.warnings + (warning,))


def _apply_strike_3d(
    radii_grid: list[list[float]],
    angles: Sequence[float],
    station_x: Sequence[float],
    center_x: float,
    axial_half_length: float,
    disc_axial_half_length: float,
    rotation_rad: float,
    support_mm: float,
    stock_radius_mm: float,
    width_mm: float,
    corner_radius_mm: float,
    target_radii_grid: Sequence[Sequence[float]],
    upper_die_radius_mm: float,
    reach_scale: float,
    closure_fraction: float,
) -> list[list[float]]:
    """Clip the whole station-by-angle grid by one die strike.

    Two rigid, finite footprints act on every strike:

    - The **striking die** (+normal side) is a genuine flat-faced circular
      disc of radius ``upper_die_radius_mm``: at axial offset ``a`` from the
      strike its tangential half-width is ``sqrt(max(0, R**2 - a**2))``, the
      same lens-shaped taper a real round punch has. That taper is
      additionally cut off at ``disc_axial_half_length`` (+ margin) along X
      -- by default the *whole width of the segment this strike targets*,
      since a strike's target support is itself the numerical average of
      the target across that whole segment, not one point -- so a disc
      wider than its own segment is simply truncated by the segment
      boundary, the way a real punch that overhangs its intended region
      would only ever contact within that region. ``disc_axial_half_length``
      is independent of ``die_length_mm`` (below, the anvil's own axial
      reach) -- widening the anvil specifically must never change how far
      the striking disc reaches.
    - The **anvil** (-normal side) never moves; within its rectangular
      footprint (``width_mm`` tangentially, ``axial_half_length`` --
      from ``die_length_mm`` -- axially, blended at its tangential edges by
      ``corner_radius_mm``) it is an impenetrable displacement boundary
      holding material at ``stock_radius_mm`` (it never reduces anything,
      only prevents bulging past the original surface).

    Material either footprint displaces is not deleted: it relaxes smoothly,
    immediately adjacent to it -- both axially and tangentially -- toward
    the target's own known boundary in that exact direction, via the bounded
    relaxation described in the module docstring, fading to zero within a
    margin of the footprint's own edge. That relaxation never pushes
    material past the target's own boundary in that exact direction (it is
    a bounded blend toward it, never an overshoot), since a rotation whose
    strike does not itself cover that direction will never bring an
    overshoot there back down again. The two footprints can never collide
    with each other -- the striking side only ever touches rays with
    ``sin(theta + rotation_rad) > 0``, the anvil side only rays with
    ``sin(theta + rotation_rad) < 0``.

    ``reach_scale`` and ``closure_fraction`` (see
    ``lcaf.toolpathing.material.formability_response``) do not touch either
    footprint's *rigid* shape or reach -- only the bulge margins (via
    ``_footprint_bias``, confinement-weighted by each footprint's own
    axial/tangential aspect ratio) and how much of the remaining gap to
    target closes this strike. Both default to values that reduce exactly
    to the previous isotropic behaviour (``reach_scale=1``), and
    ``closure_fraction=1`` collapses the relaxation to "snap fully to
    target wherever touched" -- never used in practice since no material
    band reaches a formability of exactly 1.
    """
    station_count = len(radii_grid)
    segment_count = len(angles)
    new_grid = [list(row) for row in radii_grid]

    # The raised-cosine bulge margins below must stay wide enough for the
    # angular/axial sampling grid to actually resolve them as a smooth taper:
    # an aspect-ratio bias and/or a low reach_scale can otherwise shrink a
    # margin to a fraction of a millimetre, at which point only one or two
    # samples fall inside it and the "smooth" cosine collapses into a hard
    # step between neighbouring rays -- a discontinuous, "triangulated" jump
    # in the rendered cross-section rather than a bulge.
    margin_floor_mm = max(stock_radius_mm * _MIN_MARGIN_FRACTION, 1e-6)

    half_width_anvil = width_mm / 2.0
    anvil_axial_bias, anvil_tangential_bias = _footprint_bias(2.0 * axial_half_length, width_mm)
    influence_margin_anvil_mm = max(2.0 * half_width_anvil * (2.0 * anvil_tangential_bias) * reach_scale, margin_floor_mm)
    axial_margin_mm = max(2.0 * axial_half_length * (2.0 * anvil_axial_bias) * reach_scale, margin_floor_mm)
    disc_axial_bias, disc_tangential_bias = _footprint_bias(2.0 * disc_axial_half_length, 2.0 * upper_die_radius_mm)
    disc_axial_margin_mm = max(2.0 * disc_axial_half_length * (2.0 * disc_axial_bias) * reach_scale, margin_floor_mm)

    def disc_half_width_at(axial_offset: float) -> float:
        return math.sqrt(max(0.0, upper_die_radius_mm**2 - axial_offset**2))

    in_footprint = [[False] * segment_count for _ in range(station_count)]
    touched: set[int] = set()

    # --- Striking die: a flat-faced circular disc, its lens-shaped taper
    # additionally cut off at disc_axial_half_length (+ margin) from the
    # segment it targets.
    for station in range(station_count):
        axial_offset = abs(station_x[station] - center_x)
        if axial_offset > disc_axial_half_length + disc_axial_margin_mm + 1e-9:
            continue  # far outside this strike's zone of influence -- untouched.
        touched.add(station)
        if axial_offset > disc_axial_half_length + 1e-9:
            continue  # in the bulge margin but not rigidly held at this station.
        half_width = disc_half_width_at(axial_offset)
        if half_width <= 0.0:
            continue  # the disc's own circular taper does not reach this far.
        row = new_grid[station]
        footprint_row = in_footprint[station]
        for index, theta in enumerate(angles):
            delta = theta + rotation_rad
            alignment = math.sin(delta)
            if alignment <= 1e-9:
                continue  # this ray is on the anvil's side (or tangent).
            clipped = _die_clip_radius(row[index], alignment, math.cos(delta), support_mm, half_width, 0.0)
            if clipped is not None:
                row[index] = clipped
            if abs(row[index] * math.cos(delta)) <= half_width + 1e-9:
                footprint_row[index] = True

    # --- Anvil: rectangular footprint, reaching axial_half_length (+
    # margin) from die_length_mm -- unchanged from every previous version.
    for station in range(station_count):
        axial_offset = abs(station_x[station] - center_x)
        if axial_offset > axial_half_length + axial_margin_mm + 1e-9:
            continue  # far outside this strike's zone of influence -- untouched.
        touched.add(station)
        if axial_offset > axial_half_length + 1e-9:
            continue  # in the bulge margin but not rigidly held at this station.
        row = new_grid[station]
        footprint_row = in_footprint[station]
        for index, theta in enumerate(angles):
            delta = theta + rotation_rad
            alignment = math.sin(delta)
            if alignment >= -1e-9:
                continue  # this ray is on the striking side (or tangent); the anvil is not there.
            clipped = _die_clip_radius(row[index], -alignment, math.cos(delta), stock_radius_mm, half_width_anvil, corner_radius_mm)
            if clipped is not None:
                row[index] = clipped
            if abs(row[index] * math.cos(delta)) <= half_width_anvil + 1e-9:
                footprint_row[index] = True

    touched_stations = sorted(touched)
    if not touched_stations:
        return new_grid

    # Disc bulge: relax each untouched ray on the striking side smoothly
    # toward its own known target, weighted by an axial x tangential
    # raised-cosine fade -- applied immediately and directly (no aggregate
    # volume solve), since this pass and the anvil pass below can never
    # touch the same ray (they occupy opposite hemispheres of sin(delta)).
    for station in touched_stations:
        axial_offset = abs(station_x[station] - center_x)
        axial_beyond = max(0.0, axial_offset - disc_axial_half_length)
        if axial_beyond >= disc_axial_margin_mm:
            continue
        axial_factor = (
            1.0 if axial_beyond <= 0.0
            else 0.5 * (1.0 + math.cos(math.pi * axial_beyond / disc_axial_margin_mm))
        )
        half_width = disc_half_width_at(min(axial_offset, disc_axial_half_length))
        influence_margin_disc_mm = max(2.0 * half_width * (2.0 * disc_tangential_bias) * reach_scale, margin_floor_mm)
        footprint_row = in_footprint[station]
        row = new_grid[station]
        target_row = target_radii_grid[station]
        for index, theta in enumerate(angles):
            if footprint_row[index]:
                continue
            delta = theta + rotation_rad
            if math.sin(delta) <= 1e-9:
                continue  # only the striking die's own hemisphere bulges here.
            tangential = abs(row[index] * math.cos(delta))
            tangential_beyond = tangential - half_width
            if tangential_beyond <= 0.0:
                tangential_factor = 1.0
            elif tangential_beyond < influence_margin_disc_mm:
                tangential_factor = 0.5 * (1.0 + math.cos(math.pi * tangential_beyond / influence_margin_disc_mm))
            else:
                continue
            weight = axial_factor * tangential_factor
            if weight > 0.0:
                current = row[index]
                row[index] = current + weight * closure_fraction * (target_row[index] - current)

    # Anvil bulge: the same bounded relaxation, on the anvil's hemisphere.
    for station in touched_stations:
        axial_offset = abs(station_x[station] - center_x)
        axial_beyond = max(0.0, axial_offset - axial_half_length)
        if axial_beyond >= axial_margin_mm:
            continue
        axial_factor = (
            1.0 if axial_beyond <= 0.0
            else 0.5 * (1.0 + math.cos(math.pi * axial_beyond / axial_margin_mm))
        )
        footprint_row = in_footprint[station]
        row = new_grid[station]
        target_row = target_radii_grid[station]
        for index, theta in enumerate(angles):
            if footprint_row[index]:
                continue
            delta = theta + rotation_rad
            if math.sin(delta) >= -1e-9:
                continue  # the striking die (or the tangent seam) never bulges -- only the anvil side can.
            tangential = abs(row[index] * math.cos(delta))
            tangential_beyond = tangential - half_width_anvil
            if tangential_beyond <= 0.0:
                tangential_factor = 1.0
            elif tangential_beyond < influence_margin_anvil_mm:
                tangential_factor = 0.5 * (1.0 + math.cos(math.pi * tangential_beyond / influence_margin_anvil_mm))
            else:
                continue
            weight = axial_factor * tangential_factor
            if weight > 0.0:
                current = row[index]
                row[index] = current + weight * closure_fraction * (target_row[index] - current)

    return new_grid


def _footprint_bias(axial_extent_mm: float, tangential_extent_mm: float) -> tuple[float, float]:
    """Confinement-weighted axial/tangential bias from one footprint's own aspect ratio.

    Material spreads preferentially toward whichever direction the die
    itself is smaller in -- the direction it confines least -- rather than
    with the same isotropic reach regardless of die shape: a long, narrow
    footprint is biased toward tangential spread, a short, wide one toward
    axial spread. Returns ``(axial_bias, tangential_bias)``, each in (0, 1)
    and summing to 1; a square footprint (equal extents) returns
    ``(0.5, 0.5)`` which, multiplied by 2 where it is used, reduces exactly
    to the previous isotropic margins.
    """
    axial_extent_mm = max(axial_extent_mm, 1e-9)
    tangential_extent_mm = max(tangential_extent_mm, 1e-9)
    total = axial_extent_mm + tangential_extent_mm
    return tangential_extent_mm / total, axial_extent_mm / total


def _die_clip_radius(
    old_radius: float,
    alignment: float,
    tangent_coefficient: float,
    support_mm: float,
    half_width_mm: float,
    corner_radius_mm: float,
) -> float | None:
    """Find the smallest radius along one ray where it first meets the die surface.

    The ray is parameterised by ``r``; its normal-direction coordinate is
    ``r * alignment`` and its tangential coordinate is ``r * tangent_coefficient``.
    Returns ``None`` if the ray never exceeds the die surface before ``old_radius``
    (already clear, or it leaves the footprint before that would happen).
    """

    def violation(radius: float) -> float:
        tangential = radius * tangent_coefficient
        cap = die_cap(tangential, support_mm, half_width_mm, corner_radius_mm)
        if cap is None:
            return -1.0
        return radius * alignment - cap

    if violation(old_radius) <= 0.0:
        return None
    if violation(0.0) > 0.0:
        return 0.0

    low, high = 0.0, old_radius
    for _ in range(40):
        mid = (low + high) / 2.0
        if violation(mid) > 0.0:
            high = mid
        else:
            low = mid
    return high


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
