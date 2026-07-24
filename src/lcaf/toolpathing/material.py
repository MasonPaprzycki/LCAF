"""Approximate, practical material models for the toolpath slicer.

Nothing here is a material simulation or a sourced alloy datasheet. Every
constant is a deliberately rough, order-of-magnitude engineering estimate
chosen for two narrow purposes:

1. Give ``material`` and ``target_temperature_c`` a real, felt effect on the
   still purely computational-geometry deformation preview in
   ``visualization.py`` -- via ``formability`` (see ``formability_response``
   below), not via any constitutive/FEM model.
2. Provide a separate, practical slab-method force estimate
   (``estimate_operation_force_kn``) for planning purposes.

Numbers here never feed back into the planned strike coordinates (x/y/
die_gap/rotation) themselves, and the force estimate never feeds back into
the deformation preview -- both consume the same material/temperature
inputs independently. Dies are treated as rigid and able to supply whatever
force the estimate says they need.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class MaterialBand:
    """One material's behaviour within a temperature band, up to ``max_temperature_c``.

    ``formability`` (0..1) is the only value the geometric deformation
    mechanics consume (see ``formability_response``). ``flow_stress_mpa``
    and ``friction_coefficient`` are only used by
    ``estimate_strike_force_kn``.
    """

    max_temperature_c: float
    formability: float
    flow_stress_mpa: float
    friction_coefficient: float


# Three practical bands per material -- cold/warm/hot working ranges,
# roughly matched to where each material is actually forged in practice
# (plasticine near room temperature, aluminum a few hundred C, steel in the
# 900-1250 C hot-working range). These are approximate handbook-order-of-
# magnitude constants, not sourced from a specific alloy datasheet: good
# enough to make relative comparisons (steel needs more strikes than
# aluminum at the same formability; force scales sensibly with contact area
# and material) practical without a real simulation.
_MATERIALS: dict[str, tuple[MaterialBand, ...]] = {
    "plasticine": (
        MaterialBand(max_temperature_c=15.0, formability=0.55, flow_stress_mpa=0.15, friction_coefficient=0.35),
        MaterialBand(max_temperature_c=30.0, formability=0.85, flow_stress_mpa=0.08, friction_coefficient=0.30),
        MaterialBand(max_temperature_c=float("inf"), formability=0.97, flow_stress_mpa=0.04, friction_coefficient=0.25),
    ),
    "aluminum": (
        MaterialBand(max_temperature_c=200.0, formability=0.15, flow_stress_mpa=120.0, friction_coefficient=0.40),
        MaterialBand(max_temperature_c=400.0, formability=0.50, flow_stress_mpa=60.0, friction_coefficient=0.35),
        MaterialBand(max_temperature_c=float("inf"), formability=0.90, flow_stress_mpa=25.0, friction_coefficient=0.30),
    ),
    "steel": (
        MaterialBand(max_temperature_c=600.0, formability=0.05, flow_stress_mpa=400.0, friction_coefficient=0.50),
        MaterialBand(max_temperature_c=900.0, formability=0.35, flow_stress_mpa=150.0, friction_coefficient=0.40),
        MaterialBand(max_temperature_c=float("inf"), formability=0.85, flow_stress_mpa=60.0, friction_coefficient=0.30),
    ),
}

MATERIALS: tuple[str, ...] = tuple(_MATERIALS)


def resolve_material_band(material: str, temperature_c: float) -> MaterialBand:
    """Return the temperature band a material falls into at ``temperature_c``."""
    key = material.strip().lower()
    try:
        bands = _MATERIALS[key]
    except KeyError as error:
        raise ValueError(
            f"Unknown material '{material}'. Choose one of: {', '.join(MATERIALS)}."
        ) from error
    for band in bands:
        if temperature_c <= band.max_temperature_c:
            return band
    return bands[-1]


@dataclass(frozen=True)
class FormabilityResponse:
    """The two dimensionless scalars the deformation mechanics consume."""

    reach_scale: float
    closure_fraction: float


def formability_response(formability: float) -> FormabilityResponse:
    """Map a 0..1 formability into the deformation mechanics' two knobs.

    ``reach_scale`` (0.4..1.0) scales how far displaced material bulges out
    from a die's rigid edge: cold/stiff material bulges tightly against the
    die, hot/soft material spreads much further before fading out.
    ``closure_fraction`` (0.15..1.0) scales how much of one strike's own
    exact volume-conserving redistribution is actually applied immediately,
    versus left for a later strike/cycle to keep closing -- cold, stiff
    material takes many more hits to fully spread and settle onto the
    target, exactly like real forging practice; hot, soft material nearly
    fully relaxes in one hit.
    """
    formability = max(0.0, min(1.0, formability))
    return FormabilityResponse(
        reach_scale=0.4 + 0.6 * formability,
        closure_fraction=0.15 + 0.85 * formability,
    )


def estimate_strike_force_kn(
    *,
    material: str,
    temperature_c: float,
    contact_width_mm: float,
    contact_length_mm: float,
    instantaneous_height_mm: float,
) -> float:
    """A slab-method (friction-hill) force estimate for one flat-die strike.

    ``F = flow_stress * contact_area * (1 + friction * width / (3 * height))``
    is the standard closed-form flat-die forging force estimate (the
    friction-hill correction to a plane-strain slab analysis; see any
    metal-forming text, e.g. Kalpakjian & Schmid). It is a hand-calculation-
    grade estimate, not a simulation, and is entirely separate from the
    geometric deformation preview: it never feeds back into planned strike
    coordinates or the animated shape, and dies are treated as rigid and
    able to supply whatever force it says they need.
    """
    band = resolve_material_band(material, temperature_c)
    height = max(instantaneous_height_mm, 1e-6)
    area_mm2 = max(contact_width_mm, 0.0) * max(contact_length_mm, 0.0)
    friction_hill = 1.0 + band.friction_coefficient * (contact_width_mm / (3.0 * height))
    force_n = band.flow_stress_mpa * area_mm2 * friction_hill
    return force_n / 1000.0


def estimate_operation_force_kn(operation: dict) -> float:
    """Estimate one exported STRIKE operation's forging force from its own metadata.

    Reads everything it needs straight from an operation dict a
    ``ToolpathPlan`` already produces (or one loaded back from an exported
    JSONL) -- ``metadata["material"]``, the operation's own
    ``target_temperature``, and the die/support geometry already recorded in
    ``metadata`` -- so it can be computed standalone from a plan or a JSONL
    file, without needing the original ``SliceSettings`` or mesh.
    """
    metadata = operation.get("metadata", {})
    material = metadata.get("material", "steel")
    temperature_c = float(operation.get("target_temperature", 0.0))
    stock_radius_mm = float(metadata.get("stock_radius_mm", 0.0))
    support_mm = float(metadata.get("target_support_mm", 0.0))
    die_width_mm = float(metadata.get("die_width_mm") or stock_radius_mm)
    upper_die_radius_mm = float(metadata.get("upper_die_radius_mm") or stock_radius_mm)
    die_length_mm = float(metadata.get("die_length_mm") or (2.0 * upper_die_radius_mm))

    contact_length_mm = min(2.0 * upper_die_radius_mm, die_length_mm)
    instantaneous_height_mm = stock_radius_mm + support_mm
    return estimate_strike_force_kn(
        material=material,
        temperature_c=temperature_c,
        contact_width_mm=die_width_mm,
        contact_length_mm=contact_length_mm,
        instantaneous_height_mm=instantaneous_height_mm,
    )


def plan_force_report(operations: Sequence[dict]) -> tuple[dict, ...]:
    """A per-operation force-estimate report, suitable for a UI readout or export.

    Entirely separate from the JSONL operations themselves: this is never
    written into the controller-facing metadata, only computed alongside it.
    """
    return tuple(
        {
            "step": operation["step"],
            "rotation_deg": operation["rotation"],
            "material": operation.get("metadata", {}).get("material", "steel"),
            "target_temperature_c": operation.get("target_temperature", 0.0),
            "estimated_force_kn": round(estimate_operation_force_kn(operation), 3),
        }
        for operation in operations
    )
