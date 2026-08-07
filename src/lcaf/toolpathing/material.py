"""Approximate, practical material models for the toolpath slicer.

Nothing here is a material simulation or a sourced alloy datasheet. Every
constant is a deliberately rough, order-of-magnitude engineering estimate,
used for a separate, practical slab-method force estimate
(``estimate_operation_force_kn``) for planning purposes.

Numbers here never feed back into the planned strike coordinates (x/y/
die_gap/rotation) themselves. Dies are treated as rigid and able to supply
whatever force the estimate says they need.

Historical note: ``formability``/``formability_response`` used to also
drive ``visualization.py``'s deformation preview (a purely
computational-geometry bulge heuristic, gated by how "formable" the chosen
material/temperature was). That preview is now driven by a trained neural
surrogate instead (see ``lcaf.simulation.surrogate`` and
``docs/surrogate_deformation_model.md``), which takes only geometric process
parameters as input -- a given checkpoint is trained for one material/
temperature combination, so this module's ``material``/``target_temperature_c``
no longer have any effect on the animated preview, only on the force/
pressure estimate below. ``formability_response`` is kept as a still-valid,
tested, currently-unused mapping in case a future material-conditioned
surrogate or a force-based UI hint wants it again.
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


# Multiple practical bands per material -- cold/warm/hot working ranges,
# roughly matched to where each material is actually forged/worked in
# practice (plasticine near room temperature, aluminum alloys a few hundred
# C, steels in the 600-1250 C working range). These are approximate
# handbook-order-of-magnitude constants, not sourced from a specific alloy
# datasheet or mill-test-report: good enough to make relative comparisons
# (steel needs more strikes than aluminum at the same formability; a harder
# grade needs more force/pressure than a softer one; force scales sensibly
# with contact area and material) practical without a real simulation.
#
# Each family (plasticine/aluminum/steel) keeps its original generic entry
# (the bare "plasticine"/"aluminum"/"steel" key) for backward compatibility,
# plus several named grades so the UI/CLI can offer a more realistic choice
# than one lump band per family. The generic entry is a reasonable
# "prototypical" mid-range grade of its family, unchanged from before.
_MATERIALS: dict[str, tuple[MaterialBand, ...]] = {
    # ---- Plasticine (physical-modelling clay, worked near room temperature) ----
    "plasticine": (
        MaterialBand(max_temperature_c=15.0, formability=0.55, flow_stress_mpa=0.15, friction_coefficient=0.35),
        MaterialBand(max_temperature_c=30.0, formability=0.85, flow_stress_mpa=0.08, friction_coefficient=0.30),
        MaterialBand(max_temperature_c=float("inf"), formability=0.97, flow_stress_mpa=0.04, friction_coefficient=0.25),
    ),
    "plasticine_soft": (
        MaterialBand(max_temperature_c=15.0, formability=0.65, flow_stress_mpa=0.08, friction_coefficient=0.30),
        MaterialBand(max_temperature_c=30.0, formability=0.92, flow_stress_mpa=0.045, friction_coefficient=0.25),
        MaterialBand(max_temperature_c=float("inf"), formability=0.98, flow_stress_mpa=0.02, friction_coefficient=0.20),
    ),
    "plasticine_hard": (
        MaterialBand(max_temperature_c=15.0, formability=0.40, flow_stress_mpa=0.30, friction_coefficient=0.40),
        MaterialBand(max_temperature_c=30.0, formability=0.70, flow_stress_mpa=0.16, friction_coefficient=0.35),
        MaterialBand(max_temperature_c=float("inf"), formability=0.90, flow_stress_mpa=0.08, friction_coefficient=0.30),
    ),
    # ---- Aluminum alloys ----
    "aluminum": (
        MaterialBand(max_temperature_c=200.0, formability=0.15, flow_stress_mpa=120.0, friction_coefficient=0.40),
        MaterialBand(max_temperature_c=400.0, formability=0.50, flow_stress_mpa=60.0, friction_coefficient=0.35),
        MaterialBand(max_temperature_c=float("inf"), formability=0.90, flow_stress_mpa=25.0, friction_coefficient=0.30),
    ),
    "aluminum_1100": (  # commercially pure, dead-soft
        MaterialBand(max_temperature_c=200.0, formability=0.30, flow_stress_mpa=70.0, friction_coefficient=0.35),
        MaterialBand(max_temperature_c=400.0, formability=0.65, flow_stress_mpa=35.0, friction_coefficient=0.30),
        MaterialBand(max_temperature_c=float("inf"), formability=0.95, flow_stress_mpa=15.0, friction_coefficient=0.25),
    ),
    "aluminum_6061": (  # common structural alloy
        MaterialBand(max_temperature_c=200.0, formability=0.15, flow_stress_mpa=130.0, friction_coefficient=0.40),
        MaterialBand(max_temperature_c=450.0, formability=0.50, flow_stress_mpa=65.0, friction_coefficient=0.35),
        MaterialBand(max_temperature_c=float("inf"), formability=0.88, flow_stress_mpa=28.0, friction_coefficient=0.30),
    ),
    "aluminum_7075": (  # high-strength aerospace alloy, harder to form
        MaterialBand(max_temperature_c=250.0, formability=0.08, flow_stress_mpa=190.0, friction_coefficient=0.45),
        MaterialBand(max_temperature_c=450.0, formability=0.35, flow_stress_mpa=95.0, friction_coefficient=0.38),
        MaterialBand(max_temperature_c=float("inf"), formability=0.75, flow_stress_mpa=45.0, friction_coefficient=0.32),
    ),
    # ---- Steels ----
    "steel": (
        MaterialBand(max_temperature_c=600.0, formability=0.05, flow_stress_mpa=400.0, friction_coefficient=0.50),
        MaterialBand(max_temperature_c=900.0, formability=0.35, flow_stress_mpa=150.0, friction_coefficient=0.40),
        MaterialBand(max_temperature_c=float("inf"), formability=0.85, flow_stress_mpa=60.0, friction_coefficient=0.30),
    ),
    "steel_1018": (  # mild/low-carbon steel
        MaterialBand(max_temperature_c=600.0, formability=0.08, flow_stress_mpa=280.0, friction_coefficient=0.45),
        MaterialBand(max_temperature_c=900.0, formability=0.40, flow_stress_mpa=110.0, friction_coefficient=0.38),
        MaterialBand(max_temperature_c=float("inf"), formability=0.90, flow_stress_mpa=45.0, friction_coefficient=0.28),
    ),
    "steel_4140": (  # chromoly alloy steel, stronger and stiffer
        MaterialBand(max_temperature_c=650.0, formability=0.03, flow_stress_mpa=520.0, friction_coefficient=0.52),
        MaterialBand(max_temperature_c=950.0, formability=0.28, flow_stress_mpa=190.0, friction_coefficient=0.42),
        MaterialBand(max_temperature_c=float("inf"), formability=0.80, flow_stress_mpa=75.0, friction_coefficient=0.32),
    ),
    "steel_304_stainless": (  # austenitic stainless, higher hot flow stress/friction
        MaterialBand(max_temperature_c=700.0, formability=0.03, flow_stress_mpa=580.0, friction_coefficient=0.55),
        MaterialBand(max_temperature_c=1000.0, formability=0.25, flow_stress_mpa=220.0, friction_coefficient=0.45),
        MaterialBand(max_temperature_c=float("inf"), formability=0.78, flow_stress_mpa=95.0, friction_coefficient=0.35),
    ),
}

MATERIALS: tuple[str, ...] = tuple(_MATERIALS)

# Human-readable labels for the UI's material picker -- MATERIALS itself
# stays a plain, stable key (used in JSONL metadata, CLI --material choices,
# and resolve_material_band()); this is presentation-only.
MATERIAL_LABELS: dict[str, str] = {
    "plasticine": "Plasticine (generic)",
    "plasticine_soft": "Plasticine -- soft grade",
    "plasticine_hard": "Plasticine -- hard grade",
    "aluminum": "Aluminum (generic)",
    "aluminum_1100": "Aluminum 1100 (pure, dead-soft)",
    "aluminum_6061": "Aluminum 6061 (structural)",
    "aluminum_7075": "Aluminum 7075 (aerospace, high-strength)",
    "steel": "Steel (generic)",
    "steel_1018": "Steel 1018 (mild/low-carbon)",
    "steel_4140": "Steel 4140 (chromoly alloy)",
    "steel_304_stainless": "Steel 304 (austenitic stainless)",
}


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
    """The two dimensionless scalars the old geometric deformation preview
    consumed. See the module docstring: no longer wired into
    ``visualization.py`` (the surrogate-driven preview does not take
    material/temperature as input), kept as a still-valid mapping.
    """

    reach_scale: float
    closure_fraction: float


def formability_response(formability: float) -> FormabilityResponse:
    """Map a 0..1 formability into the old geometric preview's two knobs.

    ``reach_scale`` (0.4..1.0) scaled how far displaced material bulged out
    from a die's rigid edge: cold/stiff material bulged tightly against the
    die, hot/soft material spread much further before fading out.
    ``closure_fraction`` (0.15..1.0) scaled how much of one strike's own
    exact volume-conserving redistribution was actually applied immediately,
    versus left for a later strike/cycle to keep closing -- cold, stiff
    material took many more hits to fully spread and settle onto the
    target, exactly like real forging practice; hot, soft material nearly
    fully relaxed in one hit. See ``FormabilityResponse``'s docstring: not
    currently consumed by anything.
    """
    formability = max(0.0, min(1.0, formability))
    return FormabilityResponse(
        reach_scale=0.4 + 0.6 * formability,
        closure_fraction=0.15 + 0.85 * formability,
    )


def estimate_strike_contact_pressure_mpa(
    *,
    material: str,
    temperature_c: float,
    contact_width_mm: float,
    instantaneous_height_mm: float,
) -> float:
    """The average die contact pressure (stress, in MPa) needed to induce
    plastic flow in this material at this geometry.

    ``p = flow_stress * (1 + friction * width / (3 * height))`` is the same
    friction-hill correction ``estimate_strike_force_kn`` uses, expressed as
    a pressure -- force per unit contact area -- rather than a total force.
    This is what actually answers "how hard does the die have to push," since
    two strikes needing the same total force can need very different
    pressure if their contact patches differ in size (a small die concentrates
    the same force into far higher stress than a large one). Like the force
    estimate, this is a hand-calculation-grade slab-method figure, not a
    simulation, and never feeds back into the planned coordinates or the
    deformation preview.
    """
    band = resolve_material_band(material, temperature_c)
    height = max(instantaneous_height_mm, 1e-6)
    friction_hill = 1.0 + band.friction_coefficient * (contact_width_mm / (3.0 * height))
    return band.flow_stress_mpa * friction_hill


def estimate_strike_force_kn(
    *,
    material: str,
    temperature_c: float,
    contact_width_mm: float,
    contact_length_mm: float,
    instantaneous_height_mm: float,
) -> float:
    """A slab-method (friction-hill) force estimate for one flat-die strike.

    ``F = contact_pressure * contact_area``, where ``contact_pressure`` is
    ``estimate_strike_contact_pressure_mpa`` above -- the standard closed-form
    flat-die forging force estimate (the friction-hill correction to a
    plane-strain slab analysis; see any metal-forming text, e.g. Kalpakjian &
    Schmid). It is a hand-calculation-grade estimate, not a simulation, and
    is entirely separate from the deformation preview: it never
    feeds back into planned strike coordinates or the animated shape, and
    dies are treated as rigid and able to supply whatever force it says they
    need.
    """
    pressure_mpa = estimate_strike_contact_pressure_mpa(
        material=material,
        temperature_c=temperature_c,
        contact_width_mm=contact_width_mm,
        instantaneous_height_mm=instantaneous_height_mm,
    )
    area_mm2 = max(contact_width_mm, 0.0) * max(contact_length_mm, 0.0)
    force_n = pressure_mpa * area_mm2
    return force_n / 1000.0


@dataclass(frozen=True)
class _OperationStrikeGeometry:
    """The material/geometry inputs one exported STRIKE operation's metadata
    resolves to, shared by the force and contact-pressure estimates below so
    neither has to re-parse ``metadata`` independently."""

    material: str
    temperature_c: float
    contact_width_mm: float
    contact_length_mm: float
    instantaneous_height_mm: float


def _resolve_operation_strike_geometry(operation: dict) -> _OperationStrikeGeometry:
    """Read the material/temperature/die geometry a force or pressure estimate
    needs straight from an operation dict a ``ToolpathPlan`` already produces
    (or one loaded back from an exported JSONL) -- so either estimate can be
    computed standalone from a plan or a JSONL file, without needing the
    original ``SliceSettings`` or mesh.
    """
    metadata = operation.get("metadata", {})
    stock_radius_mm = float(metadata.get("stock_radius_mm", 0.0))
    support_mm = float(metadata.get("target_support_mm", 0.0))
    die_width_mm = float(metadata.get("die_width_mm") or stock_radius_mm)
    upper_die_radius_mm = float(metadata.get("upper_die_radius_mm") or stock_radius_mm)
    die_length_mm = float(metadata.get("die_length_mm") or (2.0 * upper_die_radius_mm))
    return _OperationStrikeGeometry(
        material=metadata.get("material", "steel"),
        temperature_c=float(operation.get("target_temperature", 0.0)),
        contact_width_mm=die_width_mm,
        contact_length_mm=min(2.0 * upper_die_radius_mm, die_length_mm),
        instantaneous_height_mm=stock_radius_mm + support_mm,
    )


def estimate_operation_force_kn(operation: dict) -> float:
    """Estimate one exported STRIKE operation's forging force from its own metadata."""
    geometry = _resolve_operation_strike_geometry(operation)
    return estimate_strike_force_kn(
        material=geometry.material,
        temperature_c=geometry.temperature_c,
        contact_width_mm=geometry.contact_width_mm,
        contact_length_mm=geometry.contact_length_mm,
        instantaneous_height_mm=geometry.instantaneous_height_mm,
    )


def estimate_operation_stress_mpa(operation: dict) -> float:
    """Estimate one exported STRIKE operation's required die contact pressure
    (the stress needed to induce plastic flow) from its own metadata -- the
    same inputs ``estimate_operation_force_kn`` uses, expressed as a pressure
    rather than a total force (see ``estimate_strike_contact_pressure_mpa``).
    """
    geometry = _resolve_operation_strike_geometry(operation)
    return estimate_strike_contact_pressure_mpa(
        material=geometry.material,
        temperature_c=geometry.temperature_c,
        contact_width_mm=geometry.contact_width_mm,
        instantaneous_height_mm=geometry.instantaneous_height_mm,
    )


def plan_force_report(operations: Sequence[dict]) -> tuple[dict, ...]:
    """A per-operation force/stress-estimate report, suitable for a UI readout or export.

    Entirely separate from the JSONL operations themselves: this is never
    written into the controller-facing metadata, only computed alongside it.
    ``estimated_force_kn`` is the total slab-method force;
    ``estimated_contact_pressure_mpa`` is the same estimate expressed as a
    contact stress (force per unit contact area) -- the pressure the die
    actually has to apply to induce plastic flow in the material, independent
    of how large the contact patch happens to be.
    """
    return tuple(
        {
            "step": operation["step"],
            "rotation_deg": operation["rotation"],
            "material": operation.get("metadata", {}).get("material", "steel"),
            "target_temperature_c": operation.get("target_temperature", 0.0),
            "estimated_force_kn": round(estimate_operation_force_kn(operation), 3),
            "estimated_contact_pressure_mpa": round(estimate_operation_stress_mpa(operation), 3),
        }
        for operation in operations
    )
