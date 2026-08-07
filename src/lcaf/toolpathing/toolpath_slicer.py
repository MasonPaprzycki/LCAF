"""A conservative, computational-geometry planner for simple open-die forging.

The planner intentionally models geometry, not metal flow.  The target
mesh's longitudinal extent is divided into ``radial_segments`` axial
regions.  Each region's strike depth is not sampled at one point -- it is
the *numerical average* of the target's true cross-section across that
whole region, integrated from several sub-samples along its span, so a
region spanning a taper is struck with something resembling the mean of its
two ends rather than either end alone.  Each region is struck
``strikes_per_segment`` times at evenly spaced rotations, and the whole
region x strike sweep can be repeated ``cycles`` times end-to-end -- later
cycles true up whatever an earlier (rougher) cycle left rough, since every
operation just replays, in order, against one running material state.

Coordinates are millimetres at the planner boundary.  OBJ files have no unit
metadata, so callers must provide ``scale_mm_per_unit`` when necessary.

Generated ``(x, y, z, rotation)`` coordinates follow the machine's own frame:

- **X** is the billet's long axis, zero-based like the machine itself:
  **X=0 is the clamp** -- the same physical reference LinuxCNC's own
  homing gives machine X=0 (the negative-limit end of travel, see
  docs/hardware_setup.md) -- and every generated X is therefore >= 0,
  increasing outward from the clamp toward the free/forged end of the
  part. Which end of the *target mesh* is clamped is not implied by mesh
  geometry alone, so ``SliceSettings.stock_clamped_end`` says which end
  (``"min"`` or ``"max"`` of the resolved longitudinal axis, in the mesh's
  own coordinates) is held; every generated X is oriented and re-based off
  that end, regardless of how the source mesh happened to be authored.
  ``SliceSettings.x_offset_mm`` still adds on top of this if the clamp
  itself sits some fixed distance away from machine X=0 rather than
  exactly at it.
- **Y** is the radial axis orthogonal to X that the fixed lower die (anvil)
  lies along.
- **Z** is the radial axis orthogonal to both X and Y that the upper
  (striking) die travels along: positive Z drives it into the billet,
  negative Z retracts it. The anvil, on the -Z side, never moves -- it is a
  fixed support, not a second reciprocating tool. The billet -- not the dies
  -- rotates about X between strikes, so ``rotation`` selects which of the
  target's local features is currently presented to the machine's fixed +Z
  strike direction.

This planner only prescribes the coordinates each strike needs.  Execution
order/sequencing (which strike happens first, travel optimisation, etc.) is
the control system's responsibility and has no bearing on which strikes are
generated.
"""

from __future__ import annotations

import json
import logging
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence

from .material import MATERIALS

_logger = logging.getLogger(__name__)


Point3 = tuple[float, float, float]
Point2 = tuple[float, float]
Triangle = tuple[Point3, Point3, Point3]

# Internal, non-user-facing sampling resolutions. _INTEGRATION_SUBSAMPLES is
# how many point cross-sections along a segment's own span get averaged
# together (the "numerical integration" that produces its representative
# polygon); _SUPPORT_ANGLE_SAMPLES is the angular resolution of that
# averaged polygon, independent of (and typically much finer than)
# strikes_per_segment so the preview renders a smooth target outline even
# when few strikes are configured.
_INTEGRATION_SUBSAMPLES = 9
_SUPPORT_ANGLE_SAMPLES = 48


class ToolpathPlanningError(ValueError):
    """Raised when a mesh or requested plan cannot be safely represented."""


@dataclass(frozen=True)
class TriangleMesh:
    """A small dependency-free triangle mesh representation."""

    triangles: tuple[Triangle, ...]
    source: str = ""

    def __post_init__(self) -> None:
        if not self.triangles:
            raise ToolpathPlanningError("The model contains no triangles.")

    @property
    def vertices(self) -> tuple[Point3, ...]:
        return tuple(point for triangle in self.triangles for point in triangle)

    def bounds(self) -> tuple[Point3, Point3]:
        vertices = self.vertices
        return (
            tuple(min(point[index] for point in vertices) for index in range(3)),
            tuple(max(point[index] for point in vertices) for index in range(3)),
        )  # type: ignore[return-value]

    def scaled(self, factor: float) -> "TriangleMesh":
        if factor <= 0:
            raise ToolpathPlanningError("scale_mm_per_unit must be positive.")
        return TriangleMesh(
            tuple(
                tuple(tuple(value * factor for value in point) for point in triangle)
                for triangle in self.triangles
            ),
            self.source,
        )


@dataclass(frozen=True)
class MachineLimits:
    """Machine-coordinate limits used to reject unsafe generated operations.

    Zero-based on every axis, matching the machine's own
    [-retracted_distance, extended_distance] convention (see
    docs/hardware_setup.md) -- these defaults are only a fallback for when
    configs/axis.json can't be read (see from_lcaf_config()); real limits
    should always come from there.
    """

    x_min_mm: float = 0.0
    x_max_mm: float = 100.0
    y_min_mm: float = 0.0
    y_max_mm: float = 100.0
    z_retracted_mm: float = 0.0
    z_extended_mm: float = 100.0

    @classmethod
    def from_lcaf_config(cls, filename: str | Path) -> "MachineLimits":
        """Read machine X/Y/Z travel limits from ``configs/axis.json``.

        These are the same retracted_distance/extended_distance each
        JointConfiguration carries for LinuxCNC's own INI generation (see
        lcaf.utils.joint_configuration) -- there is a single on-disk source
        for machine travel limits, not a separate copy for the slicer.
        JointConfiguration stores these in inches (configs/machine.json
        declares LINEAR_UNITS=inch for LinuxCNC itself) measured from a
        fixed zero (every joint's travel is [-retracted_distance,
        extended_distance], see the retracted_distance/extended_distance
        docstrings), while this planner's whole coordinate space is
        millimetres (see module docstring), so both are scaled by 25.4 on
        the way in.

        This lines up with the planner's own X convention: X=0 is the clamp
        (see module docstring), the same physical point homing gives machine
        X=0, so every generated X is already >= 0 by construction -- there
        is no separate reconciliation needed between the planner's frame and
        this zero-based machine range (X/Y's retracted_distance is 0 in
        axis.json, matching that convention).

        The rotary (A) axis is continuous on this machine (see
        JointConfiguration.has_limit_switches) and is not limited here; only
        the linear X/Y/Z travel is validated against this file.
        """
        from lcaf.utils.joint_configuration import load_joint_configurations

        path = Path(filename)
        try:
            joints = load_joint_configurations(path)
        except (OSError, ValueError) as error:
            raise ToolpathPlanningError(
                f"Could not read machine limits from {path}: {error}"
            ) from error

        by_axis = {joint.axis: joint for joint in joints}

        def bound_mm(joint, distance, sign: float, field_name: str) -> float:
            """Convert one signed native-unit distance to millimetres, or --
            if the joint's axis.json leaves it null -- disable this end of
            the software travel-limit check entirely (see
            JointConfiguration.retracted_distance/extended_distance) and log
            a warning, since that removes a real crash-prevention check on
            this planner's generated moves.
            """
            if distance is None:
                _logger.warning(
                    f"MachineLimits.from_lcaf_config({path}): {joint.axis} axis has a null "
                    f"{field_name} -- this disables the toolpath planner's software "
                    f"travel-limit check on that end of {joint.axis}; only a physical limit "
                    "switch or mechanical stop protects it now."
                )
                return sign * math.inf
            return sign * float(distance) * 25.4

        try:
            x, y, z = by_axis["X"], by_axis["Y"], by_axis["Z"]

            return cls(
                x_min_mm=bound_mm(x, x.retracted_distance, -1.0, "retracted_distance"),
                x_max_mm=bound_mm(x, x.extended_distance, 1.0, "extended_distance"),
                y_min_mm=bound_mm(y, y.retracted_distance, -1.0, "retracted_distance"),
                y_max_mm=bound_mm(y, y.extended_distance, 1.0, "extended_distance"),
                z_retracted_mm=bound_mm(z, z.retracted_distance, -1.0, "retracted_distance"),
                z_extended_mm=bound_mm(z, z.extended_distance, 1.0, "extended_distance"),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ToolpathPlanningError(
                f"Joint configuration {path} is missing an X/Y/Z travel limit: {error}"
            ) from error


@dataclass(frozen=True)
class SliceSettings:
    """Inputs to the simplified toolpath slicer.

    ``die_contact_z_mm`` is a calibration value: the machine Z coordinate at
    which a die first contacts the unformed cylindrical stock.  The generated
    Z command is ``contact + stock_radius - requested_support``.  It must be
    established by setup/proving, not inferred from CAD.

    ``radial_segments`` divides the target's longitudinal extent into that
    many axial regions.  Each region's strike depth is the numerical average
    of the target's true cross-section across the whole region (not a single
    point sample), so a region spanning a taper is struck with something
    resembling the mean of its two ends.  Each region is struck
    ``strikes_per_segment`` times, at rotations ``360/strikes_per_segment``
    degrees apart.  ``cycles`` repeats the entire region x strike sweep this
    many times end-to-end; later cycles true up whatever an earlier
    (rougher) cycle left rough, since every operation just replays, in
    order, against one running material state -- no cycle-aware logic is
    needed beyond simply repeating the sweep.  ``max_reduction_per_strike_mm``
    remains a separate, orthogonal cap: any single (segment, rotation)
    strike whose needed reduction exceeds it is still split into multiple
    retract/re-strike depth passes, exactly as before.

    Two rigid surfaces act on every strike: a moving (striking) die and a
    fixed anvil, and -- for the *preview* only -- both now have a real,
    finite contact footprint by default (not the machine-command coordinates
    themselves; see below).  ``upper_die_radius_mm`` describes the striking
    die (machine +Z) as a flat-faced circular disc of that radius.
    ``die_width_mm`` and ``die_length_mm`` describe the fixed anvil (machine
    -Z, never moves): the finite rectangle of its face that supports the
    billet (width across the strike/tangential direction, length along the
    billet axis).  ``die_corner_radius_mm`` blends the tangential edges of
    the anvil's rectangle into a radius instead of a sharp corner.

    Leaving ``die_width_mm`` or ``upper_die_radius_mm`` unset (``None``)
    does not mean "unconstrained" -- it means "pick a sensible physical
    default from the stock geometry" (see ``__post_init__``), so that the
    preview conserves volume by bulging displaced material sideways, the
    way real forge-temperature steel does, without requiring the user to
    configure die dimensions first.  The default ``upper_die_radius_mm``
    (``stock_radius_mm``) is large enough to always fully cover the target
    at the one segment a strike targets, so the final preview still always
    converges exactly to the target at that default; an explicitly
    *smaller* radius is an honest physical trade-off -- like a real round
    punch smaller than a face, it can legitimately leave parts of that face
    unstruck from one static position.  ``die_length_mm`` left unset
    defaults to the *striking segment's own axial width* (resolved once the
    mesh is known, in ``plan()``) -- by default both dies span their whole
    segment, matching the segment model; an explicit ``die_length_mm`` can
    still narrow or widen the anvil specifically.

    None of these settings change the strike coordinates themselves -- the
    target's final geometry is unaffected by them.
    ``lcaf.toolpathing.visualization``'s preview is driven by a trained
    ``lcaf.simulation.surrogate`` network (see
    ``docs/surrogate_deformation_model.md``), not by these die-footprint
    settings: the network was trained assuming both dies are wide enough to
    fully support the workpiece (the paper's own assumption -- see
    ``lcaf/simulation/surrogate/README.md``), so ``die_width_mm``,
    ``upper_die_radius_mm``, and ``die_corner_radius_mm`` now only affect the
    *rendered* die/anvil footprint in the preview, not the predicted
    deformation itself. ``die_length_mm`` (the axial bite length) is the
    exception -- it is read directly as the surrogate's own bite-ratio input.

    ``material`` (one of ``lcaf.toolpathing.material.MATERIALS`` --
    currently ``"plasticine"``, ``"aluminum"``, ``"steel"``) and
    ``target_temperature_c`` do not affect the preview at all (a given
    surrogate checkpoint is trained for one material/temperature
    combination, matching the paper's own scope) -- they only drive the
    separate, independent slab-method force estimate
    (``lcaf.toolpathing.material.estimate_operation_force_kn``), for
    reporting only, never feeding back into the preview or the planned
    coordinates.

    ``stock_clamped_end`` says which end of the target mesh, along its
    resolved longitudinal axis, is held in the clamp: ``"min"`` for the
    mesh's lower bound on that axis, ``"max"`` for its upper bound.  Every
    generated X is oriented so that increasing X always means "away from the
    clamp," flipping the mesh's own local axis direction if needed -- the
    source mesh may have been authored with either end at the origin.
    """

    stock_radius_mm: float
    radial_segments: int = 4
    strikes_per_segment: int = 4
    cycles: int = 1
    max_reduction_per_strike_mm: float = 2.0
    die_contact_z_mm: float = 0.0
    x_offset_mm: float = 0.0
    y_position_mm: float = 0.0
    target_temperature_c: float = 0.0
    material: str = "steel"
    scale_mm_per_unit: float = 1.0
    longitudinal_axis: str = "auto"
    die_width_mm: float | None = None
    die_length_mm: float | None = None
    die_corner_radius_mm: float = 0.0
    upper_die_radius_mm: float | None = None
    stock_clamped_end: str = "min"

    def __post_init__(self) -> None:
        # None means "derive a sensible physical default from the stock
        # geometry," not "unconstrained" -- see the class docstring. Resolve
        # it here, before validate() runs, so every other consumer of this
        # frozen dataclass always sees concrete positive floats. die_length_mm
        # is the one exception: its natural default (the striking segment's
        # own width) depends on the mesh, which isn't known yet here -- it is
        # resolved later, per segment, inside ToolpathSlicer.plan().
        if self.die_width_mm is None:
            object.__setattr__(self, "die_width_mm", self.stock_radius_mm)
        if self.upper_die_radius_mm is None:
            object.__setattr__(self, "upper_die_radius_mm", self.stock_radius_mm)

    def validate(self) -> None:
        if self.stock_radius_mm <= 0:
            raise ToolpathPlanningError("stock_radius_mm must be positive.")
        if self.radial_segments < 1:
            raise ToolpathPlanningError("radial_segments must be at least 1.")
        if self.strikes_per_segment < 1:
            raise ToolpathPlanningError("strikes_per_segment must be at least 1.")
        if self.cycles < 1:
            raise ToolpathPlanningError("cycles must be at least 1.")
        if self.max_reduction_per_strike_mm <= 0:
            raise ToolpathPlanningError(
                "max_reduction_per_strike_mm must be positive."
            )
        if self.stock_clamped_end.lower() not in {"min", "max"}:
            raise ToolpathPlanningError("stock_clamped_end must be 'min' or 'max'.")
        if self.longitudinal_axis.lower() not in {"auto", "x", "y", "z"}:
            raise ToolpathPlanningError(
                "longitudinal_axis must be one of auto, x, y, or z."
            )
        if self.scale_mm_per_unit <= 0:
            raise ToolpathPlanningError("scale_mm_per_unit must be positive.")
        if self.material.strip().lower() not in MATERIALS:
            raise ToolpathPlanningError(
                f"Unknown material '{self.material}'. Choose one of: {', '.join(MATERIALS)}."
            )
        # die_width_mm/upper_die_radius_mm are always concrete by this point
        # -- __post_init__ already resolved any None to a default -- so an
        # explicitly-passed non-positive value is the only way these can
        # still be invalid. die_length_mm may still legitimately be None
        # (resolved later against the mesh), so it needs its own guard.
        if self.die_width_mm <= 0:
            raise ToolpathPlanningError("die_width_mm must be positive when specified.")
        if self.die_length_mm is not None and self.die_length_mm <= 0:
            raise ToolpathPlanningError("die_length_mm must be positive when specified.")
        if self.upper_die_radius_mm <= 0:
            raise ToolpathPlanningError("upper_die_radius_mm must be positive when specified.")
        if self.die_corner_radius_mm < 0:
            raise ToolpathPlanningError("die_corner_radius_mm cannot be negative.")
        if self.die_corner_radius_mm > self.die_width_mm / 2.0 + 1e-9:
            raise ToolpathPlanningError(
                "die_corner_radius_mm cannot exceed half of die_width_mm."
            )


@dataclass(frozen=True)
class Segment:
    """A representative convex cross-section for one axial (X) region.

    ``polygon_yz_mm`` is the numerically integrated average of the target's
    true cross-section across ``[x_start_mm, x_end_mm]`` -- not a single
    point sample -- so a segment spanning a taper is struck with something
    resembling the mean of its two ends, not either end alone. A "point"
    segment (used internally for fine sampling) has ``x_start_mm ==
    x_end_mm``.
    """

    x_start_mm: float
    x_end_mm: float
    polygon_yz_mm: tuple[Point2, ...]

    @property
    def x_model_mm(self) -> float:
        """The segment's axial centre -- the single X a strike is commanded at."""
        return (self.x_start_mm + self.x_end_mm) / 2.0

    def support_mm(self, rotation_deg: float) -> float:
        """Return the positive die-direction support after an X-axis rotation."""
        angle = math.radians(rotation_deg)
        sine, cosine = math.sin(angle), math.cos(angle)
        return max(y * sine + z * cosine for y, z in self.polygon_yz_mm)

    def area_mm2(self) -> float:
        """Shoelace area of the convex cross-section polygon."""
        polygon = self.polygon_yz_mm
        count = len(polygon)
        total = 0.0
        for index in range(count):
            y1, z1 = polygon[index]
            y2, z2 = polygon[(index + 1) % count]
            total += y1 * z2 - y2 * z1
        return abs(total) / 2.0


@dataclass(frozen=True)
class ToolpathPlan:
    """The controller-compatible JSONL operations plus preview geometry."""

    operations: tuple[dict, ...]
    sections: tuple[Segment, ...]
    rotations_deg: tuple[float, ...]
    warnings: tuple[str, ...] = field(default_factory=tuple)
    target_volume_mm3: float = 0.0
    recommended_stock_length_mm: float = 0.0

    def to_jsonl(self) -> str:
        return "\n".join(json.dumps(operation, sort_keys=True) for operation in self.operations) + "\n"

    def write_jsonl(self, filename: str | Path) -> Path:
        path = Path(filename)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(self.to_jsonl(), encoding="utf-8")
        return path


def load_mesh(filename: str | Path) -> TriangleMesh:
    """Load a triangulated OBJ or ASCII/binary STL mesh without extra packages.

    Native ``.sldprt`` files are proprietary feature-history containers.  They
    need a CAD translator (for example a SolidWorks/FreeCAD STL export) before
    this deterministic mesh planner can consume them.
    """
    path = Path(filename)
    if not path.exists():
        raise ToolpathPlanningError(f"Model does not exist: {path}")

    extension = path.suffix.lower()
    if extension == ".obj":
        return _load_obj(path)
    if extension == ".stl":
        return _load_stl(path)
    if extension == ".sldprt":
        raise ToolpathPlanningError(
            "Native SolidWorks .sldprt import is not implemented because the "
            "format is proprietary. Export this part as a watertight STL (or "
            "OBJ) from SolidWorks/FreeCAD, then load that exported mesh."
        )
    raise ToolpathPlanningError(
        f"Unsupported model type '{extension}'. Use OBJ, STL, or export SLDPRT to STL."
    )


def _load_obj(path: Path) -> TriangleMesh:
    vertices: list[Point3] = []
    triangles: list[Triangle] = []

    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError as error:
        raise ToolpathPlanningError(f"Could not read OBJ {path}: {error}") from error

    for line_number, raw_line in enumerate(lines, start=1):
        parts = raw_line.strip().split()
        if not parts or parts[0].startswith("#"):
            continue
        if parts[0] == "v" and len(parts) >= 4:
            try:
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError as error:
                raise ToolpathPlanningError(
                    f"Invalid OBJ vertex at {path}:{line_number}."
                ) from error
        elif parts[0] == "f" and len(parts) >= 4:
            indices: list[int] = []
            for token in parts[1:]:
                try:
                    raw_index = int(token.split("/")[0])
                except ValueError as error:
                    raise ToolpathPlanningError(
                        f"Invalid OBJ face at {path}:{line_number}."
                    ) from error
                index = raw_index - 1 if raw_index > 0 else len(vertices) + raw_index
                if index < 0 or index >= len(vertices):
                    raise ToolpathPlanningError(
                        f"OBJ face references a missing vertex at {path}:{line_number}."
                    )
                indices.append(index)
            for offset in range(1, len(indices) - 1):
                triangles.append(
                    (vertices[indices[0]], vertices[indices[offset]], vertices[indices[offset + 1]])
                )

    if not triangles:
        raise ToolpathPlanningError(f"OBJ {path} contains no triangular faces.")
    return TriangleMesh(tuple(triangles), str(path))


def _load_stl(path: Path) -> TriangleMesh:
    data = path.read_bytes()
    if len(data) < 84:
        raise ToolpathPlanningError(f"STL {path} is too short to contain a mesh.")

    # Binary STL has an exact 84 + 50*n byte layout.  This also avoids treating
    # an ASCII STL whose header starts with 'solid' as binary.
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    if len(data) == 84 + triangle_count * 50:
        return _load_binary_stl(data, path, triangle_count)
    return _load_ascii_stl(data, path)


def _load_binary_stl(data: bytes, path: Path, triangle_count: int) -> TriangleMesh:
    triangles: list[Triangle] = []
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack_from("<12fH", data, offset)
        triangles.append(
            (
                (float(values[3]), float(values[4]), float(values[5])),
                (float(values[6]), float(values[7]), float(values[8])),
                (float(values[9]), float(values[10]), float(values[11])),
            )
        )
        offset += 50
    return TriangleMesh(tuple(triangles), str(path))


def _load_ascii_stl(data: bytes, path: Path) -> TriangleMesh:
    triangles: list[Triangle] = []
    pending: list[Point3] = []
    for line_number, raw_line in enumerate(data.decode("utf-8", errors="replace").splitlines(), start=1):
        parts = raw_line.strip().split()
        if len(parts) >= 4 and parts[0].lower() == "vertex":
            try:
                pending.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError as error:
                raise ToolpathPlanningError(
                    f"Invalid STL vertex at {path}:{line_number}."
                ) from error
            if len(pending) == 3:
                triangles.append((pending[0], pending[1], pending[2]))
                pending.clear()

    if pending or not triangles:
        raise ToolpathPlanningError(f"STL {path} does not contain complete triangular facets.")
    return TriangleMesh(tuple(triangles), str(path))


class ToolpathSlicer:
    """Build controller JSONL from a simple convex radial target profile."""

    def __init__(self, mesh: TriangleMesh, settings: SliceSettings, limits: MachineLimits):
        settings.validate()
        self.mesh = mesh.scaled(settings.scale_mm_per_unit)
        self.settings = settings
        self.limits = limits
        self._axis = self._resolve_longitudinal_axis()
        self._origin = self._mesh_center()
        # Machine +X always means "away from the clamp." The mesh's own
        # local axis may point either way, so this flips it when needed.
        self._x_sign = 1.0 if settings.stock_clamped_end.lower() == "min" else -1.0
        # Machine X=0 is the clamp, not the mesh's own centre -- see the
        # module docstring. The clamp is whichever bound of the mesh's
        # longitudinal extent stock_clamped_end names; every generated X is
        # measured from that bound, not from _origin (which the two radial
        # axes still use to centre Y/Z, unrelated to this).
        lower, upper = self.mesh.bounds()
        self._x_reference = lower[self._axis] if settings.stock_clamped_end.lower() == "min" else upper[self._axis]

    def plan(self) -> ToolpathPlan:
        strike_rotations = self._strike_rotations()
        self._validate_fixed_machine_positions()
        segments, fine_samples = self._build_segments()
        self._validate_target_fits_stock(fine_samples)
        operations: list[dict] = []
        warnings = [
            "This is a geometric die-envelope plan. It does not model material flow, "
            "springback, temperature-dependent deformation, flash, or die compliance.",
            "Concave and hollow cross-sections are reduced to their convex outer support envelope.",
            "Prove die_contact_z_mm, tooling clearance, and all machine limits before execution.",
        ]
        target_volume_mm3 = _integrate_target_volume_mm3(segments)
        recommended_stock_length_mm = target_volume_mm3 / (
            math.pi * self.settings.stock_radius_mm**2
        )
        target_length_mm = segments[-1].x_model_mm - segments[0].x_model_mm if len(segments) > 1 else 0.0
        warnings.append(
            f"Target volume is ~{target_volume_mm3:.1f} mm^3. At stock_radius_mm="
            f"{self.settings.stock_radius_mm:.3f} mm, a stock cylinder needs to be "
            f"~{recommended_stock_length_mm:.1f} mm long to contain the same volume "
            f"(the target's own axial extent is {target_length_mm:.1f} mm); cut stock to "
            "this length so the process is not required to remove volume that has "
            "nowhere to go, or add volume that was never there."
        )

        # Enumerate every required (cycle, segment, strike) in a fixed,
        # simple order: cycles outermost so a partial/aborted run still
        # leaves a fully-formed earlier cycle behind. This planner only
        # prescribes the coordinates each strike needs; execution
        # order/sequencing is the control system's responsibility, so this
        # ordering must not affect which strikes are generated -- only the
        # "step" numbering used to label them.
        step = 1
        for cycle_index in range(self.settings.cycles):
            for segment_index, segment in enumerate(segments):
                # die_length_mm left unset defaults to this segment's own
                # axial width -- both dies span their whole segment by
                # default, matching the segment model (see SliceSettings).
                resolved_die_length_mm = (
                    self.settings.die_length_mm
                    if self.settings.die_length_mm is not None
                    else (segment.x_end_mm - segment.x_start_mm)
                )
                for strike_index, rotation in enumerate(strike_rotations):
                    support = segment.support_mm(rotation)
                    if support < -1e-6:
                        raise ToolpathPlanningError(
                            "The target is not centered inside the stock at "
                            f"X={segment.x_model_mm:.3f} mm; positive die support is negative."
                        )
                    if support > self.settings.stock_radius_mm + 1e-6:
                        raise ToolpathPlanningError(
                            "Target extends outside the cylindrical stock at "
                            f"X={segment.x_model_mm:.3f} mm, rotation={rotation:.1f}°: "
                            f"target support {support:.3f} mm exceeds stock radius "
                            f"{self.settings.stock_radius_mm:.3f} mm."
                        )

                    reduction = max(0.0, self.settings.stock_radius_mm - support)
                    strikes = max(
                        1,
                        math.ceil(reduction / self.settings.max_reduction_per_strike_mm),
                    )
                    for pass_index in range(1, strikes + 1):
                        applied_reduction = reduction * pass_index / strikes
                        die_z = self.settings.die_contact_z_mm + applied_reduction
                        self._validate_operation(
                            x=segment.x_model_mm + self.settings.x_offset_mm,
                            y=self.settings.y_position_mm,
                            z=die_z,
                        )
                        operations.append(
                            {
                                "step": step,
                                "operation": "STRIKE",
                                "x": _round_machine_value(segment.x_model_mm + self.settings.x_offset_mm),
                                "y": _round_machine_value(self.settings.y_position_mm),
                                "die_gap": _round_machine_value(die_z),
                                "rotation": _round_machine_value(rotation),
                                "target_temperature": _round_machine_value(
                                    self.settings.target_temperature_c
                                ),
                                "metadata": {
                                    "generator": "lcaf.toolpath_slicer",
                                    "model_x_mm": _round_machine_value(segment.x_model_mm),
                                    "segment_index": segment_index,
                                    "segment_x_start_mm": _round_machine_value(segment.x_start_mm),
                                    "segment_x_end_mm": _round_machine_value(segment.x_end_mm),
                                    "cycle_index": cycle_index,
                                    "strike_index": strike_index,
                                    "rotation_deg": _round_machine_value(rotation),
                                    "strike_pass": pass_index,
                                    "strike_pass_count": strikes,
                                    "target_support_mm": _round_machine_value(support),
                                    "stock_radius_mm": _round_machine_value(
                                        self.settings.stock_radius_mm
                                    ),
                                    "radial_reduction_mm": _round_machine_value(applied_reduction),
                                    "geometry_model": "convex_support_envelope",
                                    "die_width_mm": _round_machine_value(self.settings.die_width_mm),
                                    "die_length_mm": _round_machine_value(resolved_die_length_mm),
                                    "die_corner_radius_mm": _round_machine_value(
                                        self.settings.die_corner_radius_mm
                                    ),
                                    "die_shape": (
                                        "radiused" if self.settings.die_corner_radius_mm > 0 else "rectangular"
                                    ),
                                    "upper_die_radius_mm": _round_machine_value(
                                        self.settings.upper_die_radius_mm
                                    ),
                                    "material": self.settings.material.strip().lower(),
                                },
                            }
                        )
                        step += 1

        return ToolpathPlan(
            tuple(operations),
            segments,
            strike_rotations,
            tuple(warnings),
            target_volume_mm3=target_volume_mm3,
            recommended_stock_length_mm=recommended_stock_length_mm,
        )

    def _resolve_longitudinal_axis(self) -> int:
        requested = self.settings.longitudinal_axis.lower()
        if requested in {"x", "y", "z"}:
            return {"x": 0, "y": 1, "z": 2}[requested]
        lower, upper = self.mesh.bounds()
        dimensions = [upper[index] - lower[index] for index in range(3)]
        return max(range(3), key=dimensions.__getitem__)

    def _mesh_center(self) -> Point3:
        lower, upper = self.mesh.bounds()
        return tuple((lower[index] + upper[index]) / 2.0 for index in range(3))  # type: ignore[return-value]

    @property
    def _radial_axes(self) -> tuple[int, int]:
        return tuple(axis for axis in range(3) if axis != self._axis)  # type: ignore[return-value]

    def _build_segments(self) -> tuple[tuple[Segment, ...], tuple[Segment, ...]]:
        """Divide the mesh's longitudinal extent into ``radial_segments`` regions.

        Each region's own representative polygon is the numerical average
        (over ``_INTEGRATION_SUBSAMPLES`` evenly spaced point cross-sections
        within it) of the target's true cross-section across that region --
        not a single point sample.  Returns ``(segments, fine_samples)``:
        ``segments`` is the coarse, ``radial_segments``-long tuple used for
        striking and preview; ``fine_samples`` is every one of those
        individual point cross-sections (each returned as its own
        zero-width ``Segment``), kept *un-averaged* so
        ``_validate_target_fits_stock`` can still catch a true local
        protrusion the averaged polygon would otherwise smooth away.
        """
        lower, upper = self.mesh.bounds()
        start, end = lower[self._axis], upper[self._axis]
        length = end - start
        if length <= 1e-9:
            raise ToolpathPlanningError("The selected longitudinal axis has zero length.")

        segment_count = self.settings.radial_segments
        segments: list[Segment] = []
        fine_samples: list[Segment] = []

        for segment_index in range(segment_count):
            region_start = start + length * segment_index / segment_count
            region_end = start + length * (segment_index + 1) / segment_count
            subsamples = [
                self._point_cross_section(
                    region_start + (region_end - region_start) * sub_index / (_INTEGRATION_SUBSAMPLES - 1)
                )
                for sub_index in range(_INTEGRATION_SUBSAMPLES)
            ]
            fine_samples.extend(subsamples)

            support_samples: list[tuple[float, float]] = []
            for angle_index in range(_SUPPORT_ANGLE_SAMPLES):
                angle_deg = angle_index * 360.0 / _SUPPORT_ANGLE_SAMPLES
                angle = math.radians(angle_deg)
                average_support = sum(
                    sample.support_mm(angle_deg) for sample in subsamples
                ) / len(subsamples)
                support_samples.append((angle, average_support))
            averaged_polygon = _polygon_from_support_samples(support_samples)

            x_start_model = subsamples[0].x_model_mm
            x_end_model = subsamples[-1].x_model_mm
            segments.append(
                Segment(
                    x_start_mm=min(x_start_model, x_end_model),
                    x_end_mm=max(x_start_model, x_end_model),
                    polygon_yz_mm=tuple(averaged_polygon),
                )
            )

        if self._x_sign < 0:
            # Regions were built in ascending mesh-position order; negating
            # x_model_mm reverses that, so reverse the list back to
            # ascending machine-X order.
            segments.reverse()
        return tuple(segments), tuple(fine_samples)

    def _point_cross_section(self, model_position: float) -> Segment:
        """The mesh's exact convex cross-section at one axial position.

        Returned as a zero-width ``Segment`` (``x_start_mm == x_end_mm``)
        so it shares the same ``support_mm``/``polygon_yz_mm`` API as an
        averaged, multi-sample segment.
        """
        first_radial, second_radial = self._radial_axes
        points: list[Point2] = []
        for triangle in self.mesh.triangles:
            points.extend(
                _triangle_plane_points(
                    triangle,
                    self._axis,
                    model_position,
                    first_radial,
                    second_radial,
                )
            )

        hull = _convex_hull(_deduplicate_points(points))
        if len(hull) < 3:
            raise ToolpathPlanningError(
                "Could not form a closed cross-section at "
                f"X={self._x_sign * (model_position - self._x_reference):.3f} mm. "
                "Use a watertight triangle mesh and a valid longitudinal axis."
            )

        x_model_mm = self._x_sign * (model_position - self._x_reference)
        return Segment(
            x_start_mm=x_model_mm,
            x_end_mm=x_model_mm,
            polygon_yz_mm=tuple(
                (
                    point[0] - self._origin[first_radial],
                    point[1] - self._origin[second_radial],
                )
                for point in hull
            ),
        )

    def _strike_rotations(self) -> tuple[float, ...]:
        """``strikes_per_segment`` rotations, evenly spaced across the full 360°."""
        count = self.settings.strikes_per_segment
        return tuple(index * 360.0 / count for index in range(count))

    def _validate_target_fits_stock(self, fine_samples: Sequence[Segment]) -> None:
        """Reject a target whose true convex hull pokes outside the stock cylinder.

        ``Segment.support_mm`` is only ever sampled at the configured
        discrete strike rotations later on, so a target whose corners
        exceed the stock radius *between* those rotations (for example a
        square's diagonal corner when only its 0/90/180/270 faces are
        struck) could otherwise pass unnoticed. Checking every vertex of
        every fine, un-averaged point cross-section -- not the coarser
        averaged segment polygons used for striking -- catches this
        regardless of the chosen strikes_per_segment, and also catches a
        true local protrusion the segment averaging would otherwise smooth
        away. Every vertex of a convex polygon is necessarily its farthest
        point in some direction, so checking every vertex against the stock
        radius is sufficient.
        """
        for sample in fine_samples:
            for y, z in sample.polygon_yz_mm:
                if math.hypot(y, z) > self.settings.stock_radius_mm + 1e-6:
                    raise ToolpathPlanningError(
                        "Target extends outside the cylindrical stock at "
                        f"X={sample.x_model_mm:.3f} mm: a corner reaches "
                        f"{math.hypot(y, z):.3f} mm from the axis, exceeding "
                        f"stock_radius_mm={self.settings.stock_radius_mm:.3f}. This can "
                        "happen even when every configured rotation's own support fits, "
                        "if a corner falls between rotations -- increase stock_radius_mm "
                        "or add a rotation aligned with that corner."
                    )

    def _validate_fixed_machine_positions(self) -> None:
        if not self.limits.y_min_mm <= self.settings.y_position_mm <= self.limits.y_max_mm:
            raise ToolpathPlanningError(
                f"Configured Y={self.settings.y_position_mm:.3f} mm is outside "
                f"machine limits [{self.limits.y_min_mm:.3f}, {self.limits.y_max_mm:.3f}] mm."
            )
        if not self.limits.z_retracted_mm <= self.settings.die_contact_z_mm <= self.limits.z_extended_mm:
            raise ToolpathPlanningError(
                f"die_contact_z_mm={self.settings.die_contact_z_mm:.3f} is outside "
                f"machine limits [{self.limits.z_retracted_mm:.3f}, "
                f"{self.limits.z_extended_mm:.3f}] mm."
            )

    def _validate_operation(self, *, x: float, y: float, z: float) -> None:
        checks = (
            ("X", x, self.limits.x_min_mm, self.limits.x_max_mm),
            ("Y", y, self.limits.y_min_mm, self.limits.y_max_mm),
            ("Z/die_gap", z, self.limits.z_retracted_mm, self.limits.z_extended_mm),
        )
        for axis, value, minimum, maximum in checks:
            if not minimum - 1e-8 <= value <= maximum + 1e-8:
                raise ToolpathPlanningError(
                    f"Generated {axis}={value:.3f} is outside machine limits "
                    f"[{minimum:.3f}, {maximum:.3f}]."
                )


def _triangle_plane_points(
    triangle: Triangle,
    longitudinal_axis: int,
    plane: float,
    first_radial: int,
    second_radial: int,
) -> list[Point2]:
    """Intersect one triangle with a plane and return its radial endpoints."""
    epsilon = 1e-8
    intersections: list[Point3] = []
    for first, second in zip(triangle, triangle[1:] + triangle[:1]):
        first_distance = first[longitudinal_axis] - plane
        second_distance = second[longitudinal_axis] - plane
        if abs(first_distance) <= epsilon:
            intersections.append(first)
        if first_distance * second_distance < -epsilon * epsilon:
            fraction = first_distance / (first_distance - second_distance)
            intersections.append(
                tuple(
                    first[index] + fraction * (second[index] - first[index])
                    for index in range(3)
                )  # type: ignore[arg-type]
            )
        elif abs(second_distance) <= epsilon:
            intersections.append(second)

    return [
        (point[first_radial], point[second_radial])
        for point in _deduplicate_points_3d(intersections)
    ]


def _deduplicate_points_3d(points: Iterable[Point3], tolerance: float = 1e-7) -> list[Point3]:
    seen: set[tuple[int, int, int]] = set()
    result: list[Point3] = []
    for point in points:
        key = tuple(round(value / tolerance) for value in point)
        if key not in seen:
            seen.add(key)
            result.append(point)
    return result


def _deduplicate_points(points: Iterable[Point2], tolerance: float = 1e-7) -> list[Point2]:
    seen: set[tuple[int, int]] = set()
    result: list[Point2] = []
    for point in points:
        key = tuple(round(value / tolerance) for value in point)
        if key not in seen:
            seen.add(key)
            result.append(point)
    return result


def _convex_hull(points: Sequence[Point2]) -> list[Point2]:
    """Return a counter-clockwise monotonic-chain hull."""
    ordered = sorted(points)
    if len(ordered) <= 1:
        return list(ordered)

    def cross(origin: Point2, first: Point2, second: Point2) -> float:
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower: list[Point2] = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 1e-10:
            lower.pop()
        lower.append(point)
    upper: list[Point2] = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 1e-10:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _polygon_from_support_samples(samples: Sequence[tuple[float, float]]) -> tuple[Point2, ...]:
    """The convex polygon defined by intersecting every ``{p : p . (sin a,
    cos a) <= h}`` half-plane from a set of ``(angle_rad, h)`` support
    samples.

    This is the mathematically correct way to reconstruct/bound a convex
    shape from support-function samples.  Naively placing a "vertex" at
    ``(h*sin(a), h*cos(a))`` is only actually on the true boundary when
    ``a`` happens to align with a face normal or vertex direction -- for a
    square, only its 4 face-normal directions -- everywhere else it
    overshoots *outside* the true shape, since the support value at an
    in-between angle corresponds to a corner (not a point straight out in
    that exact direction). That overshoot is exactly what made a segment's
    averaged polygon come out larger than the true averaged shape.
    """
    if not samples:
        return ()
    bound = max(1.0, max(h for _, h in samples)) * 4.0
    polygon: list[Point2] = [(-bound, -bound), (bound, -bound), (bound, bound), (-bound, bound)]
    for angle, h in samples:
        polygon = _clip_polygon_by_halfplane(polygon, (math.sin(angle), math.cos(angle)), h)
        if not polygon:
            break
    return tuple(polygon)


def _clip_polygon_by_halfplane(polygon: Sequence[Point2], normal: Point2, offset: float) -> list[Point2]:
    """Sutherland-Hodgman clip of a convex polygon by ``{p : p . normal <= offset}``."""
    if not polygon:
        return []
    result: list[Point2] = []
    count = len(polygon)
    for index in range(count):
        current = polygon[index]
        previous = polygon[index - 1]
        current_inside = (current[0] * normal[0] + current[1] * normal[1]) <= offset + 1e-9
        previous_inside = (previous[0] * normal[0] + previous[1] * normal[1]) <= offset + 1e-9
        if current_inside:
            if not previous_inside:
                result.append(_halfplane_line_intersection(previous, current, normal, offset))
            result.append(current)
        elif previous_inside:
            result.append(_halfplane_line_intersection(previous, current, normal, offset))
    return result


def _halfplane_line_intersection(
    previous: Point2, current: Point2, normal: Point2, offset: float
) -> Point2:
    delta_y, delta_z = current[0] - previous[0], current[1] - previous[1]
    denominator = normal[0] * delta_y + normal[1] * delta_z
    fraction = (offset - (normal[0] * previous[0] + normal[1] * previous[1])) / denominator
    return (previous[0] + fraction * delta_y, previous[1] + fraction * delta_z)


def _integrate_target_volume_mm3(segments: Sequence[Segment]) -> float:
    """Trapezoidal-rule volume under the target's segment cross-sections."""
    if len(segments) < 2:
        return 0.0
    volume = 0.0
    for first, second in zip(segments, segments[1:]):
        length = second.x_model_mm - first.x_model_mm
        volume += 0.5 * (first.area_mm2() + second.area_mm2()) * length
    return volume


def _round_machine_value(value: float) -> float:
    return round(value, 6)
