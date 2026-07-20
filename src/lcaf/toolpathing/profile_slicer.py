"""A conservative, computational-geometry planner for simple open-die forging.

The planner intentionally models geometry, not metal flow.  A target mesh is
sampled along its longitudinal axis.  At every station, its radial support
values define the four (or more) planar die strikes needed to cut a cylindrical
stock envelope down to the target's convex cross-section.

Coordinates are millimetres at the planner boundary.  OBJ files have no unit
metadata, so callers must provide ``scale_mm_per_unit`` when necessary.
"""

from __future__ import annotations

import json
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Sequence


Point3 = tuple[float, float, float]
Point2 = tuple[float, float]
Triangle = tuple[Point3, Point3, Point3]


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
    """Machine-coordinate limits used to reject unsafe generated operations."""

    x_min_mm: float = -50.0
    x_max_mm: float = 50.0
    y_min_mm: float = -50.0
    y_max_mm: float = 50.0
    z_retracted_mm: float = 0.0
    z_extended_mm: float = 100.0
    rotation_min_deg: float = -90.0
    rotation_max_deg: float = 90.0

    @classmethod
    def from_lcaf_config(cls, filename: str | Path) -> "MachineLimits":
        """Read the current ``configs/forge_parameters.json`` shape."""
        path = Path(filename)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise ToolpathPlanningError(
                f"Could not read machine limits from {path}: {error}"
            ) from error

        try:
            return cls(
                x_min_mm=float(data["fully_retracted_x_position_mm"]),
                x_max_mm=float(data["fully_extended_x_position_mm"]),
                y_min_mm=float(data["fully_retracted_y_position_mm"]),
                y_max_mm=float(data["fully_extended_y_position_mm"]),
                z_retracted_mm=float(data["fully_retracted_z_position_mm"]),
                z_extended_mm=float(data["fully_extended_z_position_mm"]),
                rotation_min_deg=float(data["rotation_left_limit_degrees"]),
                rotation_max_deg=float(data["rotation_right_limit_degrees"]),
            )
        except (KeyError, TypeError, ValueError) as error:
            raise ToolpathPlanningError(
                f"Machine-limit file {path} is missing a numeric LCAF limit: {error}"
            ) from error


@dataclass(frozen=True)
class SliceSettings:
    """Inputs to the simplified profile slicer.

    ``die_contact_z_mm`` is a calibration value: the machine Z coordinate at
    which a die first contacts the unformed cylindrical stock.  The generated
    Z command is ``contact + stock_radius - requested_support``.  It must be
    established by setup/proving, not inferred from CAD.
    """

    stock_radius_mm: float
    axial_resolution_mm: float = 5.0
    rotation_increment_deg: float = 90.0
    max_reduction_per_strike_mm: float = 2.0
    die_contact_z_mm: float = 0.0
    x_offset_mm: float = 0.0
    y_position_mm: float = 0.0
    target_temperature_c: float = 0.0
    scale_mm_per_unit: float = 1.0
    longitudinal_axis: str = "auto"
    include_opposing_rotations: bool = True
    allow_out_of_limit_rotations: bool = False

    def validate(self) -> None:
        if self.stock_radius_mm <= 0:
            raise ToolpathPlanningError("stock_radius_mm must be positive.")
        if self.axial_resolution_mm <= 0:
            raise ToolpathPlanningError("axial_resolution_mm must be positive.")
        if self.max_reduction_per_strike_mm <= 0:
            raise ToolpathPlanningError(
                "max_reduction_per_strike_mm must be positive."
            )
        if not 0 < self.rotation_increment_deg <= 180:
            raise ToolpathPlanningError(
                "rotation_increment_deg must be in the range (0, 180]."
            )
        rotations = 360.0 / self.rotation_increment_deg
        if not math.isclose(rotations, round(rotations), abs_tol=1e-8):
            raise ToolpathPlanningError(
                "rotation_increment_deg must divide 360 exactly for a closed profile."
            )
        if self.longitudinal_axis.lower() not in {"auto", "x", "y", "z"}:
            raise ToolpathPlanningError(
                "longitudinal_axis must be one of auto, x, y, or z."
            )
        if self.scale_mm_per_unit <= 0:
            raise ToolpathPlanningError("scale_mm_per_unit must be positive.")


@dataclass(frozen=True)
class CrossSection:
    """A convex cross-section envelope in the local radial (y, z) plane."""

    x_model_mm: float
    polygon_yz_mm: tuple[Point2, ...]

    def support_mm(self, rotation_deg: float) -> float:
        """Return the positive die-direction support after an X-axis rotation."""
        angle = math.radians(rotation_deg)
        sine, cosine = math.sin(angle), math.cos(angle)
        return max(y * sine + z * cosine for y, z in self.polygon_yz_mm)


@dataclass(frozen=True)
class ToolpathPlan:
    """The controller-compatible JSONL operations plus preview geometry."""

    operations: tuple[dict, ...]
    sections: tuple[CrossSection, ...]
    rotations_deg: tuple[float, ...]
    warnings: tuple[str, ...] = field(default_factory=tuple)

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


class ProfileSlicer:
    """Build controller JSONL from a simple convex radial target profile."""

    def __init__(self, mesh: TriangleMesh, settings: SliceSettings, limits: MachineLimits):
        settings.validate()
        self.mesh = mesh.scaled(settings.scale_mm_per_unit)
        self.settings = settings
        self.limits = limits
        self._axis = self._resolve_longitudinal_axis()
        self._origin = self._mesh_center()

    def plan(self) -> ToolpathPlan:
        rotations = self._rotations()
        self._validate_fixed_machine_positions(rotations)
        sections = tuple(self._section_at(position) for position in self._station_positions())
        operations: list[dict] = []
        warnings = [
            "This is a geometric die-envelope plan. It does not model material flow, "
            "springback, temperature-dependent deformation, flash, or die compliance.",
            "Concave and hollow cross-sections are reduced to their convex outer support envelope.",
            "Prove die_contact_z_mm, tooling clearance, and all machine limits before execution.",
        ]
        bypassed_rotations = [
            angle
            for angle in rotations
            if not self.limits.rotation_min_deg <= angle <= self.limits.rotation_max_deg
        ]
        rotation_limit_override = (
            self.settings.allow_out_of_limit_rotations and bool(bypassed_rotations)
        )
        if rotation_limit_override:
            if bypassed_rotations:
                warnings.append(
                    "A-axis limit validation was explicitly overridden for rotations "
                    + ", ".join(f"{angle:g}°" for angle in bypassed_rotations)
                    + ". Confirm continuous/indexed rotation and LinuxCNC limits before execution."
                )

        # Sweep each orientation along X.  Reverse alternate sweeps to avoid an
        # unnecessary full-axis rapid between orientation passes.
        step = 1
        for rotation_index, rotation in enumerate(rotations):
            sweep = sections if rotation_index % 2 == 0 else tuple(reversed(sections))
            for section in sweep:
                support = section.support_mm(rotation)
                if support < -1e-6:
                    raise ToolpathPlanningError(
                        "The target is not centered inside the stock at "
                        f"X={section.x_model_mm:.3f} mm; positive die support is negative."
                    )
                if support > self.settings.stock_radius_mm + 1e-6:
                    raise ToolpathPlanningError(
                        "Target extends outside the cylindrical stock at "
                        f"X={section.x_model_mm:.3f} mm, rotation={rotation:.1f}°: "
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
                        x=section.x_model_mm + self.settings.x_offset_mm,
                        y=self.settings.y_position_mm,
                        z=die_z,
                        rotation=rotation,
                    )
                    operations.append(
                        {
                            "step": step,
                            "operation": "STRIKE",
                            "x": _round_machine_value(section.x_model_mm + self.settings.x_offset_mm),
                            "y": _round_machine_value(self.settings.y_position_mm),
                            "die_gap": _round_machine_value(die_z),
                            "rotation": _round_machine_value(rotation),
                            "target_temperature": _round_machine_value(
                                self.settings.target_temperature_c
                            ),
                            "metadata": {
                                "generator": "lcaf.profile_slicer",
                                "model_x_mm": _round_machine_value(section.x_model_mm),
                                "station_index": section_index(sections, section),
                                "rotation_deg": _round_machine_value(rotation),
                                "strike_pass": pass_index,
                                "strike_pass_count": strikes,
                                "target_support_mm": _round_machine_value(support),
                                "stock_radius_mm": _round_machine_value(
                                    self.settings.stock_radius_mm
                                ),
                                "radial_reduction_mm": _round_machine_value(applied_reduction),
                                "geometry_model": "convex_support_envelope",
                                "rotation_limit_override": rotation_limit_override,
                            },
                        }
                    )
                    step += 1

        return ToolpathPlan(tuple(operations), sections, rotations, tuple(warnings))

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

    def _station_positions(self) -> tuple[float, ...]:
        lower, upper = self.mesh.bounds()
        start, end = lower[self._axis], upper[self._axis]
        length = end - start
        if length <= 1e-9:
            raise ToolpathPlanningError("The selected longitudinal axis has zero length.")

        station_count = max(2, math.ceil(length / self.settings.axial_resolution_mm) + 1)
        return tuple(
            start + length * index / (station_count - 1)
            for index in range(station_count)
        )

    def _section_at(self, model_position: float) -> CrossSection:
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
                f"{model_position - self._origin[self._axis]:.3f} mm. "
                "Use a watertight triangle mesh and a valid longitudinal axis."
            )

        return CrossSection(
            x_model_mm=model_position - self._origin[self._axis],
            polygon_yz_mm=tuple(
                (
                    point[0] - self._origin[first_radial],
                    point[1] - self._origin[second_radial],
                )
                for point in hull
            ),
        )

    def _rotations(self) -> tuple[float, ...]:
        count = round(360.0 / self.settings.rotation_increment_deg)
        if self.settings.include_opposing_rotations:
            return tuple(index * self.settings.rotation_increment_deg for index in range(count))
        # A 180-degree range only makes sense for symmetric two-sided tooling;
        # keep this explicit rather than hiding an orientation omission.
        return tuple(index * self.settings.rotation_increment_deg for index in range((count + 1) // 2))

    def _validate_fixed_machine_positions(self, rotations: Sequence[float]) -> None:
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
        if not self.settings.allow_out_of_limit_rotations:
            invalid = [
                angle
                for angle in rotations
                if not self.limits.rotation_min_deg <= angle <= self.limits.rotation_max_deg
            ]
            if invalid:
                formatted = ", ".join(f"{angle:g}°" for angle in invalid)
                raise ToolpathPlanningError(
                    "The requested full rotary sweep needs A-axis targets "
                    f"{formatted}, outside configured limits "
                    f"[{self.limits.rotation_min_deg:g}°, {self.limits.rotation_max_deg:g}°]. "
                    "The current MotionCoordinator commands absolute A positions; add "
                    "a safe re-clamp/indexing workflow or explicitly enable the planner "
                    "override only after verifying continuous rotary hardware."
                )

    def _validate_operation(self, *, x: float, y: float, z: float, rotation: float) -> None:
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
        if (
            not self.settings.allow_out_of_limit_rotations
            and not self.limits.rotation_min_deg <= rotation <= self.limits.rotation_max_deg
        ):
            raise ToolpathPlanningError(
                f"Generated A={rotation:.3f}° is outside configured rotation limits."
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


def section_index(sections: Sequence[CrossSection], target: CrossSection) -> int:
    """Keep preview metadata stable even while alternate rotation sweeps reverse."""
    return sections.index(target)


def _round_machine_value(value: float) -> float:
    return round(value, 6)
