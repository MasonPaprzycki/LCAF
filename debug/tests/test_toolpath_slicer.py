from __future__ import annotations

import json
import math
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
import unittest

import numpy as np

from lcaf.toolpathing.toolpath_slicer import (
    MachineLimits,
    SliceSettings,
    ToolpathPlanningError,
    ToolpathSlicer,
    TriangleMesh,
    load_mesh,
)
from lcaf.toolpathing.visualization import (
    axial_trim_allowance_mm,
    die_cap,
    find_sufficient_cycles,
    material_cross_section,
    material_state,
    radial_resample,
)
from lcaf.toolpathing.material import (
    MATERIALS,
    estimate_operation_force_kn,
    formability_response,
    plan_force_report,
    resolve_material_band,
)
from lcaf.simulation.surrogate.checkpoint import Checkpoint
from lcaf.simulation.surrogate.inference import SurrogateDomainWarning, SurrogateNetwork
from lcaf.simulation.surrogate.model import ArchitectureConfig, init_params
from lcaf.simulation.surrogate.preprocessing import NormalizationStats


def box_mesh(length: float = 20.0, width: float = 10.0, height: float = 10.0) -> TriangleMesh:
    x0, x1 = -length / 2, length / 2
    y0, y1 = -width / 2, width / 2
    z0, z1 = -height / 2, height / 2
    vertices = (
        (x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
        (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1),
    )
    faces = (
        (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
    )
    return TriangleMesh(tuple(tuple(vertices[index] for index in face) for face in faces))


def tapered_square_mesh(
    length: float = 20.0, near_half_side: float = 3.0, far_half_side: float = 6.0
) -> TriangleMesh:
    """A square prism that linearly tapers from ``near_half_side`` at x=-length/2
    to ``far_half_side`` at x=+length/2 -- used to check the numerical
    integration a segment's averaged cross-section is supposed to perform.
    """
    x0, x1 = -length / 2, length / 2
    near = near_half_side
    far = far_half_side
    vertices = (
        (x0, -near, -near), (x0, near, -near), (x0, near, near), (x0, -near, near),
        (x1, -far, -far), (x1, far, -far), (x1, far, far), (x1, -far, far),
    )
    faces = (
        (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
        (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
    )
    return TriangleMesh(tuple(tuple(vertices[index] for index in face) for face in faces))


def _translate_mesh(mesh: TriangleMesh, dx: float) -> TriangleMesh:
    return TriangleMesh(
        tuple(
            tuple((x + dx, y, z) for x, y, z in triangle)
            for triangle in mesh.triangles
        )
    )


def _polygon_area(points) -> float:
    total = 0.0
    count = len(points)
    for index in range(count):
        x1, y1 = points[index]
        x2, y2 = points[(index + 1) % count]
        total += x1 * y2 - x2 * y1
    return abs(total) / 2.0


def _build_test_network(seed: int = 0) -> SurrogateNetwork:
    """A small, fast, deterministic in-memory surrogate for exercising the
    toolpathing<->surrogate integration.

    Deliberately not loaded from the committed
    ``lcaf/simulation/surrogate/trained_network_parameters/dummy_smoke_test.npz``
    fixture, so these tests do not depend on that file's exact contents or
    its own random training run -- just a tiny (2 hidden layers x 8 units),
    freshly initialised network with identity normalisation, built the same
    way ``train.py`` would but skipping training entirely (nothing here
    needs the network to have learned anything, only to behave like a
    well-formed one: deterministic, finite-valued, and callable through the
    same ``SurrogateNetwork`` API a real checkpoint uses).
    """
    architecture = ArchitectureConfig(hidden_layers=2, hidden_width=8, activation="tanh")
    params = init_params(architecture, seed)
    stats = NormalizationStats(
        input_mean=np.zeros(6), input_std=np.ones(6),
        output_mean=np.zeros(3), output_std=np.ones(3),
    )
    checkpoint = Checkpoint(params=params, architecture=architecture, stats=stats, metadata={})
    return SurrogateNetwork(checkpoint=checkpoint, path=Path("<test-network>"))


class ToolpathSlicerTests(unittest.TestCase):
    def setUp(self) -> None:
        # Zero-based on every axis, matching the machine's own
        # [-retracted_distance, extended_distance] convention with
        # retracted_distance=0 for X/Y/Z (X=0 is the clamp -- see
        # ToolpathSlicer._x_reference / the module docstring) -- not the
        # old symmetric-about-zero range.
        self.continuous_limits = MachineLimits(
            x_min_mm=0,
            x_max_mm=200,
            y_min_mm=0,
            y_max_mm=100,
            z_retracted_mm=0,
            z_extended_mm=100,
        )

    # -- Pure strike-coordinate planning (unaffected by the deformation
    # preview -- see lcaf.simulation.surrogate for that) --------------------

    def test_square_bar_generates_four_strikes_per_segment_and_controller_jsonl(self) -> None:
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=2,
                strikes_per_segment=4,
                cycles=1,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()

        self.assertEqual(plan.rotations_deg, (0.0, 90.0, 180.0, 270.0))
        self.assertEqual(len(plan.sections), 2)
        self.assertEqual(len(plan.operations), 16)  # 2 segments * 4 strikes * 1 cycle * 2 passes
        first = plan.operations[0]
        self.assertEqual(set(first), {"step", "operation", "x", "y", "die_gap", "rotation", "target_temperature", "metadata"})
        self.assertEqual(first["operation"], "STRIKE")
        self.assertEqual(first["die_gap"], 12.5)
        self.assertEqual(first["metadata"]["target_support_mm"], 5.0)
        # die_width_mm/die_length_mm/upper_die_radius_mm are never left as
        # None: unset fields resolve to a physical default (here, the stock
        # radius / the striking segment's own width) so the preview
        # conserves volume by default instead of requiring the user to
        # configure die dimensions first.
        self.assertEqual(first["metadata"]["die_width_mm"], 10.0)
        self.assertEqual(first["metadata"]["die_length_mm"], 10.0)
        self.assertEqual(first["metadata"]["upper_die_radius_mm"], 10.0)
        self.assertEqual(first["metadata"]["die_shape"], "rectangular")
        self.assertEqual(first["metadata"]["segment_index"], 0)
        self.assertEqual(first["metadata"]["cycle_index"], 0)
        self.assertEqual(json.loads(plan.to_jsonl().splitlines()[0])["step"], 1)

    def test_strike_count_matches_segments_times_strikes_times_cycles(self) -> None:
        # Directly encodes the worked examples behind the feature: total
        # strikes = radial_segments * strikes_per_segment * cycles, when no
        # single strike needs splitting into multiple depth passes.
        cases = (
            (4, 4, 1, 16),
            (4, 4, 2, 32),
            (5, 4, 1, 20),
            (5, 4, 3, 60),
        )
        for radial_segments, strikes_per_segment, cycles, expected in cases:
            with self.subTest(radial_segments=radial_segments, strikes_per_segment=strikes_per_segment, cycles=cycles):
                plan = ToolpathSlicer(
                    box_mesh(),
                    SliceSettings(
                        stock_radius_mm=10,
                        radial_segments=radial_segments,
                        strikes_per_segment=strikes_per_segment,
                        cycles=cycles,
                        # A generous cap keeps every strike a single pass so
                        # the raw segments*strikes*cycles arithmetic holds
                        # exactly.
                        max_reduction_per_strike_mm=100,
                        die_contact_z_mm=10,
                    ),
                    self.continuous_limits,
                ).plan()
                self.assertEqual(len(plan.operations), expected)

    def test_default_strikes_per_segment_produces_four_orientations(self) -> None:
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(stock_radius_mm=10),
            MachineLimits(),
        ).plan()
        self.assertEqual(plan.rotations_deg, (0.0, 90.0, 180.0, 270.0))

    def test_stock_clamped_end_orients_positive_x_away_from_the_clamp(self) -> None:
        # Shift the mesh off-centre so the two ends are numerically
        # distinguishable regardless of the (symmetric) cross-section.
        mesh = _translate_mesh(box_mesh(), 15.0)  # spans mesh X in [5, 25]

        clamped_at_min = ToolpathSlicer(
            mesh, SliceSettings(stock_radius_mm=10, radial_segments=2, stock_clamped_end="min"), self.continuous_limits
        ).plan()
        clamped_at_max = ToolpathSlicer(
            mesh, SliceSettings(stock_radius_mm=10, radial_segments=2, stock_clamped_end="max"), self.continuous_limits
        ).plan()

        min_x = [section.x_model_mm for section in clamped_at_min.sections]
        max_x = [section.x_model_mm for section in clamped_at_max.sections]

        # Both orderings stay ascending in machine X, starting from 0 at
        # whichever end is actually clamped...
        self.assertEqual(min_x, sorted(min_x))
        self.assertEqual(max_x, sorted(max_x))
        self.assertGreaterEqual(min(min_x), 0.0)
        self.assertGreaterEqual(min(max_x), 0.0)
        # ...and since X is always measured as a distance from the clamp
        # (not from the mesh's centre), a uniform mesh like this one
        # produces the identical sequence of offsets-from-the-clamp
        # regardless of which end is actually clamped.
        self.assertEqual(max_x, min_x)

    def test_invalid_stock_clamped_end_is_rejected(self) -> None:
        with self.assertRaisesRegex(ToolpathPlanningError, "stock_clamped_end"):
            SliceSettings(stock_radius_mm=10, stock_clamped_end="middle").validate()

    def test_invalid_segment_strike_cycle_counts_are_rejected(self) -> None:
        with self.assertRaisesRegex(ToolpathPlanningError, "radial_segments"):
            SliceSettings(stock_radius_mm=10, radial_segments=0).validate()
        with self.assertRaisesRegex(ToolpathPlanningError, "strikes_per_segment"):
            SliceSettings(stock_radius_mm=10, strikes_per_segment=0).validate()
        with self.assertRaisesRegex(ToolpathPlanningError, "cycles"):
            SliceSettings(stock_radius_mm=10, cycles=0).validate()

    def test_recommended_stock_length_matches_target_volume(self) -> None:
        # box_mesh() is a 20 x 10 x 10 mm box: a constant 100 mm^2
        # cross-section throughout. Volume is trapezoidal-integrated between
        # segment *centres*, so with a finite number of segments it
        # systematically misses the half-segment overhang at each end --
        # here, radial_segments=10 spans centres 18 mm apart (out of the
        # true 20 mm length), so the expected volume is exactly
        # 100 mm^2 * 18 mm, not the true 2000 mm^3 -- this is an honest,
        # documented consequence of the coarser segment model, not
        # imprecision in the test.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(stock_radius_mm=10, radial_segments=10),
            self.continuous_limits,
        ).plan()
        expected_volume = 100.0 * (20.0 * 9 / 10)
        self.assertAlmostEqual(plan.target_volume_mm3, expected_volume, places=3)
        expected_length = expected_volume / (math.pi * 10.0**2)
        self.assertAlmostEqual(plan.recommended_stock_length_mm, expected_length, places=5)
        self.assertTrue(any("stock cylinder needs to be" in warning for warning in plan.warnings))

    def test_die_corner_radius_cannot_exceed_half_the_die_width(self) -> None:
        with self.assertRaisesRegex(ToolpathPlanningError, "cannot exceed half"):
            SliceSettings(stock_radius_mm=10, die_width_mm=4.0, die_corner_radius_mm=3.0).validate()

    def test_obj_and_ascii_stl_import(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            obj = root / "triangle.obj"
            obj.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8")
            stl = root / "triangle.stl"
            stl.write_text(
                "solid triangle\nfacet normal 0 0 1\nouter loop\n"
                "vertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\n"
                "endloop\nendfacet\nendsolid triangle\n",
                encoding="utf-8",
            )
            self.assertEqual(len(load_mesh(obj).triangles), 1)
            self.assertEqual(len(load_mesh(stl).triangles), 1)

    def test_binary_stl_import(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "triangle.stl"
            binary_stl = b"binary triangle".ljust(80, b"\0")
            binary_stl += struct.pack("<I", 1)
            binary_stl += struct.pack(
                "<12fH",
                0, 0, 1,  # normal
                0, 0, 0,
                1, 0, 0,
                0, 1, 0,
                0,
            )
            path.write_bytes(binary_stl)
            self.assertEqual(len(load_mesh(path).triangles), 1)

    def test_native_solidworks_has_actionable_error(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "sample.sldprt"
            path.write_bytes(b"placeholder")
            with self.assertRaisesRegex(ToolpathPlanningError, "Export this part"):
                load_mesh(path)

    def test_all_shipped_obj_examples_generate_a_demo_plan(self) -> None:
        examples = Path(__file__).resolve().parents[2] / "examples"
        for filename in (
            "square_bar.obj",
            "hex_bar.obj",
            "tapered_square_bar.obj",
            "tapered_hex_bar.obj",
        ):
            with self.subTest(filename=filename):
                plan = ToolpathSlicer(
                    load_mesh(examples / filename),
                    SliceSettings(
                        # 12 mm covers tapered_square_bar's widest diagonal
                        # corner (an 8 mm half-side square reaches ~11.3 mm),
                        # which a 10 mm stock radius would not contain even
                        # though its own 0/90/180/270 face supports all fit.
                        stock_radius_mm=12,
                        radial_segments=6,
                        strikes_per_segment=4,
                        max_reduction_per_strike_mm=2,
                        die_contact_z_mm=10,
                        longitudinal_axis="x",
                    ),
                    self.continuous_limits,
                ).plan()
                self.assertGreater(len(plan.operations), 0)
                self.assertEqual(plan.rotations_deg, (0.0, 90.0, 180.0, 270.0))

    def test_segment_support_is_the_numerical_average_across_its_span(self) -> None:
        # A linearly tapered square prism split into a single segment: the
        # segment's own support must sit close to the mean of the target's
        # support at the segment's two end positions, not either end alone
        # -- this is the numerical integration the feature is named for.
        mesh = tapered_square_mesh(length=20.0, near_half_side=3.0, far_half_side=6.0)
        plan = ToolpathSlicer(
            mesh,
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=1,
                strikes_per_segment=4,
                longitudinal_axis="x",
            ),
            self.continuous_limits,
        ).plan()
        self.assertEqual(len(plan.sections), 1)
        segment_support = plan.sections[0].support_mm(0.0)
        # The true mean of a linear taper's end supports (3 mm and 6 mm).
        expected_mean = (3.0 + 6.0) / 2.0
        self.assertAlmostEqual(segment_support, expected_mean, delta=0.05)
        # ...and it must differ meaningfully from either end alone.
        self.assertNotAlmostEqual(segment_support, 3.0, delta=0.5)
        self.assertNotAlmostEqual(segment_support, 6.0, delta=0.5)

    def test_die_cap_blends_flat_centre_into_corner_radius(self) -> None:
        # half_width=3, corner_radius=2: flat out to |t|<=1, then a circular
        # fillet blending up to support+radius exactly at the footprint edge.
        self.assertAlmostEqual(die_cap(0.0, 10.0, 3.0, 2.0), 10.0)
        self.assertAlmostEqual(die_cap(1.0, 10.0, 3.0, 2.0), 10.0)
        self.assertAlmostEqual(die_cap(2.0, 10.0, 3.0, 2.0), 12.0 - math.sqrt(3.0))
        self.assertAlmostEqual(die_cap(3.0, 10.0, 3.0, 2.0), 12.0)
        self.assertIsNone(die_cap(3.5, 10.0, 3.0, 2.0))

    def test_disc_contact_profile_matches_the_circular_footprint_formula(self) -> None:
        from lcaf.toolpathing.visualization import disc_contact_profile

        # At the disc's own axial centre its half-width is just its radius.
        centred = disc_contact_profile(5.0, 0.0, 10.0, samples=8)
        self.assertIsNotNone(centred)
        self.assertAlmostEqual(min(offset for offset, _ in centred), -5.0, places=6)
        self.assertAlmostEqual(max(offset for offset, _ in centred), 5.0, places=6)
        self.assertTrue(all(depth == 10.0 for _, depth in centred))

        # Off-centre, the effective half-width follows sqrt(R^2 - a^2)...
        offset_profile = disc_contact_profile(5.0, 3.0, 10.0, samples=8)
        self.assertIsNotNone(offset_profile)
        self.assertAlmostEqual(max(offset for offset, _ in offset_profile), 4.0, places=6)  # sqrt(25-9)=4

        # ...and the disc is simply not present beyond its own radius.
        self.assertIsNone(disc_contact_profile(5.0, 5.1, 10.0))
        self.assertIsNone(disc_contact_profile(None, 0.0, 10.0))

    def test_disc_rim_profile_matches_the_circular_footprint_when_unclipped(self) -> None:
        from lcaf.toolpathing.visualization import disc_rim_profile

        # Narrower than its own segment: the true footprint never reaches
        # the segment boundary, so the rim is simply the full circle.
        rim = disc_rim_profile(3.0, 10.0, sides=16)
        self.assertEqual(len(rim), 16)
        for axial, tangential in rim:
            self.assertAlmostEqual(axial**2 + tangential**2, 9.0, places=6)

    def test_disc_rim_profile_clips_axially_at_full_radius_tangentially(self) -> None:
        from lcaf.toolpathing.visualization import disc_rim_profile

        # Wider than its own segment: the rim must never overshoot the
        # segment axially, but must still reach the die's full radius
        # tangentially at the segment's own centre -- the same
        # sqrt(R**2 - a**2) footprint disc_contact_profile uses, expressed
        # as a polygon instead of a per-station formula. A 3D renderer that
        # instead uniformly shrinks the whole circle to fit the segment
        # (the historical bug) would understate this tangential reach.
        rim = disc_rim_profile(10.0, 2.5, sides=24)
        self.assertAlmostEqual(max(abs(axial) for axial, _ in rim), 2.5, places=6)
        self.assertAlmostEqual(max(abs(tangential) for _, tangential in rim), 10.0, places=6)
        for axial, _ in rim:
            self.assertLessEqual(abs(axial), 2.5 + 1e-6)

    def test_radial_resample_preserves_square_axis_radii(self) -> None:
        ring = radial_resample(((-5, -5), (5, -5), (5, 5), (-5, 5)), radial_segments=32)
        self.assertAlmostEqual(ring[0][0], 5.0, places=5)
        self.assertAlmostEqual(ring[8][1], 5.0, places=5)

    # -- Material/temperature: force estimate only, no longer the deformation
    # preview (a surrogate checkpoint is trained for one material/temperature
    # combination -- see lcaf.simulation.surrogate) -------------------------

    def test_unknown_material_is_rejected(self) -> None:
        with self.assertRaisesRegex(ToolpathPlanningError, "Unknown material"):
            SliceSettings(stock_radius_mm=10, material="unobtainium").validate()

    def test_material_is_recorded_on_every_operation_but_defaults_to_steel(self) -> None:
        plan = ToolpathSlicer(
            box_mesh(), SliceSettings(stock_radius_mm=10, radial_segments=2), self.continuous_limits
        ).plan()
        self.assertTrue(all(op["metadata"]["material"] == "steel" for op in plan.operations))

    def test_material_and_temperature_never_change_the_planned_strike_coordinates(self) -> None:
        # Material/temperature drive the separate force estimate only -- the
        # machine-facing coordinates a STRIKE operation actually commands
        # (x/y/die_gap/rotation) must be identical regardless of which
        # material or temperature is chosen.
        def coordinates(material: str, temperature_c: float) -> list[tuple[float, float, float, float]]:
            plan = ToolpathSlicer(
                box_mesh(),
                SliceSettings(
                    stock_radius_mm=10,
                    radial_segments=2,
                    strikes_per_segment=4,
                    max_reduction_per_strike_mm=3,
                    die_contact_z_mm=10,
                    material=material,
                    target_temperature_c=temperature_c,
                ),
                self.continuous_limits,
            ).plan()
            return [(op["x"], op["y"], op["die_gap"], op["rotation"]) for op in plan.operations]

        self.assertEqual(coordinates("steel", 0.0), coordinates("aluminum", 450.0))
        self.assertEqual(coordinates("steel", 0.0), coordinates("plasticine", 25.0))

    def test_formability_response_scales_reach_and_closure_between_documented_bounds(self) -> None:
        cold = formability_response(0.0)
        hot = formability_response(1.0)
        self.assertAlmostEqual(cold.reach_scale, 0.4, places=6)
        self.assertAlmostEqual(cold.closure_fraction, 0.15, places=6)
        self.assertAlmostEqual(hot.reach_scale, 1.0, places=6)
        self.assertAlmostEqual(hot.closure_fraction, 1.0, places=6)
        # Monotonic in between.
        middle = formability_response(0.5)
        self.assertLess(cold.reach_scale, middle.reach_scale)
        self.assertLess(middle.reach_scale, hot.reach_scale)

    def test_resolve_material_band_picks_the_band_containing_the_temperature(self) -> None:
        cold_steel = resolve_material_band("steel", 20.0)
        hot_steel = resolve_material_band("STEEL", 1100.0)  # case-insensitive
        self.assertLess(cold_steel.formability, hot_steel.formability)
        self.assertGreater(cold_steel.flow_stress_mpa, hot_steel.flow_stress_mpa)
        with self.assertRaises(ValueError):
            resolve_material_band("unobtainium", 20.0)

    def test_estimate_operation_force_increases_with_flow_stress_and_contact_area(self) -> None:
        def force_for(material: str, temperature_c: float, die_width_mm: float = 6.0) -> float:
            operation = {
                "target_temperature": temperature_c,
                "metadata": {
                    "stock_radius_mm": 10.0,
                    "target_support_mm": 5.0,
                    "die_width_mm": die_width_mm,
                    "upper_die_radius_mm": 6.0,
                    "die_length_mm": 6.0,
                    "material": material,
                },
            }
            return estimate_operation_force_kn(operation)

        # Same geometry: hot steel needs far more force than hot aluminum,
        # which needs more than warm plasticine -- following each material's
        # own flow stress ordering.
        self.assertGreater(force_for("steel", 1100.0), force_for("aluminum", 450.0))
        self.assertGreater(force_for("aluminum", 450.0), force_for("plasticine", 25.0))
        # A wider contact patch (more area) needs more force for the same material.
        self.assertGreater(force_for("steel", 1100.0, die_width_mm=12.0), force_for("steel", 1100.0, die_width_mm=6.0))

    def test_plan_force_report_covers_every_operation_with_positive_estimates(self) -> None:
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(stock_radius_mm=10, radial_segments=2, material="steel", target_temperature_c=1100.0),
            self.continuous_limits,
        ).plan()
        report = plan_force_report(plan.operations)
        self.assertEqual(len(report), len(plan.operations))
        self.assertTrue(all(row["estimated_force_kn"] > 0.0 for row in report))
        self.assertTrue(all(row["material"] == "steel" for row in report))

    def test_all_materials_constant_is_a_valid_choice_for_every_entry(self) -> None:
        for material in MATERIALS:
            with self.subTest(material=material):
                SliceSettings(stock_radius_mm=10, material=material).validate()

    # -- Surrogate-driven deformation preview (lcaf.simulation.surrogate) --
    #
    # material_state/material_cross_section/axial_trim_allowance_mm/
    # find_sufficient_cycles are now driven entirely by a trained
    # SurrogateNetwork (see the module docstring in visualization.py) -- a
    # from-scratch reimplementation of Jagtap, Reinisch & Bailly (ESAFORM
    # 2024), generalised to 3D. Unlike the geometric heuristic these
    # replaced, a network has no built-in guarantee of exact convergence to
    # an arbitrary target, exact area/volume conservation, or any particular
    # bulge shape -- these tests check what *is* guaranteed by construction
    # (determinism, the progress=0 identity, the strike's own zone of
    # influence, out-of-domain warnings) rather than re-deriving the old
    # heuristic's specific numeric behaviour.

    def test_material_state_at_progress_zero_is_the_pristine_stock(self) -> None:
        # Before a stroke has moved at all, nothing has physically happened
        # yet -- material_state must return the exact pristine cylinder,
        # not an evaluation of the network at a near-zero (and out-of-domain)
        # reduction. See inference.SurrogateNetwork.apply_strike's own
        # stroke_progress<=0 special case.
        network = _build_test_network()
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=2, strikes_per_segment=4,
                max_reduction_per_strike_mm=3, die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()
        state = material_state(plan, 0, 0.0, network, radial_segments=16)
        for ring in state:
            for y, z in ring:
                self.assertAlmostEqual(math.hypot(y, z), 10.0, places=6)

    def test_material_state_is_deterministic_and_finite(self) -> None:
        network = _build_test_network(seed=3)
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=3, strikes_per_segment=4,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()
        last_operation = len(plan.operations) - 1
        first = material_state(plan, last_operation, 1.0, network, radial_segments=24)
        second = material_state(plan, last_operation, 1.0, network, radial_segments=24)
        self.assertEqual(first, second)
        for ring in first:
            for y, z in ring:
                self.assertTrue(math.isfinite(y) and math.isfinite(z))

    def test_material_state_leaves_stations_outside_the_strikes_reach_untouched(self) -> None:
        # A tightly bitten strike (die_length_mm small relative to how far
        # apart the stations are) must leave a distant station exactly at
        # the pristine stock radius -- geometry.affected_station_indices'
        # own zone-of-influence window, a structural guarantee independent
        # of whatever the network itself predicts near the strike.
        network = _build_test_network()
        plan = ToolpathSlicer(
            box_mesh(length=200.0),
            SliceSettings(
                stock_radius_mm=10, radial_segments=20, strikes_per_segment=4,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10, die_length_mm=1.0,
            ),
            self.continuous_limits,
        ).plan()
        state = material_state(plan, 0, 1.0, network, radial_segments=16)
        far_station = len(plan.sections) - 1  # opposite end of a 200 mm bar from the first strike
        for y, z in state[far_station]:
            self.assertAlmostEqual(math.hypot(y, z), 10.0, places=6)

    def test_surrogate_domain_warning_fires_outside_the_trained_variable_space(self) -> None:
        network = _build_test_network()
        # A large stock radius relative to the target's own support forces a
        # single-pass reduction whose eps_h (and xb) fall well outside the
        # paper's own trained variable space (Table 1).
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=30, radial_segments=2, strikes_per_segment=4,
                max_reduction_per_strike_mm=30, die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()
        with self.assertWarns(SurrogateDomainWarning):
            material_state(plan, 0, 1.0, network, radial_segments=16)

    def test_axial_trim_allowance_is_exactly_zero_before_any_strike(self) -> None:
        # Before the very first strike has moved at all, the billet is still
        # exactly the pristine stock cylinder -- there must be no phantom
        # "deficit" from comparing a polygon-sampled ring against the exact
        # circle formula, or from trapezoidally integrating between station
        # centres versus the sections' own true edge-to-edge span.
        network = _build_test_network()
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=4, strikes_per_segment=4,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()
        self.assertEqual(axial_trim_allowance_mm(plan, 0, 0.0, network), 0.0)

    def test_axial_trim_allowance_is_continuous_across_operation_boundaries(self) -> None:
        # The end of one operation's own stroke (progress=1.0) and the start
        # of the next (progress=0.0) describe the exact same material state
        # -- the next operation has not moved at all yet -- so these must
        # match exactly, not merely approximately.
        network = _build_test_network()
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=4, strikes_per_segment=4, cycles=2,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()
        for index in (0, len(plan.operations) // 2, len(plan.operations) - 2):
            end_of_this = axial_trim_allowance_mm(plan, index, 1.0, network)
            start_of_next = axial_trim_allowance_mm(plan, index + 1, 0.0, network)
            self.assertEqual(end_of_this, start_of_next)

    def test_axial_trim_allowance_matches_the_volume_balance_formula(self) -> None:
        # axial_trim_allowance_mm is defined as a volume balance over
        # whatever material_state actually returns -- this must hold at any
        # (operation_index, progress), not only once the shape happens to
        # have converged onto some target.
        network = _build_test_network()
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=4, strikes_per_segment=4,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()
        operation_index = len(plan.operations) // 2
        progress = 0.6
        radial_segments = 48

        state = material_state(plan, operation_index, progress, network, radial_segments=radial_segments)
        station_x = [section.x_model_mm for section in plan.sections]
        span_mm = station_x[-1] - station_x[0]
        pristine_area_mm2 = 0.5 * radial_segments * 10.0**2 * math.sin(2.0 * math.pi / radial_segments)
        original_stock_volume_mm3 = pristine_area_mm2 * span_mm
        current_volume_mm3 = sum(
            0.5 * (_polygon_area(state[index]) + _polygon_area(state[index + 1]))
            * (station_x[index + 1] - station_x[index])
            for index in range(len(state) - 1)
        )
        deficit_mm3 = original_stock_volume_mm3 - current_volume_mm3
        free_end_area_mm2 = _polygon_area(state[-1])
        expected = deficit_mm3 / free_end_area_mm2 if deficit_mm3 > 0.0 and free_end_area_mm2 > 1e-9 else 0.0

        actual = axial_trim_allowance_mm(plan, operation_index, progress, network, radial_segments=radial_segments)
        self.assertAlmostEqual(actual, max(0.0, expected), places=6)

    def test_axial_trim_allowance_is_zero_with_only_a_single_section(self) -> None:
        # With radial_segments=1 there is only one station -- no adjacent
        # pair to trapezoidally integrate a volume between -- so this must
        # return 0 rather than raising or dividing by zero, the same
        # convention _integrate_target_volume_mm3 already uses.
        network = _build_test_network()
        plan = ToolpathSlicer(
            box_mesh(), SliceSettings(stock_radius_mm=10, radial_segments=1), self.continuous_limits
        ).plan()
        self.assertEqual(axial_trim_allowance_mm(plan, 0, 0.0, network), 0.0)

    def test_find_sufficient_cycles_terminates_and_returns_a_usable_plan(self) -> None:
        network = _build_test_network()
        plan = find_sufficient_cycles(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=2, strikes_per_segment=4,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10,
            ),
            self.continuous_limits, network, max_cycles=3, tolerance_mm=0.5,
        )
        self.assertTrue(plan.operations)
        used_cycles = plan.operations[-1]["metadata"]["cycle_index"] + 1
        self.assertGreaterEqual(used_cycles, 1)
        self.assertLessEqual(used_cycles, 3)

    def test_find_sufficient_cycles_gives_up_and_warns_at_an_unreachable_tolerance(self) -> None:
        # A trained network predicts what a real strike actually does -- it
        # has no guarantee of ever converging exactly onto an arbitrary
        # target the way the old geometric heuristic did. An effectively
        # zero tolerance must make find_sufficient_cycles give up and warn
        # within a bounded max_cycles, not loop forever or silently return a
        # falsely "converged" plan.
        network = _build_test_network()
        plan = find_sufficient_cycles(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=2, strikes_per_segment=4,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10,
            ),
            self.continuous_limits, network, max_cycles=2, tolerance_mm=1e-9,
        )
        self.assertTrue(any("max_cycles" in warning for warning in plan.warnings))


class MachineLimitsFromLcafConfigTests(unittest.TestCase):
    """
    MachineLimits.from_lcaf_config() reads the same retracted_distance/
    extended_distance each JointConfiguration carries for LinuxCNC's own
    INI generation (lcaf.utils.joint_configuration). Either may be left
    null in axis.json to genuinely disable that end's software soft
    limit (this project's A axis does this -- A isn't read here at all,
    since only X/Y/Z bound the toolpath planner); this test pins down that
    a null X/Y/Z distance is treated as unbounded here too, with a logged
    warning, rather than crashing with a bare TypeError from float(None).
    """

    def _write_axis_json(self, path: Path, z_extended_distance) -> None:
        def joint_entry(joint, axis, extended_distance):
            return {
                "joint": joint,
                "axis": axis,
                "motor_steps_per_revolution": 200,
                "microsteps": 16,
                "travel_per_motor_rev": 0.2,
                "max_velocity": 1.0,
                "max_acceleration": 5.0,
                "mesa_stepgen": f"hm2_7i76e.0.stepgen.0{joint}",
                "has_limit_switches": False,
                "retracted_distance": 0.0,
                "extended_distance": extended_distance,
            }

        path.write_text(
            json.dumps([
                joint_entry(0, "X", 4.0),
                joint_entry(1, "Y", 2.0),
                joint_entry(2, "Z", z_extended_distance),
            ]),
            encoding="utf-8",
        )

    def test_null_extended_distance_is_treated_as_unbounded_and_warns(self):
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "axis.json"
            self._write_axis_json(path, z_extended_distance=None)

            with self.assertLogs("lcaf.toolpathing.toolpath_slicer", level="WARNING"):
                limits = MachineLimits.from_lcaf_config(path)

            self.assertEqual(limits.z_extended_mm, math.inf)
            self.assertEqual(limits.z_retracted_mm, 0.0)
            self.assertEqual(limits.x_max_mm, 4.0 * 25.4)
            self.assertEqual(limits.y_max_mm, 2.0 * 25.4)

    def test_configured_extended_distance_is_unaffected(self):
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "axis.json"
            self._write_axis_json(path, z_extended_distance=6.0)

            limits = MachineLimits.from_lcaf_config(path)

            self.assertEqual(limits.z_extended_mm, 6.0 * 25.4)


if __name__ == "__main__":
    unittest.main()
