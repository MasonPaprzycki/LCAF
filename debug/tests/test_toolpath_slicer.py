from __future__ import annotations

import json
import math
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
import unittest

from lcaf.toolpathing.toolpath_slicer import (
    MachineLimits,
    SliceSettings,
    ToolpathPlanningError,
    ToolpathSlicer,
    TriangleMesh,
    load_mesh,
)
from lcaf.toolpathing.visualization import (
    anvil_side_support,
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


class ToolpathSlicerTests(unittest.TestCase):
    def setUp(self) -> None:
        # Zero-based on every axis, matching the machine's own [0, max_travel]
        # convention (X=0 is the clamp -- see ToolpathSlicer._x_reference /
        # the module docstring) -- not the old symmetric-about-zero range.
        self.continuous_limits = MachineLimits(
            x_min_mm=0,
            x_max_mm=200,
            y_min_mm=0,
            y_max_mm=100,
            z_retracted_mm=0,
            z_extended_mm=100,
        )

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

    def test_final_deformed_geometry_matches_the_target_exactly(self) -> None:
        # The core correctness guarantee: whatever the anvil is configured
        # to do along the way, once every rotation has struck, every
        # segment's remaining material must land on its own (numerically
        # integrated) target boundary -- not merely conserve volume, not
        # bulge off in some other direction. Checked across every shipped
        # example (constant and tapered profiles) and both an unconfigured
        # and a finite, narrow anvil.
        examples = Path(__file__).resolve().parents[2] / "examples"
        cases = (
            ("square_bar.obj", 4, 10),
            ("hex_bar.obj", 6, 10),
            ("tapered_square_bar.obj", 4, 12),
            ("tapered_hex_bar.obj", 6, 10),
        )
        for filename, strikes_per_segment, stock_radius in cases:
            for die_width in (None, 4.0):
                with self.subTest(filename=filename, die_width=die_width):
                    plan = ToolpathSlicer(
                        load_mesh(examples / filename),
                        SliceSettings(
                            stock_radius_mm=stock_radius,
                            radial_segments=5,
                            strikes_per_segment=strikes_per_segment,
                            max_reduction_per_strike_mm=2,
                            die_contact_z_mm=10,
                            longitudinal_axis="x",
                            die_width_mm=die_width,
                        ),
                        self.continuous_limits,
                    ).plan()
                    last_operation = len(plan.operations) - 1
                    for segment_index, section in enumerate(plan.sections):
                        target_ring = radial_resample(section.polygon_yz_mm, radial_segments=48)
                        final_ring = material_cross_section(
                            plan, segment_index, last_operation, 1.0, radial_segments=48
                        )
                        for (ty, tz), (fy, fz) in zip(target_ring, final_ring):
                            self.assertAlmostEqual(fy, ty, delta=1e-3)
                            self.assertAlmostEqual(fz, tz, delta=1e-3)

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

    def test_material_preview_clips_the_red_stock_as_strikes_complete(self) -> None:
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=2,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()

        before = material_cross_section(plan, station_index=0, operation_index=0, operation_progress=0.0)
        after_first_strike = material_cross_section(plan, station_index=0, operation_index=0, operation_progress=1.0)
        after_all_strikes = material_cross_section(plan, station_index=0, operation_index=len(plan.operations) - 1, operation_progress=1.0)

        self.assertAlmostEqual(max(z for _, z in before), 10.0, places=5)
        self.assertAlmostEqual(max(z for _, z in after_first_strike), 7.5, places=5)
        self.assertAlmostEqual(max(abs(y) for y, _ in after_all_strikes), 5.0, places=5)
        self.assertAlmostEqual(max(abs(z) for _, z in after_all_strikes), 5.0, places=5)

    def test_default_die_settings_conserve_cross_sectional_area_by_bulging(self) -> None:
        # Leaving every die field unset must not mean "unconstrained, no
        # conservation" -- it means a sensible physical default that already
        # conserves volume by bulging, without requiring the user to
        # configure die dimensions first. This is the single most direct
        # regression test for that default.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=5,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()

        before = material_cross_section(plan, station_index=0, operation_index=0, operation_progress=0.0)
        after = material_cross_section(plan, station_index=0, operation_index=0, operation_progress=1.0)
        # Area is conserved (a ceiling, not an exact invariant -- see the
        # target-boundary cap discussed below), not simply deleted.
        self.assertLessEqual(_polygon_area(after), _polygon_area(before) + 1e-6)
        # Material displaced by the (default, narrower-than-full) anvil
        # bulges out beside it, beyond the original cylinder radius -- the
        # direct, robust signature of real conservation rather than deletion.
        self.assertGreater(max(math.hypot(y, z) for y, z in after), 10.0)

    def test_finite_die_width_conserves_cross_sectional_area_by_bulging(self) -> None:
        # die_length_mm is pinned well below the segment's own width so this
        # isolates purely tangential (within-segment) bulge behaviour; axial
        # propagation across segments is covered separately below.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=2,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
                die_width_mm=6,
                die_length_mm=1,
            ),
            self.continuous_limits,
        ).plan()

        before = material_cross_section(plan, station_index=0, operation_index=0, operation_progress=0.0)
        after = material_cross_section(plan, station_index=0, operation_index=0, operation_progress=1.0)

        # The striking die (+Z, sample index 12 of 48) is always a large
        # flat surface: it flattens the whole struck side to the requested
        # support depth, with no bulge of its own.
        y_at_die, z_at_die = after[12]
        self.assertAlmostEqual(y_at_die, 0.0, places=5)
        self.assertAlmostEqual(z_at_die, 7.5, places=2)

        # The fixed anvil (-Z, sample index 36) holds material at the
        # original stock surface directly beneath its centre...
        y_at_anvil, z_at_anvil = after[36]
        self.assertAlmostEqual(y_at_anvil, 0.0, places=5)
        self.assertAlmostEqual(z_at_anvil, -10.0, places=5)

        # ...while material displaced by its narrow footprint bulges out
        # beside it, beyond the original cylinder radius.
        self.assertGreater(max(math.hypot(y, z) for y, z in after), 10.0)

        # Bulge growth is capped at the target's own boundary in each exact
        # direction (a strike whose own footprint does not cover some other
        # direction must never push material past what the target ultimately
        # needs there, since no later strike would bring an overshoot back
        # down). For this square target that ceiling bites well before the
        # full original area could be restored, so conservation here is a
        # ceiling, not an exact invariant: area never exceeds what it started
        # with, but may legitimately fall short of it.
        self.assertLessEqual(_polygon_area(after), _polygon_area(before) + 1e-6)

    def test_die_length_propagates_anvil_contact_and_bulge_across_segments(self) -> None:
        # A die_length_mm spanning more than one segment's own width must
        # hold every segment it reaches against the fixed anvil (not just
        # the one segment the operation is nominally "at"), while volume
        # conservation bulges the immediately adjacent segments and leaves
        # segments outside its reach completely untouched.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=5,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
                die_width_mm=6,
                die_length_mm=9,
            ),
            self.continuous_limits,
        ).plan()

        segment_x = [section.x_model_mm for section in plan.sections]
        # X=0 is the clamp (the mesh's own min-X bound here, the default
        # stock_clamped_end), not the mesh's centre -- see
        # ToolpathSlicer._x_reference / the module docstring.
        self.assertEqual(segment_x, [2.0, 6.0, 10.0, 14.0, 18.0])
        before = [material_cross_section(plan, s, 0, 0.0) for s in range(len(plan.sections))]
        after = [material_cross_section(plan, s, 0, 1.0) for s in range(len(plan.sections))]

        def integrated_volume(rings):
            total = 0.0
            for index in range(len(rings) - 1):
                total += (
                    0.5
                    * (_polygon_area(rings[index]) + _polygon_area(rings[index + 1]))
                    * (segment_x[index + 1] - segment_x[index])
                )
            return total

        # See the note in the tangential-only test above: the target-boundary
        # clamp makes conservation a ceiling here, not an exact invariant.
        self.assertLessEqual(integrated_volume(after), integrated_volume(before) + 1e-6)

        # The striking die's *rigid* footprint only ever touches the one
        # segment it targets: segment 1's pole is never pulled all the way
        # down to segment 0's own commanded depth (7.5) the way segment 0's
        # own pole is...
        self.assertAlmostEqual(after[0][12][1], 7.5, places=2)
        self.assertNotAlmostEqual(after[1][12][1], 7.5, places=1)
        # ...though its bulge *margin* (unavoidably, symmetric to the
        # anvil's own margin below) can still reach the immediately
        # adjacent segment, nudging it partway toward -- never all the way
        # to, and never past -- its own eventual target (5.0, the same
        # everywhere on this constant-profile box) after just this one
        # strike: the relaxation is a bounded, gradual blend each strike,
        # never an instant snap to the final value (see the module
        # docstring in visualization.py on why that continuity matters).
        self.assertLess(after[1][12][1], 10.0)
        self.assertGreater(after[1][12][1], 5.0)

        # ...but the fixed anvil, rigidly spanning +/-4.5 mm from segment 0
        # (at x=-8), rigidly holds segment 1 (at x=-4, 4 mm away)...
        self.assertAlmostEqual(after[0][36][1], -10.0, places=5)
        self.assertAlmostEqual(after[1][36][1], -10.0, places=5)
        # ...and the disc's own bulge margin reaches forward (+X, toward the
        # free end) much further than it reaches backward, decaying
        # smoothly with distance rather than cutting off sharply -- material
        # genuinely propagates toward the free end across the whole
        # remaining bar (see test_bulge_propagates_further_toward_the_free_end_than_the_clamp),
        # so even segment 3 (12 mm away) shows a small, fading nudge, not
        # zero.
        self.assertLess(after[3][12][1], 10.0)
        self.assertGreater(after[3][12][1], 9.5)
        self.assertLess(after[3][36][1], -9.0)
        self.assertGreater(after[3][36][1], -10.0)

        # Segment 4 (16 mm away) shows a smaller nudge still than segment 3
        # -- the forward bulge margin decays smoothly with distance rather
        # than cutting off sharply, so reach fades but is never structurally
        # capped at some fixed number of segments the way the old, purely
        # symmetric margin was.
        self.assertLess(after[4][12][1], 10.0)
        self.assertGreater(after[4][12][1], after[3][12][1])
        self.assertLess(after[4][36][1], -9.5)
        self.assertLess(after[4][36][1], after[3][36][1])

    def test_die_cap_blends_flat_centre_into_corner_radius(self) -> None:
        # half_width=3, corner_radius=2: flat out to |t|<=1, then a circular
        # fillet blending up to support+radius exactly at the footprint edge.
        self.assertAlmostEqual(die_cap(0.0, 10.0, 3.0, 2.0), 10.0)
        self.assertAlmostEqual(die_cap(1.0, 10.0, 3.0, 2.0), 10.0)
        self.assertAlmostEqual(die_cap(2.0, 10.0, 3.0, 2.0), 12.0 - math.sqrt(3.0))
        self.assertAlmostEqual(die_cap(3.0, 10.0, 3.0, 2.0), 12.0)
        self.assertIsNone(die_cap(3.5, 10.0, 3.0, 2.0))

    def test_radiused_anvil_lets_a_prior_bulge_settle_higher_than_a_sharp_anvil(self) -> None:
        # Strike the +Y face first (rotation=90) with a narrow, unconfigured
        # anvil so material bulges out near +Z; then strike +Z (rotation=0)
        # with an anvil covering that bulge, sharp vs. radiused, and compare.
        def bulge_then_anvil(corner_radius_mm: float) -> float:
            plan = ToolpathSlicer(
                box_mesh(),
                SliceSettings(
                    stock_radius_mm=10,
                    radial_segments=2,
                    strikes_per_segment=4,
                    max_reduction_per_strike_mm=3,
                    die_contact_z_mm=10,
                    die_width_mm=6,
                    die_corner_radius_mm=corner_radius_mm,
                ),
                self.continuous_limits,
            ).plan()
            rotation_zero_ops = [
                index for index, op in enumerate(plan.operations) if float(op["rotation"]) == 0.0
            ]
            after = material_cross_section(
                plan, station_index=0, operation_index=rotation_zero_ops[-1], operation_progress=1.0, radial_segments=64
            )
            return max(z for _, z in after)

        sharp = bulge_then_anvil(0.0)
        radiused = bulge_then_anvil(2.0)
        self.assertGreaterEqual(radiused, sharp)

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
        # sqrt(R**2 - a**2) footprint disc_contact_profile/_apply_strike_3d
        # already use, expressed as a polygon instead of a per-station
        # formula. A 3D renderer that instead uniformly shrinks the whole
        # circle to fit the segment (the historical bug) would understate
        # this tangential reach.
        rim = disc_rim_profile(10.0, 2.5, sides=24)
        self.assertAlmostEqual(max(abs(axial) for axial, _ in rim), 2.5, places=6)
        self.assertAlmostEqual(max(abs(tangential) for _, tangential in rim), 10.0, places=6)
        for axial, _ in rim:
            self.assertLessEqual(abs(axial), 2.5 + 1e-6)

    def test_both_footprints_conserve_and_bulge_simultaneously(self) -> None:
        # A small anvil AND a small upper die radius active on the same
        # strike: both hemispheres should show real bulge growth beyond the
        # original stock radius, not just one of them.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=2,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
                die_width_mm=6,
                die_length_mm=1,
                upper_die_radius_mm=3,
            ),
            self.continuous_limits,
        ).plan()

        before = material_cross_section(plan, station_index=0, operation_index=0, operation_progress=0.0)
        after = material_cross_section(plan, station_index=0, operation_index=0, operation_progress=1.0)

        # The ceiling still holds with both footprints active at once.
        self.assertLessEqual(_polygon_area(after), _polygon_area(before) + 1e-6)

        # The striking die's own rigid centre (sample index 12) still lands
        # exactly on the commanded support...
        y_at_die, z_at_die = after[12]
        self.assertAlmostEqual(y_at_die, 0.0, places=5)
        self.assertAlmostEqual(z_at_die, 7.5, places=2)

        # ...but with a 3 mm radius it no longer covers the whole striking
        # hemisphere the way an unconstrained infinite plane would: a
        # sample well outside the disc's own footprint (index 8) does not
        # simply land on the same flat support depth as the pole.
        outside_disc_footprint = math.hypot(*after[8])
        self.assertNotAlmostEqual(outside_disc_footprint, 7.5, places=1)

        # The anvil side still bulges beyond the original stock radius too.
        self.assertGreater(max(math.hypot(y, z) for y, z in after), 10.0)

    def test_undersized_upper_die_radius_leaves_the_target_unstruck(self) -> None:
        # An upper_die_radius_mm smaller than stock_radius_mm is an honest
        # physical trade-off: like a real round punch smaller than a face,
        # the final struck geometry can legitimately fall short of the
        # target in directions the small disc's own footprint never reaches.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=2,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=2,
                die_contact_z_mm=10,
                upper_die_radius_mm=1.0,
                # The anvil is pinned narrow too: its own default width
                # (stock_radius_mm, generous) would otherwise fill in the
                # gap the small striking disc leaves, across the other
                # rotations' own wide anvils -- this isolates the striking
                # disc's own limitation instead of being masked by it.
                die_width_mm=1.0,
            ),
            self.continuous_limits,
        ).plan()

        last_operation = len(plan.operations) - 1
        final_ring = material_cross_section(plan, 0, last_operation, 1.0, radial_segments=48)
        target_ring = radial_resample(plan.sections[0].polygon_yz_mm, radial_segments=48)

        # A too-small disc leaves material protruding past the target
        # boundary in directions none of the four rotations' narrow discs
        # (nor their limited bulge margins) ever reach -- unlike the exact-
        # convergence guarantee that holds at the (adequately large) default
        # radius -- see test_final_deformed_geometry_matches_the_target_exactly.
        overshoots = [
            math.hypot(*final) - math.hypot(*target)
            for target, final in zip(target_ring, final_ring)
        ]
        self.assertGreater(max(overshoots), 0.5)

    def test_upper_die_radius_rigid_reach_is_cut_off_by_its_own_segment(self) -> None:
        # A large upper_die_radius_mm must never *rigidly* reach into a
        # neighbouring segment just because its radius is wide -- widening
        # the striking disc's radius controls its *tangential* reach, never
        # how far along X its rigid footprint spreads beyond the one
        # segment it targets. (Its bulge *margin* can still nudge an
        # adjacent segment toward -- never past -- that segment's own
        # eventual target, symmetric to the anvil's own margin; see
        # test_die_length_propagates_anvil_contact_and_bulge_across_segments.)
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=5,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
                upper_die_radius_mm=1000.0,
            ),
            self.continuous_limits,
        ).plan()
        after = [material_cross_section(plan, s, 0, 1.0) for s in range(len(plan.sections))]
        self.assertAlmostEqual(after[0][12][1], 7.5, places=2)
        # Segment 1 is never pulled down to segment 0's own commanded depth
        # (rigid spillover) -- at most it is nudged toward its own target.
        self.assertNotAlmostEqual(after[1][12][1], 7.5, places=1)
        # Segments further away show a progressively smaller nudge -- the
        # bulge margin reaches forward (+X) across the whole remaining bar,
        # decaying smoothly with distance rather than cutting off sharply
        # (see test_bulge_propagates_further_toward_the_free_end_than_the_clamp) --
        # never zero, but never the rigid spillover ruled out above either.
        self.assertLess(after[3][12][1], 10.0)
        self.assertLess(after[4][12][1], 10.0)
        self.assertGreater(after[4][12][1], after[3][12][1])

    def test_no_levitation_across_a_full_rotation_sweep(self) -> None:
        # Reproduces, then closes, the reported "upper die floats clear of
        # the billet" bug: replay every operation of a full 0/90/180/270
        # sweep and confirm the anvil's actual current boundary never
        # exceeds the pristine stock radius, and is strictly less than it
        # at some point once an earlier rotation has already reduced that
        # direction -- i.e. the anvil really would have to track inward,
        # not stay fixed at the pristine radius.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=2,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()

        stock_radius = 10.0
        saw_reduced_anvil_side = False
        for operation_index, operation in enumerate(plan.operations):
            pre_stroke_state = material_state(plan, operation_index, 0.0)
            ring = pre_stroke_state[int(operation["metadata"]["segment_index"])]
            support = anvil_side_support(ring, float(operation["rotation"]))
            self.assertLessEqual(support, stock_radius + 1e-6)
            if support < stock_radius - 1e-6:
                saw_reduced_anvil_side = True
        self.assertTrue(saw_reduced_anvil_side)

    def test_final_geometry_matches_target_with_both_footprints_configured(self) -> None:
        # test_final_deformed_geometry_matches_the_target_exactly already
        # covers an unconfigured/finite anvil in isolation; this extends
        # that same convergence guarantee to a plan where *both* the anvil
        # and the (adequately large) upper die are explicitly configured
        # together, re-validating the target-boundary ceiling now that the
        # striking side has its own footprint too, not just the anvil.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                radial_segments=2,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
                die_width_mm=4,
                die_length_mm=4,
                upper_die_radius_mm=10,
            ),
            self.continuous_limits,
        ).plan()

        last_operation = len(plan.operations) - 1
        for station_index, section in enumerate(plan.sections):
            target_ring = radial_resample(section.polygon_yz_mm, radial_segments=48)
            final_ring = material_cross_section(plan, station_index, last_operation, 1.0, radial_segments=48)
            for (ty, tz), (fy, fz) in zip(target_ring, final_ring):
                self.assertAlmostEqual(fy, ty, delta=1e-3)
                self.assertAlmostEqual(fz, tz, delta=1e-3)

    def test_cycles_repeat_the_full_sweep_and_further_true_up_the_shape(self) -> None:
        # Every operation just replays, in order, against one running
        # material state -- a second cycle is nothing more than the same
        # segment x strike sweep appended again, and it must further true
        # up (never worsen) how closely the final shape matches target.
        def worst_deviation(cycles: int) -> float:
            plan = ToolpathSlicer(
                box_mesh(),
                SliceSettings(
                    stock_radius_mm=10,
                    radial_segments=2,
                    strikes_per_segment=4,
                    cycles=cycles,
                    max_reduction_per_strike_mm=3,
                    die_contact_z_mm=10,
                    die_width_mm=6,
                    upper_die_radius_mm=3,
                ),
                self.continuous_limits,
            ).plan()
            last_operation = len(plan.operations) - 1
            worst = 0.0
            for segment_index, section in enumerate(plan.sections):
                target_ring = radial_resample(section.polygon_yz_mm, radial_segments=48)
                final_ring = material_cross_section(plan, segment_index, last_operation, 1.0, radial_segments=48)
                for (ty, tz), (fy, fz) in zip(target_ring, final_ring):
                    worst = max(worst, math.hypot(fy - ty, fz - tz))
            return worst

        one_cycle = worst_deviation(1)
        two_cycles = worst_deviation(2)
        self.assertLessEqual(two_cycles, one_cycle + 1e-9)

    def test_find_sufficient_cycles_converges_and_reports_it_in_metadata(self) -> None:
        mesh = load_mesh(Path(__file__).resolve().parents[2] / "examples" / "square_bar.obj")
        settings = SliceSettings(
            stock_radius_mm=12,
            radial_segments=4,
            strikes_per_segment=4,
            max_reduction_per_strike_mm=2,
            die_contact_z_mm=10,
            longitudinal_axis="x",
        )
        plan = find_sufficient_cycles(mesh, settings, self.continuous_limits, max_cycles=10, tolerance_mm=0.5)
        self.assertTrue(plan.operations)
        used_cycles = plan.operations[-1]["metadata"]["cycle_index"] + 1
        self.assertGreaterEqual(used_cycles, 1)
        self.assertLessEqual(used_cycles, 10)
        self.assertFalse(any("max_cycles" in warning for warning in plan.warnings))

    def test_find_sufficient_cycles_warns_when_it_cannot_converge(self) -> None:
        mesh = load_mesh(Path(__file__).resolve().parents[2] / "examples" / "square_bar.obj")
        # An upper die far too small to ever flatten the whole face, capped
        # at very few cycles: this must not loop forever, and must report
        # that it gave up rather than silently returning a bad plan.
        settings = SliceSettings(
            stock_radius_mm=12,
            radial_segments=4,
            strikes_per_segment=4,
            max_reduction_per_strike_mm=2,
            die_contact_z_mm=10,
            longitudinal_axis="x",
            upper_die_radius_mm=0.5,
            # Also pin the anvil narrow -- its generous default width would
            # otherwise fill the gap in on its own across the four rotations,
            # masking the striking disc's limitation (see the isolation note
            # in test_undersized_upper_die_radius_leaves_the_target_unstruck).
            die_width_mm=0.5,
        )
        plan = find_sufficient_cycles(mesh, settings, self.continuous_limits, max_cycles=2, tolerance_mm=0.01)
        self.assertTrue(any("max_cycles" in warning for warning in plan.warnings))

    def test_radial_resample_preserves_square_axis_radii(self) -> None:
        ring = radial_resample(((-5, -5), (5, -5), (5, 5), (-5, 5)), radial_segments=32)
        self.assertAlmostEqual(ring[0][0], 5.0, places=5)
        self.assertAlmostEqual(ring[8][1], 5.0, places=5)

    # -- Material/temperature-driven deformation mechanics ----------------

    def test_unknown_material_is_rejected(self) -> None:
        with self.assertRaisesRegex(ToolpathPlanningError, "Unknown material"):
            SliceSettings(stock_radius_mm=10, material="unobtainium").validate()

    def test_material_is_recorded_on_every_operation_but_defaults_to_steel(self) -> None:
        plan = ToolpathSlicer(
            box_mesh(), SliceSettings(stock_radius_mm=10, radial_segments=2), self.continuous_limits
        ).plan()
        self.assertTrue(all(op["metadata"]["material"] == "steel" for op in plan.operations))

    def test_material_and_temperature_never_change_the_planned_strike_coordinates(self) -> None:
        # Material/temperature drive the deformation *preview* and the
        # separate force estimate only -- the machine-facing coordinates a
        # STRIKE operation actually commands (x/y/die_gap/rotation) must be
        # identical regardless of which material or temperature is chosen.
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

    def test_footprint_bias_favors_the_direction_a_die_is_narrower_in(self) -> None:
        from lcaf.toolpathing.visualization import _footprint_bias

        # A square footprint is unbiased.
        axial_bias, tangential_bias = _footprint_bias(10.0, 10.0)
        self.assertAlmostEqual(axial_bias, 0.5, places=6)
        self.assertAlmostEqual(tangential_bias, 0.5, places=6)

        # A long, narrow footprint (large axial extent, small tangential
        # extent) is biased toward spreading tangentially -- the direction
        # it is smaller, and therefore less confining, in.
        axial_bias, tangential_bias = _footprint_bias(18.0, 2.0)
        self.assertLess(axial_bias, 0.5)
        self.assertGreater(tangential_bias, 0.5)

        # ...and the reverse for a short, wide footprint.
        axial_bias, tangential_bias = _footprint_bias(2.0, 18.0)
        self.assertGreater(axial_bias, 0.5)
        self.assertLess(tangential_bias, 0.5)

    def test_anvil_aspect_ratio_biases_which_direction_bulge_reaches_further(self) -> None:
        # Same axial length (die_length_mm=9) on both anvils, only the width
        # differs: a narrow anvil is biased to spread tangentially instead of
        # reaching further axially, a wide one the other way. Checked
        # *backward* (toward -X, the clamp) specifically, since forward
        # (+X) reach is now dominated by the much larger forward-propagation
        # multiplier regardless of aspect ratio (see
        # test_bulge_propagates_further_toward_the_free_end_than_the_clamp) --
        # this isolates the pure aspect-ratio effect from that separate
        # mechanic by calling _apply_strike_3d directly against a
        # synthetic, evenly-spaced station grid with a uniform target,
        # striking the middle station and reading the immediately-backward
        # one, rather than relying on a full plan's own segment-major strike
        # ordering (which always finishes every backward segment's own
        # dedicated strikes before an even earlier operation could isolate
        # this).
        from lcaf.toolpathing.visualization import _apply_strike_3d

        station_x = (0.0, 4.0, 8.0, 12.0, 16.0)
        angles = tuple(2.0 * math.pi * index / 48 for index in range(48))
        stock_radius_mm = 10.0
        target_radii_grid = tuple(tuple(5.0 for _ in angles) for _ in station_x)

        def backward_station_anvil_pole(die_width_mm: float) -> float:
            radii_grid = [[stock_radius_mm] * 48 for _ in station_x]
            new_grid = _apply_strike_3d(
                radii_grid, angles, station_x, station_x[2],
                axial_half_length=4.5, disc_axial_half_length=2.0,
                rotation_rad=0.0, support_mm=5.0, stock_radius_mm=stock_radius_mm,
                width_mm=die_width_mm, corner_radius_mm=0.0,
                target_radii_grid=target_radii_grid, upper_die_radius_mm=10.0,
                reach_scale=1.0, closure_fraction=1.0, stroke_progress=1.0,
            )
            return new_grid[0][36]  # station 0, 8 mm backward of the strike; anvil pole.

        self.assertAlmostEqual(backward_station_anvil_pole(2.0), 10.0, places=5)
        wide_result = backward_station_anvil_pole(20.0)
        self.assertLess(wide_result, 10.0)

    def test_bulge_propagates_further_toward_the_free_end_than_the_clamp(self) -> None:
        # Real open-die forging pushes displaced material toward the free
        # (unclamped) end, not toward the clamp, which cannot move. A
        # strike's own bulge margin must therefore reach much further
        # forward (+X) than backward (-X) at the same distance, and must
        # still show a meaningful nudge forward at a distance where the
        # backward direction has already faded to nothing.
        from lcaf.toolpathing.visualization import _apply_strike_3d

        station_x = (0.0, 4.0, 8.0, 12.0, 16.0)
        angles = tuple(2.0 * math.pi * index / 48 for index in range(48))
        stock_radius_mm = 10.0
        target_radii_grid = tuple(tuple(5.0 for _ in angles) for _ in station_x)
        radii_grid = [[stock_radius_mm] * 48 for _ in station_x]

        # A square (isotropic) footprint, so _footprint_bias contributes no
        # asymmetry of its own -- any forward/backward difference here comes
        # purely from the forward-propagation multiplier.
        new_grid = _apply_strike_3d(
            radii_grid, angles, station_x, station_x[2],
            axial_half_length=2.0, disc_axial_half_length=2.0,
            rotation_rad=0.0, support_mm=5.0, stock_radius_mm=stock_radius_mm,
            width_mm=4.0, corner_radius_mm=0.0,
            target_radii_grid=target_radii_grid, upper_die_radius_mm=10.0,
            reach_scale=1.0, closure_fraction=0.5, stroke_progress=1.0,
        )

        # At equal distance (4 mm), the forward neighbour is nudged further
        # toward target (a smaller value, since target < pristine here) than
        # the backward neighbour.
        self.assertLess(new_grid[3][36], new_grid[1][36])
        # At 8 mm, the backward direction has already faded to exactly
        # untouched, while the forward direction still shows a clear nudge.
        self.assertAlmostEqual(new_grid[0][36], stock_radius_mm, places=5)
        self.assertLess(new_grid[4][36], stock_radius_mm - 0.1)

    def test_cold_stiff_material_needs_more_cycles_to_converge_than_hot_material(self) -> None:
        # With a die too narrow for cold steel's (formability ~0.05) reduced
        # bulge reach to bridge the target's corners, find_sufficient_cycles
        # must honestly give up (warn) within a handful of cycles rather than
        # silently returning an unconverged plan -- while the identical
        # geometry with hot steel (formability ~0.85) converges immediately.
        # This is the intended, felt effect of "temperature" on the preview:
        # the *shape* still converges given enough cycles for workable
        # material/temperature combinations, but genuinely does not for an
        # impractical one, exactly like real forging practice.
        def converges(temperature_c: float) -> tuple[int, bool]:
            plan = find_sufficient_cycles(
                box_mesh(),
                SliceSettings(
                    stock_radius_mm=10,
                    radial_segments=2,
                    strikes_per_segment=4,
                    max_reduction_per_strike_mm=3,
                    die_contact_z_mm=10,
                    die_width_mm=3.0,
                    upper_die_radius_mm=3.0,
                    material="steel",
                    target_temperature_c=temperature_c,
                ),
                self.continuous_limits,
                max_cycles=5,
                tolerance_mm=0.5,
            )
            used_cycles = plan.operations[-1]["metadata"]["cycle_index"] + 1
            gave_up = any("max_cycles" in warning for warning in plan.warnings)
            return used_cycles, gave_up

        cold_cycles, cold_gave_up = converges(0.0)
        hot_cycles, hot_gave_up = converges(1100.0)
        self.assertTrue(cold_gave_up)
        self.assertFalse(hot_gave_up)
        self.assertGreater(cold_cycles, hot_cycles)

    def test_bulge_never_jumps_discontinuously_between_neighbouring_rays_or_stations(self) -> None:
        # Regression test for a reported bug: an earlier version of the bulge
        # mechanic solved one aggregate volume-conserving "growth" per strike
        # and clamped each touched ray to min(row * (1 + weight * growth),
        # target) -- for the overwhelmingly common case of a reducing target
        # (target < the ray's current, pristine radius), *any* nonzero
        # weight caused an instant, full snap straight to the target, while
        # the immediately neighbouring (zero-weight) ray stayed completely
        # untouched: a hard step, rendered as a "triangulated" spike rather
        # than a smooth bulge. The relaxation must instead move only
        # partway, continuously, so no two angularly- or axially-adjacent
        # samples should ever differ by anywhere near the full stock-radius-
        # to-target distance after a single strike.
        stock_radius = 10.0
        max_reasonable_jump_mm = 0.3 * stock_radius

        # Angular (tangential) continuity: a narrow die on a reducing target,
        # checked after just its own first strike.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=stock_radius,
                radial_segments=2,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
                die_width_mm=6,
                die_length_mm=1,
            ),
            self.continuous_limits,
        ).plan()
        for operation_index in range(3):
            ring = material_cross_section(
                plan, station_index=0, operation_index=operation_index, operation_progress=1.0, radial_segments=48
            )
            radii = [math.hypot(y, z) for y, z in ring]
            max_angular_jump = max(
                abs(radii[i] - radii[(i + 1) % len(radii)]) for i in range(len(radii))
            )
            self.assertLess(
                max_angular_jump, max_reasonable_jump_mm,
                f"operation_index={operation_index} has a discontinuous angular jump of {max_angular_jump:.3f} mm",
            )

        # Axial continuity: a wide anvil spanning several segments, checked
        # after just its own first strike.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=stock_radius,
                radial_segments=5,
                strikes_per_segment=4,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
                die_width_mm=6,
                die_length_mm=9,
            ),
            self.continuous_limits,
        ).plan()
        state = material_state(plan, operation_index=0, operation_progress=1.0, radial_segments=48)
        for angle_index in range(48):
            radii = [math.hypot(*station[angle_index]) for station in state]
            max_axial_jump = max(abs(radii[i + 1] - radii[i]) for i in range(len(radii) - 1))
            self.assertLess(
                max_axial_jump, max_reasonable_jump_mm,
                f"angle_index={angle_index} has a discontinuous axial jump of {max_axial_jump:.3f} mm",
            )

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

    # -- Global axial volume conservation (trim allowance) ----------------

    def test_axial_trim_allowance_is_exactly_zero_before_any_strike(self) -> None:
        # Before the very first strike has moved at all, the billet is still
        # exactly the pristine stock cylinder -- there must be no phantom
        # "deficit" from comparing a polygon-sampled ring against the exact
        # circle formula, or from trapezoidally integrating between station
        # centres versus the sections' own true edge-to-edge span.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=4, strikes_per_segment=4,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()
        self.assertEqual(axial_trim_allowance_mm(plan, 0, 0.0), 0.0)

    def test_axial_trim_allowance_grows_monotonically_and_is_continuous(self) -> None:
        # Forging never creates or destroys material: as strikes progress
        # and reduce cross-sections the local bulge cannot fully reabsorb,
        # the volume deficit -- and so the trim allowance -- can only grow
        # or hold steady, never shrink, and must never jump discontinuously
        # within an operation's own stroke or across the boundary between
        # two operations.
        plan = ToolpathSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10, radial_segments=4, strikes_per_segment=4, cycles=2,
                max_reduction_per_strike_mm=2, die_contact_z_mm=10,
                material="steel", target_temperature_c=1100.0,
            ),
            self.continuous_limits,
        ).plan()
        values = [axial_trim_allowance_mm(plan, index, 1.0) for index in range(len(plan.operations))]
        for previous, current in zip(values, values[1:]):
            self.assertGreaterEqual(current, previous - 1e-9)

        # Continuity within one operation's own stroke...
        sample_index = len(plan.operations) // 2
        progress_values = [axial_trim_allowance_mm(plan, sample_index, p / 20) for p in range(21)]
        for previous, current in zip(progress_values, progress_values[1:]):
            self.assertLess(abs(current - previous), 1.0)

        # ...and across the boundary from one operation's end to the next's start.
        for index in (0, len(plan.operations) // 2, len(plan.operations) - 2):
            end_of_this = axial_trim_allowance_mm(plan, index, 1.0)
            start_of_next = axial_trim_allowance_mm(plan, index + 1, 0.0)
            self.assertAlmostEqual(end_of_this, start_of_next, places=6)

    def test_axial_trim_allowance_matches_the_volume_balance_at_full_convergence(self) -> None:
        # Once the shape has fully converged onto the target, the remaining
        # trim allowance must equal exactly (original stock volume - target
        # volume) / the free end's own target cross-sectional area -- the
        # documented volume balance this function is built from, not merely
        # "some positive number."
        settings = SliceSettings(
            stock_radius_mm=10, radial_segments=4, strikes_per_segment=4,
            max_reduction_per_strike_mm=2, die_contact_z_mm=10,
            material="steel", target_temperature_c=1100.0,
        )
        plan = find_sufficient_cycles(box_mesh(), settings, self.continuous_limits, max_cycles=30, tolerance_mm=0.1)
        last_operation = len(plan.operations) - 1
        final_trim_allowance = axial_trim_allowance_mm(plan, last_operation, 1.0)

        station_x = [section.x_model_mm for section in plan.sections]
        span_mm = station_x[-1] - station_x[0]
        angle_count = 48
        pristine_area_mm2 = 0.5 * angle_count * 10.0**2 * math.sin(2.0 * math.pi / angle_count)
        original_stock_volume_mm3 = pristine_area_mm2 * span_mm
        free_end_target_area_mm2 = plan.sections[-1].area_mm2()
        expected = (original_stock_volume_mm3 - plan.target_volume_mm3) / free_end_target_area_mm2

        self.assertAlmostEqual(final_trim_allowance, expected, places=3)

    def test_axial_trim_allowance_is_zero_with_only_a_single_section(self) -> None:
        # With radial_segments=1 there is only one station -- no adjacent
        # pair to trapezoidally integrate a volume between -- so this must
        # return 0 rather than raising or dividing by zero, the same
        # convention _integrate_target_volume_mm3 already uses.
        plan = ToolpathSlicer(
            box_mesh(), SliceSettings(stock_radius_mm=10, radial_segments=1), self.continuous_limits
        ).plan()
        self.assertEqual(axial_trim_allowance_mm(plan, 0, 0.0), 0.0)


if __name__ == "__main__":
    unittest.main()
