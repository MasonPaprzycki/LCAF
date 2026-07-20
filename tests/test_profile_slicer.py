from __future__ import annotations

import json
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
import unittest

from lcaf.toolpathing.profile_slicer import (
    MachineLimits,
    ProfileSlicer,
    SliceSettings,
    ToolpathPlanningError,
    TriangleMesh,
    load_mesh,
)
from lcaf.toolpathing.visualization import material_cross_section, radial_resample


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


class ProfileSlicerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.continuous_limits = MachineLimits(
            x_min_mm=-50,
            x_max_mm=50,
            y_min_mm=-50,
            y_max_mm=50,
            z_retracted_mm=0,
            z_extended_mm=100,
            rotation_min_deg=-360,
            rotation_max_deg=360,
        )

    def test_square_bar_generates_four_face_strikes_and_controller_jsonl(self) -> None:
        plan = ProfileSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                axial_resolution_mm=10,
                rotation_increment_deg=90,
                max_reduction_per_strike_mm=3,
                die_contact_z_mm=10,
            ),
            self.continuous_limits,
        ).plan()

        self.assertEqual(plan.rotations_deg, (0.0, 90.0, 180.0, 270.0))
        self.assertEqual(len(plan.sections), 3)
        self.assertEqual(len(plan.operations), 24)  # 3 stations * 4 faces * 2 passes
        first = plan.operations[0]
        self.assertEqual(set(first), {"step", "operation", "x", "y", "die_gap", "rotation", "target_temperature", "metadata"})
        self.assertEqual(first["operation"], "STRIKE")
        self.assertEqual(first["die_gap"], 12.5)
        self.assertEqual(first["metadata"]["target_support_mm"], 5.0)
        self.assertFalse(first["metadata"]["rotation_limit_override"])
        self.assertEqual(json.loads(plan.to_jsonl().splitlines()[0])["step"], 1)

    def test_default_lcaf_rotation_limit_blocks_unexecutable_full_sweep(self) -> None:
        with self.assertRaisesRegex(ToolpathPlanningError, "absolute A positions"):
            ProfileSlicer(
                box_mesh(),
                SliceSettings(stock_radius_mm=10),
                MachineLimits(),
            ).plan()

    def test_explicit_rotary_override_is_recorded_in_jsonl_metadata(self) -> None:
        plan = ProfileSlicer(
            box_mesh(),
            SliceSettings(stock_radius_mm=10, allow_out_of_limit_rotations=True),
            MachineLimits(),
        ).plan()
        self.assertTrue(plan.operations[0]["metadata"]["rotation_limit_override"])
        self.assertTrue(any("overridden" in warning for warning in plan.warnings))

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

    def test_all_shipped_obj_examples_generate_a_four_face_demo_plan(self) -> None:
        examples = Path(__file__).resolve().parents[1] / "examples"
        for filename in (
            "square_bar.obj",
            "hex_bar.obj",
            "tapered_square_bar.obj",
            "tapered_hex_bar.obj",
        ):
            with self.subTest(filename=filename):
                plan = ProfileSlicer(
                    load_mesh(examples / filename),
                    SliceSettings(
                        stock_radius_mm=10,
                        axial_resolution_mm=5,
                        rotation_increment_deg=90,
                        max_reduction_per_strike_mm=2,
                        die_contact_z_mm=10,
                        longitudinal_axis="x",
                    ),
                    self.continuous_limits,
                ).plan()
                self.assertGreater(len(plan.operations), 0)
                self.assertEqual(plan.rotations_deg, (0.0, 90.0, 180.0, 270.0))

    def test_material_preview_clips_the_red_stock_as_strikes_complete(self) -> None:
        plan = ProfileSlicer(
            box_mesh(),
            SliceSettings(
                stock_radius_mm=10,
                axial_resolution_mm=10,
                rotation_increment_deg=90,
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

    def test_radial_resample_preserves_square_axis_radii(self) -> None:
        ring = radial_resample(((-5, -5), (5, -5), (5, 5), (-5, 5)), radial_segments=32)
        self.assertAlmostEqual(ring[0][0], 5.0, places=5)
        self.assertAlmostEqual(ring[8][1], 5.0, places=5)


if __name__ == "__main__":
    unittest.main()
