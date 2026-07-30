from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

from lcaf.control.forge_brain import ForgeBrain
from lcaf.control.motion_coordinator import MotionCoordinator
from lcaf.utils.joint_configuration import load_machine_configuration

REPO_ROOT = Path(__file__).resolve().parents[2]


class ToolpathRetractedDistanceOffsetTests(unittest.TestCase):
    """
    A JSONL toolpath's own coordinate frame treats 0 as this project's
    parked/standoff position -- but LinuxCNC's own machine coordinate 0 is
    the physical negative limit switch, and MIN_LIMIT is retracted_distance,
    not 0 (see docs/hardware_setup.md sections 7 and 10). Without
    correcting for that, a toolpath's own zero coordinate commands the
    joint below its own soft limit floor -- exactly what real-hardware
    testing hit. ForgeBrain.load_jsonl() now offsets every parsed
    x/y/die_gap by that joint's own parked reference (retracted_distance,
    or extended_distance for a flip_retraction joint -- this project's Y,
    since it parks at the far end of its travel instead of the near one),
    converted to machine units/mm via _retracted_offset_mm(), so the
    toolpath file itself never needs editing.
    """

    def setUp(self):
        self.fake_linuxcnc = sys.modules["linuxcnc"]

        self.machine_config = load_machine_configuration(REPO_ROOT / "configs" / "machine.json")
        self._tmpdir = tempfile.TemporaryDirectory()
        self.motion = MotionCoordinator(
            machine_config=self.machine_config,
            generated_config_dir=self._tmpdir.name,
        )
        self.brain = ForgeBrain(motion=self.motion, planner=None)

    def tearDown(self):
        self._tmpdir.cleanup()

    def axis_offset_mm(self, axis_name: str) -> float:
        # Mirrors ForgeBrain._retracted_offset_mm(): the joint's parked
        # reference is extended_distance for a flip_retraction joint (this
        # project's Y), retracted_distance for every other joint.
        axial = self.motion.axes[axis_name].axial_interface
        joint = axial.joint
        reference = joint.extended_distance if joint.flip_retraction else joint.retracted_distance
        return axial.to_machine_units(reference)

    def axis_bounds_mm(self, axis_name: str):
        """
        [retracted_distance, extended_distance] converted to machine units
        (mm) -- the real range a joint may be commanded within, straight
        from JointConfiguration rather than the live (only-set-after-homing)
        LinuxCNCAxialInterface.min_limit/max_limit, since these tests never
        home the fake axes.
        """
        axial = self.motion.axes[axis_name].axial_interface
        joint = axial.joint
        minimum = (
            axial.to_machine_units(joint.retracted_distance)
            if joint.retracted_distance is not None
            else float("-inf")
        )
        maximum = (
            axial.to_machine_units(joint.extended_distance)
            if joint.extended_distance is not None
            else float("inf")
        )
        return minimum, maximum

    def write_toolpath(self, operations) -> Path:
        path = Path(self._tmpdir.name) / "test_toolpath.jsonl"
        with open(path, "w") as file:
            for operation in operations:
                file.write(json.dumps(operation) + "\n")
        return path

    def test_zero_coordinate_offsets_to_retracted_distance(self):
        path = self.write_toolpath([
            {"step": 1, "operation": "strike", "x": 0.0, "y": 0.0, "die_gap": 0.0, "rotation": 0.0},
        ])

        self.brain.load_jsonl(str(path))

        self.assertFalse(self.brain.state.fault_active, self.brain.state.fault_message)
        operation = self.brain.queue.operations[0]

        self.assertAlmostEqual(operation.x, self.axis_offset_mm("x"))
        self.assertAlmostEqual(operation.y, self.axis_offset_mm("y"))
        self.assertAlmostEqual(operation.die_gap, self.axis_offset_mm("z"))

    def test_rotation_is_never_offset(self):
        # A has no retracted_distance (genuinely unbounded -- see
        # JointConfiguration.retracted_distance) -- rotation must pass
        # through completely unchanged.
        path = self.write_toolpath([
            {"step": 1, "operation": "strike", "x": 0.0, "y": 0.0, "die_gap": 0.0, "rotation": 135.0},
        ])

        self.brain.load_jsonl(str(path))

        self.assertEqual(self.brain.queue.operations[0].rotation, 135.0)

    def test_offset_preserves_relative_spacing_between_operations(self):
        path = self.write_toolpath([
            {"step": 1, "operation": "strike", "x": 1.0, "y": 0.0, "die_gap": 2.0, "rotation": 0.0},
            {"step": 2, "operation": "strike", "x": 4.0, "y": 0.0, "die_gap": 5.0, "rotation": 90.0},
        ])

        self.brain.load_jsonl(str(path))

        first, second = self.brain.queue.operations
        self.assertAlmostEqual(second.x - first.x, 3.0)
        self.assertAlmostEqual(second.die_gap - first.die_gap, 3.0)

    def test_toolpath_file_on_disk_is_never_modified(self):
        path = self.write_toolpath([
            {"step": 1, "operation": "strike", "x": 0.0, "y": 0.0, "die_gap": 0.0, "rotation": 0.0},
        ])
        original_text = path.read_text()

        self.brain.load_jsonl(str(path))

        self.assertEqual(path.read_text(), original_text)

    def test_every_operation_in_real_example_toolpaths_lands_within_soft_limits(self):
        """
        Every operation in this project's real example toolpaths
        (toolpaths/*.jsonl) must land within [retracted_distance,
        extended_distance] once offset -- the same range LinuxCNC's own
        MIN_LIMIT/MAX_LIMIT enforce -- confirming the offset is what makes
        an ordinary generated toolpath executable at all, not just
        numerically present. A toolpath commanding a joint outside that
        range regardless of this offset is a real out-of-range toolpath,
        not something this offset is meant to paper over.
        """
        toolpath_files = sorted((REPO_ROOT / "toolpaths").glob("*.jsonl"))
        self.assertTrue(toolpath_files, "expected at least one example toolpath to test against")

        for toolpath_file in toolpath_files:
            with self.subTest(toolpath=toolpath_file.name):
                self.brain.load_jsonl(str(toolpath_file))
                self.assertFalse(self.brain.state.fault_active, self.brain.state.fault_message)
                self.assertTrue(self.brain.queue.operations)

                for operation in self.brain.queue.operations:
                    for axis_name, value in (
                        ("x", operation.x),
                        ("y", operation.y),
                        ("z", operation.die_gap),
                    ):
                        minimum, maximum = self.axis_bounds_mm(axis_name)
                        self.assertTrue(
                            minimum - 1e-6 <= value <= maximum + 1e-6,
                            f"{toolpath_file.name} step {operation.step}: "
                            f"{axis_name}={value} outside [{minimum}, {maximum}]",
                        )


if __name__ == "__main__":
    unittest.main()
