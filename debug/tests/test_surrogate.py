"""Unit tests for ``lcaf.simulation.surrogate`` -- the JAGTAP-et-al.-style
neural displacement surrogate (see ``docs/surrogate_deformation_model.md``).

Integration with the toolpathing UI/preview (``material_state`` and friends
requiring a ``SurrogateNetwork``) is covered in ``test_toolpath_slicer.py``;
this file tests the surrogate package's own modules in isolation, using
small, fast, freshly initialised (not pretrained) networks throughout --
nothing here needs a network to have learned anything, only to behave like
a well-formed one.
"""

from __future__ import annotations

import math
import warnings
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import jax.numpy as jnp
import numpy as np

from lcaf.simulation.surrogate import checkpoint as checkpoint_module
from lcaf.simulation.surrogate import process_params as pp
from lcaf.simulation.surrogate.geometry import (
    LocalFrame,
    affected_station_indices,
    strike_local_frame,
    strike_process_parameters,
    support_from_row,
)
from lcaf.simulation.surrogate.inference import SurrogateDomainWarning, SurrogateNetwork
from lcaf.simulation.surrogate.model import (
    ArchitectureConfig,
    count_parameters,
    flatten_params,
    forward,
    forward_jit,
    init_params,
    unflatten_params,
)
from lcaf.simulation.surrogate.preprocessing import NormalizationStats, fit, normalize_inputs, denormalize_outputs


def _small_architecture() -> ArchitectureConfig:
    return ArchitectureConfig(hidden_layers=2, hidden_width=8, activation="tanh")


def _identity_stats() -> NormalizationStats:
    return NormalizationStats(
        input_mean=np.zeros(6), input_std=np.ones(6),
        output_mean=np.zeros(3), output_std=np.ones(3),
    )


def _build_test_network(seed: int = 0) -> SurrogateNetwork:
    architecture = _small_architecture()
    params = init_params(architecture, seed)
    checkpoint = checkpoint_module.Checkpoint(
        params=params, architecture=architecture, stats=_identity_stats(), metadata={}
    )
    return SurrogateNetwork(checkpoint=checkpoint, path=Path("<test-network>"))


class ProcessParametersTests(unittest.TestCase):
    def test_aspect_ratio_bite_ratio_height_reduction_formulas(self) -> None:
        self.assertAlmostEqual(pp.aspect_ratio(20.0, 10.0), 2.0)
        self.assertAlmostEqual(pp.bite_ratio(5.0, 20.0), 0.25)
        self.assertAlmostEqual(pp.height_reduction(20.0, 5.0), 0.25)
        self.assertAlmostEqual(pp.height_reduction_from_heights(20.0, 15.0), 0.25)

    def test_non_positive_reference_lengths_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            pp.aspect_ratio(20.0, 0.0)
        with self.assertRaises(ValueError):
            pp.bite_ratio(5.0, 0.0)
        with self.assertRaises(ValueError):
            pp.height_reduction(0.0, 5.0)

    def test_billet_dimensions_bite_length_and_reduction_invert_the_formulas(self) -> None:
        h0, w0 = pp.billet_dimensions_mm(alpha0=1.25, h0_mm=100.0)
        self.assertAlmostEqual(h0, 100.0)
        self.assertAlmostEqual(pp.aspect_ratio(h0, w0), 1.25)

        bite = pp.bite_length_mm(xb=0.4, h0_mm=100.0)
        self.assertAlmostEqual(pp.bite_ratio(bite, 100.0), 0.4)

        reduction = pp.reduction_mm(eps_h=0.15, h0_mm=100.0)
        self.assertAlmostEqual(pp.height_reduction(100.0, reduction), 0.15)

    def test_within_trained_domain_bounds(self) -> None:
        inside = pp.ProcessParameters(alpha0=1.2, xb=0.5, eps_h=0.15)
        self.assertTrue(inside.within_trained_domain())
        outside_alpha = pp.ProcessParameters(alpha0=0.5, xb=0.5, eps_h=0.15)
        self.assertFalse(outside_alpha.within_trained_domain())
        outside_eps_h = pp.ProcessParameters(alpha0=1.2, xb=0.5, eps_h=0.9)
        self.assertFalse(outside_eps_h.within_trained_domain())

    def test_latin_hypercube_samples_cover_every_stratum(self) -> None:
        samples = pp.latin_hypercube_samples({"a": (0.0, 10.0)}, n_samples=10, seed=1)["a"]
        self.assertEqual(len(samples), 10)
        # Each of the 10 equal-width strata [0,1), [1,2), ... must contain
        # exactly one sample -- the defining property of a Latin Hypercube.
        strata = sorted(int(value) for value in samples)
        self.assertEqual(strata, list(range(10)))

    def test_sample_process_parameters_stay_within_the_papers_own_ranges(self) -> None:
        samples = pp.sample_process_parameters(n_samples=25, seed=7)
        self.assertEqual(len(samples), 25)
        for sample in samples:
            self.assertTrue(sample.within_trained_domain())

    def test_sample_process_parameters_is_deterministic_given_a_seed(self) -> None:
        first = pp.sample_process_parameters(10, seed=42)
        second = pp.sample_process_parameters(10, seed=42)
        self.assertEqual(first, second)


class GeometryTests(unittest.TestCase):
    def test_press_and_tangential_directions_are_orthonormal(self) -> None:
        for rotation_deg in (0.0, 37.0, 90.0, 180.0, 271.5):
            frame = LocalFrame(rotation_rad=math.radians(rotation_deg), axial_origin_mm=0.0, anvil_support_mm=0.0)
            press_y, press_z = frame.press_direction()
            tang_y, tang_z = frame.tangential_direction()
            self.assertAlmostEqual(press_y**2 + press_z**2, 1.0, places=9)
            self.assertAlmostEqual(tang_y**2 + tang_z**2, 1.0, places=9)
            self.assertAlmostEqual(press_y * tang_y + press_z * tang_z, 0.0, places=9)

    def test_to_local_places_the_strikes_own_support_point_at_x0_zero(self) -> None:
        # A point exactly at radius R in the press direction (0 elsewhere)
        # must land at local (x0=0, y0=R) when anvil_support_mm=0.
        frame = LocalFrame(rotation_rad=math.radians(30.0), axial_origin_mm=5.0, anvil_support_mm=0.0)
        press_y, press_z = frame.press_direction()
        radius = 12.0
        x0, y0, z0 = frame.to_local(5.0, radius * press_y, radius * press_z)
        self.assertAlmostEqual(x0, 0.0, places=9)
        self.assertAlmostEqual(y0, radius, places=9)
        self.assertAlmostEqual(z0, 0.0, places=9)  # x_mm == axial_origin_mm

    def test_anvil_support_offsets_y0_so_it_is_zero_at_the_anvil_surface(self) -> None:
        frame = LocalFrame(rotation_rad=0.0, axial_origin_mm=0.0, anvil_support_mm=8.0)
        # The anvil surface itself, at radius 8 in the -press direction.
        press_y, press_z = frame.press_direction()
        _, y0, _ = frame.to_local(0.0, -8.0 * press_y, -8.0 * press_z)
        self.assertAlmostEqual(y0, 0.0, places=9)

    def test_displacement_to_global_round_trips_through_to_local(self) -> None:
        frame = LocalFrame(rotation_rad=math.radians(63.0), axial_origin_mm=0.0, anvil_support_mm=0.0)
        origin_y, origin_z = 3.0, -2.0
        delta_y, delta_z = frame.displacement_to_global(1.5, -0.7)
        x0_before, y0_before, _ = frame.to_local(0.0, origin_y, origin_z)
        x0_after, y0_after, _ = frame.to_local(0.0, origin_y + delta_y, origin_z + delta_z)
        self.assertAlmostEqual(x0_after - x0_before, 1.5, places=9)
        self.assertAlmostEqual(y0_after - y0_before, -0.7, places=9)

    def test_support_from_row_is_the_max_projection_onto_the_direction(self) -> None:
        row = [(10.0, 0.0), (0.0, 10.0), (-10.0, 0.0), (0.0, -10.0)]
        self.assertAlmostEqual(support_from_row(row, 0.0), 10.0, places=9)  # +Z direction
        self.assertAlmostEqual(support_from_row(row, math.pi / 2.0), 10.0, places=9)  # +Y direction
        self.assertAlmostEqual(support_from_row(row, math.pi), 10.0, places=9)  # -Z direction

    def test_support_from_row_never_returns_negative(self) -> None:
        row = [(1.0, 1.0), (1.0, 1.0)]
        self.assertGreaterEqual(support_from_row(row, math.pi), 0.0)

    def test_strike_process_parameters_from_a_pristine_cylinder_row(self) -> None:
        radius = 15.0
        angle_count = 32
        angles = [2.0 * math.pi * i / angle_count for i in range(angle_count)]
        row = [(radius * math.cos(a), radius * math.sin(a)) for a in angles]
        metadata = {
            "rotation_deg": 0.0,
            "die_length_mm": 6.0,
            "radial_reduction_mm": 3.0,
            "strike_pass": 1,
        }
        process = strike_process_parameters(row, metadata)
        # h0 = w0 = 2*radius for a pristine, isotropic circle.
        self.assertAlmostEqual(process.alpha0, 1.0, places=6)
        self.assertAlmostEqual(process.xb, 6.0 / (2.0 * radius), places=6)
        self.assertAlmostEqual(process.eps_h, 3.0 / (2.0 * radius), places=6)

    def test_strike_process_parameters_divides_cumulative_reduction_by_pass_number(self) -> None:
        radius = 10.0
        angle_count = 16
        angles = [2.0 * math.pi * i / angle_count for i in range(angle_count)]
        row = [(radius * math.cos(a), radius * math.sin(a)) for a in angles]
        # Pass 2 of 3: radial_reduction_mm is the *cumulative* reduction
        # through this pass, so the per-pass increment is that / strike_pass.
        metadata = {"rotation_deg": 0.0, "die_length_mm": 4.0, "radial_reduction_mm": 4.0, "strike_pass": 2}
        process = strike_process_parameters(row, metadata)
        self.assertAlmostEqual(process.eps_h, (4.0 / 2.0) / (2.0 * radius), places=6)

    def test_strike_local_frame_matches_a_real_toolpath_slicer_plan(self) -> None:
        from lcaf.toolpathing.toolpath_slicer import MachineLimits, SliceSettings, ToolpathSlicer, TriangleMesh

        def box_mesh() -> TriangleMesh:
            vertices = (
                (-10, -5, -5), (10, -5, -5), (10, 5, -5), (-10, 5, -5),
                (-10, -5, 5), (10, -5, 5), (10, 5, 5), (-10, 5, 5),
            )
            faces = (
                (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
                (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
                (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0),
            )
            return TriangleMesh(tuple(tuple(vertices[i] for i in face) for face in faces))

        limits = MachineLimits(x_min_mm=-1000, x_max_mm=1000, y_min_mm=-1000, y_max_mm=1000, z_retracted_mm=-1000, z_extended_mm=1000)
        settings = SliceSettings(stock_radius_mm=10, radial_segments=2, strikes_per_segment=4, max_reduction_per_strike_mm=3, die_contact_z_mm=10)
        plan = ToolpathSlicer(box_mesh(), settings, limits).plan()
        operation = plan.operations[0]

        radius = 10.0
        row = [(radius * math.cos(a), radius * math.sin(a)) for a in (2.0 * math.pi * i / 48 for i in range(48))]
        frame = strike_local_frame(row, operation["metadata"]["rotation_deg"], operation["metadata"]["segment_x_start_mm"])
        # For this pristine, isotropic row, the anvil-side offset the frame
        # picks up is just the stock radius itself (support_from_row of a
        # circle is the same in every direction).
        self.assertAlmostEqual(frame.anvil_support_mm, radius, places=6)
        # A point at exactly this segment's own target support, in the
        # strike's own direction, must map to y0 == that support *plus* the
        # frame's own anvil-side offset (y0=0 is defined at the anvil
        # surface, not at the rotation-axis centre) -- the same support
        # quantity Segment.support_mm computes, just re-based.
        section = plan.sections[operation["metadata"]["segment_index"]]
        target_support = section.support_mm(operation["metadata"]["rotation_deg"])
        press_y, press_z = frame.press_direction()
        _, y0, _ = frame.to_local(
            operation["metadata"]["segment_x_start_mm"], target_support * press_y, target_support * press_z
        )
        self.assertAlmostEqual(y0, target_support + frame.anvil_support_mm, places=6)

    def test_affected_station_indices_window_scales_with_bite_length(self) -> None:
        station_x = tuple(float(i * 10) for i in range(10))
        narrow = affected_station_indices(station_x, center_x_mm=50.0, bite_length_mm=1.0, reach_multiple=1.0)
        wide = affected_station_indices(station_x, center_x_mm=50.0, bite_length_mm=100.0, reach_multiple=1.0)
        self.assertLess(len(narrow), len(wide))
        self.assertIn(5, narrow)  # station at x=50 itself always included


class ModelTests(unittest.TestCase):
    def test_init_params_is_deterministic_given_a_seed_and_has_the_right_shapes(self) -> None:
        architecture = _small_architecture()
        first = init_params(architecture, seed=5)
        second = init_params(architecture, seed=5)
        self.assertEqual(len(first), architecture.hidden_layers + 1)
        for layer_a, layer_b in zip(first, second):
            self.assertTrue(np.array_equal(np.asarray(layer_a["w"]), np.asarray(layer_b["w"])))
        # First layer: (INPUT_DIM=6, hidden_width). Last layer: (hidden_width, OUTPUT_DIM=3).
        self.assertEqual(first[0]["w"].shape, (6, architecture.hidden_width))
        self.assertEqual(first[-1]["w"].shape, (architecture.hidden_width, 3))

    def test_forward_batches_over_the_leading_axis(self) -> None:
        architecture = _small_architecture()
        params = init_params(architecture, seed=0)
        x = jnp.zeros((7, 6))
        y = forward(params, x, activation=architecture.activation)
        self.assertEqual(y.shape, (7, 3))
        self.assertTrue(bool(jnp.all(jnp.isfinite(y))))

    def test_forward_jit_matches_plain_forward(self) -> None:
        architecture = _small_architecture()
        params = init_params(architecture, seed=1)
        x = jnp.asarray(np.random.default_rng(0).normal(size=(4, 6)))
        eager = forward(params, x, activation=architecture.activation)
        jitted = forward_jit(params, x, activation=architecture.activation)
        self.assertTrue(np.allclose(np.asarray(eager), np.asarray(jitted), atol=1e-6))

    def test_flatten_unflatten_round_trips(self) -> None:
        architecture = _small_architecture()
        params = init_params(architecture, seed=2)
        flat = flatten_params(params)
        restored = unflatten_params(flat, num_layers=architecture.hidden_layers + 1)
        for original, restored_layer in zip(params, restored):
            self.assertTrue(np.array_equal(np.asarray(original["w"]), np.asarray(restored_layer["w"])))
            self.assertTrue(np.array_equal(np.asarray(original["b"]), np.asarray(restored_layer["b"])))

    def test_count_parameters_matches_manual_sum(self) -> None:
        architecture = _small_architecture()
        params = init_params(architecture, seed=0)
        expected = sum(layer["w"].size + layer["b"].size for layer in params)
        self.assertEqual(count_parameters(params), expected)

    def test_unknown_activation_is_rejected(self) -> None:
        architecture = _small_architecture()
        params = init_params(architecture, seed=0)
        with self.assertRaises(ValueError):
            forward(params, jnp.zeros((1, 6)), activation="not-a-real-activation")


class PreprocessingTests(unittest.TestCase):
    def test_fit_normalize_denormalize_round_trip(self) -> None:
        rng = np.random.default_rng(0)
        inputs = rng.normal(loc=5.0, scale=2.0, size=(200, 6))
        outputs = rng.normal(loc=-1.0, scale=0.5, size=(200, 3))
        stats = fit(inputs, outputs)
        normalized_inputs = normalize_inputs(stats, inputs)
        self.assertAlmostEqual(float(normalized_inputs.mean()), 0.0, places=6)
        self.assertAlmostEqual(float(normalized_inputs.std()), 1.0, places=1)
        # denormalize_outputs(normalize(outputs)) == outputs.
        from lcaf.simulation.surrogate.preprocessing import normalize_outputs
        round_tripped = denormalize_outputs(stats, normalize_outputs(stats, outputs))
        self.assertTrue(np.allclose(round_tripped, outputs))

    def test_zero_variance_channel_does_not_divide_by_zero(self) -> None:
        inputs = np.zeros((10, 6))
        outputs = np.zeros((10, 3))
        stats = fit(inputs, outputs)
        normalized = normalize_inputs(stats, inputs)
        self.assertTrue(np.all(np.isfinite(normalized)))

    def test_stats_reject_wrong_shapes(self) -> None:
        with self.assertRaises(ValueError):
            NormalizationStats(np.zeros(5), np.ones(6), np.zeros(3), np.ones(3))


class CheckpointTests(unittest.TestCase):
    def test_save_load_round_trip_preserves_everything(self) -> None:
        architecture = _small_architecture()
        params = init_params(architecture, seed=9)
        stats = _identity_stats()
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "nested" / "model.npz"
            checkpoint_module.save(path, params, architecture, stats, {"note": "unit test"})
            self.assertTrue(path.exists())
            loaded = checkpoint_module.load(path)

        self.assertEqual(loaded.architecture, architecture)
        for original, restored in zip(params, loaded.params):
            self.assertTrue(np.array_equal(np.asarray(original["w"]), np.asarray(restored["w"])))
        self.assertTrue(np.array_equal(loaded.stats.input_mean, stats.input_mean))
        self.assertEqual(loaded.metadata["note"], "unit test")
        self.assertIn("paper_citation", loaded.metadata)
        self.assertIn("10.21741/9781644903131-253", loaded.metadata["paper_citation"])

    def test_load_missing_file_raises_a_clear_error(self) -> None:
        with self.assertRaises(FileNotFoundError):
            checkpoint_module.load("this/path/does/not/exist.npz")

    def test_load_rejects_a_mismatched_format_version(self) -> None:
        architecture = _small_architecture()
        params = init_params(architecture, seed=0)
        stats = _identity_stats()
        with TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "model.npz"
            checkpoint_module.save(path, params, architecture, stats)
            # Corrupt the format_version in place.
            with np.load(path, allow_pickle=False) as data:
                arrays = {name: data[name] for name in data.files}
            arrays["format_version"] = np.array(999)
            np.savez(path, **arrays)
            with self.assertRaises(ValueError):
                checkpoint_module.load(path)


class SurrogateNetworkTests(unittest.TestCase):
    def test_predict_local_displacement_shape_and_determinism(self) -> None:
        network = _build_test_network(seed=4)
        process = pp.ProcessParameters(1.1, 0.5, 0.15)
        x0 = np.linspace(-10.0, 10.0, 6)
        y0 = np.linspace(0.0, 20.0, 6)
        z0 = np.linspace(-5.0, 5.0, 6)
        first = network.predict_local_displacement(process, x0, y0, z0)
        second = network.predict_local_displacement(process, x0, y0, z0)
        self.assertEqual(first.shape, (6, 3))
        self.assertTrue(np.array_equal(first, second))
        self.assertTrue(np.all(np.isfinite(first)))

    def test_predict_local_displacement_rejects_mismatched_shapes(self) -> None:
        network = _build_test_network()
        process = pp.ProcessParameters(1.1, 0.5, 0.15)
        with self.assertRaises(ValueError):
            network.predict_local_displacement(process, [0.0, 1.0], [0.0], [0.0])

    def test_apply_strike_at_progress_zero_is_an_exact_identity(self) -> None:
        network = _build_test_network()
        angle_count = 16
        angles = [2.0 * math.pi * i / angle_count for i in range(angle_count)]
        row = [(10.0 * math.cos(a), 10.0 * math.sin(a)) for a in angles]
        grid = [list(row), list(row), list(row)]
        metadata = {
            "segment_index": 1, "rotation_deg": 0.0, "segment_x_start_mm": 0.0,
            "die_length_mm": 4.0, "radial_reduction_mm": 2.0, "strike_pass": 1,
        }
        result = network.apply_strike(grid, [0.0, 4.0, 8.0], metadata, stroke_progress=0.0)
        self.assertEqual(result, grid)

    def test_apply_strike_at_full_progress_moves_the_struck_station(self) -> None:
        network = _build_test_network()
        angle_count = 16
        angles = [2.0 * math.pi * i / angle_count for i in range(angle_count)]
        row = [(10.0 * math.cos(a), 10.0 * math.sin(a)) for a in angles]
        grid = [list(row), list(row), list(row)]
        metadata = {
            "segment_index": 1, "rotation_deg": 0.0, "segment_x_start_mm": 0.0,
            "die_length_mm": 4.0, "radial_reduction_mm": 2.0, "strike_pass": 1,
        }
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", SurrogateDomainWarning)
            result = network.apply_strike(grid, [0.0, 4.0, 8.0], metadata, stroke_progress=1.0)
        self.assertNotEqual(result[1], grid[1])

    def test_apply_strike_warns_outside_the_trained_domain(self) -> None:
        network = _build_test_network()
        angle_count = 16
        angles = [2.0 * math.pi * i / angle_count for i in range(angle_count)]
        row = [(30.0 * math.cos(a), 30.0 * math.sin(a)) for a in angles]
        grid = [list(row)]
        # eps_h = reduction / h0 = 25 / 60 ~= 0.42, above the trained
        # domain's own eps_h upper bound (0.26).
        metadata = {
            "segment_index": 0, "rotation_deg": 0.0, "segment_x_start_mm": 0.0,
            "die_length_mm": 4.0, "radial_reduction_mm": 25.0, "strike_pass": 1,
        }
        with self.assertWarns(SurrogateDomainWarning):
            network.apply_strike(grid, [0.0], metadata, stroke_progress=1.0)

    def test_load_missing_checkpoint_raises(self) -> None:
        with self.assertRaises(FileNotFoundError):
            SurrogateNetwork.load("this/path/does/not/exist.npz")


if __name__ == "__main__":
    unittest.main()
