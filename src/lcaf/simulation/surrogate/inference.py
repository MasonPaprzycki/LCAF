"""Load a trained checkpoint and apply it as the UI preview's deformation kernel.

``SurrogateNetwork.apply_strike`` is the direct replacement for
``lcaf.toolpathing.visualization._apply_strike_3d``: same role (one STRIKE
operation's effect on the running per-station material-state grid), same
place in the per-operation loop (see ``visualization.material_state``), but
driven by the trained network instead of a hand-tuned geometric relaxation.
"""

from __future__ import annotations

import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import jax.numpy as jnp
import numpy as np

from . import checkpoint as checkpoint_module
from .geometry import Point2, affected_station_indices, strike_local_frame, strike_process_parameters
from .model import forward_jit
from .preprocessing import denormalize_outputs, normalize_inputs
from .process_params import ProcessParameters


class SurrogateDomainWarning(UserWarning):
    """A strike's process parameters fall outside the network's trained variable space.

    The network has no accuracy guarantee under extrapolation -- see
    ``ProcessParameters.within_trained_domain`` -- so this is surfaced as a
    warning rather than silently trusting the prediction.
    """


@dataclass(frozen=True)
class SurrogateNetwork:
    """A loaded, ready-to-evaluate surrogate checkpoint."""

    checkpoint: checkpoint_module.Checkpoint
    path: Path

    @classmethod
    def load(cls, path: str | Path) -> "SurrogateNetwork":
        path = Path(path)
        return cls(checkpoint=checkpoint_module.load(path), path=path)

    @property
    def metadata(self) -> dict[str, str]:
        return self.checkpoint.metadata

    def predict_local_displacement(
        self,
        process: ProcessParameters,
        x0_mm: Sequence[float],
        y0_mm: Sequence[float],
        z0_mm: Sequence[float],
    ) -> np.ndarray:
        """Vectorised local displacement prediction.

        ``x0_mm``/``y0_mm``/``z0_mm`` must be equal-length 1D sequences.
        Returns an ``(N, 3)`` array of ``(dx0, dy0, dz0)`` in millimetres.
        """
        x0 = np.atleast_1d(np.asarray(x0_mm, dtype=np.float64))
        y0 = np.atleast_1d(np.asarray(y0_mm, dtype=np.float64))
        z0 = np.atleast_1d(np.asarray(z0_mm, dtype=np.float64))
        if not (x0.shape == y0.shape == z0.shape):
            raise ValueError("x0_mm/y0_mm/z0_mm must have matching shapes.")

        n_points = x0.shape[0]
        process_block = np.tile(process.as_array(), (n_points, 1))
        raw_inputs = np.concatenate([process_block, np.stack([x0, y0, z0], axis=1)], axis=1)
        normalized_inputs = normalize_inputs(self.checkpoint.stats, raw_inputs)

        normalized_output = forward_jit(
            self.checkpoint.params,
            jnp.asarray(normalized_inputs),
            activation=self.checkpoint.architecture.activation,
        )
        return denormalize_outputs(self.checkpoint.stats, np.asarray(normalized_output))

    def apply_strike(
        self,
        points_grid: Sequence[Sequence[Point2]],
        station_x_mm: Sequence[float],
        operation_metadata: dict,
        stroke_progress: float = 1.0,
    ) -> list[list[Point2]]:
        """One STRIKE operation's effect on the current per-station ring grid.

        Replaces ``visualization._apply_strike_3d``. Process parameters and
        the strike's local coordinate frame are derived once, at this
        strike's own center station (``segment_index``), from
        ``geometry.strike_process_parameters``/``geometry.strike_local_frame``
        -- matching the paper's own architecture, where one
        ``(alpha0, xb, eps_h)`` triple is shared by every node evaluated for
        one stroke, not re-derived per point. Every grid point within
        ``geometry.affected_station_indices`` of the strike is moved by the
        network's predicted local in-plane displacement (``dx0``: spread,
        ``dy0``: press-direction), rotated back to global (Y, Z); the axial
        component (``dz0``) is computed but not applied to a station's fixed
        X (see ``docs/surrogate_deformation_model.md``'s scope section).

        ``stroke_progress`` (0..1) scales this strike's own ``eps_h`` before
        prediction (not the predicted output afterward) -- physically, a
        partially completed stroke really has achieved a smaller reduction
        so far, not a fraction of the full stroke's displacement.
        ``stroke_progress<=0`` is a special case, returning ``points_grid``
        completely unchanged without evaluating the network at all: the die
        has not moved yet, so nothing has physically happened, and
        ``eps_h=0`` is trivially outside the trained domain (the paper's own
        variable space starts at ``eps_h=0.05``) -- evaluating it anyway
        would extrapolate for no reason and spuriously warn on every single
        stroke's very first animation frame.
        """
        station_count = len(points_grid)
        if station_count == 0 or stroke_progress <= 0.0:
            return [list(row) for row in points_grid]
        center_index = int(operation_metadata["segment_index"])
        center_row = points_grid[center_index]
        rotation_deg = float(operation_metadata["rotation_deg"])
        segment_x_start_mm = float(operation_metadata["segment_x_start_mm"])
        bite_mm = float(operation_metadata["die_length_mm"])
        center_x_mm = station_x_mm[center_index]

        frame = strike_local_frame(center_row, rotation_deg, segment_x_start_mm)
        base_process = strike_process_parameters(center_row, operation_metadata)
        if not base_process.within_trained_domain():
            warnings.warn(
                f"Strike at segment {center_index}, rotation {rotation_deg:.1f} deg has "
                f"process parameters {base_process} outside the network's trained variable "
                f"space -- prediction is an unguaranteed extrapolation.",
                SurrogateDomainWarning,
                stacklevel=2,
            )

        progress = max(0.0, min(stroke_progress, 1.0))
        process = ProcessParameters(base_process.alpha0, base_process.xb, base_process.eps_h * progress)

        new_grid: list[list[Point2]] = [list(row) for row in points_grid]
        affected = affected_station_indices(station_x_mm, center_x_mm, bite_mm)
        for station_index in affected:
            row = new_grid[station_index]
            station_x_value = station_x_mm[station_index]
            local_coords = [frame.to_local(station_x_value, y, z) for y, z in row]
            x0s, y0s, z0s = zip(*local_coords)
            displacement = self.predict_local_displacement(process, x0s, y0s, z0s)
            for index, (y, z) in enumerate(row):
                delta_x0, delta_y0 = float(displacement[index, 0]), float(displacement[index, 1])
                delta_y, delta_z = frame.displacement_to_global(delta_x0, delta_y0)
                row[index] = (y + delta_y, z + delta_z)

        return new_grid
