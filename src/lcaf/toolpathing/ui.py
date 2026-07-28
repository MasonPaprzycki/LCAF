"""A dependency-free preview and export UI for :mod:`lcaf.toolpathing`.

This is deliberately a planning/proving front end, not a machine-control UI.
It never connects to LinuxCNC or sends a command; its only output is JSONL for
review and later loading by ``ForgeBrain``.
"""

from __future__ import annotations

import dataclasses
import math
import tkinter as tk
from time import perf_counter
from pathlib import Path
from tkinter import filedialog, messagebox, ttk
from typing import Sequence

from .material import MATERIAL_LABELS, MATERIALS, plan_force_report
from .toolpath_slicer import (
    MachineLimits,
    SliceSettings,
    ToolpathPlan,
    ToolpathPlanningError,
    ToolpathSlicer,
    load_mesh,
)
from .visualization import (
    anvil_side_support,
    axial_trim_allowance_mm,
    die_contact_profile,
    disc_contact_profile,
    disc_rim_profile,
    find_sufficient_cycles,
    material_state,
    radial_resample,
)


# Presentation-only hints for the material picker -- not used by any
# mechanics, purely to help a user pick a realistic target_temperature_c for
# lcaf.toolpathing.material's own bands. Keyed by family (the part of a
# material key before its first "_"), since every grade of a family is
# worked in roughly the same temperature range.
_MATERIAL_FAMILY_TEMPERATURE_HINTS = {
    "plasticine": "Typically worked near room temperature, ~15-30 C.",
    "aluminum": "Commonly hot-forged around 400-500 C.",
    "steel": "Commonly hot-forged around 900-1250 C.",
}


def _material_temperature_hint(material_key: str) -> str:
    family = material_key.split("_", 1)[0]
    return _MATERIAL_FAMILY_TEMPERATURE_HINTS.get(family, "")


class ForgePreview(ttk.Frame):
    """Draw the 2D red-stock / green-target / gold-die playback view."""

    background = "#101827"
    grid = "#26344a"
    text = "#d9e2f2"
    stock = "#c93636"
    stock_edge = "#ff8585"
    target = "#4fc3a1"
    target_fill = "#226b50"
    die = "#ffb454"
    upper_die = "#bcdfff"
    strike = "#e77878"
    trim_allowance = "#c98a2c"
    trim_line = "#ffcf5c"

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master)
        self.canvas = tk.Canvas(self, width=920, height=320, bg=self.background, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.plan: ToolpathPlan | None = None
        self.stock_radius = 0.0
        self.die_contact_z = 0.0
        self.selected_station = 0
        self.selected_rotation = 0.0
        self.operation_index = 0
        self.motion_progress = 0.0
        self.mesh_quality = 48
        self.show_dies = True
        self._pre_stroke_state: tuple | None = None
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.redraw()

    def set_mesh_quality(self, radial_segments: int) -> None:
        self.mesh_quality = max(8, radial_segments)
        self.redraw()

    def set_show_dies(self, show: bool) -> None:
        self.show_dies = show
        self.redraw()

    def show_plan(self, plan: ToolpathPlan, settings: SliceSettings) -> None:
        self.plan = plan
        self.stock_radius = settings.stock_radius_mm
        self.die_contact_z = settings.die_contact_z_mm
        self.selected_station = min(self.selected_station, max(0, len(plan.sections) - 1))
        self.operation_index = 0
        self.motion_progress = 0.0
        self._select_operation(0)
        self.redraw()

    def choose_station(self, station: int) -> None:
        self.selected_station = station
        self.redraw()

    def show_operation(self, operation_index: int, motion_progress: float) -> None:
        """Show a generated command and the die's in-stroke animation state."""
        if self.plan is None or not self.plan.operations:
            return
        self.operation_index = max(0, min(operation_index, len(self.plan.operations) - 1))
        self.motion_progress = max(0.0, min(motion_progress, 1.0))
        self._select_operation(self.operation_index)
        self.redraw()

    def _select_operation(self, operation_index: int) -> None:
        assert self.plan is not None
        operation = self.plan.operations[operation_index]
        self.selected_station = int(operation["metadata"]["segment_index"])
        self.selected_rotation = float(operation["rotation"])

    def _active_operation(self) -> dict | None:
        if self.plan is None or not self.plan.operations:
            return None
        return self.plan.operations[self.operation_index]

    def _active_die_support(self) -> float:
        """The die's animated depth: from wherever the material actually is right
        now (not always the pristine stock radius -- a later pass on an
        already-reduced station must not appear to retract through material
        that is no longer there) to this stroke's commanded final depth.
        """
        operation = self._active_operation()
        if operation is None:
            return self.stock_radius
        reduction = float(operation["metadata"]["radial_reduction_mm"])
        end_support = self.stock_radius - reduction
        start_support = self._stroke_start_support()
        return start_support + (end_support - start_support) * self.motion_progress

    def _stroke_start_support(self) -> float:
        operation = self._active_operation()
        if operation is None or not self._pre_stroke_state:
            return self.stock_radius
        ring = self._pre_stroke_state[self.selected_station]
        if not ring:
            return self.stock_radius
        rotation = math.radians(float(operation["rotation"]))
        return max(y * math.sin(rotation) + z * math.cos(rotation) for y, z in ring)

    def _stroke_start_anvil_support(self) -> float:
        """The material's actual current boundary on the anvil's (-normal)
        side -- the anvil never moves, but an earlier rotation may already
        have reduced this side below the pristine stock radius, so the
        anvil must be drawn at wherever material actually is, not a fixed
        position that can appear to float clear of the billet.
        """
        operation = self._active_operation()
        if operation is None or not self._pre_stroke_state:
            return self.stock_radius
        ring = self._pre_stroke_state[self.selected_station]
        if not ring:
            return self.stock_radius
        rotation = float(operation["rotation"])
        return anvil_side_support(ring, rotation)

    def _active_die_geometry(self) -> tuple[float | None, float]:
        """Return the active operation's (die_width_mm, die_corner_radius_mm)."""
        operation = self._active_operation()
        if operation is None:
            return None, 0.0
        metadata = operation["metadata"]
        width = metadata.get("die_width_mm")
        corner_radius = float(metadata.get("die_corner_radius_mm", 0.0) or 0.0)
        return (float(width) if width is not None else None), corner_radius

    def _active_upper_die_radius(self) -> float | None:
        operation = self._active_operation()
        if operation is None:
            return None
        radius = operation["metadata"].get("upper_die_radius_mm")
        return float(radius) if radius is not None else None

    def _active_die_length(self) -> float | None:
        operation = self._active_operation()
        if operation is None:
            return None
        length = operation["metadata"].get("die_length_mm")
        return float(length) if length is not None else None

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 920)
        height = max(canvas.winfo_height(), 415)
        split = int(width * 0.48)
        canvas.create_text(22, 18, anchor="w", text="CROSS-SECTION / DIE ENVELOPE", fill=self.text, font=("Segoe UI", 10, "bold"))
        canvas.create_text(split + 22, 18, anchor="w", text="AXIAL STRIKE PLAN", fill=self.text, font=("Segoe UI", 10, "bold"))
        canvas.create_line(split, 10, split, height - 12, fill=self.grid, width=2)

        if self.plan is None or not self.plan.sections:
            canvas.create_text(width / 2, height / 2, text="Load a watertight OBJ or STL, then generate a preview.", fill="#8fa3c0", font=("Segoe UI", 12))
            return
        state = material_state(self.plan, self.operation_index, self.motion_progress, radial_segments=self.mesh_quality)
        self._pre_stroke_state = material_state(self.plan, self.operation_index, 0.0, radial_segments=self.mesh_quality)
        self._draw_cross_section(20, 40, split - 40, height - 60, state)
        self._draw_axial_plan(split + 20, 40, width - split - 40, height - 60, state)

    def _draw_cross_section(self, left: int, top: int, width: int, height: int, state: tuple) -> None:
        assert self.plan is not None
        section = self.plan.sections[self.selected_station]
        center_x, center_y = left + width / 2, top + height / 2
        extent = max(
            self.stock_radius,
            max(abs(value) for point in section.polygon_yz_mm for value in point),
            1.0,
        )
        scale = min(width, height) * 0.40 / extent

        for fraction in (-0.5, 0.0, 0.5):
            canvas_x = center_x + fraction * extent * 2 * scale
            canvas_y = center_y - fraction * extent * 2 * scale
            self.canvas.create_line(canvas_x, top + 10, canvas_x, top + height - 8, fill=self.grid)
            self.canvas.create_line(left + 8, canvas_y, left + width - 8, canvas_y, fill=self.grid)

        radius = self.stock_radius * scale
        self.canvas.create_oval(
            center_x - radius,
            center_y - radius,
            center_x + radius,
            center_y + radius,
            outline=self.stock_edge,
            width=1,
            dash=(5, 4),
        )
        target_polygon = [
            coordinate
            for y, z in section.polygon_yz_mm
            for coordinate in (center_x + y * scale, center_y - z * scale)
        ]
        material_polygon = [
            coordinate
            for y, z in state[self.selected_station]
            for coordinate in (center_x + y * scale, center_y - z * scale)
        ]
        self.canvas.create_polygon(*material_polygon, fill=self.stock, outline=self.stock_edge, width=2)
        self.canvas.create_polygon(
            *target_polygon,
            fill=self.target_fill,
            outline=self.target,
            width=3,
            stipple="gray25",
        )

        rotation = self._selected_rotation()
        support = section.support_mm(rotation)
        active_support = self._active_die_support()
        angle = math.radians(rotation)
        normal_y, normal_z = math.sin(angle), math.cos(angle)
        tangent_y, tangent_z = normal_z, -normal_y
        die_width, die_corner_radius = self._active_die_geometry()
        upper_die_radius = self._active_upper_die_radius()

        def draw_profile(profile, color, stipple=""):
            points = [
                (
                    center_x + (normal_y * depth + tangent_y * offset) * scale,
                    center_y - (normal_z * depth + tangent_z * offset) * scale,
                )
                for offset, depth in profile
            ]
            self.canvas.create_line(
                *(coordinate for point in points for coordinate in point),
                fill=color, width=6, joinstyle="round", capstyle="round", stipple=stipple,
            )

        if self.show_dies:
            # The lower die (the one with prescribed geometry) is the only
            # one that animates through a stroke.
            lower_profile = die_contact_profile(die_width, die_corner_radius, active_support, samples=20)
            if lower_profile is None:
                line_length = extent * 1.25
                lower_profile = ((-line_length, active_support), (line_length, active_support))
            draw_profile(lower_profile, self.die)
            # The upper die never moves in this view, but it must always be
            # drawn touching wherever the material actually currently is on
            # its side -- not a fixed pristine position, which can drift out
            # of contact ("levitate") once an earlier rotation has already
            # reduced that side -- and as its own finite circular disc, not
            # an unbounded plane.
            anvil_support = self._stroke_start_anvil_support()
            upper_profile = disc_contact_profile(upper_die_radius, 0.0, -anvil_support, samples=20)
            if upper_profile is None:
                line_length = extent * 1.25
                upper_profile = ((-line_length, -anvil_support), (line_length, -anvil_support))
            draw_profile(upper_profile, self.upper_die, stipple="gray50")

        die_dimensions = ""
        if die_width is not None:
            die_dimensions = f"  lower_die_width={die_width:.1f} mm"
            if die_corner_radius > 0:
                die_dimensions += f" R{die_corner_radius:.1f}"
        if upper_die_radius is not None:
            die_dimensions += f"  upper_die_radius={upper_die_radius:.1f} mm"
        self.canvas.create_text(
            left + 10,
            top + height - 4,
            anchor="sw",
            text=(
                f"station {self.selected_station + 1}/{len(self.plan.sections)}  "
                f"X={section.x_model_mm:.2f} mm  A={rotation:.0f}°  "
                f"die={active_support:.2f} mm  target={support:.2f} mm{die_dimensions}"
            ),
            fill=self.text,
            font=("Consolas", 9),
        )
        self.canvas.create_text(left + 10, top + 5, anchor="nw", text="red: remaining stock   green hatch: target geometry   gold: lower die (moves)   translucent blue: fixed upper die, sized circular disc (toggle below)   dashed: original cylinder", fill="#d9e2f2", font=("Segoe UI", 9))

    def _draw_axial_plan(self, left: int, top: int, width: int, height: int, state: tuple) -> None:
        assert self.plan is not None
        sections = self.plan.sections
        x_values = [section.x_model_mm for section in sections]
        stock_half_length = self.plan.recommended_stock_length_mm / 2.0
        # Forging conserves volume exactly: whatever a die's local bulge does
        # not reabsorb nearby must reappear as extra length at the free
        # (+X) end, ready to be sawn off once forging is complete -- see
        # axial_trim_allowance_mm's own docstring for the volume balance
        # this comes from.
        axial_extension_mm = axial_trim_allowance_mm(
            self.plan, self.operation_index, self.motion_progress, radial_segments=self.mesh_quality
        )
        free_end_x = sections[-1].x_model_mm
        extension_x = free_end_x + axial_extension_mm
        min_x = min(min(x_values), -stock_half_length)
        max_x = max(max(x_values), stock_half_length, extension_x)
        x_range = max(max_x - min_x, 1.0)
        centre_y = top + height * 0.5
        radial_scale = height * 0.31 / max(self.stock_radius, 1.0)

        def to_x(model_x: float) -> float:
            return left + 18 + (model_x - min_x) / x_range * (width - 36)

        stock_top = centre_y - self.stock_radius * radial_scale
        stock_bottom = centre_y + self.stock_radius * radial_scale
        # The dashed box is the volume-matched stock cylinder's true length,
        # which may run proud of the target's own axial extent at either end
        # (that overhang is stock never struck -- material to trim away).
        self.canvas.create_rectangle(
            to_x(-stock_half_length), stock_top, to_x(stock_half_length), stock_bottom,
            outline=self.stock_edge, width=1, dash=(5, 4),
        )

        # At 0° the positive-Z support is exactly the side-view top profile.
        top_profile = [to_x(section.x_model_mm) for section in sections]
        target_top = [centre_y - section.support_mm(0.0) * radial_scale for section in sections]
        target_bottom = [centre_y + (-min(z for _, z in section.polygon_yz_mm)) * radial_scale for section in reversed(sections)]
        target_polygon = [coordinate for pair in zip(top_profile, target_top) for coordinate in pair]
        target_polygon += [coordinate for pair in zip(reversed(top_profile), target_bottom) for coordinate in pair]
        material_top = [centre_y - max(z for _, z in ring) * radial_scale for ring in state]
        material_bottom = [centre_y - min(z for _, z in ring) * radial_scale for ring in reversed(state)]
        material_polygon = [coordinate for pair in zip(top_profile, material_top) for coordinate in pair]
        material_polygon += [coordinate for pair in zip(reversed(top_profile), material_bottom) for coordinate in pair]
        self.canvas.create_polygon(*material_polygon, fill=self.stock, outline=self.stock_edge, width=2)
        self.canvas.create_polygon(*target_polygon, fill=self.target_fill, outline=self.target, width=2, stipple="gray25")

        if axial_extension_mm > 0.0:
            # The trim-allowance stub: a straight extrusion of the free
            # end's own current top/bottom extent out to extension_x --
            # material volume conservation says must be there, ready to be
            # sawn off once forging is complete.
            free_top = centre_y - max(z for _, z in state[-1]) * radial_scale
            free_bottom = centre_y - min(z for _, z in state[-1]) * radial_scale
            free_x_px = to_x(free_end_x)
            extension_x_px = to_x(extension_x)
            self.canvas.create_polygon(
                free_x_px, free_top, extension_x_px, free_top,
                extension_x_px, free_bottom, free_x_px, free_bottom,
                fill=self.trim_allowance, outline=self.stock_edge, width=2,
            )
            self.canvas.create_line(
                free_x_px, free_top - 6, free_x_px, free_bottom + 6,
                fill=self.trim_line, width=2, dash=(5, 4),
            )

        selected = sections[self.selected_station]
        selected_x = to_x(selected.x_model_mm)
        self.canvas.create_line(selected_x, top + 24, selected_x, top + height - 22, fill=self.strike, width=2)
        active_support = self._active_die_support()
        die_y = centre_y - active_support * radial_scale
        die_length = self._active_die_length()
        if die_length is not None:
            pixels_per_mm = (width - 36) / x_range
            half_span = max(6.0, die_length * pixels_per_mm / 2.0)
        else:
            half_span = 16.0
        self.canvas.create_line(selected_x - half_span, die_y, selected_x + half_span, die_y, fill=self.die, width=6)
        self.canvas.create_text(selected_x + 5, top + 30, anchor="nw", text="active\nstation", fill=self.strike, font=("Segoe UI", 9, "bold"))

        # The first 32 operations make the alternating rotation sweep visible
        # without turning the preview into unreadable noise.
        for operation in self.plan.operations[:32]:
            x = to_x(float(operation["metadata"]["model_x_mm"]))
            rotation = float(operation["rotation"])
            color = (self.die, "#80b7ff", "#d68aff", "#f58a6f")[int(round(rotation / 90.0)) % 4]
            self.canvas.create_line(x, stock_top - 12, x, stock_top - 4, fill=color, width=3)
        operation = self._active_operation()
        step_text = "no operation selected"
        if operation is not None:
            step_text = (
                f"step {self.operation_index + 1}/{len(self.plan.operations)}  "
                f"A={float(operation['rotation']):.0f}°  "
                f"pass {operation['metadata']['strike_pass']}/{operation['metadata']['strike_pass_count']}"
            )
        trim_text = f"  |  amber: trim allowance {axial_extension_mm:.1f} mm (saw off)" if axial_extension_mm > 0.0 else ""
        self.canvas.create_text(left + 10, top + height - 4, anchor="sw", text=f"red: remaining stock   green hatch: target profile   gold: lower die (moves)  |  {step_text}{trim_text}", fill="#d9e2f2", font=("Segoe UI", 9))

    def _selected_rotation(self) -> float:
        return self.selected_rotation


class Forge3DPreview(ttk.Frame):
    """A dependency-free isometric 3D envelope playback view."""

    background = "#101827"
    text = "#d9e2f2"
    stock_faces = ("#7e1f28", "#a92d35", "#cc3a40")
    stock_edge = "#ff7a7a"
    target = "#58d49c"
    die = "#ffbd4a"
    upper_die = "#bcdfff"
    grid = "#26344a"
    trim_allowance = "#c98a2c"
    trim_line = "#ffcf5c"

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master)
        self.canvas = tk.Canvas(self, width=920, height=340, bg=self.background, highlightthickness=0, cursor="fleur")
        self.canvas.pack(fill="both", expand=True)
        self.plan: ToolpathPlan | None = None
        self.stock_radius = 0.0
        self.selected_station = 0
        self.operation_index = 0
        self.motion_progress = 0.0
        self.selected_rotation = 0.0
        self.yaw_deg = -32.0
        self.pitch_deg = 24.0
        self.spin_deg = 0.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.zoom = 1.0
        self.mesh_quality = 36
        self.show_dies = True
        self._pre_stroke_state: tuple | None = None
        self._drag_origin: tuple[int, int, float, float, float, float, float] | None = None
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self._on_drag_start)
        self.canvas.bind("<B1-Motion>", self._on_drag_move)

        self.mode_var = tk.StringVar(value="roll")
        mode_controls = ttk.Frame(self)
        for label, mode in (
            ("Rotate", "rotate"),
            ("Roll", "roll"),
            ("Pan", "pan"),
        ):
            ttk.Radiobutton(
                mode_controls,
                text=label,
                value=mode,
                variable=self.mode_var,
                style="Toolbutton",
            ).pack(side="left", padx=(0, 2))
        ttk.Button(mode_controls, text="Reset view", command=self.reset_view).pack(side="left", padx=(10, 0))
        mode_controls.place(x=10, y=6, anchor="nw")

        zoom_controls = ttk.Frame(self)
        ttk.Button(zoom_controls, text="−", width=3, command=self._zoom_out).pack(side="left")
        ttk.Button(zoom_controls, text="+", width=3, command=self._zoom_in).pack(side="left", padx=(4, 0))
        zoom_controls.place(relx=1.0, y=6, x=-10, anchor="ne")

        self.redraw()

    def _zoom_in(self) -> None:
        self.zoom = min(5.0, self.zoom * 1.2)
        self.redraw()

    def _zoom_out(self) -> None:
        self.zoom = max(0.25, self.zoom / 1.2)
        self.redraw()

    def _on_drag_start(self, event: tk.Event) -> None:
        self._drag_origin = (
            event.x, event.y, self.yaw_deg, self.pitch_deg, self.spin_deg, self.pan_x, self.pan_y,
        )

    def _on_drag_move(self, event: tk.Event) -> None:
        if self._drag_origin is None:
            return
        start_x, start_y, start_yaw, start_pitch, start_spin, start_pan_x, start_pan_y = self._drag_origin
        mode = self.mode_var.get()
        if mode == "roll":
            # Free 3D orbit, examining the part from any angle.
            self.yaw_deg = start_yaw + (event.x - start_x) * 0.4
            self.pitch_deg = max(-85.0, min(85.0, start_pitch - (event.y - start_y) * 0.4))
        elif mode == "rotate":
            # Spin the whole view about the fixed axis pointing into the screen.
            centre_x = max(self.canvas.winfo_width(), 1) / 2.0
            centre_y = max(self.canvas.winfo_height(), 1) / 2.0
            start_angle = math.degrees(math.atan2(start_y - centre_y, start_x - centre_x))
            current_angle = math.degrees(math.atan2(event.y - centre_y, event.x - centre_x))
            self.spin_deg = start_spin + (current_angle - start_angle)
        else:  # "pan"
            self.pan_x = start_pan_x + (event.x - start_x)
            self.pan_y = start_pan_y + (event.y - start_y)
        self.redraw()

    def reset_view(self) -> None:
        self.yaw_deg, self.pitch_deg, self.zoom = -32.0, 24.0, 1.0
        self.spin_deg, self.pan_x, self.pan_y = 0.0, 0.0, 0.0
        self.redraw()

    def set_mesh_quality(self, radial_segments: int) -> None:
        self.mesh_quality = max(8, radial_segments)
        self.redraw()

    def set_show_dies(self, show: bool) -> None:
        self.show_dies = show
        self.redraw()

    def show_plan(self, plan: ToolpathPlan, settings: SliceSettings) -> None:
        self.plan = plan
        self.stock_radius = settings.stock_radius_mm
        self.show_operation(0, 0.0)

    def show_operation(self, operation_index: int, motion_progress: float) -> None:
        if self.plan is None or not self.plan.operations:
            return
        self.operation_index = max(0, min(operation_index, len(self.plan.operations) - 1))
        self.motion_progress = max(0.0, min(motion_progress, 1.0))
        operation = self.plan.operations[self.operation_index]
        self.selected_station = int(operation["metadata"]["segment_index"])
        self.selected_rotation = float(operation["rotation"])
        self.redraw()

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 920)
        height = max(canvas.winfo_height(), 455)
        canvas.create_text(22, 18, anchor="w", text="3D GEOMETRIC-ENVELOPE PLAYBACK", fill=self.text, font=("Segoe UI", 10, "bold"))
        canvas.create_text(width - 22, 40, anchor="e", text="hold and drag to rotate/roll/pan · +/− to zoom", fill="#8fa3c0", font=("Segoe UI", 9))
        if self.plan is None or not self.plan.sections:
            canvas.create_text(width / 2, height / 2, text="Generate a plan, then switch here to watch the 3D envelope evolve.", fill="#8fa3c0", font=("Segoe UI", 12))
            return

        sections = self.plan.sections
        segment_count = self.mesh_quality
        material_rings = material_state(self.plan, self.operation_index, self.motion_progress, radial_segments=segment_count)
        self._pre_stroke_state = material_state(self.plan, self.operation_index, 0.0, radial_segments=segment_count)
        target_rings = [
            radial_resample(section.polygon_yz_mm, radial_segments=segment_count)
            for section in sections
        ]
        # Forging conserves volume exactly: whatever a die's local bulge does
        # not reabsorb nearby must reappear as extra length at the free
        # (+X) end, ready to be sawn off once forging is complete -- see
        # axial_trim_allowance_mm's own docstring for the volume balance
        # this comes from.
        axial_extension_mm = axial_trim_allowance_mm(
            self.plan, self.operation_index, self.motion_progress, radial_segments=segment_count
        )
        free_end_x = sections[-1].x_model_mm
        extension_x = free_end_x + axial_extension_mm
        project, depth = self._projector(
            width, height, sections, extra_max_x=extension_x if axial_extension_mm > 0.0 else None
        )

        faces: list[tuple[float, tuple[float, ...], str]] = []
        for station_index in range(len(sections) - 1):
            for radial_index in range(segment_count):
                next_radial = (radial_index + 1) % segment_count
                ring_a, ring_b = material_rings[station_index], material_rings[station_index + 1]
                corners = (
                    (sections[station_index].x_model_mm, *ring_a[radial_index]),
                    (sections[station_index].x_model_mm, *ring_a[next_radial]),
                    (sections[station_index + 1].x_model_mm, *ring_b[next_radial]),
                    (sections[station_index + 1].x_model_mm, *ring_b[radial_index]),
                )
                shade = self.stock_faces[radial_index % len(self.stock_faces)]
                faces.append((sum(depth(*corner) for corner in corners) / 4.0, tuple(value for corner in corners for value in project(*corner)), shade))

        if axial_extension_mm > 0.0:
            # The trim-allowance stub: a straight extrusion of the free
            # end's own current ring out to extension_x, since that ring's
            # own shape is the most honest available guess at what the
            # (not-yet-locally-reabsorbed) excess material actually looks
            # like there. Rendered in a distinct amber so it reads as "to be
            # sawn off," not more of the same forged stock.
            free_ring = material_rings[-1]
            for radial_index in range(segment_count):
                next_radial = (radial_index + 1) % segment_count
                corners = (
                    (free_end_x, *free_ring[radial_index]),
                    (free_end_x, *free_ring[next_radial]),
                    (extension_x, *free_ring[next_radial]),
                    (extension_x, *free_ring[radial_index]),
                )
                faces.append((
                    sum(depth(*corner) for corner in corners) / 4.0,
                    tuple(value for corner in corners for value in project(*corner)),
                    self.trim_allowance,
                ))

        for _, coordinates, color in sorted(faces, key=lambda face: face[0]):
            canvas.create_polygon(*coordinates, fill=color, outline="#6f1820", width=1)

        if axial_extension_mm > 0.0:
            # A dashed "cut here" ring right at the target's own free end,
            # where the trim-allowance stub begins.
            free_ring = material_rings[-1]
            coordinates = [value for y, z in free_ring for value in project(free_end_x, y, z)]
            canvas.create_line(
                *coordinates, coordinates[0], coordinates[1],
                fill=self.trim_line, width=2, dash=(6, 4),
            )

        # The target is intentionally rendered as an overlaid green wireframe
        # so it remains visible even while it sits inside the red stock solid.
        for ring_index, ring in enumerate(target_rings):
            coordinates = [
                value
                for y, z in ring
                for value in project(sections[ring_index].x_model_mm, y, z)
            ]
            canvas.create_line(*coordinates, coordinates[0], coordinates[1], fill=self.target, width=2)
        for radial_index in range(0, segment_count, 4):
            coordinates = [
                value
                for station_index, ring in enumerate(target_rings)
                for value in project(sections[station_index].x_model_mm, *ring[radial_index])
            ]
            canvas.create_line(*coordinates, fill=self.target, width=1)

        if self.show_dies:
            self._draw_upper_die(project)
            self._draw_lower_die(project)
        canvas.create_text(22, height - 20, anchor="w", text="solid red: remaining clipped stock   green wireframe: target geometry   gold: lower die (moves)   translucent blue: fixed upper die, sized circular disc (toggle below)", fill=self.text, font=("Segoe UI", 9))
        operation = self.plan.operations[self.operation_index]
        canvas.create_text(width - 22, height - 20, anchor="e", text=f"step {self.operation_index + 1}/{len(self.plan.operations)} · A={float(operation['rotation']):.0f}°", fill=self.text, font=("Segoe UI", 9))
        self._draw_axis_gizmo(width, height)

    def _draw_axis_gizmo(self, width: int, height: int) -> None:
        """A small oriented X/Y/Z triad in the bottom-right corner, tracking the current view rotation only (not zoom/pan)."""
        box = 78
        margin = 12
        centre_x = width - margin - box / 2.0
        centre_y = height - margin - box / 2.0
        self.canvas.create_rectangle(
            centre_x - box / 2.0, centre_y - box / 2.0, centre_x + box / 2.0, centre_y + box / 2.0,
            outline=self.grid, fill=self.background, width=1,
        )

        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        spin = math.radians(self.spin_deg)
        spin_cos, spin_sin = math.cos(spin), math.sin(spin)

        def gizmo_project(model_x: float, y: float, z: float) -> tuple[float, float, float]:
            horizontal = model_x * math.cos(yaw) + y * math.sin(yaw)
            view_depth = -model_x * math.sin(yaw) + y * math.cos(yaw)
            vertical = z * math.cos(pitch) - view_depth * math.sin(pitch)
            spun_horizontal = horizontal * spin_cos - vertical * spin_sin
            spun_vertical = horizontal * spin_sin + vertical * spin_cos
            return centre_x + spun_horizontal, centre_y - spun_vertical, view_depth

        axis_length = box * 0.32
        axes = (
            ("X", (axis_length, 0.0, 0.0), "#ff6b6b"),
            ("Y", (0.0, axis_length, 0.0), "#5fd97a"),
            ("Z", (0.0, 0.0, axis_length), "#5b9dff"),
        )
        entries = sorted(
            (gizmo_project(*vector) + (label, color) for label, vector, color in axes),
            key=lambda entry: entry[2],
        )
        for end_x, end_y, _depth, label, color in entries:
            self.canvas.create_line(centre_x, centre_y, end_x, end_y, fill=color, width=3)
            self.canvas.create_text(end_x, end_y, text=label, fill=color, font=("Segoe UI", 9, "bold"))

    def _projector(self, width: int, height: int, sections: tuple, extra_max_x: float | None = None) -> tuple:
        min_x = min(section.x_model_mm for section in sections)
        max_x = max(section.x_model_mm for section in sections)
        if extra_max_x is not None:
            # The axial trim-allowance stub (see _draw_axial_extension) can
            # extend past the last real section -- the camera must scale to
            # fit that too, or the stub would render past the frame edge.
            max_x = max(max_x, extra_max_x)
        span_x = max(max_x - min_x, 1.0)
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        spin = math.radians(self.spin_deg)
        spin_cos, spin_sin = math.cos(spin), math.sin(spin)
        scale = self.zoom * min(
            width * 0.70 / (span_x + self.stock_radius * 1.4), height * 0.72 / (self.stock_radius * 2.5)
        )
        centre_x = width * 0.50 + self.pan_x
        centre_y = height * 0.54 + self.pan_y

        def transform(model_x: float, y: float, z: float) -> tuple[float, float, float]:
            horizontal = model_x * math.cos(yaw) + y * math.sin(yaw)
            view_depth = -model_x * math.sin(yaw) + y * math.cos(yaw)
            vertical = z * math.cos(pitch) - view_depth * math.sin(pitch)
            # "Rotate" mode spins the whole rendered view about the axis
            # pointing into the screen -- a pure screen-space roll.
            spun_horizontal = horizontal * spin_cos - vertical * spin_sin
            spun_vertical = horizontal * spin_sin + vertical * spin_cos
            return (centre_x + spun_horizontal * scale, centre_y - spun_vertical * scale, view_depth)

        def project(model_x: float, y: float, z: float) -> tuple[float, float]:
            projected_x, projected_y, _ = transform(model_x, y, z)
            return projected_x, projected_y

        def depth(model_x: float, y: float, z: float) -> float:
            return transform(model_x, y, z)[2]

        return project, depth

    def _stroke_start_support(self) -> float:
        assert self.plan is not None
        if not self._pre_stroke_state:
            return self.stock_radius
        ring = self._pre_stroke_state[self.selected_station]
        if not ring:
            return self.stock_radius
        angle = math.radians(self.selected_rotation)
        return max(y * math.sin(angle) + z * math.cos(angle) for y, z in ring)

    def _stroke_start_anvil_support(self) -> float:
        """The material's actual current boundary on the anvil's (-normal)
        side -- see the identical rationale on ``ForgePreview``."""
        assert self.plan is not None
        if not self._pre_stroke_state:
            return self.stock_radius
        ring = self._pre_stroke_state[self.selected_station]
        if not ring:
            return self.stock_radius
        return anvil_side_support(ring, self.selected_rotation)

    def _draw_solid_profile_prism(
        self,
        project,
        section_x: float,
        x_half_width: float,
        normal_y: float,
        normal_z: float,
        tangent_y: float,
        tangent_z: float,
        front_profile,
        back_profile,
        color: str,
        stipple: str = "",
    ) -> None:
        """Render a finite extruded block between a front and back tangential profile.

        Draws the contact (front) face, its opposite (back) face, and the
        two axial end walls connecting them, so the die reads as a real
        solid block rather than an infinitely thin, unbounded sheet. Pass a
        canvas ``stipple`` pattern (e.g. ``"gray50"``) for a translucent look.
        """

        def vertex(x_offset: float, tangential_offset: float, depth: float) -> tuple[float, float, float]:
            return (
                section_x + x_offset,
                normal_y * depth + tangent_y * tangential_offset,
                normal_z * depth + tangent_z * tangential_offset,
            )

        def fill_quad(*corners: tuple[float, float, float]) -> None:
            coordinates = [value for corner in corners for value in project(*corner)]
            self.canvas.create_polygon(*coordinates, fill=color, outline="#fff0a3", width=1, stipple=stipple)

        for profile in (front_profile, back_profile):
            for index in range(len(profile) - 1):
                near_offset, near_depth = profile[index]
                far_offset, far_depth = profile[index + 1]
                fill_quad(
                    vertex(-x_half_width, near_offset, near_depth),
                    vertex(x_half_width, near_offset, near_depth),
                    vertex(x_half_width, far_offset, far_depth),
                    vertex(-x_half_width, far_offset, far_depth),
                )

        for x_offset in (-x_half_width, x_half_width):
            for index in range(len(front_profile) - 1):
                near_offset, near_depth = front_profile[index]
                far_offset, far_depth = front_profile[index + 1]
                back_near_offset, back_near_depth = back_profile[index]
                back_far_offset, back_far_depth = back_profile[index + 1]
                fill_quad(
                    vertex(x_offset, near_offset, near_depth),
                    vertex(x_offset, far_offset, far_depth),
                    vertex(x_offset, back_far_offset, back_far_depth),
                    vertex(x_offset, back_near_offset, back_near_depth),
                )

    def _draw_solid_disc(
        self,
        project,
        section_x: float,
        rim: Sequence[tuple[float, float]],
        depth_mm: float,
        thickness_mm: float,
        normal_y: float,
        normal_z: float,
        tangent_y: float,
        tangent_z: float,
        color: str,
        stipple: str = "",
    ) -> None:
        """Render a flat-faced puck: a front face at ``depth_mm``, a back
        face ``thickness_mm`` further along the same normal direction, and a
        rim connecting them, tracing the ``(axial_offset, tangential_offset)``
        perimeter points in ``rim``.

        The caller passes the die's real, full-size footprint here (see
        :func:`lcaf.toolpathing.visualization.disc_rim_profile`) -- the
        rendered tool is always the true, physical shape of the die, never
        flattened or shrunk to reflect how far this one strike's rigid
        contact mechanics happen to trust it for.
        """

        def vertex(x_offset: float, tangential_offset: float, depth: float) -> tuple[float, float, float]:
            return (
                section_x + x_offset,
                normal_y * depth + tangent_y * tangential_offset,
                normal_z * depth + tangent_z * tangential_offset,
            )

        def fill_polygon(points: Sequence[tuple[float, float, float]]) -> None:
            coordinates = [value for point in points for value in project(*point)]
            self.canvas.create_polygon(*coordinates, fill=color, outline="#fff0a3", width=1, stipple=stipple)

        back_depth = depth_mm + thickness_mm

        fill_polygon([vertex(x_offset, tangential_offset, depth_mm) for x_offset, tangential_offset in rim])
        fill_polygon(
            [vertex(x_offset, tangential_offset, back_depth) for x_offset, tangential_offset in reversed(rim)]
        )
        sides = len(rim)
        for index in range(sides):
            next_index = (index + 1) % sides
            x0, t0 = rim[index]
            x1, t1 = rim[next_index]
            fill_polygon(
                [
                    vertex(x0, t0, depth_mm),
                    vertex(x1, t1, depth_mm),
                    vertex(x1, t1, back_depth),
                    vertex(x0, t0, back_depth),
                ]
            )

    def _draw_lower_die(self, project) -> None:
        """The lower die (the one with prescribed geometry): the only die that animates.

        It does not actually move in real life -- the billet's X/Y/rotation
        and the upper die's Z do -- but this view always shows only the
        lower die's depth animating through a stroke, since it is the one
        whose geometry the UI actually prescribes.
        """
        assert self.plan is not None
        operation = self.plan.operations[self.operation_index]
        metadata = operation["metadata"]
        section = self.plan.sections[self.selected_station]
        end_support = self.stock_radius - float(metadata["radial_reduction_mm"])
        start_support = self._stroke_start_support()
        # The die's depth animates from wherever the material actually is
        # right now, not always the pristine stock radius -- a later pass on
        # an already-reduced station must not appear to retract through
        # material that is no longer there.
        support = start_support + (end_support - start_support) * self.motion_progress
        angle = math.radians(self.selected_rotation)
        normal_y, normal_z = math.sin(angle), math.cos(angle)
        tangent_y, tangent_z = normal_z, -normal_y

        die_width = metadata.get("die_width_mm")
        corner_radius = float(metadata.get("die_corner_radius_mm", 0.0) or 0.0)
        die_length = metadata.get("die_length_mm")
        axial_half_length = die_length / 2.0 if die_length is not None else max(0.75, self.stock_radius * 0.3)
        contact_profile = die_contact_profile(die_width, corner_radius, support, samples=10)
        if contact_profile is None:
            plate_half_width = self.stock_radius * 1.8
            contact_profile = ((-plate_half_width, support), (plate_half_width, support))
        # A generous but finite depth so the block reads as solid, not an
        # infinitely thin ribbon.
        block_depth = max(self.stock_radius * 1.4, 4.0)
        back_profile = tuple((offset, depth + block_depth) for offset, depth in contact_profile)
        self._draw_solid_profile_prism(
            project, section.x_model_mm, axial_half_length, normal_y, normal_z, tangent_y, tangent_z,
            contact_profile, back_profile, self.die,
        )

    def _draw_upper_die(self, project) -> None:
        """The upper die: never moves in this view, but must be drawn as a
        real, sized circular disc actually touching wherever the material
        currently sits on its side.

        In real life it is the die that actually travels in Z, but it is
        rendered here as a fixed, translucent plate always on the opposite
        side of the billet from the lower die. Its rendered depth tracks the
        material's actual current boundary there -- not a fixed pristine
        stock-radius position, which can drift out of contact ("levitate")
        once an earlier rotation has already reduced that side -- so it
        never appears to float clear of the billet.
        """
        assert self.plan is not None
        operation = self.plan.operations[self.operation_index]
        metadata = operation["metadata"]
        upper_die_radius = float(metadata.get("upper_die_radius_mm") or self.stock_radius)
        section = self.plan.sections[self.selected_station]
        angle = math.radians(self.selected_rotation)
        normal_y, normal_z = math.sin(angle), math.cos(angle)
        tangent_y, tangent_z = normal_z, -normal_y

        anvil_support = self._stroke_start_anvil_support()
        plate_thickness = max(self.stock_radius * 0.35, 1.0)
        # The upper die is a real, physical circular disc: it always renders
        # as its full true circle at upper_die_radius_mm, never flattened or
        # shrunk. The contact *mechanics* still confine this strike's own
        # rigid effect to the one segment it targets (see
        # visualization._apply_strike_3d's disc_axial_half_length) -- that is
        # a computational-geometry modelling choice about how far this one
        # strike's commanded depth is trusted to apply, not a claim that the
        # tool itself is somehow physically smaller or clipped, so the
        # rendered tool is decoupled from it and always shown at full size.
        rim = disc_rim_profile(upper_die_radius, upper_die_radius, sides=24)
        self._draw_solid_disc(
            project, section.x_model_mm, rim,
            -anvil_support, -plate_thickness, normal_y, normal_z, tangent_y, tangent_z,
            self.upper_die, stipple="gray50",
        )


class _SeriesPlot(tk.Canvas):
    """A minimal line-plot of one numeric series, with a marker for the
    current step. Shared by the force and contact-pressure plots below so
    neither duplicates the other's drawing code."""

    def __init__(self, master: tk.Widget, *, background: str, grid: str, text: str, line: str, marker: str, unit: str, empty_message: str, height: int = 160) -> None:
        super().__init__(master, bg=background, highlightthickness=0, height=height)
        self._grid, self._text, self._line, self._marker = grid, text, line, marker
        self._unit = unit
        self._empty_message = empty_message
        self.values: tuple[float, ...] = ()
        self.current_index = 0
        self.bind("<Configure>", lambda _event: self.redraw())

    def set_values(self, values: tuple[float, ...], current_index: int) -> None:
        self.values = values
        self.current_index = current_index
        self.redraw()

    def set_current_index(self, current_index: int) -> None:
        self.current_index = current_index
        self.redraw()

    def redraw(self) -> None:
        self.delete("all")
        width = max(self.winfo_width(), 1)
        height = max(self.winfo_height(), 1)
        if not self.values:
            self.create_text(width / 2, height / 2, text=self._empty_message, fill=self._text, font=("Segoe UI", 10))
            return

        margin = 30
        plot_width = max(width - 2 * margin, 1)
        plot_height = max(height - 2 * margin, 1)
        peak = max(self.values) if max(self.values) > 0 else 1.0
        count = len(self.values)

        for fraction in (0.0, 0.5, 1.0):
            y = margin + plot_height * (1.0 - fraction)
            self.create_line(margin, y, margin + plot_width, y, fill=self._grid)
            self.create_text(
                margin - 4, y, text=f"{peak * fraction:.0f} {self._unit}", fill=self._text, anchor="e", font=("Segoe UI", 8),
            )

        def point(index: int, value: float) -> tuple[float, float]:
            x = margin + (plot_width * index / (count - 1) if count > 1 else 0.0)
            y = margin + plot_height * (1.0 - value / peak)
            return x, y

        coordinates = [coordinate for index, value in enumerate(self.values) for coordinate in point(index, value)]
        if count > 1:
            self.create_line(*coordinates, fill=self._line, width=2)
        current_index = max(0, min(self.current_index, count - 1))
        marker_x, marker_y = point(current_index, self.values[current_index])
        self.create_oval(marker_x - 4, marker_y - 4, marker_x + 4, marker_y + 4, fill=self._marker, outline="")
        self.create_line(marker_x, margin, marker_x, margin + plot_height, fill=self._marker, dash=(3, 3))


class ForceReadout(ttk.Frame):
    """A read-only tab reporting the separate, independent forging force and
    die contact pressure (stress) estimates.

    Neither feeds the deformation preview or the plan above it -- both are a
    standalone slab-method calculation (see ``lcaf.toolpathing.material``)
    from the same material/temperature/die geometry, computed purely for
    process-planning information, assuming rigid, indestructible dies able
    to supply whatever force/pressure it reports. Force alone does not say
    how hard the die presses -- the same force spread over a larger contact
    patch needs far less stress to induce plastic flow than the same force
    concentrated over a small one, so both are shown.
    """

    background = "#101827"
    grid = "#26344a"
    text = "#d9e2f2"
    line = "#ffb454"
    stress_line = "#7ec8ff"
    marker = "#4fc3a1"

    def __init__(self, master: tk.Widget) -> None:
        super().__init__(master, padding=10)
        self.report: tuple[dict, ...] = ()
        self.current_index = 0

        header = ttk.Label(
            self,
            text=(
                "Estimated forging force and die contact pressure per strike -- a separate "
                "slab-method (friction-hill) hand-calculation from the chosen material/"
                "temperature and die contact geometry, not a simulation. The contact pressure "
                "is the stress the die must actually apply to induce plastic flow in the "
                "material at that geometry -- force alone does not capture this, since the same "
                "force concentrated over a small contact patch needs far more stress than the "
                "same force spread over a large one. Neither affects the deformation preview or "
                "the planned coordinates above: dies are treated as rigid and able to supply "
                "whatever force/pressure is shown here."
            ),
            foreground="#9fb0c9",
            wraplength=1000,
        )
        header.pack(fill="x", pady=(0, 8))

        summary = ttk.Frame(self)
        summary.pack(fill="x", pady=(0, 8))
        self.current_force_text = tk.StringVar(value="Current step: —")
        self.peak_force_text = tk.StringVar(value="Peak force across plan: —")
        self.current_stress_text = tk.StringVar(value="Contact pressure: —")
        self.peak_stress_text = tk.StringVar(value="Peak pressure across plan: —")
        force_row = ttk.Frame(summary)
        force_row.pack(fill="x")
        ttk.Label(force_row, textvariable=self.current_force_text, font=("Segoe UI", 12, "bold")).pack(side="left")
        ttk.Label(force_row, textvariable=self.peak_force_text, foreground="#9fb0c9").pack(side="left", padx=(24, 0))
        stress_row = ttk.Frame(summary)
        stress_row.pack(fill="x", pady=(4, 0))
        ttk.Label(stress_row, textvariable=self.current_stress_text, font=("Segoe UI", 12, "bold")).pack(side="left")
        ttk.Label(stress_row, textvariable=self.peak_stress_text, foreground="#9fb0c9").pack(side="left", padx=(24, 0))

        ttk.Label(self, text="Force per strike (kN)", foreground=self.text).pack(anchor="w", pady=(4, 0))
        self.force_plot = _SeriesPlot(
            self, background=self.background, grid=self.grid, text=self.text, line=self.line,
            marker=self.marker, unit="kN", empty_message="Generate a preview to see the force estimate.",
        )
        self.force_plot.pack(fill="both", expand=True)

        ttk.Label(self, text="Die contact pressure needed for plastic flow (MPa)", foreground=self.text).pack(anchor="w", pady=(8, 0))
        self.stress_plot = _SeriesPlot(
            self, background=self.background, grid=self.grid, text=self.text, line=self.stress_line,
            marker=self.marker, unit="MPa", empty_message="Generate a preview to see the contact-pressure estimate.",
        )
        self.stress_plot.pack(fill="both", expand=True)

    def show_plan(self, plan: ToolpathPlan, _settings: SliceSettings) -> None:
        self.report = plan_force_report(plan.operations) if plan.operations else ()
        self.current_index = 0
        forces = tuple(row["estimated_force_kn"] for row in self.report)
        pressures = tuple(row["estimated_contact_pressure_mpa"] for row in self.report)
        self.peak_force_text.set(f"Peak force across plan: {max(forces, default=0.0):.1f} kN")
        self.peak_stress_text.set(f"Peak pressure across plan: {max(pressures, default=0.0):.1f} MPa")
        self.force_plot.set_values(forces, self.current_index)
        self.stress_plot.set_values(pressures, self.current_index)

    def show_operation(self, operation_index: int, _progress: float) -> None:
        if not self.report:
            self.current_force_text.set("Current step: —")
            self.current_stress_text.set("Contact pressure: —")
            return
        self.current_index = max(0, min(operation_index, len(self.report) - 1))
        row = self.report[self.current_index]
        self.current_force_text.set(
            f"Current step: {row['estimated_force_kn']:.1f} kN "
            f"({row['material']} at {row['target_temperature_c']:.0f} C)"
        )
        self.current_stress_text.set(f"Contact pressure: {row['estimated_contact_pressure_mpa']:.1f} MPa")
        self.force_plot.set_current_index(self.current_index)
        self.stress_plot.set_current_index(self.current_index)


class ToolpathApp(tk.Tk):
    """Interactive parameters, preview, and JSONL export."""

    DIE_SHAPE_RECTANGULAR = "Full rectangular (sharp edge)"
    DIE_SHAPE_RADIUSED = "Radiused edge"

    def __init__(self) -> None:
        super().__init__()
        self.title("LCAF Toolpath Slicer")

        self.geometry("1200x800")
        self.minsize(900, 600)
        self.configure(bg="#101827")
        self.plan: ToolpathPlan | None = None
        self.settings: SliceSettings | None = None
        self.limits = self._default_limits()
        self.example_paths = self._example_paths()
        self.playing = False
        self.playback_job: str | None = None
        self.last_playback_tick: float | None = None
        self.operation_index = 0
        self.operation_progress = 0.0

        self.model_path = tk.StringVar()
        self.stock_radius = tk.StringVar(value="20")
        self.radial_segments = tk.StringVar(value="4")
        self.strikes_per_segment = tk.StringVar(value="4")
        self.cycles = tk.StringVar(value="1")
        self.auto_cycles = tk.BooleanVar(value=False)
        self.max_reduction = tk.StringVar(value="2")
        self.die_contact_z = tk.StringVar(value="0")
        self.x_offset = tk.StringVar(value="0")
        self.y_position = tk.StringVar(value="0")
        self.scale = tk.StringVar(value="1")
        self.axis = tk.StringVar(value="auto")
        self.clamped_end = tk.StringVar(value="min")
        self.temperature = tk.StringVar(value="0")
        self.material = tk.StringVar(value="steel")
        self.material_label = tk.StringVar(value=MATERIAL_LABELS["steel"])
        self.die_shape = tk.StringVar(value=self.DIE_SHAPE_RECTANGULAR)
        self.die_length = tk.StringVar(value="")
        self.die_width = tk.StringVar(value="")
        self.die_corner_radius = tk.StringVar(value="0")
        self.upper_die_radius = tk.StringVar(value="")
        self.example_target = tk.StringVar(value="")
        self.playback_speed = tk.DoubleVar(value=1.0)
        self.mesh_quality = tk.IntVar(value=48)
        self.mesh_quality_label = tk.StringVar(value="48 segments")
        self.show_dies = tk.BooleanVar(value=True)
        self.step_counter_text = tk.StringVar(value="Step — of —")
        self.playback_caption = tk.StringVar(value="Generate a preview to enable toolpath playback.")
        self.status = tk.StringVar(value="Planning only — no machine connection or motion commands.")
        self.stock_length_info = tk.StringVar(
            value="Recommended stock length appears here after Generate preview."
        )

        self._build_layout()

    @staticmethod
    def _default_limits() -> MachineLimits:
        root = Path(__file__).resolve().parents[3]
        config = root / "configs" / "axis.json"
        try:
            return MachineLimits.from_lcaf_config(config)
        except ToolpathPlanningError:
            return MachineLimits()

    @staticmethod
    def _example_paths() -> dict[str, Path]:
        examples = Path(__file__).resolve().parents[3] / "examples"
        names = (
            ("Square bar — 10 mm", "square_bar.obj"),
            ("Hex bar — 14 mm across corners", "hex_bar.obj"),
            ("Tapered square bar — 8 → 16 → 8 mm", "tapered_square_bar.obj"),
            ("Tapered hex bar — R5 → R8 → R5 mm", "tapered_hex_bar.obj"),
        )
        return {label: examples / filename for label, filename in names if (examples / filename).exists()}

    def _build_layout(self) -> None:
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TFrame", background="#101827")
        style.configure("TLabel", background="#101827", foreground="#d9e2f2")
        style.configure("TLabelframe", background="#101827", foreground="#d9e2f2")
        style.configure("TLabelframe.Label", background="#101827", foreground="#d9e2f2", font=("Segoe UI", 10, "bold"))
        style.configure("TButton", padding=(10, 5))
        style.configure("TEntry", fieldbackground="#1a2638", foreground="#edf3ff")
        style.configure("TCombobox", fieldbackground="#1a2638", foreground="#edf3ff")

        container = ttk.Frame(self)
        container.pack(fill="both", expand=True)

        canvas = tk.Canvas(
            container,
            bg="#101827",
            highlightthickness=0,
        )

        scrollbar = ttk.Scrollbar(
            container,
            orient="vertical",
            command=canvas.yview,
        )

        canvas.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        shell = ttk.Frame(canvas, padding=14)

        window = canvas.create_window(
            (0, 0),
            window=shell,
            anchor="nw",
        )

        def _configure_scrollregion(event):
            canvas.configure(scrollregion=canvas.bbox("all"))

        shell.bind("<Configure>", _configure_scrollregion)

        def _resize_canvas(event):
            canvas.itemconfigure(window, width=event.width)

        canvas.bind("<Configure>", _resize_canvas)

        def _mousewheel(event):
            canvas.yview_scroll(int(-event.delta / 120), "units")

        canvas.bind_all("<MouseWheel>", _mousewheel)

        header = ttk.Frame(shell)
        header.pack(fill="x", pady=(0, 10))
        ttk.Label(header, text="LCAF toolpath slicer", font=("Segoe UI", 18, "bold")).pack(anchor="w")
        ttk.Label(header, text="OBJ/STL → conservative convex-envelope forge strikes → ForgeBrain JSONL", foreground="#9fb0c9").pack(anchor="w")

        model_box = ttk.LabelFrame(shell, text="1. Target mesh", padding=10)
        model_box.pack(fill="x", pady=(0, 10))
        ttk.Entry(model_box, textvariable=self.model_path).pack(side="left", fill="x", expand=True, padx=(0, 8))
        ttk.Button(model_box, text="Choose OBJ / STL", command=self._choose_model).pack(side="left")
        ttk.Label(model_box, text="Example target").pack(side="left", padx=(14, 5))
        self.example_picker = ttk.Combobox(
            model_box,
            textvariable=self.example_target,
            values=tuple(self.example_paths),
            state="readonly",
            width=34,
        )
        self.example_picker.pack(side="left")
        self.example_picker.bind("<<ComboboxSelected>>", self._choose_example)

        parameters = ttk.LabelFrame(shell, text="2. Geometry and strike resolution (millimetres unless stated)", padding=10)
        parameters.pack(fill="x", pady=(0, 10))
        ttk.Label(
            parameters,
            text=(
                "The part's length is divided into this many radial segments; each segment's "
                "strike depth is the numerical average of the target across that whole "
                "segment, not a single point. Each segment is struck this many times, evenly "
                "spaced around the full 360°. Cycles repeat the entire segment x strike sweep "
                "end-to-end -- later cycles true up whatever an earlier, rougher cycle left "
                "rough, the same way a real finishing pass would."
            ),
            foreground="#9fb0c9",
            wraplength=1040,
        ).grid(row=0, column=0, columnspan=3, sticky="w", padx=6, pady=(0, 6))
        entries = (
            ("Stock radius", self.stock_radius),
            ("Radial segments", self.radial_segments),
            ("Strikes per segment", self.strikes_per_segment),
            ("Max reduction / strike", self.max_reduction),
            ("Die contact Z", self.die_contact_z),
            ("Model scale", self.scale),
            ("Target temperature (°C)", self.temperature),
            ("X offset from clamp (mm)", self.x_offset),
            ("Y tool position", self.y_position),
            ("Cycles", self.cycles),
        )
        for index, (label, variable) in enumerate(entries):
            row, column = divmod(index, 3)
            field = ttk.Frame(parameters)
            field.grid(row=row + 1, column=column, sticky="ew", padx=6, pady=4)
            ttk.Label(field, text=label).pack(anchor="w")
            entry = ttk.Entry(field, textvariable=variable, width=18)
            entry.pack(fill="x")
            if variable is self.cycles:
                self.cycles_entry = entry
            parameters.columnconfigure(column, weight=1)
        auto_cycles_field = ttk.Frame(parameters)
        auto_cycles_field.grid(row=4, column=1, sticky="ew", padx=6, pady=4)
        ttk.Checkbutton(
            auto_cycles_field,
            text="Complete necessary cycles automatically",
            variable=self.auto_cycles,
            command=self._on_auto_cycles_changed,
        ).pack(anchor="w", pady=(16, 0))
        axis_field = ttk.Frame(parameters)
        axis_field.grid(row=5, column=0, sticky="ew", padx=6, pady=4)
        ttk.Label(axis_field, text="Billet longitudinal axis").pack(anchor="w")
        ttk.Combobox(axis_field, textvariable=self.axis, values=("auto", "x", "y", "z"), state="readonly", width=15).pack(fill="x")
        clamp_field = ttk.Frame(parameters)
        clamp_field.grid(row=5, column=1, sticky="ew", padx=6, pady=4)
        ttk.Label(clamp_field, text="Clamped end (machine +X points away from this)").pack(anchor="w")
        ttk.Combobox(
            clamp_field,
            textvariable=self.clamped_end,
            values=("min", "max"),
            state="readonly",
            width=15,
        ).pack(fill="x")
        material_field = ttk.Frame(parameters)
        material_field.grid(row=5, column=2, sticky="ew", padx=6, pady=4)
        ttk.Label(material_field, text="Billet material").pack(anchor="w")
        self.material_picker = ttk.Combobox(
            material_field,
            textvariable=self.material_label,
            values=[MATERIAL_LABELS[key] for key in MATERIALS],
            state="readonly",
            width=32,
        )
        self.material_picker.pack(fill="x")
        self.material_picker.bind("<<ComboboxSelected>>", self._on_material_changed)
        self.material_info = tk.StringVar(value="")
        self._on_material_changed()
        ttk.Label(
            parameters,
            text=(
                "Material and target temperature together drive how convincingly the preview's "
                "deformation bulges/settles (cold, stiff material spreads less and needs more "
                "strikes/cycles to fully converge) and the separate force estimate on its own "
                "tab -- never the planned strike coordinates themselves."
            ),
            foreground="#9fb0c9",
            wraplength=1040,
        ).grid(row=6, column=0, columnspan=2, sticky="w", padx=6, pady=(2, 4))
        ttk.Label(
            parameters, textvariable=self.material_info, foreground="#9fb0c9",
        ).grid(row=6, column=2, sticky="w", padx=6, pady=(2, 4))
        ttk.Label(
            parameters,
            textvariable=self.stock_length_info,
            foreground="#9fb0c9",
            wraplength=700,
        ).grid(row=7, column=0, columnspan=3, sticky="w", padx=6, pady=4)

        die_box = ttk.LabelFrame(
            shell, text="3. Die geometry (lower die: rectangle · upper die: circular disc)", padding=10
        )
        die_box.pack(fill="x", pady=(0, 10))
        ttk.Label(
            die_box,
            text=(
                "Two rigid, finite contact faces act on every strike: the lower die (a "
                "rectangular prism you shape below) and the upper die (a flat circular disc "
                "of the radius below). Leave any field blank for a sensible physical default "
                "sized from the stock and station spacing -- not an unconstrained/infinite die -- "
                "so displaced material bulges sideways by default instead of vanishing. An "
                "upper die radius smaller than the stock radius is a deliberate, honest trade-off: "
                "like a real round punch smaller than a face, it can leave part of that face unstruck."
            ),
            foreground="#9fb0c9",
            wraplength=1040,
        ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))

        shape_field = ttk.Frame(die_box)
        shape_field.grid(row=1, column=0, sticky="ew", padx=6, pady=4)
        ttk.Label(shape_field, text="Die face shape").pack(anchor="w")
        self.die_shape_picker = ttk.Combobox(
            shape_field,
            textvariable=self.die_shape,
            values=(self.DIE_SHAPE_RECTANGULAR, self.DIE_SHAPE_RADIUSED),
            state="readonly",
            width=22,
        )
        self.die_shape_picker.pack(fill="x")
        self.die_shape_picker.bind("<<ComboboxSelected>>", self._on_die_shape_changed)

        die_entries = (
            ("Contact length, X (mm)", self.die_length),
            ("Contact width, Y (mm)", self.die_width),
        )
        for index, (label, variable) in enumerate(die_entries, start=1):
            field = ttk.Frame(die_box)
            field.grid(row=1, column=index, sticky="ew", padx=6, pady=4)
            ttk.Label(field, text=label).pack(anchor="w")
            ttk.Entry(field, textvariable=variable, width=18).pack(fill="x")
        for column in range(3):
            die_box.columnconfigure(column, weight=1)

        radius_field = ttk.Frame(die_box)
        radius_field.grid(row=2, column=0, sticky="ew", padx=6, pady=4)
        ttk.Label(radius_field, text="Corner radius (mm)").pack(anchor="w")
        self.die_radius_entry = ttk.Entry(radius_field, textvariable=self.die_corner_radius, width=18)
        self.die_radius_entry.pack(fill="x")
        ttk.Label(
            die_box,
            text=(
                "Radiused edge: the contact face is flat in the middle and blends into this "
                "corner radius toward each Y edge, easing into the surrounding material instead "
                "of leaving a sharp die line."
            ),
            foreground="#9fb0c9",
            wraplength=1040,
        ).grid(row=2, column=1, columnspan=2, sticky="w", padx=6, pady=4)

        upper_die_field = ttk.Frame(die_box)
        upper_die_field.grid(row=3, column=0, sticky="ew", padx=6, pady=4)
        ttk.Label(upper_die_field, text="Upper die radius, Z (mm)").pack(anchor="w")
        ttk.Entry(upper_die_field, textvariable=self.upper_die_radius, width=18).pack(fill="x")
        ttk.Label(
            die_box,
            text=(
                "The upper (striking) die's flat-faced circular contact radius. Blank keeps it "
                "large enough to always fully cover the target at the station it strikes."
            ),
            foreground="#9fb0c9",
            wraplength=1040,
        ).grid(row=3, column=1, columnspan=2, sticky="w", padx=6, pady=4)
        self._on_die_shape_changed()

        actions = ttk.Frame(shell)
        actions.pack(fill="x", pady=(0, 10))
        ttk.Button(actions, text="Generate preview", command=self._generate).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Export JSONL", command=self._export).pack(side="left")
        ttk.Label(actions, textvariable=self.status, foreground="#9fb0c9").pack(side="left", padx=18)

        playback = ttk.LabelFrame(shell, text="4. Animated toolpath preview", padding=8)
        playback.pack(fill="x", pady=(0, 10))
        playback_row = ttk.Frame(playback)
        playback_row.pack(fill="x")
        self.back_button = ttk.Button(playback_row, text="◀ Step", command=lambda: self._step_playback(-1), state="disabled")
        self.back_button.pack(side="left")
        self.play_button = ttk.Button(playback_row, text="Play", command=self._toggle_playback, state="disabled")
        self.play_button.pack(side="left", padx=6)
        self.forward_button = ttk.Button(playback_row, text="Step ▶", command=lambda: self._step_playback(1), state="disabled")
        self.forward_button.pack(side="left")
        self.restart_button = ttk.Button(playback_row, text="Restart", command=self._restart_playback, state="disabled")
        self.restart_button.pack(side="left", padx=6)
        ttk.Label(playback_row, text="Speed").pack(side="left", padx=(14, 4))
        self.speed_slider = ttk.Scale(playback_row, from_=0.5, to=4.0, variable=self.playback_speed, orient="horizontal", length=105)
        self.speed_slider.pack(side="left")
        ttk.Label(playback_row, text="0.5×–4×").pack(side="left", padx=(4, 14))
        ttk.Label(
            playback_row, textvariable=self.step_counter_text, foreground="#edf3ff", font=("Segoe UI", 11, "bold")
        ).pack(side="left", padx=(0, 14))
        ttk.Label(playback_row, textvariable=self.playback_caption, foreground="#9fb0c9").pack(side="left")

        quality_row = ttk.Frame(playback)
        quality_row.pack(fill="x", pady=(6, 0))
        ttk.Label(quality_row, text="Mesh quality (deformed-preview smoothness)").pack(side="left", padx=(0, 6))
        self.quality_slider = ttk.Scale(
            quality_row, from_=16, to=128, variable=self.mesh_quality, orient="horizontal", length=160,
            command=self._on_mesh_quality_changed,
        )
        self.quality_slider.pack(side="left")
        ttk.Label(quality_row, textvariable=self.mesh_quality_label, foreground="#9fb0c9").pack(side="left", padx=(6, 0))
        ttk.Checkbutton(
            quality_row, text="Show dies", variable=self.show_dies, command=self._on_show_dies_changed,
        ).pack(side="left", padx=(18, 0))

        self.view_notebook = ttk.Notebook(shell)
        self.view_notebook.pack(fill="both", expand=True)
        self.preview = ForgePreview(self.view_notebook)
        self.view_notebook.add(self.preview, text="2D cross-section + axial plan")
        self.preview_3d = Forge3DPreview(self.view_notebook)
        self.view_notebook.add(self.preview_3d, text="3D view (rotate / roll / pan)")
        self.force_readout = ForceReadout(self.view_notebook)
        self.view_notebook.add(self.force_readout, text="Force estimate")
        self.view_notebook.bind("<<NotebookTabChanged>>", self._on_view_tab_changed)

        note = ttk.Label(
            shell,
            text=(
                "Safety: output contains calibrated axis targets, not a verified forming simulation. "
                "Review, prove off-material, and use ForgeBrain/LinuxCNC safety controls before any motion."
            ),
            foreground="#ffcc7a",
            wraplength=1000,
        )
        note.pack(fill="x", pady=(10, 0))

    def _choose_model(self) -> None:
        filename = filedialog.askopenfilename(
            title="Choose target mesh",
            filetypes=(
                ("Triangle mesh", "*.stl *.obj"),
                ("STL", "*.stl"),
                ("OBJ", "*.obj"),
                ("SolidWorks part (export to STL first)", "*.sldprt"),
                ("All files", "*.*"),
            ),
        )
        if filename:
            self.model_path.set(filename)
            self.example_target.set("")

    def _choose_example(self, _event: tk.Event | None = None) -> None:
        path = self.example_paths.get(self.example_target.get())
        if path is None:
            return
        self.model_path.set(str(path))
        # The tapered square bar's widest corner reaches ~11.3 mm even though
        # its own faces only need ~8 mm -- a 10 mm stock radius would reject
        # it (the diagonal corner falls between the 0/90/180/270 strikes).
        self.stock_radius.set("12" if "Tapered square" in self.example_target.get() else "10")
        self.radial_segments.set("6" if "Tapered" in self.example_target.get() or "Hex" in self.example_target.get() else "4")
        self.strikes_per_segment.set("4")
        self.cycles.set("1")
        self.max_reduction.set("2")
        self.die_contact_z.set("10")
        self.axis.set("x")
        self.status.set("Example loaded. Generate a preview, then use Play to watch the die-envelope animation.")

    def _on_material_changed(self, _event: tk.Event | None = None) -> None:
        label = self.material_label.get()
        key = next((candidate for candidate in MATERIALS if MATERIAL_LABELS[candidate] == label), "steel")
        self.material.set(key)
        self.material_info.set(_material_temperature_hint(key))

    def _on_die_shape_changed(self, _event: tk.Event | None = None) -> None:
        radiused = self.die_shape.get() == self.DIE_SHAPE_RADIUSED
        self.die_radius_entry.configure(state="normal" if radiused else "disabled")
        if not radiused:
            self.die_corner_radius.set("0")

    def _on_auto_cycles_changed(self) -> None:
        self.cycles_entry.configure(state="disabled" if self.auto_cycles.get() else "normal")

    def _on_view_tab_changed(self, _event: tk.Event | None = None) -> None:
        if self.view_notebook.select() == str(self.preview_3d):
            self.preview_3d.show_operation(self.operation_index, self.operation_progress)

    def _on_mesh_quality_changed(self, _value: str | None = None) -> None:
        segments = int(round(self.mesh_quality.get()))
        self.mesh_quality_label.set(f"{segments} segments")
        self.preview.set_mesh_quality(segments)
        self.preview_3d.set_mesh_quality(segments)

    def _on_show_dies_changed(self) -> None:
        show = self.show_dies.get()
        self.preview.set_show_dies(show)
        self.preview_3d.set_show_dies(show)

    @staticmethod
    def _parse_optional_mm(value: str) -> float | None:
        text = value.strip()
        return float(text) if text else None

    def _current_settings(self) -> SliceSettings:
        try:
            return SliceSettings(
                stock_radius_mm=float(self.stock_radius.get()),
                radial_segments=int(self.radial_segments.get()),
                strikes_per_segment=int(self.strikes_per_segment.get()),
                cycles=int(self.cycles.get()),
                max_reduction_per_strike_mm=float(self.max_reduction.get()),
                die_contact_z_mm=float(self.die_contact_z.get()),
                x_offset_mm=float(self.x_offset.get()),
                y_position_mm=float(self.y_position.get()),
                target_temperature_c=float(self.temperature.get()),
                material=self.material.get(),
                scale_mm_per_unit=float(self.scale.get()),
                longitudinal_axis=self.axis.get(),
                stock_clamped_end=self.clamped_end.get(),
                die_width_mm=self._parse_optional_mm(self.die_width.get()),
                die_length_mm=self._parse_optional_mm(self.die_length.get()),
                die_corner_radius_mm=float(self.die_corner_radius.get().strip() or "0"),
                upper_die_radius_mm=self._parse_optional_mm(self.upper_die_radius.get()),
            )
        except ValueError as error:
            raise ToolpathPlanningError("All numeric fields must contain valid numbers.") from error

    def _generate(self) -> None:
        self._stop_playback()
        try:
            filename = self.model_path.get().strip()
            if not filename:
                raise ToolpathPlanningError("Choose an OBJ or STL target mesh first.")
            self.settings = self._current_settings()
            mesh = load_mesh(filename)
            if self.auto_cycles.get():
                self.plan = find_sufficient_cycles(mesh, self.settings, self.limits)
                # find_sufficient_cycles ignores settings.cycles and searches its
                # own -- reflect back whatever it actually converged at.
                self.settings = dataclasses.replace(
                    self.settings, cycles=self.plan.operations[-1]["metadata"]["cycle_index"] + 1
                )
            else:
                self.plan = ToolpathSlicer(mesh, self.settings, self.limits).plan()
        except ToolpathPlanningError as error:
            self.plan = None
            self._set_playback_enabled(False)
            self.status.set("No JSONL generated — fix the validation issue.")
            messagebox.showerror("Cannot generate toolpath", str(error), parent=self)
            return

        self.preview.show_plan(self.plan, self.settings)
        self.preview_3d.show_plan(self.plan, self.settings)
        self.force_readout.show_plan(self.plan, self.settings)
        self.operation_index = 0
        self.operation_progress = 0.0
        self._set_playback_operation(0, 0.0)
        self._set_playback_enabled(True)
        cycles_note = f" · converged at {self.settings.cycles} cycle(s)" if self.auto_cycles.get() else ""
        self.status.set(
            f"Preview ready: {len(self.plan.sections)} radial segments, "
            f"{len(self.plan.rotations_deg)} strikes/segment, {len(self.plan.operations)} strike operations{cycles_note}."
        )
        final_trim_allowance_mm = axial_trim_allowance_mm(self.plan, len(self.plan.operations) - 1, 1.0)
        trim_note = (
            f" This stock length still leaves ~{final_trim_allowance_mm:,.1f} mm to saw off the free end once "
            "forging is complete (volume conservation pushes whatever the local bulge cannot reabsorb out that "
            "end -- see the amber stub in the 2D/3D previews)."
            if final_trim_allowance_mm > 0.05
            else " Forging conserves volume exactly, with no leftover length to trim at this stock length."
        )
        self.stock_length_info.set(
            f"Target volume ~{self.plan.target_volume_mm3:,.0f} mm³ → at this stock radius, cut stock "
            f"~{self.plan.recommended_stock_length_mm:,.1f} mm long to match it (no volume to create or discard)."
            f"{trim_note}"
        )

    def _set_playback_enabled(self, enabled: bool) -> None:
        state = "normal" if enabled else "disabled"
        for control in (self.back_button, self.play_button, self.forward_button, self.restart_button, self.speed_slider):
            control.configure(state=state)
        if not enabled:
            self.play_button.configure(text="Play")

    def _set_playback_operation(self, index: int, progress: float) -> None:
        if self.plan is None or not self.plan.operations:
            return
        self.operation_index = max(0, min(index, len(self.plan.operations) - 1))
        self.operation_progress = max(0.0, min(progress, 1.0))
        self.preview.show_operation(self.operation_index, self.operation_progress)
        self.force_readout.show_operation(self.operation_index, self.operation_progress)
        self._on_view_tab_changed()
        operation = self.plan.operations[self.operation_index]
        metadata = operation["metadata"]
        state = "complete" if self.operation_progress >= 1.0 else f"in-stroke {self.operation_progress * 100:.0f}%"
        self.step_counter_text.set(f"Step {self.operation_index + 1} of {len(self.plan.operations)}")
        self.playback_caption.set(
            f"A {float(operation['rotation']):.0f}° · X {float(operation['x']):.2f} mm · "
            f"pass {metadata['strike_pass']}/{metadata['strike_pass_count']} · {state}"
        )

    def _toggle_playback(self) -> None:
        if self.plan is None or not self.plan.operations:
            return
        if self.playing:
            self._stop_playback()
            return
        if self.operation_index == len(self.plan.operations) - 1 and self.operation_progress >= 1.0:
            self._set_playback_operation(0, 0.0)
        self.playing = True
        self.play_button.configure(text="Pause")
        self.last_playback_tick = perf_counter()
        self._advance_playback()

    def _stop_playback(self) -> None:
        self.playing = False
        self.last_playback_tick = None
        if self.playback_job is not None:
            self.after_cancel(self.playback_job)
            self.playback_job = None
        if hasattr(self, "play_button"):
            self.play_button.configure(text="Play")

    def _advance_playback(self) -> None:
        if not self.playing or self.plan is None:
            return
        self.playback_job = None
        now = perf_counter()
        elapsed = now - (self.last_playback_tick or now)
        self.last_playback_tick = now
        operation_duration_seconds = 0.90 / max(self.playback_speed.get(), 0.1)
        progress = self.operation_progress + elapsed / operation_duration_seconds
        index = self.operation_index
        while progress >= 1.0:
            if index >= len(self.plan.operations) - 1:
                self._set_playback_operation(index, 1.0)
                self._stop_playback()
                return
            progress -= 1.0
            index += 1
        self._set_playback_operation(index, progress)
        self.playback_job = self.after(25, self._advance_playback)

    def _step_playback(self, direction: int) -> None:
        if self.plan is None:
            return
        self._stop_playback()
        self._set_playback_operation(self.operation_index + direction, 1.0)

    def _restart_playback(self) -> None:
        self._stop_playback()
        self._set_playback_operation(0, 0.0)

    def _export(self) -> None:
        if self.plan is None:
            self._generate()
        if self.plan is None:
            return
        default_dir = Path(__file__).resolve().parents[3] / "toolpaths"
        default_dir.mkdir(parents=True, exist_ok=True)
        filename = filedialog.asksaveasfilename(
            title="Export ForgeBrain JSONL",
            defaultextension=".jsonl",
            filetypes=(("JSON Lines", "*.jsonl"), ("All files", "*.*")),
            initialdir=str(default_dir),
        )
        if not filename:
            return
        try:
            output = self.plan.write_jsonl(filename)
        except OSError as error:
            messagebox.showerror("Could not export JSONL", str(error), parent=self)
            return
        self.status.set(f"Exported {len(self.plan.operations)} operations to {output}")
        messagebox.showinfo(
            "JSONL exported",
            f"Wrote {len(self.plan.operations)} controller operations.\n\n"
            "Load the file into ForgeBrain only after reviewing the limits and proving the process.",
            parent=self,
        )

def main() -> None:
    ToolpathApp().mainloop()


if __name__ == "__main__":
    main()
