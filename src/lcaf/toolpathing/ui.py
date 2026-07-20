"""A dependency-free preview and export UI for :mod:`lcaf.toolpathing`.

This is deliberately a planning/proving front end, not a machine-control UI.
It never connects to LinuxCNC or sends a command; its only output is JSONL for
review and later loading by ``ForgeBrain``.
"""

from __future__ import annotations

import math
import tkinter as tk
from time import perf_counter
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from .profile_slicer import (
    MachineLimits,
    ProfileSlicer,
    SliceSettings,
    ToolpathPlan,
    ToolpathPlanningError,
    load_mesh,
)
from .visualization import material_cross_section, radial_resample


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
    strike = "#e77878"

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
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
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
        self.selected_station = int(operation["metadata"]["station_index"])
        self.selected_rotation = float(operation["rotation"])

    def _active_operation(self) -> dict | None:
        if self.plan is None or not self.plan.operations:
            return None
        return self.plan.operations[self.operation_index]

    def _active_die_support(self) -> float:
        operation = self._active_operation()
        if operation is None:
            return self.stock_radius
        reduction = float(operation["metadata"]["radial_reduction_mm"])
        return self.stock_radius - reduction * self.motion_progress

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
        self._draw_cross_section(20, 40, split - 40, height - 60)
        self._draw_axial_plan(split + 20, 40, width - split - 40, height - 60)

    def _draw_cross_section(self, left: int, top: int, width: int, height: int) -> None:
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
            for y, z in material_cross_section(
                self.plan,
                self.selected_station,
                self.operation_index,
                self.motion_progress,
                radial_segments=64,
            )
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
        line_length = extent * 1.25
        base_y, base_z = normal_y * active_support, normal_z * active_support
        die_a = (base_y + tangent_y * line_length, base_z + tangent_z * line_length)
        die_b = (base_y - tangent_y * line_length, base_z - tangent_z * line_length)
        self.canvas.create_line(
            center_x + die_a[0] * scale,
            center_y - die_a[1] * scale,
            center_x + die_b[0] * scale,
            center_y - die_b[1] * scale,
            fill=self.die,
            width=6,
        )
        self.canvas.create_text(
            left + 10,
            top + height - 4,
            anchor="sw",
            text=(
                f"station {self.selected_station + 1}/{len(self.plan.sections)}  "
                f"X={section.x_model_mm:.2f} mm  A={rotation:.0f}°  "
                f"die={active_support:.2f} mm  target={support:.2f} mm"
            ),
            fill=self.text,
            font=("Consolas", 9),
        )
        self.canvas.create_text(left + 10, top + 5, anchor="nw", text="red: remaining stock   green hatch: target geometry   gold: active die   dashed: original cylinder", fill="#d9e2f2", font=("Segoe UI", 9))

    def _draw_axial_plan(self, left: int, top: int, width: int, height: int) -> None:
        assert self.plan is not None
        sections = self.plan.sections
        x_values = [section.x_model_mm for section in sections]
        min_x, max_x = min(x_values), max(x_values)
        x_range = max(max_x - min_x, 1.0)
        centre_y = top + height * 0.5
        radial_scale = height * 0.31 / max(self.stock_radius, 1.0)

        def to_x(model_x: float) -> float:
            return left + 18 + (model_x - min_x) / x_range * (width - 36)

        stock_top = centre_y - self.stock_radius * radial_scale
        stock_bottom = centre_y + self.stock_radius * radial_scale
        self.canvas.create_rectangle(left + 18, stock_top, left + width - 18, stock_bottom, outline=self.stock_edge, width=1, dash=(5, 4))

        # At 0° the positive-Z support is exactly the side-view top profile.
        top_profile = [to_x(section.x_model_mm) for section in sections]
        target_top = [centre_y - section.support_mm(0.0) * radial_scale for section in sections]
        target_bottom = [centre_y + (-min(z for _, z in section.polygon_yz_mm)) * radial_scale for section in reversed(sections)]
        target_polygon = [coordinate for pair in zip(top_profile, target_top) for coordinate in pair]
        target_polygon += [coordinate for pair in zip(reversed(top_profile), target_bottom) for coordinate in pair]
        material_sections = [
            material_cross_section(
                self.plan,
                station_index,
                self.operation_index,
                self.motion_progress,
                radial_segments=64,
            )
            for station_index in range(len(sections))
        ]
        material_top = [centre_y - max(z for _, z in ring) * radial_scale for ring in material_sections]
        material_bottom = [centre_y - min(z for _, z in ring) * radial_scale for ring in reversed(material_sections)]
        material_polygon = [coordinate for pair in zip(top_profile, material_top) for coordinate in pair]
        material_polygon += [coordinate for pair in zip(reversed(top_profile), material_bottom) for coordinate in pair]
        self.canvas.create_polygon(*material_polygon, fill=self.stock, outline=self.stock_edge, width=2)
        self.canvas.create_polygon(*target_polygon, fill=self.target_fill, outline=self.target, width=2, stipple="gray25")

        selected = sections[self.selected_station]
        selected_x = to_x(selected.x_model_mm)
        self.canvas.create_line(selected_x, top + 24, selected_x, top + height - 22, fill=self.strike, width=2)
        active_support = self._active_die_support()
        die_y = centre_y - active_support * radial_scale
        self.canvas.create_line(selected_x - 16, die_y, selected_x + 16, die_y, fill=self.die, width=6)
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
        self.canvas.create_text(left + 10, top + height - 4, anchor="sw", text=f"red: remaining stock   green hatch: target profile   gold: die  |  {step_text}", fill="#d9e2f2", font=("Segoe UI", 9))

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
    grid = "#26344a"

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master)
        self.canvas = tk.Canvas(self, width=920, height=340, bg=self.background, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.plan: ToolpathPlan | None = None
        self.stock_radius = 0.0
        self.selected_station = 0
        self.operation_index = 0
        self.motion_progress = 0.0
        self.selected_rotation = 0.0
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
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
        self.selected_station = int(operation["metadata"]["station_index"])
        self.selected_rotation = float(operation["rotation"])
        self.redraw()

    def redraw(self) -> None:
        canvas = self.canvas
        canvas.delete("all")
        width = max(canvas.winfo_width(), 920)
        height = max(canvas.winfo_height(), 455)
        canvas.create_text(22, 18, anchor="w", text="3D GEOMETRIC-ENVELOPE PLAYBACK", fill=self.text, font=("Segoe UI", 10, "bold"))
        if self.plan is None or not self.plan.sections:
            canvas.create_text(width / 2, height / 2, text="Generate a plan, then switch here to watch the 3D envelope evolve.", fill="#8fa3c0", font=("Segoe UI", 12))
            return

        sections = self.plan.sections
        segment_count = 36
        material_rings = [
            material_cross_section(
                self.plan,
                station_index,
                self.operation_index,
                self.motion_progress,
                radial_segments=segment_count,
            )
            for station_index in range(len(sections))
        ]
        target_rings = [
            radial_resample(section.polygon_yz_mm, radial_segments=segment_count)
            for section in sections
        ]
        project, depth = self._projector(width, height, sections)

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

        for _, coordinates, color in sorted(faces, key=lambda face: face[0]):
            canvas.create_polygon(*coordinates, fill=color, outline="#6f1820", width=1)

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

        self._draw_die(project)
        canvas.create_text(22, height - 20, anchor="w", text="solid red: remaining clipped stock   green wireframe: target geometry   gold plate: active die", fill=self.text, font=("Segoe UI", 9))
        operation = self.plan.operations[self.operation_index]
        canvas.create_text(width - 22, height - 20, anchor="e", text=f"step {self.operation_index + 1}/{len(self.plan.operations)} · A={float(operation['rotation']):.0f}°", fill=self.text, font=("Segoe UI", 9))

    def _projector(self, width: int, height: int, sections: tuple) -> tuple:
        min_x = min(section.x_model_mm for section in sections)
        max_x = max(section.x_model_mm for section in sections)
        span_x = max(max_x - min_x, 1.0)
        yaw = math.radians(-32)
        pitch = math.radians(24)
        scale = min(width * 0.70 / (span_x + self.stock_radius * 1.4), height * 0.72 / (self.stock_radius * 2.5))
        centre_x = width * 0.50
        centre_y = height * 0.54

        def transform(model_x: float, y: float, z: float) -> tuple[float, float, float]:
            horizontal = model_x * math.cos(yaw) + y * math.sin(yaw)
            view_depth = -model_x * math.sin(yaw) + y * math.cos(yaw)
            vertical = z * math.cos(pitch) - view_depth * math.sin(pitch)
            return (centre_x + horizontal * scale, centre_y - vertical * scale, view_depth)

        def project(model_x: float, y: float, z: float) -> tuple[float, float]:
            projected_x, projected_y, _ = transform(model_x, y, z)
            return projected_x, projected_y

        def depth(model_x: float, y: float, z: float) -> float:
            return transform(model_x, y, z)[2]

        return project, depth

    def _draw_die(self, project) -> None:
        assert self.plan is not None
        operation = self.plan.operations[self.operation_index]
        metadata = operation["metadata"]
        section = self.plan.sections[self.selected_station]
        reduction = float(metadata["radial_reduction_mm"]) * self.motion_progress
        support = self.stock_radius - reduction
        angle = math.radians(self.selected_rotation)
        normal_y, normal_z = math.sin(angle), math.cos(angle)
        tangent_y, tangent_z = normal_z, -normal_y
        x_half_width = max(0.75, (self.plan.sections[-1].x_model_mm - self.plan.sections[0].x_model_mm) / max(12.0, len(self.plan.sections) * 2.0))
        tangent_half_width = self.stock_radius * 1.22
        corners = (
            (section.x_model_mm - x_half_width, normal_y * support + tangent_y * tangent_half_width, normal_z * support + tangent_z * tangent_half_width),
            (section.x_model_mm + x_half_width, normal_y * support + tangent_y * tangent_half_width, normal_z * support + tangent_z * tangent_half_width),
            (section.x_model_mm + x_half_width, normal_y * support - tangent_y * tangent_half_width, normal_z * support - tangent_z * tangent_half_width),
            (section.x_model_mm - x_half_width, normal_y * support - tangent_y * tangent_half_width, normal_z * support - tangent_z * tangent_half_width),
        )
        coordinates = [value for corner in corners for value in project(*corner)]
        self.canvas.create_polygon(*coordinates, fill=self.die, outline="#fff0a3", width=2)


class ToolpathApp(tk.Tk):
    """Interactive parameters, preview, and JSONL export."""

    def __init__(self) -> None:
        super().__init__()
        self.title("LCAF Profile Slicer")

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
        self.axial_resolution = tk.StringVar(value="5")
        self.rotation_step = tk.StringVar(value="90")
        self.max_reduction = tk.StringVar(value="2")
        self.die_contact_z = tk.StringVar(value="0")
        self.x_offset = tk.StringVar(value="0")
        self.y_position = tk.StringVar(value="0")
        self.scale = tk.StringVar(value="1")
        self.axis = tk.StringVar(value="auto")
        self.temperature = tk.StringVar(value="0")
        self.allow_rotation_override = tk.BooleanVar(value=False)
        self.example_target = tk.StringVar(value="")
        self.playback_speed = tk.DoubleVar(value=1.0)
        self.playback_caption = tk.StringVar(value="Generate a preview to enable toolpath playback.")
        self.status = tk.StringVar(value="Planning only — no machine connection or motion commands.")

        self._build_layout()

    @staticmethod
    def _default_limits() -> MachineLimits:
        root = Path(__file__).resolve().parents[3]
        config = root / "configs" / "forge_parameters.json"
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
        ttk.Label(header, text="LCAF simple profile slicer", font=("Segoe UI", 18, "bold")).pack(anchor="w")
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
        entries = (
            ("Stock radius", self.stock_radius),
            ("Axial spacing", self.axial_resolution),
            ("Rotation step (°)", self.rotation_step),
            ("Max reduction / strike", self.max_reduction),
            ("Die contact Z", self.die_contact_z),
            ("Model scale", self.scale),
            ("Target temperature (°C)", self.temperature),
            ("X centre offset", self.x_offset),
            ("Y tool position", self.y_position),
        )
        for index, (label, variable) in enumerate(entries):
            row, column = divmod(index, 3)
            field = ttk.Frame(parameters)
            field.grid(row=row, column=column, sticky="ew", padx=6, pady=4)
            ttk.Label(field, text=label).pack(anchor="w")
            ttk.Entry(field, textvariable=variable, width=18).pack(fill="x")
            parameters.columnconfigure(column, weight=1)
        axis_field = ttk.Frame(parameters)
        axis_field.grid(row=3, column=0, sticky="ew", padx=6, pady=4)
        ttk.Label(axis_field, text="Billet longitudinal axis").pack(anchor="w")
        ttk.Combobox(axis_field, textvariable=self.axis, values=("auto", "x", "y", "z"), state="readonly", width=15).pack(fill="x")
        ttk.Checkbutton(
            parameters,
            text="Override configured A-axis limits (only after verifying continuous/indexed rotary hardware)",
            variable=self.allow_rotation_override,
        ).grid(row=3, column=1, columnspan=2, sticky="w", padx=6, pady=5)

        actions = ttk.Frame(shell)
        actions.pack(fill="x", pady=(0, 10))
        ttk.Button(actions, text="Generate preview", command=self._generate).pack(side="left", padx=(0, 8))
        ttk.Button(actions, text="Export JSONL", command=self._export).pack(side="left")
        ttk.Label(actions, textvariable=self.status, foreground="#9fb0c9").pack(side="left", padx=18)

        playback = ttk.LabelFrame(shell, text="3. Animated toolpath preview", padding=8)
        playback.pack(fill="x", pady=(0, 10))
        self.back_button = ttk.Button(playback, text="◀ Step", command=lambda: self._step_playback(-1), state="disabled")
        self.back_button.pack(side="left")
        self.play_button = ttk.Button(playback, text="Play", command=self._toggle_playback, state="disabled")
        self.play_button.pack(side="left", padx=6)
        self.forward_button = ttk.Button(playback, text="Step ▶", command=lambda: self._step_playback(1), state="disabled")
        self.forward_button.pack(side="left")
        self.restart_button = ttk.Button(playback, text="Restart", command=self._restart_playback, state="disabled")
        self.restart_button.pack(side="left", padx=6)
        ttk.Label(playback, text="Speed").pack(side="left", padx=(14, 4))
        self.speed_slider = ttk.Scale(playback, from_=0.5, to=4.0, variable=self.playback_speed, orient="horizontal", length=105)
        self.speed_slider.pack(side="left")
        ttk.Label(playback, text="0.5×–4×").pack(side="left", padx=(4, 14))
        ttk.Label(playback, textvariable=self.playback_caption, foreground="#9fb0c9").pack(side="left")

        self.preview = ForgePreview(shell)
        self.preview.pack(fill="both", expand=True)

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
        self.stock_radius.set("10")
        self.axial_resolution.set("5" if "Tapered" in self.example_target.get() or "Hex" in self.example_target.get() else "10")
        self.rotation_step.set("90")
        self.max_reduction.set("2")
        self.die_contact_z.set("10")
        self.axis.set("x")
        self.status.set("Example loaded. Generate a preview; use the rotary override only for visualisation or proven hardware.")

    def _current_settings(self) -> SliceSettings:
        try:
            return SliceSettings(
                stock_radius_mm=float(self.stock_radius.get()),
                axial_resolution_mm=float(self.axial_resolution.get()),
                rotation_increment_deg=float(self.rotation_step.get()),
                max_reduction_per_strike_mm=float(self.max_reduction.get()),
                die_contact_z_mm=float(self.die_contact_z.get()),
                x_offset_mm=float(self.x_offset.get()),
                y_position_mm=float(self.y_position.get()),
                target_temperature_c=float(self.temperature.get()),
                scale_mm_per_unit=float(self.scale.get()),
                longitudinal_axis=self.axis.get(),
                allow_out_of_limit_rotations=self.allow_rotation_override.get(),
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
            self.plan = ProfileSlicer(load_mesh(filename), self.settings, self.limits).plan()
        except ToolpathPlanningError as error:
            self.plan = None
            self._set_playback_enabled(False)
            self.status.set("No JSONL generated — fix the validation issue.")
            messagebox.showerror("Cannot generate toolpath", str(error), parent=self)
            return

        self.preview.show_plan(self.plan, self.settings)
        self.operation_index = 0
        self.operation_progress = 0.0
        self._set_playback_operation(0, 0.0)
        self._set_playback_enabled(True)
        self.status.set(
            f"Preview ready: {len(self.plan.sections)} axial stations, "
            f"{len(self.plan.rotations_deg)} orientations, {len(self.plan.operations)} strike operations."
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
        operation = self.plan.operations[self.operation_index]
        metadata = operation["metadata"]
        state = "complete" if self.operation_progress >= 1.0 else f"in-stroke {self.operation_progress * 100:.0f}%"
        self.playback_caption.set(
            f"Step {self.operation_index + 1}/{len(self.plan.operations)} · "
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
        filename = filedialog.asksaveasfilename(
            title="Export ForgeBrain JSONL",
            defaultextension=".jsonl",
            filetypes=(("JSON Lines", "*.jsonl"), ("All files", "*.*")),
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
