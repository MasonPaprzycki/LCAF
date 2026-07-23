"""Command-line entry point for the simple toolpath slicer."""

from __future__ import annotations

import argparse
from pathlib import Path

from .toolpath_slicer import MachineLimits, SliceSettings, ToolpathPlanningError, ToolpathSlicer, load_mesh
from .visualization import find_sufficient_cycles


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Slice an OBJ/STL convex forging target into LCAF controller JSONL."
    )
    parser.add_argument("model", type=Path, help="Watertight OBJ or STL target mesh")
    parser.add_argument("output", type=Path, help="Generated controller JSONL file")
    parser.add_argument("--stock-radius", type=float, required=True, help="Starting cylindrical stock radius in mm")
    parser.add_argument(
        "--radial-segments",
        type=int,
        default=4,
        help="Axial regions the target's length is divided into; each region's strike depth "
        "is the numerical average of the target across that whole region, not one point",
    )
    parser.add_argument(
        "--strikes-per-segment",
        type=int,
        default=4,
        help="Rotations struck per segment, evenly spaced (360/N degrees apart)",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=1,
        help="Times to repeat the entire segment x strike sweep end-to-end; later cycles true "
        "up whatever an earlier, rougher cycle left rough",
    )
    parser.add_argument(
        "--auto-cycles",
        action="store_true",
        help="Ignore --cycles and instead keep adding cycles until the simulated geometry "
        "matches the target within tolerance (see --auto-cycles-tolerance)",
    )
    parser.add_argument(
        "--auto-cycles-tolerance",
        type=float,
        default=0.5,
        help="Convergence tolerance in mm for --auto-cycles",
    )
    parser.add_argument(
        "--auto-cycles-max",
        type=int,
        default=20,
        help="Maximum cycles to try for --auto-cycles before giving up and warning",
    )
    parser.add_argument("--max-reduction", type=float, default=2.0, help="Maximum radial reduction per strike in mm")
    parser.add_argument("--die-contact-z", type=float, default=0.0, help="Calibrated Z coordinate at first stock contact")
    parser.add_argument("--x-offset", type=float, default=0.0, help="Machine X coordinate for the target centre")
    parser.add_argument("--y-position", type=float, default=0.0, help="Machine Y coordinate for the tool")
    parser.add_argument("--temperature", type=float, default=0.0, help="Target billet temperature metadata in C")
    parser.add_argument("--scale", type=float, default=1.0, help="Millimetres per input model unit")
    parser.add_argument("--axis", choices=("auto", "x", "y", "z"), default="auto", help="Billet longitudinal axis")
    parser.add_argument(
        "--clamped-end",
        choices=("min", "max"),
        default="min",
        help="Which end of the mesh (along its resolved axis) is clamped; machine +X always points away from it",
    )
    parser.add_argument(
        "--machine-config",
        type=Path,
        default=Path("configs/forge_parameters.json"),
        help="LCAF machine-limit JSON file",
    )
    parser.add_argument(
        "--die-width",
        type=float,
        default=None,
        help="Finite lower (anvil) die contact width across the strike direction in mm "
        "(default: a sensible physical default sized from the stock radius)",
    )
    parser.add_argument(
        "--die-length",
        type=float,
        default=None,
        help="Finite lower (anvil) die contact length along the billet axis in mm "
        "(default: a sensible physical default sized from the striking segment's own width)",
    )
    parser.add_argument(
        "--die-corner-radius",
        type=float,
        default=0.0,
        help="Radius blending the lower die's tangential edges into the rectangular face, in mm",
    )
    parser.add_argument(
        "--upper-die-radius",
        type=float,
        default=None,
        help="Finite upper (striking) die contact radius in mm, modelled as a flat-faced "
        "circular disc (default: large enough to always fully cover the target segment struck)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    settings = SliceSettings(
        stock_radius_mm=arguments.stock_radius,
        radial_segments=arguments.radial_segments,
        strikes_per_segment=arguments.strikes_per_segment,
        cycles=arguments.cycles,
        max_reduction_per_strike_mm=arguments.max_reduction,
        die_contact_z_mm=arguments.die_contact_z,
        x_offset_mm=arguments.x_offset,
        y_position_mm=arguments.y_position,
        target_temperature_c=arguments.temperature,
        scale_mm_per_unit=arguments.scale,
        longitudinal_axis=arguments.axis,
        stock_clamped_end=arguments.clamped_end,
        die_width_mm=arguments.die_width,
        die_length_mm=arguments.die_length,
        die_corner_radius_mm=arguments.die_corner_radius,
        upper_die_radius_mm=arguments.upper_die_radius,
    )
    try:
        mesh = load_mesh(arguments.model)
        if arguments.auto_cycles:
            plan = find_sufficient_cycles(
                mesh,
                settings,
                MachineLimits.from_lcaf_config(arguments.machine_config),
                max_cycles=arguments.auto_cycles_max,
                tolerance_mm=arguments.auto_cycles_tolerance,
            )
        else:
            plan = ToolpathSlicer(
                mesh, settings, MachineLimits.from_lcaf_config(arguments.machine_config)
            ).plan()
        output = plan.write_jsonl(arguments.output)
    except ToolpathPlanningError as error:
        print(f"Toolpath not generated: {error}")
        return 2

    if arguments.auto_cycles and plan.operations:
        used_cycles = plan.operations[-1]["metadata"]["cycle_index"] + 1
        print(f"Converged at {used_cycles} cycle(s).")
    print(f"Wrote {len(plan.operations)} controller operations to {output}")
    for warning in plan.warnings:
        print(f"WARNING: {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
