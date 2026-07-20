"""Command-line entry point for the simple profile slicer."""

from __future__ import annotations

import argparse
from pathlib import Path

from .profile_slicer import MachineLimits, ProfileSlicer, SliceSettings, ToolpathPlanningError, load_mesh


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Slice an OBJ/STL convex forging target into LCAF controller JSONL."
    )
    parser.add_argument("model", type=Path, help="Watertight OBJ or STL target mesh")
    parser.add_argument("output", type=Path, help="Generated controller JSONL file")
    parser.add_argument("--stock-radius", type=float, required=True, help="Starting cylindrical stock radius in mm")
    parser.add_argument("--axial-resolution", type=float, default=5.0, help="Maximum axial spacing in mm")
    parser.add_argument("--rotation-step", type=float, default=90.0, help="Rotary increment in degrees")
    parser.add_argument("--max-reduction", type=float, default=2.0, help="Maximum radial reduction per strike in mm")
    parser.add_argument("--die-contact-z", type=float, default=0.0, help="Calibrated Z coordinate at first stock contact")
    parser.add_argument("--x-offset", type=float, default=0.0, help="Machine X coordinate for the target centre")
    parser.add_argument("--y-position", type=float, default=0.0, help="Machine Y coordinate for the tool")
    parser.add_argument("--temperature", type=float, default=0.0, help="Target billet temperature metadata in C")
    parser.add_argument("--scale", type=float, default=1.0, help="Millimetres per input model unit")
    parser.add_argument("--axis", choices=("auto", "x", "y", "z"), default="auto", help="Billet longitudinal axis")
    parser.add_argument(
        "--machine-config",
        type=Path,
        default=Path("configs/forge_parameters.json"),
        help="LCAF machine-limit JSON file",
    )
    parser.add_argument(
        "--allow-out-of-limit-rotations",
        action="store_true",
        help="Bypass A-axis checks only for proven continuous/indexed hardware",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    try:
        plan = ProfileSlicer(
            load_mesh(arguments.model),
            SliceSettings(
                stock_radius_mm=arguments.stock_radius,
                axial_resolution_mm=arguments.axial_resolution,
                rotation_increment_deg=arguments.rotation_step,
                max_reduction_per_strike_mm=arguments.max_reduction,
                die_contact_z_mm=arguments.die_contact_z,
                x_offset_mm=arguments.x_offset,
                y_position_mm=arguments.y_position,
                target_temperature_c=arguments.temperature,
                scale_mm_per_unit=arguments.scale,
                longitudinal_axis=arguments.axis,
                allow_out_of_limit_rotations=arguments.allow_out_of_limit_rotations,
            ),
            MachineLimits.from_lcaf_config(arguments.machine_config),
        ).plan()
        output = plan.write_jsonl(arguments.output)
    except ToolpathPlanningError as error:
        print(f"Toolpath not generated: {error}")
        return 2

    print(f"Wrote {len(plan.operations)} controller operations to {output}")
    for warning in plan.warnings:
        print(f"WARNING: {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
