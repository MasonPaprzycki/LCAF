"""
Entry point for running the LCAF control process.

Usage (from the repo root, with LinuxCNC already running against the
generated INI -- see docs/software_setup.md):

    python3 -m lcaf.control.main [path/to/toolpath.jsonl]

If no path is given, this lists every .jsonl file in toolpaths/ (see
docs/software_setup.md) and lets you pick one, or type/paste a path to any
other file. Either way, nothing moves until you press Enter to confirm --
homing and toolpath execution both happen automatically after that, inside
ForgeBrain's own state machine (see docs/state_machine.md).
"""

from __future__ import annotations

import sys
from pathlib import Path

from lcaf.control.controller import Controller

# Repo root: src/lcaf/control/main.py -> repo root
_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_TOOLPATH_DIR = _REPO_ROOT / "toolpaths"


def _discover_toolpaths(directory: Path) -> list[Path]:
    if not directory.is_dir():
        return []
    return sorted(directory.glob("*.jsonl"))


def _choose_toolpath(cli_arg: str | None) -> Path:
    if cli_arg:
        path = Path(cli_arg)
        if not path.exists():
            print(f"No such file: {path}")
            sys.exit(1)
        return path

    candidates = _discover_toolpaths(_DEFAULT_TOOLPATH_DIR)

    while True:
        print()
        print(f"Toolpaths in {_DEFAULT_TOOLPATH_DIR}:")
        if not candidates:
            print("  (none found -- generate one with 'python3 -m lcaf.toolpathing.ui')")
        for index, path in enumerate(candidates, start=1):
            print(f"  {index}. {path.name}")
        print()

        choice = input(
            "Enter a number above, or type/paste a path to a .jsonl file: "
        ).strip()

        if not choice:
            continue

        if choice.isdigit():
            index = int(choice)
            if 1 <= index <= len(candidates):
                return candidates[index - 1]
            print(f"No toolpath numbered {index}.")
            continue

        path = Path(choice)
        if path.exists():
            return path
        print(f"No such file: {path}")


def _connect() -> Controller:
    try:
        return Controller()
    except Exception as error:
        print(
            "\nCould not connect to LinuxCNC. Is it already running against the "
            "generated INI?\n"
            "  linuxcnc configs/generated/<machine_name>.ini\n"
            "See docs/software_setup.md section 5.\n"
            f"\nUnderlying error: {error}"
        )
        sys.exit(1)


def main() -> None:
    cli_arg = sys.argv[1] if len(sys.argv) > 1 else None
    toolpath = _choose_toolpath(cli_arg)

    print(f"\nConnecting to LinuxCNC and loading '{toolpath}'...")
    controller = _connect()
    controller.brain.load_jsonl(str(toolpath))

    if controller.brain.state.fault_active:
        print(f"Could not load toolpath: {controller.brain.state.fault_message}")
        sys.exit(1)

    print(f"Loaded {len(controller.brain.queue.operations)} operations from '{toolpath}'.")
    print(
        "\nNothing has moved yet. Pressing Enter below will home every axis, then "
        "run this toolpath automatically once homing completes."
    )
    input("\nPress Enter to home the machine and begin (Ctrl+C to cancel)... ")

    print("\nRunning. Ctrl+C to stop.\n")
    controller.run()


if __name__ == "__main__":
    main()
