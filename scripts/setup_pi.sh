#!/usr/bin/env bash
set -euo pipefail

# Bootstraps this repo on a LinuxCNC Raspberry Pi image, automating the
# parts of docs/software_setup.md that are safe to run unattended:
# verifying prerequisites (sections 1-2), wiring PYTHONPATH into
# ~/.bashrc (section 4), and regenerating configs/generated/* (section 5).
#
# It deliberately stops there and does NOT start LinuxCNC or this
# project's control process. Both are meant to run in their own
# foreground terminal and be watched live the first time (see
# docs/software_setup.md section 5 and docs/hardware_setup.md section 11)
# -- collapsing them into this script would hide exactly the failures
# those docs tell you to check for by hand.
#
# Usage (from anywhere): bash /path/to/LCAF/scripts/setup_pi.sh

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR"

echo "==> Repo: $REPO_DIR"

echo "==> Checking for git"
if ! command -v git >/dev/null 2>&1; then
  echo "git not found. Run: sudo apt update && sudo apt install git" >&2
  exit 1
fi
git --version

echo "==> Checking for python3"
if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 not found -- this should ship with the LinuxCNC Pi image." >&2
  exit 1
fi
python3 --version

echo "==> Checking linuxcnc/hal python modules import"
if ! python3 -c "import linuxcnc, hal" 2>/dev/null; then
  cat >&2 <<'EOF'
Could not import 'linuxcnc'/'hal' in python3. This means either:
  - this python3 isn't the LinuxCNC image's system python (see the venv
    warning in docs/software_setup.md section 4), or
  - LinuxCNC itself isn't actually installed/working yet.
Fix this before continuing -- see docs/software_setup.md sections 1-2.
EOF
  exit 1
fi
echo "ok"

echo "==> Checking python3-tk (only needed for lcaf.toolpathing.ui)"
if ! python3 -c "import tkinter" 2>/dev/null; then
  echo "tkinter missing; installing python3-tk (sudo)..."
  sudo apt-get update && sudo apt-get install -y python3-tk
else
  echo "ok"
fi

echo "==> Ensuring PYTHONPATH is set in ~/.bashrc"
BASHRC="$HOME/.bashrc"
EXPORT_LINE="export PYTHONPATH=\"$REPO_DIR/src\${PYTHONPATH:+:\$PYTHONPATH}\""
if [ -f "$BASHRC" ] && grep -Fq "PYTHONPATH=\"$REPO_DIR/src" "$BASHRC"; then
  echo "~/.bashrc already sets PYTHONPATH for this repo."
else
  echo "$EXPORT_LINE" >> "$BASHRC"
  echo "Added to ~/.bashrc: $EXPORT_LINE"
fi
export PYTHONPATH="$REPO_DIR/src${PYTHONPATH:+:$PYTHONPATH}"

echo "==> Regenerating configs/generated/* from configs/machine.json + configs/axis.jsonl"
python3 -c "
from lcaf.utils.joint_configuration import load_machine_configuration, write_config_files
write_config_files(load_machine_configuration('configs/machine.json'), 'configs/generated')
"
echo "Wrote: configs/generated/LCAF_Forge.hal, LCAF_Forge.ini, tool.tbl"

cat <<EOF

==> Setup complete.

Before going further, confirm configs/axis.jsonl and configs/machine.json
actually match YOUR wiring (docs/hardware_setup.md sections 7-9) -- this
script regenerated from whatever is already checked in, it did not verify
that against real hardware.

This script does NOT start LinuxCNC or the control process. Next, in order:

  1. Open a new terminal (to pick up the ~/.bashrc PYTHONPATH change), or:
       source ~/.bashrc

  2. Terminal 1 -- start LinuxCNC against the generated INI:
       cd $REPO_DIR
       linuxcnc configs/generated/LCAF_Forge.ini
     Expect the Axis GUI with all joints "not homed". Leave this running.

  3. Jog/home each joint by hand and confirm correct direction/limits
     before trusting anything else (docs/hardware_setup.md section 11).

  4. Terminal 2 -- start this project's control process:
       cd $REPO_DIR
       export PYTHONPATH=$REPO_DIR/src
       python3 -m lcaf.control.main

See docs/software_setup.md section 5 for what to expect at each step.
EOF
