# Software Setup: Getting LCAF Running on the Raspberry Pi

This is the guide for getting this repository (the "LCAF" folder) onto the
Raspberry Pi that runs LinuxCNC, and for what you should expect to see once
it's there. For wiring the Mesa card, motors, and limit switches, see
[hardware_setup.md](hardware_setup.md).

## 1. What the Pi needs on it before any of this

LinuxCNC itself needs a realtime-capable Linux kernel; don't install
LinuxCNC on a stock Raspberry Pi OS image yourself. Use the **official
LinuxCNC Raspberry Pi image** from the downloads page at linuxcnc.org --
it's a full Debian-based OS image with LinuxCNC, its realtime kernel, and
the `linuxcnc`/`hal` Python modules already built and installed. Flash it
to an SD card with Raspberry Pi Imager (or `dd`), matching your Pi model
(this project targets a Pi running the Mesa 7I76E over Ethernet, which
works fine on any Pi model the image supports -- it doesn't need the
parallel port, so it isn't limited to older Pi boards the way a
parallel-port LinuxCNC setup would be).

Boot it, complete the normal first-boot setup (locale, password, Wi-Fi if
you're using it for anything other than the Mesa card's dedicated NIC --
see `hardware_setup.md` section 4 for that connection), and confirm
LinuxCNC itself launches from its desktop shortcut with a stock demo config
before going any further. If that doesn't work, hardware/network wiring
questions in this repo are premature -- fix the base LinuxCNC install
first.

## 2. What you should see

A LinuxCNC Raspberry Pi image boots to a lightweight desktop (LXDE or
similar) with a LinuxCNC launcher, a file manager, and a terminal. There is
no code from this project on it yet -- that's section 3. Two things worth
confirming right away in a terminal, since the rest of this project depends
on both:

```
python3 -c "import linuxcnc, hal; print('ok')"
```

should print `ok` with no import error. This confirms the LinuxCNC image's
Python and its compiled `linuxcnc`/`hal` extension modules match -- see the
warning in section 4 about virtual environments.

```
git --version
```

should print a version. If it's missing, `sudo apt update && sudo apt
install git`.

## 3. Getting this repository onto the Pi

Clone it directly on the Pi over the network (recommended -- keeps it easy
to `git pull` later), or copy it over if the Pi has no internet access:

```
git clone <this repository's URL> ~/LCAF
cd ~/LCAF
```

If the Pi is offline, clone or download the repo on another machine and
copy the folder over with `scp -r LCAF pi@<pi-ip>:~/` or a USB drive
instead.

## 4. Python environment

**Do not create an isolated virtual environment with `python3 -m venv`
alone.** The `linuxcnc`/`hal` modules (section 2) are compiled C extensions
installed into the LinuxCNC image's *system* Python -- a normal venv is
isolated from that and `import linuxcnc` will fail inside it. If you want a
venv anyway (e.g. to pin dependency versions), create it with
`--system-site-packages` so it still sees the system-installed
`linuxcnc`/`hal`:

```
python3 -m venv --system-site-packages ~/lcaf-venv
source ~/lcaf-venv/bin/activate
```

This project's control/config code (`lcaf.control`, `lcaf.utils`) has no
third-party dependencies beyond `linuxcnc`/`hal` themselves. The toolpath
slicer UI (`lcaf.toolpathing.ui`) uses `tkinter`, which ships with the
LinuxCNC image's Python already; if it's ever missing, `sudo apt install
python3-tk`.

Every command in this project's docs assumes `PYTHONPATH` includes this
repo's `src/` directory:

```
export PYTHONPATH=~/LCAF/src
```

Add that line to `~/.bashrc` if you don't want to retype it every session.

## 5. Running it

With the Mesa card wired and networked (`hardware_setup.md` sections 4-5)
and `configs/axis.jsonl`/`machine.json` edited for your hardware
(`hardware_setup.md` sections 8-9), run these in order. Each step says what
you should actually see before moving to the next one -- if it doesn't
match, stop and fix that step rather than continuing.

```
cd ~/LCAF
export PYTHONPATH=~/LCAF/src
```

No output expected from either line. If `cd` fails, the clone from section
3 didn't land where this guide assumes.

```
# Regenerate the .hal/.ini from your config files:
python3 -c "from lcaf.utils.joint_configuration import load_machine_configuration, write_config_files; write_config_files(load_machine_configuration('configs/machine.json'), 'configs/generated')"
```

You should see no output and get your shell prompt back immediately (this
only renders text files -- it never touches `linuxcnc`/`hal`). Check
`configs/generated/LCAF_Forge.hal` and `.ini` now exist. A traceback here
means a problem in `configs/machine.json`/`axis.jsonl` themselves (fix the
config, not the generator).

```
# Terminal 1: start LinuxCNC itself against the generated INI.
linuxcnc configs/generated/LCAF_Forge.ini
```

This opens LinuxCNC's own Axis GUI window. Expect the usual LinuxCNC
startup: it loads the generated HAL, and the GUI shows all four joints as
**not homed**. Leave this running in its own terminal -- this is the
realtime process everything else connects to. See
`hardware_setup.md` section 11 for jogging/homing each joint by hand in
this GUI before trusting anything below.

```
# Terminal 2: start this project's control process, which connects to the
# already-running LinuxCNC over NML.
python3 -m lcaf.control.main
```

You'll first be asked to pick a `.jsonl` toolpath from `toolpaths/` (or
type/paste a path to one); if none exist yet, generate one with
`python3 -m lcaf.toolpathing.ui` first (see below). After you pick one, it
prints how many operations it loaded, then prompts:

```
Press Enter to home the machine and begin (Ctrl+C to cancel)...
```

Nothing moves until you press Enter. Once you do, expect log lines
(`%(asctime)s | %(levelname)s | %(name)s | %(message)s` format) reporting
each axis homing in turn, then the toolpath executing operation by
operation. It refuses to move any axis until homing finishes -- if the log
keeps repeating a "not homed yet" warning instead of progressing, stop and
check the physical homing behavior in the LinuxCNC GUI directly first
(`hardware_setup.md` section 11), rather than assuming this project's code
is at fault. `Ctrl+C` in this terminal stops the control process; LinuxCNC
itself (terminal 1) keeps running.

To run the toolpath slicer UI instead (a separate, offline planning tool
that never touches LinuxCNC -- see
[toolpath_slicer_ui_guide.md](toolpath_slicer_ui_guide.md)) on the Pi's own
display, or over `ssh -X` from your laptop:

```
python3 -m lcaf.toolpathing.ui
```

Expect a Tk window to open with the toolpath planning form described in
that guide -- no LinuxCNC connection is made or required for this one.

## 6. Updating

```
cd ~/LCAF
git pull
```

Regenerate `configs/generated/*` (section 5) and restart LinuxCNC to pick
up any config or generator change -- it does not hot-reload.
