# Windows Manual Testing

This is the by-hand (not AI-run) reference for checking this project's
code and configs from a Windows dev machine, without the Pi or any real
Mesa/LinuxCNC hardware attached. [software_setup.md](software_setup.md) is
the Pi-side counterpart to this doc -- read that one for anything that
actually needs to run against real LinuxCNC/hardware.

## 0. What can and can't run on Windows

`linuxcnc`/`hal` are compiled C extensions that only exist inside a
LinuxCNC install (see `software_setup.md` section 2) -- they cannot be
`pip install`ed or otherwise obtained on Windows. That means:

**Can run on Windows, no Pi needed:**
- `lcaf.utils.joint_configuration` -- pure text generation (`generate_hal`/
  `generate_ini`/`write_config_files`). Nothing in it imports `linuxcnc`/
  `hal`.
- The automated test suite (`debug/tests`) -- `debug/tests/conftest.py`
  installs fake `linuxcnc`/`hal` modules into `sys.modules` before
  anything imports them, specifically so the suite runs anywhere.
- `lcaf.toolpathing` (the slicer/planner + its Tk UI) -- a fully separate,
  offline tool that never touches LinuxCNC at all. See
  [toolpath_slicer_ui_guide.md](toolpath_slicer_ui_guide.md).

**Cannot run on Windows -- Pi only:**
- LinuxCNC itself (`linuxcnc configs/generated/LCAF_Forge.ini`).
- `python -m lcaf.control.main` and anything else importing
  `lcaf.control.linuxcnc_interface` outside of the test suite's fakes --
  it does a real `import linuxcnc` / `import hal` at module load time and
  will fail with `ModuleNotFoundError` on Windows.

So: use this machine to edit configs, regenerate/inspect the `.hal`/`.ini`
text, run the test suite, and iterate on toolpath generation. Anything
that needs to actually move a motor still has to happen on the Pi.

## 1. One-time setup

Open PowerShell in the repo root (`c:\GitHub\LCAF`) and confirm Python is
on `PATH`:

```powershell
python --version
```

Every command below assumes `PYTHONPATH` includes this repo's `src\`
directory, same as the Pi docs assume for `~/LCAF/src` (`software_setup.md`
section 4):

```powershell
$env:PYTHONPATH = "src"
```

This only lasts for the current PowerShell window -- set it again in every
new terminal, or persist it once for your user account:

```powershell
[Environment]::SetEnvironmentVariable("PYTHONPATH", "C:\GitHub\LCAF\src", "User")
```

(requires a new terminal to take effect; use the repo's actual absolute
path if it isn't `C:\GitHub\LCAF`).

## 2. Regenerate the `.hal`/`.ini` files

After editing `configs/axis.json` or `configs/machine.json`
(`docs/hardware_setup.md` sections 8-9), regenerate the derived files the
same way the Pi does:

```powershell
python -c "from lcaf.utils.joint_configuration import load_machine_configuration, write_config_files; write_config_files(load_machine_configuration('configs/machine.json'), 'configs/generated')"
```

No output means it worked. Check that `configs/generated/LCAF_Forge.hal`,
`.ini`, and `tool.tbl` now exist (or were updated -- `tool.tbl` is created
once and never overwritten, see `software_setup.md` section 5). A
traceback here points at a problem in `machine.json`/`axis.json`
themselves, not the generator -- read the `ValueError` message, it names
the joint and field at fault.

You never need to copy these generated files to the Pi by hand -- they're
just as easy (and less error-prone) to regenerate over there directly with
the Linux equivalent of this same command. Use the Windows-side run mainly
to sanity-check a config edit before it ever reaches the Pi.

### Eyeballing the result

Print just the travel-limit lines instead of opening the whole `.ini`:

```powershell
python -c "from lcaf.utils.joint_configuration import load_machine_configuration, generate_ini; print(generate_ini(load_machine_configuration('configs/machine.json')))" | Select-String "LIMIT|^\["
```

Confirm each `[JOINT_n]`/`[AXIS_x]` pair's `MIN_LIMIT`/`MAX_LIMIT` matches
what you expect from that joint's `retracted_distance`/`extended_distance`
in `axis.json` (`MIN_LIMIT` should always be `-retracted_distance`,
`MAX_LIMIT` always `extended_distance`). A joint with either field left
`null` (this project's A) has that entry omitted entirely instead --
`Select-String "LIMIT|^\["` will show `[JOINT_3]`/`[AXIS_A]` with no
`MIN_LIMIT`/`MAX_LIMIT` line at all, which is expected (LinuxCNC itself
then defaults those to `-1e99`/`1e99`), not a bug.

To look at the wiring instead (which HAL pin nets to which), print the
`.hal` text the same way:

```powershell
python -c "from lcaf.utils.joint_configuration import load_machine_configuration, generate_hal; print(generate_hal(load_machine_configuration('configs/machine.json')))"
```

Add `simulate=True` to `generate_hal(...)` to render the loopback/
fake-limit-switch version instead (`generate_hal(m, simulate=True)`) --
useful for seeing what a hardware-free LinuxCNC sim config would look
like, though actually *running* it still requires LinuxCNC (Pi-only, or a
Linux VM/WSL with LinuxCNC installed -- not covered here).

## 3. Just validate the configs load (no file output)

If you only want to confirm `axis.json`/`machine.json` parse and pass
`JointConfiguration`/`MachineConfiguration` validation -- e.g. after
hand-editing a field -- without touching `configs/generated/`:

```powershell
python -c "from lcaf.utils.joint_configuration import load_machine_configuration; m = load_machine_configuration('configs/machine.json'); print([ (j.axis, j.retracted_distance, j.extended_distance) for j in m.joints ])"
```

A clean list printout means the config loaded and validated; a
`ValueError`/`FileNotFoundError` traceback tells you exactly which joint
and rule failed.

## 4. Run the automated test suite

```powershell
python -m pytest debug/tests -q
```

(`$env:PYTHONPATH` from section 1 must already be set in this terminal, or
prefix the call: `$env:PYTHONPATH = "src"; python -m pytest debug/tests -q`.)
This is the same suite described in
[debug/tests/README.md](../debug/tests/README.md) -- it runs entirely
against the fake `linuxcnc`/`hal` modules, so it's a full, real check of
this project's config/homing/retract logic even though no Mesa card or
LinuxCNC instance is anywhere nearby. Run it after any change to
`lcaf.utils.joint_configuration`, `lcaf.control.*`, or the config files
themselves.

To run just one file or one test while iterating:

```powershell
python -m pytest debug/tests/test_joint_configuration.py -q
python -m pytest debug/tests/test_linuxcnc_interface.py::RetractToTests -q
```

## 5. Exercise the toolpath slicer (fully offline, no Pi needed)

The toolpath slicer/planner never imports `linuxcnc`/`hal` at all, so it's
the one piece of this project meant to be run and iterated on directly on
Windows. CLI:

```powershell
python -m lcaf.toolpathing.cli path\to\model.obj path\to\output.jsonl --stock-radius 25
```

Or the Tk UI (ships with the standard python.org Windows installer, no
separate install needed):

```powershell
python -m lcaf.toolpathing.ui
```

See [toolpath_slicer_ui_guide.md](toolpath_slicer_ui_guide.md) for what
the UI's fields mean and how to read its preview. Generated `.jsonl`
toolpaths can be dropped into `toolpaths/` and are what
`python3 -m lcaf.control.main` picks from on the Pi later -- but running
that file *through* the machine still only happens there.

## 6. Common gotchas

- **`ModuleNotFoundError: No module named 'lcaf'`** -- `$env:PYTHONPATH`
  isn't set in this terminal (section 1). It does not persist across
  PowerShell windows unless you set it at the User/Machine level.
- **`ModuleNotFoundError: No module named 'linuxcnc'`** from anything
  other than `debug/tests` -- expected on Windows (section 0); that
  module/path is Pi-only.
- **Git LF/CRLF warnings** (`warning: LF will be replaced by CRLF...`) on
  `configs/axis.json`, files under `debug/tests/`, `docs/*.md`, etc. --
  harmless line-ending normalization from `.gitattributes`/Git's
  `core.autocrlf`; it doesn't affect anything in this doc or the tests.
- **`python` vs `python3`** -- the Pi docs use `python3` (Debian's
  convention); the standard Windows python.org installer only registers
  `python`, so every command above uses that instead. They're the same
  interpreter otherwise.
