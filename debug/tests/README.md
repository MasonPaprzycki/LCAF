# debug/tests

Automated test suite for this repo, kept under `debug/` (rather than a
top-level `tests/`) so it doesn't compete for attention with the actual
control/toolpathing source under `src/lcaf/` when browsing the repo root.
It's still an ordinary `pytest` suite -- nothing about running it changes
because of where it lives.

## Running

```
cd <repo root>
PYTHONPATH=src python -m pytest debug/tests -q
```

`PYTHONPATH=src` is required so `import lcaf...` resolves (same requirement
as running the control process itself -- see
[docs/software_setup.md](../../docs/software_setup.md) section 4). No
`pytest.ini`/`pyproject.toml` pins the rootdir or test paths, so `pytest`
run from the repo root with no arguments will also discover everything
here on its own.

## What each file covers

- **`test_toolpath_slicer.py`** -- the offline toolpath slicer/planner
  (`lcaf.toolpathing`): mesh slicing, material-mechanics estimates, and
  visualization support functions. No LinuxCNC dependency at all -- this
  is the one file here that predates the rest of this folder.

- **`test_joint_configuration.py`** -- `lcaf.utils.joint_configuration`:
  that negative/positive limit-switch inputs are rejected when they're
  wired to the same pin (`JointConfiguration.__post_init__`), and that
  `generate_hal()` actually emits distinct HAL nets for them from the real
  `configs/axis.jsonl`. Pure text generation, no `linuxcnc`/`hal` import,
  so no fakes needed.

- **`test_linuxcnc_interface.py`** -- `LinuxCNCAxialInterface`'s software
  homing path (`use_linuxcnc_native_processes: false`, this project's
  default): that every deliberate jog toward a limit switch during homing
  is bracketed with `command.override_limits()` (re-armed every heartbeat,
  not just once) so LinuxCNC's own hard-limit fault doesn't trip on
  intentional contact, and that `is_on_hard_limit()` correctly reads
  `status.joint[n]['min_hard_limit']`/`['max_hard_limit']` as a signal
  distinct from `is_faulted()`. See
  [docs/hardware_setup.md](../../docs/hardware_setup.md) sections 5 and 7
  for the behavior this is pinning down, and why it matters.

- **`test_axis.py`** -- `Axis.poll()`: that a hard-limit-switch trip
  outside of homing moves the axis to `FAULT` (via `is_on_hard_limit()`),
  that the same condition is *not* treated as a fault while that axis's
  own state is `HOMING` (driving onto the switch it's searching for is
  expected there), and that this didn't change the pre-existing
  following-error fault behavior (`is_faulted()`).

- **`conftest.py`** -- installs the fake `linuxcnc`/`hal` modules (below)
  into `sys.modules` before any test module is collected, so
  `lcaf.control.linuxcnc_interface` (the only module that imports the real
  ones) can be imported here at all. Runs once per test session.

- **`fake_linuxcnc.py`** -- not a test file itself. Minimal stand-ins for
  the `linuxcnc`/`hal` compiled extension modules, which only exist inside
  a real LinuxCNC install (see
  [docs/software_setup.md](../../docs/software_setup.md)) and can't be
  installed here. These model LinuxCNC's Python *API surface* (what
  methods exist, what they're called with) well enough to exercise this
  project's own call sequencing -- they do not model LinuxCNC's actual
  servo-thread/hard-limit logic (`control.c`). Passing tests here confirm
  this project's code does the right thing *if* LinuxCNC behaves the way
  its source/docs say it does; they are not a substitute for the real
  end-to-end hardware check in
  [docs/hardware_setup.md](../../docs/hardware_setup.md) section 11. See
  also [docs/potential_issues.md](../../docs/potential_issues.md).
