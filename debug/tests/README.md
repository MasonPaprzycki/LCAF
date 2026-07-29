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
  `generate_hal()` actually emits the negative-limit HAL net from the real
  `configs/axis.json` (X/Y/Z are all single-switch there -- no positive
  net to check). Also covers `dual_limit_switches`: rejected combinations
  (dual mode without a positive input, single-switch mode with one), and
  that `generate_hal(simulate=True)` doesn't fabricate a simulated positive
  switch for a single-switch joint. Pure text generation, no `linuxcnc`/
  `hal` import, so no fakes needed.

- **`test_linuxcnc_interface.py`** -- `LinuxCNCAxialInterface`'s native
  homing path (this project always homes via LinuxCNC's own native homing
  sequence -- see `docs/hardware_setup.md` section 7): `start_homing()`
  calls `command.home(joint)`, `poll_homing()` waits (without blocking) for
  `status.joint[n]['homed']`, raises on a reported fault or on timeout, and
  never remeasures `max_limit` regardless of `dual_limit_switches`. Also
  covers `is_on_hard_limit()` reading
  `status.joint[n]['min_hard_limit']`/`['max_hard_limit']` as a signal
  distinct from `is_faulted()`, and a `dual_limit_switches=False` joint's
  homing (still just trusts configured `extended_distance`, same as the
  dual-switch case). `RetractToZeroTests` covers
  `start_retract_to_zero()`/`poll_retract_to_zero()`: refuses to run before
  initial homing or on a switchless joint, reruns native `command.home()`,
  and never remeasures `max_limit`. `RetractToTests` covers the
  `retract_to` exception (this project's Y, set to `1.5`): retract instead
  commands a plain MDI move to `retract_to`, completes on `inpos` rather
  than a re-home, and never touches `position_offset_to_native`/
  `max_limit`. See [docs/hardware_setup.md](../../docs/hardware_setup.md)
  sections 5 and 7 for the behavior this is pinning down, and why it
  matters.

- **`test_axis.py`** -- `Axis.poll()`: that a hard-limit-switch trip
  outside of homing moves the axis to `FAULT` (via `is_on_hard_limit()`),
  that the same condition is *not* treated as a fault while that axis's
  own state is `HOMING` or `RETRACTING` (driving onto the switch it's
  searching for is expected there), and that this didn't change the
  pre-existing following-error fault behavior (`is_faulted()`). Also covers
  `retract_to_zero()`/`is_retracted()`: completes once the negative switch
  trips, and resets on the next retract call.

- **`test_motion_coordinator.py`** -- `MotionCoordinator`'s
  `RETRACT_Z`/`RETRACT_Y`/`RETRACT_X` states: that each one drives the
  matching `Axis` into `RETRACTING` (not `MOVING`) via `retract_axis()`,
  that `VERIFY_*_RETRACTED` waits for `Axis.is_retracted()` before
  advancing, that the full Z->Y->X sequence reaches `ROTATE_A`, and that a
  joint which never completed initial homing surfaces as a motion fault
  instead of silently moving. `ConcurrentHomingTests` covers `home_all()`:
  every axis's own `command.home()` is issued in the same heartbeat (not
  one at a time), each axis homes and backs off to its own
  `retracted_distance` fully independently of the others, and
  `all_homed()` only reports True once every axis has finished.

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
