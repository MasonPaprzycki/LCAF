"""
Minimal stand-ins for the `linuxcnc`/`hal` compiled extension modules.

The real modules only exist inside a LinuxCNC install (see
docs/software_setup.md) -- they are not installable here, so
lcaf.control.linuxcnc_interface (the only module that imports them) cannot
be imported at all without something in sys.modules named `linuxcnc`/`hal`
first. These fakes exist purely so this repo's own test suite can exercise
that module's *logic* (homing sequencing, limit-switch polling, HAL pin
reads) deterministically, without a real LinuxCNC instance. They do not
attempt to model LinuxCNC's actual trajectory planner, servo thread, or
hard-limit fault handling -- see debug/tests/test_linuxcnc_interface.py for
what each test actually asserts about the calls made into these fakes.
"""

from __future__ import annotations

import sys
import types


# --- linuxcnc ----------------------------------------------------------

MODE_MANUAL = 1
MODE_MDI = 3
JOG_STOP = 0
JOG_CONTINUOUS = 1
STATE_ESTOP = 1
STATE_ESTOP_RESET = 2
STATE_OFF = 3
STATE_ON = 4
INTERP_READING = 2


class FakeCommand:
    def __init__(self):
        self.calls: list[tuple] = []
        self.override_limits_calls = 0

    def _record(self, name, *args):
        self.calls.append((name, *args))

    def mode(self, mode):
        self._record("mode", mode)

    def teleop_enable(self, enable):
        self._record("teleop_enable", enable)

    def wait_complete(self):
        self._record("wait_complete")

    def jog(self, kind, is_joint, joint_num, speed=None):
        self._record("jog", kind, is_joint, joint_num, speed)

    def home(self, joint):
        self._record("home", joint)

    def state(self, state):
        self._record("state", state)

    def mdi(self, command):
        self._record("mdi", command)

    def abort(self):
        self._record("abort")

    def override_limits(self):
        self.override_limits_calls += 1
        self._record("override_limits")


class FakeJointStatus(dict):
    """dict subclass so status.joint[n]['fault'] indexing works like the real stat channel."""


class FakeStat:
    def __init__(self, num_joints: int = 4):
        self.task_state = STATE_ON
        self.estop = 0
        self.interp_state = INTERP_READING
        self.joint_actual_position = [0.0] * num_joints
        self.joint_position = [0.0] * num_joints
        self.joint = [
            FakeJointStatus(
                enabled=True,
                inpos=True,
                fault=False,
                velocity=0.0,
                homed=False,
                min_hard_limit=False,
                max_hard_limit=False,
            )
            for _ in range(num_joints)
        ]

    def poll(self):
        pass


class FakeErrorChannel:
    def poll(self):
        return None


class FakeLinuxCNCModule(types.ModuleType):
    def __init__(self):
        super().__init__("linuxcnc")
        self.MODE_MANUAL = MODE_MANUAL
        self.MODE_MDI = MODE_MDI
        self.JOG_STOP = JOG_STOP
        self.JOG_CONTINUOUS = JOG_CONTINUOUS
        self.STATE_ESTOP = STATE_ESTOP
        self.STATE_ESTOP_RESET = STATE_ESTOP_RESET
        self.STATE_OFF = STATE_OFF
        self.STATE_ON = STATE_ON
        self.INTERP_READING = INTERP_READING
        self._last_command: FakeCommand | None = None
        self._last_stat: FakeStat | None = None

    def command(self):
        self._last_command = FakeCommand()
        return self._last_command

    def stat(self):
        self._last_stat = FakeStat()
        return self._last_stat

    def error_channel(self):
        return FakeErrorChannel()


# --- hal -----------------------------------------------------------------

class FakeHALComponent:
    """
    Stand-in for the object hal.component(name) returns. Real HAL requires
    ready() before the component is usable and refuses hal.get_value() calls
    anywhere in the process until some component has been created -- see
    LinuxCNCMachineInterface.__init__.
    """

    def __init__(self, name):
        self.name = name
        self._ready = False

    def ready(self):
        self._ready = True


class FakeHALModule(types.ModuleType):
    """
    hal.get_value(pin_name) reads from a plain dict of pin name -> value
    that tests set up directly (e.g. fake_hal.pins["joint.0.neg-lim-sw-in"]
    = True) to simulate a physical switch tripping.
    """

    def __init__(self):
        super().__init__("hal")
        self.pins: dict[str, object] = {}
        self._component_created = False

    def component(self, name):
        self._component_created = True
        return FakeHALComponent(name)

    def get_value(self, pin_name):
        if not self._component_created:
            raise RuntimeError("Cannot call before creating component")
        return self.pins.get(pin_name, False)


def install_fake_linuxcnc_modules() -> tuple[FakeLinuxCNCModule, FakeHALModule]:
    """
    Install fresh fake `linuxcnc`/`hal` modules into sys.modules and return
    them, so lcaf.control.linuxcnc_interface can be imported and its calls
    into "linuxcnc"/"hal" inspected. Safe to call more than once (each call
    replaces the previous fakes with new ones).
    """

    fake_linuxcnc = FakeLinuxCNCModule()
    fake_hal = FakeHALModule()

    sys.modules["linuxcnc"] = fake_linuxcnc
    sys.modules["hal"] = fake_hal

    return fake_linuxcnc, fake_hal
