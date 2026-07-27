"""
Installs fake `linuxcnc`/`hal` modules (debug/tests/fake_linuxcnc.py) into
sys.modules before any test module is collected. lcaf.control.linuxcnc_interface
is the only module that imports the real `linuxcnc`/`hal` extensions, and
those only exist inside a LinuxCNC install (see docs/software_setup.md) --
without this, test_linuxcnc_interface.py could not import it at all.
"""

from __future__ import annotations

from fake_linuxcnc import install_fake_linuxcnc_modules

install_fake_linuxcnc_modules()
