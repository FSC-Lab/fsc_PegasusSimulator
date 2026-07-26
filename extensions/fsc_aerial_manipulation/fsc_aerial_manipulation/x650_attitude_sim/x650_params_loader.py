#!/usr/bin/env python
"""
| File: x650_params_loader.py
| License: BSD-3-Clause
| Description: Loads rotorcraft/x650_params.py by file path via importlib,
|   bypassing `fsc_aerial_manipulation.rotorcraft`'s __init__.py.

x650_params.py itself has no Isaac Sim dependency (see its own docstring),
but it lives inside the `rotorcraft` package, and Python always executes a
package's __init__.py before any of its submodules -- `rotorcraft/__init__.py`
imports `rotorcraft_utils`, which imports `pegasus.simulator....` at module
scope. That import chain is fine/expected in an Isaac Sim session (this
whole repo assumes Isaac Sim for real use), but it means the normal
`from fsc_aerial_manipulation.rotorcraft import x650_params` fails outside
one. This module exists so `x650_attitude_sim` (a deliberately standalone,
no-Isaac-Sim-required validation package -- see this package's own
__init__.py) can reach the same numbers without requiring Isaac Sim.

Not a general-purpose pattern -- just the minimal fix for this one
cross-package need. If `rotorcraft/__init__.py`'s eager Isaac-Sim-only
import is ever made lazy/optional, this loader becomes unnecessary and a
plain import can replace it.
"""
import importlib.util
import os

_X650_PARAMS_PATH = os.path.join(
    os.path.dirname(__file__), "..", "rotorcraft", "x650_params.py"
)


def load_x650_params():
    spec = importlib.util.spec_from_file_location("x650_params", os.path.abspath(_X650_PARAMS_PATH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
