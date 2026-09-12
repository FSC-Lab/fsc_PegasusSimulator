#!/usr/bin/env python3
"""The slosh model copy CONSTRUCTS, and constructs as the baseline aircraft.

    python3 tests/test_slosh_import.py

`tests/test_slosh_scaffold.py` reads the copy with `ast` -- it can prove what the
source says, not that the source runs.  The things most likely to break at
launch time are exactly the runtime ones: a dataclass subclass whose fields do
not line up, a field-wise config copy that drops something, a refusal that never
fires.  Isaac cannot be imported here, so `pxr`, `omni.isaac.dynamic_control` and
the pegasus package chain are STUBBED and the real `vtol.py` / `vtol_slosh.py`
are then imported and exercised.

WHAT THIS DOES AND DOES NOT PROVE.  It proves the Python semantics: the classes
build, the config copy is faithful, the step order is the baseline's, and a
configured tank is refused.  It does NOT prove anything about USD, PhysX or the
forces -- only a real Isaac launch does that (VTOL_SLOSH=1 on scene 03).

If this file starts failing after an edit to `vtol.py`, the likely cause is a
NEW Isaac import at the top of that module: add it to the stub list below.
"""
import importlib.util
import os
import sys
import types
from dataclasses import fields
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
LOGIC = os.path.join(HERE, "..", "extensions", "pegasus.simulator", "pegasus",
                     "simulator", "logic")


def stub_isaac():
    """Everything `vtol.py` imports that only exists inside Isaac Sim."""
    for name in ("pxr", "omni", "omni.isaac", "omni.isaac.dynamic_control"):
        sys.modules[name] = types.ModuleType(name)
    for attr in ("Gf", "Sdf", "UsdGeom", "UsdPhysics", "PhysxSchema"):
        setattr(sys.modules["pxr"], attr, mock.MagicMock())
    sys.modules["omni.isaac.dynamic_control"]._dynamic_control = mock.MagicMock()

    def mkpkg(name):
        m = types.ModuleType(name)
        m.__path__ = []
        sys.modules[name] = m
        return m

    for n in ("pegasus", "pegasus.simulator", "pegasus.simulator.logic",
              "pegasus.simulator.logic.vehicles", "pegasus.simulator.logic.dynamics",
              "pegasus.simulator.logic.sensors"):
        mkpkg(n)

    class Vehicle:                               # stand-in for the pegasus base class
        def __init__(self, **kw):
            self.kw = kw

    mkpkg("pegasus.simulator.logic.vehicles.vehicle").Vehicle = Vehicle
    for s in ("Barometer", "GPS", "IMU", "Magnetometer"):
        setattr(sys.modules["pegasus.simulator.logic.sensors"], s, mock.MagicMock())


def load(modname, relpath):
    spec = importlib.util.spec_from_file_location(modname, os.path.join(LOGIC, relpath))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[modname] = mod                   # dataclasses resolve through sys.modules
    spec.loader.exec_module(mod)
    return mod


def main():
    bad = []
    stub_isaac()
    load("pegasus.simulator.logic.dynamics.vtol_aerodynamics", "dynamics/vtol_aerodynamics.py")
    slosh = load("pegasus.simulator.logic.dynamics.slosh", "dynamics/slosh.py")
    vtol = load("pegasus.simulator.logic.vehicles.vtol", "vehicles/vtol.py")
    vs = load("pegasus.simulator.logic.vehicles.vtol_slosh", "vehicles/vtol_slosh.py")

    # 1. the config subclass carries every baseline field, with identical defaults
    base, sl = vtol.VTOLConfig(), vs.VTOLSloshConfig()
    for f in fields(base):
        if repr(getattr(base, f.name)) != repr(getattr(sl, f.name)):
            bad.append(f"VTOLSloshConfig default differs on {f.name}")
    if len(fields(sl)) != len(fields(base)) + 1:
        bad.append(f"VTOLSloshConfig should add exactly one field (tank); "
                   f"it has {len(fields(sl))} vs {len(fields(base))}")
    print(f"  1. VTOLSloshConfig: {len(fields(base))} inherited fields + tank, defaults identical")

    # 2. the field-wise copy `fsc_cad_vtol_slosh_config` performs keeps the airframe intact
    airframe = vtol.quad_tiltrotor_config()
    copy = vs.VTOLSloshConfig(**{f.name: getattr(airframe, f.name) for f in fields(airframe)})
    for f in fields(airframe):
        a, b = getattr(airframe, f.name), getattr(copy, f.name)
        if a is not b and repr(a) != repr(b):
            bad.append(f"field-wise config copy lost {f.name}")
    if copy.rotors is not airframe.rotors:
        bad.append("field-wise copy re-created the rotor list instead of sharing it")
    print(f"  2. field-wise copy: {len(copy.rotors)} rotors, {len(copy.surfaces)} surfaces, "
          f"mass {copy.mass} -- all carried")

    # 3. no tank -> constructs, and the per-step order is still the baseline's
    v = vs.VTOLSlosh(config=copy)
    if not isinstance(v, vtol.VTOL):
        bad.append("VTOLSlosh is not a VTOL")
    calls = []
    v._read_command = lambda: {}
    v._validate_channels = lambda c: None
    v._drive_actuators = lambda c, dt: calls.append("actuators")
    v._apply_propulsion = lambda c: calls.append("propulsion")
    v._apply_aerodynamics = lambda c: calls.append("aerodynamics")
    v._backends = []
    v._cfg.enable_forces = True
    v.update(0.008)
    if calls != ["actuators", "propulsion", "aerodynamics"]:
        bad.append(f"the baseline step order changed: {calls}")
    print(f"  3. VTOLSlosh(tank=None).update -> {calls} + _apply_slosh (no-op)")

    # 4. a configured tank must be refused at CONSTRUCTION
    try:
        vs.VTOLSlosh(config=vs.VTOLSloshConfig(tank=slosh.TankConfig()))
        bad.append("a configured tank was accepted -- phase 1 has no slosh physics to fly")
    except NotImplementedError:
        print("  4. a configured tank is refused at construction, not mid-flight")

    # 5. an explicitly disabled tank is allowed through, with its parameters derived
    v2 = vs.VTOLSlosh(config=vs.VTOLSloshConfig(tank=slosh.TankConfig(enabled=False)))
    if v2._tank is None or v2._slosh is None:
        bad.append("a disabled tank lost its config or its derived parameters")
    print("  5. tank(enabled=False) constructs and keeps its derived parameters")

    print()
    if bad:
        print(f"FAIL ({len(bad)}):")
        for b in bad:
            print("  -", b)
        return 1
    print("PASS -- the copy constructs as the baseline aircraft")
    return 0


if __name__ == "__main__":
    sys.exit(main())
