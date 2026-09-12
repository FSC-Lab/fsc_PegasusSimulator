#!/usr/bin/env python3
"""Do the controllers and the SIM agree on what a normalised surface command MEANS?

`test_controller_aero_match.py` feeds deflections in RADIANS to both models, so it verifies the
aero FUNCTIONS agree. It never exercises the step before that: controllers emit a NORMALISED
command in [-1,1] on the actuator channel, and the sim multiplies it by its own limit to get
radians. A disagreement there is invisible to every existing test and to both the offline gate and
HIL -- it just silently scales the delivered moment.

It had one: DR_MAX was radians(30) while the sim applies radians(25) to ALL surfaces, so every
yaw-moment demand was delivered at 25/30 = 83% of what was asked.
"""
import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# The flight controller lives in fsc_autopilot_ros2 now, not in this repo.
# controller_path resolves it (workspace symlink, or $FSC_CONTROLLER_DIR), so a
# future move of that repository needs no change here.
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import controller_dir, inner_dir   # noqa: E402
_CTRL = controller_dir()
_CTRL_INNER = inner_dir()

sys.path.insert(0, os.path.join(_CTRL,
                                "px4_vtol_direct_actuator"))
AERO = os.path.join(HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
                    "logic", "dynamics", "vtol_aerodynamics.py")
VTOL = os.path.join(HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
                    "logic", "vehicles", "vtol.py")


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def main():
    aero = load(AERO, "aero_mod")
    sim_rad = float(aero.VTOLAerodynamicsConfig().max_surface_deflection_rad)

    import nmpc_so3_transition_node as so3
    K = so3.SO3TransitionNMPC
    ctrl = {"aileron": K.DA_MAX, "elevator": K.DE_MAX, "rudder": K.DR_MAX}

    print(f"  SIM applies {math.degrees(sim_rad):.1f} deg to a full [-1,1] command, ALL surfaces")
    bad = []
    for nm, v in ctrl.items():
        ratio = v / sim_rad
        flag = "" if abs(ratio - 1.0) < 1e-6 else \
               f"   <-- delivers {1.0/ratio*100:.0f}% of the demanded moment"
        print(f"  controller {nm:9s} {math.degrees(v):5.1f} deg{flag}")
        if abs(ratio - 1.0) > 1e-6:
            bad.append(f"{nm}: controller {math.degrees(v):.1f} deg vs sim "
                       f"{math.degrees(sim_rad):.1f} deg -> moments delivered at "
                       f"{1.0/ratio*100:.0f}% of demand")

    # the airframe definitions must agree too -- a per-surface override would reintroduce this
    src = open(VTOL).read()
    import re
    degs = set(re.findall(r"max_deflection_deg=([0-9.]+)", src))
    print(f"  vtol.py surface max_deflection_deg values present: {sorted(degs)}")
    for d in degs:
        if abs(float(d) - math.degrees(sim_rad)) > 1e-6:
            bad.append(f"vtol.py has a surface at {d} deg but the aero model applies "
                       f"{math.degrees(sim_rad):.1f} deg to every surface")

    if bad:
        print("\nNORMALISATION MISMATCH:")
        for b in bad:
            print("   -", b)
        return 1
    print("\nSURFACE NORMALISATION MATCHES THE SIM")
    return 0


if __name__ == "__main__":
    sys.exit(main())
