#!/usr/bin/env python3
"""The PLANT's propeller and the CONTROLLER's propeller must be the same propeller.

    python3 tests/test_prop_pairing.py

The momentum thrust model derives CT0 so that STATIC thrust is preserved for any
(max_thrust, rotor_omega_max, diameter) -- which is exactly why a mismatch hides:
hover is identical, and only the advance ratio J = V_ax / (n D), i.e. the thrust
at AIRSPEED, differs. The flown FSC quad ran the VTOLConfig default of 1000 rad/s
while the controller, the offline gate, tests/plant_double.py and
tests/test_actuator_models.py all model 749 (bag slosh_20260913_103526: cruise
0.45 m/s above command on less throttle than the gate needs -- the prop at 0.80
throttle and 16.5 m/s makes 3.5 N in the sim where the controller expects 0.7).
Same class as the mass pairing: the number the plant flies must be the number
the controller inverts, in the same commit.

Checked here, for every rotor of the flown airframe, against the controller's
`nmpc_cascade_node` constants: thrust model, diameter, zero-thrust advance
ratio, omega_max, max_thrust -- and the resulting thrust at 16 m/s, computed by
BOTH sides' own functions.
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_slosh_import import stub_isaac, load          # noqa: E402
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import controller_dir              # noqa: E402

CTRL = os.path.join(controller_dir(), "px4_vtol_direct_actuator", "nmpc_cascade_node.py")


def controller_prop():
    """PROP_D, PROP_J0, OMEGA_MAX, MAX_THRUST -- parsed, not imported (importing pulls casadi)."""
    src = open(CTRL).read()
    out = {}
    for k in ("PROP_D", "PROP_J0", "OMEGA_MAX"):
        m = re.search(rf"^{k}\s*=\s*([0-9.]+)", src, re.M)
        assert m, k
        out[k] = float(m.group(1))
    m = re.search(r"^MAX_THRUST\s*=\s*([0-9.]+)", src, re.M)
    if m:
        out["MAX_THRUST"] = float(m.group(1))
    else:   # imported from the model hub
        hub = open(os.path.join(controller_dir(), "px4_vtol_direct_actuator",
                                "nmpc_so3_transition_node.py")).read()
        m = re.search(r"^MAX_THRUST\s*=\s*([0-9.]+)", hub, re.M)
        assert m, "MAX_THRUST"
        out["MAX_THRUST"] = float(m.group(1))
    return out


def thrust_momentum(T_max, omega_max, D, J0, throttle, v_ax):
    n_max = omega_max / (2.0 * math.pi)
    ct0 = T_max / (1.225 * n_max ** 2 * D ** 4)
    n = throttle * n_max
    return max(0.0, 1.225 * D * D * (ct0 * (n * D) ** 2 - (ct0 / J0) * (n * D) * v_ax))


def main():
    stub_isaac()
    load("pegasus.simulator.logic.dynamics.vtol_aerodynamics", "dynamics/vtol_aerodynamics.py")
    vtol = load("pegasus.simulator.logic.vehicles.vtol", "vehicles/vtol.py")
    cfg = vtol.quad_tiltrotor_config()
    c = controller_prop()
    bad = []
    if getattr(cfg, "thrust_model", "instant") != "momentum":
        bad.append(f"airframe thrust_model={cfg.thrust_model!r}; the controller inverts a momentum prop")
    if abs(cfg.rotor_omega_max - c["OMEGA_MAX"]) > 1e-6:
        bad.append(f"rotor_omega_max: airframe {cfg.rotor_omega_max} vs controller OMEGA_MAX {c['OMEGA_MAX']}")
    for r in cfg.rotors:
        D, J0 = vtol.prop_spec(r.prop, r.prop_diameter_m, r.prop_zero_thrust_J)
        if abs(D - c["PROP_D"]) > 1e-9:
            bad.append(f"{r.name}: diameter {D} vs controller PROP_D {c['PROP_D']}")
        if abs(J0 - c["PROP_J0"]) > 1e-9:
            bad.append(f"{r.name}: J0 {J0} vs controller PROP_J0 {c['PROP_J0']}")
        if abs(r.max_thrust - c["MAX_THRUST"]) > 1e-9:
            bad.append(f"{r.name}: max_thrust {r.max_thrust} vs controller MAX_THRUST {c['MAX_THRUST']}")
        # the number that actually matters: thrust at cruise airspeed, both sides' own formula
        for thr in (0.6, 0.8, 1.0):
            t_sim = thrust_momentum(r.max_thrust, cfg.rotor_omega_max, D, J0, thr, 16.0)
            t_ctl = thrust_momentum(c["MAX_THRUST"], c["OMEGA_MAX"], c["PROP_D"], c["PROP_J0"], thr, 16.0)
            if abs(t_sim - t_ctl) > 0.05:
                bad.append(f"{r.name}: thrust at throttle {thr}, 16 m/s: sim {t_sim:.2f} N vs controller {t_ctl:.2f} N")
    print(f"airframe: {cfg.thrust_model}, omega_max {cfg.rotor_omega_max} rad/s, "
          f"{len(cfg.rotors)} rotors x {cfg.rotors[0].max_thrust} N, prop {cfg.rotors[0].prop} "
          f"-> at 0.8 throttle / 16 m/s: {thrust_momentum(cfg.rotors[0].max_thrust, cfg.rotor_omega_max, *vtol.prop_spec(cfg.rotors[0].prop), 0.8, 16.0):.2f} N per rotor")
    print(f"controller: PROP_D {c['PROP_D']}, J0 {c['PROP_J0']}, OMEGA_MAX {c['OMEGA_MAX']}, MAX_THRUST {c['MAX_THRUST']}")
    for b in bad:
        print("FAIL", b)
    print("PASS: plant and controller fly the same propeller" if not bad else f"{len(bad)} mismatch(es)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
