#!/usr/bin/env python3
"""The PLANT's tilt servo range and the CONTROLLER's tilt bounds must be the same range.

    python3 tests/test_tilt_limit_pairing.py

The FSC quad's front nacelles stop at tilt_min_deg = -20 (vtol.py quad_tiltrotor_config,
enabled 2026-09-07 for the back-transition's aft braking). The cascade's outer solver bounds
its tilt control at -BETA_AFT_MAX, and that constant was 25 deg: the outer PLANNED -25, the
node's own applied-tilt estimate followed the plan to -25, and the plant stopped at -20 --
24 N of front thrust x (sin 25 - sin 20) = 1.9 N of braking that never arrived, at the
low-speed end of the stop where the aft tilt is the only authority left. Flight 8 of
2026-09-14 (bag slosh_20260914_132154, 100 mm tank 0.3 m ahead): the plant lagged the ramp by
1.4 m/s, the outer pitched nose-down, the stop rang at ~1 Hz. scripts/ana_force_model.py shows
the +1.4..+1.95 N forward residual switching on exactly when the command passes -20 deg. The
offline gate could not see it because so3.build_ode chases any tilt it is handed (the gate's
plant now has `--plant_tilt_min`). Same class as the propeller and mass pairings: the number
the plant enforces must be the number the controller plans with, in the same commit.

Checked here: every tiltable rotor's [tilt_min_deg, tilt_max_deg] against the cascade's
BETA_AFT_MAX and the 90 deg forward limit, the schedule's beta_aft_deg default inside it,
and the gate's --plant_tilt_min default against the servo; the rear pair must be fixed.
"""
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_slosh_import import stub_isaac, load          # noqa: E402
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import controller_dir, inner_dir, load_airframe   # noqa: E402

CTRL = os.path.join(controller_dir(), "px4_vtol_direct_actuator", "nmpc_cascade_node.py")


def controller_limits():
    src = open(CTRL).read()
    # since piece 3 (2026-09-15) the aft limit and the gate's plant default both read the
    # airframe file's tilt.min_deg (text pinned); the schedule's beta_aft_deg is a CHOICE and
    # stays a literal, compared to the servo below
    af = load_airframe()
    assert re.search(r"^BETA_AFT_MAX = math\.radians\(-float\(so3\._AF\.tilt\.min_deg\)\)", src, re.M), "BETA_AFT_MAX"
    aft = -float(af.tilt.min_deg)
    m = re.search(r'P\("beta_aft_deg",\s*([0-9.]+)\)', src)
    assert m, 'P("beta_aft_deg")'
    sched = float(m.group(1))
    assert re.search(r'argf\("--plant_tilt_min", -float\(so3\._AF\.tilt\.min_deg\)\)', src), "--plant_tilt_min"
    gate = -float(af.tilt.min_deg)
    # the solver's lower bound must be the same constant, not a literal
    assert re.search(r"self\.lbx\[self\.iU \+ self\.nu \* k \+ 2\]\s*=\s*-BETA_AFT_MAX", src), \
        "the outer's tilt lower bound is not -BETA_AFT_MAX"
    # and the node's applied-tilt estimate must be clipped to it
    assert re.search(r"self\._beta = _cl\(self\._beta \+ _cl\(beta_cmd - self\._beta, -slew_up, slew_up\),\s*\n\s*-BETA_AFT_MAX, math\.pi / 2\.0\)", src), \
        "the node's applied-tilt estimate is not clipped to [-BETA_AFT_MAX, pi/2]"
    return aft, sched, gate


def main():
    stub_isaac()
    load("pegasus.simulator.logic.dynamics.vtol_aerodynamics", "dynamics/vtol_aerodynamics.py")
    vtol = load("pegasus.simulator.logic.vehicles.vtol", "vehicles/vtol.py")
    cfg = vtol.quad_tiltrotor_config()
    aft, sched, gate = controller_limits()
    bad = []
    tilting = [r for r in cfg.rotors if r.tiltable]
    if len(tilting) != 2:
        bad.append(f"{len(tilting)} tiltable rotors (the cascade tilts the FRONT pair)")
    for r in tilting:
        if abs(-r.tilt_min_deg - aft) > 1e-9:
            bad.append(f"{r.name}: servo tilt_min {r.tilt_min_deg} deg vs controller BETA_AFT_MAX {-aft} deg")
        # The invariant is CONTAINMENT, not equality: the controller's planned range must fit
        # inside the servo's travel. A servo that goes further than the loop plans is headroom
        # (the MN_4014 build's nacelle reaches +100 deg, but past 90 the thrust vectors forward
        # and DOWN, which no schedule wants). The failure that matters is the reverse, and it has
        # happened: the outer planned -25 deg of aft tilt on a servo that stops at -20, and the
        # 1.9 N of braking it counted on never arrived (2026-09-14).
        if r.tilt_max_deg < 90.0 - 1e-9:
            bad.append(f"{r.name}: servo tilt_max {r.tilt_max_deg} deg is SHORT of the 90 the controller plans")
        # ★ THE SCALE (2026-10-27). The plant flies cmd x tilt_max_deg (vtol.py _tilt_target_rad), so the
        # controllers' servo command must divide by THIS rotor's travel: airframe_id.tilt_to_servo divides
        # by the file's tilt.max_deg. They differed by 100/90 from 2026-09-17 (every HIL cruise ~10 deg
        # past the controller's belief, found by ana_force_model --tilt_max); hold them equal.
        _af = load_airframe(os.path.join(inner_dir(), "airframe_id.cfg"))
        if abs(float(_af.tilt.max_deg) - r.tilt_max_deg) > 1e-9:
            bad.append(f"{r.name}: servo travel {r.tilt_max_deg} deg vs airframe_id.cfg tilt.max_deg {_af.tilt.max_deg}: "
                       f"the controllers' servo command (beta / max_deg) lands at the wrong angle")
    # and every node that PUBLISHES a tilt does it through that one function (a `beta / (pi/2)` left
    # anywhere is the 2026-09-17..10-27 mismatch again)
    for node in ("nmpc_cascade_node.py", "nmpc6_cruise_node.py", "nmpc_full_smc_node.py", "nmpc_so3_cascade_node.py",
                 "nmpc_so3_transition_node.py"):
        src = open(os.path.join(inner_dir(), node)).read()
        if re.search(r"(_beta|beta|betacmd|_beta_cmd|tilt)\s*/\s*\(math\.pi\s*/\s*2(\.0)?\)", src):
            bad.append(f"{node} still divides a tilt by 90 deg for the servo; use tilt_to_servo (the plant scales by its travel)")
    if len([r for r in cfg.rotors if not r.tiltable]) != 2:
        bad.append("the rear pair must be fixed (the cascade's allocation holds it vertical)")
    if sched > aft + 1e-9:
        bad.append(f"schedule beta_aft_deg {sched} exceeds BETA_AFT_MAX {aft}: the reference asks a tilt the bound refuses")
    servo_aft = -tilting[0].tilt_min_deg if tilting else float("nan")
    if abs(gate - servo_aft) > 1e-9:
        bad.append(f"gate --plant_tilt_min default {gate} vs the airframe's servo {servo_aft}: the gate's plant is not the plant")
    print(f"airframe: front pair tilts {tilting[0].tilt_min_deg:+.0f}..{tilting[0].tilt_max_deg:+.0f} deg; "
          f"controller: BETA_AFT_MAX {aft:.0f}, schedule beta_aft_deg {sched:.0f}, gate plant_tilt_min {gate:.0f}")
    for b in bad:
        print("FAIL", b)
    print("PASS: plant and controller agree on the tilt range" if not bad else f"{len(bad)} mismatch(es)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
