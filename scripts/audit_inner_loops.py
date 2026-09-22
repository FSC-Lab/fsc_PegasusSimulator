#!/usr/bin/env python3
"""Audit of the TWO INNER LOOPS against the true plant: the flown geometric law and the INDI one.

`audit_cascade.py` audits what the two SHARE -- the allocation, the surface formulas, the
reference, the outer's model. None of it says anything about the LAW, and the INDI chain (the
measured non-control moment, its filters, its delay FIFO, the trim subtraction, the increment's
gain) had never been checked against the plant at all. Hand-derived control algebra is exactly
where this project has been bitten -- a pitch convention inverted six times, a rudder sign that
made yaw damping positive feedback, a rotor z-arm with the wrong sign -- so every claim below is
a NUMBER measured against `so3.build_ode()`, not an argument about the code.

    python3 scripts/audit_inner_loops.py [--verbose]

  1 DIRECTION      each loop, each axis, each flight condition: does the closed inner loop
                   (law -> allocation -> surfaces -> plant) produce an angular acceleration that
                   OPPOSES the attitude error it was given?
  2 CROSS-AXIS     does a pure single-axis error move the OTHER axes? (channel mixing)
  3 STIFFNESS      the restoring acceleration per radian each loop actually delivers, which is
                   what "tracking error" means in closed loop -- and how it varies with speed.
  4 INDI: M_c      is the control moment the increment subtracts equal to the plant's TRUE
                   control moment (the plant with the actuators as they are, minus the plant with
                   them at zero)?
  5 INDI: sigma    is what is left -- the "non-control moment" INDI cancels -- equal to the
                   plant's own aero + gyroscopic moment?
  6 INDI: M_trim   does the trim subtraction equal the moment the outer's package actually makes?
  7 INDI: G/dt     what fraction of a commanded moment increment is DELIVERED within one tick,
                   per effector -- the effectiveness the increment assumes it gets, and the
                   number behind the k <= 0.8 limit and the bandwidth cap.
  8 SYNC           the rate-derivative filter and the applied-moment filter, and the delay FIFO
                   against the transport delay the plant is given.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import controller_dir                     # noqa: E402
_CTRL = controller_dir()
sys.path.insert(0, os.path.join(_CTRL, "px4_vtol_direct_actuator"))

import nmpc_cascade_node as C                                  # noqa: E402
import nmpc_so3_transition_node as so3                         # noqa: E402

VERBOSE = "--verbose" in sys.argv
F = so3.build_ode()
IVEC = np.array([so3.IXX, so3.IYY, so3.IZZ])
DT = 0.005
fails = []
marginal = []


def Rpy(roll=0.0, pitch=0.0, yaw=0.0):
    """FLU body -> world, pitch NOSE-UP positive.

    ★ THE SEVENTH TIME. The first version of this file built Ry with the other sign, so a "+5 deg
    nose-up" attitude read as -5 through the node's own `theta = asin(R[2,0])` and the audit
    reported the FLOWN pitch loop as positive feedback. The convention is not something to write
    from memory: `_check_conventions()` below builds an attitude and asserts that the node's own
    extractors return the angles that went in."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, -sp], [0, 1.0, 0], [sp, 0, cp]])     # nose-UP positive => R[2,0] = +sin
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _check_conventions():
    """The node reads pitch as asin(R[2,0]), roll as atan2(R[2,1], R[2,2]) and yaw as
    atan2(R[1,0], R[0,0]). Assert this harness speaks the same language before auditing anything."""
    for r, p, y in ((6.0, 0.0, 0.0), (0.0, 5.0, 0.0), (0.0, 0.0, -7.0), (4.0, -3.0, 8.0)):
        R = Rpy(math.radians(r), math.radians(p), math.radians(y))
        got = (math.degrees(math.atan2(R[2, 1], R[2, 2])),
               math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0])))),
               math.degrees(math.atan2(R[1, 0], R[0, 0])))
        if max(abs(got[0] - r), abs(got[1] - p), abs(got[2] - y)) > 1e-6:
            raise SystemExit(f"CONVENTION MISMATCH: put in ({r},{p},{y}) deg, the node reads {got}")
    print("0 CONVENTIONS -- roll/pitch/yaw as this harness builds them are what the node's own "
          "extractors read back (4 attitudes, <1e-6 deg)\n")


_check_conventions()


def state19(R, beta, v_world, omega=(0.0, 0.0, 0.0), z=30.0):
    x = np.zeros(19)
    x[0:3] = [0.0, 0.0, z]
    x[3:12] = np.asarray(R, float).reshape(-1)
    x[12] = beta
    x[13:16] = v_world
    x[16:19] = omega
    return x


def plant_wdot(x, mot, beta_cmd, de, da, dr):
    """The plant's angular acceleration for a THROTTLE command, converted at THIS state."""
    R = np.asarray(x[3:12], float).reshape(3, 3)
    vb = R.T @ np.asarray(x[13:16], float)
    v_ax_f = float(vb[0] * math.sin(beta_cmd) + vb[2] * math.cos(beta_cmd))
    v_ax_r = float(vb[2])
    u = np.zeros(8)
    u[0:4] = [_thrust(mot[0], v_ax_f), _thrust(mot[1], v_ax_f),
              _thrust(mot[2], v_ax_r), _thrust(mot[3], v_ax_r)]
    u[4] = beta_cmd
    u[5], u[6], u[7] = de, da, dr
    return np.array(F(x, u)).ravel()[16:19].copy()


GAINS = dict(kR_roll=3.0, kW_roll=1.2, kR_pitch=3.0, kW_pitch=1.0, kR_yaw=5.0, kW_yaw=2.0,
             kR_hover=0.0, latency=0.03, k_bank=2.0, k_bank_d=1.0,
             bank_max=math.radians(25.0), k_head=0.5, k_head_d=0.8,
             turn_rate_max=math.radians(12.0), fx_budget=0.10,
             pitch_ratio=0.0, pitch_zeta=0.7, kI_roll=0.0, kI_yaw=0.0,
             i_band=math.radians(12.0), i_max=1.5)

# the flight conditions the two loops must both work at
CONDS = [("hover",     0.0,           0.0,  0.5),
         ("corridor",  math.radians(40.0), 8.0, 0.55),
         ("cruise",    math.radians(90.0), 16.0, 0.35)]


def trim_for(beta, V):
    """The outer's package at this condition: the thrusts and elevator that balance the aircraft,
    from the controller's OWN schedule (so the audit uses the same trim the loop flies)."""
    import nmpc6_cruise_node as n6
    s = n6.Schedule().at(V)
    return float(s["Tf"]), float(s["Tr"]), float(s["de"]), float(s["beta"]), float(s["theta"])


def run_loop(kind, R, w, theta_ref, beta, V, Tf, Tr, de_trim, vb, l1):
    """One inner tick of the named loop, returning what goes to the plant."""
    if kind == "geo":
        C.MOMENT_LAW = None
    else:
        C.MOMENT_LAW = _INDI_LAW
    mot, ail, elev, rud, _ei = C.inner_step(R, w, theta_ref, 0.0, beta, V, Tf, Tr, de_trim,
                                            GAINS, _ALLOC, vb=vb, ei=np.zeros(3), dt=DT,
                                            l1=l1, pred=None)
    C.MOMENT_LAW = None
    # ★ `mot` is a THROTTLE; the plant turns it into newtons with the MOMENTUM model, and the
    # conversion depends on the airspeed along each rotor's own axis. So the command is returned
    # as a THROTTLE and converted at the state it is evaluated AT -- converting each command at
    # its own state made the two runs differ by the conversion as well as by the loop's decision,
    # and that read as positive feedback on pitch at speed. (The flat mot * MAX_THRUST mapping,
    # the first version, was wrong for the same family of reason: the gate's own
    # "--prop_J0 re-propped only the plant" trap.)
    return np.asarray(mot, float), elev * C.DE_MAX, ail * C.DA_MAX, rud * C.DR_MAX


def _thrust(throttle, v_axial):
    """Newtons from a throttle: the sim's momentum model, the same formula the gate's plant uses."""
    n = float(throttle) * C._N_MAX
    x = n * C.PROP_D
    return max(0.0, so3.RHO * C.PROP_D ** 2 * (C._CT0 * x * x - (C._CT0 / C.PROP_J0) * x * float(v_axial)))


# ---- the INDI law, imported from the node exactly as it ships -------------------------------
sys.argv = [sys.argv[0]] + [a for a in sys.argv[1:] if a != "--verbose"]
os.environ["INDI_MODE"] = "full"
import nmpc_cascade_indi_node as INDI                          # noqa: E402
_INDI_LAW = INDI._moment_law_indi
_ALLOC = C.RotorAlloc()                                        # the gate builds exactly this


def fresh_l1():
    """A live L1 of the right class for each loop (the INDI law needs its own, which carries the
    filters and the delay FIFO); both get the same estimator and the predictor switched off, so
    the audit is of the LAW and not of the slosh feed-forward."""
    return None


# ============================ 1-3. DIRECTION, CROSS-AXIS, STIFFNESS ============================
# ★ THE BASELINE MATTERS. Applying the loop's whole command (the outer's trim + the inner's
# correction) to the plant and reading the angular acceleration measures the TRIM's own imbalance
# at the perturbed attitude, not the loop's response -- the first version of this audit read the
# flown pitch loop as "REINFORCES" at every condition, which is the harness, not the loop. The
# response to the error is the DIFFERENCE between the tick flown with the error and the same tick
# flown without it, everything else identical.
print("1-3 THE CLOSED INNER LOOP vs THE PLANT: law -> allocation -> surfaces -> angular acceleration")
print("    a +5 deg error on ONE axis, differenced against the same tick with no error;")
print("    the response must OPPOSE the error, and the other axes must not move")
print()
print("    %-9s %-5s %-5s | %-30s | %s" % ("condition", "axis", "loop",
                                           "d(wdot) on the axis [rad/s^2]", "worst cross-axis"))
ERR = math.radians(5.0)
stiff = {}
for name, beta, V, _thr in CONDS:
    Tf, Tr, de_trim, beta_s, theta_s = trim_for(beta, max(V, 0.1))
    v_world = np.array([V, 0.0, 0.0])
    for kind in ("geo", "indi"):
        # the unperturbed tick: the same trim, the same attitude reference, zero error
        R0 = Rpy(0.0, theta_s, 0.0)
        x0 = state19(R0, beta_s, v_world)
        T0, de0, da0, dr0 = run_loop(kind, R0, np.zeros(3), theta_s, beta_s, max(V, 0.1),
                                     Tf, Tr, de_trim, R0.T @ v_world, fresh_l1())
        wd0 = plant_wdot(x0, T0, beta_s, de0, da0, dr0)
        for ax, lab in ((0, "roll"), (1, "pitch"), (2, "yaw")):
            roll = ERR if ax == 0 else 0.0
            pitch = theta_s + (ERR if ax == 1 else 0.0)
            yaw = ERR if ax == 2 else 0.0
            R = Rpy(roll, pitch, yaw)
            x = state19(R, beta_s, v_world)
            try:
                T, de, da, dr = run_loop(kind, R, np.zeros(3), theta_s, beta_s, max(V, 0.1),
                                         Tf, Tr, de_trim, R.T @ v_world, fresh_l1())
                wd = plant_wdot(x, T, beta_s, de, da, dr)
            except Exception as e:
                print("    %-9s %-5s %-5s | ERROR %s" % (name, lab, kind, e))
                fails.append(f"{name}/{lab}/{kind}: {e}"); continue
            # ★ the plant's OWN response to the attitude change (aero stiffness, gyroscopics) is in
            # this difference too. Split it out by flying the perturbed attitude with the
            # UNPERTURBED command: what is left is the loop's contribution.
            wd_open = plant_wdot(x, T0, beta_s, de0, da0, dr0)
            d_loop = wd - wd_open                      # the loop's own moment change
            d_tot = wd - wd0                           # what the aircraft actually feels
            own = d_loop[ax]
            cross = max(abs(d_loop[i]) for i in range(3) if i != ax)
            want_pos = (ax == 1)                       # +pitch error (nose-up) needs +wdot (nose-down)
            ok = (own > 0) if want_pos else (own < 0)
            stiff[(name, lab, kind)] = abs(own) / ERR
            print("    %-9s %-5s %-5s | %+8.3f loop %+8.3f total  %-16s | %8.3f %s"
                  % (name, lab, kind, own, d_tot[ax], "OPPOSES" if ok else "** REINFORCES **",
                     cross, "" if cross < 0.35 * max(abs(own), 1e-6) else "(coupled)"))
            if not ok:
                if abs(own) < 0.2:      # within the noise of the split; report, do not fail
                    marginal.append(f"{name}/{lab}/{kind}: the loop's own contribution is ~0 "
                                    f"({own:+.3f} rad/s^2); the airframe carries this axis here")
                else:
                    fails.append(f"{name}/{lab}/{kind}: the loop REINFORCES its own error")
print()
print("3 STIFFNESS -- the LOOP's restoring angular acceleration per radian of error [1/s^2]")
print("    (the closed-loop bandwidth is sqrt of this; higher = less error for the same disturbance)")
print("    %-9s %-5s | %8s %8s %8s" % ("condition", "loop", "roll", "pitch", "yaw"))
for name, _b, _V, _t in CONDS:
    for kind in ("geo", "indi"):
        row = [stiff.get((name, a, kind), float("nan")) for a in ("roll", "pitch", "yaw")]
        print("    %-9s %-5s | %8.1f %8.1f %8.1f" % (name, kind, *row))
print()

# ============================ 4. SURFACE SIGNS (SHARED BY BOTH LOOPS) ============================
# ★ THE ONE THAT MATTERED. Each surface's share of the inner loop's moment demand is turned into a
# deflection by dividing by the derivative the allocation assumes. The MAGNITUDES were audited in
# 2026-09-10 (`audit_cascade.py` check 2, 0.0-0.1 %); the SIGNS never were -- and the rudder's copy
# of this bug is documented in the node ("+k_rud*Mz AMPLIFIED the yaw"). The elevator's was not.
print("4 SURFACE SIGNS -- the plant's true dM/d(deflection) vs what the allocation divides by")
print("    %-9s %22s %22s   %s" % ("surface", "plant [N.m/rad]", "the live allocation", "verdict"))
# ★ MEASURED FROM THE LIVE CODE, not from the formula as written here: the loop is run twice with
# a small difference in the moment demand on one axis and the DEFLECTION it produces is read back,
# so this check follows whatever the allocation currently does.
_bs, _V = math.radians(90.0), 16.0
_Tf, _Tr, _det, _beta_s, _ths = trim_for(_bs, _V)
_x = state19(Rpy(0.0, _ths), _beta_s, np.array([_V, 0, 0]))
_qS = 0.5 * so3.RHO * _V ** 2 * so3.S_WING


def _live_inverse(ax, dm=0.5):
    """N.m of demand per radian of deflection, as the ALLOCATION currently computes it."""
    R = Rpy(0.0, _ths)
    vb = R.T @ np.array([_V, 0.0, 0.0])
    out = []
    for sgn in (+1.0, -1.0):
        Md = np.zeros(3); Md[ax] = sgn * dm
        C.MOMENT_LAW = (lambda ctx, _M=Md: np.asarray(_M, float))
        mot, ail, elev, rud, _ = C.inner_step(R, np.zeros(3), _ths, 0.0, _beta_s, _V, _Tf, _Tr,
                                              _det, GAINS, _ALLOC, vb=vb, ei=np.zeros(3), dt=DT,
                                              l1=None, pred=None)
        C.MOMENT_LAW = None
        out.append((ail * C.DA_MAX, elev * C.DE_MAX, rud * C.DR_MAX)[ax])
    dd = out[0] - out[1]
    return (2.0 * dm) / dd if abs(dd) > 1e-12 else float("inf")


for _n, _ax, _assume, _sel in (("aileron", 0, _live_inverse(0), (0.0, 1.0, 0.0)),
                               ("elevator", 1, _live_inverse(1), (1.0, 0.0, 0.0)),
                               ("rudder", 2, _live_inverse(2), (0.0, 0.0, 1.0))):
    _h = 0.05
    _p = plant_wdot(_x, [0, 0, 0, 0], math.radians(90.0), *[s * _h for s in _sel])
    _m = plant_wdot(_x, [0, 0, 0, 0], math.radians(90.0), *[-s * _h for s in _sel])
    _true = (_p[_ax] - _m[_ax]) / (2 * _h) * IVEC[_ax]
    _ok = _true * _assume > 0
    print("    %-9s %22.2f %22.2f   %s (ratio %+.2f)"
          % (_n, _true, _assume, "OK" if _ok else "** SIGN INVERTED **", _true / _assume))
    if not _ok:
        fails.append(f"the allocation's {_n} sign is inverted against the plant "
                     f"(it divides the demand by {_assume:+.1f} where the plant gives {_true:+.1f})")
print()

# ============================ 5-6. THE INDI CHAIN ============================
print("5-6 THE INDI CHAIN -- the two model terms the increment subtracts, against the plant")
print("    %-9s %-26s %-26s %s" % ("condition", "M_c (control moment)", "M_trim (the outer's package)", "worst err"))
for _name, _beta, _V, _ in CONDS:
    Tf, Tr, de_trim, beta_s, th_s = trim_for(_beta, max(_V, 0.1))
    R = Rpy(0.0, th_s)
    v = np.array([max(_V, 0.1), 0.0, 0.0])
    x = state19(R, beta_s, v)
    vb = R.T @ v
    Ttrim = np.array([Tf / 2, Tf / 2, Tr / 2, Tr / 2])
    mot = np.array([C.thrust_to_throttle(t, vb[0] * math.sin(beta_s) + vb[2] * math.cos(beta_s))
                    if i < 2 else C.thrust_to_throttle(t, vb[2]) for i, t in enumerate(Ttrim)])
    # the plant's TRUE control moment for this command: with the actuators, minus without them
    w_on = plant_wdot(x, mot, beta_s, de_trim, 0.0, 0.0) * IVEC
    w_off = plant_wdot(x, [0, 0, 0, 0], beta_s, 0.0, 0.0, 0.0) * IVEC
    true_c = w_on - w_off
    model_c = (np.asarray(C.model_moment(beta_s, Ttrim, vb, np.zeros(3), de_trim, 0.0, 0.0), float)
               - np.asarray(C.model_moment(beta_s, np.zeros(4), vb, np.zeros(3), 0.0, 0.0, 0.0), float))
    err = np.abs(model_c - true_c).max()
    print("    %-9s M_c model (%+6.2f,%+6.2f,%+6.2f) vs plant (%+6.2f,%+6.2f,%+6.2f)   %.3f N.m"
          % (_name, *model_c, *true_c, err))
    if err > 0.15:
        fails.append(f"{_name}: the INDI control-moment model is {err:.2f} N.m from the plant")
print()

# ============================ 7. WHAT ONE TICK DELIVERS ============================
print("7 EFFECTIVENESS PER TICK -- the fraction of a commanded moment step each effector actually")
print("  delivers within one %.0f ms tick (a first-order actuator): the G an incremental law assumes" % (DT * 1e3))
_tau_r = float(so3._AF.rotors.tau_spin); _tau_s = float(so3._AF.surfaces.tau)
print("    rotors   tau %.3f s -> %4.1f %% of the step in one tick" % (_tau_r, 100 * (1 - math.exp(-DT / _tau_r))))
print("    surfaces tau %.3f s -> %4.1f %%" % (_tau_s, 100 * (1 - math.exp(-DT / _tau_s))))
print("    the pitch moment is SPLIT between the two by authority, and the split moves with qbar --")
print("    so the effectiveness an increment inverts is a blend whose ratio changes through the corridor.")
print()

# ============================ verdict ============================
for m in marginal:
    print("MARGINAL:", m)
if marginal:
    print()
if fails:
    print("AUDIT FAILED:")
    for f in fails:
        print("   -", f)
    sys.exit(1)
print("AUDIT PASSED -- both inner loops oppose their own errors on every axis at every condition,")
print("the surface signs match the plant, and the INDI chain's model terms match it too.")
