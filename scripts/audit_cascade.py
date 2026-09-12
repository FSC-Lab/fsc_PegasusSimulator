#!/usr/bin/env python3
"""Dynamics audit of the CASCADE controller, against the TRUE 19-state plant.

`scripts/audit_reference.py` audits nmpc_full's reference. The cascade has a DIFFERENT one
(build_refs -> Schedule -> trim_split -> flare/aft-tilt -> slew limits) and, on top of that, an
allocation matrix and a set of surface-authority formulas that were DERIVED BY HAND. None of that
had been checked against the plant. Hand-derived geometry is exactly where this project has been
bitten before (a sign-flipped rotor z-arm, a pitch convention inverted six times), so every piece
is checked NUMERICALLY here rather than by reading the algebra.

Four checks, all against `so3.build_ode()` -- the plant the sim implements:

  1 ALLOCATION      does B_rot(beta) reproduce the plant's dF/dT and dM/dT, column by column?
  2 SURFACES        does surface_authority(V) match the moment the plant makes at full deflection?
  3 REFERENCE       is the cascade's reference a TRIM? (residual accelerations, forward AND the
                    back-transition branch, which has its own flare + aft-tilt trim_split calls)
  4 OUTER MODEL     does build_ode6 -- what the outer NMPC plans with -- match the true plant in
                    the longitudinal plane?
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "ros2", "px4_vtol_direct_actuator",
                                "px4_vtol_direct_actuator"))

import nmpc_cascade_node as C          # noqa: E402
import nmpc_so3_transition_node as so3  # noqa: E402
import nmpc6_cruise_node as n6         # noqa: E402

F = so3.build_ode()
IVEC = np.array([so3.IXX, so3.IYY, so3.IZZ])
fails = []


def state19(z=30.0, theta=0.0, beta=0.0, vs=0.0, vz=0.0, omega=(0.0, 0.0, 0.0)):
    """theta is NOSE-UP positive -> R = Ry(-theta). Velocity is given in the WORLD frame, so
    level flight means vz = 0 and the angle of attack equals theta (the probe trap that once made
    a longitudinal event read as a lateral departure)."""
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])
    x = np.zeros(19)
    x[0:3] = [0.0, 0.0, z]
    x[3:12] = R.reshape(-1)
    x[12] = beta
    x[13:16] = [vs, 0.0, vz]
    x[16:19] = omega
    return x


def accel(x, u):
    """(force/mass, angular acceleration) the plant produces. Gravity removed from the vertical so
    a balanced hover reads zero rather than -g."""
    xd = np.array(F(x, u)).ravel()
    a = xd[13:16].copy()
    return a, xd[16:19].copy()


# ============================ 1. ALLOCATION ============================
print("1 ALLOCATION -- B_rot(beta) vs the plant, column by column")
print("     tilt | rotor |   dFz      dMx      dMy      dMz    | worst err")
worst_alloc = 0.0
for bd in (0, 30, 60, 90):
    b = math.radians(bd)
    B = C.B_rot(b)
    for i in range(4):
        base = np.array([4.0, 4.0, 4.0, 4.0, b, 0.0, 0.0, 0.0])
        eps = 0.01
        up, dn = base.copy(), base.copy()
        up[i] += eps
        dn[i] -= eps
        x = state19(beta=b)
        a_u, w_u = accel(x, up)
        a_d, w_d = accel(x, dn)
        dFz = (a_u[2] - a_d[2]) / (2 * eps) * so3.M          # world z == body z at theta=0
        dM = (w_u - w_d) / (2 * eps) * IVEC
        got = np.array([dFz, dM[0], dM[1], dM[2]])
        err = np.max(np.abs(got - B[:, i]))
        worst_alloc = max(worst_alloc, err)
        if i == 0 or err > 1e-3:
            print(f"   {bd:5.0f} | {i:5d} | " + " ".join(f"{v:+8.4f}" for v in got) +
                  f" | {err:.2e}{'   MISMATCH' if err > 1e-3 else ''}")
if worst_alloc > 1e-3:
    fails.append(f"allocation B_rot disagrees with the plant by {worst_alloc:.2e}")
print(f"   worst column error across all tilts: {worst_alloc:.2e}\n")

# ============================ 2. SURFACES ============================
print("2 SURFACES -- surface_authority(V) vs the moment the plant makes at full deflection")
print("        V |   roll          pitch          yaw     (N.m)  formula vs plant")
worst_surf = 0.0
for V in (8.0, 12.0, 16.0):
    A = C.surface_authority(V)
    x = state19(vs=V)
    u0 = np.array([4.0, 4.0, 4.0, 4.0, 0.0, 0.0, 0.0, 0.0])
    got = []
    for k, dmax in ((6, C.DA_MAX), (5, C.DE_MAX), (7, C.DR_MAX)):
        up, dn = u0.copy(), u0.copy()
        up[k] += dmax
        dn[k] -= dmax
        _, w_u = accel(x, up)
        _, w_d = accel(x, dn)
        dM = (w_u - w_d) / 2.0 * IVEC
        got.append(abs(dM[{6: 0, 5: 1, 7: 2}[k]]))
    rel = [abs(g - a) / max(a, 1e-9) for g, a in zip(got, A)]
    worst_surf = max(worst_surf, max(rel))
    print(f"   {V:6.1f} | " + "  ".join(f"{g:7.3f}/{a:-7.3f}" for g, a in zip(got, A)) +
          f"   worst {max(rel)*100:.1f}%")
if worst_surf > 0.15:
    fails.append(f"surface authority formula is {worst_surf*100:.0f}% off the plant")
print()

# ============================ 3. REFERENCE IS A TRIM ============================
sched = n6.Schedule(V_cruise=16.0, tilt_cap_deg=90.0, alpha_max_deg=8.0)


def audit_refs(title, decel, ttr, flare=0.0, beta_aft=0.0):
    print(f"3 REFERENCE -- {title}")
    print("      V   tilt  pitch |    ax     az   |    p       q       r    (rad/s^2)")
    worst_q = worst_az = 0.0
    b_prev = th_prev = None
    speeds = (16.0, 12.0, 8.0, 4.0, 1.0) if decel else (0.0, 4.0, 8.0, 12.0, 16.0)
    for v in speeds:
        refs, b_prev, th_prev = C.build_refs(
            sched, 14, 0.06, 0.05, v, 16.0, ttr, 30.0,
            decel=decel, decel_rate=1.5, flare=flare, beta_aft=beta_aft,
            b_slew=1e9, b_prev=None, th_slew=1e9, th_prev=None)
        r = refs[0]
        x = state19(z=r["z"], theta=r["theta"], beta=r["beta"], vs=r["vs"], vz=r["vz"])
        u = np.array([r["Tf"] / 2, r["Tf"] / 2, r["Tr"] / 2, r["Tr"] / 2,
                      r["beta"], r["de"], 0.0, 0.0])
        a, wd = accel(x, u)
        worst_q = max(worst_q, abs(wd[1]))
        worst_az = max(worst_az, abs(a[2]))
        print(f"   {r['vs']:5.1f} {math.degrees(r['beta']):6.1f} "
              f"{math.degrees(r['theta']):+6.1f} | {a[0]:+6.2f} {a[2]:+6.2f} | "
              f"{wd[0]:+7.3f} {wd[1]:+7.3f} {wd[2]:+7.3f}")
    print(f"   worst |pitch accel| {worst_q:.3f} rad/s^2, worst |az| {worst_az:.3f} m/s^2")
    if worst_q > 0.5:
        fails.append(f"{title}: reference is not a trim, |qdot| = {worst_q:.2f} rad/s^2")
    if worst_az > 1.5:
        fails.append(f"{title}: reference vertical residual {worst_az:.2f} m/s^2")
    print()


audit_refs("FORWARD corridor (ttr=20, the shipped default)", False, 20.0)
audit_refs("BACK-TRANSITION (flare 15 deg + aft tilt 20 deg)", True, 20.0,
           flare=math.radians(15.0), beta_aft=math.radians(20.0))

# ============================ 4. OUTER MODEL vs PLANT ============================
print("4 OUTER MODEL -- build_ode6 (what the outer NMPC plans with) vs the true plant")
print("      V   tilt |      vsdot         vzdot          qdot      (6-state / plant)")
F6 = n6.build_ode6()
worst_model = 0.0
for V in (0.0, 8.0, 16.0):
    s = sched.at(V)
    X6 = np.array([0.0, 30.0, V, 0.0, s["theta"], 0.0])
    U6 = np.array([s["Tf"], s["Tr"], s["beta"], s["de"]])
    d6 = np.array(F6(X6, U6)).ravel()
    x = state19(theta=s["theta"], beta=s["beta"], vs=V)
    u = np.array([s["Tf"] / 2, s["Tf"] / 2, s["Tr"] / 2, s["Tr"] / 2,
                  s["beta"], s["de"], 0.0, 0.0])
    a, wd = accel(x, u)
    pairs = ((d6[2], a[0]), (d6[3], a[2]), (d6[5], -wd[1]))   # q is NOSE-UP = -omega_y
    worst_model = max(worst_model, max(abs(m - p) for m, p in pairs))
    print(f"   {V:5.1f} {math.degrees(s['beta']):6.1f} | " +
          "  ".join(f"{m:+7.3f}/{p:+7.3f}" for m, p in pairs))
if worst_model > 0.05:
    fails.append(f"outer model disagrees with the plant by {worst_model:.3f}")
print(f"   worst disagreement: {worst_model:.4f}\n")

# ============================ verdict ============================
if fails:
    print("AUDIT FAILED:")
    for f in fails:
        print("   -", f)
    sys.exit(1)
print("CASCADE DYNAMICS AUDIT PASSED")
