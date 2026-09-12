#!/usr/bin/env python3
"""Are the reference profiles TRIMS for the plant, and are the trajectory files flyable?

A reference the controllers are asked to hold should be a state the aircraft can actually
sit in: plug (xref, uref) into the plant model and the accelerations should be what the
reference intends -- zero vertically, zero in pitch, and whatever forward acceleration the
transition schedule is deliberately commanding. That check has found two real defects:

  * the aero pitch moment entered the front/rear thrust split without being negated into
    the FLU convention the thrust moment is written in;
  * the AVTOL trim generator returns a single Ttot on the premise that the WHOLE rotor set
    tilts, which is true of the aircraft it was written for and false of this one -- only
    the front pair tilts, and mid-corridor that under-counted the vertical thrust by 7.8 N,
    23% of the aircraft's weight.

Together they left up to 5.3 rad/s^2 of residual pitch acceleration: the "trim" the
wing-borne controllers were asked to hold was never a trim (2026-09-06 reference audit).

    ~/envs/isaac_env/bin/python3 tests/test_reference_profiles.py
"""
import csv
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "ros2", "px4_vtol_direct_actuator",
                                "px4_vtol_direct_actuator"))
import nmpc_so3_transition_node as so3            # noqa: E402

_ODE = so3.build_ode()
FAILURES = []
TRAJ_DIRS = [os.path.expanduser("~/ros2_ws/src/ros2_ground_station_gui/trajectories")]

# this airframe's envelope, from tests/test_aero_consistency.py
STALL_V = 11.5
V_MAX_SANE = 20.0
FENCE_R, FENCE_Z = 500.0, 60.0     # ground-station playback fence


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"   {detail}" if detail else ""))
    if not ok:
        FAILURES.append(name)


def residual(V, tilt, theta, Tf_pair, Tr_pair, de, gamma=0.0):
    """World-frame ax, az and body pitch acceleration at a reference point."""
    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])           # Ry(-theta): +theta = nose-UP
    v = np.array([V * math.cos(gamma), 0.0, V * math.sin(gamma)])
    x = np.concatenate([[0, 0, 20.0], R.reshape(-1), [tilt], v, [0, 0, 0]])
    u = np.array([Tf_pair / 2, Tf_pair / 2, Tr_pair / 2, Tr_pair / 2, tilt, de, 0.0, 0.0])
    xd = np.array(_ODE(x, u)).ravel()
    return float(xd[13]), float(xd[15]), float(xd[17])


def sweep(name, at):
    print(f"\n{name}")
    print(f"      {'V':>5} {'tilt':>6} {'theta':>6} {'Tf':>6} {'Tr':>6} {'de':>7} | "
          f"{'ax':>6} {'az':>6} {'qdot':>6}")
    wz = wq = 0.0
    for V in (0, 2, 4, 6, 8, 10, 12, 14, 16):
        r = at(float(V))
        ax, az, qd = residual(V, r["beta"], r["theta"], r["Tf"], r["Tr"], r["de"])
        wz = max(wz, abs(az)); wq = max(wq, abs(qd))
        print(f"      {V:5.1f} {math.degrees(r['beta']):6.1f} {math.degrees(r['theta']):6.2f} "
              f"{r['Tf']:6.1f} {r['Tr']:6.1f} {r['de']:7.3f} | {ax:+6.2f} {az:+6.2f} {qd:+6.2f}")
    check(f"{name}: vertically trimmed", wz < 0.15, f"worst |az| = {wz:.3f} m/s^2")
    check(f"{name}: pitch trimmed", wq < 0.20, f"worst |qdot| = {wq:.3f} rad/s^2")


def test_schedules():
    print("\n1. wing-borne trim schedules  (a trim leaves ax = the commanded acceleration,")
    print("   az = 0, qdot = 0)")
    import nmpc6_cruise_node as n6
    sch = n6.Schedule(V_cruise=16.0, tilt_cap_deg=90.0, alpha_max_deg=8.0)
    sweep("nmpc6.Schedule.at  (also used by nmpc_full_smc_node)", sch.at)

    import vtol_nmpc as vn
    sweep("vtol_nmpc.TRIM  (baked table)", lambda V: vn.trim_at(V))


def test_forward_acceleration_is_intended():
    """ax should be POSITIVE through the corridor -- the schedule is a hover->cruise
    transition, so it is meant to accelerate -- and should taper at the cruise end."""
    print("\n2. the forward acceleration the schedule commands")
    import nmpc6_cruise_node as n6
    sch = n6.Schedule(V_cruise=16.0, tilt_cap_deg=90.0, alpha_max_deg=8.0)
    acc = []
    for V in (2, 6, 10, 14, 16):
        r = sch.at(float(V))
        acc.append(residual(V, r["beta"], r["theta"], r["Tf"], r["Tr"], r["de"])[0])
    print("      ax at V = 2, 6, 10, 14, 16 m/s: " + ", ".join(f"{a:+.2f}" for a in acc))
    check("accelerates through the corridor", min(acc[:3]) > 0.3, f"min {min(acc[:3]):+.2f}")
    check("tapers toward cruise", acc[-1] < acc[1], f"{acc[-1]:+.2f} vs {acc[1]:+.2f}")


def test_time_parameterised_profiles():
    """For a MISSION the plant should reproduce the reference's OWN acceleration, not zero:
    a climb leg is meant to accelerate upward. So compare the plant's vdot at each reference
    point against d(v_ref)/dt, and require the pitch acceleration to be ~0 throughout (every
    reference here commands omega = 0)."""
    print("\n3. time-parameterised mission profile")
    try:
        from px4_vtol_direct_actuator.lib.fsc_mission_ref import FSCMissionProfile
    except Exception as exc:
        print(f"      (unavailable: {exc})")
        return
    m = FSCMissionProfile(target=(120.0, 0.0), z_cruise=15.0, V_cruise=13.0)
    ts = np.linspace(0.0, float(m.tf), 120)
    vref = np.array([m.get(float(t))[0][13:16] for t in ts])
    aref = np.stack([np.gradient(vref[:, i], ts) for i in range(3)], axis=1)
    worst_a = worst_q = 0.0
    for i in range(2, len(ts) - 2):
        xr, ur = m.get(float(ts[i]))
        xd = np.array(_ODE(np.asarray(xr, float), np.asarray(ur, float))).ravel()
        worst_a = max(worst_a, float(np.max(np.abs(xd[13:16] - aref[i]))))
        worst_q = max(worst_q, abs(float(xd[17])))
    print(f"      worst |plant a - d(v_ref)/dt| = {worst_a:.3f} m/s^2")
    print(f"      worst |pitch acceleration|    = {worst_q:.3f} rad/s^2")
    check("mission reference is dynamically consistent", worst_a < 0.60, f"{worst_a:.3f} m/s^2")
    check("mission reference is pitch-trimmed", worst_q < 0.20, f"{worst_q:.3f} rad/s^2")


def test_transition_ref():
    """TransitionRef is what the SO(3) node itself flies."""
    print("\n4. nmpc_so3_transition_node.TransitionRef.get")
    ref = so3.TransitionRef(0.0, 0.0, 20.0, 0.0, Vcruise=16.0, beta_cruise_deg=90.0, tf=20.0)
    print(f"      {'s':>5} {'V':>6} {'tilt':>6} {'theta':>7} | {'ax':>6} {'az':>6} {'qdot':>6}")
    wz = wq = 0.0
    for s_ in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        xr, ur = ref.get(s_, 0.0, theta_scale=1.0)
        xd = np.array(_ODE(np.asarray(xr, float), np.asarray(ur, float))).ravel()
        ax, az, qd = float(xd[13]), float(xd[15]), float(xd[17])
        wz = max(wz, abs(az)); wq = max(wq, abs(qd))
        R = np.asarray(xr[3:12], float).reshape(3, 3)      # ROW-major: no transpose
        print(f"      {s_:5.2f} {np.linalg.norm(xr[13:16]):6.1f} "
              f"{math.degrees(float(xr[12])):6.1f} "
              f"{math.degrees(math.asin(float(np.clip(R[2,0],-1,1)))):7.2f} | "
              f"{ax:+6.2f} {az:+6.2f} {qd:+6.2f}")
    check("TransitionRef: vertically trimmed", wz < 0.15, f"worst |az| = {wz:.3f}")
    check("TransitionRef: pitch trimmed", wq < 0.20, f"worst |qdot| = {wq:.3f}")


def test_trajectory_files():
    """The bundled reference trajectories should be flyable by THIS aircraft."""
    print("\n5. bundled trajectory files")
    d = next((p for p in TRAJ_DIRS if os.path.isdir(p)), None)
    if d is None:
        print("      (trajectories directory not found -- skipped)")
        return
    print(f"      {'file':34s} {'dur':>6} {'reach':>7} {'peak V':>7} {'v vs dp/dt':>11}")
    too_fast, inconsistent, outside = [], [], []
    for f in sorted(os.listdir(d)):
        if not f.endswith(".csv"):
            continue
        rows = list(csv.DictReader(open(os.path.join(d, f))))
        if not rows:
            continue
        k = {c.lower().strip(): c for c in rows[0]}

        def col(*names):
            for n in names:
                if n in k:
                    return np.array([float(r[k[n]]) for r in rows])
            return None

        t, x, y, z = col("t", "time"), col("x"), col("y"), col("z")
        if t is None or x is None:
            continue
        v_fd = np.stack([np.gradient(x, t), np.gradient(y, t), np.gradient(z, t)])
        peak = float(np.max(np.linalg.norm(v_fd, axis=0)))
        vx, vy, vz = col("vx"), col("vy"), col("vz")
        err = (float(np.max(np.linalg.norm(np.stack([vx, vy, vz]) - v_fd, axis=0)))
               if vx is not None else float("nan"))
        reach = float(np.max(np.hypot(x, y)))
        print(f"      {f:34s} {t[-1]:6.1f} {reach:7.0f} {peak:7.1f} {err:11.2f}")
        if peak > V_MAX_SANE:
            too_fast.append((f, peak))
        if not math.isnan(err) and err > 0.5:
            inconsistent.append((f, err))
        if reach > FENCE_R or float(np.max(z)) > FENCE_Z:
            outside.append((f, reach))
    check("no file demands a speed this airframe cannot reach",
          not too_fast, ", ".join(f"{f} peaks at {v:.0f} m/s" for f, v in too_fast))
    check("velocity columns agree with the position track",
          not inconsistent, ", ".join(f"{f} off by {e:.1f} m/s" for f, e in inconsistent))
    check("no file leaves the geofence",
          not outside, ", ".join(f"{f} reaches {r:.0f} m" for f, r in outside))


if __name__ == "__main__":
    print("reference profiles vs the plant")
    print("=" * 78)
    test_schedules()
    test_forward_acceleration_is_intended()
    test_time_parameterised_profiles()
    test_transition_ref()
    test_trajectory_files()
    print("\n" + "=" * 78)
    if FAILURES:
        print(f"{len(FAILURES)} CHECK(S) FAILED:")
        for f in FAILURES:
            print("  -", f)
        sys.exit(1)
    print("all checks passed")
