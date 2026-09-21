#!/usr/bin/env python3
"""Can the cascade's inner loop simply HOLD A HOVER?

WHY THIS EXISTS: every offline gate in this project drives the aircraft straight into a
transition and spends its time at cruise. Hover is passed through in a few seconds on the way,
so the LOW-SPEED end of the envelope is barely exercised -- and that is precisely where the
rotor-differential path is load-bearing: at V ~ 0 the aero surfaces have no dynamic pressure, so
100% of the commanded roll/yaw moment goes through B_rot^-1 to the rotors.

Three HIL flights (bags vtol_20260911_102706 / _122940 / _124102) all showed the SAME thing in
the first second after engage, sitting in hover with nothing disturbing them: the motors
alternating on the DIAGONAL -- FL/RR against FR/RL, which is the spin-reaction YAW pair -- at the
sample rate, with yaw swinging +-5 deg. Every later departure grew out of that. No offline gate
caught it because none of them stayed in hover long enough to look.
RESOLVED 2026-09-13: it was the PLANT, not the gains -- the prop links were pinned at PhysX's
default 100 rad/s and the velocity drive's reaction was a yaw torque 3x the modelled one on
any throttle differential (vtol.py::_unclamp_prop_spin). This test's plant never had it, which
is why no gain pair here ever reproduced the thrash; the verdict is now the diagonal's
AMPLITUDE (the thrash: sd 0.4-0.8; a clean hover under noise: ~0.01).

    usage: test_hover_hold.py [--kR_yaw K] [--kW_yaw K] [--dur S] [--yaw0 DEG] [--verbose]
"""
import math
import os
import sys

import numpy as np

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
import nmpc_cascade_node as C          # noqa: E402
import nmpc_so3_transition_node as so3  # noqa: E402


def argf(n, d):
    return float(sys.argv[sys.argv.index(n) + 1]) if n in sys.argv else d


def fly(kR_yaw, kW_yaw, dur=12.0, yaw0=0.0, lat=0.03, dt=1.0 / 120.0, verbose=False,
        noise_att=0.1, noise_rate=1.0, seed=1):
    """Hold a hover with the real inner loop and a FIXED hover package, on the true plant.
    Returns (yaw history, per-sample diagonal motor split, max |attitude|)."""
    F = so3.build_ode()
    alloc = C.RotorAlloc()
    g = dict(kR_roll=3.0, kW_roll=1.8, kR_pitch=3.0, kW_pitch=1.8,
             kR_yaw=kR_yaw, kW_yaw=kW_yaw, k_bank=1.2, k_bank_d=0.0,
             bank_max=math.radians(25.0),
             kI_roll=argf('--kI_roll', 0.0), kI_yaw=argf('--kI_yaw', 0.0),
             i_band=math.radians(12.0), i_max=1.5)
    cy, sy = math.cos(math.radians(yaw0)), math.sin(math.radians(yaw0))
    R0 = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    x = np.zeros(19)
    x[0:3] = [0.0, 0.0, 10.0]
    x[3:12] = R0.reshape(-1)
    Tf = Tr = so3.M * so3.G / 2.0          # balanced hover package, tilt 0
    rng = np.random.default_rng(int(seed))
    buf, n_lat = [], max(0, int(round(lat / dt)))
    ei = np.zeros(3)
    yaws, diag, worst = [], [], 0.0
    for k in range(int(dur / dt)):
        R = x[3:12].reshape(3, 3)
        V = float(np.linalg.norm(x[13:16]))
        vb = R.T @ x[13:16]
        # what the CONTROLLER sees. Representative mocap noise -- without it the hover is
        # perfectly symmetric and the yaw loop is never excited at all.
        w_meas = x[16:19] + rng.normal(0.0, math.radians(noise_rate), 3)
        if noise_att > 0.0:
            dd = rng.normal(0.0, math.radians(noise_att), 3)
            K = np.array([[0.0, -dd[2], dd[1]], [dd[2], 0.0, -dd[0]], [-dd[1], dd[0], 0.0]])
            Rm = R @ (np.eye(3) + K)
            U0, _, V0 = np.linalg.svd(Rm)
            Rm = U0 @ V0
        else:
            Rm = R
        mot, ail, elev, rud, ei = C.inner_step(Rm, w_meas, 0.0, 0.0, 0.0, V, Tf, Tr, 0.0,
                                               g, alloc, vb=vb, ei=ei, dt=dt)
        # motors are THROTTLE; convert back to newtons exactly as the plant would
        Ns = [(m ** 2) * so3.MAX_THRUST for m in mot]
        uu = np.array(Ns + [0.0, elev * C.DE_MAX, ail * C.DA_MAX, rud * C.DR_MAX])
        buf.append(uu)
        ua = buf.pop(0) if len(buf) > n_lat else np.array([so3.M * so3.G / 4.0] * 4 + [0.0] * 4)
        x = x + dt * np.array(F(x, ua)).ravel()
        U_, _, Vt_ = np.linalg.svd(x[3:12].reshape(3, 3))
        x[3:12] = (U_ @ Vt_).reshape(-1)
        R = x[3:12].reshape(3, 3)
        yaws.append(math.degrees(math.atan2(R[1, 0], R[0, 0])))
        # the signature from the bags: FL/RR against FR/RL
        diag.append((mot[0] + mot[3]) - (mot[1] + mot[2]))
        worst = max(worst, abs(math.degrees(math.atan2(R[2, 1], R[2, 2]))),
                    abs(math.degrees(math.asin(max(-1.0, min(1.0, R[2, 0]))))))
        if verbose and k % int(0.25 / dt) == 0:
            print(f"   t={k*dt:5.2f} yaw {yaws[-1]:+7.2f}  diag {diag[-1]:+6.3f}  "
                  f"mot [{','.join(f'{m:.2f}' for m in mot)}]")
    return np.array(yaws), np.array(diag), worst


def yaw_torque_per_diag():
    """N.m of yaw torque per unit of the diagonal split, at the hover throttle: the split is in
    THROTTLE, and what the airframe feels is cQ * (T_FL - T_FR - T_RL + T_RR) with T = mot^2 * T_max.
    ★ 2026-09-21: the measured cQ (0.0190, was the sim-only 0.12) makes the allocation ask 6.3x the
    throttle differential for the same moment, so a bar written in throttle units at cQ 0.12 tripped
    on the new build (diag sd 0.064 against 0.05 -- "OSCILLATES" in the 2026-09-17 record) while
    the yaw held to 0.04 deg and the torque chatter was 3.5x UNDER the old bar. The verdict is the
    torque; the throttle chatter is still printed (it is what the ESCs see)."""
    mot_h = math.sqrt(so3.M * so3.G / 4.0 / so3.MAX_THRUST)
    return so3.CQ * 2.0 * mot_h * so3.MAX_THRUST


TAU_BAR = 0.16      # N.m: the old 0.05 of diagonal split at cQ 0.12 / 18 N (0.733 hover throttle)


def metrics(yaws, diag):
    """Oscillation = high-frequency content. A settled hover has a near-zero sample-to-sample
    change in the diagonal split; an oscillating one alternates every sample."""
    d = np.diff(diag)
    flips = int(np.sum(d[:-1] * d[1:] < 0))          # sign changes = alternation
    return float(np.std(yaws[-len(yaws) // 3:])), float(np.std(d)), flips


def main():
    dur = argf("--dur", 12.0)
    yaw0 = argf("--yaw0", 0.0)
    verbose = "--verbose" in sys.argv
    if "--kR_yaw" in sys.argv or "--kW_yaw" in sys.argv:
        kr, kw = argf("--kR_yaw", 5.0), argf("--kW_yaw", 2.0)
        y, d, worst = fly(kr, kw, dur, yaw0, verbose=verbose)
        ysd, dsd, flips = metrics(y, d)
        print(f"  kR_yaw={kr} kW_yaw={kw}: yaw sd {ysd:.3f} deg, diag-split sd {dsd:.4f} "
              f"(= {dsd * yaw_torque_per_diag():.3f} N.m of yaw chatter), "
              f"alternations {flips}/{len(d)-1}, worst attitude {worst:.2f} deg")
        return 0

    print("  hover hold, 12 s, true plant, 30 ms latency, REPRESENTATIVE MOCAP NOISE")
    print("  (0.1 deg attitude, 1 deg/s rate -- a noise-free hover holds at exactly 0")
    print("   and tells you nothing, which is how this mode stayed hidden)")
    k_tau = yaw_torque_per_diag()
    print(f"   (cQ {so3.CQ} m, T_max {so3.MAX_THRUST} N: one unit of diagonal split = {k_tau:.2f} N.m; "
          f"bar {TAU_BAR} N.m = diag sd {TAU_BAR / k_tau:.3f})")
    print("   kR_yaw  kW_yaw |  yaw sd   diag sd   yaw N.m   alternations   verdict")
    best = None
    rows = []
    for kr, kw in ((5.0, 2.0), (3.0, 1.5), (2.0, 1.2), (1.5, 1.0), (1.0, 0.8), (0.6, 0.6)):
        y, d, worst = fly(kr, kw, dur, yaw0)
        ysd, dsd, flips = metrics(y, d)
        frac = flips / max(1, len(d) - 1)
        # ★ AMPLITUDE, NOT SIGN CHANGES (2026-09-13). The thrash this test was written for
        # (bags vtol_20260911_*) was the diagonal rail to rail -- sd 0.4-0.8, yaw +-5 deg -- and
        # it was the PLANT: the prop links sat on PhysX's 100 rad/s cap and the drive's reaction
        # was a 3x spurious yaw torque (see vtol.py::_unclamp_prop_spin, the 09 probe, and
        # scripts/bag_axis_fit.py). This plant (so3.build_ode) never had the artefact, so the
        # test could not reproduce the thrash and its bar sank to "the command changes sign a
        # lot" -- which a rate feedback on 1 deg/s of white rate noise does 50-66% of the time by
        # construction, at a diagonal sd of 0.01. That is noise following, not oscillation.
        tsd = dsd * k_tau
        ok = (tsd < TAU_BAR) and (ysd < 1.0)
        rows.append((kr, kw, ysd, dsd, frac, ok))
        v = "OK" if ok else ("OSCILLATING" if tsd > 4.0 * TAU_BAR or ysd > 3.0 else "marginal")
        print(f"   {kr:6.1f}  {kw:6.1f} | {ysd:7.3f}  {dsd:8.4f}  {tsd:7.3f}   {frac*100:5.0f}%        {v}")
        if ok and best is None:
            best = (kr, kw)

    print()
    if best is None:
        print("  NO GAIN PAIR HOLDS A CLEAN HOVER -- the yaw loop needs more than a gain change")
        return 1
    print(f"  highest gains that hold a clean hover: kR_yaw={best[0]}, kW_yaw={best[1]}")
    print(f"  shipped default is kR_yaw=5.0 / kW_yaw=2.0 -> {'OK' if rows[0][5] else 'NOT CLEAN (see the row)'}")
    return 0 if rows[0][5] else 1


if __name__ == "__main__":
    sys.exit(main())
