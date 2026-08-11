#!/usr/bin/env python3
"""Derive a T650 parameter set from the same-command plant test.

Same discipline as the 2026-08-06 tuning: **parameters only**, no modelling change, mass
pinned. What differs is the instrument -- 2026-08-06 fitted through a closed loop (hover
command, and yaw step shape), which entangles plant error with controller compensation.
This fits against the open-loop plant response to the commands the vehicle actually flew,
which is controller-independent, so one parameter set can serve baseline and DIRECT alike.

Fit rules, and why each:

  k        from the specific-thrust gain, taken in the EARLIEST window of the FRESHEST
           flight. Battery sag inflates the apparent gain monotonically through every
           flight (measured: +2.7 pts over D1, +5.2 pts over D2), so any later window
           biases k low.

  Ixx,Iyy  from the per-axis roll/pitch angular-acceleration gain. The thrust gain is
           near 1, so k is roughly right and the roll/pitch error is torque/inertia, not
           force. Fitted per axis because the two axes disagree by more than run-to-run
           spread.

  Izz      scaled by the MEAN of the Ixx/Iyy factors. There is no independent yaw-inertia
           measurement -- yaw only ever constrains the ratio c/Izz. The physical argument
           for moving it: all three came from the same X650 CAD panel for a 1.467 kg bare
           frame under the same "the extra mass is centrally concentrated" assumption, so
           if two of them are underestimated the third is too. Holding Izz fixed and
           putting the whole yaw correction on c is what produced the x3.0 that then
           failed to transfer between modes.

  c        set to close whatever yaw gain remains AFTER Izz moves. Whatever is left is a
           genuine torque-model deficit -- the unmodelled I_rotor*omega_dot reaction --
           rather than a stand-in for an inertia error.

Yaw wrap: this fit reads angular acceleration from the gyro, which never wraps, so the
+-180 deg ambiguity that corrupts the D2 yaw TRAJECTORY does not touch these numbers.
"""
import json
import sys

import numpy as np

SRC = open(__file__.replace("fit_params.py", "plant_replay.py")).read().split("def main()")[0]
_ns = {}
exec(SRC, _ns)
P, predict, lp = _ns["P"], _ns["predict"], _ns["lp"]

DATA = "/home/fsc-jupiter/Source/fsc_PegasusSimulator/docs/sim_to_real_t650/data"


def gains(tag, fc=12.0, fhp=0.2, thrust_window_s=20.0):
    d = np.load(f"{DATA}/real_{tag}.npz", allow_pickle=True)
    t_sc = d["sc_ts"].astype(float) * 1e-6
    t_mot = d["mot_ts"].astype(float) * 1e-6
    u = d["mot_ctrl"].astype(float)
    ok = ~np.isnan(u).any(axis=1)
    t_mot, u = t_mot[ok], u[ok]
    m = (t_sc >= t_mot[0]) & (t_sc <= t_mot[-1])
    t = t_sc[m]
    gyro = d["sc_gyro"][m].astype(float)
    az = d["sc_accel"][m, 2].astype(float)
    uu = u[np.clip(np.searchsorted(t_mot, t, side="right") - 1, 0, len(u) - 1)]

    pr = predict(uu, t, P.ROTOR_LAMBDA, P.ROTOR_CONSTANT, P.ROLLING_MOMENT_COEFFICIENT,
                 P.MASS, P.INERTIA_DIAG)
    fs = 1.0 / np.median(np.diff(t))
    hp = lambda x: x - lp(x, fs, fhp)
    am = hp(lp(np.gradient(gyro, t, axis=0), fs, fc))
    ap = hp(lp(pr["alpha"], fs, fc))

    out = {}
    for j, nm in enumerate(("roll", "pitch", "yaw")):
        a, b = am[:, j], ap[:, j]
        out[nm] = float((b @ a) / (b @ b))
    rel = t - t[0]
    w = rel < thrust_window_s
    out["thrust_early"] = float(lp(pr["a_z"], fs, fc)[w].mean() / lp(az, fs, fc)[w].mean())
    out["thrust_all"] = float(lp(pr["a_z"], fs, fc).mean() / lp(az, fs, fc).mean())
    out["mean_cmd_early"] = float(uu[w].mean())
    return out


def main():
    tags = sys.argv[1:] or ["D1", "D2"]
    g = {t: gains(t) for t in tags}
    print(f"{'flight':8} {'roll':>7} {'pitch':>7} {'yaw':>7} {'thrust(early)':>14} "
          f"{'thrust(all)':>12} {'mean cmd':>9}")
    for t in tags:
        v = g[t]
        print(f"{t:8} {v['roll']:7.3f} {v['pitch']:7.3f} {v['yaw']:7.3f} "
              f"{v['thrust_early']:14.3f} {v['thrust_all']:12.3f} {v['mean_cmd_early']:9.4f}")

    roll = float(np.mean([g[t]["roll"] for t in tags]))
    pitch = float(np.mean([g[t]["pitch"] for t in tags]))
    yaw = float(np.mean([g[t]["yaw"] for t in tags]))
    # Freshest flight only: sag makes every later measurement biased.
    fresh = min(tags, key=lambda t: g[t]["mean_cmd_early"])
    thrust = g[fresh]["thrust_early"]

    f_ixx, f_iyy = 1.0 / roll, 1.0 / pitch
    f_izz = 0.5 * (f_ixx + f_iyy)
    f_k = 1.0 / thrust
    # yaw gain scales with 1/Izz, so moving Izz first leaves this residual for c
    f_c = yaw * f_izz

    new = dict(
        ROTOR_CONSTANT=P.ROTOR_CONSTANT * f_k,
        ROLLING_MOMENT_COEFFICIENT=P.ROLLING_MOMENT_COEFFICIENT * f_c,
        INERTIA_DIAG=[P.INERTIA_DIAG[0] * f_ixx, P.INERTIA_DIAG[1] * f_iyy,
                      P.INERTIA_DIAG[2] * f_izz],
    )
    print(f"\nthrust reference flight: {fresh} (lowest early mean command)")
    print(f"\n{'parameter':30} {'current':>14} {'factor':>9} {'fitted':>14} "
          f"{'vs bench/CAD':>13}")
    print(f"{'ROTOR_CONSTANT':30} {P.ROTOR_CONSTANT:14.6e} {f_k:9.4f} "
          f"{new['ROTOR_CONSTANT']:14.6e} "
          f"{new['ROTOR_CONSTANT']/P.BENCH_ROTOR_CONSTANT:12.3f}x")
    print(f"{'ROLLING_MOMENT_COEFFICIENT':30} {P.ROLLING_MOMENT_COEFFICIENT:14.6e} "
          f"{f_c:9.4f} {new['ROLLING_MOMENT_COEFFICIENT']:14.6e} "
          f"{new['ROLLING_MOMENT_COEFFICIENT']/P.BENCH_ROLLING_MOMENT_COEFFICIENT:12.3f}x")
    for i, (nm, f) in enumerate((("INERTIA Ixx", f_ixx), ("INERTIA Iyy", f_iyy),
                                 ("INERTIA Izz", f_izz))):
        print(f"{nm:30} {P.INERTIA_DIAG[i]:14.6e} {f:9.4f} {new['INERTIA_DIAG'][i]:14.6e} "
              f"{new['INERTIA_DIAG'][i]/P.CAD_INERTIA_DIAG[i]:12.3f}x")

    g_ = 9.81
    T = P.MASS * g_ / 4
    wh = np.sqrt(T / new["ROTOR_CONSTANT"])
    span = P.MAX_ROTOR_VEL - P.ZERO_POSITION_ARMED
    hover = (wh - P.ZERO_POSITION_ARMED) / span
    print(f"\nmass unchanged at {P.MASS:.6f} kg; implied hover command "
          f"{hover:.4f}  (was {P.HOVER_COMMAND:.4f})")
    print(f"thrust/weight {(4*new['ROTOR_CONSTANT']*P.MAX_ROTOR_VEL**2)/(P.MASS*g_):.3f}")

    json.dump({"gains": g, "factors": dict(k=f_k, c=f_c, ixx=f_ixx, iyy=f_iyy, izz=f_izz),
               "fitted": {k: (v if not isinstance(v, list) else v) for k, v in new.items()},
               "hover_command": float(hover), "thrust_reference_flight": fresh},
              open(f"{DATA}/fit_20260807.json", "w"), indent=2)
    print(f"\nwrote {DATA}/fit_20260807.json")


if __name__ == "__main__":
    main()
