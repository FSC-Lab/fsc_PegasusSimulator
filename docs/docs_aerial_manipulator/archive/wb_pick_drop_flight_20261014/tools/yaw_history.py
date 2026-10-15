"""Standing yaw torque (and roll/pitch trim) the rotors produce in steady DIRECT flight, every hardware bag since 2026-09-18.

One FIXED rotor model for every flight, so the number does not depend on the controller or its tune:
  omega_i = omega_idle + u_i (omega_max - omega_idle)       (alloc_omega_idle/max, MN4010: 64.0603 / 730.0507 rad/s)
  f_i     = kf omega_i^2                                    (kf 4.260431e-05, the hardware yaml's alloc_thrust_coeff)
  tau_z   = -km sum_i s_i f_i,  s = +1 CCW (FR, RL), -1 CW (FL, RR), km = 0.018164 m (bench c/kf)
  tau_x   = sum_i y_i f_i, tau_y = -sum_i x_i f_i           (FLU, rotors at (+-0.2299, +-0.2299) m)
This is the allocator's own model (its tau_z equals the whole-body law's commanded tau_flu z to 1e-3 N.m, checked);
the REAL yaw torque per unit of motor split is smaller (sim-to-real fit: 0.43-0.78 of bench c), but the model is the
same for every flight, so changes between flights are real changes in what the motors had to hold off.
A sample counts when it is DIRECT, >= 3 s after entry, |v| < 0.15 m/s and |yaw rate| < 3 deg/s (quasi-static).
Output: ../analysis/yaw_history.json + a printed table.
"""
import os, sys, json, glob
import numpy as np
from pd_common import load, edges, mocap_pose, yaw_of, wbdebug, HIST, NPZ, OUT, SPIN, ARM_XY, RUNS

KF, KM, W0, W1 = 4.260431e-05, 0.018164, 64.0603, 730.0507
X = np.array([+1, -1, +1, -1]) * ARM_XY          # FR, RL, FL, RR  (FLU: x forward)
Y = np.array([-1, +1, +1, -1]) * ARM_XY          # FLU: y left


def rotor_wrench(U):
    om = W0 + np.clip(U, 0, 1) * (W1 - W0)
    f = KF * om ** 2
    return f.sum(1), -KM * (f * SPIN).sum(1), (f * Y).sum(1), -(f * X).sum(1), f


def direct_spans(d, t0):
    key = "wbmode" if "wbmode__recv" in d.files else "gmode"
    ev = edges(d, t0, key)
    out, a = [], None
    for t, v in ev:
        if v == "DIRECT" and a is None:
            a = t
        elif v != "DIRECT" and a is not None:
            out.append((a, t)); a = None
    if a is not None:
        out.append((a, float(d[f"{key}__recv"][-1] - t0)))
    return key, out


def analyse(path, label):
    d, t0 = load(path)
    key, spans = direct_spans(d, t0)
    if not spans or "act_motors__control" not in d.files:
        return None
    tu = d["act_motors__recv"] - t0
    U = d["act_motors__control"][:, :4]
    tm, P, Q, V = mocap_pose(d, t0)
    yaw = np.unwrap(yaw_of(Q))
    yr = np.gradient(yaw, tm)
    sp = np.linalg.norm(V, axis=1)
    ok = np.all(np.isfinite(U), 1) & (U.min(1) > 0.05)
    inn = np.zeros(len(tu), bool)
    for a, b in spans:
        inn |= (tu >= a + 3.0) & (tu <= b - 0.3)
    q = ok & inn & (np.interp(tu, tm, sp) < 0.15) & (np.abs(np.interp(tu, tm, yr)) < np.radians(3.0))
    if q.sum() < 250:
        return None
    T, tz, tx, ty, f = rotor_wrench(U[q])
    res = dict(label=label, rig="WB" if key == "wbmode" else "DEC", n=int(q.sum()), direct_s=float(sum(b - a for a, b in spans)),
               T=float(np.median(T)), tau_z=float(np.median(tz)), tau_z_p10=float(np.percentile(tz, 10)),
               tau_z_p90=float(np.percentile(tz, 90)), lever_mm=float(np.median(tz / T) * 1e3),
               tau_x=float(np.median(tx)), tau_y=float(np.median(ty)),
               u=[float(x) for x in np.median(U[q], 0)])
    # time course: first vs last third of the quasi-static samples (battery sag / anything that changes in flight)
    tq = tu[q]
    k = len(tq) // 3
    res["tau_z_first"] = float(np.median(tz[:k])); res["tau_z_last"] = float(np.median(tz[-k:]))
    res["T_first"] = float(np.median(T[:k])); res["T_last"] = float(np.median(T[-k:]))
    if key == "wbmode":
        tw, Wd = wbdebug(d, t0)
        qq = np.zeros(len(tw), bool)
        for a, b in spans:
            qq |= (tw >= a + 3.0) & (tw <= b - 0.3)
        res["dhat_rz"] = float(np.median(Wd[qq, 36])); res["tau_cmd_z"] = float(np.median(Wd[qq, 23]))
    if "batt__voltage_v" in d.files:
        tb = d["batt__recv"] - t0; vb = d["batt__voltage_v"]
        res["V"] = float(np.median(np.interp(tq, tb, vb)))
    return res


def main():
    rows = []
    for p in sorted(glob.glob(os.path.join(HIST, "*.npz"))):
        nm = os.path.basename(p)[:-4]
        stamp = nm.split("_")[-2] + "_" + nm.split("_")[-1]
        r = analyse(p, nm)
        if r:
            r["stamp"] = stamp; rows.append(r)
    for k, v in RUNS.items():
        r = analyse(os.path.join(NPZ, f"{k}.npz"), v[2])
        if r:
            r["stamp"] = v[2].split("_")[-2] + "_" + v[2].split("_")[-1]; rows.append(r)
    rows.sort(key=lambda r: r["stamp"].replace("ps4", ""))
    print(f"{'flight':58s} {'rig':3s} {'n':>6s} {'V':>5s} {'T N':>6s} {'tau_z':>7s} {'(p10..p90)':>15s} {'lever mm':>8s} "
          f"{'first/last':>13s} {'dhat_rz':>7s} {'tau_x':>6s} {'tau_y':>6s}  u FR RL FL RR")
    for r in rows:
        print(f"{r['label']:58s} {r['rig']:3s} {r['n']:6d} {r.get('V', np.nan):5.2f} {r['T']:6.2f} {r['tau_z']:+7.3f} "
              f"({r['tau_z_p10']:+.3f}..{r['tau_z_p90']:+.3f}) {r['lever_mm']:+8.2f} {r['tau_z_first']:+6.3f}/{r['tau_z_last']:+6.3f} "
              f"{r.get('dhat_rz', np.nan):+7.3f} {r['tau_x']:+6.3f} {r['tau_y']:+6.3f}  {np.round(r['u'], 3)}")
    os.makedirs(OUT, exist_ok=True)
    json.dump(rows, open(os.path.join(OUT, "yaw_history.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
