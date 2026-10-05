#!/usr/bin/env python3
"""Suarez et al., "Benchmarks for Aerial Manipulation", IEEE RA-L 5(2), 2020 --
the grasping benchmark (Sec. IV-B) metrics on the pick-and-place runs:

  eps_UAV(t) = || r_UAV^ref(t) - r_UAV(t) ||,   rho_UAV = eps_UAV / L      (Eq. 9)

r_UAV = the aerial platform (body origin) position, Earth frame (the odometry).
r_UAV^ref = the PLANNED body position: the planner streams the CoM reference
x_cd, the arm reference q_d and the heading b1_d (model frame), so
x_b_ref = x_cd - R_z(psi_d) r_0c(q_d) on the planner's own model (base_com
included). The neglected planned tilt is <= 3.5 deg at these accelerations
(< 3 mm at |r_0c| ~ 4 cm); the --check line verifies the reconstruction.
L = L1 + L2, the manipulator's maximum reach (Sec. III): OM-X shoulder -> elbow
0.130 m + elbow -> claw 0.132 + 0.108 m = 0.370 m (the whole-body model's links).

Per run: success; per phase (approach, grab, carry, place, return) max / rms
eps and max rho (the paper's "maximum deviation of the multirotor during the
grabbing phase" is the grab row); the way-point condition eps < 0.25 L at every
leg end; t_10% (time until eps <= 0.1 L and stays 1 s) after the lift-off and
after the release; phase times.

    PYTHONNOUSERSITE=1 /usr/bin/python3 bench_metrics.py [--check] [--plot out.png] runs...
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
RUNS = os.path.join(HERE, "..", "runs")
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

P = TP.make_params_t650(armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
P["base_com"] = np.array([0.0, -0.017854, 0.0])          # the planner's (sim mirror / pick-place yaml)
L = float(sum(np.linalg.norm(P["l_i"][k]) for k in (1, 2, 3)))   # 0.370 m
PHASES = [("approach", "go_to_start:start", "ready_pick:end"),
          ("grab", "pick:start", "exit_pick:end"),
          ("carry", "go_to_place_start:start", "ready_place:end"),
          ("place", "place:start", "exit_place:end"),
          ("return", "go_to_land_start:start", "execute_land:end")]
WAYPOINTS = ["go_to_start:end", "ready_pick:end", "exit_pick:end", "go_to_place_start:end",
             "ready_place:end", "exit_place:end", "go_to_land_start:end", "execute_land:end"]
PLACE_XY, CAP_TOP, BOX_HALF, CAP_R = np.array([-1.0, -1.0]), 1.0, 0.0325, 0.08


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ref_body(ref):
    """x_b_ref(t) from the streamed x_cd, q_d, b1_d (model frame)."""
    out = np.zeros((len(ref), 3))
    for k, row in enumerate(ref):
        r0c, _, _ = TP.arm_fk_model(row[7:11], P)
        phi = np.arctan2(row[13], row[12])
        out[k] = row[4:7] - Rz(phi) @ r0c
    return out


def analyse(fn):
    d = np.load(os.path.join(RUNS, fn + ".npz"), allow_pickle=True)
    od, ref = d["odom"], d["ref"]
    names, mt = list(d["marks_name"]), d["marks_t"]
    if ref.shape[1] < 15:
        raise SystemExit(f"{fn}: no b1_d in the reference record (run before 2026-10-03 23:30)")
    xb_ref = ref_body(ref)
    live = (od[:, 0] >= ref[0, 0]) & (od[:, 0] <= ref[-1, 0]) & (od[:, 11] > 0.5)
    t = od[live, 0]
    rr = np.stack([np.interp(t, ref[:, 0], xb_ref[:, i]) for i in range(3)], axis=1)
    eps = np.linalg.norm(rr - od[live, 1:4], axis=1)
    out = dict(run=fn, rig=str(d["rig"]), aborted=bool(d["aborted"]), reason=str(d["reason"]), t=t, eps=eps)
    pay = d["payload"]
    p_end = pay[-1, 1:4]
    w, x, y, z = pay[-1, 4:8]
    tilt_end = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    out["success"] = bool((not out["aborted"]) and np.linalg.norm(p_end[:2] - PLACE_XY) < CAP_R
                          and abs(p_end[2] - (CAP_TOP + BOX_HALF)) < 0.01 and tilt_end < 5.0)
    g = lambda s: mt[names.index(s)] if s in names else None   # noqa: E731
    out["phase"] = {}
    for nm, a, b in PHASES:
        ta, tb = g(a), g(b)
        if ta is None or tb is None:
            continue
        m = (t >= ta) & (t <= tb)
        if m.sum() < 5:
            continue
        out["phase"][nm] = dict(max=float(eps[m].max()), rms=float(np.sqrt(np.mean(eps[m] ** 2))),
                                T=float(tb - ta))
    out["wp"] = {}
    for s in WAYPOINTS:
        tw = g(s)
        if tw is None:
            continue
        m = (t >= tw - 0.2) & (t <= tw + 0.2)
        if m.any():
            out["wp"][s[:-4]] = float(np.mean(eps[m]))
    # t_10 % after the lift-off (the payload leaves the cap) and after the release
    out["t10"] = {}
    tc = g("gripper_close:end")
    if tc is not None:
        lift = np.where((pay[:, 0] > tc) & (pay[:, 3] > CAP_TOP + BOX_HALF + 0.005))[0]
        t_lift = pay[lift[0], 0] if len(lift) else None
        if t_lift is not None:
            out["t10"]["lift"] = t10(t, eps, t_lift, g("go_to_place_start:end"))
    to = g("gripper_open:start")
    if to is not None:
        out["t10"]["release"] = t10(t, eps, to, g("go_to_land_start:end"))
    out["check"] = check(d, ref, xb_ref)
    return out


def t10(t, eps, t0, t_end, hold=1.0):
    """time after t0 until eps <= 0.1 L and stays there >= hold s (NaN: never, before t_end)."""
    m = (t >= t0) & ((t <= t_end) if t_end is not None else True)
    tt, ee = t[m], eps[m]
    ok = ee <= 0.1 * L
    for i in range(len(tt)):
        if ok[i]:
            j = np.searchsorted(tt, tt[i] + hold)
            if j <= len(tt) and ok[i:j].all() and tt[min(j, len(tt) - 1)] - tt[i] >= hold * 0.9:
                return float(tt[i] - t0)
    return float("nan")


def check(d, ref, xb_ref):
    """Reconstruction check in the quiet hover before Ready To Pick: the measured
    CoM (odometry + encoders through the same model) against x_cd, and the body
    against x_b_ref -- they must differ by the same vector (the r_0c model)."""
    names, mt = list(d["marks_name"]), d["marks_t"]
    if "ready_pick:start" not in names:
        return None
    t0 = mt[names.index("ready_pick:start")] - 1.5
    od, J = d["odom"], d["joints"]
    m = (od[:, 0] > t0) & (od[:, 0] < t0 + 1.0)
    i = np.where(m)[0]
    if not len(i):
        return None
    k = i[len(i) // 2]
    w, x, y, z = od[k, 7:11]
    Rb = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                   [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                   [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    R0m = Rb @ Rz(-np.pi / 2)                             # R0_model = R0_actual Rz(-90)
    j = np.searchsorted(J[:, 0], od[k, 0])
    q_meas = J[min(j, len(J) - 1), 1:5]
    r0c_meas, _, _ = TP.arm_fk_model(q_meas, P)
    xc_meas = od[k, 1:4] + R0m @ r0c_meas
    r = np.searchsorted(ref[:, 0], od[k, 0])
    e_com = np.linalg.norm(ref[r, 4:7] - xc_meas)
    e_body = np.linalg.norm(xb_ref[r] - od[k, 1:4])
    return dict(e_com_mm=e_com * 1e3, e_body_mm=e_body * 1e3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--plot", default="")
    a = ap.parse_args()
    R = [analyse(r) for r in a.runs]
    print(f"L = L1 + L2 = {L:.3f} m;  0.25 L = {0.25 * L * 1e3:.0f} mm, 0.1 L = {0.1 * L * 1e3:.0f} mm\n")
    for r in R:
        ph = r["phase"]
        line = " | ".join(f"{k} {v['max'] * 1e3:5.1f}/{v['rms'] * 1e3:4.1f}" for k, v in ph.items())
        wp_bad = [k for k, v in r["wp"].items() if v >= 0.25 * L]
        t10s = " ".join(f"{k} {v:4.1f}s" for k, v in r["t10"].items())
        print(f"{r['run']:14s} {'OK  ' if r['success'] else 'FAIL'} eps max/rms mm: {line}   t10: {t10s}"
              f"   wp>0.25L: {wp_bad or 'none'}" + (f"   [{r['reason'][:50]}]" if r["aborted"] else ""))
        if a.check and r["check"]:
            print(f"{'':14s} check (hover): |x_cd - x_c,meas| {r['check']['e_com_mm']:.1f} mm, "
                  f"|x_b_ref - x_b| {r['check']['e_body_mm']:.1f} mm")
    print()
    for rig in sorted({r["rig"] for r in R}):
        rs = [r for r in R if r["rig"] == rig]
        print(f"== {rig}: success {sum(r['success'] for r in rs)}/{len(rs)}")
        for nm, _, _ in PHASES:
            v = [r["phase"][nm] for r in rs if nm in r["phase"]]
            if not v:
                continue
            mx = np.array([x["max"] for x in v]); rm = np.array([x["rms"] for x in v]); T = [x["T"] for x in v]
            print(f"   {nm:9s} eps max {mx.mean() * 1e3:6.1f} +- {mx.std() * 1e3:4.1f} mm (rho {mx.mean() / L:.3f})"
                  f"   rms {rm.mean() * 1e3:5.1f} mm (rho {rm.mean() / L:.3f})   time {np.mean(T):5.1f} s")
        for k in ("lift", "release"):
            v = [r["t10"].get(k, np.nan) for r in rs]
            n_ok = np.sum(np.isfinite(v))
            print(f"   t10 after {k:8s}: {np.nanmean(v) if n_ok else float('nan'):4.1f} s  (reached in {n_ok}/{len(rs)})")
        wpv = np.array([v for r in rs for v in r["wp"].values()])
        print(f"   way-points: max eps {wpv.max() * 1e3:5.1f} mm (rho {wpv.max() / L:.3f}); "
              f"{np.sum(wpv >= 0.25 * L)}/{len(wpv)} above 0.25 L")
    if a.plot:
        plot(R, a.plot)


def plot(R, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 1, figsize=(11, 6.5), sharex=False)
    ymax = max(0.3, max(np.max(r["eps"] / L) for r in R) * 1.08)     # ONE scale for both rigs
    cols = {"wb": "#1f6fb4", "geo": "#d9541e"}
    lab = {"wb": "whole-body 4-D L1", "geo": "decoupled geometric + L1"}
    for k, rig in enumerate(("wb", "geo")):
        rs = [r for r in R if r["rig"] == rig]
        for i, r in enumerate(rs):
            t0 = r["t"][0]
            ax[k].plot(r["t"] - t0, r["eps"] / L, color=cols[rig], lw=0.8, alpha=0.9 if i == 0 else 0.35,
                       label=lab[rig] if i == 0 else None)
        ax[k].axhline(0.25, color="k", ls="--", lw=0.8); ax[k].axhline(0.1, color="k", ls=":", lw=0.8)
        ax[k].set_ylabel(r"$\rho_{UAV} = \|\varepsilon_{UAV}\|/L$"); ax[k].legend(loc="upper right")
        ax[k].set_ylim(0, ymax); ax[k].grid(alpha=0.3)
    ax[1].set_xlabel("time in DIRECT [s]  (dashed 0.25 L, dotted 0.1 L)")
    fig.tight_layout(); fig.savefig(path, dpi=150)
    print(f"figure: {path}")


if __name__ == "__main__":
    main()
