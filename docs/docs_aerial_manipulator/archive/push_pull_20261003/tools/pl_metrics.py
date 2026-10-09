#!/usr/bin/env python3
"""The push-and-pull EVALUATION metrics (user's definitions, 2026-10-05), from a
pl_mission.py recording made with the contact-truth scene (07, 2026-10-05):

    PYTHONNOUSERSITE=1 /usr/bin/python3 pl_metrics.py ../runs/pl_15.npz [more ...]

1. IMPEDANCE RESIDUAL  e_imp = F_ext - (M_d edot_v + D_d e_v + K_d e_y), per task
   channel: x, y, z [N] and the heading [N.m]. M_d / D_d / K_d = the law's
   wb_my_* / wb_dy_* / wb_ky_* (read from the yaml the run flew, --yaml). The law
   renders M_y e_ddot + D_y e_dot + K_y e = F_ext with e = y - y_d, so e_imp = 0
   is a perfect impedance.
     F_ext   the TRUE wrench the environment puts on the claw: minus the box's
             PhysX contact wrench from the vehicle (normal + friction anchors),
             heading = its moment about world z at the claw point.
     e_y     position: the TRUE claw point (claw_0) minus the streamed r_ed;
             heading: the law's own e_y[3] (attitude and joints are exact in
             both feedback modes; only the base position estimate differs).
     e_v, edot_v  derivatives of e_y (claw truth differentiated; reference
             derivatives streamed by the planner, r_ed_dot / r_ed_ddot).
   Every term goes through the SAME zero-phase low-pass (--fc, default 5 Hz) so
   the comparison is made inside the law's bandwidth, not on 62.5 Hz contact
   noise.
2. PLATFORM DEVIATION  eps_UAV = || r_UAV^ref - r_UAV ||,  rho_UAV = eps / L,
   L = L1 + L2 = 0.370 m -- exactly the pick-and-place benchmark's
   (pick_place_controllers_20261003/tools/bench_metrics.py): r_UAV^ref = the
   planned airframe x_cd - R_z(phi_d) r_0c(q_d) on the planner's model; r_UAV =
   the TRUE airframe pose (Isaac /uav_0/state/pose) when recorded, else the
   odometry.

Windows (the driver's marks): CONTACT = end of Ready -> start of Exit;
inside it HOLD (Ready end -> slide start, the grip and the 1.5 s settle), SLIDE
(the 12 s move), RELEASE (Push end -> Exit start). eps also over APPROACH
(Go To Start + Ready) and EXIT. The contact sign convention is CALIBRATED per run
on the table (its normal force holds the box up; its friction opposes the
slide) and printed, with the slide-force check F_ext ~ mu m g.
"""
import argparse
import os
import re
import sys

import numpy as np
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

YAML = os.path.expanduser(
    "~/ros2_ws/src/fsc_autopilot_ros2/config/"
    "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_push_pull.yaml")
P = TP.make_params_t650(armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
P["base_com"] = np.array([0.0, -0.017854, 0.0])
L = float(sum(np.linalg.norm(P["l_i"][k]) for k in (1, 2, 3)))     # 0.370 m
D_EY_HDG = 24 + 3 + 2                    # wb_control_debug e_y[3], recorder column
SLIDE_DELAY = 1.5                        # the planner's push settle [s]


def yaml_gains(path):
    txt = open(path).read()
    g = {}
    for k in ("my", "dy", "ky"):
        g[k] = np.array([float(re.search(rf"^\s*wb_{k}_{a}:\s*([-+0-9.eE]+)", txt, re.M).group(1))
                         for a in ("x", "y", "z", "psi")])
    return g


def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def mark(z, name):
    names = list(z["marks_name"])
    return float(z["marks_t"][names.index(name)]) if name in names else None


def interp(t, src, cols):
    return np.column_stack([np.interp(t, src[:, 0], src[:, c]) for c in cols])


def lpf(x, fs, fc):
    b, a = butter(2, fc / (0.5 * fs))
    return filtfilt(b, a, x, axis=0)


def stats(v):
    v = np.asarray(v)
    return (float(np.sqrt(np.mean(v ** 2))), float(np.max(np.abs(v)))) if len(v) else (np.nan, np.nan)


def analyse(path, gains, fc):
    z = np.load(path, allow_pickle=True)
    tag = os.path.splitext(os.path.basename(path))[0]
    out = {"run": tag, "aborted": bool(z["aborted"]), "reason": str(z["reason"])}
    ct, claw, ref, refd, dbg = z["contact"], z["claw"], z["ref"], z["refd"], z["dbg"]
    if ct.size <= 1 or refd.size <= 1:
        raise SystemExit(f"{tag}: no contact truth / reference derivatives (recorded before 2026-10-05)")
    t_ready, t_push, t_pend, t_exit = (mark(z, n) for n in ("ready:end", "push:start", "push:end", "exit:start"))
    t_end_contact = t_exit if t_exit is not None else ct[-1, 0]
    # ---- the contact sign convention, calibrated on the table --------------
    tn, tf = ct[:, 7:10], ct[:, 10:13]
    sn = 1.0 if np.nanmean(tn[np.abs(tn[:, 2]) > 0.5, 2]) > 0 else -1.0
    box = z["box"]
    sl0 = t_push + SLIDE_DELAY if t_push is not None else None
    sf = sn
    if sl0 is not None:
        w = (ct[:, 0] > sl0 + 2.0) & (ct[:, 0] < min(sl0 + 10.0, t_end_contact))
        if w.sum() > 10:
            vb = np.gradient(interp(ct[w, 0], box, (1, 2, 3)), ct[w, 0], axis=0)
            sf = 1.0 if np.nanmean(np.sum(tf[w] * vb, axis=1)) < 0 else -1.0
    out["sign"] = (sn, sf)
    F_box = sn * ct[:, 1:4] + sf * ct[:, 4:7]              # on the box, by the vehicle
    M_box_O = sn * ct[:, 13:16] + sf * ct[:, 16:19]
    table_N = sn * tn
    # ---- common time base: the contact samples inside CONTACT --------------
    w = (ct[:, 0] >= t_ready) & (ct[:, 0] <= t_end_contact)
    t = ct[w, 0]
    fs = 1.0 / np.median(np.diff(t))
    c = interp(t, claw, (1, 2, 3))
    F_ext = -F_box[w]
    M_claw = -(M_box_O[w] - np.cross(c, F_box[w]))           # moment ON the claw about the claw
    Fx = np.column_stack([F_ext, M_claw[:, 2]])              # task wrench [N, N, N, N.m]
    r = interp(t, ref, (1, 2, 3))
    rd = interp(t, refd, (1, 2, 3))
    rdd = interp(t, refd, (4, 5, 6))
    hd = np.interp(t, dbg[:, 0], dbg[:, D_EY_HDG])
    e_y = np.column_stack([c - r, hd])
    e_y_f = lpf(e_y, fs, fc)
    ydot = np.gradient(e_y_f, t, axis=0)
    cdot = np.gradient(lpf(c, fs, fc), t, axis=0)
    cddot = np.gradient(cdot, t, axis=0)
    e_v = np.column_stack([cdot - lpf(rd, fs, fc), ydot[:, 3]])
    e_a = np.column_stack([cddot - lpf(rdd, fs, fc), np.gradient(ydot[:, 3], t)])
    Fx_f = lpf(Fx, fs, fc)
    rendered = gains["my"] * e_a + gains["dy"] * e_v + gains["ky"] * e_y_f
    e_imp = Fx_f - rendered
    out["fs"] = fs
    # ---- windows ------------------------------------------------------------
    wins = {"CONTACT": (t_ready, t_end_contact)}
    if t_push is not None:
        wins["HOLD"] = (t_ready, t_push + SLIDE_DELAY)
        wins["SLIDE"] = (t_push + SLIDE_DELAY, t_pend if t_pend is not None else t_end_contact)
    if t_pend is not None:
        wins["RELEASE"] = (t_pend, t_end_contact)
    out["imp"] = {}
    for k, (a, b) in wins.items():
        m = (t >= a) & (t <= b)
        if m.sum() < 5:
            continue
        out["imp"][k] = dict(
            T=b - a,
            e_xyz=stats(np.linalg.norm(e_imp[m, :3], axis=1)),
            e_psi=stats(e_imp[m, 3]),
            F_xyz=stats(np.linalg.norm(Fx_f[m, :3], axis=1)),
            F_psi=stats(Fx_f[m, 3]),
            ey_mm=stats(np.linalg.norm(e_y_f[m, :3], axis=1) * 1e3),
            e_comp=np.sqrt(np.mean(e_imp[m, :3] ** 2, axis=0)))
    # slide check: F_ext along the push (world -y here) vs the table's friction
    if "SLIDE" in wins:
        a, b = wins["SLIDE"]
        m = (ct[:, 0] > a + 2.0) & (ct[:, 0] < b - 2.0)
        if m.sum() > 5:
            out["slide_check"] = (np.mean(-F_box[m], axis=0), np.mean(sf * tf[m], axis=0),
                                  float(np.mean(table_N[m, 2])))
    # ---- eps_UAV / rho_UAV ----------------------------------------------------
    bt = z["base_truth"] if "base_truth" in z.files and z["base_truth"].size > 1 else None
    src = "truth" if bt is not None else "odometry"
    pos = bt if bt is not None else z["odom"]
    xb_ref = np.zeros((len(ref), 3))
    for k, row in enumerate(ref):
        r0c, _, _ = TP.arm_fk_model(row[7:11], P)
        xb_ref[k] = row[4:7] - Rz(np.arctan2(row[12], row[11])) @ r0c
    tt = pos[(pos[:, 0] >= ref[0, 0]) & (pos[:, 0] <= ref[-1, 0]), 0]
    rr = np.column_stack([np.interp(tt, ref[:, 0], xb_ref[:, i]) for i in range(3)])
    pp = interp(tt, pos, (1, 2, 3))
    eps = np.linalg.norm(rr - pp, axis=1)
    out["eps_src"] = src
    ewins = {"APPROACH": (mark(z, "go_to_start:start"), t_ready)}
    ewins.update(wins)
    ewins["EXIT"] = (t_exit, mark(z, "exit:end"))
    out["eps"] = {}
    for k, (a, b) in ewins.items():
        if a is None or b is None:
            continue
        m = (tt >= a) & (tt <= b)
        if m.sum() < 5:
            continue
        rms, mx = stats(eps[m])
        out["eps"][k] = (mx, rms)
    out["series"] = dict(t=t, e_imp=e_imp, F=Fx_f, rendered=rendered, e_y=e_y_f, eps_t=tt, eps=eps)
    return out


def report(o):
    print(f"\n=== {o['run']}  aborted={o['aborted']} {o['reason'][:90]}")
    print(f"contact sign (normal, friction) = {o['sign']}  (calibrated on the table)   fs {o['fs']:.1f} Hz")
    if "slide_check" in o:
        f, tfric, N = o["slide_check"]
        print(f"slide check: F_ext on claw [{f[0]:+.2f} {f[1]:+.2f} {f[2]:+.2f}] N | table friction on box "
              f"[{tfric[0]:+.2f} {tfric[1]:+.2f} {tfric[2]:+.2f}] N | table normal {N:.2f} N")
    print(f"{'window':8s} {'T':>5s} | {'e_imp xyz rms/pk N':>19s} {'e_imp psi rms/pk N.m':>21s} | "
          f"{'F_ext rms/pk N':>15s} {'|e_y| rms/pk mm':>16s} | e_imp x/y/z rms N")
    for k, v in o["imp"].items():
        print(f"{k:8s} {v['T']:5.1f} | {v['e_xyz'][0]:8.3f} / {v['e_xyz'][1]:6.3f}   "
              f"{v['e_psi'][0]:9.4f} / {v['e_psi'][1]:7.4f}   | {v['F_xyz'][0]:6.2f} / {v['F_xyz'][1]:5.2f}  "
              f"{v['ey_mm'][0]:7.1f} / {v['ey_mm'][1]:6.1f} | "
              f"{v['e_comp'][0]:.3f} {v['e_comp'][1]:.3f} {v['e_comp'][2]:.3f}")
    print(f"eps_UAV ({o['eps_src']}), L = {L:.3f} m:  " + "   ".join(
        f"{k} max {mx * 1e3:.0f} mm (rho {mx / L:.3f}) rms {rm * 1e3:.0f}" for k, (mx, rm) in o["eps"].items()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--yaml", default=YAML, help="the yaml the run flew (impedance gains)")
    ap.add_argument("--fc", type=float, default=5.0, help="common zero-phase low-pass [Hz]")
    ap.add_argument("--plot", help="write a figure (residual, force, eps) of the runs")
    a = ap.parse_args()
    gains = yaml_gains(a.yaml)
    print(f"M_d {gains['my']}  D_d {gains['dy']}  K_d {gains['ky']}  (from {os.path.basename(a.yaml)})")
    outs = [analyse(p, gains, a.fc) for p in a.runs]
    for o in outs:
        report(o)
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=False)
        for o in outs:
            s = o["series"]
            t0 = s["t"][0]
            ax[0].plot(s["t"] - t0, np.linalg.norm(s["e_imp"][:, :3], axis=1), label=o["run"])
            ax[1].plot(s["t"] - t0, s["F"][:, 1], label=o["run"])
            m = (s["eps_t"] >= t0) & (s["eps_t"] <= s["t"][-1])
            ax[2].plot(s["eps_t"][m] - t0, s["eps"][m] / L, label=o["run"])
        ax[0].set_ylabel("|e_imp| xyz [N]")
        ax[1].set_ylabel("F_ext world y [N]\n(+ = resists the push)")
        ax[2].set_ylabel(r"$\rho_{UAV}$")
        ax[2].set_xlabel("t from CONTACT on [s]")
        for x in ax:
            x.grid(alpha=0.3)
            x.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(a.plot, dpi=110)
        print(f"wrote {a.plot}")


if __name__ == "__main__":
    main()
