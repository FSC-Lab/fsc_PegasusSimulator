"""Figure: D-1 (basket) vs E-1 (no payload) from the pick to the end of the mission, time-aligned on the leg
starts. Rows: vehicle pitch and roll (0.3-2 Hz), wrist roll q4 (measured) with its reference, arm yaw q1,
the j4 torque the law commands, and the basket across the claw (D-1).  -> ../figures/carry.png"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pd15_common import *   # noqa: F401,F403
from pp_common import quat_R

C_D1, C_E1, C_REF = "#2a78d6", "#eb6834", "#8a8984"
ANCH = ["exit_pick (up the vertical margin)", "go_to_place_start", "execute_place (approach)",
        "exit_place (up the vertical margin)", "go_to_land_start", "execute_land"]
SHORT = ["lift", "to drop start\n(180° turn, q1 0→12°)", "to drop\n(q1 12→0°)", "exit", "to land start", "land"]


def anchors(d, t0):
    w = leg_windows(d, t0)
    a = [w[k][0] for k in ANCH] + [w[ANCH[-1]][1]]
    return np.array(a)


def warp(t, src, dst):
    return np.interp(t, src, dst)


def main():
    runs = {}
    for r in ("d1", "e1"):
        d, t0 = load(r)
        runs[r] = (d, t0, anchors(d, t0))
    ref_anch = runs["d1"][2]
    fig, ax = plt.subplots(6, 1, figsize=(13, 13), sharex=True)
    for r, col, lab in (("e1", C_E1, "E-1 no payload"), ("d1", C_D1, "D-1 basket")):
        d, t0, an = runs[r]
        ta, rr, pp, _ = px4_rpy(d, t0)
        m = (ta >= an[0] - 3) & (ta <= an[-1])
        tu, op = band(np.degrees(pp[m]), ta[m], 0.3, 2.0)
        _, orr = band(np.degrees(rr[m]), ta[m], 0.3, 2.0)
        x = warp(tu, an, ref_anch)
        ax[0].plot(x, op, color=col, lw=1.2, label=lab)
        ax[1].plot(x, orr, color=col, lw=1.2, label=lab)
        tw, D = wbdebug(d, t0)
        mw = (tw >= an[0] - 3) & (tw <= an[-1])
        xw = warp(tw[mw], an, ref_anch)
        ax[2].plot(xw, np.degrees(D[mw, 8]), color=col, lw=1.2, label=lab)
        ax[3].plot(xw, np.degrees(D[mw, 5]), color=col, lw=1.2, label=lab)
        ax[4].plot(xw, D[mw, 16], color=col, lw=0.8, label=lab)
        if r == "d1":
            ax[2].plot(xw, np.degrees(D[mw, 12]), color=C_REF, lw=1.0, ls="--", label="q4 reference")
            ax[3].plot(xw, np.degrees(D[mw, 9]), color=C_REF, lw=1.0, ls="--", label="q1 reference")
            to, Po, Qo = obj_pose(d, t0)
            te = d["pl_current_ee__recv"] - t0
            Pe = np.column_stack([d[f"pl_current_ee__pose.position.{k}"] for k in "xyz"])
            Qe = np.column_stack([d[f"pl_current_ee__pose.orientation.{k}"] for k in "wxyz"])
            t_open = [t for t, n, mm in rosout(d, t0, contains="gripper OPEN")][0]
            mo = (to >= an[0] - 3) & (to <= t_open)
            j = np.clip(np.searchsorted(te, to[mo]), 0, len(te) - 1)
            rel = np.array([quat_R(*Qe[jj]).T @ (p - Pe[jj]) for p, jj in zip(Po[mo], j)])
            ax[5].plot(warp(to[mo], an, ref_anch), rel[:, 1] * 1e3, color=col, lw=1.2, label="basket across the claw")
            for a_ in ax:
                a_.axvline(warp(t_open, an, ref_anch), color="#0b0b0b", lw=1.0)
            ax[0].text(warp(t_open, an, ref_anch) + 0.3, ax[0].get_ylim()[1] if False else 3.2, "Drop (open)", fontsize=9)
    ylab = ["pitch, 0.3-2 Hz (deg)", "roll, 0.3-2 Hz (deg)", "wrist roll q4 (deg)", "arm yaw q1 (deg)",
            "j4 torque cmd (N·m)", "basket across claw (mm)"]
    for a_, yl in zip(ax, ylab):
        a_.set_ylabel(yl, fontsize=9)
        a_.grid(True, color="#e6e5e0", lw=0.6)
        for v in ref_anch:
            a_.axvline(v, color="#c3c2b7", lw=0.8, ls=":")
        a_.legend(loc="upper right", fontsize=8, frameon=False)
    for k in range(len(SHORT)):
        ax[0].text(0.5 * (ref_anch[k] + ref_anch[k + 1]), 4.2, SHORT[k], ha="center", va="bottom", fontsize=8, color="#52514e")
    ax[0].set_ylim(-4.5, 5.5)
    ax[-1].set_xlabel("time (s, D-1 clock; E-1 aligned on the leg starts)")
    fig.suptitle("2026-10-15 pick-and-drop: the loaded carry (D-1) vs the same legs with no payload (E-1)", fontsize=11)
    fig.tight_layout()
    out = os.path.join(FIG, "carry.png")
    fig.savefig(out, dpi=110)
    print("wrote", out)


if __name__ == "__main__":
    main()
