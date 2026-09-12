#!/usr/bin/env python3
"""Arm command-vs-delivered torque figures for a count<->torque mismatch run.

    /usr/bin/python3 wb_arm_torque_plot.py <run.npz> [--leg traj_both] [--out DIR]

Two figures, both over ONE leg of the mission (default the whole-system
compatible trajectory, where the base translates and yaws while all four joints
move at once):

  1_arm_torque_cmd_vs_applied.png
      per joint, what the whole-body law COMMANDED against what the servo
      actually DELIVERED. The gap is the plant's calibration error: the command
      chain converts N.m to Goal PWM counts with `nm_to_counts_nominal` and the
      winding makes torque per count with `nm_to_counts_true`, so delivered =
      nominal/true x commanded. Nothing tells the controller.

  2_arm_torque_gap_vs_l1.png
      the same gap against the L1 observer's arm-channel estimate. This is the
      non-circular half: the gap is a deterministic function of the command by
      construction, so what is worth plotting is whether the ESTIMATOR sees it.

DATA PROVENANCE, which is the whole point of keeping these two apart:
  * commanded  -- wb_control_debug[13..16], the law's own order joint1..joint4;
  * delivered  -- the driver's log `tau_app*`, taken from the arm
                  joint_states .effort that Isaac fills with the torque the
                  servo model actually applied, resolved BY NAME into model
                  order (the broadcaster publishes [j2, j3, j1, j4]);
  * L1 estimate -- wb_control_debug[31..40], d_e_hat FILTERED (what the loops
                  see); entries [6..9] of that block are the arm joints.

The two sources are on the same driver clock, so they are interpolated onto a
common grid rather than assumed sample-aligned.
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# dbg column offsets (dbg[:, 0] is t, so debug index i is column i+1)
C_TAU = slice(1 + 13, 1 + 17)        # commanded joint torque, model order
C_DHAT_ARM = slice(1 + 31 + 6, 1 + 31 + 10)   # d_e_hat, arm channels
# log columns
L_T, L_DIRECT = 0, 17
L_TAU_APP = slice(18, 22)

# Duty counts per N.m -- one count is 1/this, the register's step. Taken from
# the plant model rather than copied, so a re-calibration cannot leave this
# tool scoring against the previous arm: the 2026-09-11 campaign moved it from
# [155.54, 168.95, 125.14, 155.54] to [169.47, 149.70, 135.25, 148.51], which
# changes the count floor this file uses to decide whether a coverage ratio
# means anything at all. servo_model.py is pure numpy -- no Isaac import.
sys.path.insert(0, str(Path(__file__).resolve().parents[3]
                       / "extensions" / "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.servo_model import NM_TO_DUTY  # noqa: E402

JOINTS = ("joint 1  (arm yaw)", "joint 2  (shoulder)",
          "joint 3  (elbow)", "joint 4  (wrist roll)")

INK = "#141A1F"
FAINT = "#7D8C96"
RULE = "#D6DDE1"
CMD = "#0E6F79"      # accent
APP = "#A6482A"      # warn
GAP = "#141A1F"
L1C = "#0E6F79"


def load(path, leg, pad):
    z = np.load(path, allow_pickle=True)
    log, dbg = z["log"], z["dbg"]
    if log.shape[1] < 22:
        raise SystemExit(
            f"{path}: no tau_app columns -- this run predates the driver's "
            "applied-torque logging, refly it")

    marks = []
    for entry in z["leg_marks"]:
        tt, _, nm = str(entry).partition(" ")
        marks.append((float(tt), nm))
    if not marks:
        raise SystemExit(f"{path}: no leg marks")
    names = [n for _, n in marks]
    if leg not in names:
        raise SystemExit(f"{path}: leg {leg!r} not in {names}")
    k = names.index(leg)
    t0 = marks[k][0]
    t1 = marks[k + 1][0] if k + 1 < len(marks) else log[-1, L_T]

    lt = log[:, L_T]
    m = (lt >= t0 - pad) & (lt <= t1 + pad)
    t = lt[m]
    tau_app = log[m][:, L_TAU_APP]

    # Commanded and the estimate come off the debug array; put them on the
    # log's clock rather than assuming the two streams line up sample for
    # sample (they are separate subscriptions at different rates).
    dt_, d = dbg[:, 0], dbg
    tau_cmd = np.stack([np.interp(t, dt_, d[:, C_TAU][:, j]) for j in range(4)], 1)
    dhat = np.stack([np.interp(t, dt_, d[:, C_DHAT_ARM][:, j]) for j in range(4)], 1)
    return t - t0, tau_cmd, tau_app, dhat, t1 - t0, leg


def standing_offset(path, settle=20.0):
    """Arm-channel d_hat during the matched run's pure static hold, N.m.

    The window is DIRECT entry + settle up to the FIRST leg mark: the vehicle
    is holding the anchor it captured at entry, so both flights are in the same
    configuration and the comparison is valid. It is NOT valid later -- after a
    leg each flight sits wherever its own planner solution left it, and the
    commanded torques then differ by ~100 mN.m between runs, more than the
    effect being measured. That is why this is a single CONSTANT taken from one
    comparable window rather than a sample-by-sample paired difference.
    """
    z = np.load(path, allow_pickle=True)
    dbg = z["dbg"]
    t, d = dbg[:, 0], dbg
    direct = np.nan_to_num(d[:, 1 + 0]) > 0.5
    if not direct.any():
        raise SystemExit(f"{path}: never entered DIRECT")
    t_dir = t[direct][0]
    marks = [float(str(e).partition(" ")[0]) for e in z["leg_marks"]]
    t_end = marks[0] if marks else t[direct][-1]
    w = direct & (t >= t_dir + settle) & (t < t_end)
    if w.sum() < 50:
        raise SystemExit(f"{path}: hold window too short ({w.sum()} samples)")
    off = np.nanmean(d[w][:, C_DHAT_ARM], axis=0)
    return off, float(t[w][-1] - t[w][0])


def style(ax, xlab=False):
    ax.set_facecolor("white")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("bottom", "left"):
        ax.spines[s].set_color(RULE)
    ax.grid(True, color=RULE, lw=.6, alpha=.7)
    ax.set_axisbelow(True)
    ax.tick_params(colors=FAINT, labelsize=8.5, length=3)
    if xlab:
        ax.set_xlabel("time in leg [s]", fontsize=9, color=INK)


def fig_cmd_vs_applied(t, cmd, app, dur, leg, out):
    fig, axes = plt.subplots(4, 1, figsize=(9.2, 8.6), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t, cmd[:, j], color=CMD, lw=1.5, label="commanded by the law",
                zorder=3)
        ax.plot(t, app[:, j], color=APP, lw=1.5, ls="--",
                label="delivered by the servo", zorder=4)
        ax.fill_between(t, cmd[:, j], app[:, j], color=APP, alpha=.13,
                        lw=0, zorder=2)
        ax.axvspan(0, dur, color="#0E6F79", alpha=.045, lw=0, zorder=1)
        ax.set_ylabel("N·m", fontsize=9, color=INK)
        style(ax, xlab=(j == 3))
        pk = np.nanmax(np.abs(cmd[:, j] - app[:, j]))
        ax.text(.006, .93, JOINTS[j], transform=ax.transAxes, fontsize=9.5,
                fontweight="bold", color=INK, va="top")
        ax.text(.994, .06,
                f"peak shortfall {pk * 1e3:.0f} mN·m", ha="right",
                transform=ax.transAxes, fontsize=8.5, color=FAINT,
                va="bottom")
    fig.suptitle("Arm joint torque: commanded against delivered  —  "
                 f"{leg}, +10 % calibration mismatch",
                 fontsize=11.5, fontweight="bold", color=INK, y=.985)
    fig.tight_layout(rect=(0, 0, 1, .945))
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", bbox_to_anchor=(.5, .955),
               ncol=2, frameon=False, fontsize=9)
    p = os.path.join(out, "1_arm_torque_cmd_vs_applied.png")
    fig.savefig(p, dpi=150, facecolor="white")
    plt.close(fig)
    return p


def fig_gap_vs_l1(t, cmd, app, dcomp, dur, leg, out, paired):
    """Gap against the L1 arm-channel compensation.

    `dcomp` is ALREADY the isolated compensation. Raw -d_hat cannot be plotted
    against the gap: on this config the arm channels carry a large standing
    disturbance from the OTHER injections (mass and inertia x1.10, 10 mm CoM),
    measured at +0.212 / +0.084 N.m on joints 2 / 3 with a perfectly calibrated
    arm, which swamps the ~0.08 N.m the calibration error adds. What isolates
    the arm term is the PAIRED DIFFERENCE against a matched-calibration flight
    of the same leg -- everything else in the plant is identical, so what is
    left is what the observer booked for the servo.
    """
    gap = cmd - app
    fig, axes = plt.subplots(4, 1, figsize=(9.2, 8.6), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t, gap[:, j] * 1e3, color=GAP, lw=1.6,
                label="torque gap, commanded − delivered", zorder=4)
        ax.plot(t, dcomp[:, j] * 1e3, color=L1C, lw=1.7, ls="--",
                label="L1 compensation, −(d̂ − standing offset)", zorder=3)
        ax.axhline(0, color=RULE, lw=.8, zorder=1)
        ax.axvspan(0, dur, color="#0E6F79", alpha=.045, lw=0, zorder=1)
        ax.set_ylabel("mN·m", fontsize=9, color=INK)
        style(ax, xlab=(j == 3))
        ax.text(.006, .93, JOINTS[j], transform=ax.transAxes, fontsize=9.5,
                fontweight="bold", color=INK, va="top")
        g, c = gap[:, j], dcomp[:, j]
        ok = np.isfinite(g) & np.isfinite(c)
        # A coverage ratio only means something where the gap is bigger than
        # the register's own step. On a joint whose command never leaves the
        # count floor the "gap" is quantization noise about zero and the ratio
        # is arbitrary -- joints 1 and 4 carry 0.02-0.04 N.m here.
        quantum = 1.0 / NM_TO_DUTY[j]
        if ok.sum() > 20 and abs(np.nanmean(g[ok])) > 2.0 * quantum:
            cov = float(np.nanmean(c[ok]) / np.nanmean(g[ok])) * 100.0
            ax.text(.994, .06, f"covered {cov:.0f} %", ha="right",
                    transform=ax.transAxes, fontsize=8.5, color=FAINT,
                    va="bottom")
        else:
            ax.text(.994, .06, "command below the count floor", ha="right",
                    transform=ax.transAxes, fontsize=8.5, color=FAINT,
                    va="bottom")
    fig.suptitle("The torque the servo did not deliver, against what the L1 "
                 f"observer books for it  —  {leg}",
                 fontsize=11.5, fontweight="bold", color=INK, y=.985)
    fig.tight_layout(rect=(0, 0, 1, .945))
    h, l = axes[0].get_legend_handles_labels()
    fig.legend(h, l, loc="upper center", bbox_to_anchor=(.5, .955),
               ncol=2, frameon=False, fontsize=9)
    p = os.path.join(out, "2_arm_torque_gap_vs_l1.png")
    fig.savefig(p, dpi=150, facecolor="white")
    plt.close(fig)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--leg", default="traj_both")
    ap.add_argument("--pad", type=float, default=1.5,
                    help="seconds of hold to show either side of the leg")
    ap.add_argument("--matched", required=True,
                    help="npz of the MATCHED-calibration flight of the same "
                         "leg; the arm compensation is isolated as the paired "
                         "difference against it")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(a.npz)), "fig")
    os.makedirs(out, exist_ok=True)

    t, cmd, app, dhat, dur, leg = load(a.npz, a.leg, a.pad)
    print(f"leg {leg}: {dur:.2f} s, {len(t)} samples")
    gap = cmd - app
    with np.errstate(invalid="ignore", divide="ignore"):
        frac = np.where(np.abs(cmd) > 0.02, gap / cmd, np.nan)
    for j in range(4):
        print(f"  joint{j + 1}: |cmd| max {np.nanmax(np.abs(cmd[:, j])):.4f}  "
              f"gap mean {np.nanmean(gap[:, j]) * 1e3:+7.2f} mN.m  "
              f"peak {np.nanmax(np.abs(gap[:, j])) * 1e3:6.2f}  "
              f"gap/cmd {np.nanmean(frac[:, j]):.4f}")
    off, hold_s = standing_offset(a.matched)
    print(f"standing arm-channel d_hat from the matched run's {hold_s:.0f} s "
          f"hold: {np.round(off, 4).tolist()} N.m")
    dcomp = -(dhat - off)
    for j in range(4):
        print(f"  joint{j + 1}: gap mean {np.nanmean(gap[:, j]) * 1e3:+7.2f}  "
              f"compensation mean {np.nanmean(dcomp[:, j]) * 1e3:+7.2f} mN.m")
    print(fig_cmd_vs_applied(t, cmd, app, dur, leg, out))
    print(fig_gap_vs_l1(t, cmd, app, dcomp, dur, leg, out, a.matched))


if __name__ == "__main__":
    main()
