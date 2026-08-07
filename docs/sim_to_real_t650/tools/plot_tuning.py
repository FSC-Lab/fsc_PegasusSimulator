#!/usr/bin/env python3
"""Before/after of the t650_params tuning, against real flight C.

Baseline  = bench constants, 2.95 kg TOTAL       (sim_stack_C.npz)
Tuned     = k x1.0307, c x3.0, Ixx/Iyy x1.0307, 2.95 kg BODY  (sim_stack_C_tuned.npz)

Yaw is the axis the tuning targeted; the hover-thrust panel shows the other target.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_odom import detect_steps, load_real, load_sim

REAL, BASE, TUNE = "#c0392b", "#8c8c8c", "#1b6ca8"


def yaw_of(s):
    return np.degrees(np.unwrap(np.radians(s["eul"][:, 2])))


def thrust_on_sim_axis(npz):
    d = np.load(npz)
    od = d["odom"]; t0 = float(d["seq_t0_us"][0])
    w, p = od[:, 0].astype(float), od[:, 1].astype(float)
    keep = np.concatenate([[True], np.diff(w) > 0]); w, p = w[keep], p[keep]
    t = (np.interp(d["sp_ts"].astype(float), w, p) - t0) * 1e-6
    return t, -d["sp_thrust"][:, 2]


def main():
    real = load_real("exp_C.npz")
    base = load_sim("sim_stack_C.npz")
    tune = load_sim("sim_stack_C_tuned.npz")
    steps = detect_steps(real["ref_t"], real["ref_pos"], real["ref_yaw"])
    ysteps = [s for s in steps if s["axis"] == "yaw"]

    fig = plt.figure(figsize=(16, 9))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 1.0], hspace=0.32, wspace=0.24)

    # --- top row: the two yaw steps, before vs after ---
    for i, st in enumerate(ysteps):
        ax = fig.add_subplot(gs[0, i])
        for side, colour, lbl in ((real, REAL, "real"), (base, BASE, "sim baseline"),
                                  (tune, TUNE, "sim tuned")):
            y = yaw_of(side)
            m = (side["t"] >= st["t"] - 1.5) & (side["t"] <= st["t"] + 12.0)
            tw, yw = side["t"][m] - st["t"], y[m]
            if len(tw) < 3:
                continue
            y0 = yw[tw < -0.2].mean()
            ax.plot(tw, yw - y0, color=colour, lw=1.8 if side is not base else 1.3,
                    ls="--" if side is base else "-", label=lbl)
        ax.axhline(st["to"] - st["frm"], color="0.6", lw=0.9, ls=":")
        ax.axvline(0, color="0.85", lw=0.8)
        ax.set_title(f"yaw step {st['frm']:.0f} $\\to$ {st['to']:.0f} deg", fontsize=11)
        ax.set_xlabel("t since step [s]", fontsize=9)
        ax.set_ylabel(r"$\Delta$yaw [deg]", fontsize=9)
        ax.grid(alpha=0.25); ax.tick_params(labelsize=8)
        if i == 0:
            ax.legend(fontsize=9)

    # --- top-right: yaw metric summary ---
    ax = fig.add_subplot(gs[0, 2])
    labels = ["overshoot\n[%]", "rise 10-90\n[s x10]", "shape RMSE\n[deg]"]
    rvals = [9.158, 0.555 * 10, 0.0]
    bvals = [44.884, 0.840 * 10, 4.109]
    tvals = [7.804, 0.588 * 10, 1.268]
    x = np.arange(3); w = 0.27
    ax.bar(x - w, rvals, w, color=REAL, label="real")
    ax.bar(x, bvals, w, color=BASE, label="sim baseline")
    ax.bar(x + w, tvals, w, color=TUNE, label="sim tuned")
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=9)
    ax.set_title("yaw step metrics (n=2)", fontsize=11)
    ax.grid(alpha=0.25, axis="y"); ax.tick_params(labelsize=8); ax.legend(fontsize=8)

    # --- bottom-left/middle: whole-run yaw trace ---
    ax = fig.add_subplot(gs[1, :2])
    for side, colour, lbl, lw in ((real, REAL, "real", 1.2), (base, BASE, "sim baseline", 1.0),
                                  (tune, TUNE, "sim tuned", 1.2)):
        ax.plot(side["t"], yaw_of(side), color=colour, lw=lw,
                ls="--" if side is base else "-", label=lbl)
    ax.set_xlabel("time since first reference waypoint [s]", fontsize=10)
    ax.set_ylabel("yaw [deg]", fontsize=10)
    ax.set_title("whole-run yaw", fontsize=11)
    ax.grid(alpha=0.25); ax.legend(fontsize=9, ncol=3); ax.tick_params(labelsize=8)

    # --- bottom-right: hover thrust command ---
    ax = fig.add_subplot(gs[1, 2])
    e = np.load("exp_C.npz")
    te = (e["sp_ts"] - e["sp_ts"][0]) * 1e-6 - (e["ref_t"][0] - e["odom_t"][0])
    ax.plot(te, -e["sp_thrust"][:, 2], color=REAL, lw=1.0, label="real")
    for npz, colour, lbl, ls in (("sim_stack_C.npz", BASE, "sim baseline", "--"),
                                 ("sim_stack_C_tuned.npz", TUNE, "sim tuned", "-")):
        t, y = thrust_on_sim_axis(npz)
        ax.plot(t, y, color=colour, lw=1.0, ls=ls, label=lbl)
    ax.set_xlim(-5, 210); ax.set_ylim(0.49, 0.53)
    ax.set_xlabel("time [s]", fontsize=10)
    ax.set_ylabel("thrust command [-]", fontsize=10)
    ax.set_title("hover thrust command\n(real rise = battery sag, not modelled)", fontsize=10)
    ax.grid(alpha=0.25); ax.legend(fontsize=8); ax.tick_params(labelsize=8)

    fig.suptitle("T650 parameter tuning against flight C: bench constants vs tuned "
                 "(mass unchanged)", fontsize=13)
    fig.savefig("figures/C_tuning_before_after.png", dpi=110, bbox_inches="tight")
    print("wrote figures/C_tuning_before_after.png")


if __name__ == "__main__":
    main()
