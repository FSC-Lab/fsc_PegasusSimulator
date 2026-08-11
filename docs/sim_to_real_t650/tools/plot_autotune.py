#!/usr/bin/env python3
"""Why PX4's autotune identified roll and pitch but failed on yaw, from the ULog.

The interesting panel is the last one: the identification input/output pair. Roll and pitch
answer the injected excitation strongly enough for the recursive least-squares estimate to
converge in ~5 s; yaw does not, and times out at 20 s.
"""
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog

STATE = {0: "IDLE", 1: "INIT", 2: "ROLL", 3: "ROLL_PAUSE", 4: "PITCH", 5: "PITCH_PAUSE",
         6: "YAW", 7: "YAW_PAUSE", 8: "VERIF", 9: "APPLY", 10: "TEST", 11: "COMPLETE",
         12: "FAIL", 13: "WAIT_DISARM"}
BANDS = [("ROLL", 2, 3, "#2166ac"), ("PITCH", 4, 5, "#1b7837"), ("YAW", 6, 12, "#c0392b")]


def main(ulg, out):
    u = ULog(ulg)
    d = {x.name: x for x in u.data_list}
    a = d["autotune_attitude_control_status"].data
    t = (a["timestamp"] - u.start_timestamp) / 1e6
    st = a["state"].astype(int)

    av = d["vehicle_angular_velocity"].data
    tv = (av["timestamp"] - u.start_timestamp) / 1e6

    t0, t1 = t[0] - 2, t[-1] + 2
    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)

    # shade each identification phase
    def shade(ax):
        for name, s_run, s_end, c in BANDS:
            idx = np.where(st == s_run)[0]
            if len(idx):
                ax.axvspan(t[idx[0]], t[idx[-1]], color=c, alpha=0.10)
                ax.text(t[idx[0]], ax.get_ylim()[1], f" {name}", color=c, fontsize=8,
                        va="top")

    ax = axes[0]
    ax.step(t, st, where="post", color="0.2", lw=1.2)
    ax.set_yticks(sorted(STATE)[:14])
    ax.set_yticklabels([STATE[k] for k in sorted(STATE)[:14]], fontsize=7)
    ax.set_ylabel("autotune state", fontsize=9)
    ax.grid(alpha=0.25)
    for name, s_run, s_end, c in BANDS:
        idx = np.where(st == s_run)[0]
        if len(idx):
            ax.axvspan(t[idx[0]], t[idx[-1]], color=c, alpha=0.12)

    ax = axes[1]
    for k, (name, c) in enumerate((("roll", "#2166ac"), ("pitch", "#1b7837"),
                                   ("yaw", "#c0392b"))):
        ax.plot(t, a["rate_sp[%d]" % k], color=c, lw=0.9, label=f"{name} excitation")
    ax.set_ylabel("injected rate sp\n[rad/s]", fontsize=9)
    ax.legend(fontsize=8, ncol=3); ax.grid(alpha=0.25); shade(ax)

    ax = axes[2]
    for k, (name, c) in enumerate((("roll", "#2166ac"), ("pitch", "#1b7837"),
                                   ("yaw", "#c0392b"))):
        ax.plot(tv, av["xyz[%d]" % k], color=c, lw=0.7, label=f"{name} rate")
    ax.set_xlim(t0, t1)
    ax.set_ylabel("measured body rate\n[rad/s]", fontsize=9)
    ax.legend(fontsize=8, ncol=3); ax.grid(alpha=0.25); shade(ax)

    ax = axes[3]
    ax.plot(t, a["u_filt"], color="0.45", lw=0.9, label="u_filt (sysid input)")
    ax.plot(t, a["y_filt"], color="#b35806", lw=1.1, label="y_filt (sysid output)")
    ax.set_ylabel("identification\nsignals", fontsize=9)
    ax.set_xlabel("time [s]", fontsize=10)
    ax.legend(fontsize=8, ncol=2); ax.grid(alpha=0.25); shade(ax)

    # per-phase response ratio, printed and annotated
    print(f"{'phase':6} {'dur[s]':>7} {'|u| rms':>9} {'|y| rms':>9} {'y/u':>7} {'fitness':>8}")
    for name, s_run, s_end, c in BANDS:
        m = st == s_run
        if not m.any():
            continue
        uu = np.sqrt(np.mean(a["u_filt"][m] ** 2))
        yy = np.sqrt(np.mean(a["y_filt"][m] ** 2))
        dur = t[m][-1] - t[m][0]
        print(f"{name:6} {dur:7.2f} {uu:9.4f} {yy:9.4f} {yy/uu if uu else np.nan:7.3f} "
              f"{a['fitness'][m][-1]:8.4f}")

    axes[0].set_title("PX4 attitude autotune against the IsaacSim T650 plant "
                      "(roll/pitch converge in ~5 s, yaw times out at 20 s)", fontsize=12)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
