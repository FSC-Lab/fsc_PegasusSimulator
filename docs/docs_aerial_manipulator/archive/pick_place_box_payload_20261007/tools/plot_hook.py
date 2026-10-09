#!/usr/bin/env python3
"""q2 / q3 against the real arm's limits, the basket's height and tilt, per
step, for hook pick-and-place runs (2026-10-07).

    PYTHONNOUSERSITE=1 /usr/bin/python3 plot_hook.py ../runs/hook5.npz [...] --out ../runs/hook_joints.png
"""
import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "pick_place_controllers_20261003", "tools"))
import pnp_score as PS  # noqa: E402

SHADE = {"pick": "#cfe3ff", "exit_pick": "#ffe1c4", "place": "#d8f2d0", "exit_place": "#f1d9f5"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    fig, axes = plt.subplots(3, len(a.runs), figsize=(7.5 * len(a.runs), 9), sharex="col", squeeze=False)
    for c, path in enumerate(a.runs):
        d = np.load(path, allow_pickle=True)
        names, mt = list(d["marks_name"]), d["marks_t"]
        t0 = mt[names.index("direct")]
        J, pay = d["joints"], d["payload"]
        t = J[:, 0] - t0
        ax = axes[0, c]
        ax.plot(t, np.degrees(J[:, 1]), lw=0.8, color="#2a9d3a", label="q1")
        ax.plot(t, np.degrees(J[:, 2]), lw=0.8, color="#1f6fb4", label="q2")
        ax.plot(t, np.degrees(J[:, 3]), lw=0.8, color="#c8551b", label="q3")
        ax.axhline(45.0, color="#1f6fb4", ls="--", lw=1)
        ax.axhline(-20.0, color="#1f6fb4", ls="--", lw=1)
        ax.axhline(50.0, color="#c8551b", ls="--", lw=1)
        ax.text(0.01, 45.5, "q2 stop +45", color="#1f6fb4", fontsize=8, transform=ax.get_yaxis_transform())
        ax.text(0.01, 50.5, "q3 stop +50", color="#c8551b", fontsize=8, transform=ax.get_yaxis_transform())
        ax.set_ylabel("joint [deg]")
        ax.set_ylim(-25, 55)
        ax.set_title(os.path.basename(path)[:-4])
        ax.legend(loc="lower right", fontsize=8)
        tp = pay[:, 0] - t0
        axes[1, c].plot(tp, pay[:, 3], lw=0.8, color="#333")
        axes[1, c].set_ylabel("basket centre z [m]")
        axes[2, c].plot(tp, PS.tilt_deg(pay[:, 4:8]), lw=0.8, color="#7a3fb0")
        axes[2, c].set_ylabel("basket tilt [deg]")
        axes[2, c].set_xlabel("time since DIRECT [s]")
        for s, col in SHADE.items():
            if f"{s}:start" in names and f"{s}:end" in names:
                for r in range(3):
                    axes[r, c].axvspan(mt[names.index(f"{s}:start")] - t0, mt[names.index(f"{s}:end")] - t0,
                                       color=col, alpha=0.8, lw=0)
        for s in ("pick", "exit_pick", "place", "exit_place"):
            if f"{s}:start" in names:
                axes[0, c].text(mt[names.index(f"{s}:start")] - t0, -23, s, fontsize=7, rotation=90, va="bottom")
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
