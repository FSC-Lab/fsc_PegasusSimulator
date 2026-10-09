#!/usr/bin/env python3
"""Joint window figure for the 2026-10-06 q2-limit runs: q2 / q3 through the push
step against the real arm's limits (q2 -20..45, q3 0..50) and the platform
deviation along the push, one column per run.

    PYTHONNOUSERSITE=1 /usr/bin/python3 plot_q2limit.py out.png tag[:label] ...
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pl_metrics as PM
from pl_metrics import mark, interp, Rz

TP, P = PM.TP, PM.P


def series(tag):
    z = np.load(f"../runs/{tag}.npz", allow_pickle=True)
    ps = mark(z, "push:start")
    pe = mark(z, "push:end")
    end = (pe + 2.0) if pe is not None else ps + 9.0
    j = z["joints"]
    m = (j[:, 0] > ps - 1.0) & (j[:, 0] < end)
    tj, q = j[m, 0] - ps, np.degrees(j[m, 1:5])
    ref, bt = z["ref"], z["base_truth"]
    xb = np.array([r[4:7] - Rz(np.arctan2(r[12], r[11])) @ TP.arm_fk_model(r[7:11], P)[0] for r in ref])
    tb = np.arange(ps - 1.0, min(end, bt[-1, 0], ref[-1, 0]), 0.02)
    b = interp(tb, bt, (1, 2, 3)) - np.column_stack([np.interp(tb, ref[:, 0], xb[:, i]) for i in range(3)])
    ab = bool(z["aborted"]) if "aborted" in z.files else False
    return tj, q, tb - ps, b, (pe - ps) if pe is not None else None, ab


def main():
    out = sys.argv[1]
    runs = [a.split(":", 1) if ":" in a else (a, a) for a in sys.argv[2:]]
    fig, ax = plt.subplots(2, len(runs), figsize=(4.2 * len(runs), 5.6), sharex=True, squeeze=False)
    for c, (tag, lab) in enumerate(runs):
        tj, q, tb, b, dur, ab = series(tag)
        a = ax[0, c]
        a.axhspan(45, 60, color="#d9534f", alpha=0.10, lw=0)
        a.axhspan(-60, 0, color="#d9534f", alpha=0.10, lw=0)
        a.axhline(45, color="#d9534f", lw=0.8, ls="--")
        a.axhline(50, color="#888", lw=0.8, ls=":")
        a.axhline(0, color="#d9534f", lw=0.8, ls="--")
        a.plot(tj, q[:, 1], color="#1f6fb4", lw=1.2, label="q2")
        a.plot(tj, q[:, 2], color="#e08a1e", lw=1.2, label="q3")
        a.set_ylim(-30, 55)
        a.set_title(lab + ("  (aborted)" if ab else ""), fontsize=9)
        if c == 0:
            a.set_ylabel("joint angle [deg]")
            a.legend(loc="lower left", fontsize=8, frameon=False)
        a2 = ax[1, c]
        a2.plot(tb, b[:, 0] * 1e3, color="#555", lw=1.0, label="x")
        a2.plot(tb, b[:, 1] * 1e3, color="#2a9d5c", lw=1.2, label="y (push axis)")
        a2.set_ylim(-60, 60)
        a2.axhline(0, color="#aaa", lw=0.6)
        a2.set_xlabel("time from grip [s]")
        if c == 0:
            a2.set_ylabel("base: true - planned [mm]")
            a2.legend(loc="lower left", fontsize=8, frameon=False)
        for aa in (a, a2):
            aa.axvline(1.5, color="#bbb", lw=0.6)
            if dur is not None:
                aa.axvline(dur, color="#bbb", lw=0.6)
            aa.tick_params(labelsize=8)
    fig.suptitle("Dashed red: q2 <= 45 deg (real arm) and q3 >= 0 (elbow-singular branch below); "
                 "dotted: q3's +50 deg stop. Grey lines: slide start (1.5 s) and push end.", fontsize=8.5)
    fig.tight_layout()
    fig.savefig(out, dpi=115)
    print("wrote", out)


if __name__ == "__main__":
    main()
