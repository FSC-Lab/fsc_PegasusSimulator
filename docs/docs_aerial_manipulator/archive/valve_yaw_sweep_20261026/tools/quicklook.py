#!/usr/bin/env python3
"""Quick look at one or more sweep runs: tilt, q1/q4, yaw torque, the heading couple vs rendered,
F_t r, the beam lag -- over the push step +- a few seconds.  quicklook.py tag [tag ...] --out fig.png"""
import argparse, os, sys
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import yaw_score as YS
ap = argparse.ArgumentParser(); ap.add_argument("tags", nargs="+"); ap.add_argument("--out", required=True)
a = ap.parse_args()
fig, ax = plt.subplots(5, 1, figsize=(11, 12), sharex=True)
for tag in a.tags:
    row, S = YS.score(tag)
    t0 = S["a"]
    lab = f"{tag} (K {row['K_psi']:.3g}, tau {row['tau_f']:.3f})"
    m = (S["bt_t"] > S["tA"] - 3) & (S["bt_t"] < S["tE"] + 8)
    ax[0].plot(S["bt_t"][m] - t0, S["tilt"][m], label=lab)
    m = (S["js_t"] > S["tA"] - 3) & (S["js_t"] < S["tE"] + 8)
    l, = ax[1].plot(S["js_t"][m] - t0, S["q"][m, 0], label=lab + " q1")
    ax[1].plot(S["js_t"][m] - t0, S["q"][m, 3], "--", color=l.get_color(), label="q4")
    m = (S["dbg_t"] > S["tA"] - 3) & (S["dbg_t"] < S["tE"] + 8)
    ax[2].plot(S["dbg_t"][m] - t0, S["tz"][m], lw=0.6, label=lab)
    l, = ax[3].plot(S["t"] - t0, S["M"], label=lab + " measured")
    ax[3].plot(S["t"] - t0, S["R"], ":", color=l.get_color(), label="rendered")
    ax[4].plot(S["t"] - t0, S["Ft"] * S["r"], label=lab + " F_t r")
    ax[4].plot(S["t"] - t0, S["M"] + S["Ft"] * S["r"], ":", color=l.get_color(), label="M + F_t r")
    ax[4].axhline(row["tau_f"], color=l.get_color(), lw=0.5)
for x in ax:
    x.axvspan(0, 12, color="0.9"); x.grid(alpha=0.3); x.legend(fontsize=7)
ax[0].set_ylabel("tilt [deg]"); ax[1].set_ylabel("q1, q4 [deg]"); ax[2].set_ylabel("tau_z cmd [N.m]")
ax[3].set_ylabel("heading couple\nresisting + [N.m]"); ax[4].set_ylabel("about the axis [N.m]"); ax[4].set_xlabel("s from turn start")
fig.tight_layout(); fig.savefig(a.out, dpi=90); print("wrote", a.out)
