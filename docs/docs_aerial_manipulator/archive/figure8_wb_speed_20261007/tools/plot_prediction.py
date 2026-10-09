#!/usr/bin/env python3
"""plot_prediction.py -- one figure: EE position rms vs mean speed on the A 0.70 / B 0.35 figure-8.
Hardware whole-body runs, hardware decoupled runs (and their band), the Isaac whole-body sweep, and the
predicted hardware whole-body band (envelope of models A-D, central = model C). Reads ../analysis/prediction.json.
    PYTHONNOUSERSITE=1 /usr/bin/python3 plot_prediction.py
"""
import json, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(os.path.join(HERE, "..", "analysis", "prediction.json")))
WB, DEC, SIM = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e6e5e0", "#fcfcfb"
pr = P["params"]; lin = np.array(P["sim_fit"])
sim = lambda v: np.polyval(lin, v)
v = np.linspace(0.10, 0.25, 300)
mA = pr["R"] * sim(v)
mB = np.sqrt((pr["c"] * sim(v)) ** 2 + pr["nB"] ** 2)
mC = np.sqrt((pr["c"] * sim(v)) ** 2 + pr["n0"] ** 2 + (pr["kC"] * v) ** 2)
mD = pr["aD"] + pr["bD"] * v
lo = np.min([mA, mB, mC, mD], 0); hi = np.max([mA, mB, mC, mD], 0)

fig, ax = plt.subplots(figsize=(8.6, 6.2), dpi=150)
fig.patch.set_facecolor(SURF); ax.set_facecolor(SURF)
dec = P["hw"]["dec"]
ax.axhspan(min(dec), max(dec), color=DEC, alpha=0.12, lw=0)
ax.text(0.0915, max(dec) + 1.2, "decoupled, hardware (72.6–76.9 mm)", color=INK2, ha="left", va="bottom", fontsize=9)
ax.fill_between(v, lo, hi, color=WB, alpha=0.16, lw=0, label="whole-body, predicted hardware (models A–D)")
ax.plot(v, mC, color=WB, lw=2, label="whole-body, predicted hardware (central, model C)")
ax.axvspan(0.2005, 0.26, color=GRID, alpha=0.6, lw=0)
ax.text(0.2025, 4, "sim-only planner bounds", color=INK2, fontsize=8.5, rotation=90, va="bottom")
hv, hr = np.array(P["hw"]["v"]), np.array(P["hw"]["rms"])
ax.plot(hv, hr, "o", ms=8, color=WB, mec=SURF, mew=2, label="whole-body, hardware 2026-10-05")
ax.plot([0.10, 0.13], dec, "s", ms=8, color=DEC, mec=SURF, mew=2, label="decoupled, hardware 2026-10-05")
raw = [r for r in P["runs"] if not r.get("no_run") and not r.get("fused") and not r["aborted"] and r["v"] <= 0.26]
fz = [r for r in P["runs"] if not r.get("no_run") and r.get("fused") and not r["aborted"]]
ax.plot([r["v"] for r in raw], [r["ee_rms"] for r in raw], "D", ms=7, color=SIM, mec=SURF, mew=2, label="whole-body, Isaac (raw mocap)")
if fz:
    ax.plot([r["v"] for r in fz], [r["ee_rms"] for r in fz], "D", ms=7, mfc=SURF, color=SIM, mew=2, label="whole-body, Isaac (EKF2-fused)")
i20 = np.argmin(np.abs(v - 0.20))
ax.annotate(f"0.20 m/s: {lo[i20]:.0f}–{hi[i20]:.0f} mm", (0.20, mC[i20]), (0.168, 88), color=INK, fontsize=9.5,
            arrowprops=dict(arrowstyle="-", color=INK2, lw=1))
ax.set_xlim(0.09, 0.26); ax.set_ylim(0, 100)
ax.set_xlabel("mean end-effector speed [m/s]", color=INK2); ax.set_ylabel("end-effector position rms [mm]", color=INK2)
ax.set_title("Figure-8 A 0.70 / B 0.35 m: whole-body EE error vs speed", color=INK, fontsize=11, loc="left")
ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True)
for s in ("top", "right"): ax.spines[s].set_visible(False)
for s in ("left", "bottom"): ax.spines[s].set_color(GRID)
ax.tick_params(colors=INK2)
leg = ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=2, fontsize=8.5, frameon=False)
for t in leg.get_texts(): t.set_color(INK)
fig.tight_layout()
out = os.path.join(HERE, "..", "figures", "prediction.png"); os.makedirs(os.path.dirname(out), exist_ok=True)
fig.savefig(out, facecolor=SURF); print("wrote", out)
