"""Figures for the 2026-09-29 PS4 teleoperation flight. Run: PYTHONNOUSERSITE=1 /usr/bin/python3 figures.py
Writes ../figures/*.png. Needs p1.npz (and x_*.npz for the revert figure) in $AM_NPZ after np1_compat.py."""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from common import *

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "figures")
os.makedirs(OUT, exist_ok=True)
C_REF, C_MEAS, C_TGT, C_GRAY = "#2a78d6", "#eb6834", "#52514e", "#b9b8b2"
C_PAD = {"com_fwd": "#2a78d6", "com_left": "#1baf7a", "ee_fwd": "#eb6834", "ee_up": "#e87ba4",
         "ee_left": "#eda100", "roll": "#4a3aa7", "home": "#0b0b0b"}
plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                     "axes.grid": True, "grid.color": "#e6e5e0", "grid.linewidth": 0.6, "lines.linewidth": 1.2,
                     "legend.frameon": False, "figure.dpi": 130})

d, t0 = load("p1")
a, b, c, e = windows(d, t0)
tw, D = wbdebug(d, t0)
T = d["tel_state__data"]; tt = d["tel_state__recv"] - t0; ok = np.isfinite(T[:, 0]); T = T[ok]; tt = tt[ok]
tj, ch = pad(d, t0)
tq, qm, _ = joints_model(d, t0)
tv = d["vatt__recv"] - t0; tilt = tilt_deg(*d["vatt__q"].T)


def shade_pad(ax, chans=None, y=None):
    for k, v in ch.items():
        if chans and k not in chans:
            continue
        for r0, r1 in runs((np.abs(v) > 0) & (tj > c) & (tj < e)):
            ax.axvspan(tj[r0], tj[min(r1, len(tj) - 1)], color=C_PAD.get(k, C_GRAY), alpha=0.25, lw=0)


def phases(ax):
    for x in (a, c, e, b):
        ax.axvline(x, color="#0b0b0b", lw=0.6, ls=":")


# ---------------- Fig 1: overview
fig, axs = plt.subplots(5, 1, figsize=(11, 10), sharex=True, gridspec_kw=dict(height_ratios=[0.8, 1.4, 1.4, 1.2, 1.2]))
ax = axs[0]
rows = ["com_fwd", "com_left", "ee_fwd", "ee_up", "home"]
lab = {"com_fwd": "D-pad fwd/back", "com_left": "D-pad left/right", "ee_fwd": "L-stick EE fwd", "ee_up": "R-stick EE up/down", "home": "PS (arm home)"}
for i, k in enumerate(rows):
    v = ch[k]
    for r0, r1 in runs((np.abs(v) > 0) & (tj > c) & (tj < e)):
        sgn = np.sign(v[r0:r1].mean())
        ax.broken_barh([(tj[r0], max(0.15, tj[min(r1, len(tj) - 1)] - tj[r0]))], (i - 0.35, 0.7),
                       color=C_PAD[k] if sgn >= 0 else C_PAD[k], alpha=1.0 if sgn >= 0 else 0.45)
bt = d["joy__buttons"]
for bi, nm in ((4, "L1 open"), (5, "R1 close")):
    for r0, r1 in runs(bt[:, bi] > 0):
        ax.broken_barh([(tj[r0], max(0.15, tj[r1 - 1] - tj[r0]))], (len(rows) - 0.35, 0.7), color="#4a3aa7")
        ax.annotate(nm, (tj[r0] + 0.6, len(rows) - 0.2), fontsize=7, va="center")
ax.set_yticks(range(len(rows) + 1)); ax.set_yticklabels([lab[k] for k in rows] + ["L1/R1 gripper"], fontsize=7); ax.set_ylim(-0.6, len(rows) + 0.6)
ax.set_title("Pad input (dark = +, pale = - direction); dotted lines = DIRECT on / TELEOP on / TELEOP off / SAFETY", fontsize=9, loc="left")
ax.grid(False)
for i, (nm, sl) in enumerate((("x", 0), ("y", 1))):
    ax = axs[1 + i]
    ax.plot(tt, T[:, TS["com"]][:, sl], color=C_TGT, ls="--", lw=1.0, label="pad target")
    ax.plot(tw, D[:, 45 + sl], color=C_REF, lw=1.2, label="reference x_cd")
    ax.plot(tw, D[:, 48 + sl], color=C_MEAS, lw=1.0, label="measured CoM x_c")
    ax.set_ylabel(f"CoM {nm} [m]"); phases(ax)
    if i == 0:
        ax.legend(loc="upper left", ncol=3, fontsize=8)
ax = axs[3]
for j, (col, nm) in enumerate(((C_REF, "q2"), (C_MEAS, "q3"))):
    ax.plot(tt, np.degrees(T[:, TS["q"]][:, 1 + j]), color=col, ls="--", lw=1.0)
    ax.plot(tw, np.degrees(D[:, 10 + j]), color=col, lw=1.4, alpha=0.5)
    ax.plot(tq, np.degrees(qm[:, 1 + j]), color=col, lw=0.9, label=f"{nm}: target (dashed) / ref (thick) / measured (thin)")
ax.axhline(37, color=C_GRAY, lw=1.0); ax.text(15, 37.4, "pad inner bound q2 = 37 deg", fontsize=7, color=C_TGT)
ax.set_ylabel("joint [deg]"); ax.legend(loc="lower left", fontsize=7); phases(ax)
ax = axs[4]
ex = 1e3 * np.linalg.norm(D[:, 48:51] - D[:, 45:48], axis=1)
ax.plot(tw, ex, color=C_REF, lw=0.8, label="|CoM error| [mm]")
ax.plot(tw, 1e3 * np.linalg.norm(D[:, 24:27], axis=1), color=C_MEAS, lw=0.8, label="|EE task error| [mm]")
ax.plot(tv, tilt * 10, color=C_TGT, lw=0.7, alpha=0.8, label="tilt x10 [deg]")
ax.set_ylim(0, 60); ax.set_ylabel("error"); ax.legend(loc="upper left", ncol=3, fontsize=8); phases(ax)
ax.set_xlabel("time since bag start [s]"); ax.set_xlim(8, 122)
fig.tight_layout(); fig.savefig(f"{OUT}/fig1_overview.png"); plt.close(fig)

# ---------------- Fig 2: D-pad taps zoom
fig, axs = plt.subplots(2, 2, figsize=(11, 5.6), sharey="row")
for col, (p0, p1, sl, nm) in enumerate(((16.5, 32, 0, "x (4 forward taps)"), (51, 63, 0, "x (3 forward + 3 back taps)"))):
    ax = axs[0, col]
    m = (tw > p0) & (tw < p1); mt = (tt > p0) & (tt < p1)
    shade_pad(ax, ["com_fwd"])
    ax.plot(tt[mt], T[mt, 1 + sl], color=C_TGT, ls="--", lw=1.0, label="pad target")
    ax.plot(tw[m], D[m, 45 + sl], color=C_REF, label="reference")
    ax.plot(tw[m], D[m, 48 + sl], color=C_MEAS, lw=1.0, label="measured CoM")
    ax.set_xlim(p0, p1); ax.set_title(f"CoM {nm}", loc="left", fontsize=9); ax.set_ylabel("[m]")
    if col == 0:
        ax.legend(fontsize=8, loc="upper left")
    ax = axs[1, col]
    ax.plot(tw[m], 1e3 * (D[m, 48 + sl] - D[m, 45 + sl]), color=C_MEAS, lw=0.9, label="measured - reference")
    ax.plot(tw[m], 1e3 * (D[m, 45 + sl] - np.interp(tw[m], tt, T[:, 1 + sl])), color=C_REF, lw=0.9, label="reference - target (smoother lag)")
    ax.set_xlim(p0, p1); ax.set_ylabel("[mm]"); ax.set_xlabel("time [s]")
    if col == 0:
        ax.legend(fontsize=8, loc="lower left")
fig.tight_layout(); fig.savefig(f"{OUT}/fig2_dpad_taps.png"); plt.close(fig)

# ---------------- Fig 3: arm channel
fig, axs = plt.subplots(3, 1, figsize=(11, 7.5), sharex=True)
p0, p1 = 60, 100
m = (tw > p0) & (tw < p1); mt = (tt > p0) & (tt < p1); mq = (tq > p0) & (tq < p1)
ax = axs[0]
shade_pad(ax, ["ee_fwd", "ee_up", "home"])
S = 1e3 * T[:, TS["s"]]
ax.plot(tt[mt], S[mt, 0] - S[mt, 0][0], color=C_REF, label="target EE fwd (relative to start)")
ax.plot(tt[mt], S[mt, 2] - S[mt, 2][0], color=C_MEAS, label="target EE up (relative to start)")
wall = (T[:, TS["arm_blocked"]] > 0) & mt
ax.plot(tt[wall], np.zeros(wall.sum()) + 5, "|", color="#e34948", ms=10, label="pad wall: 'joint 2 at inner bound'")
ax.set_ylabel("[mm]"); ax.legend(fontsize=8, loc="lower left"); ax.set_title("EE target relative to the airframe (blue band = L-stick fwd, pink = R-stick down, black = PS)", loc="left", fontsize=9)
for j, (ax, nm) in enumerate(zip(axs[1:], ("q2", "q3"))):
    ax.plot(tt[mt], np.degrees(T[mt, TS["q"]][:, 1 + j]), color=C_TGT, ls="--", label="pad target")
    ax.plot(tw[m], np.degrees(D[m, 10 + j]), color=C_REF, label="law reference")
    ax.plot(tq[mq], np.degrees(qm[mq, 1 + j]), color=C_MEAS, lw=1.0, label="measured")
    if j == 0:
        ax.axhline(37, color=C_GRAY); ax.legend(fontsize=8, loc="lower left")
    ax.set_ylabel(f"{nm} [deg]")
axs[-1].set_xlabel("time [s]")
fig.tight_layout(); fig.savefig(f"{OUT}/fig3_arm.png"); plt.close(fig)

# ---------------- Fig 4: SAFETY revert dip, every 4-D hardware flight
FL = [("x_20260918_144828", "0918"), ("x_20260921_112328", "0921 #1"), ("x_20260921_115207", "0921 #2"),
      ("x_20260924_120546", "0924 F2"), ("x_20260924_165654", "0924 F3"), ("p1", "0929 PS4")]
fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
for nm, lbl in FL:
    dd, tz = load(nm)
    md = edges(dd, tz, "wbmode"); rv = [t for t, v in md if v == "SAFETY" and t > 5][-1]
    to = dd["odom__recv"] - tz; z = dd["odom__pose.pose.position.z"]
    tu = dd["ude__recv"] - tz; U = dd["ude__disturbance_estimate.z"]
    # clip at the operator's landing command (SAFETY position reference leaves the revert altitude by > 5 cm)
    tp = dd["pcstate__recv"] - tz; zr = dd["pcstate__position_reference.z"]
    zr0 = np.interp(rv + 0.2, tp, zr); land = tp[(tp > rv + 0.2) & (np.abs(zr - zr0) > 0.05)]
    tend = min(rv + 8, land[0] if len(land) else rv + 8)
    mo = (to > rv - 2) & (to < tend); mu = (tu > rv - 2) & (tu < tend) & (dd["ude__is_active"] > 0)
    z0 = np.interp(rv, to, z); u0 = np.interp(rv, tu, U)
    hi = nm == "p1"
    kw = dict(color=C_REF if hi else C_GRAY, lw=1.8 if hi else 1.0, zorder=3 if hi else 1)
    axs[0].plot(to[mo] - rv, 1e3 * (z[mo] - z0), **kw)
    axs[1].plot(tu[mu] - rv, U[mu] - u0, **kw)
axs[0].set_title("Altitude change after DIRECT -> SAFETY, up to the landing command\n(6 flights; blue = 0929 PS4)", loc="left", fontsize=9)
axs[0].set_xlabel("time since revert [s]"); axs[0].set_ylabel("z - z(revert) [mm]")
axs[1].set_title("SAFETY UDE vertical estimate change [N]\n(re-learning the SAFETY map's thrust error)", loc="left", fontsize=9)
axs[1].axhline(-2.35, color="#e34948", ls="--", lw=1.0); axs[1].text(-1.9, -2.3, "predicted from the 6.2 % kf mismatch (-2.35 N)", color="#e34948", fontsize=7)
axs[1].set_xlabel("time since revert [s]")
fig.tight_layout(); fig.savefig(f"{OUT}/fig4_revert_dip.png"); plt.close(fig)
print("figures ->", os.path.abspath(OUT))
