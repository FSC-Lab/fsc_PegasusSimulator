#!/usr/bin/env python3
"""fig_damper.png: (A) the 10-27 coast-downs against the friction laws, (B) each coast's mean
resisting torque against its mean speed, beside what the planned damper would have given.
Torque = I x deceleration at the CAD 204 g rotor (I = 0.00257 kg m^2)."""
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
C = np.load("coasts_th_q_25.npz"); F = np.load("fit_th_q_25.npz")
t, th, w, segs, rows = C["t"], C["th"], C["w"], C["segs"], C["rows"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, VIOLET, AQUA = "#2a78d6", "#eb6834", "#6250d6", "#1baf7a"
I = 0.204 * 0.003058028 / 0.24278454
a = np.median(F["BE"][:, 0]); a_lo, a_hi = np.percentile(np.r_[F["BE"][:, 0], F["BO"][:, 0]], [2.5, 97.5])
plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.spines.top": False, "axes.spines.right": False})
fig, ax = plt.subplots(1, 2, figsize=(13, 5.2), facecolor="#fcfcfb")
for x_ in ax: x_.set_facecolor("#fcfcfb"); x_.grid(color=GRID, lw=0.6); x_.set_axisbelow(True)
# A: energy vs remaining travel
for i0, ir, s, ths in segs:
    i0, ir = int(i0), int(ir)
    D = np.clip(s * (ths - th[i0:ir + 1]), 0, None); v = np.clip(s * w[i0:ir + 1], 0, None)
    ax[0].plot(np.degrees(D), 0.5 * v ** 2, "-", lw=0.8, color="#a9a8a2")
ax[0].plot([], [], "-", color="#a9a8a2", lw=0.8, label=f"{len(segs)} free coast-downs (start 25 ms after the peak)")
x = np.radians(np.linspace(0, 80, 50))
for acc, col, lab in [(a, BLUE, f"pad, Coulomb fit: {a:.0f} rad/s² = {a*I*1e3:.0f} mN·m"),
                      (0.048 / I, VIOLET, "sim default, 48 mN·m"),
                      (3.48, ORANGE, "10-23, bearing alone: 9 mN·m")]:
    ax[0].plot(np.degrees(x), acc * x, color=col, lw=2, label=lab)
ax[0].set_xlim(0, 80); ax[0].set_ylim(0, 34)
ax[0].set_xlabel("remaining travel to rest [deg]"); ax[0].set_ylabel(r"$\omega^2/2$ [rad$^2$/s$^2$]")
ax[0].set_title("A. Coast-downs: straight lines = constant (Coulomb) torque", color=INK, loc="left", fontsize=11)
ax[0].legend(loc="upper left", frameon=False, fontsize=9)
# B: per-coast mean torque vs the coast's mean speed. Energy: w0^2/(2D) = a + c * wbar,
# wbar = int w dx / D, so torque = I a_D = tau_c + b * wbar exactly, and a damper is a straight line.
wbar = []
for i0, ir, s, ths in segs:
    i0, ir = int(i0), int(ir)
    x_ = s * (th[i0:ir + 1] - th[i0]); v = np.clip(s * w[i0:ir + 1], 0, None)
    wbar.append(np.trapz(v, x_) / x_[-1])
wbar = np.degrees(np.array(wbar)); tq = rows[:, 6] * I * 1e3
ax[1].fill_between([0, 300], a_lo * I * 1e3, a_hi * I * 1e3, color=BLUE, alpha=0.18, lw=0)
ax[1].plot([0, 300], [a * I * 1e3] * 2, color=BLUE, lw=2, label=f"measured: {a*I*1e3:.0f} mN·m, flat (95 % band)")
ax[1].plot(wbar, tq, "o", ms=6, mfc="#fcfcfb", mec=INK2, mew=1.2, label="one coast-down")
ws = np.linspace(0, 300, 200)
for b, ls in [(0.25, "-"), (1.0, "--")]:
    ax[1].plot(ws, (a * I + b * np.radians(ws)) * 1e3, color=AQUA, lw=2, ls=ls,
               label=f"the planned damper, b = {b:g} N·m·s/rad")
ax[1].axvspan(0, 24, color=INK2, alpha=0.10, lw=0)
ax[1].text(27, 285, "flight turn ≤ 24°/s", color=INK2, fontsize=8.5, va="top")
ax[1].set_xlim(0, 300); ax[1].set_ylim(0, 300)
ax[1].set_xlabel("coast's mean speed over its travel [deg/s]"); ax[1].set_ylabel("mean resisting torque [mN·m]")
ax[1].set_title("B. No speed dependence: a friction brake, not a damper", color=INK, loc="left", fontsize=11)
ax[1].legend(loc="upper right", frameon=False, fontsize=9)
fig.tight_layout(); fig.savefig("fig_damper.png", dpi=110, facecolor=fig.get_facecolor())
print(f"a {a:.2f} [{a_lo:.2f}, {a_hi:.2f}] rad/s^2 -> {a*I*1e3:.1f} [{a_lo*I*1e3:.1f}, {a_hi*I*1e3:.1f}] mN.m at I {I:.5f}")
