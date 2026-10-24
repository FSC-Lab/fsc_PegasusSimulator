#!/usr/bin/env python3
"""fig_bearing.png: (A) the free spin-downs against the friction estimates, (B) the torque they imply
versus the one unknown, the rotor's inertia (CAD radius of gyration, mass on the x axis)."""
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
S = np.load("stopping.npz"); F = np.load("decel_fit.npz")
D, w, sid = S["D"], S["w"], S["sid"]
INK, INK2, GRID = "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE, VIOLET, AQUA = "#2a78d6", "#eb6834", "#6250d6", "#1baf7a"
K2 = 0.003058028 / 0.24278454            # CAD k^2 = I/m [m^2]
I_cad = 0.204 * K2
a_c = np.median(F["B1"]); a_lo, a_hi = np.percentile(F["B1"], [2.5, 97.5])
q = F["B2"][:, 0] + F["B2"][:, 1] * np.radians(24)
s_c = np.median(q); s_lo, s_hi = np.percentile(q, [2.5, 97.5])
plt.rcParams.update({"font.size": 10, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK2, "axes.spines.top": False,
                     "axes.spines.right": False})
fig, ax = plt.subplots(1, 2, figsize=(13, 5.2), facecolor="#fcfcfb")
for a in ax: a.set_facecolor("#fcfcfb"); a.grid(color=GRID, lw=0.6); a.set_axisbelow(True)
# A
ax[0].plot(np.degrees(D), 0.5 * w ** 2, ".", ms=2.5, color="#a9a8a2", label="13 free spin-downs (one outlier, dropped from the fit)")
x = np.radians(np.linspace(0, 640, 50))
for acc, col, lab, yl in [(a_c, BLUE, f"fit: {a_c:.2f} rad/s$^2$ = {a_c*I_cad*1e3:.1f} mN·m", 26.5),
                          (0.038 / I_cad, ORANGE, "10-22 gauge test, 38 mN·m", 21.0),
                          (0.048 / I_cad, VIOLET, "sim default, 48 mN·m", 27.5)]:
    ax[0].plot(np.degrees(x), acc * x, color=col, lw=2)
    ax[0].text(np.degrees(yl / acc) + 12, yl, lab, color=INK, va="center", fontsize=9)
ax[0].set_xlim(0, 900); ax[0].set_ylim(0, 30)
ax[0].set_xlabel("remaining travel to rest [deg]"); ax[0].set_ylabel(r"$\omega^2/2$ [rad$^2$/s$^2$]")
ax[0].set_title("A. Spin-downs vs the estimates (I at the CAD 204 g)", color=INK, loc="left", fontsize=11)
ax[0].legend(loc="lower right", frameon=False, fontsize=9, markerscale=4)
# B
m = np.linspace(0.14, 0.26, 50)
ax[1].fill_between(m * 1e3, m * K2 * a_lo * 1e3, m * K2 * a_hi * 1e3, color=BLUE, alpha=0.25, lw=0)
ax[1].plot(m * 1e3, m * K2 * a_c * 1e3, color=BLUE, lw=2)
ax[1].text(262, 0.26 * K2 * a_c * 1e3 + 2.0, " Coulomb-equivalent\n (all spin speeds)", color=INK, va="center", fontsize=9)
ax[1].fill_between(m * 1e3, m * K2 * s_lo * 1e3, m * K2 * s_hi * 1e3, color=AQUA, alpha=0.22, lw=0)
ax[1].plot(m * 1e3, m * K2 * s_c * 1e3, color=AQUA, lw=2)
ax[1].text(262, 0.26 * K2 * s_c * 1e3 - 2.0, " at the turn's\n <= 24 deg/s", color=INK, va="center", fontsize=9)
for y, col, lab in [(38, ORANGE, "10-22 gauge test 38"), (48, VIOLET, "sim default 48"), (32, INK2, "10-17 fin-pull bound 32")]:
    ax[1].axhline(y, color=col, lw=1.5, ls="-" if col != INK2 else "--")
    ax[1].text(141, y + 0.8, lab, color=INK, fontsize=9)
ax[1].axvline(204, color=INK2, lw=1, ls=":"); ax[1].text(205, 2, "204 g\n(CAD estimate)", color=INK2, fontsize=8.5)
ax[1].axvline(242.8, color=INK2, lw=1, ls=":"); ax[1].text(244, 2, "243 g\nsolid PLA", color=INK2, fontsize=8.5)
ax[1].set_xlim(140, 300); ax[1].set_ylim(0, 52)
ax[1].set_xlabel("rotor mass [g]  (inertia = mass x the CAD k² = (0.112 m)²)"); ax[1].set_ylabel("bearing friction torque [mN·m]")
ax[1].set_title("B. Torque implied, 95% bands (bootstrap over spin-downs)", color=INK, loc="left", fontsize=11)
fig.tight_layout(); fig.savefig("fig_bearing.png", dpi=110, facecolor=fig.get_facecolor())
print(f"a {a_c:.3f} [{a_lo:.3f},{a_hi:.3f}]  slow {s_c:.3f} [{s_lo:.3f},{s_hi:.3f}]  I_cad {I_cad:.5f}")
print(f"torque at 204 g: {a_c*I_cad*1e3:.1f} [{a_lo*I_cad*1e3:.1f},{a_hi*I_cad*1e3:.1f}] / slow {s_c*I_cad*1e3:.1f} [{s_lo*I_cad*1e3:.1f},{s_hi*I_cad*1e3:.1f}] mN.m")
print(f"ceiling (all 243 g at r 0.2001): {a_hi*0.2428*0.2001**2*1e3:.1f} mN.m;  204 g at tip: {a_hi*0.204*0.2001**2*1e3:.1f}")
