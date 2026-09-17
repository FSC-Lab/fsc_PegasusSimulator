#!/usr/bin/env python3
"""Figures for the 0912 whole-body-L1 hardware flight.

Run with the system python3 and PYTHONNOUSERSITE=1 (the user-site numpy 2.x
breaks the apt matplotlib built for numpy 1.x).

    PYTHONNOUSERSITE=1 python3 figures.py analysis.npz outdir/
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

# --- design tokens (dataviz skill reference palette, light surface) ---------
SURF = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
INK3 = "#8a8985"
GRID = "#e3e2dd"
# categorical slots 1-3; this triple is the documented all-pairs-validated set
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
S4 = "#eda100"          # slot 4, lines only (adjacent pairlist)
REF = "#8a8985"         # references are neutral; the measurement carries hue
BAND = "#ecebe6"        # phase shading
CRIT = "#e34948"

plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "font.family": "DejaVu Sans", "font.size": 9,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "axes.titlesize": 9.5,
    "axes.titleweight": "semibold", "axes.titlecolor": INK,
    "xtick.color": INK3, "ytick.color": INK3,
    "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
    "grid.color": GRID, "grid.linewidth": 0.7,
    "legend.frameon": False, "legend.fontsize": 8,
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.3, "lines.solid_capstyle": "round",
})

# planner EXECUTING windows (from whole_body_planner/status)
PHASES = [(26.33, 29.34, "arm transition"),
          (49.23, 54.25, "base +0.5 m"),
          (70.86, 73.87, "go home")]
T0, T1 = 10.49, 86.97


def frame(ax, ylab, first=False, last=False):
    ax.grid(True, axis="both", linewidth=0.7)
    ax.set_axisbelow(True)
    ax.set_xlim(T0, T1)
    ax.set_ylabel(ylab, fontsize=8.5)
    ax.yaxis.set_major_locator(MaxNLocator(4))
    for a, b, _ in PHASES:
        ax.axvspan(a, b, color=BAND, zorder=0, lw=0)
    if last:
        ax.set_xlabel("time from record start  [s]", fontsize=8.5)
    else:
        ax.tick_params(labelbottom=False)


def headroom(ax, frac=0.38):
    """Extra space at the top of an axis so an in-panel legend clears the data."""
    lo, hi = ax.get_ylim()
    ax.set_ylim(lo, lo + (hi - lo) * (1.0 + frac))


def phase_labels(ax):
    for a, b, nm in PHASES:
        ax.text((a + b) / 2, 1.04, nm, transform=ax.get_xaxis_transform(),
                ha="center", va="bottom", fontsize=8, color=INK2)


def save(fig, out, name):
    fig.savefig(os.path.join(out, name), dpi=125, bbox_inches="tight",
                pad_inches=0.16)
    plt.close(fig)
    print("  ", name)



def smooth(x, tt, win=0.25):
    """Moving average over `win` seconds; the 250 Hz traces are unreadable raw."""
    dt = float(np.median(np.diff(tt)))
    n = max(int(round(win / dt)) | 1, 3)
    k = np.ones(n) / n
    pad = n // 2
    xp = np.concatenate([np.full(pad, x[0]), x, np.full(pad, x[-1])])
    return np.convolve(xp, k, mode="valid")[:len(x)]


def dual(ax, tt, y, color, label=None, win=0.25, lw=1.5):
    """Raw at low alpha with the moving average on top, so both the band and
    the trend are visible in one axis."""
    ax.plot(tt, y, color=color, lw=0.7, alpha=0.22, zorder=2)
    ax.plot(tt, smooth(y, tt, win), color=color, lw=lw, label=label, zorder=3)


def rms(x):
    """Root-mean-square, NaN-safe."""
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.nanmean(x ** 2)))


def fmt_num(v):
    """Adaptive precision so an rms reads naturally at any magnitude."""
    av = abs(v)
    if av >= 100:
        return f"{v:.0f}"
    if av >= 10:
        return f"{v:.1f}"
    if av >= 1:
        return f"{v:.2f}"
    if av >= 0.01:
        return f"{v:.3f}"
    return f"{v:.4f}"


# The law<->vehicle frame relation, from the fork's frame_adapter.hpp:
#   R0_model = R0_actual * R_MODEL,  R_MODEL columns = the MODEL axes in the
#   ACTUAL frame (model +y = actual +x, where the arm is).
R_MODEL = np.array([[0., 1., 0.],
                    [-1., 0., 0.],
                    [0., 0., 1.]])


def quat_to_R(qw, qx, qy, qz):
    n = np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    R = np.empty((len(qw), 3, 3))
    R[:, 0, 0] = 1 - 2 * (qy**2 + qz**2)
    R[:, 0, 1] = 2 * (qx * qy - qw * qz)
    R[:, 0, 2] = 2 * (qx * qz + qw * qy)
    R[:, 1, 0] = 2 * (qx * qy + qw * qz)
    R[:, 1, 1] = 1 - 2 * (qx**2 + qz**2)
    R[:, 1, 2] = 2 * (qy * qz - qw * qx)
    R[:, 2, 0] = 2 * (qx * qz - qw * qy)
    R[:, 2, 1] = 2 * (qy * qz + qw * qx)
    R[:, 2, 2] = 1 - 2 * (qx**2 + qy**2)
    return R


def euler_zyx(R):
    """roll, pitch, yaw [deg] from an ENU<-FLU rotation matrix stack."""
    roll = np.degrees(np.arctan2(R[:, 2, 1], R[:, 2, 2]))
    pitch = np.degrees(np.arcsin(np.clip(-R[:, 2, 0], -1, 1)))
    yaw = np.degrees(np.arctan2(R[:, 1, 0], R[:, 0, 0]))
    return roll, pitch, yaw


def desired_attitude(R_actual, e_R):
    """Recover the law's own commanded attitude from the logged geometric error.

    e_R = 1/2 vee(R0c^T R0 - R0^T R0c) = sin(theta) * axis for the error
    rotation A = R0c^T R0, so the inversion is exact for |e_R| <= 1 (this
    flight peaks at 0.17).  Returns the desired rotation in the ACTUAL frame.
    Round-trip verified against the logged e_R to 6e-17.
    """
    R0 = R_actual @ R_MODEL
    nrm = np.linalg.norm(e_R, axis=1)
    th = np.arcsin(np.clip(nrm, 0, 1))
    axis = np.where(nrm[:, None] > 1e-12,
                    e_R / np.maximum(nrm, 1e-12)[:, None],
                    np.array([1., 0., 0.]))
    K = np.zeros((len(th), 3, 3))
    K[:, 0, 1] = -axis[:, 2]; K[:, 0, 2] = axis[:, 1]
    K[:, 1, 0] = axis[:, 2];  K[:, 1, 2] = -axis[:, 0]
    K[:, 2, 0] = -axis[:, 1]; K[:, 2, 1] = axis[:, 0]
    I = np.eye(3)[None, :, :]
    A = (I + np.sin(th)[:, None, None] * K
         + (1 - np.cos(th))[:, None, None] * (K @ K))
    R0c = R0 @ np.transpose(A, (0, 2, 1))
    return R0c @ R_MODEL.T


def rms_label(lab, x, suffix=""):
    """A legend label carrying that series' rms, e.g. 'x  rms 73 mm' — or,
    with an empty lab (a lone error trace with no other series to name),
    just 'rms 73 mm'."""
    val = f"rms {fmt_num(rms(x))}{suffix}"
    return val if not lab else f"{lab}  {val}"


# ---------------------------------------------------------------------------
def main(src, out):
    os.makedirs(out, exist_ok=True)
    d = np.load(src, allow_pickle=True)
    t = d["t"]; w = d["dbg"]; D = d["direct"]
    td = t[D]; wd = w[D]
    tr = d["t_ref"]

    def S(k):
        return np.column_stack([np.interp(td, tr, d[k][:, j]) for j in range(3)])

    # ============================ 1. CoM tracking ==========================
    xcd = wd[:, 45:48]; xc = wd[:, 48:51]; e = xc - xcd
    fig, ax = plt.subplots(4, 1, figsize=(10, 7.4), sharex=True,
                           gridspec_kw=dict(hspace=0.22, height_ratios=[1, 1, 1, 1.25]))
    for i, (lab, col) in enumerate(zip(["model x", "model y", "model z"], [S1, S2, S3])):
        ax[i].plot(td, xcd[:, i], color=REF, lw=1.5, ls=(0, (4, 2)), label="reference")
        ax[i].plot(td, xc[:, i], color=col, lw=1.3, label="measured")
        frame(ax[i], f"{lab}  [m]")
        ax[i].legend(loc="upper left", ncol=2, handlelength=1.8)
    for i, (lab, col) in enumerate(zip(["x", "y", "z"], [S1, S2, S3])):
        ax[3].plot(td, e[:, i] * 1e3, color=col, lw=1.2,
                  label=rms_label(lab, e[:, i] * 1e3, " mm"))
    ax[3].axhline(0, color=INK3, lw=0.8)
    frame(ax[3], "error  [mm]", last=True)
    ax[3].legend(loc="upper left", ncol=3, handlelength=1.8)
    phase_labels(ax[0])
    ax[0].set_title("System centre of mass — reference, measurement and error  (model frame)",
                    loc="left", pad=22)
    save(fig, out, "f1_com_tracking.png")

    # ============================ 2. EE tracking ===========================
    red = S("ref_r_ed")                       # planned EE, world
    eff = red + e                             # CoM-anchored reference the law flew
    ey = wd[:, 24:28]
    act = eff + ey[:, :3]                     # reconstructed actual EE
    fig, ax = plt.subplots(5, 1, figsize=(10, 8.6), sharex=True,
                           gridspec_kw=dict(hspace=0.22,
                                            height_ratios=[1, 1, 1, 1.25, 1.0]))
    for i, (lab, col) in enumerate(zip(["world x", "world y", "world z"], [S1, S2, S3])):
        ax[i].plot(td, red[:, i], color=REF, lw=1.5, ls=(0, (4, 2)), label="planned reference")
        ax[i].plot(td, act[:, i], color=col, lw=1.3, label="actual")
        frame(ax[i], f"{lab}  [m]")
        ax[i].legend(loc="upper left", ncol=2, handlelength=1.8)
    for i, (lab, col) in enumerate(zip(["x", "y", "z"], [S1, S2, S3])):
        ax[3].plot(td, ey[:, i] * 1e3, color=col, lw=1.2,
                  label=rms_label(lab, ey[:, i] * 1e3, " mm"))
    ax[3].axhline(0, color=INK3, lw=0.8)
    frame(ax[3], "impedance error $e_y$  [mm]")
    ax[3].legend(loc="upper left", ncol=3, handlelength=1.8, bbox_to_anchor=(0, 1.32))
    ax[4].plot(td, ey[:, 3], color=S1, lw=1.2, label=rms_label("", ey[:, 3]))
    ax[4].axhline(0, color=INK3, lw=0.8)
    frame(ax[4], "heading error  [-]", last=True)
    ax[4].legend(loc="upper left", ncol=1, handlelength=1.8, bbox_to_anchor=(0, 1.28))
    phase_labels(ax[0])
    ax[0].set_title("End effector — planned reference, actual position, and the impedance "
                    "error the law regulates", loc="left", pad=22)
    save(fig, out, "f2_ee_tracking.png")

    # ============================ 3. joints ================================
    q = np.degrees(wd[:, 5:9]); qr = np.degrees(wd[:, 9:13])
    fig, ax = plt.subplots(4, 2, figsize=(10, 7.6), sharex=True,
                           gridspec_kw=dict(hspace=0.22, wspace=0.19))
    for j in range(4):
        a = ax[j, 0]
        a.plot(td, qr[:, j], color=REF, lw=1.5, ls=(0, (4, 2)), label="reference")
        a.plot(td, q[:, j], color=S1, lw=1.3, label="measured")
        frame(a, f"joint {j+1}  [deg]", last=(j == 3))
        if j == 0:
            a.legend(loc="upper left", ncol=2, handlelength=1.8)
        b = ax[j, 1]
        err = q[:, j] - qr[:, j]
        b.plot(td, err, color=S2, lw=1.2, label=rms_label("", err, "\u00b0"))
        b.axhline(0, color=INK3, lw=0.8)
        frame(b, "error  [deg]", last=(j == 3))
        b.legend(loc="upper left", ncol=1, handlelength=1.8)
    phase_labels(ax[0, 0]); phase_labels(ax[0, 1])
    ax[0, 0].set_title("Arm joints — reference against measurement", loc="left", pad=22)
    ax[0, 1].set_title("Joint tracking error", loc="left", pad=22)
    save(fig, out, "f3_joint_tracking.png")

    # ============================ 4. attitude ==============================
    to = d["t_odom"]; od = d["odom"]
    mo = (to >= T0) & (to <= T1)
    tom = to[mo]; odm = od[mo]
    R_act = quat_to_R(odm[:, 3], odm[:, 4], odm[:, 5], odm[:, 6])
    eR = np.column_stack([np.interp(tom, td, wd[:, 28 + i]) for i in range(3)])
    R_des = desired_attitude(R_act, eR)
    meas = euler_zyx(R_act)
    des = euler_zyx(R_des)

    fig, ax = plt.subplots(3, 2, figsize=(10, 6.2), sharex=True,
                           gridspec_kw=dict(hspace=0.22, wspace=0.19))
    for k, (nm, col) in enumerate(zip(["roll", "pitch", "yaw"], [S1, S2, S3])):
        a = ax[k, 0]
        dual(a, tom, des[k], REF, "commanded", lw=1.6)
        dual(a, tom, meas[k], col, "measured", lw=1.4)
        frame(a, f"{nm}  [deg]", last=(k == 2))
        headroom(a)
        if k == 0:
            a.legend(loc="upper left", ncol=2, handlelength=1.8)
        err = (meas[k] - des[k] + 180) % 360 - 180
        b = ax[k, 1]
        dual(b, tom, err, S2, rms_label("", err, "\u00b0"), lw=1.4)
        b.axhline(0, color=INK3, lw=0.9)
        frame(b, "error  [deg]", last=(k == 2))
        headroom(b)
        b.legend(loc="upper left", ncol=1, handlelength=1.8)
    phase_labels(ax[0, 0]); phase_labels(ax[0, 1])
    ax[0, 0].set_title("Attitude — commanded against measured", loc="left", pad=22)
    ax[0, 1].set_title("Attitude tracking error", loc="left", pad=22)
    save(fig, out, "f4_attitude.png")

    # ============================ 5. body inputs ===========================
    MG = 3.746170 * 9.80665
    fig, ax = plt.subplots(3, 1, figsize=(10, 6.2), sharex=True,
                           gridspec_kw=dict(hspace=0.22))
    dual(ax[0], td, wd[:, 17], S1, "$u_1$ commanded")
    ax[0].axhline(MG, color=REF, lw=1.4, ls=(0, (4, 2)), label="model weight $mg$")
    frame(ax[0], "collective  [N]")
    ax[0].legend(loc="upper left", ncol=2, handlelength=1.8)
    for i, (lab, col) in enumerate(zip(["roll", "pitch", "yaw"], [S1, S2, S3])):
        dual(ax[1], td, wd[:, 21 + i], col, lab)
    ax[1].axhline(0, color=INK3, lw=0.8)
    frame(ax[1], "body torque  [N m]")
    ax[1].legend(loc="upper left", ncol=3, handlelength=1.8)
    for i, col in enumerate([S1, S2, S3, S4]):
        dual(ax[2], td, wd[:, 41 + i], col, f"M{i+1}")
    frame(ax[2], "motor command  [-]", last=True)
    ax[2].legend(loc="upper left", ncol=4, handlelength=1.8)
    ax[2].set_ylim(0.2, 0.95)
    phase_labels(ax[0])
    ax[0].set_title("Body control inputs — collective, torque (FLU) and the four motor commands",
                    loc="left", pad=22)
    save(fig, out, "f5_body_inputs.png")

    # ============================ 6. arm torque ============================
    tjs = d["t_js"]; ta = d["tau_applied"]
    tc = d["t_cmd"]; cm = d["tau_cmd"]
    m = (tc >= T0) & (tc <= T1)
    fig, ax = plt.subplots(4, 2, figsize=(10, 7.6), sharex=True,
                           gridspec_kw=dict(hspace=0.22, wspace=0.19))
    for j in range(4):
        app = np.interp(tc[m], tjs, ta[:, j])
        a = ax[j, 0]
        dual(a, tc[m], cm[m, j], REF, "commanded", lw=1.6)
        dual(a, tc[m], app, S1, "applied (measured)", lw=1.4)
        frame(a, f"joint {j+1}  [N m]", last=(j == 3))
        if j == 0:
            a.legend(loc="upper left", ncol=2, handlelength=1.8)
        b = ax[j, 1]
        err = app - cm[m, j]
        dual(b, tc[m], err, S2, rms_label("", err, " N\u00b7m"), lw=1.4)
        b.axhline(0, color=INK3, lw=0.9)
        frame(b, "error  [N m]", last=(j == 3))
        b.legend(loc="upper left", ncol=1, handlelength=1.8)
    phase_labels(ax[0, 0]); phase_labels(ax[0, 1])
    ax[0, 0].set_title("Arm torque — commanded against measured applied", loc="left", pad=22)
    ax[0, 1].set_title("Applied minus commanded", loc="left", pad=22)
    save(fig, out, "f6_arm_torque.png")

    # =================== 7. disturbance A: internal ========================
    fig, ax = plt.subplots(3, 1, figsize=(10, 6.2), sharex=True,
                           gridspec_kw=dict(hspace=0.22))
    for i, (lab, col) in enumerate(zip(["x", "y", "z"], [S1, S2, S3])):
        dual(ax[0], td, wd[:, 78 + i], col, lab)
    ax[0].axhline(0, color=INK3, lw=0.8)
    frame(ax[0], "force  [N]")
    ax[0].legend(loc="upper left", ncol=3, handlelength=1.8)
    ax[0].annotate("x and y are identically zero: the attribution's weighted\n"
                   "least-norm solution leaves the lateral rows unreachable",
                   xy=(0.985, 0.90), xycoords="axes fraction", ha="right",
                   va="top", fontsize=8, color=INK2)
    for i, (lab, col) in enumerate(zip(["x", "y", "z"], [S1, S2, S3])):
        dual(ax[1], td, wd[:, 81 + i], col, lab)
    ax[1].axhline(0, color=INK3, lw=0.8)
    frame(ax[1], "moment  [N m]")
    ax[1].legend(loc="upper left", ncol=3, handlelength=1.8)
    for j, col in enumerate([S1, S2, S3, S4]):
        dual(ax[2], td, wd[:, 84 + j], col, f"j{j+1}")
    ax[2].axhline(0, color=INK3, lw=0.8)
    frame(ax[2], "joint  [N m]", last=True)
    ax[2].legend(loc="upper left", ncol=4, handlelength=1.8)
    phase_labels(ax[0])
    ax[0].set_title(r"Disturbance, part 1 — the internal estimate $\hat{w}$, everything "
                    "except end-effector interaction", loc="left", pad=22)
    save(fig, out, "f7_disturbance_internal.png")

    # =================== 8. disturbance B: EE interaction ==================
    fig, ax = plt.subplots(3, 1, figsize=(10, 6.2), sharex=True,
                           gridspec_kw=dict(hspace=0.22))
    for i, (lab, col) in enumerate(zip(["x", "y", "z"], [S1, S2, S3])):
        dual(ax[0], td, wd[:, 72 + i], col, lab)
    ax[0].axhline(0, color=INK3, lw=1.2, ls=(0, (4, 2)))
    frame(ax[0], "EE force  [N]")
    ax[0].legend(loc="upper left", ncol=3, handlelength=1.8)
    for i, (lab, col) in enumerate(zip(["x", "y", "z"], [S1, S2, S3])):
        dual(ax[1], td, wd[:, 75 + i], col, lab)
    ax[1].axhline(0, color=INK3, lw=1.2, ls=(0, (4, 2)))
    frame(ax[1], "EE moment  [N m]")
    ax[1].legend(loc="upper left", ncol=3, handlelength=1.8)
    for i, (lab, col) in enumerate(zip(["x", "y", "z", r"$\psi$"], [S1, S2, S3, S4])):
        dual(ax[2], td, wd[:, 58 + i], col, lab)
    ax[2].axhline(0, color=INK3, lw=0.8)
    frame(ax[2], r"$\hat{F}_y$ into $u_3$  [N]", last=True)
    ax[2].legend(loc="upper left", ncol=4, handlelength=1.8)
    phase_labels(ax[0])
    ax[0].set_title(r"Disturbance, part 2 — the assumed end-effector interaction $\hat{w}_e$. "
                    "Nothing touched the arm: the true value is the dashed zero line",
                    loc="left", pad=22)
    save(fig, out, "f8_disturbance_ee.png")


if __name__ == "__main__":
    print("figures ->", sys.argv[2])
    main(sys.argv[1], sys.argv[2])
