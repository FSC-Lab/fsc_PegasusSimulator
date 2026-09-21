#!/usr/bin/env python3
"""Figures for the 0918 whole-body-L1-4D hardware flight.

    PYTHONNOUSERSITE=1 /usr/bin/python3 figures.py flight_0918.npz figures/

(the user-site numpy 2.x breaks the apt matplotlib built for numpy 1.x).
Style and palette follow the 0912 report so the two hardware reports read as one series.
"""
import sys, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

SURF = "#fcfcfb"; INK = "#0b0b0b"; INK2 = "#52514e"; INK3 = "#8a8985"; GRID = "#e3e2dd"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"; S4 = "#eda100"; REF = "#8a8985"; BAND = "#ecebe6"; CRIT = "#e34948"
plt.rcParams.update({
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "font.family": "DejaVu Sans", "font.size": 9,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "axes.titlesize": 9.5,
    "axes.titleweight": "semibold", "axes.titlecolor": INK,
    "xtick.color": INK3, "ytick.color": INK3, "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
    "grid.color": GRID, "grid.linewidth": 0.7, "legend.frameon": False, "legend.fontsize": 8,
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.3, "lines.solid_capstyle": "round",
})
PHASES = [(27.97, 30.98, "arm out"), (39.70, 42.71, "arm home"), (52.52, 58.16, "base +0.61 m"),
          (71.90, 74.91, "arm out"), (85.99, 89.00, "arm home")]
BLIND = (44.65, 46.87)          # mocap NO DATA (broadcast_position_velocity log)
T0, T1 = 9.66, 92.65
MG = 3.746170 * 9.80665

def frame(ax, ylab, last=False):
    ax.grid(True, axis="both", linewidth=0.7); ax.set_axisbelow(True); ax.set_xlim(T0, T1)
    ax.set_ylabel(ylab, fontsize=8.5); ax.yaxis.set_major_locator(MaxNLocator(4))
    for a, b, _ in PHASES: ax.axvspan(a, b, color=BAND, zorder=0, lw=0)
    ax.axvspan(*BLIND, color=CRIT, alpha=0.10, zorder=0, lw=0)
    if last: ax.set_xlabel("time from record start  [s]", fontsize=8.5)
    else: ax.tick_params(labelbottom=False)

def headroom(ax, frac=0.38):
    lo, hi = ax.get_ylim(); ax.set_ylim(lo, lo + (hi - lo) * (1.0 + frac))

def phase_labels(ax):
    for a, b, nm in PHASES:
        ax.text((a + b) / 2, 1.04, nm, transform=ax.get_xaxis_transform(), ha="center", va="bottom", fontsize=8, color=INK2)
    ax.text(sum(BLIND) / 2, 1.13, "mocap lost", transform=ax.get_xaxis_transform(), ha="center", va="bottom", fontsize=8, color=CRIT)

def save(fig, out, name):
    fig.savefig(os.path.join(out, name), dpi=125, bbox_inches="tight", pad_inches=0.16); plt.close(fig); print("  ", name)

def smooth(x, tt, win=0.25):
    dt = float(np.median(np.diff(tt))); n = max(int(round(win / dt)) | 1, 3); k = np.ones(n) / n; pad = n // 2
    xp = np.concatenate([np.full(pad, x[0]), x, np.full(pad, x[-1])]); return np.convolve(xp, k, mode="valid")[:len(x)]

def dual(ax, tt, y, color, label=None, win=0.25, lw=1.5):
    ax.plot(tt, y, color=color, lw=0.7, alpha=0.22, zorder=2); ax.plot(tt, smooth(y, tt, win), color=color, lw=lw, label=label, zorder=3)

def rms(x): x = np.asarray(x, float); return float(np.sqrt(np.nanmean(x ** 2)))
def fmt_num(v):
    av = abs(v)
    return f"{v:.0f}" if av >= 100 else f"{v:.1f}" if av >= 10 else f"{v:.2f}" if av >= 1 else f"{v:.3f}" if av >= 0.01 else f"{v:.4f}"
def rms_label(lab, x, suffix=""):
    val = f"rms {fmt_num(rms(x))}{suffix}"; return val if not lab else f"{lab}  {val}"

R_MODEL = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
def quat_to_R(qw, qx, qy, qz):
    n = np.sqrt(qw*qw+qx*qx+qy*qy+qz*qz); qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    R = np.empty((len(qw), 3, 3))
    R[:,0,0]=1-2*(qy**2+qz**2); R[:,0,1]=2*(qx*qy-qw*qz); R[:,0,2]=2*(qx*qz+qw*qy)
    R[:,1,0]=2*(qx*qy+qw*qz); R[:,1,1]=1-2*(qx**2+qz**2); R[:,1,2]=2*(qy*qz-qw*qx)
    R[:,2,0]=2*(qx*qz-qw*qy); R[:,2,1]=2*(qy*qz+qw*qx); R[:,2,2]=1-2*(qx**2+qy**2)
    return R
def euler_zyx(R):
    return (np.degrees(np.arctan2(R[:,2,1], R[:,2,2])), np.degrees(np.arcsin(np.clip(-R[:,2,0],-1,1))), np.degrees(np.arctan2(R[:,1,0], R[:,0,0])))
def desired_attitude(R_actual, e_R):
    R0 = R_actual @ R_MODEL; nrm = np.linalg.norm(e_R, axis=1); th = np.arcsin(np.clip(nrm, 0, 1))
    axis = np.where(nrm[:,None] > 1e-12, e_R/np.maximum(nrm,1e-12)[:,None], np.array([1.,0.,0.]))
    K = np.zeros((len(th),3,3)); K[:,0,1]=-axis[:,2]; K[:,0,2]=axis[:,1]; K[:,1,0]=axis[:,2]; K[:,1,2]=-axis[:,0]; K[:,2,0]=-axis[:,1]; K[:,2,1]=axis[:,0]
    I = np.eye(3)[None]; A = I + np.sin(th)[:,None,None]*K + (1-np.cos(th))[:,None,None]*(K@K)
    return (R0 @ np.transpose(A,(0,2,1))) @ R_MODEL.T
def lp(x, tt, wc):
    y = np.zeros_like(x)
    for i in range(1, len(x)):
        al = 1 - np.exp(-wc*(tt[i]-tt[i-1])); y[i] = y[i-1] + al*(x[i]-y[i-1])
    return y

def main(src, out):
    os.makedirs(out, exist_ok=True)
    d = np.load(src)
    t = d["t"]; w = d["dbg"]; D = d["direct"]; td = t[D]; wd = w[D]; tr = d["t_ref"]
    def S(k): return np.column_stack([np.interp(td, tr, d[k][:, j]) for j in range(3)])

    # 1. CoM
    xcd = wd[:,45:48]; xc = wd[:,48:51]; e = xc - xcd
    fig, ax = plt.subplots(4,1,figsize=(10,7.4),sharex=True,gridspec_kw=dict(hspace=0.22,height_ratios=[1,1,1,1.25]))
    for i,(lab,col) in enumerate(zip(["model x (lateral)","model y (along arm)","model z"],[S1,S2,S3])):
        ax[i].plot(td, xcd[:,i], color=REF, lw=1.5, ls=(0,(4,2)), label="reference"); ax[i].plot(td, xc[:,i], color=col, lw=1.3, label="measured")
        frame(ax[i], f"{lab}  [m]"); ax[i].legend(loc="upper left", ncol=2, handlelength=1.8)
    for i,(lab,col) in enumerate(zip(["x","y","z"],[S1,S2,S3])):
        ax[3].plot(td, e[:,i]*1e3, color=col, lw=1.2, label=rms_label(lab, e[:,i]*1e3, " mm"))
    ax[3].axhline(0, color=INK3, lw=0.8); frame(ax[3], "error  [mm]", last=True); ax[3].legend(loc="upper left", ncol=3, handlelength=1.8)
    phase_labels(ax[0]); ax[0].set_title("System centre of mass — reference, measurement and error  (model frame)", loc="left", pad=30)
    save(fig, out, "f1_com_tracking.png")

    # 2. EE
    red = S("ref_r_ed"); eff = red + e; ey = wd[:,24:28]; act = eff + ey[:,:3]
    fig, ax = plt.subplots(5,1,figsize=(10,8.6),sharex=True,gridspec_kw=dict(hspace=0.22,height_ratios=[1,1,1,1.25,1.0]))
    for i,(lab,col) in enumerate(zip(["world x","world y","world z"],[S1,S2,S3])):
        ax[i].plot(td, red[:,i], color=REF, lw=1.5, ls=(0,(4,2)), label="planned reference"); ax[i].plot(td, act[:,i], color=col, lw=1.3, label="actual")
        frame(ax[i], f"{lab}  [m]"); ax[i].legend(loc="upper left", ncol=2, handlelength=1.8)
    for i,(lab,col) in enumerate(zip(["x","y","z"],[S1,S2,S3])):
        ax[3].plot(td, ey[:,i]*1e3, color=col, lw=1.2, label=rms_label(lab, ey[:,i]*1e3, " mm"))
    ax[3].axhline(0, color=INK3, lw=0.8); frame(ax[3], "impedance error $e_y$  [mm]"); ax[3].legend(loc="upper left", ncol=3, handlelength=1.8, bbox_to_anchor=(0,1.32))
    hd = np.degrees(np.arcsin(np.clip(ey[:,3],-1,1)))
    ax[4].plot(td, hd, color=S1, lw=1.2, label=rms_label("", hd, "°")); ax[4].axhline(0, color=INK3, lw=0.8)
    frame(ax[4], "heading error  [deg]", last=True); ax[4].legend(loc="upper left", ncol=1, handlelength=1.8, bbox_to_anchor=(0,1.28))
    phase_labels(ax[0]); ax[0].set_title("End effector — planned reference, actual position, and the impedance error the law regulates (CoM-anchored)", loc="left", pad=30)
    save(fig, out, "f2_ee_tracking.png")

    # 3. joints
    q = np.degrees(wd[:,5:9]); qr = np.degrees(wd[:,9:13])
    fig, ax = plt.subplots(4,2,figsize=(10,7.6),sharex=True,gridspec_kw=dict(hspace=0.22,wspace=0.19))
    for j in range(4):
        a = ax[j,0]; a.plot(td, qr[:,j], color=REF, lw=1.5, ls=(0,(4,2)), label="reference"); a.plot(td, q[:,j], color=S1, lw=1.3, label="measured")
        frame(a, f"joint {j+1}  [deg]", last=(j==3))
        if j==0: a.legend(loc="upper left", ncol=2, handlelength=1.8)
        b = ax[j,1]; err = q[:,j]-qr[:,j]; b.plot(td, err, color=S2, lw=1.2, label=rms_label("", err, "°")); b.axhline(0, color=INK3, lw=0.8)
        frame(b, "error  [deg]", last=(j==3)); b.legend(loc="upper left", ncol=1, handlelength=1.8)
    phase_labels(ax[0,0]); phase_labels(ax[0,1])
    ax[0,0].set_title("Arm joints — reference vs measured", loc="left", pad=30); ax[0,1].set_title("Joint error", loc="left", pad=30)
    save(fig, out, "f3_joint_tracking.png")

    # 4. attitude
    to = d["t_odom"]; od_q = d["odom_quat"]; mo = (to>=T0)&(to<=T1); tom = to[mo]; qq = od_q[mo]
    R_act = quat_to_R(qq[:,0],qq[:,1],qq[:,2],qq[:,3])
    eR = np.column_stack([np.interp(tom, td, wd[:,28+i]) for i in range(3)])
    R_des = desired_attitude(R_act, eR); meas = euler_zyx(R_act); des = euler_zyx(R_des)
    fig, ax = plt.subplots(3,2,figsize=(10,6.2),sharex=True,gridspec_kw=dict(hspace=0.22,wspace=0.19))
    for k,(nm,col) in enumerate(zip(["roll","pitch","yaw"],[S1,S2,S3])):
        a = ax[k,0]; dual(a, tom, des[k], REF, "commanded", lw=1.6); dual(a, tom, meas[k], col, "measured", lw=1.4)
        frame(a, f"{nm}  [deg]", last=(k==2)); headroom(a)
        if k==0: a.legend(loc="upper left", ncol=2, handlelength=1.8)
        err = (meas[k]-des[k]+180)%360-180; b = ax[k,1]; dual(b, tom, err, S2, rms_label("", err, "°"), lw=1.4); b.axhline(0, color=INK3, lw=0.9)
        frame(b, "error  [deg]", last=(k==2)); headroom(b); b.legend(loc="upper left", ncol=1, handlelength=1.8)
    ax[0,1].set_ylim(-8, 8); ax[1,1].set_ylim(-8, 8); ax[2,1].set_ylim(-8, 8)
    phase_labels(ax[0,0]); phase_labels(ax[0,1])
    ax[0,0].set_title("Attitude — commanded vs measured", loc="left", pad=30); ax[0,1].set_title("Attitude error (clipped ±8°)", loc="left", pad=30)
    save(fig, out, "f4_attitude.png")

    # 5. body inputs
    fig, ax = plt.subplots(3,1,figsize=(10,6.2),sharex=True,gridspec_kw=dict(hspace=0.22))
    dual(ax[0], td, wd[:,17], S1, "$u_1$ commanded"); ax[0].axhline(MG, color=REF, lw=1.4, ls=(0,(4,2)), label="model weight $mg$ = 36.74 N")
    frame(ax[0], "collective  [N]"); ax[0].set_ylim(30, 42); ax[0].legend(loc="upper left", ncol=2, handlelength=1.8)
    for i,(lab,col) in enumerate(zip(["roll","pitch","yaw"],[S1,S2,S3])): dual(ax[1], td, wd[:,21+i], col, lab)
    ax[1].axhline(0, color=INK3, lw=0.8); frame(ax[1], "body torque (FLU)  [N m]"); ax[1].legend(loc="upper left", ncol=3, handlelength=1.8)
    for i,col in enumerate([S1,S2,S3,S4]): dual(ax[2], td, wd[:,41+i], col, f"M{i+1}")
    frame(ax[2], "motor command  [-]", last=True); ax[2].legend(loc="upper left", ncol=4, handlelength=1.8); ax[2].set_ylim(0.35, 0.9)
    phase_labels(ax[0]); ax[0].set_title("Body control inputs — collective, torque and the four motor commands", loc="left", pad=30)
    save(fig, out, "f5_body_inputs.png")

    # 6. arm torque: streamed command, duty-implied (with the arm-side corrections), applied
    tl = d["t_law"]; law = d["law"]; tjs = d["t_js"]; ta = d["tau_applied"]
    m = (tl>=T0)&(tl<=T1); tlm = tl[m]; ext = law[m,9:13]; duty = d["tau_duty_nm"][m]
    fig, ax = plt.subplots(4,2,figsize=(10,7.6),sharex=True,gridspec_kw=dict(hspace=0.22,wspace=0.19))
    for j in range(4):
        app = np.interp(tlm, tjs, ta[:,j]); a = ax[j,0]
        dual(a, tlm, ext[:,j], REF, "streamed by the law", lw=1.6); dual(a, tlm, duty[:,j], S3, "written duty (with corrections)", lw=1.2); dual(a, tlm, app, S1, "applied (Kt·I measured)", lw=1.4)
        frame(a, f"joint {j+1}  [N m]", last=(j==3))
        if j==0: a.legend(loc="upper left", ncol=1, handlelength=1.6, fontsize=7.5)
        b = ax[j,1]; err = app - duty[:,j]; dual(b, tlm, err, S2, rms_label("", err, " N·m"), lw=1.4); b.axhline(0, color=INK3, lw=0.9)
        frame(b, "error  [N m]", last=(j==3)); b.legend(loc="upper left", ncol=1, handlelength=1.8)
    phase_labels(ax[0,0]); phase_labels(ax[0,1])
    ax[0,0].set_title("Arm torque — streamed, written, applied", loc="left", pad=30); ax[0,1].set_title("Applied minus written", loc="left", pad=30)
    save(fig, out, "f6_arm_torque.png")

    # 7. disturbance estimate (filtered) + deadbeat
    fig, ax = plt.subplots(3,1,figsize=(10,6.2),sharex=True,gridspec_kw=dict(hspace=0.22))
    ax[0].plot(td, wd[:,64], color=S3, lw=0.5, alpha=0.18, label="z, deadbeat (unfiltered)")
    for i,(lab,col) in enumerate(zip(["x","y","z"],[S1,S2,S3])): ax[0].plot(td, wd[:,31+i], color=col, lw=1.4, label=lab+" filtered")
    ax[0].axhline(0, color=INK3, lw=0.8); frame(ax[0], "force  [N]"); ax[0].set_ylim(-4, 4); ax[0].legend(loc="upper left", ncol=4, handlelength=1.8)
    for i,(lab,col) in enumerate(zip(["x","y","z"],[S1,S2,S3])): ax[1].plot(td, wd[:,34+i], color=col, lw=1.4, label=lab)
    ax[1].axhline(0, color=INK3, lw=0.8); frame(ax[1], "moment  [N m]"); ax[1].legend(loc="upper left", ncol=3, handlelength=1.8)
    for j,col in enumerate([S1,S2,S3,S4]): ax[2].plot(td, wd[:,37+j], color=col, lw=1.4, label=f"j{j+1}")
    ax[2].axhline(0, color=INK3, lw=0.8); frame(ax[2], "joint  [N m]", last=True); ax[2].legend(loc="upper left", ncol=4, handlelength=1.8)
    phase_labels(ax[0]); ax[0].set_title(r"Disturbance estimate $\hat{d}$ the loops see (filtered), with the unfiltered deadbeat z reading behind it", loc="left", pad=30)
    save(fig, out, "f7_disturbance.png")

    # 8. 4-D attribution
    Fr = wd[:,97:101]; Frf = np.column_stack([lp(Fr[:,i], td, 0.25) for i in range(3)]); Fy = wd[:,58:62]; wq = wd[:,101:105]
    fig, ax = plt.subplots(3,1,figsize=(10,6.2),sharex=True,gridspec_kw=dict(hspace=0.22))
    for i,(lab,col) in enumerate(zip(["x","y","z"],[S1,S2,S3])):
        ax[0].plot(td, Fr[:,i], color=col, lw=0.5, alpha=0.18); ax[0].plot(td, Frf[:,i], color=col, lw=1.5, label=lab+"  (0.25 rad/s filtered)")
    ax[0].axhline(0, color=INK3, lw=1.2, ls=(0,(4,2))); frame(ax[0], r"raw $\hat{F}$ reading  [N]"); ax[0].set_ylim(-12, 12); ax[0].legend(loc="upper left", ncol=3, handlelength=1.8)
    ax[0].annotate("faint: the 250 Hz deadbeat reading, std 3–6 N on this vehicle\nline: what a collision threshold would see", xy=(0.985,0.06), xycoords="axes fraction", ha="right", va="bottom", fontsize=8, color=INK2)
    for i,(lab,col) in enumerate(zip(["x","y","z",r"$\psi$"],[S1,S2,S3,S4])): ax[1].plot(td, Fy[:,i], color=col, lw=1.4, label=lab)
    ax[1].axhline(0, color=INK3, lw=0.8); frame(ax[1], r"$\hat{F}_y$ into $u_3$  [N]"); ax[1].set_ylim(-1, 1); ax[1].legend(loc="upper left", ncol=4, handlelength=1.8)
    ax[1].annotate("identically zero for every DIRECT tick: $\\chi$ = free the whole flight", xy=(0.985,0.06), xycoords="axes fraction", ha="right", va="bottom", fontsize=8, color=INK2)
    for j,col in enumerate([S1,S2,S3,S4]): ax[2].plot(td, wq[:,j], color=col, lw=1.4, label=f"j{j+1}")
    ax[2].axhline(0, color=INK3, lw=0.8); frame(ax[2], r"joint-row trim $\hat{w}_q$  [N m]", last=True); ax[2].legend(loc="upper left", ncol=4, handlelength=1.8)
    phase_labels(ax[0]); ax[0].set_title("Four-dimensional attribution — the raw interaction reading, what the impedance consumed, and the joint-row trim", loc="left", pad=30)
    save(fig, out, "f8_attribution.png")

    # 9. sag story: u1, d_hat_z, battery
    fig, ax = plt.subplots(3,1,figsize=(10,5.6),sharex=True,gridspec_kw=dict(hspace=0.22))
    dual(ax[0], td, wd[:,17], S1, "$u_1$", win=2.0); ax[0].axhline(MG, color=REF, lw=1.4, ls=(0,(4,2)), label="$mg$"); frame(ax[0], "collective  [N]"); ax[0].set_ylim(35, 39.5); ax[0].legend(loc="upper left", ncol=2, handlelength=1.8)
    dual(ax[1], td, wd[:,33], S3, r"$\hat{d}_z$", win=2.0); ax[1].axhline(0, color=INK3, lw=0.8); frame(ax[1], "vertical estimate  [N]"); ax[1].legend(loc="upper left", handlelength=1.8)
    tb = d["t_batt"]; mb = (tb>=T0)&(tb<=T1); ax[2].plot(tb[mb], d["batt_v"][mb], color=S2, lw=1.4, label="pack voltage"); frame(ax[2], "battery  [V]", last=True); ax[2].legend(loc="upper left", handlelength=1.8)
    phase_labels(ax[0]); ax[0].set_title("Thrust bookkeeping over the flight — the observer follows the pack", loc="left", pad=30)
    save(fig, out, "f9_sag.png")

if __name__ == "__main__":
    print("figures ->", sys.argv[2]); main(sys.argv[1], sys.argv[2])
