#!/usr/bin/env python3
"""0902 WB hardware flight analysis: tracking errors, control inputs, arm torque
execution, trajectory checks, steady-state diagnostics.

wb_control_debug layout (57):
[0] mode [1] hold [2] gmo [3] armref fresh [4] armstate fresh
[5..8] q  [9..12] q_d_smooth  [13..16] tau_joint  [17] u1
[18..20] tau_body model  [21..23] tau_body FLU  [24..27] e_y (r_e-r_ed, heading)
[28..30] e_R  [31..40] d_e_hat (trans3, rot3, joint4)  [41..44] motors
[45..47] x_cd  [48..50] x_c  [51] n_sat  [52..54] unalloc torque FRD
[55] unalloc thrust [56] stream fresh

law_debug (arm side, 18): [9..12] received torque (arm sign convention),
[13..16] servo goal-current ticks, [17] counter-ish.
"""
import os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

BASE = "/home/fsc-jupiter/Source/fsc_PegasusSimulator/docs/experimental_data_ros2_bag/0902 - T650-AM whole-body/analysis"
SIGN = np.array([-1.0, 1.0, 1.0, -1.0])          # model = SIGN * arm convention
NM2EFF = np.array([160.0, 173.8, 146.7, 160.0])  # counts per N.m (calibrated 2026-08-17)
JN = ["j1", "j2", "j3", "j4"]

def yaw_of(qwxyz):
    w, x, y, z = qwxyz.T
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def wrap(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def onto(ts, x, td):
    if x.ndim == 1:
        return np.interp(td, ts, x)
    return np.stack([np.interp(td, ts, x[:, i]) for i in range(x.shape[1])], axis=1)

def segments(d):
    """(mode transitions, governor hold/exec windows)."""
    mt, ms = d["mode_t"], d["mode_s"]
    direct = []
    start = None
    for t, s in zip(mt, ms):
        if s == "DIRECT" and start is None:
            start = t
        elif s != "DIRECT" and start is not None:
            direct.append((start, t)); start = None
    if start is not None:
        direct.append((start, d["wb_t"][-1]))
    gt, gs = d["gstat_t"], d["gstat_s"]
    execs, holds = [], []
    cur = None; curt = None
    prev_s = None; prev_t = None
    for t, s in zip(gt, gs):
        if prev_s is not None and s != prev_s:
            if prev_s.startswith("EXECUTING"):
                execs.append((prev_t, t))
            elif prev_s == "HOLD":
                holds.append((prev_t, t))
            prev_t = t
        if prev_s is None:
            prev_t = t
        prev_s = s
    if prev_s == "HOLD":
        holds.append((prev_t, gt[-1] if gt[-1] > prev_t else d["wb_t"][-1]))
    return direct, holds, execs

def shade(ax, holds, execs):
    for a, b in execs:
        ax.axvspan(a, b, color="tab:orange", alpha=0.15, lw=0)
    for a, b in holds:
        ax.axvspan(a, b, color="tab:green", alpha=0.07, lw=0)

def analyze(tag):
    d = dict(np.load(os.path.join(BASE, tag + ".npz"), allow_pickle=True))
    out = os.path.join(BASE, tag + "_figs")
    os.makedirs(out, exist_ok=True)
    rep = []
    P = rep.append
    P(f"=================== {tag} ===================")

    wt, wb = d["wb_t"], d["wb"]
    direct, holds, execs = segments(d)
    P(f"DIRECT windows: {[(round(a,1), round(b,1)) for a, b in direct]}")
    P(f"HOLD windows:   {[(round(a,1), round(b,1)) for a, b in holds]}")
    P(f"EXEC windows:   {[(round(a,1), round(b,1)) for a, b in execs]}")
    if not direct:
        return "\n".join(rep)
    D0, D1 = direct[0][0], direct[-1][1]
    dm = (wt >= D0) & (wt <= D1)

    x_cd, x_c = wb[:, 45:48], wb[:, 48:51]
    e_com = x_cd - x_c
    e_y = wb[:, 24:28]
    e_R = wb[:, 28:31]
    q, qd, tau = wb[:, 5:9], wb[:, 9:13], wb[:, 13:17]
    u1 = wb[:, 17]
    tb_m, tb_flu = wb[:, 18:21], wb[:, 21:24]
    dhat = wb[:, 31:41]
    motors = wb[:, 41:45]
    nsat = wb[:, 51]

    # reconstruct measured EE (world) = e_y + r_ed(interp)
    r_ed = onto(d["ref_t"], d["ref_r_ed"], wt)
    r_e = e_y[:, :3] + r_ed

    # heading: model azimuth of b1_d vs measured
    b1d = onto(d["ref_t"], d["ref_b1_d"], wt)
    head_d = np.arctan2(b1d[:, 1], b1d[:, 0])
    yaw_meas = yaw_of(onto(d["od_t"], d["od_quat"], wt))
    head_m = wrap(yaw_meas - np.pi/2)

    # ---------------- fig 1: CoM + heading tracking ----------------
    fig, axs = plt.subplots(4, 2, figsize=(15, 11), sharex=True)
    for i, lab in enumerate("xyz"):
        ax = axs[i, 0]
        ax.plot(wt[dm], x_cd[dm, i], "k--", lw=1, label="x_cd ref")
        ax.plot(wt[dm], x_c[dm, i], lw=0.8, label="x_c meas")
        ax.set_ylabel(f"CoM {lab} [m]"); shade(ax, holds, execs)
        ax2 = axs[i, 1]
        ax2.plot(wt[dm], 1000*e_com[dm, i], lw=0.8, color="tab:red")
        ax2.set_ylabel(f"err {lab} [mm]"); ax2.axhline(0, color="k", lw=0.5)
        shade(ax2, holds, execs)
    axs[0, 0].legend(fontsize=8)
    axs[3, 0].plot(wt[dm], np.degrees(head_d[dm]), "k--", lw=1, label="heading ref")
    axs[3, 0].plot(wt[dm], np.degrees(head_m[dm]), lw=0.8, label="meas")
    axs[3, 0].set_ylabel("model heading [deg]"); axs[3, 0].legend(fontsize=8)
    axs[3, 1].plot(wt[dm], np.degrees(wrap(head_m - head_d))[dm], lw=0.8, color="tab:red")
    axs[3, 1].set_ylabel("heading err [deg]"); axs[3, 1].axhline(0, color="k", lw=0.5)
    shade(axs[3, 0], holds, execs); shade(axs[3, 1], holds, execs)
    for ax in axs[3]: ax.set_xlabel("t [s]")
    fig.suptitle(f"{tag} — CoM + heading tracking (green=hold, orange=execute)")
    fig.tight_layout(); fig.savefig(os.path.join(out, "01_com_heading.png"), dpi=130); plt.close(fig)

    # ---------------- fig 2: EE tracking ----------------
    fig, axs = plt.subplots(4, 2, figsize=(15, 11), sharex=True)
    for i, lab in enumerate("xyz"):
        axs[i, 0].plot(wt[dm], r_ed[dm, i], "k--", lw=1, label="r_ed ref")
        axs[i, 0].plot(wt[dm], r_e[dm, i], lw=0.8, label="r_e meas")
        gm = (d["gee_t"] >= D0) & (d["gee_t"] <= D1)
        axs[i, 0].plot(d["gee_t"][gm], d["gee_pos"][gm, i], lw=0.8, alpha=0.6, label="governor EE" if i == 0 else None)
        axs[i, 0].set_ylabel(f"EE {lab} [m]")
        axs[i, 1].plot(wt[dm], 1000*e_y[dm, i], lw=0.8, color="tab:red")
        axs[i, 1].set_ylabel(f"e_y {lab} [mm]"); axs[i, 1].axhline(0, color="k", lw=0.5)
        shade(axs[i, 0], holds, execs); shade(axs[i, 1], holds, execs)
    axs[0, 0].legend(fontsize=8)
    axs[3, 0].plot(wt[dm], e_y[dm, 3], lw=0.8)
    axs[3, 0].set_ylabel("e_y heading [-]")
    axs[3, 1].plot(wt[dm], 1000*np.linalg.norm(e_y[dm, :3], axis=1), lw=0.8)
    axs[3, 1].set_ylabel("|EE err| [mm]")
    shade(axs[3, 0], holds, execs); shade(axs[3, 1], holds, execs)
    for ax in axs[3]: ax.set_xlabel("t [s]")
    fig.suptitle(f"{tag} — end-effector tracking")
    fig.tight_layout(); fig.savefig(os.path.join(out, "02_ee_tracking.png"), dpi=130); plt.close(fig)

    # ---------------- fig 3: attitude + arm joints ----------------
    fig, axs = plt.subplots(4, 2, figsize=(15, 11), sharex=True)
    for i in range(3):
        axs[0, 0].plot(wt[dm], e_R[dm, i], lw=0.8, label=f"e_R[{i}]")
    axs[0, 0].legend(fontsize=8); axs[0, 0].set_ylabel("e_R (model)")
    eul = d["ot_eul"]
    em = (d["ot_eul_t"] >= D0) & (d["ot_eul_t"] <= D1)
    axs[0, 1].plot(d["ot_eul_t"][em], eul[em, 0], lw=0.8, label="roll")
    axs[0, 1].plot(d["ot_eul_t"][em], eul[em, 1], lw=0.8, label="pitch")
    axs[0, 1].set_ylabel("attitude [deg?]"); axs[0, 1].legend(fontsize=8)
    shade(axs[0, 0], holds, execs); shade(axs[0, 1], holds, execs)
    for j in range(4):
        r, c = 1 + j // 2, j % 2
        axs[r, c].plot(wt[dm], np.degrees(qd[dm, j]), "k--", lw=1, label="q_d")
        axs[r, c].plot(wt[dm], np.degrees(q[dm, j]), lw=0.8, label="q")
        axs[r, c].set_ylabel(f"{JN[j]} [deg]"); shade(axs[r, c], holds, execs)
    axs[1, 0].legend(fontsize=8)
    axs[3, 0].plot(wt[dm], np.degrees(q[dm] - qd[dm]), lw=0.8)
    axs[3, 0].legend(JN, fontsize=7); axs[3, 0].set_ylabel("q - q_d [deg]")
    axs[3, 1].plot(wt[dm], nsat[dm], lw=0.8); axs[3, 1].set_ylabel("n_sat")
    shade(axs[3, 0], holds, execs); shade(axs[3, 1], holds, execs)
    for ax in axs[3]: ax.set_xlabel("t [s]")
    fig.suptitle(f"{tag} — attitude error + arm joints")
    fig.tight_layout(); fig.savefig(os.path.join(out, "03_att_arm.png"), dpi=130); plt.close(fig)

    # ---------------- fig 4: control inputs ----------------
    fig, axs = plt.subplots(5, 1, figsize=(15, 13), sharex=True)
    axs[0].plot(wt[dm], u1[dm], lw=0.8); axs[0].axhline(3.746170*9.81, color="k", ls=":", lw=1, label="mg")
    axs[0].set_ylabel("u1 [N]"); axs[0].legend(fontsize=8)
    for i, lab in enumerate("xyz"):
        axs[1].plot(wt[dm], tb_m[dm, i], lw=0.8, label=f"tau_{lab} model")
    axs[1].legend(fontsize=8); axs[1].set_ylabel("tau_body model [N.m]")
    for j in range(4):
        axs[2].plot(wt[dm], tau[dm, j], lw=0.8, label=JN[j])
    axs[2].axhline(3.0, color="k", ls=":", lw=0.8); axs[2].axhline(-3.0, color="k", ls=":", lw=0.8)
    axs[2].legend(fontsize=8); axs[2].set_ylabel("tau_joint cmd [N.m]")
    for j in range(4):
        axs[3].plot(wt[dm], motors[dm, j], lw=0.8, label=f"m{j+1}")
    axs[3].legend(fontsize=8); axs[3].set_ylabel("motor cmd [0..1]")
    for i in range(3):
        axs[4].plot(wt[dm], dhat[dm, i], lw=0.8, label=f"d_t{'xyz'[i]}")
    for i in range(3):
        axs[4].plot(wt[dm], dhat[dm, 3+i], lw=0.8, ls="--", label=f"d_r{'xyz'[i]}")
    axs[4].legend(fontsize=7, ncol=3); axs[4].set_ylabel("GMO d_hat (trans N / rot N.m)")
    axs[4].set_xlabel("t [s]")
    for ax in axs: shade(ax, holds, execs)
    fig.suptitle(f"{tag} — control inputs")
    fig.tight_layout(); fig.savefig(os.path.join(out, "04_control_inputs.png"), dpi=130); plt.close(fig)

    # ---------------- fig 5: arm torque execution ----------------
    lt, law = d["law_t"], d["law"]
    lm = (lt >= D0) & (lt <= D1)
    tl = lt[lm]
    tau_f = onto(wt, tau, tl)                       # flight cmd, model conv
    tau_recv = law[lm, 9:13] * SIGN                 # arm received -> model conv
    ticks_cmd = law[lm, 13:17]
    tau_srv_cmd = ticks_cmd / NM2EFF * SIGN         # what the servo was asked, model conv
    eff = onto(d["js_t"], d["js_eff"][:, 2:6], tl)  # joint1..4 present current ticks
    tau_del = eff / NM2EFF * SIGN                   # delivered, model conv
    fig, axs = plt.subplots(4, 2, figsize=(15, 11))
    P("\nARM TORQUE EXECUTION (model convention, N.m):")
    P("joint |  corr(recv,flight) slope |  corr(deliv,flight) slope  mean(fl) mean(del)")
    for j in range(4):
        ax = axs[j, 0]
        ax.plot(tl, tau_f[:, j], "k", lw=0.9, label="flight cmd")
        ax.plot(tl, tau_srv_cmd[:, j], lw=0.7, alpha=0.8, label="servo goal (ticks/k)")
        ax.plot(tl, tau_del[:, j], lw=0.7, alpha=0.7, label="delivered (present current/k)")
        ax.set_ylabel(f"{JN[j]} [N.m]"); shade(ax, holds, execs)
        if j == 0: ax.legend(fontsize=7)
        ax2 = axs[j, 1]
        ax2.plot(tau_f[:, j], tau_del[:, j], ".", ms=1, alpha=0.3)
        lim = [tau_f[:, j].min(), tau_f[:, j].max()]
        ax2.plot(lim, lim, "k--", lw=0.8)
        ax2.set_xlabel("flight cmd"); ax2.set_ylabel("delivered")
        c1 = np.corrcoef(tau_recv[:, j], tau_f[:, j])[0, 1]
        s1 = np.polyfit(tau_f[:, j], tau_recv[:, j], 1)[0]
        c2 = np.corrcoef(tau_del[:, j], tau_f[:, j])[0, 1]
        s2, o2 = np.polyfit(tau_f[:, j], tau_del[:, j], 1)
        P(f"  {JN[j]} |  {c1:+.3f} {s1:+.3f}        |  {c2:+.3f} {s2:+.3f} (off {o2:+.3f})   {tau_f[:,j].mean():+.3f}  {tau_del[:,j].mean():+.3f}")
    axs[3, 0].set_xlabel("t [s]")
    fig.suptitle(f"{tag} — arm torque execution: flight cmd vs servo goal vs delivered")
    fig.tight_layout(); fig.savefig(os.path.join(out, "05_arm_torque_execution.png"), dpi=130); plt.close(fig)

    # ---------------- fig 6: reference detail (compatible trajectory) ----------------
    rt = d["ref_t"]
    rm = (rt >= D0) & (rt <= D1)
    fig, axs = plt.subplots(5, 1, figsize=(15, 13), sharex=True)
    for i, lab in enumerate("xyz"):
        axs[0].plot(rt[rm], d["ref_x_cd"][rm, i], lw=0.8, label=f"x_cd {lab}")
        axs[1].plot(rt[rm], d["ref_x_cd_dot"][rm, i], lw=0.8, label=f"v {lab}")
        axs[2].plot(rt[rm], d["ref_x_cd_ddot"][rm, i], lw=0.8, label=f"a {lab}")
        axs[3].plot(rt[rm], d["ref_r_ed"][rm, i], lw=0.8, label=f"r_ed {lab}")
    for j in range(4):
        axs[4].plot(rt[rm], np.degrees(d["ref_q_d"][rm, j]), lw=0.8, label=JN[j])
    for ax, yl in zip(axs, ["x_cd [m]", "x_cd_dot [m/s]", "x_cd_ddot [m/s2]", "r_ed [m]", "q_d [deg]"]):
        ax.set_ylabel(yl); ax.legend(fontsize=7, ncol=4); shade(ax, holds, execs)
    axs[4].set_xlabel("t [s]")
    fig.suptitle(f"{tag} — streamed WholeBodyReference")
    fig.tight_layout(); fig.savefig(os.path.join(out, "06_reference.png"), dpi=130); plt.close(fig)

    # FD consistency + continuity of the reference
    P("\nREFERENCE CHECKS:")
    dt = np.diff(rt)
    good = dt > 1e-4
    fd = np.diff(d["ref_x_cd"], axis=0)[good] / dt[good, None]
    an = 0.5 * (d["ref_x_cd_dot"][1:] + d["ref_x_cd_dot"][:-1])[good]
    P(f"  x_cd FD vs x_cd_dot: rms diff = {np.sqrt(np.mean((fd-an)**2)):.4f} m/s (stream), max |v| = {np.abs(d['ref_x_cd_dot'][rm]).max():.3f} m/s, max |a| = {np.abs(d['ref_x_cd_ddot'][rm]).max():.3f} m/s2")
    jx = np.abs(np.diff(d["ref_x_cd"], axis=0))
    P(f"  max inter-sample step x_cd = {1000*jx.max():.1f} mm; r_ed = {1000*np.abs(np.diff(d['ref_r_ed'],axis=0)).max():.1f} mm; q_d = {np.degrees(np.abs(np.diff(d['ref_q_d'],axis=0)).max()):.2f} deg")
    P(f"  max |qdot_d| = {np.degrees(np.abs(d['ref_qdot_d'][rm]).max()):.1f} deg/s")

    # ---------------- arm feedback health figure ----------------
    jt = d["js_t"]
    fig, axs = plt.subplots(3, 1, figsize=(15, 8), sharex=True)
    axs[0].plot(jt, np.degrees(d["js_pos"][:, 3]), lw=0.8, label="q2 joint_states")
    axs[0].plot(jt, np.degrees(d["js_pos"][:, 4]), lw=0.8, label="q3 joint_states")
    axs[0].plot(wt, np.degrees(qd[:, 1]), "k--", lw=0.8, label="q2_d")
    axs[0].plot(wt, np.degrees(qd[:, 2]), "r--", lw=0.8, label="q3_d")
    axs[0].set_ylabel("arm pos [deg]"); axs[0].legend(fontsize=8)
    axs[1].plot(jt, d["js_eff"][:, 3], lw=0.8, label="eff2 [ticks]")
    axs[1].plot(jt, d["js_eff"][:, 4], lw=0.8, label="eff3 [ticks]")
    axs[1].set_ylabel("present current [ticks]"); axs[1].legend(fontsize=8)
    # rolling std of q2 (1 s window) -- zero == frozen stream
    w = 250
    if len(jt) > 2*w:
        q2 = np.degrees(d["js_pos"][:, 3])
        rs = np.array([q2[i:i+w].std() for i in range(0, len(q2)-w, w//4)])
        rt_ = jt[np.arange(0, len(q2)-w, w//4) + w//2]
        axs[2].semilogy(rt_, np.maximum(rs, 1e-7), lw=0.9)
    axs[2].axhline(1e-3, color="r", ls=":", lw=1)
    axs[2].set_ylabel("rolling std q2 [deg] (1 s)"); axs[2].set_xlabel("t [s]")
    axs[2].set_title("frozen feedback shows as std == 0 (floored at 1e-7)", fontsize=9)
    for ax in axs: shade(ax, holds, execs)
    fig.suptitle(f"{tag} — arm feedback health (encoder noise vanishes when the read path freezes)")
    fig.tight_layout(); fig.savefig(os.path.join(out, "07_arm_feedback_health.png"), dpi=130); plt.close(fig)

    # ---------------- steady-state table ----------------
    P("\nSTEADY-STATE (mean over last 60% of each HOLD window):")
    P(" window        | CoM err mm (x,y,z) | EE e_y mm (x,y,z) | e_y_hd | headerr deg | q-q_d deg | d_t N (x,y,z) | d_r N.m | u1 N")
    for a, b in holds:
        a2 = a + 0.4 * (b - a)
        m = (wt >= a2) & (wt <= b)
        if m.sum() < 10:
            continue
        ec = 1000 * e_com[m].mean(axis=0)
        ee = 1000 * e_y[m, :3].mean(axis=0)
        eh = e_y[m, 3].mean()
        hh = np.degrees(wrap(head_m - head_d)[m].mean())
        dq = np.degrees((q - qd)[m].mean(axis=0))
        dt_ = dhat[m, :3].mean(axis=0)
        dr_ = dhat[m, 3:6].mean(axis=0)
        P(f" {a:6.1f}-{b:6.1f} | {ec[0]:+7.1f} {ec[1]:+7.1f} {ec[2]:+7.1f} | {ee[0]:+7.1f} {ee[1]:+7.1f} {ee[2]:+7.1f} | {eh:+.3f} | {hh:+7.2f} | " +
          " ".join(f"{v:+5.1f}" for v in dq) + f" | {dt_[0]:+5.2f} {dt_[1]:+5.2f} {dt_[2]:+5.2f} | {dr_[0]:+5.3f} {dr_[1]:+5.3f} {dr_[2]:+5.3f} | {u1[m].mean():6.2f}")

    # convergence trend inside each hold: |CoM err| mean of thirds
    P("\nHOLD CONVERGENCE (|CoM err| horizontal, mean per third of window, mm) + e_R norm:")
    for a, b in holds:
        th = (b - a) / 3
        vals = []
        ers = []
        for k in range(3):
            m = (wt >= a + k*th) & (wt < a + (k+1)*th)
            if m.sum() < 5:
                vals.append(np.nan); ers.append(np.nan); continue
            vals.append(1000*np.linalg.norm(e_com[m, :2], axis=1).mean())
            ers.append(np.linalg.norm(e_R[m], axis=1).mean())
        P(f" {a:6.1f}-{b:6.1f}: |e_com| {vals[0]:7.1f} -> {vals[1]:7.1f} -> {vals[2]:7.1f} mm   |e_R| {ers[0]:.3f} -> {ers[1]:.3f} -> {ers[2]:.3f}")

    # base_com residual: steady rotational GMO / u1 (model frame: y = forward)
    P("\nBASE-CoM RESIDUAL from steady d_r (model x = pitch-ish axis, forward = +y):")
    for a, b in holds:
        a2 = a + 0.4 * (b - a)
        m = (wt >= a2) & (wt <= b)
        if m.sum() < 10:
            continue
        drx = dhat[m, 3].mean(); dry = dhat[m, 4].mean(); u1m = u1[m].mean()
        qm = np.degrees(q[m].mean(axis=0))
        # tau = r x (0,0,u1): tau_x = u1*ry  -> ry = tau_x/u1 ; tau_y = -u1*rx -> rx = -tau_y/u1
        P(f" {a:6.1f}-{b:6.1f}  q=({qm[0]:.0f},{qm[1]:.0f},{qm[2]:.0f},{qm[3]:.0f})deg  d_r=({drx:+.3f},{dry:+.3f}) N.m  => CoM residual model (dx,dy)=({-dry/u1m*1000:+.1f},{drx/u1m*1000:+.1f}) mm")

    # anti-damping check: tau_body model x vs forward velocity (model +y = world?)
    vel = onto(d["od_t"], d["od_vel"], wt)
    yawm = yaw_meas
    fwd_w = np.stack([-np.sin(yawm), np.cos(yawm)], axis=1)  # body +y(model fwd)? verify: model fwd = actual body +x rotated..
    # actual front = body +x (actual FLU); forward velocity in world:
    fwd = np.stack([np.cos(yawm), np.sin(yawm)], axis=1)
    v_fwd = vel[:, 0]*fwd[:, 0] + vel[:, 1]*fwd[:, 1]
    mm = dm & (np.abs(v_fwd) > 0.0)
    s, o = np.polyfit(v_fwd[mm], tb_m[mm, 0], 1)
    r = np.corrcoef(v_fwd[mm], tb_m[mm, 0])[0, 1]
    P(f"\nANTI-DAMPING CHECK: tau_model_x vs v_fwd slope = {s:+.3f} N.m/(m/s), r = {r:+.2f} (0901 was -0.36; positive/small = healthy)")

    txt = "\n".join(rep)
    with open(os.path.join(out, "stats.txt"), "w") as f:
        f.write(txt + "\n")
    return txt

if __name__ == "__main__":
    tags = sys.argv[1:] or ["wb_tracking_20260902_124652", "wb_tracking_20260902_125222", "wb_tracking_20260902_125951"]
    for tg in tags:
        print(analyze(tg))
        print()
