#!/usr/bin/env python3
"""The whole-body GMO-vs-L1 report figures: tracking, inputs, disturbances.

    PYTHONNOUSERSITE=1 /usr/bin/python3 wb_report_figures.py \
        --gmo gmo.npz --l1 l1.npz --outdir DIR [--marks-from l1.npz]

Run under PYTHONNOUSERSITE=1 (apt matplotlib is built for numpy 1.x while
~/.local carries 2.x); the object arrays are read separately by the wrapper,
so this script touches only plain numeric arrays.

THREE FIGURES, because they answer three different questions:

  1_tracking      every reference the law consumes against the state it
                  achieved -- system CoM, base heading, END-EFFECTOR position
                  and heading, and the joints. The EE state is not logged
                  directly and is NOT reconstructed by forward kinematics:
                  e_y[0:3] is exactly r_e - r_ed by definition, so
                  r_e = r_ed + e_y is exact and carries no attitude error.
  2_inputs        every control input: collective u1, the four joint torques,
                  and the four normalised rotor commands.
  3_disturbance   what each observer believes is acting on the vehicle, and
                  the phantom force -- the free-flight bias that the impedance
                  law spends as if it were a contact wrench. Nothing touches
                  the arm in either flight, so the true value is exactly zero
                  and every newton plotted is error.
"""

import argparse
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

D_MODE, D_U1, D_NSAT = 0, 17, 51
D_TAU = slice(13, 17)
D_EY = slice(24, 28)
D_ER = slice(28, 31)
D_DHAT = slice(31, 41)
D_MOT = slice(41, 45)
D_XCD = slice(45, 48)
D_XC = slice(48, 51)
D_FY = slice(58, 62)
D_WHATE = slice(72, 78)

# wbref columns after t: x_cd(3) x_cd_dot(3) b1_d(3) r_ed(3) r_ed_dot(3)
# b1_de(3) q_d(4) qdot_d(4)
W_XCD, W_B1D, W_RED, W_B1DE = slice(1, 4), slice(7, 10), slice(10, 13), slice(16, 19)
W_QD = slice(19, 23)

C_G, C_L, C_REF = "#B4451F", "#0E6F79", "#9AA0A6"
LW, LWR = 1.0, 2.4


def load(path):
    z = np.load(path, allow_pickle=False)
    dbg, log = z["dbg"], z["log"]
    wb = z["wbref"] if "wbref" in z.files else np.zeros((0, 1))
    d = dbg[:, 1:]
    if d.shape[1] <= D_WHATE.stop:
        d = np.hstack([d, np.full((d.shape[0], D_WHATE.stop + 1 - d.shape[1]),
                                  np.nan)])
    m = np.nan_to_num(d[:, D_MODE]) > 0.5
    if not m.any():
        raise SystemExit(f"{path}: never entered DIRECT")
    t0 = dbg[m, 0][0]
    lm = log[:, 17] > 0.5
    wm = (wb[:, 0] >= t0) if wb.shape[1] > 1 else np.zeros(len(wb), bool)
    return dict(t=dbg[m, 0] - t0, d=d[m],
                lt=log[lm, 0] - t0, log=log[lm],
                wt=wb[wm, 0] - t0 if wb.shape[1] > 1 else np.zeros(0),
                wb=wb[wm] if wb.shape[1] > 1 else np.zeros((0, 1)))


def decorate(ax, marks, ylab, title=None, legend=True):
    ax.grid(alpha=.25, lw=.5)
    ax.set_ylabel(ylab, fontsize=9)
    if title:
        # pad clears the rotated leg labels drawn above the top axes
        ax.set_title(title, fontsize=9.5, loc="left",
                     pad=(30 if ax.get_subplotspec().rowspan.start == 0 else 4))
    for _, tm in marks:
        ax.axvline(tm, color="k", lw=.5, alpha=.22, ls="--")
    if legend:
        ax.legend(loc="upper right", fontsize=7.5, ncol=4, framealpha=.9)


def yaw_from_b1(b1):
    return np.degrees(np.arctan2(b1[:, 1], b1[:, 0]))


def fig_tracking(G, L, marks, out):
    fig, ax = plt.subplots(6, 1, figsize=(13.5, 17), sharex=True,
                           gridspec_kw=dict(hspace=.18))
    fig.suptitle("Reference vs state — whole-body DIRECT", fontsize=13, y=.965)

    for k, c in enumerate("xyz"):
        ax[0].plot(G["t"], G["d"][:, D_XCD][:, k], color=C_REF, lw=LWR,
                   alpha=.75, zorder=1, label="reference" if k == 0 else None)
        ax[0].plot(G["t"], G["d"][:, D_XC][:, k], color=C_G, lw=LW,
                   label="GMO" if k == 0 else None)
        ax[1].plot(L["t"], L["d"][:, D_XCD][:, k], color=C_REF, lw=LWR,
                   alpha=.75, zorder=1, label="reference" if k == 0 else None)
        ax[1].plot(L["t"], L["d"][:, D_XC][:, k], color=C_L, lw=LW,
                   label="L1" if k == 0 else None)
    decorate(ax[0], marks, "CoM $x_c$ [m]", "System CoM — GMO")
    decorate(ax[1], marks, "CoM $x_c$ [m]", "System CoM — L1")

    # END-EFFECTOR: r_e = r_ed + e_y[0:3], exact by definition of e_y.
    for R, C, nm, a in ((G, C_G, "GMO", ax[2]), (L, C_L, "L1", ax[3])):
        if len(R["wt"]) > 8:
            red = R["wb"][:, W_RED]
            ey = np.vstack([np.interp(R["wt"], R["t"], R["d"][:, D_EY][:, k])
                            for k in range(3)]).T
            for k in range(3):
                a.plot(R["wt"], red[:, k], color=C_REF, lw=LWR, alpha=.75,
                       zorder=1, label="reference" if k == 0 else None)
                a.plot(R["wt"], red[:, k] + ey[:, k], color=C, lw=LW,
                       label=nm if k == 0 else None)
        decorate(a, marks, "EE $r_e$ [m]", f"End-effector position — {nm}")

    # Headings, reference AND state. The base heading state is the logged yaw
    # mapped into the law's model frame (phi_model = psi_actual - pi/2, the
    # same conversion frame_adapter.hpp makes). The EE heading state comes
    # from its own error channel: e_y[3] = e_RE3 is the heading error, so
    # state = reference - e_RE3 to first order.
    for R, C, nm in ((G, C_G, "GMO"), (L, C_L, "L1")):
        if len(R["wt"]) > 8:
            ax[4].plot(R["wt"], yaw_from_b1(R["wb"][:, W_B1D]), color=C_REF,
                       lw=LWR, alpha=.75, zorder=1,
                       label="reference" if nm == "GMO" else None)
            ax[4].plot(R["wt"], yaw_from_b1(R["wb"][:, W_B1DE]), color=C_REF,
                       lw=LWR, alpha=.75, zorder=1, ls=":")
            eb = np.degrees(np.interp(R["wt"], R["t"], R["d"][:, D_EY][:, 3]))
            ax[4].plot(R["wt"], yaw_from_b1(R["wb"][:, W_B1DE]) - eb,
                       color=C, lw=LW, ls=":",
                       label=f"{nm} EE state" if True else None)
        ax[4].plot(R["lt"], np.degrees(R["log"][:, 8]) - 90.0, color=C,
                   lw=LW, label=f"{nm} base state")
    decorate(ax[4], marks, "heading [deg]",
             "Base heading (solid) and end-effector heading (dotted) — "
             "reference and state, model frame")

    # COLOUR BY JOINT, not by observer: four references and eight measured
    # traces in two colours is unreadable, and it reads as a large tracking
    # error that is not there (measured max |q - q_d| is 0.3-3.5 deg).
    # THE TWO SOURCES ARE IN DIFFERENT ORDERS. wbref's q_d is the law's
    # q1..q4; the joint_state_broadcaster on this rig publishes
    # [q2, q3, q1, q4] -- at the ground pose it reads (40,40,0,0) for the home
    # [0,40,40,0] deg, and in flight column 2 pins at exactly -35, which only
    # q1 can do. Verified against q_d at every leg hold before plotting.
    QC = ("#1B4F72", "#B4451F", "#0E6F79", "#7D6608")
    for R, ls, nm in ((G, "--", "GMO"), (L, "-", "L1")):
        if len(R["wt"]) > 8:
            qd = np.degrees(R["wb"][:, W_QD])
            for k in range(4):
                ax[5].plot(R["wt"], qd[:, k], color=C_REF, lw=LWR + 1.0,
                           alpha=.55, zorder=1,
                           label="reference" if (k == 0 and nm == "L1") else None)
        q = np.degrees(R["log"][:, [15, 13, 14, 16]])
        for k in range(4):
            ax[5].plot(R["lt"], q[:, k], color=QC[k], lw=LW, ls=ls,
                       label=(f"$q_{k+1}$" if nm == "L1" else None))
    ax[5].plot([], [], color="k", lw=LW, ls="-", label="L1")
    ax[5].plot([], [], color="k", lw=LW, ls="--", label="GMO")
    decorate(ax[5], marks, "joints [deg]",
             "Arm joints — reference (grey) and measured; solid L1, dashed GMO")
    ax[5].set_xlabel("time since DIRECT entry [s]")
    for _, tm in marks:
        pass
    for nm, tm in marks:
        ax[0].annotate(nm, (tm, 1.03), xycoords=("data", "axes fraction"),
                       fontsize=7, rotation=38, ha="left", va="bottom")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print("wrote", out)


def fig_inputs(G, L, marks, out):
    fig, ax = plt.subplots(4, 1, figsize=(13.5, 12), sharex=True,
                           gridspec_kw=dict(hspace=.18))
    fig.suptitle("Control inputs — whole-body DIRECT", fontsize=13, y=.965)

    for R, C, nm in ((G, C_G, "GMO"), (L, C_L, "L1")):
        ax[0].plot(R["t"], R["d"][:, D_U1], color=C, lw=LW, label=nm)
    # The plant is the NOMINAL mass x 1.10, so its weight is 40.41 N, and the
    # allocator believes a kf 1.1765x the truth -- so the COMMANDED collective
    # must sit at 40.41 x 1.1765 = 47.55 N for the vehicle to hover. Both
    # observers finding exactly that is the check that the injected mismatch
    # was really active and was really absorbed.
    ax[0].axhline(40.41, color="k", lw=.8, ls="--",
                  label="true weight $mg$ = 40.41 N")
    ax[0].axhline(47.55, color="k", lw=.8, ls=":",
                  label="commanded hover (kf mismatch) = 47.55 N")
    decorate(ax[0], marks, "$u_1$ [N]",
             "Collective thrust — commanded, against what the plant weighs")

    for a, (R, C, nm) in zip(ax[1:3], ((G, C_G, "GMO"), (L, C_L, "L1"))):
        for k in range(4):
            a.plot(R["t"], R["d"][:, D_TAU][:, k], lw=LW,
                   label=f"$\\tau_{{q{k+1}}}$")
        a.axhline(3.0, color="k", lw=.8, ls=":")
        a.axhline(-3.0, color="k", lw=.8, ls=":", label="clamp $\\pm$3 N·m")
        decorate(a, marks, "$u_3$ [N·m]", f"Arm joint torques — {nm}")

    # All four rotors individually. A min-max band is useless here: the four
    # commands differ by a near-constant 0.15 (the yaw torque the allocator
    # needs against km), so the band is a solid block that hides everything.
    RC = ("#1B4F72", "#B4451F", "#0E6F79", "#7D6608")
    for R, ls, nm in ((G, "--", "GMO"), (L, "-", "L1")):
        mot = R["d"][:, D_MOT]
        for k in range(4):
            ax[3].plot(R["t"], mot[:, k], color=RC[k], lw=0.9, ls=ls,
                       label=(f"rotor {k}" if nm == "L1" else None))
    ax[3].plot([], [], color="k", lw=LW, ls="-", label="L1")
    ax[3].plot([], [], color="k", lw=LW, ls="--", label="GMO")
    decorate(ax[3], marks, "rotor cmd [–]",
             "Normalised rotor commands — all four; solid L1, dashed GMO")
    ax[3].set_xlabel("time since DIRECT entry [s]")
    for nm, tm in marks:
        ax[0].annotate(nm, (tm, 1.03), xycoords=("data", "axes fraction"),
                       fontsize=7, rotation=38, ha="left", va="bottom")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print("wrote", out)


def fig_disturbance(G, L, marks, out):
    fig, ax = plt.subplots(4, 1, figsize=(13.5, 12.5), sharex=True,
                           gridspec_kw=dict(hspace=.18))
    fig.suptitle("Disturbance estimate and the phantom end-effector force",
                 fontsize=13, y=.965)

    for R, C, nm in ((G, C_G, "GMO"), (L, C_L, "L1")):
        dh = R["d"][:, D_DHAT]
        for k, c in enumerate("xyz"):
            ax[0].plot(R["t"], dh[:, k], color=C, lw=LW,
                       label=nm if k == 0 else None)
            ax[1].plot(R["t"], dh[:, 3 + k], color=C, lw=LW,
                       label=nm if k == 0 else None)
        for k in range(4):
            ax[2].plot(R["t"], dh[:, 6 + k], color=C, lw=LW,
                       label=nm if k == 0 else None)
    decorate(ax[0], marks, r"$\hat d_t$ [N]",
             "Translational disturbance estimate — the ~10.8 N of injected "
             "mismatch lives here")
    decorate(ax[1], marks, r"$\hat d_r$ [N·m]",
             "Rotational disturbance estimate")
    decorate(ax[2], marks, r"$\hat d_\rho$ [N·m]",
             "Arm-channel disturbance estimate")

    for R, C, nm in ((G, C_G, "GMO"), (L, C_L, "L1")):
        fy = np.linalg.norm(R["d"][:, D_FY][:, :3], axis=1)
        ax[3].plot(R["t"], fy, color=C, lw=LW, label=nm)
    ax[3].axhline(0.0, color="k", lw=.9, ls=":", label="truth (free flight)")
    decorate(ax[3], marks, r"$|\hat F_y|$ [N]",
             "PHANTOM end-effector force — nothing touches the arm, so the "
             "true value is exactly 0")
    ax[3].set_xlabel("time since DIRECT entry [s]")
    for nm, tm in marks:
        ax[0].annotate(nm, (tm, 1.03), xycoords=("data", "axes fraction"),
                       fontsize=7, rotation=38, ha="left", va="bottom")
    fig.savefig(out, dpi=120, bbox_inches="tight")
    print("wrote", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gmo", required=True)
    ap.add_argument("--l1", required=True)
    ap.add_argument("--outdir", required=True)
    ap.add_argument("--marks", default="")
    a = ap.parse_args()
    G, L = load(a.gmo), load(a.l1)
    marks = []
    for tok in filter(None, a.marks.split(",")):
        n, v = tok.rsplit(":", 1)
        marks.append((n, float(v)))
    os.makedirs(a.outdir, exist_ok=True)
    fig_tracking(G, L, marks, os.path.join(a.outdir, "1_tracking.png"))
    fig_inputs(G, L, marks, os.path.join(a.outdir, "2_inputs.png"))
    fig_disturbance(G, L, marks, os.path.join(a.outdir, "3_disturbance.png"))


if __name__ == "__main__":
    main()
