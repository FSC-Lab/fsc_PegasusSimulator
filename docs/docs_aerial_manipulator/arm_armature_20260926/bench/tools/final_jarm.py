#!/usr/bin/env python3
"""The law's J_arm, calibrated from the ground-bench bags only: every method variant, the
systematic band, per-unit values in flight order, and what the value does to the law's terms.

    PYTHONNOUSERSITE=1 /usr/bin/python3 final_jarm.py        (writes ../results.txt, ../jarm_bench.png)

Structure fitted = the law's own (controller.make_params:236-239, wb_model.cpp:135-140):
J_arm h_i h_i^T on every arm link, so a single moving joint's row carries
    j1: J1 + cos^2(q2+q3) J4 (0.03 at home)   j2: J2 + J3   j3: J3   j4: J4.
0911 slot j1 is EXCLUDED from the fit: the joint-1 mounting screws were loose until the
"joint1 resolution" before eval_final (bag README) and its windows are all before that or
ambiguous; it is reported separately.
"""
import io, os, sys, contextlib
import numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import ident_bench as IB
import ident_plateau as IPL
import calib_jarm as CJ
import calib_jarm_v2 as V2
import arm_stiffness_sim as AS
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT

OUTD = os.path.join(IB.HERE, "..")
UNITS = (11, 12, 13, 14)
SE_FLOOR = 0.0015
EXCLUDE = {("s0911", 0)}


def coef_row(sess, k, C, structure):
    c = np.zeros(4)
    for i in range(4):
        c[UNITS.index(IB.SLOT_UNIT[sess][i])] += C[i, k] if structure == "hhT" else float(i == k)
    return c


def wls(rows, per_unit, structure):
    A, y, w = [], [], []
    for (sess, k, J, se, C) in rows:
        c = coef_row(sess, k, C, structure)
        A.append(c if per_unit else [c.sum()]); y.append(J); w.append(1.0 / max(se, SE_FLOOR))
    A = np.array(A) * np.array(w)[:, None]; y = np.array(y) * np.array(w)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    r = y - A @ x
    return x, np.sqrt(np.diag(np.linalg.pinv(A.T @ A))), r @ r, len(y) - len(x)


def law_terms(J, q_deg):
    X = np.zeros(18 + 2 * AS.N); X[3:12] = np.eye(3).reshape(9, order="F"); X[12:16] = np.radians(q_deg)
    d = CT.dynamics(X, AS.model_params("hht", J))
    Mr = d["M_tilde"][6:, 6:]; J3 = d["J_3y"]; Lam = d["Lambda_y"]
    G = J3.T @ np.linalg.solve(J3 @ J3.T + 0.09 * np.eye(4), Lam @ np.linalg.inv(np.diag([1, 1, 1, 0.05])))
    Kq = G @ np.diag([20, 20, 20, 0.3]) @ d["J_y"][:, 6:]
    return dict(Mr=np.diag(Mr), Mr_eig=np.linalg.eigvalsh(Mr[1:3, 1:3]),
                Kq_eig=np.linalg.eigvalsh(0.5 * (Kq[1:3, 1:3] + Kq[1:3, 1:3].T)), Kq=np.diag(Kq),
                M_r=np.diag(d["M_r"]), N1x=d["N1"][:, 0])


if __name__ == "__main__":
    out = io.StringIO()
    def P(*a):
        print(*a); print(*a, file=out)

    Ds = IB.prepare_all(2.0)
    Wall = IPL.sine_windows(Ds)
    Cpose = {}
    for W in Wall:
        W["C"] = Cpose.setdefault(W["pose"], CJ.unit_columns(np.median(W["D"]["q"][W["s"]], axis=0)))
    MW = CJ.move_windows(Ds)

    # ---------- sine variants ----------
    SV = {}
    for vf in (0.3, 0.5, 0.7, 0.8):
        for dl in (0, 2):
            IPL.VFRAC = vf
            rows = []
            for sess in ("s0909", "s0911"):
                for k in range(4):
                    WW = [W for W in Wall if W["D"]["sess"] == sess and W["k"] == k]
                    J, se, N, rs, Kg = IPL.fit(WW, dl)
                    rows.append((sess, k, J, se, WW[0]["C"]))
            SV[(vf, dl * 4)] = rows
    # ---------- move variants (j2, j3 rows) ----------
    MV = {}
    for d in (3.0, 5.0, 7.0):
        for mu in (0.7, 1.0, 1.3):
            rows = []
            for sess in ("s0909", "s0911"):
                for k in (1, 2):
                    MM = [W for W in MW if W["D"]["sess"] == sess and W["k"] == k]
                    J, se, N = V2.fit_moves(MM, d, mu)
                    C = Cpose[[key for key in Cpose if key[0] == sess and key[1] == k][0]]
                    rows.append((sess, k, J, se, C))
            MV[(d, mu)] = rows

    P("=" * 100)
    P("MEASURED armature coefficient on each joint's own row, J_row [kg m^2]  (median [min..max] over method variants)")
    P("  sines: kinetic threshold 0.3/0.5/0.7/0.8 x current delay 0/8 ms;  moves: d_min 3/5/7 deg x mu 0.7/1.0/1.3")
    P("=" * 100)
    summary = {}
    for sess in ("s0909", "s0911"):
        for k in range(4):
            sv = np.array([[r[2] for r in rows if r[0] == sess and r[1] == k][0] for rows in SV.values()])
            line = (f"  {sess} slot j{k+1} (unit {IB.SLOT_UNIT[sess][k]}):  sines {np.median(sv):.4f} "
                    f"[{sv.min():+.4f}..{sv.max():+.4f}]")
            mv = [[r[2] for r in rows if r[0] == sess and r[1] == k] for rows in MV.values()]
            if mv[0]:
                mv = np.array([m[0] for m in mv])
                line += f"   moves {np.median(mv):.4f} [{mv.min():+.4f}..{mv.max():+.4f}]"
            else:
                mv = None
            if (sess, k) in EXCLUDE:
                line += "   (EXCLUDED: loose joint-1 screws)"
            summary[(sess, k)] = (sv, mv)
            P(line)

    P("\n" + "=" * 100)
    P("THE LAW'S J_arm (h h^T on every link), weighted fits; distribution over method variants")
    P("=" * 100)
    fits = {}
    for label, variants in (("sines only", [(v, None) for v in SV]),
                            ("sines + j3-row moves", [(v, m) for v in SV for m in MV])):
        for structure in ("hhT", "diag"):
            xs, xu, chis = [], [], []
            for (v, m) in variants:
                rows = [r for r in SV[v] if (r[0], r[1]) not in EXCLUDE]
                if m is not None:
                    rows = rows + [r for r in MV[m] if r[1] == 2]
                x1, s1, c1, d1 = wls(rows, False, structure)
                x4, s4, c4, d4 = wls(rows, True, structure)
                xs.append(x1[0]); xu.append(x4); chis.append((c1, d1, c4, d4))
            xs = np.array(xs); xu = np.array(xu); ch = np.array(chis)
            fits[(label, structure)] = (xs, xu, ch)
            nm = "h h^T (THE LAW)" if structure == "hhT" else "joint-diagonal"
            P(f"\n  [{label}]  {nm}")
            P(f"     ONE SCALAR J_arm = {np.median(xs):.4f}  [{xs.min():.4f}..{xs.max():.4f}] over {len(xs)} variants"
              f"   chi2/dof median {np.median(ch[:,0]):.1f}/{int(ch[0,1])}")
            med = np.median(xu, axis=0); lo = xu.min(0); hi = xu.max(0)
            P("     PER UNIT: " + "  ".join(f"unit {u} {med[j]:.4f} [{lo[j]:+.4f}..{hi[j]:+.4f}]" for j, u in enumerate(UNITS))
              + f"   chi2/dof median {np.median(ch[:,2]):.1f}/{int(ch[0,3])}")
            P("     => FLIGHT J1..J4 (units 12, 11, 14, 13): " +
              ", ".join(f"{med[UNITS.index(u)]:.4f}" for u in IB.FLIGHT_UNIT))

    xs = fits[("sines + j3-row moves", "hhT")][0]
    Jcal = float(np.median(xs))
    P("\n" + "=" * 100)
    P(f"WHAT J_arm {IB.JA:.4f} -> {Jcal:.4f} DOES TO THE LAW (h h^T kept; floating base; K_y 20, M_y 1/1/1/0.05, DLS 0.3)")
    P("=" * 100)
    for qd in ([0, 40, 40, 0], [0, 25, 30, 0]):
        P(f"  q = {qd} deg")
        for J in (IB.JA, Jcal):
            t = law_terms(J, qd)
            P(f"     J_arm {J:.4f}: M_rho diag {np.round(t['Mr'],4)}  j2/j3 eig {np.round(t['Mr_eig'],4)} | "
              f"Kq diag {np.round(t['Kq'],3)} j2/j3 eig {np.round(t['Kq_eig'],3)} N.m/rad | "
              f"M_r diag {np.round(t['M_r'],4)} | N1 x-row {np.round(t['N1x'],3)}")

    # ---------- figure: measured J_row vs the law's prediction ----------
    fig, ax = plt.subplots(figsize=(11, 5.2))
    labels = []; xpos = 0
    for sess in ("s0909", "s0911"):
        for k in range(4):
            sv, mv = summary[(sess, k)]
            C = [W["C"] for W in Wall if W["D"]["sess"] == sess and W["k"] == k][0]
            cr = coef_row(sess, k, C, "hhT").sum()
            ex = (sess, k) in EXCLUDE
            ax.errorbar(xpos - 0.12, np.median(sv), yerr=[[np.median(sv) - sv.min()], [sv.max() - np.median(sv)]],
                        fmt="o", color="0.6" if ex else "C0", capsize=4, label="sines" if xpos == 0 else None)
            if mv is not None:
                ax.errorbar(xpos + 0.12, np.median(mv), yerr=[[np.median(mv) - mv.min()], [mv.max() - np.median(mv)]],
                            fmt="s", color="C1", capsize=4, label="fast moves" if xpos == 1 else None)
            ax.hlines(cr * IB.JA, xpos - 0.35, xpos + 0.35, colors="C3", linestyles="--",
                      label="law now, J_arm 0.020" if xpos == 0 else None)
            ax.hlines(cr * Jcal, xpos - 0.35, xpos + 0.35, colors="C2",
                      label=f"law calibrated, J_arm {Jcal:.4f}" if xpos == 0 else None)
            labels.append(f"{sess[1:]}\nslot j{k+1}\nunit {IB.SLOT_UNIT[sess][k]}" + ("\n(excl.)" if ex else ""))
            xpos += 1
    ax.set_xticks(range(len(labels))); ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("armature on the moving joint's row [kg m$^2$]")
    ax.set_title("Bench-identified armature per joint row vs the law's J_arm h h$^T$ prediction "
                 "(j2 row = J2 + J3 in the law's structure)", fontsize=10)
    ax.axhline(0, color="k", lw=0.5); ax.grid(alpha=0.3); ax.legend(fontsize=8, loc="upper right")
    plt.tight_layout(); plt.savefig(os.path.join(OUTD, "jarm_bench.png"), dpi=110)
    open(os.path.join(OUTD, "results.txt"), "w").write(out.getvalue())
