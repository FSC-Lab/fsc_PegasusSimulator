#!/usr/bin/env python3
"""Calibrate the whole-body law's J_arm from the arm's GROUND-BENCH bags only.

    PYTHONNOUSERSITE=1 /usr/bin/python3 calib_jarm.py [scan]

THE MODEL BEING CALIBRATED (controller.make_params:236-239, wb_model.cpp:135-140, parity-locked):
    I_i_i[link i] += J_arm * h_i h_i^T          (h_i = joint i axis in the child link's frame)
One scalar, currently 353.5^2 * 1.6e-7 = 0.0200 kg m^2, on every arm link. Because j2/j3 are
parallel and the wrist roll is perpendicular to both, a single moving joint k sees, on its own
row, the armature Sum_i c_ik J_i with c_ik = [M(J_i = 1) - M(links)]_kk at the window pose:
    j1 row: J_1 + cos^2(q2+q3) J_4      j2 row: J_2 + J_3      j3 row: J_3      j4 row: J_4
(the j2 row carries link 3's armature too -- the h h^T structure). A joint-DIAGONAL armature
(one rotor per joint, the physical structure) would instead give c_ik = delta_ik. Both are fitted.

MEASUREMENT per (session, slot) = the armature coefficient on the moving joint's row, J_row:
  SINES (primary). Kinetic plateau of every single-joint sine window (|qdot| > vfrac x peak):
      y = tau_meas - [M_links qdd + C qdot + S g]              (tau_meas = Present Current / kappa_unit)
      y = J_row qdd + K_g (q-c) + Sum_w [a + F sgn(qdot) + b qdot + s sgn(qdot)(q-c)]_w
    K_g is shared per (pose, amplitude class): J_row comes ONLY from exciting the same pose and
    amplitude at 0.133 and 0.266 Hz (inertia ~ omega^2, any rate-independent stiffness is not).
  MOVES (independent check). The 1.3-1.4 s min-jerk repositioning moves between the sine poses
    (20-25 deg at 25-30 deg/s, |qdd| up to ~1.3 rad/s^2, 2-5x the sines), kinetic part only:
      y = J_row qdd + Sum_w [a + b qdot]_w          (gravity from the model, S as calibrated)

UNITS: link i's armature belongs to the SERVO in slot i. Session 0909 slots j1..j4 = units 11,12,13,14;
session 0911 and FLIGHT = 12,11,14,13. A per-unit fit therefore lands directly on the flight joints.
"""
import copy, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ident_bench as IB
import ident_plateau as IPL
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT

HERE = os.path.dirname(os.path.abspath(__file__))
n = IB.n
JA_NOW = 353.5 ** 2 * 1.6e-7


def unit_columns(q):
    """c_ik: diagonal of M(J_i = 1 on link i, h h^T) - M(links), fixed base, at pose q."""
    X = np.zeros(18 + 2 * n); X[3:12] = np.eye(3).reshape(9, order="F"); X[12:12 + n] = q
    M0 = CT.dynamics(X, IB.PL)["M"][6:, 6:]
    C = np.zeros((4, 4))
    for i in range(n):
        Pi = copy.deepcopy(IB.PL)
        h = np.asarray(IB.P["h_i_im1"][i], float)
        Pi["I_i_i"][i + 1] = np.asarray(IB.PL["I_i_i"][i + 1]) + np.outer(h, h)
        C[i] = np.diag(CT.dynamics(X, Pi)["M"][6:, 6:] - M0)       # C[i, k] = c_ik
    return C


def move_windows(Ds, vmin_dps=15.0):
    out = []
    for D in Ds:
        if "coupling" in D["name"]:
            continue                                     # duplicates the eval bags' moves
        for (k, i0, i1, kd) in D["wins"]:
            if kd != "move":
                continue
            s = D["sel"][(D["sel"] >= i0) & (D["sel"] < i1)]
            qd = D["qd"][s, k]
            if np.degrees(np.abs(qd).max()) < vmin_dps:
                continue
            out.append(dict(D=D, k=k, s=s, q_med=np.median(D["q"][s], axis=0)))
    return out


def fit_moves(WW, vfrac=0.3, dS=0.0):
    blocks = []
    for w, W in enumerate(WW):
        D, k, s = W["D"], W["k"], W["s"]
        qd = D["qd"][s, k]; m = np.abs(qd) > vfrac * np.abs(qd).max(); s = s[m]
        # gravity-scale sensitivity: y -> y - dS * g_model (dS = relative error in S)
        gk = (D["tl"][s, k] - D["mjj"][s, k] * D["qdd"][s, k])       # ~ S g (C = 0 on own row)
        y = D["y"][s, k] - dS * gk
        A = np.zeros((len(s), 1 + 2 * len(WW)))
        A[:, 0] = D["qdd"][s, k]; A[:, 1 + 2 * w] = 1.0; A[:, 2 + 2 * w] = D["qd"][s, k]
        blocks.append((A, y))
    A = np.vstack([b[0] for b in blocks]); y = np.concatenate([b[1] for b in blocks])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    Js = []
    for w in range(len(WW)):
        keep = np.ones(A.shape[1], bool); keep[1 + 2 * w: 3 + 2 * w] = False
        rows = np.ones(len(y), bool); o = 0
        for ww, (Ab, _) in enumerate(blocks):
            if ww == w:
                rows[o:o + len(Ab)] = False
            o += len(Ab)
        cc, *_ = np.linalg.lstsq(A[rows][:, keep], y[rows], rcond=None)
        Js.append(cc[0])
    Js = np.array(Js); mm = len(Js)
    se = np.sqrt((mm - 1) / mm * np.sum((Js - Js.mean()) ** 2)) if mm > 2 else np.nan
    return c[0], se, len(y), np.percentile(np.abs(A[:, 0]), 95)


def global_fit(meas, structure, per_unit):
    """meas: list of (sess, k, J_row, se, C(4x4)). Weighted LS for J_unit (4) or J (1)."""
    units = (11, 12, 13, 14)
    rows, ys, ws = [], [], []
    for (sess, k, Jr, se, C) in meas:
        slot_unit = IB.SLOT_UNIT[sess]
        coef = np.zeros(4)
        for i in range(4):
            cik = C[i, k] if structure == "hhT" else float(i == k)
            coef[units.index(slot_unit[i])] += cik
        rows.append(coef if per_unit else [coef.sum()]); ys.append(Jr); ws.append(1 / se)
    A = np.array(rows) * np.array(ws)[:, None]; y = np.array(ys) * np.array(ws)
    x, *_ = np.linalg.lstsq(A, y, rcond=None)
    r = y - A @ x; chi2 = r @ r; dof = len(y) - len(x)
    cov = np.linalg.pinv(A.T @ A)
    return x, np.sqrt(np.diag(cov)), chi2, dof, r


if __name__ == "__main__":
    scan = len(sys.argv) > 1 and sys.argv[1] == "scan"
    Ds = IB.prepare_all(2.0)
    Wall = IPL.sine_windows(Ds)
    Cpose = {}
    for W in Wall:
        W["C"] = Cpose.setdefault(W["pose"], unit_columns(np.median(W["D"]["q"][W["s"]], axis=0)))

    def sine_meas(vfrac, delay, exclude_pre_tighten=False):
        IPL.VFRAC = vfrac
        out = []
        for sess in ("s0909", "s0911"):
            for k in range(4):
                WW = [W for W in Wall if W["D"]["sess"] == sess and W["k"] == k]
                if exclude_pre_tighten and sess == "s0911" and k == 0:
                    WW = [W for W in WW if W["D"]["t0"] + W["D"]["t"][W["i0"]] > Wall_t_screw]
                if len(WW) < 2:
                    continue
                J, se, N, rs, Kg = IPL.fit(WW, delay)
                out.append((sess, k, J, se, WW[0]["C"], len(WW), N))
        return out

    # 0911 joint-1 screws were loose until the "joint1 resolution" (README); the last j1 window
    # before eval_final (19:04:56) is taken as the boundary -- eval_final and later are clean.
    Wall_t_screw = min(D["t0"] for D in Ds if D["name"] == "s0911__eval_final_10") - 1.0

    print("C = c_ik at the sine poses (row i = link armature, col k = joint row), h h^T structure:")
    for key, C in Cpose.items():
        print(f"   {key}: " + "  ".join(f"row j{k+1}: " + "+".join(f"{C[i,k]:.2f}J{i+1}" for i in range(4) if C[i, k] > 0.005)
                                        for k in range(4) if k == key[1]))

    base = sine_meas(0.5, 0)
    print("\n==== SINES, per (session, slot): J_row = armature coefficient on the moving joint's row ====")
    for (sess, k, J, se, C, nw, N) in base:
        u = IB.SLOT_UNIT[sess][k]
        print(f"  {sess} slot j{k+1} (unit {u}): J_row = {J:.4f} +/- {se:.4f}   ({nw} windows, {N} samples)")

    if scan:
        print("\n==== robustness: J_row vs kinetic threshold / current delay / low-pass ====")
        hdr = []
        cols = []
        for vf in (0.3, 0.5, 0.7, 0.8):
            for dl in (0, 2):
                cols.append((f"v{vf} d{dl*4}ms", sine_meas(vf, dl)))
        print("  " + " " * 22 + "".join(f"{c[0]:>14s}" for c in cols))
        for r, (sess, k, *_rest) in enumerate(base):
            print(f"  {sess} slot j{k+1} (u{IB.SLOT_UNIT[sess][k]})".ljust(24) +
                  "".join(f"{c[1][r][2]:>14.4f}" for c in cols))

    print("\n==== MOVES (independent: 2-5x the acceleration, one direction, gravity from the model) ====")
    MW = move_windows(Ds)
    mv_meas = {}
    for sess in ("s0909", "s0911"):
        for k in range(4):
            WW = [W for W in MW if W["D"]["sess"] == sess and W["k"] == k]
            if len(WW) < 2:
                continue
            J, se, N, q95 = fit_moves(WW)
            Jp, *_ = fit_moves(WW, dS=+0.05); Jm, *_ = fit_moves(WW, dS=-0.05)
            J3, *_ = fit_moves(WW, vfrac=0.5)
            mv_meas[(sess, k)] = (J, se)
            print(f"  {sess} slot j{k+1} (unit {IB.SLOT_UNIT[sess][k]}): J_row = {J:.4f} +/- {se:.4f}  "
                  f"({len(WW)} moves, {N} samples, |qdd|95 {q95:.2f} rad/s^2)   gravity +/-5 %: {Jm:.4f}/{Jp:.4f}   vfrac 0.5: {J3:.4f}")

    print("\n==== THE LAW'S J_arm: global weighted fits over the SINE measurements ====")
    for excl in (False, True):
        meas = [(s, k, J, se, C) for (s, k, J, se, C, nw, N) in sine_meas(0.5, 0, exclude_pre_tighten=excl)]
        tag = "0911 j1 only AFTER the screw fix" if excl else "all windows"
        print(f"\n  -- {tag}: {len(meas)} slot measurements --")
        for structure in ("hhT", "diag"):
            for per_unit in (False, True):
                x, sx, chi2, dof, r = global_fit(meas, structure, per_unit)
                name = f"{'h h^T (the law)' if structure == 'hhT' else 'joint-diagonal'}, {'per unit' if per_unit else 'one scalar'}"
                if per_unit:
                    vals = "  ".join(f"unit {u}: {x[j]:.4f}+/-{sx[j]:.4f}" for j, u in enumerate((11, 12, 13, 14)))
                    fl = "  => FLIGHT J1..J4 (units 12,11,14,13): " + ", ".join(
                        f"{x[(11, 12, 13, 14).index(u)]:.4f}" for u in IB.FLIGHT_UNIT)
                else:
                    vals = f"J_arm = {x[0]:.4f} +/- {sx[0]:.4f}"; fl = ""
                print(f"    {name:34s} {vals}   chi2/dof {chi2:.1f}/{dof}{fl}")
                if not per_unit:
                    print("        normalized residuals: " + ", ".join(
                        f"{m[0][1:]} j{m[1]+1} {rr:+.1f}" for m, rr in zip(meas, r)))
