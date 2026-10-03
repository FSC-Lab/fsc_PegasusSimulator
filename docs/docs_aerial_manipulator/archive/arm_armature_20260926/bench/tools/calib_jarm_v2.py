#!/usr/bin/env python3
"""J_row per (session, slot) with the post-onset friction transient EXCLUDED BY TRAVEL DISTANCE.

    PYTHONNOUSERSITE=1 /usr/bin/python3 calib_jarm_v2.py

Why (see ../README.md, plateau_shapes figure): after every reversal or start the gear friction
is HIGH and decays over ~4-5 deg of travel (strongest on j3, +60 mN.m). In a one-direction move
that decay coincides with the acceleration phase and inflates J; in a sine it tilts the plateau
differently at the two frequencies. Here only samples more than d_min deg past the last
reversal / start are used, and friction gets a smooth speed law per window:
    F(qdot) = sgn(qdot) (f0 + f1 |qdot| + f2 qdot^2)
SINES: y = J qdd + K_group (q - c) + Sum_w [a + F_w(qdot) + s_w sgn(qdot)(q - c)]
MOVES: y = J qdd + Sum_w [a_w + f1_w |qdot| + f2_w qdot^2]     (+ the gearbox-efficiency
       correction: lowering reads J - mu M_links, lifting J + mu M_links, mu = friction_load_coeff)
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ident_bench as IB
import ident_plateau as IPL
import calib_jarm as CJ

MU = np.array([0.0, 0.246, 0.161, 0.0])      # friction_load_coeff (arm yaml / calibration report eq. 5)


def travel_since_reversal(q, qd):
    """|q - q at the last velocity sign change (or window start)| along the samples."""
    out = np.zeros_like(q); ref = q[0]; sg = np.sign(qd[0])
    for i in range(len(q)):
        s = np.sign(qd[i])
        if s != 0 and s != sg:
            sg = s; ref = q[i]
        out[i] = abs(q[i] - ref)
    return out


def sine_rows(W, dmin):
    D, k = W["D"], W["k"]
    s_all = np.arange(W["i0"], W["i1"])
    qd_all = D["qd"][s_all, k]
    trav = travel_since_reversal(D["q"][s_all, k], np.where(np.abs(qd_all) > np.radians(0.3), qd_all, 0.0))
    s = W["s"]                                             # 50 Hz analysis samples
    tr = np.interp(s, s_all, trav)
    t = D["t"][s]; T0, T1 = t[0], t[-1]
    m = (tr > np.radians(dmin)) & (np.abs(D["qd"][s, k]) > np.radians(1.0))
    m &= (t > T0 + 0.1 * (T1 - T0)) & (t < T1 - 0.1 * (T1 - T0))
    return s[m]


def fit_sines(WW, dmin, jack=True):
    groups = {}
    for W in WW:
        W["gkey"] = groups.setdefault(W["pose"] + (int(round(np.degrees(W["amp"]) / 3.0)),), len(groups))
    nG = len(groups); NW = 6

    def solve(Wl):
        blocks = []
        for w, W in enumerate(Wl):
            D, k = W["D"], W["k"]; s = sine_rows(W, dmin)
            dq = D["q"][s, k] - W["c"]; qd = D["qd"][s, k]; sg = np.sign(qd)
            A = np.zeros((len(s), 1 + nG + NW * len(Wl)))
            A[:, 0] = D["qdd"][s, k]; A[:, 1 + W["gkey"]] = dq
            b0 = 1 + nG + NW * w
            A[:, b0:b0 + NW] = np.column_stack([np.ones_like(dq), sg, qd, sg * qd ** 2, sg * dq, np.abs(qd)])
            blocks.append((A, D["y"][s, k]))
        A = np.vstack([b[0] for b in blocks]); y = np.concatenate([b[1] for b in blocks])
        used = np.any(A != 0, axis=0); c = np.zeros(A.shape[1])
        c[used], *_ = np.linalg.lstsq(A[:, used], y, rcond=None)
        return c[0], len(y)
    J, N = solve(WW)
    se = np.nan
    if jack and len(WW) > 2:
        Js = np.array([solve(WW[:w] + WW[w + 1:])[0] for w in range(len(WW))])
        m = len(Js); se = np.sqrt((m - 1) / m * np.sum((Js - Js.mean()) ** 2))
    return J, se, N


def fit_moves(WW, dmin, mu_scale=1.0):
    blocks = []
    for w, W in enumerate(WW):
        D, k, s = W["D"], W["k"], W["s"]
        q = D["q"][s, k]; qd = D["qd"][s, k]
        m = (np.abs(q - q[0]) > np.radians(dmin)) & (np.abs(qd) > np.radians(2.0))
        s = s[m]; qd = D["qd"][s, k]
        A = np.zeros((len(s), 1 + 3 * len(WW)))
        A[:, 0] = D["qdd"][s, k]
        A[:, 1 + 3 * w: 4 + 3 * w] = np.column_stack([np.ones_like(qd), np.abs(qd), qd ** 2])
        # efficiency: lifting (qdot along S g) reads +mu M_links qdd, lowering -mu M_links qdd
        # the calibration report's gearbox law: friction = sgn(qdot) (fc + mu |tau_transmitted|),
        # tau_transmitted = S g + M_links qdd (the rotor's own inertia torque is NOT transmitted).
        # fc goes into the per-move constant; the mu part varies with q AND qdd inside a move, so it
        # is removed explicitly (mu_scale scans the report's mu).
        ttx = D["tl"][s, k]                                  # = S g + M_links qdd (+ C = 0 on own row)
        corr = mu_scale * MU[k] * np.sign(qd) * np.abs(ttx)
        blocks.append((A, D["y"][s, k] - corr, W))
    A = np.vstack([b[0] for b in blocks]); y = np.concatenate([b[1] for b in blocks])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    Js = []
    for w in range(len(blocks)):
        keep = np.ones(A.shape[1], bool); keep[1 + 3 * w: 4 + 3 * w] = False
        rows = np.concatenate([np.full(len(b[0]), ww != w) for ww, b in enumerate(blocks)])
        cc, *_ = np.linalg.lstsq(A[rows][:, keep], y[rows], rcond=None); Js.append(cc[0])
    Js = np.array(Js); m = len(Js)
    se = np.sqrt((m - 1) / m * np.sum((Js - Js.mean()) ** 2)) if m > 2 else np.nan
    return c[0], se, len(y)


if __name__ == "__main__":
    Ds = IB.prepare_all(2.0)
    Wall = IPL.sine_windows(Ds)
    MW = CJ.move_windows(Ds)
    DMINS = (0.0, 3.0, 5.0, 7.0)
    print("J_row [kg m^2] vs d_min (deg of travel excluded after each reversal / start)\n")
    print("  " + " " * 26 + "".join(f"   sine d{d:.0f}" for d in DMINS) + "".join(f"   move d{d:.0f}" for d in DMINS))
    table = {}
    for sess in ("s0909", "s0911"):
        for k in range(4):
            WW = [W for W in Wall if W["D"]["sess"] == sess and W["k"] == k]
            MM = [W for W in MW if W["D"]["sess"] == sess and W["k"] == k]
            line = f"  {sess} slot j{k+1} (unit {IB.SLOT_UNIT[sess][k]})".ljust(28)
            for d in DMINS:
                J, se, N = fit_sines(WW, d) if len(WW) >= 2 else (np.nan, np.nan, 0)
                table[(sess, k, "sine", d)] = (J, se)
                line += f"  {J:+.4f}({se*1e4:3.0f})" if np.isfinite(J) else "        --   "
            for d in DMINS:
                if len(MM) >= 2:
                    J, se, N = fit_moves(MM, d)
                    table[(sess, k, "move", d)] = (J, se)
                    line += f"  {J:+.4f}({se*1e4:3.0f})"
                else:
                    line += "        --   "
            print(line)
    print("\n  (value(jackknife se x1e4)); moves carry the gearbox-efficiency correction mu*M_links")
    np.save(os.path.join(IB.HERE, "..", "jrow_table.npy"), table, allow_pickle=True)
