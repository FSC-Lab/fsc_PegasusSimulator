#!/usr/bin/env python3
"""J_row on j2/j3 from the EMPTY-claw lever sweeps (min-jerk passes, 72-108 deg at 30 deg/s,
both directions, 3 wrist angles, 3-4 lever geometries per slot, ~6 cycles per block).

    PYTHONNOUSERSITE=1 /usr/bin/python3 calib_lever.py

Per pass p (one direction, onset transient cut by travel distance d_min):
    y = tau_meas - [M_links qdd + C qdot + S g]   - efficiency (+/- mu M_links qdd, lifting/lowering)
    y = J qdd + G_b(q) + a_p + f1_p |qdot| + f2_p qdot^2
G_b is a polynomial in the swept angle SHARED by both directions of block b (gravity-model
error is direction-free); up and down passes share q-dependence, so G_b cannot absorb the
friction, and the min-jerk acceleration is concentrated in the first/last ~7 % of travel, a
shape a low-order polynomial over the whole arc cannot mimic. Both the degree of G_b and
d_min are scanned.
"""
import glob, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ident_bench as IB
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT

MU = np.array([0.0, 0.246, 0.161, 0.0])
n = IB.n
CACHE = os.path.join(IB.HERE, "..", "prepared_lever_fc2.npy")


def prepare(fn):
    d = dict(np.load(fn))
    th = d["js_t"]; t = np.arange(th[0], th[-1], 1.0 / IB.FS)
    q = np.column_stack([np.interp(t, th, d["js_q"][:, k]) for k in range(4)])
    cur = np.column_stack([np.interp(t, th, d["js_i"][:, k]) for k in range(4)])
    qf = IB.lp(q, 2.0); qd = np.gradient(qf, 1 / IB.FS, axis=0); qdd = np.gradient(qd, 1 / IB.FS, axis=0)
    sess = os.path.basename(fn).split("__")[0]
    kap = np.array([IB.KAPPA_UNIT[u] for u in IB.SLOT_UNIT[sess]])
    tau = IB.lp(cur, 2.0) / kap
    k = int(np.argmax(np.ptp(q, axis=0)))
    moving = np.abs(qd[:, k]) > np.radians(2.0)
    sel = np.where(moving)[0][::IB.DEC]
    tl = np.full_like(q, np.nan); mjj = np.full_like(q, np.nan)
    X = np.zeros(18 + 2 * n); X[3:12] = np.eye(3).reshape(9, order="F")
    for i in sel:
        X[12:12 + n] = qf[i]; X[18 + n:18 + 2 * n] = qd[i]
        dy = CT.dynamics(X, IB.PL); M = dy["M"]; C = dy["C"]
        V = np.concatenate([np.zeros(6), qd[i]])
        tl[i] = M[6:, 6:] @ qdd[i] + (C @ V)[6:] + IB.S_G * (M[6:, 0:3] @ IB.G_BENCH)
        mjj[i] = np.diag(M[6:, 6:])
    # passes: runs of the swept joint moving in one direction, others still
    passes = []
    idx = np.where(moving)[0]
    for run in np.split(idx, np.where(np.diff(idx) > 5)[0] + 1):
        if len(run) < 250:
            continue
        others = [np.ptp(q[run, j]) for j in range(4) if j != k]
        if max(others) > np.radians(1.0) or np.ptp(q[run, k]) < np.radians(30):
            continue
        if not np.all(np.sign(qd[run, k]) == np.sign(qd[run[len(run) // 2], k])):
            continue
        passes.append(run)
    return dict(name=os.path.basename(fn)[:-4], sess=sess, k=k, q=qf, qd=qd, qdd=qdd, tau=tau, tl=tl,
                y=tau - tl, mjj=mjj, sel=sel, passes=passes)


def load():
    if os.path.exists(CACHE):
        return list(np.load(CACHE, allow_pickle=True))
    Ds = [prepare(f) for f in sorted(glob.glob(os.path.join(IB.DATA, "*collect_kt_*empty*.npz")))]
    np.save(CACHE, np.array(Ds, dtype=object), allow_pickle=True)
    return Ds


def fit(Ds, sess, k, dmin, deg, vmin_frac=0.0, jack=True):
    Bs = [D for D in Ds if D["sess"] == sess and D["k"] == k]

    def solve(blocks):
        cols = []; ys = []; npass = sum(len(D["passes"]) for D in blocks)
        ncol = 1 + deg * len(blocks) + 3 * npass; p0 = 0
        for b, D in enumerate(blocks):
            qc = np.median(D["q"][:, k])
            for run in D["passes"]:
                s = np.intersect1d(run, D["sel"])
                q = D["q"][s, k]; qd = D["qd"][s, k]
                m = (np.abs(q - D["q"][run[0], k]) > np.radians(dmin)) & (np.abs(qd) > vmin_frac * np.abs(qd).max())
                s = s[m]; q = D["q"][s, k]; qd = D["qd"][s, k]; qdd = D["qdd"][s, k]
                gk = D["tl"][s, k] - D["mjj"][s, k] * qdd
                lift = np.sign(qd) == np.sign(gk)
                y = D["y"][s, k] - MU[k] * D["mjj"][s, k] * qdd * np.where(lift, 1.0, -1.0)
                A = np.zeros((len(s), ncol))
                A[:, 0] = qdd
                for p in range(deg):
                    A[:, 1 + deg * b + p] = (q - qc) ** (p + 1)
                c0 = 1 + deg * len(blocks) + 3 * p0
                A[:, c0:c0 + 3] = np.column_stack([np.ones_like(qd), np.abs(qd), qd ** 2])
                cols.append(A); ys.append(y); p0 += 1
        A = np.vstack(cols); y = np.concatenate(ys)
        c, *_ = np.linalg.lstsq(A, y, rcond=None)
        return c[0], len(y), np.percentile(np.abs(A[:, 0]), 95), (y - A @ c).std()
    J, N, q95, rs = solve(Bs)
    se = np.nan
    if jack and len(Bs) > 2:
        Js = np.array([solve(Bs[:b] + Bs[b + 1:])[0] for b in range(len(Bs))])
        m = len(Js); se = np.sqrt((m - 1) / m * np.sum((Js - Js.mean()) ** 2))
    return J, se, N, q95, rs, len(Bs), sum(len(D["passes"]) for D in Bs)


if __name__ == "__main__":
    Ds = load()
    for D in Ds:
        print(f"  {D['name']:40s} swept j{D['k']+1}, {len(D['passes'])} passes")
    print("\nJ_row [kg m^2] (block-jackknife se x1e4), rows: d_min [deg], cols: gravity-error poly degree")
    for sess in ("s0909", "s0911"):
        for k in (1, 2):
            J, se, N, q95, rs, nb, npas = fit(Ds, sess, k, 5.0, 3)
            print(f"\n  {sess} slot j{k+1} (unit {IB.SLOT_UNIT[sess][k]}): {nb} blocks, {npas} passes, "
                  f"{N} samples, |qdd|95 {q95:.2f} rad/s^2, resid {rs*1e3:.1f} mN.m")
            for dmin in (0.0, 3.0, 5.0, 8.0):
                print(f"     d_min {dmin:3.0f}: " + "  ".join(
                    "deg {}: {:+.4f}({:3.0f})".format(dg, *[(v if i == 0 else v * 1e4) for i, v in enumerate(fit(Ds, sess, k, dmin, dg)[:2])])
                    for dg in (0, 2, 3, 5)))
