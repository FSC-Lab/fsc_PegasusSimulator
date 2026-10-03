#!/usr/bin/env python3
"""Armature from the KINETIC PLATEAUS of the bench sine windows (ident_bench.py's estimators
fail: friction is ~10x the inertial torque and its stick transitions at every reversal leak
into the in-phase component differently at the two speeds).

    PYTHONNOUSERSITE=1 /usr/bin/python3 ident_plateau.py [fc_hz] [vfrac] [delay_ms]

Only samples with |qdot| > vfrac * peak are used, i.e. the middle of each half-cycle, where
the joint slides and friction is a plateau. On those samples, per (session, slot):

    y = tau_meas - [M_links qdd + C qdot + S g(q)]           (whole-body model, links only)
    y = J qdd + K_g (q - c)                                    shared per (pose, amplitude) group
        + sum_windows [ a_w + F_w sgn(qdot) + b_w qdot + s_w sgn(qdot)(q - c) ]
          (offset,  Coulomb plateau, viscous,  load-dependent friction -- all per window)

J (armature) and K_g (rate-independent stiffness: gravity-model error, cable spring,
presliding) are both in-phase with q; they separate ONLY because each group was excited at two
frequencies. sgn(qdot)(q - c) flips between half-cycles and qdot is symmetric inside one, so
neither can mimic J. Windows recorded twice (the 76-min coupling_j1j3 bag also captured the
eval runs) are de-duplicated by absolute time. Error bar = leave-one-window-out jackknife.
"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ident_bench as IB

FC = (float(sys.argv[1]) if __name__ == '__main__' and len(sys.argv) > 1 else 2.0)
VFRAC = (float(sys.argv[2]) if __name__ == '__main__' and len(sys.argv) > 2 else 0.5)
DELAY_MS = (float(sys.argv[3]) if __name__ == '__main__' and len(sys.argv) > 3 else 0.0)


def sine_windows(Ds):
    """(D, k, i0, i1, amp, om, c, pose) for every sine window, de-duplicated by absolute time."""
    out, seen = [], []
    for D in sorted(Ds, key=lambda D: ("coupling" in D["name"], D["name"])):   # eval bags win
        for (k, i0, i1, kd) in D["wins"]:
            if kd != "sine":
                continue
            ta = D["t0"] + D["t"][i0]
            if any(s == D["sess"] and kk == k and abs(ta - tb) < 5.0 for (s, kk, tb) in seen):
                continue
            seen.append((D["sess"], k, ta))
            s = D["sel"][(D["sel"] >= i0) & (D["sel"] < i1)]
            qj = D["q"][s, k]; c = np.median(qj); x = qj - c; t = D["t"][s]
            # fundamental by a frequency grid search (robust to dither at the crossings)
            best = None
            for f in np.linspace(0.08, 0.40, 641):
                B = np.column_stack([np.cos(2 * np.pi * f * t), np.sin(2 * np.pi * f * t), np.ones_like(t)])
                cc, res, *_ = np.linalg.lstsq(B, x, rcond=None)
                r = np.sum((x - B @ cc) ** 2)
                if best is None or r < best[0]:
                    best = (r, f, np.hypot(cc[0], cc[1]))
            _, f, amp = best
            others = np.delete(D["q"][s].mean(0), k)
            pose = (D["sess"], k, tuple(np.round(np.degrees(others)).astype(int)), int(round(np.degrees(c))))
            out.append(dict(D=D, k=k, i0=i0, i1=i1, s=s, amp=amp, om=2 * np.pi * f, c=c, pose=pose))
    return out


def rows_for(W, delay_samples):
    D, k, s = W["D"], W["k"], W["s"]
    s = s[(s + delay_samples >= 0) & (s + delay_samples < len(D["t"]))]
    qd = D["qd"][s, k]; vpk = np.percentile(np.abs(qd), 98)
    m = np.abs(qd) > VFRAC * vpk
    # drop the amplitude ramps: keep the time between the first and last 10 % of the window
    t = D["t"][s]; m &= (t > t[0] + 0.1 * (t[-1] - t[0])) & (t < t[-1] - 0.1 * (t[-1] - t[0]))
    s = s[m]
    # current delayed relative to the encoder by delay_samples (tau(t + d) pairs with q(t))
    y = D["tau"][s + delay_samples, k] - D["tl"][s, k]
    return s, y


def fit(Wlist, delay_samples=0, jack=True):
    groups = {}
    for W in Wlist:
        key = W["pose"] + (int(round(np.degrees(W["amp"]) / 3.0)),)     # amplitude class, 3 deg bins
        W["gkey"] = groups.setdefault(key, len(groups))
    nG = len(groups)

    def solve(WW):
        blocks = []
        for w, W in enumerate(WW):
            D, k = W["D"], W["k"]
            s, y = rows_for(W, delay_samples)
            dq = D["q"][s, k] - W["c"]; qd = D["qd"][s, k]; sg = np.sign(qd)
            A = np.zeros((len(s), 1 + nG + 4 * len(WW)))
            A[:, 0] = D["qdd"][s, k]
            A[:, 1 + W["gkey"]] = dq
            b0 = 1 + nG + 4 * w
            A[:, b0:b0 + 4] = np.column_stack([np.ones_like(dq), sg, qd, sg * dq])
            blocks.append((A, y))
        A = np.vstack([b[0] for b in blocks]); y = np.concatenate([b[1] for b in blocks])
        used = np.any(A != 0, axis=0)
        c = np.zeros(A.shape[1])
        c[used], *_ = np.linalg.lstsq(A[:, used], y, rcond=None)
        return c, A, y

    c, A, y = solve(Wlist)
    r = y - A @ c
    se = np.nan
    if jack and len(Wlist) > 2:
        Js = np.array([solve(Wlist[:w] + Wlist[w + 1:])[0][0] for w in range(len(Wlist))])
        m = len(Js); se = np.sqrt((m - 1) / m * np.sum((Js - Js.mean()) ** 2))
    return c[0], se, len(y), r.std(), c[1:1 + nG]


def per_window_k(W, delay_samples=0):
    """in-phase tilt of the plateau without J: y = a + F sgn + b qd + s sgn dq + k dq; and the
    effective omega^2 = -(qdd regressed on dq) on the same samples."""
    D, k = W["D"], W["k"]
    s, y = rows_for(W, delay_samples)
    dq = D["q"][s, k] - W["c"]; qd = D["qd"][s, k]; sg = np.sign(qd)
    B = np.column_stack([np.ones_like(dq), sg, qd, sg * dq, dq])
    cy, *_ = np.linalg.lstsq(B, y, rcond=None)
    cq, *_ = np.linalg.lstsq(B, D["qdd"][s, k], rcond=None)
    ct, *_ = np.linalg.lstsq(B, D["tau"][s + delay_samples, k], rcond=None)
    return cy[4], -cq[4], ct[4], cy[1], len(s)


if __name__ == "__main__":
    Ds = IB.prepare_all(FC)
    Wall = sine_windows(Ds)
    dsm = int(round(DELAY_MS / 4.0))
    print(f"low-pass {FC:g} Hz, kinetic |qdot| > {VFRAC:g} x peak, current delay {dsm*4} ms; "
          f"{len(Wall)} unique sine windows")
    res = {}
    for sess in ("s0909", "s0911"):
        for k in range(4):
            unit = IB.SLOT_UNIT[sess][k]
            WW = [W for W in Wall if W["D"]["sess"] == sess and W["k"] == k]
            if len(WW) < 2:
                continue
            print(f"\n  {sess} slot j{k+1} = unit {unit}: {len(WW)} windows")
            for W in WW:
                ky, om2, kt, F, N = per_window_k(W, dsm)
                W["ky"], W["om2"] = ky, om2
                print(f"     {W['D']['name'].split('__')[1]:22s} t {W['D']['t'][W['i0']]:6.0f}s  c {np.degrees(W['c']):5.1f}  "
                      f"A {np.degrees(W['amp']):4.1f}  f {W['om']/2/np.pi:.3f} Hz  om2_eff {om2:5.2f}  "
                      f"k_y {ky*1e3:+7.1f}  k_tau {kt*1e3:+7.1f} mN.m/rad  plateau F {F*1e3:5.1f} mN.m  N {N}")
            J, se, N, rs, Kg = fit(WW, dsm)
            mj = np.nanmean([np.nanmean(W["D"]["mjj"][W["s"], k]) for W in WW])
            # transparent pairs: same pose + amplitude class, omega ratio > 1.4
            pairs = []
            for a in WW:
                for b in WW:
                    if (a["pose"] == b["pose"] and abs(a["amp"] - b["amp"]) < np.radians(2.5)
                            and b["om2"] > 2.0 * a["om2"]):
                        pairs.append(-(b["ky"] - a["ky"]) / (b["om2"] - a["om2"]))
            pairs = np.array(pairs)
            ps = (f"pairs {len(pairs)}: median {np.median(pairs):+.4f} [{pairs.min():+.4f}..{pairs.max():+.4f}]"
                  if len(pairs) else "pairs 0")
            print(f"   => J = {J:+.4f} +/- {se:.4f} kg m^2   (M_links {mj:.4f}, M_tot {J+mj:.4f})   "
                  f"resid {rs*1e3:.1f} mN.m, N {N};  {ps};  K_g {np.round(Kg*1e3,1)} mN.m/rad")
            res[(sess, k)] = (J, se, mj, pairs)
    print("\n==== per UNIT, FLIGHT order (J1..J4 = units 12, 11, 14, 13) ====")
    for jj, u in enumerate(IB.FLIGHT_UNIT):
        parts = []
        for sess in ("s0909", "s0911"):
            k = IB.SLOT_UNIT[sess].index(u)
            if (sess, k) in res:
                J, se, mj, pr = res[(sess, k)]
                parts.append(f"{sess} slot j{k+1}: {J:+.4f} +/- {se:.4f}")
        print(f"  flight J{jj+1} = unit {u}:  " + "  |  ".join(parts))
