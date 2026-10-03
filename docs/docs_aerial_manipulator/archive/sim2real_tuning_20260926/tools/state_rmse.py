#!/usr/bin/env python3
"""Sim-to-real STATE agreement of one hardware flight and its Isaac replay.

    /usr/bin/python3 state_rmse.py <real.npz> <sim.npz> <tag>
        -> analysis/states_<tag>.json

Where compare.py sets each run's TRACKING statistics side by side, this file
measures how far the simulated STATE trajectory is from the flown one:
RMSE(sim - real) per state, over the same mission phases (the k-th planner leg
against the k-th leg, aligned at its start, compared over the overlap), on
PLANT time (the sim's wall clock scaled by its measured RTF).

States, the same processing on both sides:
  CoM position  x_c - x_cd(first DIRECT tick), i.e. relative to the mission
                anchor both runs capture at DIRECT entry [mm]
  CoM velocity  slope of x_c over a 0.2 s local linear fit [mm/s]
  attitude      roll / pitch from the odometry quaternion; yaw relative to
                the anchor heading [deg]
  body rate     sensor_combined gyro (FRD), 0.2 s window mean [deg/s] --
                the rigid-body band; propeller vibration (unmodelled in Isaac)
                is averaged out on purpose
  joints        q1..q4 from the law's own debug array [deg]; rates = 0.2 s
                slope [deg/s]
  inputs        u1 [N], rotor commands [0..1], joint torques [N.m]

ALIGNMENT, the same for every state:
  * move legs are TIME-WARPED onto the flight's plan time (the sim's plant-time
    duration differs by the RTF's drift during the leg, not by physics), and
    the sim's rate-type states are rescaled by the same factor;
  * --rigid FIT=A,B fits one rotation about z + translation of the sim's plan
    (x_cd) onto the flight's over phase FIT and applies it to phases A,B
    (a circle anchored at a different place/heading is the same plan);
  * a phase is COMPARED only if the two plans agree there (x_cd within
    REF_POS_TOL mm rms, q_d within REF_Q_TOL deg rms); the others are listed
    with their plan difference and left out of the RMSE;
  * --exclude PHASE=reason leaves a phase out for a stated reason (a
    feedback fault on the flight, e.g. a mocap dropout).
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import compare as CP  # noqa: E402

DT = 0.02          # comparison grid, 50 Hz
WIN_POS = 0.10     # window for levels
WIN_RATE = 0.20    # window for slopes / rates
REF_POS_TOL = 15.0 # mm rms, plan agreement for a phase to be compared
REF_Q_TOL = 1.0    # deg rms
RATE_KEYS = ("vel_", "rate_", "qd")


def _cs(t, *ys):
    return [np.concatenate([[0.0], np.cumsum(y)]) for y in (np.ones_like(t), t, *ys)]


def win_mean(t, y, g, w):
    """Mean of y over samples with |t - g| <= w/2 (uneven sampling safe)."""
    y = np.asarray(y, float)
    ok = np.isfinite(y)
    t, y = t[ok], y[ok]
    a = np.searchsorted(t, g - w / 2); b = np.searchsorted(t, g + w / 2, side="right")
    c = np.concatenate([[0.0], np.cumsum(y)])
    n = (b - a).astype(float)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(n > 0, (c[b] - c[a]) / n, np.nan)


def win_slope(t, y, g, w):
    """Least-squares slope of y(t) over each window (a local linear fit)."""
    y = np.asarray(y, float)
    ok = np.isfinite(y)
    t, y = t[ok], y[ok]
    a = np.searchsorted(t, g - w / 2); b = np.searchsorted(t, g + w / 2, side="right")
    tc = t - t[0]
    S = [np.concatenate([[0.0], np.cumsum(v)]) for v in (np.ones_like(tc), tc, y, tc * tc, tc * y)]
    n, st, sy, stt, sty = [s[b] - s[a] for s in S]
    den = n * stt - st * st
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where((n > 2) & (den > 1e-12), (n * sty - st * sy) / den, np.nan)


def load(path, sim):
    r = CP.load(path, sim)
    d = np.load(path, allow_pickle=True)
    rtf = r["rtf"]
    t0 = d["wb__recv"][0]
    T = lambda key: (d[f"{key}__recv"] - t0) * rtf  # noqa: E731
    D, t = r["D"], r["t"]
    idx = np.where(r["direct"])[0]
    r["anchor"] = D[idx[0], 45:48].copy()
    q = np.column_stack([d[f"odom__pose.pose.orientation.{k}"] for k in "wxyz"])
    w, x, y, z = q.T
    r["roll"] = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    r["pitch"] = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    r["yaw_u"] = np.degrees(np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))
    k0 = np.searchsorted(r["t_odom"], t[idx[0]])
    r["yaw0"] = r["yaw_u"][min(k0, len(r["yaw_u"]) - 1)]
    r["t_sc"] = T("sc")
    r["gyro"] = np.degrees(np.stack(d["sc__gyro_rad"]).astype(float))
    return r


def series(r):
    """name -> (time array, value array, 'level'|'slope', scale) on the raw samples."""
    t, D = r["t"], r["D"]
    xc = (D[:, 48:51] - r["anchor"]) * 1e3
    xcd = (D[:, 45:48] - r["anchor"]) * 1e3
    s = {}
    for k, ax in enumerate("xyz"):
        s[f"pos_{ax}"] = (t, xc[:, k], "level")
        s[f"vel_{ax}"] = (t, xc[:, k], "slope")
        s[f"ref_pos_{ax}"] = (t, xcd[:, k], "level")
    to = r["t_odom"]
    s["roll"] = (to, r["roll"], "level"); s["pitch"] = (to, r["pitch"], "level")
    s["yaw"] = (to, r["yaw_u"] - r["yaw0"], "level")
    for k, ax in enumerate(("p", "q", "r")):
        s[f"rate_{ax}"] = (r["t_sc"], r["gyro"][:, k], "rate")
    for j in range(4):
        s[f"q{j+1}"] = (t, np.degrees(D[:, 5 + j]), "level")
        s[f"qd{j+1}"] = (t, np.degrees(D[:, 5 + j]), "slope")
        s[f"ref_q{j+1}"] = (t, np.degrees(D[:, 9 + j]), "level")
    s["u1"] = (t, D[:, 17], "level")
    for m in range(4):
        s[f"mot{m+1}"] = (t, D[:, 41 + m], "level")
    s["tau2"] = (t, D[:, 14], "level"); s["tau3"] = (t, D[:, 15], "level")
    for k, ax in enumerate("xyz"):
        s[f"dhat_{ax}"] = (t, D[:, 31 + k], "level")
    return s


def sample(r, ser, name, ta, g, f=1.0, gg=None):
    """Value at phase-relative times gg (default g*f); f = the local warp rate
    d(sim time)/d(flight time), scalar or per sample; rates are returned per
    FLIGHT second (x f)."""
    tt, yy, kind = ser[name]
    gg = g * f if gg is None else gg
    if kind == "slope":
        return win_slope(tt - ta, yy, gg, WIN_RATE) * f
    if kind == "rate":
        return win_mean(tt - ta, yy, gg, WIN_RATE) * f
    return win_mean(tt - ta, yy, gg, WIN_POS)


def rigid_fit(a, b):
    """Rotation about z + translation taking points b (N x 2) onto a."""
    ma, mb = a.mean(0), b.mean(0)
    A, B = a - ma, b - mb
    th = np.arctan2(np.sum(B[:, 0] * A[:, 1] - B[:, 1] * A[:, 0]), np.sum(B[:, 0] * A[:, 0] + B[:, 1] * A[:, 1]))
    c, s_ = np.cos(th), np.sin(th)
    Rz = np.array([[c, -s_], [s_, c]])
    return Rz, ma - Rz @ mb, np.degrees(th)


STATES = ([f"pos_{a}" for a in "xyz"] + [f"vel_{a}" for a in "xyz"] + ["roll", "pitch", "yaw"]
          + [f"rate_{a}" for a in "pqr"] + [f"q{j}" for j in range(1, 5)] + [f"qd{j}" for j in range(1, 5)]
          + ["u1"] + [f"mot{m}" for m in range(1, 5)] + ["tau2", "tau3"])
REFS = [f"ref_pos_{a}" for a in "xyz"] + [f"ref_q{j}" for j in range(1, 5)]
TRACE = ["pos_x", "pos_y", "pos_z", "ref_pos_x", "ref_pos_y", "ref_pos_z", "roll", "pitch", "yaw",
         "q2", "q3", "ref_q2", "ref_q3", "u1", "dhat_x", "dhat_y", "dhat_z", "vel_x", "vel_y"]


def main():
    real_p, sim_p, tag = sys.argv[1], sys.argv[2], sys.argv[3]
    rigid = {}
    if "--rigid" in sys.argv:
        spec = sys.argv[sys.argv.index("--rigid") + 1]
        fit, apply_ = spec.split("=")
        rigid = {"fit": fit, "apply": apply_.split(",")}
    manual = {}
    if "--exclude" in sys.argv:
        for item in sys.argv[sys.argv.index("--exclude") + 1].split(";"):
            k, v = item.split("=", 1)
            manual[k] = v
    R, S = load(real_p, False), load(sim_p, True)
    PR, PS = CP.phases(R), CP.phases(S)
    SR, SS = series(R), series(S)
    names = [n for n, _, _ in PR if n in [x for x, _, _ in PS]]
    acc = {k: [] for k in STATES + REFS}
    spans = {k: [] for k in STATES}
    out = {"tag": tag, "sim_rtf": S["rtf"], "phases": {}, "trace_hz": 1 / DT / 2, "rigid": None, "excluded": []}
    win = {n: (next(x for x in PR if x[0] == n)[1:], next(x for x in PS if x[0] == n)[1:]) for n in names}

    def progress(r, ser, ta, L):
        """Normalised arc length of the plan's CoM path over [0, L] (rigid-motion invariant)."""
        h = np.arange(0.0, L, DT)
        P = np.column_stack([sample(r, ser, f"ref_pos_{ax}", ta, h) for ax in "xyz"])
        P = np.where(np.isfinite(P), P, 0.0)
        a = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
        return h, a

    def grid(n):
        """Flight grid g, the sim times gg it maps to, and the local rate factor."""
        (ra, rb), (sa, sb) = win[n]
        if not n.endswith("_move"):
            g = np.arange(0.0, min(rb - ra, sb - sa), DT)
            return g, g.copy(), np.ones_like(g)
        g = np.arange(0.0, rb - ra, DT)
        hr, ar = progress(R, SR, ra, rb - ra)
        hs, as_ = progress(S, SS, sa, sb - sa)
        if ar[-1] > 50.0 and as_[-1] > 50.0:          # the plan moves: warp by path progress
            ur = np.maximum.accumulate(ar / ar[-1]); us = np.maximum.accumulate(as_ / as_[-1])
            us_u, iu = np.unique(us, return_index=True)
            gg = np.interp(np.interp(g, hr, ur), us_u, hs[iu])
        else:                                         # it does not: linear
            gg = g * (sb - sa) / (rb - ra)
        k = max(1, int(round(1.0 / DT)))
        fr = np.gradient(gg, DT)
        fr = np.convolve(fr, np.ones(k) / k, mode="same")
        return g, gg, np.clip(fr, 0.2, 5.0)

    xf = None
    if rigid:
        n = rigid["fit"]; g, gg, f = grid(n); (ra, _), (sa, _) = win[n]
        a = np.column_stack([sample(R, SR, f"ref_pos_{ax}", ra, g) for ax in "xy"])
        b = np.column_stack([sample(S, SS, f"ref_pos_{ax}", sa, g, f, gg) for ax in "xy"])
        ok = np.all(np.isfinite(a), 1) & np.all(np.isfinite(b), 1)
        Rz, c, th = rigid_fit(a[ok], b[ok])
        xf = (Rz, c, th)
        out["rigid"] = {"fit": n, "apply": rigid["apply"], "yaw_deg": th, "shift_mm": c.tolist()}

    for n in names:
        (ra, rb), (sa, sb) = win[n]
        g, gg, f = grid(n)
        mR = np.interp(g, R["t"] - ra, R["direct"].astype(float)) > 0.5
        mS = np.interp(gg, S["t"] - sa, S["direct"].astype(float)) > 0.5
        m = mR & mS
        V = {}
        for k in set(STATES + REFS + TRACE):
            V[k] = (sample(R, SR, k, ra, g), sample(S, SS, k, sa, g, f, gg))
        if xf is not None and n in rigid["apply"]:
            Rz, c, th = xf
            for pre, off in (("pos_", c), ("ref_pos_", c), ("vel_", np.zeros(2)), ("dhat_", np.zeros(2))):
                ks = [f"{pre}{ax}" for ax in "xy"]
                P = np.column_stack([V[k][1] for k in ks]) @ Rz.T + off
                for i, k in enumerate(ks):
                    V[k] = (V[k][0], P[:, i])
            V["yaw"] = (V["yaw"][0], V["yaw"][1] + th)
        ok = lambda k: m & np.isfinite(V[k][0]) & np.isfinite(V[k][1])  # noqa: E731
        rr = lambda k: float(np.sqrt(np.mean((V[k][1][ok(k)] - V[k][0][ok(k)]) ** 2)))  # noqa: E731
        ref_pos = float(np.sqrt(np.mean([rr(f"ref_pos_{ax}") ** 2 for ax in "xyz"])))
        ref_q = float(np.sqrt(np.mean([rr(f"ref_q{j}") ** 2 for j in range(1, 5)])))
        use = ref_pos <= REF_POS_TOL and ref_q <= REF_Q_TOL and n not in manual
        ph = {"dur_real": rb - ra, "dur_sim": sb - sa, "warp": float(np.median(f)), "compared_s": float(m.sum() * DT) if use else 0.0,
              "ref_pos_rmse_mm": ref_pos, "ref_q_rmse_deg": ref_q, "used": use, "traces": {}}
        if not use:
            out["excluded"].append({"phase": n, "ref_pos_rmse_mm": ref_pos, "ref_q_rmse_deg": ref_q,
                                    "reason": manual.get(n, "plans differ")})
        for k in STATES + REFS:
            if use:
                acc[k].append(V[k][1][ok(k)] - V[k][0][ok(k)])
                if k in spans:
                    spans[k].append(V[k][0][ok(k)])
        for k in TRACE:
            ph["traces"][k] = {"real": np.where(m, V[k][0], np.nan)[::2].round(3).tolist(),
                               "sim": np.where(m, V[k][1], np.nan)[::2].round(3).tolist()}
        ph["t"] = g[::2].round(3).tolist()
        out["phases"][n] = ph
    res = {}
    for k in STATES + REFS:
        e = np.concatenate(acc[k]) if acc[k] else np.array([])
        row = {"rmse": float(np.sqrt(np.mean(e ** 2))) if e.size else None,
               "bias": float(np.mean(e)) if e.size else None, "n": int(e.size)}
        if k in spans and spans[k]:
            v = np.concatenate(spans[k])
            row["real_p1_p99"] = float(np.percentile(v, 99) - np.percentile(v, 1))
            row["real_std"] = float(np.std(v))
        res[k] = row
    out["rmse"] = res
    out["compared_s"] = float(sum(p["compared_s"] for p in out["phases"].values()))
    fp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "analysis", f"states_{tag}.json")
    json.dump(out, open(fp, "w"))
    print(f"{tag}: RTF {S['rtf']:.3f}, compared {out['compared_s']:.1f} s; rigid {out['rigid']}")
    for n, ph in out["phases"].items():
        print(f"  {n:13} warp {ph['warp']:.3f} plan diff {ph['ref_pos_rmse_mm']:6.1f} mm / {ph['ref_q_rmse_deg']:5.2f} deg -> {'compared' if ph['used'] else 'EXCLUDED'}")
    for k in STATES + REFS:
        r = res[k]
        print(f"  {k:10} rmse {r['rmse']:9.3f}  bias {r['bias']:+9.3f}  real span {r.get('real_p1_p99', float('nan')):9.3f}")
    print("wrote", fp)


if __name__ == "__main__":
    main()
