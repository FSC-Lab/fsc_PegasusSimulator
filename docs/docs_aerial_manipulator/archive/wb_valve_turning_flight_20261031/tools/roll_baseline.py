#!/usr/bin/env python3
"""Slow roll residual during Go To Start on every whole-body hardware flight with an extracted npz: sd of the
DIRECT-tick raw residual d_sigma_c[66] averaged over 0.5 s, and its correlation with the body-lateral CoM error.
    PYTHONNOUSERSITE=1 /usr/bin/python3 roll_baseline.py"""
import glob, os
import numpy as np
import vt_common as V

A = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/"
for npz in sorted(glob.glob(A + "*flight*/npz/b*.npz")):
    d = os.path.dirname(npz); tag = os.path.basename(npz)[1:-4]
    try:
        V._c.NPZ = d
        b = V.Bag(tag)
        if not b.has("pl_pstatus") or not b.has("wb"):
            continue
        ed = b.edges("pl_pstatus")
        tg = [t for t, s in ed if s.startswith("FLYING go_to_start")]
        th = [t for t, s in ed if s.startswith("DONE go_to_start") and tg and t > tg[0]]
        if not tg or not th:
            continue
        tg, th = tg[0], th[0]
        tw = b.t("wb"); W = b.f("wb", "data")
        ok = np.array([len(r) >= 72 for r in W]); Wm = np.array([np.asarray(r[:72], float) for r in W[ok]]); tw = tw[ok]
        m = (tw >= tg) & (tw <= th) & (Wm[:, 0] == 1.0) & np.isfinite(Wm[:, 66])
        if m.sum() < 500:
            continue
        t, X = tw[m], Wm[m]
        bins = np.arange(t[0], t[-1], 0.5)
        idx = np.digitize(t, bins)
        r = np.array([X[idx == k, 66].mean() for k in np.unique(idx)])
        e = np.array([np.linalg.norm(X[idx == k, 49].mean() - X[idx == k, 46].mean()) for k in np.unique(idx)])
        tm = b.t("mocap"); g = lambda k: b.f("mocap", k)
        qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
        ps = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2)))
        tb = np.array([t[idx == k].mean() for k in np.unique(idx)])
        psb = np.interp(tb, tm, ps)
        ex = np.array([(X[idx == k, 48] - X[idx == k, 45]).mean() for k in np.unique(idx)])
        ey = np.array([(X[idx == k, 49] - X[idx == k, 46]).mean() for k in np.unique(idx)])
        lat = -np.sin(psb) * ex + np.cos(psb) * ey
        c = np.corrcoef(r, lat)[0, 1]
        print(f"{d.split('/')[-2][:45]:45s} {tag}  GTS {th - tg:5.1f} s  slow roll residual sd {r.std():.3f} N.m  corr with lat CoM err {c:+.2f}  lat err sd {1e3 * lat.std():4.0f} mm")
    except Exception as ex_:
        print(d.split('/')[-2], tag, "skip:", type(ex_).__name__, ex_)
