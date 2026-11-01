#!/usr/bin/env python3
"""Phase of the slow roll residual against the base's motion: cross-correlation (+lag = residual LEADS) of the raw
roll residual (0.5 s average) with the body-lateral CoM error, velocity and acceleration (mocap), roll angle, heading.
Window: Go To Start start -> Ready.     PYTHONNOUSERSITE=1 /usr/bin/python3 roll_phase.py 180423 211808"""
import sys
import numpy as np
import vt_common as V

FS = 25.0
for tag in sys.argv[1:]:
    b = V.Bag(tag)
    ed = b.edges("pl_pstatus")
    tg = [t for t, s in ed if s.startswith("FLYING go_to_start")][0]
    tr = [t for t, s in ed if s.startswith("FLYING ready")][0]
    tw = b.t("wb"); W = b.f("wb", "data")
    ok = np.array([len(r) >= 118 for r in W]); Wm = np.array([np.asarray(r[:118], float) for r in W[ok]]); tw = tw[ok]
    tm = b.t("mocap"); g = lambda k: b.f("mocap", k)
    qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
    yaw = np.unwrap(np.arctan2(2 * (qw * qx * 0 + qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2)))
    roll = np.degrees(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2)))
    x, y = g("pose.position.x"), g("pose.position.y")
    tu = np.arange(tg, tr, 1 / FS)
    k = np.ones(13) / 13
    sm = lambda a: np.convolve(a, k, mode="same")   # noqa: E731
    idx = np.clip(np.searchsorted(tw, tu), 1, len(tw) - 1)       # bin-average the 250 Hz residual (it is
    v66 = Wm[:, 66]; fin = np.isfinite(v66) & (Wm[:, 0] == 1.0)    # a deadbeat difference, ~2 N.m of noise)
    cs = np.r_[0, np.cumsum(np.where(fin, v66, 0.0))]; cn = np.r_[0, np.cumsum(fin)]
    lo = np.searchsorted(tw, tu - 0.5 / FS); hi = np.searchsorted(tw, tu + 0.5 / FS)
    d = sm((cs[hi] - cs[lo]) / np.maximum(cn[hi] - cn[lo], 1))
    ps = np.interp(tu, tm, yaw)
    X, Y = np.interp(tu, tm, x), np.interp(tu, tm, y)
    vx, vy = np.gradient(sm(X), 1 / FS), np.gradient(sm(Y), 1 / FS)
    ax_, ay_ = np.gradient(sm(vx), 1 / FS), np.gradient(sm(vy), 1 / FS)
    lat = lambda a, c: -np.sin(ps) * a + np.cos(ps) * c   # noqa: E731
    ex = np.interp(tu, tw, Wm[:, 48] - Wm[:, 45]); ey = np.interp(tu, tw, Wm[:, 49] - Wm[:, 46])
    sig = {"lat CoM err": lat(ex, ey), "lat vel": lat(vx, vy), "lat acc": lat(ax_, ay_),
           "roll angle": sm(np.interp(tu, tm, roll)), "yaw rate": np.gradient(sm(ps), 1 / FS)}
    s = slice(10, -10)
    dd = d[s] - d[s].mean()
    print(f"\n### {tag}  (sd raw roll residual, 0.5 s avg: {dd.std():.3f} N.m)")
    for n, v in sig.items():
        vv = v[s] - v[s].mean()
        best = None
        for L in range(-int(3 * FS), int(3 * FS) + 1):
            a = dd[max(0, L):len(dd) + min(0, L)]; c = vv[max(0, -L):len(vv) + min(0, -L)]
            r = np.corrcoef(a, c)[0, 1]
            if best is None or abs(r) > abs(best[1]):
                best = (L / FS, r)
        r0 = np.corrcoef(dd, vv)[0, 1]
        print(f"  {n:12s} corr at 0 lag {r0:+.2f} | strongest {best[1]:+.2f} at residual {'leading' if best[0] > 0 else 'lagging'} by {abs(best[0]):.2f} s")
    # dominant period of the residual
    f = np.fft.rfftfreq(len(dd), 1 / FS); P = np.abs(np.fft.rfft(dd * np.hanning(len(dd)))) ** 2
    m = (f > 0.05) & (f < 2)
    print(f"  residual spectral peak {f[m][np.argmax(P[m])]:.2f} Hz")
