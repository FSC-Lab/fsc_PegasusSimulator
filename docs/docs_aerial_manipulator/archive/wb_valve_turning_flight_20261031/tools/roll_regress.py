#!/usr/bin/env python3
"""Is the roll residual a spring (∝ roll angle), a damper (∝ roll rate), or tied to a lateral force?
Window = Go To Start + room hold (before Ready). Signals at 0.25 s bins (slow part):
  d   = raw roll residual d_sigma_c[66] (model y = roll), N.m
  cmd = commanded roll torque [21] (FLU x), N.m
  phi = mocap roll (deg), p = gyro-free roll rate from mocap (deg/s)
  fy  = translational estimate [31..33] rotated into the body (lateral, left +), N
  ey  = CoM error [48..50]-[45..47] in the body (lateral, left +), mm
    PYTHONNOUSERSITE=1 /usr/bin/python3 roll_regress.py"""
import numpy as np
import vt_common as V

A = "/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/"
FL = [("wb_valve_turning_flight_20261028", "180652", "10-28 18:06"), ("wb_valve_turning_flight_20261028", "181833", "10-28 18:18"),
      ("wb_valve_turning_flight_20261029", "145526", "10-29 14:55"), ("wb_valve_turning_flight_20261031", "174943", "10-31 17:49"),
      ("wb_valve_turning_flight_20261031", "175725", "10-31 17:57"), ("wb_valve_turning_flight_20261031", "180423", "10-31 18:04"),
      ("wb_valve_turning_flight_20261031", "211808", "10-31 21:18")]
DT = 0.25
for d, tag, lab in FL:
    V._c.NPZ = A + d + "/npz"
    b = V.Bag(tag)
    ed = b.edges("pl_pstatus")
    tg = [t for t, s in ed if s.startswith("FLYING go_to_start")][0]
    tr = [t for t, s in ed if s.startswith("FLYING ready")][0]
    tw = b.t("wb"); W = b.f("wb", "data")
    ok = np.array([len(r) >= 118 for r in W]); Wm = np.array([np.asarray(r[:118], float) for r in W[ok]]); tw = tw[ok]
    tm = b.t("mocap"); g = lambda k: b.f("mocap", k)
    qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
    roll = np.degrees(np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx ** 2 + qy ** 2)))
    yaw = np.unwrap(np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy ** 2 + qz ** 2)))
    tb = np.arange(tg, tr, DT)
    def bin_(t, x):
        return np.array([np.nanmean(x[(t >= a) & (t < a + DT)]) if ((t >= a) & (t < a + DT)).any() else np.nan for a in tb])
    dr = bin_(tw, Wm[:, 66]); cmd = bin_(tw, Wm[:, 21]); dfr = bin_(tw, Wm[:, 35])
    phi = bin_(tm, roll); ps = bin_(tm, yaw)
    p = np.gradient(phi, DT)
    fx, fyw = bin_(tw, Wm[:, 31]), bin_(tw, Wm[:, 32])
    ex, eyw = bin_(tw, Wm[:, 48] - Wm[:, 45]), bin_(tw, Wm[:, 49] - Wm[:, 46])
    lat = lambda ax, ay: -np.sin(ps) * ax + np.cos(ps) * ay   # noqa: E731  body left +
    fy = lat(fx, fyw); ey = 1e3 * lat(ex, eyw)
    ok_ = np.isfinite(dr) & np.isfinite(phi) & np.isfinite(fy)
    D = dr[ok_] - dr[ok_].mean()
    def c(x):
        x = x[ok_] - np.nanmean(x[ok_])
        return np.corrcoef(D, x)[0, 1]
    X = np.column_stack([phi[ok_] - phi[ok_].mean(), p[ok_] - p[ok_].mean(), fy[ok_] - fy[ok_].mean()])
    coef, *_ = np.linalg.lstsq(X, D, rcond=None)
    r2 = 1 - np.var(D - X @ coef) / np.var(D)
    print(f"{lab}: sd d_raw {D.std():.3f} filt {np.nanstd(dfr):.3f} | corr d with: roll {c(phi):+.2f}, roll rate {c(p):+.2f}, "
          f"lat force est {c(fy):+.2f}, lat CoM err {c(ey):+.2f}, cmd {c(cmd):+.2f} | fit d = {coef[0]:+.3f}/deg roll {coef[1]:+.4f}/(deg/s) {coef[2]:+.3f}/N  R2 {r2:.2f} "
          f"| sd roll {np.nanstd(phi):.2f} deg, lat force {np.nanstd(fy):.2f} N, lat err {np.nanstd(ey):.0f} mm")
