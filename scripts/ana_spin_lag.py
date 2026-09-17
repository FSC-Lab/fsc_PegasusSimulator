#!/usr/bin/env python3
"""The rotor's spin-up lag and static model, fitted from a thrust-stand log.

  ~/envs/isaac_env/bin/python3 scripts/ana_spin_lag.py --dir <motor dir>   (all steps_*_{inputs,outputs}.csv)
  ~/envs/isaac_env/bin/python3 scripts/ana_spin_lag.py --inputs A.csv --outputs B.csv
  ~/envs/isaac_env/bin/python3 scripts/ana_spin_lag.py --selftest [--noise 40] [--tau 0.12] [--dead 0.02]

WHAT IT FITS, and why in this form. The plant (vtol.py, thrust_model "lag"/"momentum") integrates a
SPEED state: omega_dot = (omega_cmd - omega)/tau with omega_cmd = throttle * rotor_omega_max, and then
T = max_thrust*(omega/omega_max)^2. The controller's L1 predictor models the SAME lag
(`L1Aug.applied_moment`, `l1_tau_spin`). So the number both sides need is a first-order time constant
ON OMEGA -- fitting the raw THRUST trace instead gives a systematically different (smaller) constant,
because thrust is omega SQUARED and its transient is therefore not first-order. This tool fits omega,
and fits sqrt(T) as an independent cross-check of the same constant.

Each throttle step is fitted as a DEAD TIME then a first-order rise,
    omega(t) = omega_inf + (omega_0 - omega_inf) * exp(-(t - t_step - L)/tau)   for t >= t_step + L,
with (tau, L, omega_0, omega_inf) free. L is reported separately and NOT folded into tau: a command
transport delay belongs in the latency budget (`latency_s`), not in the actuator model, and a fit
with t0 pinned at the step would quietly absorb one into the other.

Spin-UP and spin-DOWN are reported separately: the motor accelerates on torque and decelerates on
drag, so they need not match, and the model carries ONE constant -- if they differ materially that is
a finding, not something to average away.

It also reports the static model the same log gives for free (the settled points of every step):
rpm(throttle), thrust(omega), torque/thrust, and prints the `airframe_id.cfg` export block.
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

try:
    from scipy.optimize import least_squares
except ImportError:
    least_squares = None

RPM2RAD = 2.0 * math.pi / 60.0


def _f(row, key, default=np.nan):
    v = row.get(key)
    if v is None or v == "" or v == "nan":
        return default
    try:
        return float(v)
    except ValueError:
        return default


def load(inputs_csv, outputs_csv):
    """-> t [s, common clock], throttle [%], omega [rad/s], thrust [N], torque [N.m]"""
    o = list(csv.DictReader(open(outputs_csv, encoding="utf-8-sig")))
    i = list(csv.DictReader(open(inputs_csv, encoding="utf-8-sig")))
    col = lambda rows, k: np.array([_f(r, k) for r in rows])
    to = col(o, "System Time (Unix ms)") / 1e3
    rpm = col(o, "Motor Optical Speed (RPM)")
    if not np.isfinite(rpm).any() or np.nanmax(rpm) <= 0:
        rpm = col(o, "Motor Electrical Speed (RPM)")
    T, Q = col(o, "Thrust (N)"), col(o, "Torque (N·m)")
    esc = col(o, "ESC signal (µs)")
    ti = np.array([_f(r, "timestamp_unix_ms") for r in i]) / 1e3
    th = np.array([_f(r, "throttle_cmd_pct") for r in i])
    ok = np.isfinite(to) & np.isfinite(rpm)
    to, rpm, T, Q, esc = to[ok], rpm[ok], T[ok], Q[ok], esc[ok]
    # the command is on its own clock and rate: hold it onto the measurement's samples
    thr = np.interp(to, ti[np.isfinite(ti)], th[np.isfinite(ti)], left=th[0], right=th[-1])
    t0 = to[0]
    return to - t0, thr, rpm * RPM2RAD, T, Q, esc


def steps(t, thr, min_jump=2.0):
    """-> [(t_step, thr_before, thr_after, i0, i1)] over the whole log."""
    d = np.diff(thr)
    k = np.where(np.abs(d) > min_jump)[0]
    # collapse a ramp that the interpolation spread over a few samples
    keep = [j for n, j in enumerate(k) if n == 0 or t[j] - t[k[n - 1]] > 0.25]
    out = []
    for n, j in enumerate(keep):
        i1 = keep[n + 1] if n + 1 < len(keep) else len(t) - 1
        out.append((t[j], thr[max(j - 1, 0)], thr[min(j + 2, len(thr) - 1)], j, i1))
    return out


def fit_step(t, y, t_step, t_end):
    """Dead time + first order on y. -> tau, L, y0, yinf, r2  (nan if it cannot be fitted)."""
    w = (t >= t_step - 0.05) & (t <= t_end)
    tw, yw = t[w], y[w]
    if len(tw) < 8 or not np.isfinite(yw).all():
        return (np.nan,) * 5
    y0g, yinfg = float(yw[0]), float(np.median(yw[-max(3, len(yw) // 5):]))
    if abs(yinfg - y0g) < 1e-6:
        return (np.nan,) * 5

    def model(p):
        tau, L, a, b = p
        x = np.clip((tw - t_step - L) / max(tau, 1e-4), 0.0, 60.0)
        return b + (a - b) * np.exp(-x)

    if least_squares is None:                      # no scipy: log-linear fallback on the normalised error
        e = (yw - yinfg) / (y0g - yinfg)
        m = (e > 0.05) & (e < 0.95)
        if m.sum() < 4:
            return (np.nan,) * 5
        A = np.vstack([tw[m] - t_step, np.ones(m.sum())]).T
        s, _ = np.linalg.lstsq(A, np.log(e[m]), rcond=None)[0], None
        tau = -1.0 / s[0] if s[0] < 0 else np.nan
        return tau, 0.0, y0g, yinfg, np.nan
    r = least_squares(lambda p: model(p) - yw, [0.1, 0.02, y0g, yinfg],
                      bounds=([1e-3, 0.0, -np.inf, -np.inf], [2.0, 0.5, np.inf, np.inf]))
    res = model(r.x) - yw
    ss = float(np.sum((yw - yw.mean()) ** 2))
    r2 = 1.0 - float(np.sum(res ** 2)) / ss if ss > 0 else np.nan
    return float(r.x[0]), float(r.x[1]), float(r.x[2]), float(r.x[3]), r2


def med(v):
    return float(np.median(v)) if len(v) else float("nan")


def analyse(pairs, label, quiet=False):
    rows, static = [], []
    for inputs_csv, outputs_csv in pairs:
        t, thr, om, T, Q, esc = load(inputs_csv, outputs_csv)
        run = os.path.basename(outputs_csv).replace("_outputs.csv", "")
        for (ts, tha, thb, j, i1) in steps(t, thr):
            t_end = min(t[i1], ts + 2.5)
            tau, L, o0, oinf, r2 = fit_step(t, om, ts, t_end)
            sq = np.sqrt(np.clip(T, 0.0, None))
            tau_T, _, _, _, r2_T = fit_step(t, sq, ts, t_end)
            if not np.isfinite(tau) or r2 < 0.9 or oinf < 1.0:
                continue
            L_esc = np.nan
            if np.isfinite(esc).sum() > 10:
                we = (t >= ts - 0.6) & (t <= ts + 0.6) & np.isfinite(esc)
                if we.sum() > 6:
                    de = np.diff(esc[we])
                    if np.abs(de).max() > 5.0:                     # the stand saw the PWM move
                        t_esc = t[we][1:][np.argmax(np.abs(de))]
                        _, L2, _, _, r2b = fit_step(t, om, t_esc, t_end)
                        if np.isfinite(r2b) and r2b > 0.9:
                            L_esc = L2
            rows.append(dict(run=run, t=ts, a=tha, b=thb, up=thb > tha, tau=tau, L=L,
                             om=oinf, r2=r2, tau_T=tau_T, r2_T=r2_T, L_esc=L_esc))
            # the settled point of this step: the last 0.5 s before the next one
            w = (t > t_end - 0.5) & (t <= t_end)
            if w.sum() >= 3:
                # nanmedian: the stand drops the odd thrust/torque sample, and a plain median
                # propagates that nan through the whole static fit
                static.append((thb, float(np.nanmedian(om[w])), float(np.nanmedian(T[w])), float(np.nanmedian(Q[w]))))
    if not rows:
        print(f"{label}: no fittable steps"); return None
    up = [r for r in rows if r["up"]]; dn = [r for r in rows if not r["up"]]
    if not quiet:
        print(f"\n=== {label}: {len(rows)} steps fitted ({len(up)} up, {len(dn)} down)")
        print(f"{'run':>10s} {'thr %':>11s} {'omega_inf':>9s} {'tau [s]':>8s} {'dead [ms]':>9s} {'R2':>6s} {'tau(sqrtT)':>10s}")
        for r in rows:
            print(f"{r['run']:>10s} {r['a']:5.0f}->{r['b']:4.0f} {r['om']:9.1f} {r['tau']:8.3f} "
                  f"{1e3 * r['L']:9.1f} {r['r2']:6.3f} {r['tau_T']:10.3f}")
    out = dict(rows=rows, tau_up=med([r["tau"] for r in up]), tau_dn=med([r["tau"] for r in dn]),
               tau_all=med([r["tau"] for r in rows]), L=med([r["L"] for r in rows]),
               tau_T=med([r["tau_T"] for r in rows if np.isfinite(r["tau_T"])]),
               spread=(med([r["tau"] for r in rows]), float(np.percentile([r["tau"] for r in rows], 10)),
                       float(np.percentile([r["tau"] for r in rows], 90))),
               static=static, n=len(rows))
    print(f"\n{label}: tau  UP {out['tau_up']:.3f} s | DOWN {out['tau_dn']:.3f} s | all {out['tau_all']:.3f} s "
          f"(10-90% {out['spread'][1]:.3f}-{out['spread'][2]:.3f})")
    print(f"{' ' * len(label)}  dead time {1e3 * out['L']:.0f} ms (command -> first rpm response; NOT part of tau)")
    print(f"{' ' * len(label)}  cross-check on sqrt(thrust): {out['tau_T']:.3f} s")
    esc_L = [r["L_esc"] for r in rows if np.isfinite(r.get("L_esc", np.nan))]
    if esc_L:
        print(f"{' ' * len(label)}  dead time referenced to the ESC PWM the STAND logged (one clock, no skew): "
              f"{1e3 * float(np.median(esc_L)):.0f} ms")
    # tau vs omega, both directions: the number that matters is the one at the HOVER operating point
    print(f"\n  tau [s] by speed band (the model carries ONE constant -- take it where the aircraft lives):")
    print(f"    {'omega band':>16s} {'up':>8s} {'down':>8s}")
    edges = [0, 200, 300, 400, 500, 600, 1e9]
    for lo, hi in zip(edges[:-1], edges[1:]):
        u = [r["tau"] for r in rows if r["up"] and lo <= r["om"] < hi]
        d = [r["tau"] for r in rows if not r["up"] and lo <= r["om"] < hi]
        if u or d:
            hh = "inf" if hi > 1e8 else f"{hi:.0f}"
            print(f"    {lo:6.0f}-{hh:>6s} rad/s {med(u):8.3f} {med(d):8.3f}")
    return out


def static_model(static, label):
    if len(static) < 4:
        return
    a = np.array(static)
    thr, om, T, Q = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    m = (thr > 5) & (om > 1) & np.isfinite(om) & np.isfinite(T) & np.isfinite(Q)
    thr, om, T, Q = thr[m], om[m], T[m], Q[m]
    k_lin = float(np.sum(thr * om) / np.sum(thr * thr))        # through the origin: the sim's own map
    p_aff = np.polyfit(thr, om, 1)
    ct = float(np.sum(om ** 2 * T) / np.sum(om ** 4))          # T = ct * omega^2
    cq = float(np.sum(T * Q) / np.sum(T * T))                  # Q = cq * T   [m]
    om100, T100 = k_lin * 100.0, ct * (k_lin * 100.0) ** 2
    print(f"\n--- {label} static model, from {len(thr)} settled step points (throttle {thr.min():.0f}-{thr.max():.0f}%)")
    print(f"  omega(throttle): through origin {k_lin:.3f} rad/s per %   affine {p_aff[0]:.3f}x + {p_aff[1]:.1f}")
    print(f"  thrust = {ct:.4e} * omega^2   [N, rad/s]     torque/thrust = {cq:.4f} m")
    print(f"  extrapolated to 100%: omega_max {om100:.0f} rad/s ({om100 / RPM2RAD:.0f} rpm), max_thrust {T100:.1f} N")
    print(f"  measured max in this log: omega {om.max():.0f} rad/s, thrust {T.max():.1f} N at {thr.max():.0f}%")
    return dict(k=k_lin, ct=ct, cq=cq, om100=om100, T100=T100, om_meas=om.max(), T_meas=T.max(), thr_max=thr.max())


def selftest(tau, dead, noise):
    print(f"--- selftest: a first-order rotor, tau {tau:.3f} s, dead time {1e3 * dead:.0f} ms, "
          f"{noise:.0f} rpm of noise, sampled at 71 Hz like the bench")
    rng = np.random.default_rng(7)
    dt, errs, errsL = 1.0 / 71.0, [], []
    for step in (0.2, 0.4, 0.6, 0.8):
        t = np.arange(0.0, 3.0, dt)
        om_cmd = step * 800.0
        om = om_cmd * (1.0 - np.exp(-np.clip((t - dead) / tau, 0, None)))
        om = om + rng.normal(0.0, noise * RPM2RAD, len(t))
        tau_f, L_f, _, _, r2 = fit_step(t, om, 0.0, 2.5)
        errs.append(abs(tau_f - tau) / tau); errsL.append(abs(L_f - dead))
        print(f"  step to {step * 100:3.0f}%: tau {tau_f:.4f} s ({100 * (tau_f / tau - 1):+.1f}%), "
              f"dead {1e3 * L_f:5.1f} ms, R2 {r2:.4f}")
    ok = max(errs) < 0.10 and max(errsL) < 0.010
    print(f"  worst tau error {100 * max(errs):.1f}%, worst dead-time error {1e3 * max(errsL):.1f} ms -> "
          f"{'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir"); ap.add_argument("--inputs"); ap.add_argument("--outputs")
    ap.add_argument("--label", default=None)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--tau", type=float, default=0.12); ap.add_argument("--dead", type=float, default=0.02)
    ap.add_argument("--noise", type=float, default=40.0)
    ap.add_argument("--hover_thrust", type=float, default=9.69,
                    help="N per rotor at hover (default: the 3.9516 kg tank airframe on four rotors)")
    a = ap.parse_args()
    if a.selftest:
        return selftest(a.tau, a.dead, a.noise)
    if a.dir:
        pairs = []
        for out in sorted(glob.glob(os.path.join(a.dir, "**", "steps_*_outputs.csv"), recursive=True)):
            inp = out.replace("_outputs.csv", "_inputs.csv")
            if os.path.exists(inp):
                pairs.append((inp, out))
        label = a.label or os.path.basename(os.path.normpath(a.dir))
    elif a.inputs and a.outputs:
        pairs, label = [(a.inputs, a.outputs)], a.label or os.path.basename(a.outputs)
    else:
        ap.error("--dir, or --inputs and --outputs, or --selftest")
    if not pairs:
        print("no steps_*_{inputs,outputs}.csv found"); return 1
    r = analyse(pairs, label)
    if r is None:
        return 1
    s = static_model(r["static"], label)
    om_h = tau_h = float("nan")
    if s and s["ct"] > 0:
        om_h = math.sqrt(a.hover_thrust / s["ct"])               # the omega this rotor hovers the aircraft at
        near = sorted(r["rows"], key=lambda q: abs(q["om"] - om_h))[:6]
        tau_h = float(np.median([q["tau"] for q in near]))
        print(f"\n--- the hover operating point: {a.hover_thrust:.2f} N per rotor -> omega {om_h:.0f} rad/s "
              f"({om_h / RPM2RAD:.0f} rpm, {om_h / s['k']:.0f}% throttle)")
        print(f"    tau from the {len(near)} steps nearest it: {tau_h:.3f} s "
              f"(up {med([q['tau'] for q in near if q['up']]):.3f}, down {med([q['tau'] for q in near if not q['up']]):.3f})")
    tau = tau_h if np.isfinite(tau_h) else r["tau_all"]
    print(f"\n--- airframe_id.cfg export for {label}")
    print(f"[rotors]")
    print(f"tau_spin = {tau:.3f}            # measured AT THE HOVER POINT (spin-down is drag-driven: its constant scales with 1/omega)")
    if s:
        print(f"omega_max = {s['om100']:.1f}       # extrapolated to 100% throttle from <= {s['thr_max']:.0f}%")
        print(f"max_thrust = {s['T100']:.2f}       # ditto -- confirm with a 100% run before flying it")
        print(f"cq = {s['cq']:.4f}             # torque/thrust [m], measured")
    v = ("PASS -- inside the measured envelope" if tau <= 0.16 else
         "MARGINAL -- 0.18 s loses 5 of 16 seeds" if tau <= 0.18 else
         "OUTSIDE THE ENVELOPE -- 0.20 s is unflyable at any model value")
    print(f"\nagainst the 2026-09-16 actuator-lag sweep (<= 0.16 pass / 0.18 marginal / 0.20 fail): {v}")
    print(f"and set l1_tau_spin to the same number -- the sweep's cost below 0.18 s is the MISMATCH, not the lag.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
