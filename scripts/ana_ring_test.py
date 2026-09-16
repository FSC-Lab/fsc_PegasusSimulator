#!/usr/bin/env python3
"""The tank's first mode from a ring test (hardware-transfer piece 7, 2026-09-16).

PROCEDURE (on the airframe, on the ground, the flight battery in, the tank filled to the flight
mass, the vehicle held or on its gear, PX4 logging `sensor_combined` to the DDS bag as in flight):
  1. record 10 s of quiet (the accelerometer's floor);
  2. tap the tank laterally (a finger on the tank wall, along body y for the mode the cancellation
     uses, then along body x) -- ONE tap, no push; the liquid rings, the airframe does not;
  3. leave it alone for 20 s (the 0.5% smooth-wall damping needs ~20 cycles to show);
  4. repeat three times per axis. The reading is the median of the three.
The bob's reaction on the airframe, m1 w^2 x, is what the accelerometer sees (0.1 m/s^2 for a
20 mm swing of the 100 mm tank on this 4 kg aircraft -- 5x the sim IMU's per-sample noise, so
the FFT peak is unmistakable and the envelope fit is the noisy part).

  ~/envs/isaac_env/bin/python3 scripts/ana_ring_test.py --bag <dir|db3> [--t0 T --t1 T] [--axis x|y|auto]
  ~/envs/isaac_env/bin/python3 scripts/ana_ring_test.py --csv file.csv          (columns t, ax, ay[, az], m/s^2, body FLU)
  ~/envs/isaac_env/bin/python3 scripts/ana_ring_test.py --selftest [--noise 0.02] [--zeta 0.0025]

WHAT IT MEASURES: f1 = the FFT peak of the windowed, detrended lateral acceleration (Hann, zero
padded, parabolic peak refinement); zeta = the log decrement of the band-passed signal's envelope
(the analytic signal's magnitude, a straight line fitted to its log over the ring), cross-checked
by the classical successive-peaks decrement. Both are compared to slosh_parameters() of the tank
the environment describes (VTOL_TANK_KG / VTOL_TANK_R, imported BY PATH from the simulator's
slosh.py), and the export line for a measured override is printed:
  export VTOL_TANK_F1=<f1> VTOL_TANK_ZETA=<zeta>
which the plant (default_tank) AND the predictor (_tank_params -> slosh_parameters) both read.
--selftest rings the simulator's own PendulumSlosh (by path) at a 20 mm initial swing, samples its
reaction at 200 Hz with white accelerometer noise, and requires f1 within 2% and zeta within 30%."""
import argparse
import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
SLOSH_PY = os.path.join(ROOT, "extensions", "pegasus.simulator", "pegasus", "simulator", "logic", "dynamics", "slosh.py")
G = 9.80665


def load_slosh():
    spec = importlib.util.spec_from_file_location("slosh", SLOSH_PY)
    sl = importlib.util.module_from_spec(spec); spec.loader.exec_module(sl)
    return sl


def analyse(t, a, f_lo=0.3, f_hi=10.0, verbose=True):
    """t (n,), a (n,) lateral acceleration [m/s^2] over the ring. Returns dict(f1, zeta, zeta_peaks,
    amp, snr)."""
    t = np.asarray(t, float); a = np.asarray(a, float)
    n = len(t); fs = (n - 1) / (t[-1] - t[0])
    # uniform resampling (a bag's IMU stream jitters), detrend
    tu = np.linspace(t[0], t[-1], n); au = np.interp(tu, t, a)
    au = au - np.polyval(np.polyfit(tu - tu[0], au, 1), tu - tu[0])
    # FFT peak with a Hann window and 8x zero padding, parabolic refinement
    w = np.hanning(n); nfft = int(2 ** math.ceil(math.log2(n * 8)))
    Sp = np.abs(np.fft.rfft(au * w, nfft)); fr = np.fft.rfftfreq(nfft, 1.0 / fs)
    band = (fr >= f_lo) & (fr <= f_hi)
    k = np.argmax(np.where(band, Sp, 0.0))
    if 0 < k < len(Sp) - 1:
        y0, y1, y2 = np.log(Sp[k - 1] + 1e-30), np.log(Sp[k] + 1e-30), np.log(Sp[k + 1] + 1e-30)
        d = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) if (y0 - 2 * y1 + y2) != 0 else 0.0
    else:
        d = 0.0
    f1 = fr[k] + d * (fr[1] - fr[0])
    # noise floor: the spectrum's median outside +-30% of the peak
    off = band & (np.abs(fr - f1) > 0.3 * f1)
    snr = Sp[k] / max(np.median(Sp[off]), 1e-12)
    # band-pass +-35% around f1 in the frequency domain, then the analytic signal's envelope
    A = np.fft.fft(au); f = np.fft.fftfreq(n, 1.0 / fs)
    keep = (np.abs(f) >= 0.65 * f1) & (np.abs(f) <= 1.35 * f1)
    Ab = np.where(keep, A, 0.0)
    h = np.zeros(n); h[0] = 1.0
    if n % 2 == 0:
        h[1:n // 2] = 2.0; h[n // 2] = 1.0
    else:
        h[1:(n + 1) // 2] = 2.0
    z = np.fft.ifft(Ab * h); env = np.abs(z); xb = np.real(z)
    # the log decrement on the envelope: skip the first and last 5% (the filter's edges), fit ln(env)
    i0, i1 = int(0.05 * n), int(0.95 * n)
    tt = tu[i0:i1] - tu[i0]; ev = env[i0:i1]
    # the noise floor: the envelope's tail, if the ring has died there (below 30% of the peak);
    # noise adds to the envelope in quadrature (a Rician floor that flattens the decay), so the
    # fit is on ln(env^2 - floor^2), where the ring stands above 15% of its peak and 2x the floor
    mx = ev.max(); tail = float(np.median(env[i1 - max(n // 10, 10):i1]))
    floor = tail if tail < 0.3 * mx else 0.0
    thr = max(0.15 * mx, 2.0 * floor)
    good = ev > thr
    e2 = np.maximum(ev[good] ** 2 - floor ** 2, (0.05 * mx) ** 2)
    slope, icpt = np.polyfit(tt[good], 0.5 * np.log(e2), 1)
    zeta = -slope / (2.0 * math.pi * f1)
    # cross-check: successive positive peaks of the band-passed signal
    pk = [i for i in range(i0 + 1, i1 - 1) if xb[i] > xb[i - 1] and xb[i] >= xb[i + 1] and xb[i] > thr]
    zeta_pk = float("nan")
    if len(pk) >= 4:
        ratios = np.log(xb[pk[:-1]] / xb[pk[1:]])
        zeta_pk = float(np.median(ratios)) / (2.0 * math.pi)     # delta = 2 pi zeta / sqrt(1 - zeta^2) ~ 2 pi zeta
    amp = env[i0:i1].max()
    out = dict(f1=float(f1), zeta=float(zeta), zeta_peaks=zeta_pk, amp=float(amp), snr=float(snr), fs=float(fs), n=int(n), cycles=float((tu[-1] - tu[0]) * f1))
    if verbose:
        print(f"  {n} samples at {fs:.1f} Hz over {tu[-1] - tu[0]:.1f} s ({out['cycles']:.0f} cycles)")
        print(f"  FFT peak f1 = {f1:.3f} Hz (SNR {snr:.0f}), ring amplitude {amp:.3f} m/s^2")
        print(f"  damping: envelope fit zeta = {100 * zeta:.2f}%  |  successive peaks {100 * zeta_pk:.2f}% ({len(pk)} peaks)")
    return out


def compare(res, sl, tank, axis=""):
    p = sl.slosh_parameters(tank)
    print(f"  closed form for this tank ({sl.describe(tank)})")
    df = 100.0 * (res["f1"] / p.freq_hz - 1.0) if p.freq_hz > 0 else float("nan")
    dz = 100.0 * (res["zeta"] / p.zeta - 1.0) if p.zeta > 0 else float("nan")
    print(f"  f1 measured {res['f1']:.3f} vs closed form {p.freq_hz:.3f} Hz: {df:+.1f}%  ({'within 10%: the closed form stands' if abs(df) <= 10 else 'OUTSIDE 10%: use the override'})")
    print(f"  zeta measured {100 * res['zeta']:.2f}% vs closed form {100 * p.zeta:.2f}%: {dz:+.0f}%  (the smooth-wall form is a floor; a real tank with fittings damps more)")
    print(f"  -> export VTOL_TANK_F1={res['f1']:.3f} VTOL_TANK_ZETA={max(res['zeta'], 0.0):.4f}   # {axis or 'lateral'} ring test; both the plant and the predictor read these")
    return df, dz


def selftest(noise, zeta_true, seed=1):
    sl = load_slosh()
    os.environ.setdefault("VTOL_TANK_R", "0.10")
    tank = sl.default_tank(damping_zeta=zeta_true if zeta_true is not None else None)
    p = sl.slosh_parameters(tank)
    M = 3.449572 + p.fluid_mass
    pend = sl.PendulumSlosh(p); pend.x = [0.0, 0.020]
    rng = np.random.default_rng(seed)
    dtp = 1.0 / 240.0; fs = 200.0; T = 25.0
    t = []; ay = []; tt = 0.0; nxt = 0.0
    while tt < T:
        F = np.array(pend.step(np.array([0.0, 0.0, G]), dtp))
        tt += dtp
        if tt >= nxt:
            t.append(tt); ay.append(F[1] / M + noise * rng.standard_normal()); nxt += 1.0 / fs
    print(f"SELFTEST: the simulator's PendulumSlosh rung at 20 mm (m1 {p.slosh_mass:.3f} kg, f1 {p.freq_hz:.3f} Hz, zeta {100 * p.zeta:.2f}%), "
          f"reaction / {M:.2f} kg + {noise} m/s^2 white noise at {fs:.0f} Hz")
    res = analyse(np.array(t), np.array(ay))
    df, dz = compare(res, sl, tank, "y")
    ok_f = abs(df) < 2.0; ok_z = abs(dz) < 30.0
    print(f"  f1 within 2%: {'OK' if ok_f else 'FAIL'}   zeta within 30%: {'OK' if ok_z else 'FAIL'}")
    return 0 if (ok_f and ok_z) else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag"); ap.add_argument("--csv"); ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--t0", type=float, default=None); ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--axis", default="auto"); ap.add_argument("--noise", type=float, default=0.02); ap.add_argument("--zeta", type=float, default=None)
    a = ap.parse_args()
    if a.selftest:
        return selftest(a.noise, a.zeta)
    if a.csv:
        d = np.genfromtxt(a.csv, delimiter=",", names=True)
        t = d["t"]; ax = d["ax"]; ay = d["ay"]
    elif a.bag:
        import glob, sqlite3
        from rclpy.serialization import deserialize_message
        from px4_msgs.msg import SensorCombined
        bag = a.bag
        if os.path.isdir(bag):
            bag = glob.glob(os.path.join(bag, "*.db3"))[0]
        cur = sqlite3.connect(bag).cursor()
        tid = {n: i for n, i in cur.execute("select name, id from topics")}
        topic = [k for k in tid if k.endswith("/fmu/out/sensor_combined")]
        if not topic:
            print("no sensor_combined in the bag"); return 2
        t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]
        rows = [((ts - t0) / 1e9, deserialize_message(dd, SensorCombined)) for ts, dd in
                cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[topic[0]],))]
        t = np.array([r[0] for r in rows]); ax = np.array([r[1].accelerometer_m_s2[0] for r in rows]); ay = np.array([-r[1].accelerometer_m_s2[1] for r in rows])   # FRD -> FLU
        print(f"bag {a.bag}: sensor_combined {len(t)} samples, {t[0]:.1f}-{t[-1]:.1f} s")
    else:
        ap.print_help(); return 2
    if a.t0 is not None or a.t1 is not None:
        m = (t >= (a.t0 if a.t0 is not None else t[0])) & (t <= (a.t1 if a.t1 is not None else t[-1]))
        t, ax, ay = t[m], ax[m], ay[m]
    sl = load_slosh(); tank = sl.default_tank()
    axes = {"x": [("x", ax)], "y": [("y", ay)]}.get(a.axis, [("x", ax), ("y", ay)])
    rc = 0
    for name, sig in axes:
        print(f"\n-- body {name} --")
        res = analyse(t, sig)
        if res["snr"] < 5.0:
            print(f"  (SNR {res['snr']:.1f} < 5: no ring on this axis, or the window misses it -- use --t0/--t1 on the tap)")
            continue
        compare(res, sl, tank, name)
    return rc


if __name__ == "__main__":
    sys.exit(main())
