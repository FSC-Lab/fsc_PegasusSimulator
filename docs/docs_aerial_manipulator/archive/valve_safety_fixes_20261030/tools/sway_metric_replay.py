#!/usr/bin/env python3
"""Candidate sway-guard metrics, replayed on the recorded valve contacts (claw on the handle).
  E  = |x_c - x_cd| horizontal (CoM error)                                   [mm]
  V  = |d/dt (x_c - x_cd)| horizontal, low-passed TAU s (CoM velocity error)  [mm/s]
A metric trips when it stays above its limit for PERSIST s. Isaac: completed turns must not trip.
Hardware: 10-28 180652 / 181833 (contacts ended by the false tilt trip) and 10-29 145526 (the
collapse; q3 rate guard at the end of the window).
Run: PYTHONNOUSERSITE=1 nice -n 19 /usr/bin/python3 sway_metric_replay.py"""
import glob, os
import numpy as np
A = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
TAU, PERSIST = 0.25, 0.2
VLIMS = [0.13, 0.15, 0.16, 0.17, 0.18]


def metrics(t, xcd, xc):
    o = np.argsort(t); t, xcd, xc = t[o], xcd[o], xc[o]
    k = np.r_[True, np.diff(t) > 1e-4]; t, xcd, xc = t[k], xcd[k], xc[k]
    e = (xc - xcd)[:, :2]
    v = np.zeros_like(e); vf = np.zeros(2)
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        raw = (e[i] - e[i - 1]) / dt if dt < 0.1 else vf
        a = 1 - np.exp(-dt / TAU)
        vf = vf + a * (raw - vf); v[i] = vf
    return t, np.linalg.norm(e, axis=1), np.linalg.norm(v, axis=1)


def first_trip(t, x, lim):
    t0 = None
    for ti, xi in zip(t, x):
        if xi > lim:
            t0 = ti if t0 is None else t0
            if ti - t0 >= PERSIST:
                return ti
        else:
            t0 = None
    return None


def mark(z, name):
    t = [tt for tt, n in zip(z["marks_t"], z["marks_name"]) if str(n) == name]
    return t[0] if t else None


rows = []
for pat in ["valve_turning_v2_20261027/runs/*.npz", "wb_valve_turning_flight_20261028/sim/runs/*.npz",
            "valve_turning_safety_20261022/runs/*.npz", "valve_bench_friction_20261023/runs/*.npz"]:
    for f in sorted(glob.glob(os.path.join(A, pat))):
        z = np.load(f, allow_pickle=True)
        if "dbg" not in z.files or "marks_t" not in z.files or mark(z, "ready:end") is None:
            continue
        d = z["dbg"]
        if d.ndim != 2 or d.shape[1] < 53:
            continue
        a, b = mark(z, "ready:end"), mark(z, "exit:start")
        ab = None
        for e in z["events"]:
            e = str(e)
            if "ABORT" in e or "SAFETY GUARD" in e:
                ab = float(e.split("s]")[0].strip("[ ")); break
        end = min(x for x in (b, ab, d[-1, 0]) if x is not None)
        m = (d[:, 0] >= a) & (d[:, 0] <= end) & np.isfinite(d[:, 52])
        if m.sum() < 20:
            continue
        t, E, V = metrics(d[m, 0], d[m, 47:50], d[m, 50:53])
        ok = (not bool(z["aborted"])) and ab is None
        rows.append(("isaac", os.path.basename(f), ok, E.max(), V.max(), [first_trip(t, V, L) for L in VLIMS], end - a))

for tag, folder, w in [("180652", "wb_valve_turning_flight_20261028", (250.81, 260.72)),
                       ("181833", "wb_valve_turning_flight_20261028", None),
                       ("145526", "wb_valve_turning_flight_20261029", (163.84, 172.58))]:
    z = np.load(os.path.join(A, folder, "npz", f"b{tag}.npz"), allow_pickle=True)
    D, R = z["wb__data"], z["wb__recv"]
    st, sr = z["pl_pstatus__data"], z["pl_pstatus__recv"]
    t_ready = [t for s, t in zip(st, sr) if str(s).startswith("DONE ready")]
    t_ab = [t for s, t in zip(st, sr) if str(s).startswith("ABORTING")]
    a, b = t_ready[0], t_ab[0]
    m = (R >= a) & (R <= b) & np.isfinite(D[:, 50]) & (D[:, 0] == 1)
    t, E, V = metrics(R[m], D[m, 45:48], D[m, 48:51])
    rows.append(("HW", tag, None, E.max(), V.max(), [first_trip(t, V, L) for L in VLIMS], b - a))
    for L in VLIMS:
        tr = first_trip(t, V, L)
        print(f"HW {tag}: V > {L*1e3:.0f} mm/s trips {'never' if tr is None else f'{b - tr:.1f} s before the window ended ({tr - a:.1f} s after the grasp)'}")

print(f"\n{'':6s}{'run':28s} {'outcome':9s} {'contact s':>9s} {'max E mm':>8s} {'max V mm/s':>10s}   V trips at " +
      " ".join(f"{int(L*1e3):>4d}" for L in VLIMS))
for r in rows:
    print(f"{r[0]:6s}{r[1]:28s} {('-' if r[2] is None else 'complete' if r[2] else 'ABORT'):9s} {r[6]:9.1f} "
          f"{r[3]*1e3:8.1f} {r[4]*1e3:10.1f}   " + " ".join(f"{'-' if x is None else 'TRIP':>4s}" for x in r[5]))
good = [r for r in rows if r[2] is True]
bad = [r for r in rows if r[2] is False]
for i, L in enumerate(VLIMS):
    print(f"V {L*1e3:.0f} mm/s: Isaac completed tripped {sum(r[5][i] is not None for r in good)}/{len(good)}, "
          f"Isaac aborted {sum(r[5][i] is not None for r in bad)}/{len(bad)}")
