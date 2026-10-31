#!/usr/bin/env python3
"""Would a CoM-error sway guard (|x_c - x_cd| horizontal > L for P s while the claw is on the
handle) have tripped the recorded Isaac valve runs? Completed turns must not trip; failed ones
should trip before they failed. Window: ready:end (claw at the handle) -> exit:start (or the abort).
dbg columns = wb_control_debug index + 2 (pl_mission.py): x_cd [45..47] -> 47..49, x_c -> 50..52.
Run: PYTHONNOUSERSITE=1 nice -n 19 /usr/bin/python3 sway_guard_replay.py"""
import glob, os
import numpy as np
A = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
PAT = ["valve_turning_v2_20261027/runs/*.npz", "wb_valve_turning_flight_20261028/sim/runs/*.npz",
       "valve_turning_safety_20261022/runs/*.npz", "valve_bench_friction_20261023/runs/*.npz"]
LIMS = [0.04, 0.05, 0.06, 0.08, 0.10]
PERSIST = 0.2


def mark(z, name):
    t = [tt for tt, n in zip(z["marks_t"], z["marks_name"]) if str(n) == name]
    return t[0] if t else None


def longest_above(t, e, lim):
    """first time e stays > lim for PERSIST s, and the peak."""
    above = e > lim
    t0 = None
    for ti, a in zip(t, above):
        if a:
            if t0 is None:
                t0 = ti
            if ti - t0 >= PERSIST:
                return ti
        else:
            t0 = None
    return None


rows = []
for pat in PAT:
    for f in sorted(glob.glob(os.path.join(A, pat))):
        try:
            z = np.load(f, allow_pickle=True)
        except Exception:
            continue
        if "dbg" not in z.files or "marks_t" not in z.files:
            continue
        a = mark(z, "ready:end")
        if a is None:
            continue
        b = mark(z, "exit:start") or mark(z, "abort") or z["dbg"][-1, 0]
        ev = [str(e) for e in z["events"]]
        abort_t = None
        for e in ev:
            if "ABORT" in e or "SAFETY GUARD" in e or "abort:" in e.lower():
                try:
                    abort_t = float(e.split("s]")[0].strip("[ "))
                except ValueError:
                    pass
                break
        d = z["dbg"]
        d = d[d.shape[1] > 52 and np.isfinite(d[:, 52])] if d.ndim == 2 else d
        if d.ndim != 2 or d.shape[1] < 53:
            continue
        t = d[:, 0]
        m = (t >= a) & (t <= (min(b, abort_t) if abort_t else b))
        if m.sum() < 10:
            continue
        e = np.hypot(d[m, 50] - d[m, 47], d[m, 51] - d[m, 48])
        ok = (not bool(z["aborted"])) and abort_t is None
        rows.append((os.path.relpath(f, A), ok, t[m][-1] - t[m][0], e.max(), np.sqrt((e**2).mean()),
                     [longest_above(t[m], e, L) for L in LIMS], abort_t, str(z["reason"])[:60]))

print(f"{'run':58s} {'outcome':10s} {'contact s':>9s} {'max mm':>7s} {'rms mm':>7s}   trips at " +
      " ".join(f"{int(L*1e3):>4d}mm" for L in LIMS))
for r in rows:
    print(f"{r[0]:58s} {'complete' if r[1] else 'ABORT':10s} {r[2]:9.1f} {r[3]*1e3:7.1f} {r[4]*1e3:7.1f}   " +
          " ".join(f"{'-' if x is None else 'TRIP':>6s}" for x in r[5]) + ("" if r[1] else f"   {r[7]}"))
print()
for i, L in enumerate(LIMS):
    good = [r for r in rows if r[1]]
    bad = [r for r in rows if not r[1]]
    print(f"{int(L*1e3)} mm: completed runs tripped {sum(r[5][i] is not None for r in good)}/{len(good)}, "
          f"aborted runs tripped {sum(r[5][i] is not None for r in bad)}/{len(bad)}")
