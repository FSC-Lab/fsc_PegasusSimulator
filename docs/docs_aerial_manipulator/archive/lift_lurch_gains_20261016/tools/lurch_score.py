#!/usr/bin/env python3
"""Score the 2026-10-16 lift/release-lurch gain test (pnp_mission_v2.py recordings).

    PYTHONNOUSERSITE=1 /usr/bin/python3 lurch_score.py ../runs/A1.npz ...    (or no args: every run, by variant)

Per flight: completed; the CoM excursion (the law's own x_c - x_cd, debug [48:51] - [45:48]) over the 6 s
after the LIFT (exit_pick start) and after the RELEASE (gripper_open start) -- peak horizontal and vertical --
and the pitch swing (p-p) there; the carry's roll / pitch oscillation (0.3-2 Hz rms, Drop Start -> drop); the
1.2-1.8 Hz pitch band over all of DIRECT (the mode the softer pick-and-place attitude gains were chosen for);
peak tilt; where the payload came to rest (offset from the tray centre (-1, -1), tilt).
Recorder columns: dbg = [t, step, wb_control_debug...] (debug index i at column i + 2); odom = [t, p(3), v(3), q(wxyz), ...].
"""
import glob
import os
import sys

import numpy as np
from scipy.signal import butter, filtfilt

HERE = os.path.dirname(os.path.abspath(__file__))
TRAY_C = np.array([-1.0, -1.0])


def band(t, x, lo, hi):
    tu = np.arange(t[0], t[-1], 0.01)
    xu = np.interp(tu, t, x)
    b, a = butter(2, [lo / 50.0, hi / 50.0], btype="band")
    return filtfilt(b, a, xu)


def rp(q):
    w, x, y, z = q.T
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    return roll, pitch, tilt


def score(path):
    d = np.load(path, allow_pickle=True)
    marks = {str(n): float(t) for n, t in zip(d["marks_name"], d["marks_t"])}
    dbg = np.asarray([r for r in d["dbg"] if len(r) >= 53], dtype=float) if d["dbg"].dtype == object else d["dbg"].astype(float)
    td = dbg[:, 0]
    e = dbg[:, 50:53] - dbg[:, 47:50]                 # x_c - x_cd
    o = d["odom"]; to = o[:, 0]
    roll, pitch, tilt = rp(o[:, 7:11])
    r = {"run": os.path.basename(path)[:-4], "done": (not bool(d["aborted"])) and "execute_land:end" in marks}

    def win(a, span=6.0):
        m = (td >= a) & (td <= a + span)
        mo = (to >= a) & (to <= a + span)
        return (np.linalg.norm(e[m, :2], axis=1).max() * 1e3, np.abs(e[m, 2]).max() * 1e3,
                np.ptp(pitch[mo]))
    for key, mark in (("lift", "exit_pick:start"), ("rel", "gripper_open:start")):
        r[key] = win(marks[mark]) if mark in marks else (np.nan, np.nan, np.nan)
    if "go_to_place_start:start" in marks and "ready_place:end" in marks:
        a, b = marks["go_to_place_start:start"], marks["ready_place:end"]
        mo = (to >= a) & (to <= b)
        r["carry_roll"] = np.sqrt(np.mean(band(to[mo], roll[mo], 0.3, 2.0) ** 2))
        r["carry_pitch"] = np.sqrt(np.mean(band(to[mo], pitch[mo], 0.3, 2.0) ** 2))
    else:
        r["carry_roll"] = r["carry_pitch"] = np.nan
    a = marks.get("direct", to[0]); b = marks.get("execute_land:end", to[-1])
    mo = (to >= a) & (to <= b)
    r["mode15"] = np.sqrt(np.mean(band(to[mo], pitch[mo], 1.2, 1.8) ** 2))
    # worst 5 s window of the 1.2-1.8 Hz band, roll or pitch: a GROWING wobble shows here
    # even when the flight survives it (the whole-flight rms dilutes it)
    bp, br = band(to[mo], pitch[mo], 1.2, 1.8), band(to[mo], roll[mo], 1.2, 1.8)
    n5 = 500
    r["mode15_peak"] = max(np.sqrt(np.mean(np.r_[x[i:i + n5]] ** 2))
                           for x in (bp, br) for i in range(0, max(1, len(x) - n5), 50))
    r["tilt"] = tilt[mo].max()
    pf = d["payload_final"]
    r["pay_off"] = np.linalg.norm(pf[:2] - TRAY_C) * 1e3
    r["pay_tilt"] = rp(np.atleast_2d(pf[3:7]))[2][0]
    return r


def main():
    paths = sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "..", "runs", "*.npz")))
    rows = [score(p) for p in paths]
    hdr = (f"{'run':5s} {'done':4s} | {'LIFT horiz':>10s} {'vert':>5s} {'pitch':>6s} | {'RELEASE horiz':>13s} {'vert':>5s} "
           f"{'pitch':>6s} | {'carry roll':>10s} {'pitch':>6s} | {'1.5 Hz':>6s} {'worst5s':>7s} {'tilt':>5s} | {'basket off':>10s} {'tilt':>5s}")
    print(hdr)
    print(f"{'':5s} {'':4s} | {'mm':>10s} {'mm':>5s} {'deg pp':>6s} | {'mm':>13s} {'mm':>5s} {'deg pp':>6s} | "
          f"{'deg rms':>10s} {'rms':>6s} | {'deg rms':>6s} {'deg rms':>7s} {'max':>5s} | {'mm':>10s} {'deg':>5s}")
    for r in rows:
        print(f"{r['run']:5s} {str(r['done'])[0]:4s} | {r['lift'][0]:10.0f} {r['lift'][1]:5.0f} {r['lift'][2]:6.1f} | "
              f"{r['rel'][0]:13.0f} {r['rel'][1]:5.0f} {r['rel'][2]:6.1f} | {r['carry_roll']:10.2f} {r['carry_pitch']:6.2f} | "
              f"{r['mode15']:6.2f} {r['mode15_peak']:7.2f} {r['tilt']:5.1f} | {r['pay_off']:10.0f} {r['pay_tilt']:5.1f}")
    by = {}
    for r in rows:
        by.setdefault(r["run"][0], []).append(r)
    print("\nper variant (mean of its flights)")
    for v, rs in sorted(by.items()):
        f = lambda key, i=None: np.nanmean([x[key][i] if i is not None else x[key] for x in rs])
        print(f"  {v}: n={len(rs)} done {sum(x['done'] for x in rs)}  lift {f('lift', 0):.0f} mm / {f('lift', 2):.1f} deg  "
              f"release {f('rel', 0):.0f} mm / {f('rel', 2):.1f} deg  carry {f('carry_roll'):.2f}/{f('carry_pitch'):.2f}  "
              f"1.5Hz {f('mode15'):.2f} (worst 5 s {max(x['mode15_peak'] for x in rs):.2f})  tilt {f('tilt'):.1f}  basket {f('pay_off'):.0f} mm {f('pay_tilt'):.1f} deg")


if __name__ == "__main__":
    main()
