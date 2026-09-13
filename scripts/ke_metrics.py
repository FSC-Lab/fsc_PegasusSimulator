#!/usr/bin/env python3
"""Back-transition metrics from ke_sweep.sh traces (--every 0.1): peak pitch/AoA, pitch-reference
error through the stall crossing, reversals, solve failures, elevator saturation, altitude band,
stop time, peak roll; a SUMMARY line at the end. usage: ke_metrics.py <dir> [t_decel]
(2026-09-13; the "revs"/"fails" columns of the knife-edge tables in CLAUDE.md)."""
import sys, re, glob, os, math
d = sys.argv[1]; t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
rows = []
for f in sorted(glob.glob(os.path.join(d, "s*.txt")), key=lambda p: int(re.search(r"s(\d+)\.txt", p).group(1))):
    sd = int(re.search(r"s(\d+)\.txt", f).group(1))
    T = []
    for line in open(f):
        m = re.match(r"^\s+(\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+)\s+([+-]\d+\.\d+)\s+([+-]\d+\.\d+)\s+([+-]\d+\.\d+)\s+([+-]\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+).*ok=(\d).*thR=\s*([+-]\d+\.\d+).*elv=([+-]\d+\.\d+)", line)
        if m:
            t, z, V, vcmd, tilt, tcmd, th, aoa, roll, yaw, wing, thr, ms, ok, thR, elv = m.groups()
            T.append(dict(t=float(t), z=float(z), V=float(V), th=float(th), aoa=float(aoa), roll=float(roll),
                          wing=float(wing), thr=float(thr), ok=int(ok), thR=float(thR), elv=float(elv), tilt=float(tilt)))
    if not T:
        print(f"s{sd}: no rows"); continue
    fin = T[-1]
    bt = [r for r in T if t0 <= r["t"] <= t0 + 20]
    if not bt:
        print(f"s{sd}: no BT rows"); continue
    # crossing window: while V between 15.5 and 9
    cross = [r for r in bt if 9.0 <= r["V"] <= 15.5]
    pk_th = max(r["th"] for r in bt); pk_aoa = max(r["aoa"] for r in bt)
    err = [r["th"] - r["thR"] for r in cross]
    rms = math.sqrt(sum(e * e for e in err) / max(len(err), 1)); pk_err = max(abs(e) for e in err) if err else 0
    # oscillation: number of pitch reversals with amplitude > 3 deg in the crossing
    ths = [r["th"] for r in cross]; rev = 0; ext = ths[0] if ths else 0; dirn = 0
    for a, b in zip(ths, ths[1:]):
        d_ = b - a
        if d_ * dirn < 0 and abs(b - ext) > 3.0:
            rev += 1; ext = a
        if d_ != 0: dirn = 1 if d_ > 0 else -1
    fails = sum(1 for r in bt if r["ok"] == 0)
    elv_sat = sum(1 for r in cross if abs(r["elv"]) >= 0.99)
    minz = min(r["z"] for r in bt); maxz = max(r["z"] for r in bt)
    t_stop = next((r["t"] - t0 for r in bt if r["V"] < 3.0), None)
    peak_roll = max(abs(r["roll"]) for r in bt)
    rows.append((sd, fin["z"], pk_th, pk_aoa, rms, pk_err, rev, fails, elv_sat, minz, maxz, t_stop, peak_roll))
    print(f"s{sd:2d} final z={fin['z']:5.1f}  pkpitch {pk_th:5.1f} pkAoA {pk_aoa:5.1f}  err rms {rms:4.2f} pk {pk_err:4.1f}  revs {rev:2d}  fails {fails:2d} elvsat {elv_sat:2d}  z[{minz:5.1f},{maxz:5.1f}]  t_stop {t_stop if t_stop is None else round(t_stop,1)}  roll {peak_roll:5.1f}")
if rows:
    ok = sum(1 for r in rows if r[1] > 20)
    print(f"SUMMARY {os.path.basename(d.rstrip('/'))}: {ok}/{len(rows)} | worst pkpitch {max(r[2] for r in rows):5.1f} pkAoA {max(r[3] for r in rows):5.1f} | err rms mean {sum(r[4] for r in rows)/len(rows):4.2f} worst pk {max(r[5] for r in rows):4.1f} | revs mean {sum(r[6] for r in rows)/len(rows):4.1f} max {max(r[6] for r in rows)} | fails sum {sum(r[7] for r in rows)} elvsat sum {sum(r[8] for r in rows)} | minz {min(r[9] for r in rows):5.1f} maxz {max(r[10] for r in rows):5.1f} | t_stop max {max((r[11] or 99) for r in rows):4.1f} | roll worst {max(r[12] for r in rows):4.1f}")
