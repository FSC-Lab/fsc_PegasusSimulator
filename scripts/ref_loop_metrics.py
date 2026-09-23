"""Score scripts/ref_loop_cells.sh: does each reference SHAPING term have a per-loop optimum?

  scripts/ref_loop_metrics.py [dir]      (default ~/flight_bags/ref_series2)

Reads the ref_probe CSVs (t, theta_ref, theta, V, tilt, ...) and prints, per term and per
loop, the quantity that term exists to set -- the corridor peak for `ttr`, the
back-transition peak and its reversals for `decel_rate` and `th_slew_bt` -- so the two
columns can be compared for whether the BEST value moves, not merely whether each flies.
"""
import os, re, sys
import numpy as np
S = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/flight_bags/ref_series2")

def m(tag):
    c, l = f"{S}/{tag}.csv", f"{S}/{tag}.log"
    if not os.path.exists(c) or os.path.getsize(c) < 200: return None
    D = np.genfromtxt(c, delimiter=",", names=True)
    if D.size < 100: return None
    t, V, th, ref = D["t"], D["V"], D["theta"], D["theta_ref"]
    cor, bt = (V > 2.0) & (V < 14.0) & (t < 55), (t > 60.2) & (V > 1.0)
    o = {}
    o["cor_pk"] = th[cor].max() if cor.any() else np.nan
    o["cor_e"] = np.sqrt(((th - ref)[cor] ** 2).mean()) if cor.any() else np.nan
    if bt.any():
        o["bt_pk"], o["bt_e"] = th[bt].max(), np.sqrt(((th - ref)[bt] ** 2).mean())
        x = th[bt]; rev = 0; d = 0; ext = x[0]          # excursions > 3 deg, CLAUDE.md's rule
        for v in x:
            if d >= 0 and v > ext: ext = v
            elif d <= 0 and v < ext: ext = v
            if abs(v - ext) > 3.0:
                dn = 1 if v > ext else -1
                if d and dn != d: rev += 1
                d, ext = dn, v
        o["bt_rev"] = rev
    else:
        o["bt_pk"] = o["bt_e"] = o["bt_rev"] = np.nan
    txt = open(l).read() if os.path.exists(l) else ""
    g = lambda p, d=np.nan: (float(re.search(p, txt).group(1)) if re.search(p, txt) else d)
    o["fails"], o["V_end"] = g(r"solve fails (\d+)/", np.nan), g(r"final: z=[\d.-]+ V=([\d.-]+)")
    o["z_min"] = min([float(x) for x in re.findall(r"^\s+[\d.]+\s+([\d.]+)\s", txt, re.M)] or [np.nan])
    return o

def table(title, note, pre, vals, cols, hdr):
    print("\n" + title); print("  " + note); print("-" * 88)
    print("%-10s" % "value" + "".join("%13s" % f"{h} {k}" for h in hdr for k in ("geo", "indi")))
    for v in vals:
        line = "%-10s" % v
        for c in cols:
            for k in ("geo", "indi"):
                r = [m(f"{pre}{v}_{k}_s{s}") for s in (1, 2)]; r = [x for x in r if x]
                a = np.array([x[c] for x in r], float) if r else np.array([np.nan])
                line += "%13s" % ("--" if not r else ("%.2f" % np.nanmean(a) if c not in ("fails", "bt_rev") else "%.0f" % np.nansum(a)))
        print(line)

table("1. `ttr` -- the FORWARD speed ramp (sets the corridor's AoA peak)",
      "prediction: INDI crosses 2.2 deg flatter, so if the term is loop-dependent its optimum moves",
      "ttr", ["6", "8", "10", "14"], ["cor_pk", "cor_e", "fails"], ["corridor pk", "cor e rms", "fails"])
table("2. `decel_rate` -- the BACK-TRANSITION speed ramp (documented boundary: 2.5 crashes)",
      "each loop flies its OWN shipped reference (INDI at flare 8)",
      "dec", ["1.5", "2.0", "2.5"], ["bt_pk", "bt_rev", "V_end", "z_min"], ["BT peak", "revs", "V end", "z min"])
table("3. `th_slew_bt` -- the reference PITCH ramp rate",
      "its stated reason is that the OUTER solver overshoots a step -- an outer property, so predict NO move",
      "ths", ["4", "8", "16"], ["bt_pk", "bt_e", "bt_rev"], ["BT peak", "e rms", "revs"])
