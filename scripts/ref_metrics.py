"""Score a scripts/ref_series.sh directory: does each inner loop fly the FULL reference,
and is the SHAPED part of that reference still the same manoeuvre when the loop changes?

  scripts/ref_metrics.py [dir]        (default ~/flight_bags/ref_series)
"""
import sys, os, glob, re
import numpy as np
S = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/flight_bags/ref_series")

def metrics(tag):
    c, l = f"{S}/rs_{tag}.csv", f"{S}/rs_{tag}.log"
    if not os.path.exists(c) or os.path.getsize(c) < 200: return None
    D = np.genfromtxt(c, delimiter=",", names=True)
    if D.size < 100: return None
    t, V, th, ref = D["t"], D["V"], D["theta"], D["theta_ref"]
    out = {}
    cor = (V > 2.0) & (V < 14.0) & (t < 55)
    bt  = (t > 60.2) & (V > 1.0)
    out["cor_pk"] = th[cor].max() if cor.any() else float("nan")
    out["cor_e"]  = np.sqrt(((th-ref)[cor]**2).mean()) if cor.any() else float("nan")
    if bt.any():
        out["bt_pk"], out["bt_ref"] = th[bt].max(), ref[bt].max()
        # CLAUDE.md's definition: pitch REVERSALS of more than 3 deg through the crossing.
        # A plain sign-change count reads the noise (it gave 109 vs 72 where the traces show
        # 5 and 8), so walk the trace and only close out an excursion once it has turned by 3 deg.
        x = th[bt]; rev = 0; direction = 0; ext = x[0]
        for v in x:
            if direction >= 0 and v > ext: ext = v
            elif direction <= 0 and v < ext: ext = v
            if abs(v - ext) > 3.0:
                d_new = 1 if v > ext else -1
                if direction != 0 and d_new != direction: rev += 1
                direction = d_new; ext = v
        out["bt_rev"] = rev
        out["bt_e"] = np.sqrt(((th-ref)[bt]**2).mean())
    else:
        out["bt_pk"] = out["bt_ref"] = out["bt_rev"] = out["bt_e"] = float("nan")
    txt = open(l).read() if os.path.exists(l) else ""
    m = re.search(r"solve fails (\d+)/", txt);            out["fails"] = int(m.group(1)) if m else -1
    m = re.search(r"worst displacement from the held point ([\d.]+)", txt)
    out["hold_m"] = float(m.group(1)) if m else float("nan")
    m = re.search(r"final: z=([\d.-]+) V=([\d.-]+)", txt)
    out["z_end"], out["V_end"] = (float(m.group(1)), float(m.group(2))) if m else (float("nan"),)*2
    out["z_min"] = min([float(x) for x in re.findall(r"^\s+[\d.]+\s+([\d.]+)\s", txt, re.M)] or [float("nan")])
    return out

def table(title, rows, cols, hdr):
    print("\n" + title); print("-" * 96)
    print("%-22s" % "cell" + "".join("%12s" % h for h in hdr))
    for name, tags in rows:
        vals = [metrics(t) for t in tags]; vals = [v for v in vals if v]
        if not vals: print("%-22s  (no runs)" % name); continue
        line = "%-22s" % ("%s  [n=%d]" % (name, len(vals)))
        for c in cols:
            a = np.array([v[c] for v in vals], float)
            line += "%12s" % ("%.2f" % np.nanmean(a) if c != "fails" else "%d" % int(np.nansum(a)))
        print(line)

seeds = [1,2,3,4,5,6]
table("1. THE FULL REFERENCE -- straight mission + back-transition, 6 seeds",
      [("geometric",  [f"full_geo_s{s}"  for s in seeds]),
       ("INDI full",  [f"full_indi_s{s}" for s in seeds])],
      ["cor_pk","cor_e","bt_pk","bt_rev","z_min","V_end","fails"],
      ["corridor pk","cor e rms","BT peak","BT revs","z min","V end","fails"])

table("2. THE FLARE REFERENCE -- does the 2026-09-13 cap still mean the same thing?  3 seeds",
      [("geometric, cap 8 (flown)", [f"flare_geo8_s{s}"  for s in (1,2,3)]),
       ("INDI, cap 8 (flown)",      [f"flare_indi8_s{s}" for s in (1,2,3)]),
       ("INDI, cap 7",              [f"flare_indi7_s{s}" for s in (1,2,3)]),
       ("INDI, cap 6",              [f"flare_indi6_s{s}" for s in (1,2,3)])],
      ["bt_ref","bt_pk","bt_rev","bt_e","z_min","fails"],
      ["ref peak","delivered","revs","e rms","z min","fails"])

print("\n3. THE HOVER HOLD -- the geometric loop's standing pitch was part of its braking")
print("-" * 96)
print("%-28s%16s%16s" % ("hold_k", "geometric [m]", "INDI [m]"))
for h in ("0.5", "0.8", "1.2"):
    g, i = metrics(f"hold_geo_k{h}"), metrics(f"hold_indi_k{h}")
    print("%-28s%16s%16s" % (h + ("  (flown)" if h == "0.5" else ""),
          "%.2f" % g["hold_m"] if g else "-", "%.2f" % i["hold_m"] if i else "-"))
