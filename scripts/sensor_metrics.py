#!/usr/bin/env python3
"""The sensor sweep's merged table: one row per cell from the sens_<cell> sweeps.

The summary columns come from est_metrics' own summary.txt; the two extra ones -- the LATERAL
(y) and FORE-AFT (x) bob force in the steady orbit -- are recomputed per seed from the gate's
`--log` CSVs, because the split is what the sweep is for: the y bob is the channel the
cancellation acts on (the tank's only moment arm is r_x, so M = r_x F_y), the x bob has no arm
and is the LOOP's own response to a degraded state source.

Usage: scripts/sensor_metrics.py <cell> [<cell> ...]   (EST_DIR overrides ~/flight_bags/est_sweeps)
"""
import glob
import importlib.util
import os
import re
import sys

import numpy as np

E = os.path.expanduser(os.environ.get("EST_DIR", "~/flight_bags/est_sweeps"))
_spec = importlib.util.spec_from_file_location(
    "est_metrics", os.path.join(os.path.dirname(os.path.abspath(__file__)), "est_metrics.py"))
_em = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_em)


def per_axis(cell):
    """(x-bob, y-bob) rms [N] in the steady orbit, averaged over the cell's seeds."""
    ox, oy = [], []
    for f in sorted(glob.glob(os.path.join(E, f"sens_{cell}", "*_pon_s*.csv"))):
        try:
            o = _em.parse_csv(f, _em.parse_txt(f[:-4] + ".txt"))
        except Exception:
            continue
        if "orbit_bobFx" in o:
            ox.append(o["orbit_bobFx"]); oy.append(o["orbit_bobFy"])
    return (float(np.mean(ox)), float(np.mean(oy))) if ox else (float("nan"),) * 2


cells = sys.argv[1:]
print(f"{'cell':16s} {'kind':9s} {'seeds':6s} {'fails':11s} {'stop m':11s} {'y-bob':6s} {'x-bob':6s} "
      f"{'hits':11s} {'roll sd':11s} {'u_ad':11s} {'raw corr':11s} {'pred corr':11s} {'pred25':11s}")
for c in cells:
    p = os.path.join(E, f"sens_{c}", "summary.txt")
    if not os.path.exists(p):
        print(f"{c:16s} (no summary)"); continue
    x, y = per_axis(c)
    for l in [l for l in open(p) if re.match(r"^(imu-)?(steso|leso|mdob|mdob22|pwc)\s", l)]:
        f = l.split()
        # kind pred seeds fails stop bob_turns bob_orbit bob_stop hits roll_orbit roll_stop uad rawcorr predcorr pred25 worstroll
        if len(f) < 16:
            print(f"{c:16s} {l.strip()[:120]}"); continue
        print(f"{c:16s} {f[0]:9s} {f[2]:6s} {f[3]:11s} {f[4]:11s} {y:6.3f} {x:6.3f} "
              f"{f[8]:11s} {f[9]:11s} {f[11]:11s} {f[12]:11s} {f[13]:11s} {f[14]:11s}")
