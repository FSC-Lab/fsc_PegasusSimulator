"""Event timeline of one flight: mode, arm law, planner status, pick-and-place status (waiting lines collapsed),
and the /rosout lines that are not the once-a-second watch lines.  usage: timeline.py p1|p2|p3 [--all-log]"""
import os, sys, re
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
NPZ = os.environ.get("AM_NPZ", os.path.join(HERE, "..", "npz"))
nm = sys.argv[1]
d = np.load(f"{NPZ}/{nm}.npz")
t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def edges(key):
    out, last = [], None
    if key + "__recv" not in d.files:
        return out
    for ti, v in zip(d[key + "__recv"] - t0, d[key + "__data"]):
        v = str(v)
        if v != last:
            out.append((float(ti), v)); last = v
    return out


ev = []
for k in ("wbmode", "ctype", "actlaw", "pl_status", "mocap_status", "fused_status"):
    ev += [(t, k, v) for t, v in edges(k)]
last_wait = None
for t, v in edges("pp_status"):
    key = v.split(":")[0] if v.startswith("WAITING") else v
    if v.startswith("WAITING"):
        if key == last_wait:
            continue
        last_wait = key
    else:
        last_wait = None
    ev.append((t, "pp_status", v))
skip = ("posture PID", "4D attribution", "Subscribed to topic", "Loop health")
if "rosout__recv" in d.files:
    for t, n, m in zip(d["rosout__recv"] - t0, d["rosout__name"], d["rosout__msg"]):
        m = ANSI.sub("", str(m))
        if "--all-log" not in sys.argv and any(s in m for s in skip):
            continue
        ev.append((float(t), "LOG " + str(n).split(".")[-1], m))
for t, k, v in sorted(ev, key=lambda e: e[0]):
    print(f"{t:8.2f}  {k:28s} {v[:400]}")
