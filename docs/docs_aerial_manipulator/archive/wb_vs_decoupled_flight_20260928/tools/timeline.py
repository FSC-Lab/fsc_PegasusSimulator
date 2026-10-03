"""Per-flight timeline: control mode edges, planner / EE-trajectory status, trajectory settings, notable rosout lines.
t = 0 at the first recorded message of the bag (receive clock)."""
import sys, numpy as np, os
DATA = os.environ.get("AM_NPZ", "/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/bf8404ab-c60b-4598-9acf-e8041f10c130/scratchpad/npz")
for nm in sys.argv[1:]:
    d = np.load(f"{DATA}/{nm}.npz", allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    print(f"\n==================== {nm}   t0 = {t0:.3f}")
    for mk in ("wbmode", "gmode"):
        if f"{mk}__recv" in d:
            last = None
            for ti, v in zip(d[f"{mk}__recv"] - t0, d[f"{mk}__data"]):
                if v != last: print(f"  {ti:7.2f} {mk}: {v}"); last = v
    for k in ("pl_status", "ee_status", "ctype"):
        if f"{k}__recv" in d:
            last = None
            for ti, v in zip(d[f"{k}__recv"] - t0, d[f"{k}__data"]):
                if v != last: print(f"  {ti:7.2f} {k}: {v}"); last = v
    for k in ("ee_info2", "ee_path", "ee_start"):
        if f"{k}__recv" in d:
            for i, ti in enumerate(d[f"{k}__recv"] - t0):
                if k == "ee_start":
                    p = [d[f"ee_start__pose.position.{c}"][i] for c in "xyz"]; print(f"  {ti:7.2f} ee_start pos {np.round(p,4)}")
                else:
                    x = d[f"{k}__data"][i]; x = x[~np.isnan(x)]
                    print(f"  {ti:7.2f} {k} len {len(x)}: {np.round(x[:24],4)}")
    if "rosout__recv" in d:
        for ti, n_, m in zip(d["rosout__recv"] - t0, d["rosout__name"], d["rosout__msg"]):
            s = str(m).replace("\n", " ")
            if any(w in s.lower() for w in ("direct", "safety", "abort", "watchdog", "refus", "trip", "fail", "warn", "stale", "trajectory", "circle", "plan", "gate")):
                print(f"  {ti:7.2f} rosout {n_}: {s[:230]}")
