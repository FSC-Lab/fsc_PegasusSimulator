"""Two operator-visible yaw symptoms per flight, plus the yaw lever's time course.

  spool-up drift : heading change while ARMED and still on the ground before lift-off (z < 0.35 m), deg
                   (the skids' friction holds the vehicle until the yaw torque wins; negative = clockwise)
  touchdown drift: heading change from 1 s before to 2 s after the last time z drops below 0.35 m
  lever course   : tau_z / T (yaw_history.py rotor model) in 10 s medians of quasi-static DIRECT samples, mm
Writes ../analysis/yaw_events.json."""
import os, json
import numpy as np
from pd_common import load, mocap_pose, yaw_of, OUT, HIST, NPZ, RUNS
import yaw_history as YH


def flight(path):
    d, t0 = load(path)
    tm, P, Q, V = mocap_pose(d, t0)
    yaw = np.degrees(np.unwrap(yaw_of(Q)))
    z = P[:, 2]
    out = {}
    if "vstatus__arming_state" in d.files:
        tv = d["vstatus__recv"] - t0; armed = d["vstatus__arming_state"] == 2
        air = np.where(z > 0.35)[0]
        if len(air):
            tl = tm[air[0]]
            ta = tv[armed & (tv < tl)]
            if len(ta) and ta[0] < tl - 1.0:
                a = max(ta[0], tm[0])
                out["spool_s"] = float(tl - a)
                out["spool_drift_deg"] = float(np.interp(tl - 0.3, tm, yaw) - np.interp(a, tm, yaw))
        low = np.where((z[:-1] > 0.35) & (z[1:] <= 0.35))[0]
        if len(low):
            tt = tm[low[-1]]
            if tt + 2 < tm[-1]:
                out["touchdown_drift_deg"] = float(np.interp(tt + 2, tm, yaw) - np.interp(tt - 1, tm, yaw))
    key, spans = YH.direct_spans(d, t0)
    if spans and "act_motors__control" in d.files:
        tu = d["act_motors__recv"] - t0; U = d["act_motors__control"][:, :4]
        yr = np.gradient(np.radians(yaw), tm); sp = np.linalg.norm(V, axis=1)
        ok = np.all(np.isfinite(U), 1) & (U.min(1) > 0.05)
        inn = np.zeros(len(tu), bool)
        for a, b in spans:
            inn |= (tu >= a + 3) & (tu <= b - 0.3)
        q = ok & inn & (np.interp(tu, tm, sp) < 0.15) & (np.abs(np.interp(tu, tm, yr)) < np.radians(3))
        T, tz, *_ = YH.rotor_wrench(U[q])
        lev = tz / T * 1e3; tq = tu[q]
        course = []
        for a in np.arange(tq[0], tq[-1], 10.0) if len(tq) else []:
            s = (tq >= a) & (tq < a + 10)
            if s.sum() > 250:
                course.append((float(a), float(np.median(lev[s]))))
        out["lever_course"] = course
    return out


def main():
    rows = json.load(open(os.path.join(OUT, "yaw_history.json")))
    res = []
    for r in rows:
        nm = r["label"]; p = os.path.join(HIST, nm + ".npz")
        if not os.path.exists(p):
            p = os.path.join(NPZ, [k for k, v in RUNS.items() if v[2] == nm][0] + ".npz")
        e = flight(p); e["label"] = nm; e["lever_mm"] = r["lever_mm"]; res.append(e)
        lc = " ".join(f"{v:.1f}" for _, v in e.get("lever_course", []))
        sd = f"{e['spool_drift_deg']:+6.1f} deg in {e['spool_s']:4.1f} s" if "spool_drift_deg" in e else "        (bag starts airborne)"
        td = f"{e['touchdown_drift_deg']:+6.1f}" if "touchdown_drift_deg" in e else "   -  "
        print(f"{nm[14:]:52s} lever {r['lever_mm']:4.2f} mm | spool-up {sd} | touchdown {td} deg | 10 s course: {lc}")
    json.dump(res, open(os.path.join(OUT, "yaw_events.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
