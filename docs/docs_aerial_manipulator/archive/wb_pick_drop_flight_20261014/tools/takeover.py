"""Pilot takeovers at landing, every recent hardware flight: when PX4 left OFFBOARD, what thrust the airframe actually
produced afterwards (body-z specific force from sensor_combined, low-passed 10 Hz, x mass / weight -> 'thrust/weight'),
when it disarmed and why (VehicleStatus.latest_disarming_reason), and how it ended (final tilt).
Writes ../analysis/takeover.json."""
import os, json, glob
import numpy as np
from scipy.signal import butter, filtfilt
from pd_common import load, mocap_pose, tilt_of, OUT, HIST, NPZ, RUNS

NAV = {14: "OFFBOARD", 15: "STAB", 1: "ALTCTL", 2: "POSCTL", 0: "MANUAL", 18: "AUTO_LAND", 12: "DESCEND", 4: "LOITER"}
WHY = {0: "standby", 1: "stick gesture", 2: "RC switch", 3: "internal cmd", 4: "external cmd (companion)", 7: "auto-disarm land",
       8: "auto-disarm preflight", 9: "kill switch", 10: "lockdown", 11: "failure detector", 12: "shutdown"}


def flight(path, label):
    d, t0 = load(path)
    if "vstatus__recv" not in d.files:
        return None
    tv = d["vstatus__recv"] - t0; nav = d["vstatus__nav_state"].astype(int); arm = d["vstatus__arming_state"].astype(int)
    why = d["vstatus__latest_disarming_reason"].astype(int)
    tm, P, Q, V = mocap_pose(d, t0)
    ts = d["sc__recv"] - t0; az = -d["sc__accelerometer_m_s2"][:, 2]
    fs = 1 / np.median(np.diff(d["sc__timestamp"] * 1e-6)); b, a = butter(2, 10 / (fs / 2)); azf = filtfilt(b, a, az) / 9.81
    dis = np.where((arm[:-1] == 2) & (arm[1:] == 1))[0]
    if not len(dis):
        return dict(label=label, note="never disarmed in the bag")
    k = dis[-1] + 1; t_dis = tv[k]
    # the last flight-mode change before the disarm
    ch = np.where(nav[1:] != nav[:-1])[0]
    ch = ch[tv[ch + 1] < t_dis]
    t_sw = float(tv[ch[-1] + 1]) if len(ch) else np.nan
    m_from = NAV.get(int(nav[ch[-1]]), str(nav[ch[-1]])) if len(ch) else "-"; m_to = NAV.get(int(nav[ch[-1] + 1]), "?") if len(ch) else "-"
    zf = lambda t: float(np.interp(t, tm, P[:, 2])); tf = lambda t: float(np.interp(t, tm, tilt_of(Q)))
    tw = lambda t: float(np.interp(t, ts, azf))
    # thrust/weight trace after the switch, 0.1 s steps
    trace = [(round(t - t_sw, 2), round(tw(t), 2), round(zf(t), 3)) for t in np.arange(t_sw - 0.2, min(t_dis + 0.6, ts[-1]), 0.1)] if np.isfinite(t_sw) else []
    return dict(label=label, switch=f"{m_from}->{m_to}", t_switch=t_sw, z_switch=zf(t_sw) if np.isfinite(t_sw) else None,
                t_disarm=float(t_dis), dt=float(t_dis - t_sw), z_disarm=zf(t_dis), why=WHY.get(int(why[k]), str(why[k])),
                why_before=WHY.get(int(why[k - 1]), str(why[k - 1])), tilt_end=tf(min(t_dis + 2.5, tm[-1])), trace=trace)


def main():
    rows = []
    for p in sorted(glob.glob(os.path.join(HIST, "*2026092[89]*.npz")) + glob.glob(os.path.join(HIST, "*202610*.npz"))):
        nm = os.path.basename(p)[:-4]; r = flight(p, nm)
        if r: rows.append(r)
    for k, v in RUNS.items():
        r = flight(os.path.join(NPZ, f"{k}.npz"), v[2])
        if r: rows.append(r)
    rows.sort(key=lambda r: r["label"][-15:])
    for r in rows:
        if "note" in r:
            print(f"{r['label'][14:]:46s} {r['note']}"); continue
        tw = [x[1] for x in r["trace"] if 0 <= x[0] <= r["dt"]]
        print(f"{r['label'][14:]:46s} {r['switch']:15s} at z {r['z_switch']:.2f} | disarm +{r['dt']:4.2f} s at z {r['z_disarm']:.2f}, "
              f"reason {r['why']:12s} (was {r['why_before']}) | thrust/weight after switch min {min(tw) if tw else float('nan'):.2f} "
              f"| final tilt {r['tilt_end']:5.1f} deg")
    json.dump(rows, open(os.path.join(OUT, "takeover.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
