"""Airframe vibration per flight: sensor_combined accelerometer / gyro rms in 15-95 Hz (the rotor band) over every steady
DIRECT stretch (|v| < 0.15 m/s for >= 4 s, up to 20 s each), median over a flight.  A loose arm usually shows here first.
Writes ../analysis/vibration.json and prints one line per flight."""
import os, json
import numpy as np
from scipy.signal import welch
from pd_common import load, mocap_pose, OUT, HIST, NPZ, RUNS
import yaw_history as YH


def stretches(d, t0, spans):
    tm, P, Q, V = mocap_pose(d, t0); sp = np.linalg.norm(V, axis=1)
    out = []
    for a, b in spans:
        tt = np.arange(a + 3, b - 1, 0.5)
        if len(tt) < 2:
            continue
        ok = np.interp(tt, tm, sp) < 0.15
        start = None
        for i, o in enumerate(ok):
            if o and start is None:
                start = tt[i]
            if (not o or i == len(ok) - 1) and start is not None:
                if tt[i] - start >= 4:
                    out.append((start, min(tt[i], start + 20)))
                start = None
    return out


def flight(path):
    d, t0 = load(path)
    _, spans = YH.direct_spans(d, t0)
    ts = d["sc__recv"] - t0; A = d["sc__accelerometer_m_s2"]; G = d["sc__gyro_rad"]
    fs = 1 / np.median(np.diff(d["sc__timestamp"] * 1e-6))
    res = []
    for a, b in stretches(d, t0, spans):
        s = (ts >= a) & (ts < b)
        if s.sum() < 4 * 1024:
            continue
        f, Pa = welch(A[s], fs=fs, nperseg=1024, axis=0); _, Pg = welch(G[s], fs=fs, nperseg=1024, axis=0)
        band = (f > 15) & (f < 95)
        ra = np.sqrt(np.trapz(Pa[band], f[band], axis=0)); rg = np.degrees(np.sqrt(np.trapz(Pg[band], f[band], axis=0)))
        res.append((np.hypot(ra[0], ra[1]), ra[2], np.linalg.norm(rg)))
    if not res:
        return None
    r = np.array(res)
    return dict(fs=float(fs), n=len(res), acc_xy=float(np.median(r[:, 0])), acc_z=float(np.median(r[:, 1])),
                gyro_dps=float(np.median(r[:, 2])), acc_xy_max=float(r[:, 0].max()))


def main():
    rows = json.load(open(os.path.join(OUT, "yaw_history.json")))
    out = []
    for r in rows:
        nm = r["label"]
        p = os.path.join(HIST, nm + ".npz")
        if not os.path.exists(p):
            p = os.path.join(NPZ, [k for k, v in RUNS.items() if v[2] == nm][0] + ".npz")
        v = flight(p)
        if v is None:
            continue
        v["label"] = nm; out.append(v)
        print(f"{nm:58s} fs {v['fs']:3.0f}  stretches {v['n']:2d}  acc_xy {v['acc_xy']:.2f} (max {v['acc_xy_max']:.2f})  "
              f"acc_z {v['acc_z']:.2f} m/s^2  gyro {v['gyro_dps']:.2f} dps")
    json.dump(out, open(os.path.join(OUT, "vibration.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
