"""Each pilot takeover from switch to touchdown: climb after the switch (stick above hover), sink rate and tilt at
contact (first z <= 0.27 m), yaw turned while airborne. Uses takeover.json + mocap. -> ../analysis/touchdown.json"""
import os, json
import numpy as np
from pd_common import load, mocap_pose, tilt_of, yaw_of, OUT, HIST, NPZ, RUNS

rows = json.load(open(os.path.join(OUT, "takeover.json")))
res = []
print(f"{'flight':16s} {'switch z':>8s} {'climb':>6s} {'peak z':>6s} {'sink at contact':>15s} {'tilt at contact':>15s} {'yaw turned':>10s} {'final tilt':>10s}")
for r in rows:
    if "trace" not in r or "180228" in r["label"]:
        continue
    nm = r["label"]
    p = os.path.join(HIST, nm + ".npz")
    if not os.path.exists(p):
        p = os.path.join(NPZ, [k for k, v in RUNS.items() if v[2] == nm][0] + ".npz")
    d, t0 = load(p)
    tm, P, Q, V = mocap_pose(d, t0)
    a = r["t_switch"]
    s = (tm >= a) & (tm <= r["t_disarm"] + 1.0)
    t, z, vz, tl = tm[s], P[s, 2], V[s, 2], tilt_of(Q[s])
    yw = np.degrees(np.unwrap(yaw_of(Q[s])))
    c = np.where(z <= 0.27)[0]
    if not len(c):
        continue
    c = c[0]
    pk = z[:c + 1].max()
    row = dict(flight=nm[-15:], z_switch=r["z_switch"], climb=float(pk - z[0]), peak=float(pk), sink=float(-vz[max(0, c - 3):c + 1].min()),
               tilt_contact=float(tl[c]), yaw_turn=float(yw[c] - yw[0]), tilt_end=r["tilt_end"], t_contact=float(t[c] - a))
    res.append(row)
    print(f"{row['flight']:16s} {row['z_switch']:8.2f} {row['climb']:+6.2f} {row['peak']:6.2f} {row['sink']:11.2f} m/s {row['tilt_contact']:11.1f} deg "
          f"{row['yaw_turn']:+7.1f} deg {row['tilt_end']:7.1f} deg")
json.dump(res, open(os.path.join(OUT, "touchdown.json"), "w"), indent=1)
