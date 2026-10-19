"""Quick health checks of the 2026-10-16 (and, for reference, 10-15) flights: pack voltage, the 1.2-2 Hz pitch band in the
carry (the k_R/k_w 2.134/1.567 Isaac caveat), peak tilt, the touchdown, the lift / release CoM peaks, obj_0's feed.
usage: health16.py"""
import os
import numpy as np
import pd15_common as P
from pd15_common import band, rms, rosout, tilt_of, mocap_pose

OLD = os.path.join(P.NPZ, "..", "..", "wb_pick_drop_flight_20261015", "npz")


def one(npz, tag):
    d = np.load(npz)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
    win = P.leg_windows(d, t0)
    ta, rr, pp, yy = P.px4_rpy(d, t0)
    tw, D = P.wbdebug(d, t0)
    tb = d["batt__recv"] - t0; V = d["batt__voltage_v"]
    dir_ = D[:, 0] > 0.5 if D.shape[1] > 0 else None
    tilt = np.degrees(np.arccos(np.clip(np.cos(rr) * np.cos(pp), -1, 1)))
    a, b = win["go_to_place_start"][0], win["execute_place (approach)"][1]
    m = (ta >= a) & (ta < b)
    _, p15 = band(np.degrees(pp[m]), ta[m], 1.2, 2.0)
    _, r15 = band(np.degrees(rr[m]), ta[m], 1.2, 2.0)
    _, p05 = band(np.degrees(pp[m]), ta[m], 0.3, 1.0)
    com = lambda lab: np.linalg.norm(D[(tw >= win[lab][0]) & (tw < win[lab][1]), 45:48] - D[(tw >= win[lab][0]) & (tw < win[lab][1]), 48:51], axis=1).max() * 1e3
    land_a, land_b = win["execute_land"]
    after = ta > land_b
    to, Po, Qo, _ = mocap_pose(d, t0, "obj_vrpn")
    dto = np.diff(to)
    print(f"{tag:9s} V {V[(tb > 15) & (tb < 25)].mean():5.2f}->{V[tb > land_a - 5][:20].mean():5.2f}  "
          f"tilt max DIRECT {tilt[(ta > win['go_to_start'][0]) & (ta < land_b)].max():4.1f}  after-land max {tilt[after].max() if after.any() else np.nan:5.1f}"
          f"  carry pitch 0.3-1 Hz rms {rms(p05):.2f}  1.2-2 Hz pitch/roll rms {rms(p15):.2f}/{rms(r15):.2f}"
          f"  CoM peak lift {com('exit_pick (up the vertical margin)'):4.0f} release {com('exit_place (up the vertical margin)'):4.0f} mm"
          f"  obj_0 {1 / np.median(dto):4.0f} Hz max gap {dto.max() * 1e3:4.0f} ms")


if __name__ == "__main__":
    for r in ("e1", "d1", "d2"):
        one(os.path.join(P.NPZ, f"{r}.npz"), f"1016 {r}")
    for r in ("e1", "d1"):
        one(os.path.join(OLD, f"{r}.npz"), f"1015 {r}")
