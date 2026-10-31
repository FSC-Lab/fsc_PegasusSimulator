#!/usr/bin/env python3
"""Flight 145526, the DIRECT -> SAFETY handover: the baseline's attitude setpoint (tilt, yaw, collective)
against PX4's attitude, the body rates, the gyro/accel clipping, the arm's commanded torque and measured
current, the mocap guard's state + reason, the UDE estimate."""
import sys
import numpy as np
from vt_common import Bag, tilt_deg
b = Bag(sys.argv[1] if len(sys.argv) > 1 else "145526")
t0, t1, dt = 45.0, 48.6, 0.05
def near(t, v, tt):
    return v[np.clip(np.searchsorted(t, tt), 0, len(t)-1)]
def eul(q):  # w x y z (PX4 NED/FRD) -> roll pitch yaw deg
    w, x, y, z = q.T
    r = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    p = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
    yw = np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    return np.stack([r, p, yw], 1)
ts, qs = b.t("attsp_in"), b.f("attsp_in", "q_d"); es = eul(qs); ths = b.f("attsp_in", "thrust_body")[:, 2]
ta, qa = b.t("vatt"), b.f("vatt", "q"); ea = eul(qa)
tg, gy = b.t("sc"), np.degrees(b.f("sc", "gyro_rad")); gc = b.f("sc", "gyro_clipping"); ac = b.f("sc", "accelerometer_clipping")
tu = b.t("ude"); ui = np.stack([b.f("ude", f"integral.{c}") for c in "xyz"], 1) if "ude__integral.x" in b.z.files else None
tgd, gd = b.t("guard"), b.f("guard", "data")
tj = b.t("js"); eff = b.f("js", "effort")
tc = b.t("tcmd"); tce = b.f("tcmd", "effort")
print("guard data row at 46.9..47.2:")
for tt in (46.95, 47.0, 47.05, 47.14, 48.0): print(f"  {tt}: ", np.round(near(tgd, gd, tt), 3))
print("   t   | sp roll pitch yaw  thr | att roll pitch yaw | gyro p q r dps      | clip g/a | js effort(raw) j2 j3 j1 j4 | tcmd j2 j3 j1 j4")
for tt in np.arange(t0, t1, dt):
    w = (tg >= tt - dt/2) & (tg < tt + dt/2)
    g = gy[w]; gm = g[np.argmax(np.abs(g).max(1))] if len(g) else [np.nan]*3
    s = f"{tt:6.2f} | " + " ".join(f"{x:+6.1f}" for x in near(ts, es, tt)) + f" {near(ts, ths, tt):+.2f} | "
    s += " ".join(f"{x:+6.1f}" for x in near(ta, ea, tt)) + " | " + " ".join(f"{x:+6.0f}" for x in gm)
    s += f" | {int(gc[w].max()) if w.any() else -1}/{int(ac[w].max()) if w.any() else -1} | " + " ".join(f"{x:+6.0f}" for x in near(tj, eff, tt)[:4])
    s += " | " + " ".join(f"{x:+5.2f}" for x in near(tc, tce, tt)[:4])
    print(s)
