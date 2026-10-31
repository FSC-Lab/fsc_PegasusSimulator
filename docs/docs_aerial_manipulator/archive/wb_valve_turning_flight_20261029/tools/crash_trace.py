#!/usr/bin/env python3
"""Flight 145526 from the q3-guard abort through the watchdog, the mocap fault and the pilot's
takeover: drone mocap (as forwarded) vs raw VRPN vs the estimator's odometry, PX4 tilt, motor
commands, the IMU's specific force, the arm joints, the gripper, the valve angle and the guard state."""
import sys
import numpy as np
from vt_common import Bag, tilt_deg
tag = sys.argv[1] if len(sys.argv) > 1 else "145526"
t0, t1, dt = (float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])) if len(sys.argv) > 4 else (44.5, 56.5, 0.1)
b = Bag(tag)
def at(key, field, tt):
    t = b.t(key); v = b.f(key, field); j = np.clip(np.searchsorted(t, tt), 0, len(t)-1); return v[j], t[j]
tm = b.t("mocap"); pm = np.stack([b.f("mocap", f"pose.position.{c}") for c in "xyz"], 1)
tv = b.t("vrpn"); vk = [k for k in b.z.files if k.startswith("vrpn__pose")]
pv = np.stack([b.f("vrpn", f"pose.position.{c}") for c in "xyz"], 1) if "vrpn__pose.position.x" in b.z.files else None
to = b.t("odom"); po = np.stack([b.f("odom", f"pose.pose.position.{c}") for c in "xyz"], 1)
vo = np.stack([b.f("odom", f"twist.twist.linear.{c}") for c in "xyz"], 1)
ta = b.t("vatt"); qa = b.f("vatt", "q"); tilt = tilt_deg(qa[:, 0], qa[:, 1], qa[:, 2], qa[:, 3])
yaw = np.degrees(np.arctan2(2*(qa[:, 0]*qa[:, 3]+qa[:, 1]*qa[:, 2]), 1-2*(qa[:, 2]**2+qa[:, 3]**2)))
tmo = b.t("motors"); mo = b.f("motors", "control")[:, :4]
ts = b.t("sc"); acc = b.f("sc", "accelerometer_m_s2"); an = np.linalg.norm(acc, axis=1)
tj = b.t("js"); qj = np.degrees(b.f("js", "position")); names = b.f("js", "name")[0]
tg = b.t("guard"); gs = b.f("guard", "data")[:, 0]
tob = b.t("obj"); qo = [b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]
vyaw = np.degrees(np.unwrap(np.arctan2(2*(qo[0]*qo[3]+qo[1]*qo[2]), 1-2*(qo[2]**2+qo[3]**2))))
po_obj = np.stack([b.f("obj", f"pose.position.{c}") for c in "xyz"], 1)
print("joint_states names:", names)
print("   t   | mocap fwd xyz         | vrpn xyz               | odom xyz             | odom v xyz       | tilt  yaw  | motors           | |a|max  | q[0..5]                          | guard | valve yaw  obj z")
for tt in np.arange(t0, t1, dt):
    def nearest(t, v): j = np.clip(np.searchsorted(t, tt), 0, len(t)-1); return v[j]
    w = (ts >= tt - dt/2) & (ts < tt + dt/2)
    s = f"{tt:6.2f} | " + " ".join(f"{x:+.3f}" for x in nearest(tm, pm)) + " | "
    if pv is not None: s += " ".join(f"{x:+.3f}" for x in nearest(tv, pv)) + " | "
    s += " ".join(f"{x:+.3f}" for x in nearest(to, po)) + " | " + " ".join(f"{x:+.2f}" for x in nearest(to, vo)) + " | "
    s += f"{nearest(ta, tilt):5.1f} {nearest(ta, yaw):+6.1f} | " + " ".join(f"{x:.2f}" for x in nearest(tmo, mo)) + " | "
    s += f"{an[w].max() if w.any() else np.nan:6.1f} | " + " ".join(f"{x:+6.1f}" for x in nearest(tj, qj)) + " | "
    s += f"{int(nearest(tg, gs))}  | {nearest(tob, vyaw):+7.1f} {nearest(tob, po_obj[:, 2]):.3f}"
    print(s)
