#!/usr/bin/env python3
"""The RTH decel onset and stop from a guided bag: distance to home, V, roll, pitch, z, tilt
through the back-transition, then the braking distance, the mean deceleration and the roll at
the arm.

    source scripts/rosenv.sh; ~/envs/isaac_env/bin/python3 scripts/ana_rth_onset.py ~/flight_bags/slosh/<bag_dir>

The onset is the first `decel=1` on /uav_0/cascade/status (2026-09-13+ bags); older bags fall
back to the tilt leaving 90 deg after the rth command. Pitch is printed NOSE-UP positive (the
mocap quaternion's asin(2(wy - zx)) reads nose-down positive here). In HIL the bag's clock is
WALL time while Isaac runs at 0.86-0.93x, so "mean decel" in sim-time is the printed value
divided by the RTF -- the wall-clock 1.5 m/s^2 ramp is 1.66-1.74 m/s^2 of sim braking, which is
why HIL stops ~10 m shorter than the gate (2026-09-13 evening finding; lived in /tmp and was
wiped by a reboot once -- hence this file).
"""
import glob, sqlite3, sys, math
import numpy as np
from rclpy.serialization import deserialize_message
from fsc_autopilot_ros2_msgs.msg import Mocap
from px4_msgs.msg import ActuatorServos
from std_msgs.msg import String

bag = glob.glob(sys.argv[1].rstrip("/") + "/*.db3")[0]
cur = sqlite3.connect(bag).cursor()
topics = {tid: name for tid, name, typ in cur.execute("select id, name, type from topics")}
t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]


def _cascade_topic(sub):
    """'cascade/status' or 'cascade/cmd' -> the cascade topic that actually carried the flight:
    the base cascade's /uav_0/cascade/<kind> or the adaptive one's /uav_0/cascade_adaptive/<kind>
    (2026-09-14: two cascades stand by, each on its own pair; the flown one has the messages)."""
    kind = sub.split("/")[-1]
    cands = [(cur.execute("select count(*) from messages where topic_id=?", (k,)).fetchone()[0], v)
             for k, v in topics.items() if v.endswith("/" + kind) and "cascade" in v]
    return max(cands)[1] if cands else sub

def rows(sub, cls):
    if sub.startswith("cascade/"):
        sub = _cascade_topic(sub)
    tid = [k for k, v in topics.items() if v.endswith(sub)]
    if not tid:
        return []
    return [((ts - t0) / 1e9, deserialize_message(d, cls)) for ts, d in
            cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[0],))]


mc = rows("/mocap", Mocap)
T = np.array([t for t, _ in mc]); P = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in mc])
Vv = np.array([[m.twist.linear.x, m.twist.linear.y] for _, m in mc]); V = np.linalg.norm(Vv, axis=1)
Q = np.array([[m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z] for _, m in mc])


def rp(q):
    w, x, y, z = q
    return (math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))),
            -math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))))


E = np.array([rp(q) for q in Q])
srv = rows("/actuator_servos", ActuatorServos); Ts = np.array([t for t, _ in srv])
tilt = np.array([m.control[0] for _, m in srv]) * 90.0
cmds = [(t, m.data) for t, m in rows("cascade/cmd", String)]
t_rth = next((t for t, c in cmds if c.strip() == "rth"), None)
st = [(t, m.data) for t, m in rows("cascade/status", String)]
t_dec = next((t for t, s in st if "decel=1" in s), None)
arm = next((s.split("|")[-1].strip() for t, s in st if "braking" in s), "")
if t_dec is None and t_rth is not None:
    j = np.where((Ts > t_rth) & (tilt < 88))[0]; t_dec = Ts[j[0]] if len(j) else None
if t_dec is None:
    sys.exit("no decel onset found (no decel=1 status, no rth command)")
print(f"rth cmd t={t_rth}; decel onset t={t_dec:.1f}; {arm}")
d_home = np.linalg.norm(P[:, :2], axis=1)
print("   dt   d_home     V    roll  pitch(up)    z   tilt")
for t in np.arange(t_dec - 2.0, t_dec + 13.0, 0.5):
    i = min(np.searchsorted(T, t), len(T) - 1); j = min(np.searchsorted(Ts, t), len(tilt) - 1)
    print(f"{t - t_dec:+5.1f} {d_home[i]:7.1f} {V[i]:6.2f} {E[i, 0]:+6.1f} {E[i, 1]:+8.1f} {P[i, 2]:6.1f} {tilt[j]:6.1f}")
i0 = np.searchsorted(T, t_dec)
k = np.where((np.arange(len(T)) > i0) & (V < 1.0))[0]
if len(k):
    k = k[0]
    print(f"onset: d_home {d_home[i0]:.1f} m, V {V[i0]:.2f}, roll {E[i0, 0]:+.1f} -> stop at t+{T[k] - t_dec:.1f} s, "
          f"{d_home[k]:.1f} m from home: braking distance {d_home[i0] - d_home[k]:.1f} m "
          f"(V^2/(2*1.5) = {V[i0] ** 2 / 3.0:.1f}); mean decel {(V[i0] - V[k]) / (T[k] - t_dec):.2f} m/s^2 wall")
    w = slice(i0, k)
    print(f"crossing: peak nose-up pitch {E[w, 1].max():+.1f}, z {P[w, 2].min():.1f}-{P[w, 2].max():.1f}, |roll| max {np.abs(E[w, 0]).max():.1f}")
