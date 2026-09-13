#!/usr/bin/env python3
"""Score a GUIDED cascade mission (turns / loiter / rth) from its bag.

    source scripts/rosenv.sh; ~/envs/isaac_env/bin/python3 scripts/ana_guided.py ~/flight_bags/slosh/<bag_dir>

fly_transition.py's verdict is a straight-line metric (the longest run of controlled+level
samples) and reports a turning mission as INCONCLUSIVE; night_metrics.py scores it LANDED with
maxroll = the bank limit and maxyaw 180, which are the turns. This reads the bag directly (db3,
no metadata.yaml) and prints what a guided mission is judged on: the 4 s track, the fly-by turn
overshoots, the loiter entry vs steady radius about (100,400), the RTH turn / track capture /
stop distance from home, and the touchdown. Flight turns_loiter_rth_a1_151327 (2026-09-13):
steady rho 103.3 +- 0.5 m at R 100 (gate 103.3 +- 0.3), stop 24 m from home, landed 0.00 m.
The waypoints are the mission's defaults; edit LOITER_C / the turn windows for another plan.
"""
import glob, os, sqlite3, sys, math
import numpy as np

LOITER_C = np.array([100.0, 400.0])   # the default plan's orbit centre
from rclpy.serialization import deserialize_message
from fsc_autopilot_ros2_msgs.msg import Mocap
from px4_msgs.msg import ActuatorServos
from std_msgs.msg import String

bagdir = sys.argv[1]
bag = glob.glob(os.path.join(bagdir, "*.db3"))[0]
con = sqlite3.connect(bag); cur = con.cursor()
topics = {tid: name for tid, name, typ in cur.execute("select id, name, type from topics")}
t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]
def rows(sub, cls):
    tid = [k for k, v in topics.items() if v.endswith(sub)][0]
    return [((ts - t0) / 1e9, deserialize_message(d, cls)) for ts, d in cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid,))]
ac = rows("active_controller", String)
eng = next((t for t, m in ac if m.data == "cascade"), None)
mc = rows("/mocap", Mocap)
T = np.array([t for t, _ in mc]); P = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in mc])
Vv = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for _, m in mc])
Q = np.array([[m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z] for _, m in mc])
def rpy(q):
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)); pitch = math.asin(max(-1, min(1, 2*(w*y - z*x)))); yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw
E = np.degrees(np.array([rpy(q) for q in Q]))
V = np.linalg.norm(Vv[:, :2], axis=1)
srv = rows("/actuator_servos", ActuatorServos); Ts = np.array([t for t, _ in srv]); tilt = np.array([m.control[0] for _, m in srv]) * 90
print(f"cascade engaged at {eng} s; bag {T[-1]:.0f} s; {len(T)} mocap")
cmds = [(t, m.data) for t, m in rows("cascade/cmd", String)] if any(v.endswith("cascade/cmd") for v in topics.values()) else []
for t, c in cmds: print(f"  cmd t={t:.1f}: {c}")
sel = T > (eng or 0)
# 2 s summary through the mission
print(" t     x      y     z    V   roll  pitch  yaw  tilt")
for t in np.arange(eng or 0, T[-1], 4.0):
    i = np.searchsorted(T, t); j = np.searchsorted(Ts, t)
    if i >= len(T): break
    print(f"{t:6.0f} {P[i,0]:6.0f} {P[i,1]:6.0f} {P[i,2]:5.1f} {V[i]:5.1f} {E[i,0]:6.1f} {E[i,1]:6.1f} {E[i,2]:6.0f} {tilt[min(j,len(tilt)-1)]:5.0f}")
# loiter radius about LOITER_C when within 200 m and wing-borne
d = np.linalg.norm(P[:, :2] - LOITER_C, axis=1)
lo = sel & (d < 160) & (V > 12)
if lo.sum() > 100:
    tl = T[lo]; print(f"loiter (within 160 m of LOITER_C, V>12): t {tl[0]:.0f}-{tl[-1]:.0f} s, rho mean {d[lo].mean():.1f} sd {d[lo].std():.1f} min {d[lo].min():.1f} max {d[lo].max():.1f}; roll mean {E[lo,0].mean():+.1f} sd {E[lo,0].std():.1f}; z {P[lo,2].min():.1f}-{P[lo,2].max():.1f}; V {V[lo].min():.1f}-{V[lo].max():.1f}")
    # laps: unwrap angle
    ang = np.unwrap(np.arctan2(P[lo,1]-400, P[lo,0]-100)); print(f"  orbit angle swept {np.degrees(ang[-1]-ang[0]):+.0f} deg")
wb = sel & (V > 12)
print(f"wing-borne (V>12): {wb.sum()/120:.0f} s, z {P[wb,2].min():.1f}-{P[wb,2].max():.1f}, |roll| max {np.abs(E[wb,0]).max():.1f}, pitch {E[wb,1].min():+.1f}..{E[wb,1].max():+.1f}, V max {V.max():.1f}")
# stop after RTH: first time V<1 after the last V>12
last_wb = np.where(wb)[0][-1] if wb.sum() else 0
st = np.where((np.arange(len(T)) > last_wb) & (V < 1.0))[0]
if len(st): i = st[0]; print(f"stopped at t={T[i]:.0f} pos=({P[i,0]:.1f},{P[i,1]:.1f}) z={P[i,2]:.1f}  {np.linalg.norm(P[i,:2]):.1f} m from home; back-transition {T[i]-T[last_wb]:.1f} s from V=12, z min {P[last_wb:i,2].min():.1f}")
print(f"final pos ({P[-1,0]:.2f},{P[-1,1]:.2f},{P[-1,2]:.2f}) roll {E[-1,0]:+.1f} pitch {E[-1,1]:+.1f}")

# --- guided-mission specifics: steady loiter (after the entry, before rth), turn overshoots, rth track
t_rth = next((t for t, c in cmds if c.strip() == "rth"), None)
t_lo = next((t for t, c in cmds if c.startswith("wp")), None)
if t_rth:
    for a, b, name in ((t_rth - 65, t_rth, "loiter steady (last 65 s before rth)"), (t_rth - 90, t_rth - 65, "loiter entry (25 s)")):
        w = (T > a) & (T < b)
        print(f"{name}: rho mean {d[w].mean():.1f} sd {d[w].std():.2f} min {d[w].min():.1f} max {d[w].max():.1f}; roll mean {E[w,0].mean():+.1f} sd {E[w,0].std():.2f}; z {P[w,2].min():.1f}-{P[w,2].max():.1f}; V {V[w].min():.2f}-{V[w].max():.2f}")
    # turn 1: leg (0,0)->(500,0) then (500,0)->(500,400): overshoot = max x beyond 500 while y in (0,300)
    w = sel & (P[:,1] > 20) & (P[:,1] < 300) & (T < t_rth); print(f"turn 1 overshoot: max x - 500 = {P[w,0].max()-500:+.1f} m, settles |x-500|<2 at y={P[w][np.abs(P[w,0]-500)<2][0,1]:.0f} m" if w.sum() else "")
    w = sel & (P[:,0] < 480) & (P[:,0] > 150) & (T < t_rth) & (T < t_rth-70); print(f"turn 2 overshoot: max y - 400 = {P[w,1].max()-400:+.1f} m")
    # bank in the turns and the heading overshoot
    for t0, t1, name in ((t_lo + 18, t_lo + 40, "turn 1"), (t_lo + 45, t_lo + 70, "turn 2")):
        w = (T > t0) & (T < t1); print(f"{name}: |roll| max {np.abs(E[w,0]).max():.1f} deg, z {P[w,2].min():.1f}-{P[w,2].max():.1f}, V {V[w].min():.2f}-{V[w].max():.2f}")
    # rth: cross-track to the home line from the point where the rth turn finished
    w = (T > t_rth) & (V > 12); Pw = P[w, :2]; p0 = Pw[0]; u = -p0 / np.linalg.norm(p0)
    ct = Pw[:, 0] * (-u[1]) + Pw[:, 1] * u[0]
    print(f"rth: from ({p0[0]:.0f},{p0[1]:.0f}) home bearing {math.degrees(math.atan2(-p0[1], -p0[0])):.0f} deg; |roll| max {np.abs(E[w,0]).max():.1f}; cross-track to the home line max {np.abs(ct).max():.1f} m, last 5 s {np.abs(ct[-600:]).max():.1f} m")
if t_rth:
    for a, b, name in ((t_rth - 40, t_rth, "loiter steady (last 40 s)"), (t_rth - 65, t_rth - 40, "loiter entry (first 25 s)")):
        w = (T > a) & (T < b)
        print(f"{name}: rho mean {d[w].mean():.1f} sd {d[w].std():.2f} min {d[w].min():.1f} max {d[w].max():.1f}; roll mean {E[w,0].mean():+.1f} sd {E[w,0].std():.2f}")
    w = sel & (T > t_lo + 40) & (T < t_lo + 70); i = np.argmax(P[w,1]) ; 
