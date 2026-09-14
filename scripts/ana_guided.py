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
if "--center" in sys.argv:            # the GS lets the operator pick any orbit point
    i = sys.argv.index("--center"); LOITER_C = np.array([float(sys.argv[i + 1]), float(sys.argv[i + 2])])
bag = glob.glob(os.path.join(bagdir, "*.db3"))[0]
con = sqlite3.connect(bag); cur = con.cursor()
topics = {tid: name for tid, name, typ in cur.execute("select id, name, type from topics")}
t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]
def rows(sub, cls):
    tid = [k for k, v in topics.items() if v.endswith(sub)][0]
    return [((ts - t0) / 1e9, deserialize_message(d, cls)) for ts, d in cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid,))]
ac = rows("active_controller", String)
# the cascade FAMILY: "cascade" (base/tank node) or "cascade_adaptive" (STESO + predictor, 2026-09-14);
# the guidance topics carry the id, so read the one that flew
cid = next((m.data for t, m in ac if m.data.startswith("cascade")), "cascade")
eng = next((t for t, m in ac if m.data == cid), None)
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
print(f"{cid} engaged at {eng} s; bag {T[-1]:.0f} s; {len(T)} mocap")
cmds = [(t, m.data) for t, m in rows(f"{cid}/cmd", String)] if any(v.endswith(f"{cid}/cmd") for v in topics.values()) else []
for t, c in cmds: print(f"  cmd t={t:.1f}: {c}")
# the node's 5 Hz status feed (2026-09-13+): the rth arm line says the roll it armed at and how
# long the wings-level guard held it off (2026-09-13 evening)
stat = [(t, m.data) for t, m in rows("cascade/status", String)] if any(v.endswith("cascade/status") for v in topics.values()) else []
arm = next(((t, x) for t, x in stat if "braking" in x), None)
if arm: print(f"  rth arm t={arm[0]:.1f}: {arm[1].split('|')[-1].strip()}")
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

# --- guided-mission specifics ---------------------------------------------------------------
t_rth = next((t for t, c in cmds if c.strip() == "rth"), None)
t_wp = next((t for t, c in cmds if c.startswith("wp")), None)
t_lo = next((t for t, c in cmds if c.startswith("loiter")), None)
t_end = t_rth if t_rth else T[-1]
# loiter: entry = first 25 s after the orbit was reached (leg hand-over or the loiter command),
# steady = the rest until rth. Plan-agnostic: the centre is LOITER_C (--center).
t_orb = t_lo if t_lo else (T[np.argmax(sel & (d < 130))] if (sel & (d < 130)).any() else None)
if t_orb is not None and t_end - t_orb > 30.0:
    for a, b, name in ((t_orb, t_orb + 25.0, "loiter entry (25 s)"), (t_orb + 25.0, t_end, "loiter steady")):
        w = (T > a) & (T < b)
        if w.sum() > 10:
            print(f"{name}: rho mean {d[w].mean():.1f} sd {d[w].std():.2f} min {d[w].min():.1f} max {d[w].max():.1f}; "
                  f"roll mean {E[w,0].mean():+.1f} sd {E[w,0].std():.2f}; z {P[w,2].min():.1f}-{P[w,2].max():.1f}")
elif t_lo is not None:
    print(f"loiter commanded at t={t_lo:.0f} but rth at t={t_rth:.0f}: {t_end - t_lo:.0f} s later -- no orbit to score")
# the default plan's two 90 deg fly-by turns (wp 500 0 500 400 100 400): overshoot past each new leg
if t_wp is not None and "500" in next(c for t, c in cmds if c.startswith("wp")):
    w = sel & (P[:, 1] > 20) & (P[:, 1] < 300) & (T < t_end)
    if w.sum(): print(f"turn 1 overshoot: max x - 500 = {P[w,0].max()-500:+.1f} m")
    w = sel & (P[:, 0] < 480) & (P[:, 0] > 150) & (T < t_end - 70)
    if w.sum(): print(f"turn 2 overshoot: max y - 400 = {P[w,1].max()-400:+.1f} m")
    for t0, t1, name in ((t_wp + 18, t_wp + 40, "turn 1"), (t_wp + 45, t_wp + 70, "turn 2")):
        w = (T > t0) & (T < t1)
        if w.sum(): print(f"{name}: |roll| max {np.abs(E[w,0]).max():.1f} deg, z {P[w,2].min():.1f}-{P[w,2].max():.1f}, V {V[w].min():.2f}-{V[w].max():.2f}")
# rth: the turn, then cross-track to the home line from where the turn ended
if t_rth:
    w = (T > t_rth) & (V > 12)
    if w.sum() > 10:
        Pw = P[w, :2]; p0 = Pw[0]; u = -p0 / max(np.linalg.norm(p0), 1e-6)
        ct = Pw[:, 0] * (-u[1]) + Pw[:, 1] * u[0]
        print(f"rth: from ({p0[0]:.0f},{p0[1]:.0f}) home bearing {math.degrees(math.atan2(-p0[1], -p0[0])):.0f} deg; "
              f"|roll| max {np.abs(E[w,0]).max():.1f}; cross-track to the home line max {np.abs(ct).max():.1f} m, last 5 s {np.abs(ct[-600:]).max():.1f} m")
