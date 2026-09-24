"""Commanded tilt, altitude and speed on straight and banked wing-borne legs of a bag (2026-09-23: INDI turn compensation, the cruise wander). Usage: bag_cruise_tilt.py <bag dir>"""
import sqlite3, sys, glob, math
import numpy as np
from rclpy.serialization import deserialize_message
from px4_msgs.msg import ActuatorServos
from fsc_autopilot_ros2_msgs.msg import Mocap
d = glob.glob(sys.argv[1] + "/*.db3")[0]; c = sqlite3.connect(d)
tops = {n: i for i, n, *_ in c.execute("select id,name,type from topics")}
def rows(name, cls):
    return [(ts, deserialize_message(b, cls)) for ts, b in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tops[name],))]
moc = rows("/uav_0/mocap", Mocap); srv = rows("/uav_0/fmu/in/actuator_servos", ActuatorServos)
tm = np.array([t for t, _ in moc]) / 1e9
V = np.array([math.hypot(m.twist.linear.x, m.twist.linear.y) for _, m in moc])
def rpy(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)); pitch = math.asin(max(-1, min(1, 2*(w*y-z*x))))
    return math.degrees(roll), math.degrees(pitch)
RP = np.array([rpy(m.pose.orientation) for _, m in moc]); Z = np.array([m.pose.position.z for _, m in moc])
ts = np.array([t for t, _ in srv]) / 1e9; tilt = np.array([s.control[0] * 90.0 for _, s in srv])
i = np.clip(np.searchsorted(tm, ts), 0, len(tm) - 1)
m = (V[i] > 14.0) & (np.abs(RP[i, 0]) < 3.0)
print(f"straight wing-borne samples {m.sum()}: tilt cmd mean {tilt[m].mean():.1f} sd {tilt[m].std():.2f} p5 {np.percentile(tilt[m],5):.1f} min {tilt[m].min():.1f}; "
      f"pitch {RP[i[m],1].mean():+.1f} sd {RP[i[m],1].std():.2f}; V {V[i[m]].mean():.2f}; z sd {Z[i[m]].std():.2f}")
mb = (V[i] > 14.0) & (np.abs(RP[i, 0]) > 10.0)
print(f"   banked (|roll| > 10) samples {mb.sum()}: tilt cmd median {np.median(tilt[mb]):.1f} p5 {np.percentile(tilt[mb],5):.1f}; z {Z[i[mb]].min():.1f}-{Z[i[mb]].max():.1f}; V {V[i[mb]].mean():.2f}")
ms = m & (tilt > 45)
print(f"   straight, tilt > 45 (no back-transition): median {np.median(tilt[ms]):.1f} p5 {np.percentile(tilt[ms],5):.1f}; z sd {Z[i[ms]].std():.2f}; V {V[i[ms]].mean():.2f}")
