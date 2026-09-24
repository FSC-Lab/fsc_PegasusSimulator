"""PX4 land detector vs altitude, the controller switch and EKF2-vs-truth, one line per change -- landed=1 at altitude means EKF2 was frozen (2026-09-23). Usage: bag_land_detect.py <bag dir>"""
import sqlite3, sys, glob
import numpy as np
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleLandDetected, VehicleOdometry
from fsc_autopilot_ros2_msgs.msg import Mocap
from std_msgs.msg import String
d = glob.glob(sys.argv[1] + "/*.db3")[0]; c = sqlite3.connect(d)
tops = {n: i for i, n, *_ in c.execute("select id,name,type from topics")}
def rows(name, cls):
    return [(ts, deserialize_message(b, cls)) for ts, b in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tops[name],))]
moc = rows("/uav_0/mocap", Mocap); t0 = moc[0][0]
tm = np.array([(t - t0) / 1e9 for t, _ in moc]); pm = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in moc])
odo = rows("/uav_0/fmu/out/vehicle_odometry", VehicleOdometry)
to = np.array([(t - t0) / 1e9 for t, _ in odo]); po = np.array([[o.position[1], o.position[0], -o.position[2]] for _, o in odo])
ac = rows([n for n in tops if n.endswith("active_controller")][0], String)
ld = rows([n for n in tops if n.endswith("vehicle_land_detected")][0], VehicleLandDetected)
ev = [((t - t0) / 1e9, "ctrl=" + m.data) for t, m in ac]
prev = None; evs = []
for t, m in ld:
    k = (m.landed, m.at_rest)
    if k != prev: evs.append(((t - t0) / 1e9, f"landed={int(m.landed)} at_rest={int(m.at_rest)}")); prev = k
pc = None
for t, s in ev:
    if s != pc: evs.append((t, s)); pc = s
for t, s in sorted(evs):
    i = min(np.searchsorted(tm, t), len(tm) - 1); j = min(np.searchsorted(to, t), len(to) - 1)
    print(f"{t:7.1f} {s:24s} truth z={pm[i,2]:5.1f} x={pm[i,0]:6.2f} | ekf2-truth {np.hypot(*(po[j,:2]-pm[i,:2])):5.2f} m")
