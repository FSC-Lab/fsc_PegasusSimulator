"""EKF2 odometry vs the sim truth (mocap), position and velocity ENU, with the GPS alongside -- found the frozen estimate of 2026-09-23. Usage: bag_ekf_vs_truth.py <bag dir> <t0> <t1> [step] [gps]"""
import sqlite3, sys, glob
import numpy as np
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleOdometry, SensorGps
from fsc_autopilot_ros2_msgs.msg import Mocap
d = glob.glob(sys.argv[1] + "/*.db3")[0]
c = sqlite3.connect(d)
tops = {n: i for i, n, *_ in c.execute("select id,name,type from topics")}
def rows(name, cls):
    return [(ts, deserialize_message(b, cls)) for ts, b in c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tops[name],))]
moc = rows("/uav_0/mocap", Mocap)
odo = rows("/uav_0/fmu/out/vehicle_odometry", VehicleOdometry)
gps_name = [n for n in tops if n.endswith("vehicle_gps_position")]
gps = rows(gps_name[0], SensorGps) if gps_name else []
t0 = moc[0][0]
tm = np.array([(t - t0) / 1e9 for t, _ in moc]); pm = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in moc])
vm = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for _, m in moc])
to = np.array([(t - t0) / 1e9 for t, _ in odo])
# NED -> ENU
po = np.array([[o.position[1], o.position[0], -o.position[2]] for _, o in odo])
vo = np.array([[o.velocity[1], o.velocity[0], -o.velocity[2]] for _, o in odo])
a, b, step = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]) if len(sys.argv) > 4 else 2.0
print(f"topics: gps={gps_name}")
print("   t   | mocap x,y  vx,vy        | ekf2 x,y  vx,vy          | dpos    dvel")
for t in np.arange(a, b, step):
    i = np.searchsorted(tm, t); j = np.searchsorted(to, t)
    if i >= len(tm) or j >= len(to): break
    dp = po[j, :2] - pm[i, :2]; dv = vo[j, :2] - vm[i, :2]
    print(f"{t:6.1f} | {pm[i,0]:6.2f} {pm[i,1]:5.2f} {vm[i,0]:+5.2f} {vm[i,1]:+5.2f} | {po[j,0]:6.2f} {po[j,1]:5.2f} {vo[j,0]:+5.2f} {vo[j,1]:+5.2f} | {np.hypot(*dp):5.2f} {np.hypot(*dv):5.2f}")
if gps:
    tg = np.array([(t - t0) / 1e9 for t, _ in gps])
    print("gps samples", len(gps), "rate", len(gps) / (tg[-1] - tg[0]))
if gps and len(sys.argv) > 5:
    g0 = gps[0][1]
    lat0, lon0 = g0.latitude_deg, g0.longitude_deg
    R = 6378137.0
    import math
    print("   t   | gps E,N rel first | gps vel E,N (vel_n,vel_e) | fix eph epv vel_ok")
    for t in np.arange(a, b, step):
        k = np.searchsorted(tg, t)
        if k >= len(gps): break
        g = gps[k][1]
        n = math.radians(g.latitude_deg - lat0) * R; e = math.radians(g.longitude_deg - lon0) * R * math.cos(math.radians(lat0))
        print(f"{t:6.1f} | {e:7.2f} {n:6.2f} | {g.vel_e_m_s:+5.2f} {g.vel_n_m_s:+5.2f} | {g.fix_type} {g.eph:.2f} {g.epv:.2f} {g.vel_ned_valid}")
