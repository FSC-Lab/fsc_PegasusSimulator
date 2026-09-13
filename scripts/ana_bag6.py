#!/usr/bin/env python3
"""Analyze an nmpc6/full-state HIL bag. Usage: ana_bag6.py <name> [stride_s]
Reads /uav_0/mocap + commanded actuators; prints pos/vel, body-pitch, AoA (sim-exact v_body=R^T v),
roll/yaw, motors, tilt servo, elevator/aileron/rudder + a summary. (anabag.py chokes on the px4
out-topics recorded by recbag_nmpc6.sh -- use THIS one.) Lives in the repo: /tmp gets wiped."""
import os, sys, math
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from fsc_autopilot_ros2_msgs.msg import Mocap
from px4_msgs.msg import ActuatorMotors, ActuatorServos
from std_msgs.msg import String
name = sys.argv[1]; stride = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
path = name if "/" in name else os.path.expanduser(f"~/flight_bags/{name}")
r = SequentialReader(); r.open(StorageOptions(uri=path, storage_id="sqlite3"), ConverterOptions("", ""))
CLS = {}
for t in r.get_all_topics_and_types():
    if t.name.endswith("/mocap"): CLS[t.name] = Mocap
    elif t.name.endswith("actuator_motors"): CLS[t.name] = ActuatorMotors
    elif t.name.endswith("actuator_servos"): CLS[t.name] = ActuatorServos
    elif t.name.endswith("active_controller"): CLS[t.name] = String
def q2R(x, y, z, w):
    n = math.sqrt(x*x+y*y+z*z+w*w)+1e-12; x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
moc, mot, srv, act = [], [], [], []
while r.has_next():
    topic, data, t = r.read_next()
    if topic not in CLS: continue
    m = deserialize_message(data, CLS[topic]); ts = t/1e9
    if CLS[topic] is Mocap: moc.append((ts, m))
    elif CLS[topic] is ActuatorMotors: mot.append((ts, list(m.control[:4])))
    elif CLS[topic] is ActuatorServos: srv.append((ts, list(m.control[:5])))
    elif CLS[topic] is String: act.append((ts, m.data))
if not moc: print("no mocap in bag"); sys.exit(0)
t0 = moc[0][0]
print(f"bag={path} dur={moc[-1][0]-t0:.1f}s mocap={len(moc)} mot={len(mot)} srv={len(srv)}")
for ts, d in act: print(f"  [t={ts-t0:+.1f}s] active_controller = {d}")
near = lambda L, ts: (min(L, key=lambda e: abs(e[0]-ts)) if L else None)
print(" t     x    y    z   |V|  vs   vz | roll pitch  yaw | AoA | mFL mFR mRL mRR tilt elev ail rud")
zs=[];pit=[];aoas=[];rolls=[];yaws=[];tilts=[];last=-9
for ts, m in moc:
    if ts-last < stride: continue
    last = ts
    p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
    v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
    q = m.pose.orientation; R = q2R(q.x, q.y, q.z, q.w); vb = R.T @ v
    aoa = math.degrees(math.atan2(vb[2], vb[0])) if abs(vb[0]) > 0.3 else 0.0
    pitch = -math.degrees(math.asin(max(-1, min(1, R[2, 0]))))   # nose-up +, == sim AoA convention
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2])); yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    V = float(np.linalg.norm(v))
    vs = v[0]*math.cos(math.radians(yaw)) + v[1]*math.sin(math.radians(yaw))
    mm = near(mot, ts); ss = near(srv, ts)
    mc = mm[1] if mm else [0]*4; sc = ss[1] if ss else [0]*5
    print("%4.1f %5.1f %5.1f %4.1f %4.1f %5.1f %5.1f |%5.0f %5.0f %5.0f |%5.0f | %.2f %.2f %.2f %.2f %.2f %+.2f %+.2f %+.2f"
          % (ts-t0, p[0], p[1], p[2], V, vs, v[2], roll, pitch, yaw, aoa,
             mc[0], mc[1], mc[2], mc[3], sc[0], sc[3], sc[2], sc[4]))
    zs.append(p[2]); pit.append(pitch); aoas.append(aoa); rolls.append(roll); yaws.append(yaw)
    tilts.append(sc[0]*90.0)
print("--- summary ---")
print(" altitude %.1f..%.1f (range %.1f) | pitch %.0f..%.0f | AoA %.0f..%.0f | roll %.0f..%.0f | yaw %.0f..%.0f | tilt %.0f..%.0f"
      % (min(zs), max(zs), max(zs)-min(zs), min(pit), max(pit), min(aoas), max(aoas),
         min(rolls), max(rolls), min(yaws), max(yaws), min(tilts), max(tilts)))
print(" HEALTHY wing-borne: pitch~=AoA~=+6, roll/yaw~=0, tilt->90, rear motors->0, altitude flat")
