#!/usr/bin/env python3
"""`state_source: px4` (2026-09-15): the base cascade can read PX4's EKF2 odometry instead of the
mocap. This pins the one converter it rests on, `nmpc_cascade_node.odom_to_state` (NED/FRD, PX4's
[w,x,y,z] quaternion -> the controller's ENU/FLU), two ways:
  1. synthetic: a level body at NED (1, 2, -3) heading east is ENU (2, 1, 3) with yaw 0; heading
     north is yaw +90 in ENU; a body-FRD velocity is rotated, not just re-labelled; other frames raise.
  2. against real data: flight 11's bag carries both the mocap and EKF2's odometry (EV-fed); the
     converted odometry must sit on the mocap to within EKF2's own latency (position < 0.8 m rms at
     16 m/s, velocity < 0.15 m/s, attitude < 0.6 deg mean, rates < 0.02 rad/s) -- a frame slip is
     metres and tens of degrees, not that.
    ~/envs/isaac_env/bin/python3 tests/test_state_source.py [bag] [--gps] [--no-bag] [--no-boot]
  3. the SAFE controller boots on state_source px4 (subscribes to the odometry, not the mocap).
"""
import glob
import math
import os
import sqlite3
import sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from controller_path import controller_dir     # noqa: E402
sys.path.insert(0, os.path.join(controller_dir(), "px4_vtol_direct_actuator"))
FLAGS = [a for a in sys.argv[1:] if a.startswith("--")]          # our own flags, kept aside so the
sys.argv = [sys.argv[0]] + [a for a in sys.argv[1:] if not a.startswith("--")]   # controller's argf() never sees them
import nmpc_cascade_node as base                # noqa: E402
fails = 0


def check(cond, msg):
    global fails
    print(("PASS " if cond else "FAIL ") + msg); fails += 0 if cond else 1


def q_yaw_ned(psi):     # [w, x, y, z] for a rotation psi about D (NED yaw)
    return [math.cos(psi / 2), 0.0, 0.0, math.sin(psi / 2)]


# 1. synthetic
p, R, v, w = base.odom_to_state([1.0, 2.0, -3.0], q_yaw_ned(math.pi / 2), [1.0, 0.0, 0.0], [0.1, 0.2, 0.3])
check(np.allclose(p, [2.0, 1.0, 3.0]), f"NED (1,2,-3) -> ENU {p}")
yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
check(abs(yaw) < 1e-9, f"NED heading east (psi 90) -> ENU yaw {yaw:.1f} (0 = +x east)")
check(np.allclose(R @ np.array([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]), "body x points east in ENU")
check(np.allclose(v, [0.0, 1.0, 0.0]), f"NED velocity (1,0,0) north -> ENU {v}")
check(np.allclose(w, [0.1, -0.2, -0.3]), f"FRD rates (0.1,0.2,0.3) -> FLU {w}")
p, R, v, w = base.odom_to_state([0.0, 0.0, 0.0], q_yaw_ned(0.0), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
check(abs(math.degrees(math.atan2(R[1, 0], R[0, 0])) - 90.0) < 1e-9, "NED heading north -> ENU yaw +90")
p, R, v, w = base.odom_to_state([0.0, 0.0, 0.0], q_yaw_ned(0.0), [5.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1, 3)
check(np.allclose(v, [0.0, 5.0, 0.0]), f"body-FRD velocity 5 forward while heading north -> ENU {v} (rotated, not re-labelled)")
for pf, vf in [(2, 1), (1, 2)]:
    try:
        base.odom_to_state([0, 0, 0], q_yaw_ned(0.0), [0, 0, 0], [0, 0, 0], pf, vf); check(False, f"frames ({pf},{vf}) accepted")
    except ValueError:
        check(True, f"frames ({pf},{vf}) refused")
# 2. flight 11's bag (skipped without the bag or the ROS python; --no-bag skips it explicitly)
def bag_check():
    bag = next((a for a in sys.argv[1:] if not a.startswith("--")), os.path.expanduser("~/flight_bags/slosh/slosh_20260914_191928"))
    dbs = glob.glob(os.path.join(bag, "*.db3"))
    if not dbs:
        print(f"SKIP bag check: {bag} not found"); return
    try:
        from rclpy.serialization import deserialize_message
        from fsc_autopilot_ros2_msgs.msg import Mocap
        from px4_msgs.msg import VehicleOdometry
    except Exception as e:      # noqa: BLE001
        print(f"SKIP bag check: no ROS python ({e})"); return
    cur = sqlite3.connect(dbs[0]).cursor(); tid = dict(cur.execute("select name, id from topics"))
    t0 = cur.execute("select min(timestamp) from messages").fetchone()[0]
    rows = lambda n, c: [((ts - t0) / 1e9, deserialize_message(d, c)) for ts, d in cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[n],))]
    moc = rows("/uav_0/mocap", Mocap); odo = rows("/uav_0/fmu/out/vehicle_odometry", VehicleOdometry)
    tm = np.array([t for t, _ in moc]); to = np.array([t for t, _ in odo])
    Pm = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in moc])
    Vm = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for _, m in moc])
    Wm = np.array([[m.twist.angular.x, m.twist.angular.y, m.twist.angular.z] for _, m in moc])
    Rm = [base.n6._q2R(m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w) for _, m in moc]
    ep, ev, ea, ew = [], [], [], []
    for i in [i for i, t in enumerate(to) if 230 < t < 400][::5]:
        o = odo[i][1]; p, R, v, w = base.odom_to_state(o.position, o.q, o.velocity, o.angular_velocity, o.pose_frame, o.velocity_frame)
        j = min(np.searchsorted(tm, to[i]), len(tm) - 1)
        ep.append(p - Pm[j]); ev.append(v - Vm[j]); ew.append(w - Wm[j])
        ea.append(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Rm[j].T @ R) - 1) / 2)))))
    ep, ev, ew = np.array(ep), np.array(ev), np.array(ew)
    rp = float(np.sqrt((ep ** 2).sum(1).mean())); rv = float(np.sqrt((ev ** 2).sum(1).mean())); rw = float(np.sqrt((ew ** 2).sum(1).mean()))
    lim_p = 2.5 if "--gps" in FLAGS else 0.8      # a GPS-fed EKF2 carries the receiver's error, not a frame slip
    check(rp < lim_p, f"bag: converted EKF2 position vs mocap rms {rp:.2f} m (mean offset {np.round(ep.mean(0), 2)}; a frame slip is metres)")
    check(rv < 0.25 if "--gps" in FLAGS else rv < 0.15, f"bag: velocity rms {rv:.3f} m/s")
    check(np.mean(ea) < 1.0, f"bag: attitude error mean {np.mean(ea):.2f} deg (max {np.max(ea):.1f} in the turns)")
    check(rw < 0.02, f"bag: body-rate error rms {rw:.3f} rad/s")


if "--no-bag" not in FLAGS:
    bag_check()

# 3. the SAFE controller on state_source px4 (2026-09-15): boots, reports it, subscribes to the
#    odometry and NOT the mocap. Needs the ROS env; skipped without it.
if "--no-boot" not in FLAGS:
    import subprocess, time
    PY = os.path.expanduser("~/envs/isaac_env/bin/python3")
    node = os.path.join(controller_dir(), "px4_vtol_direct_actuator", "geometric_hover_node.py")
    log = os.path.join(os.environ.get("TMPDIR", "/tmp"), "geo_boot_px4src.log")
    cmd = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; exec {PY} {node} --ros-args -p px4_ns:=uav_0 -p auto_arm:=false "
           f"-p controller_id:=geometric -p active_at_start:=false -p state_source:=px4")
    with open(log, "w") as lf:
        proc = subprocess.Popen(["bash", "-c", cmd], stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time(); up = False
    while time.time() - t0 < 40:
        time.sleep(1.0); txt = open(log, errors="replace").read()
        if "geometric_hover up" in txt or "Traceback" in txt or proc.poll() is not None:
            up = "geometric_hover up" in txt; break
    info = subprocess.run(["bash", "-c", f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; timeout 20 ros2 node info /geometric_hover_control"],
                          capture_output=True, text=True).stdout if up else ""
    txt = open(log, errors="replace").read()
    os.killpg(proc.pid, 15); proc.wait(10)
    check(up and "state source: px4" in txt, "safe controller boots on state_source px4 and reports it")
    check("/uav_0/fmu/out/vehicle_odometry: px4_msgs/msg/VehicleOdometry" in info, "safe controller subscribes to EKF2's odometry")
    check("/uav_0/mocap:" not in info, "safe controller on px4 does NOT also subscribe to the mocap")
print("ALL PASS" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
