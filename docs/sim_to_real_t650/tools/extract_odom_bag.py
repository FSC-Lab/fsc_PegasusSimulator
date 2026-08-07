#!/usr/bin/env python3
"""Extract the controller-feedback odometry and the reference sequence from a bag that
contains /uav_0/state_estimator/local_position/odom.

Timing (see fsc_autopilot_ros2/docs/sim_to_real_fidelity.md section 7):
  * rosbag2's `messages.timestamp` is the RECEIVE time and is bursty -- unusable for
    plotting or for step onsets. Odometry carries a clean 100 Hz `header.stamp`, so that is
    the time axis.
  * The reference topic's header stamps sit on a different clock epoch from odometry's, so
    they cannot be compared directly. Reference onsets are recovered by mapping the
    reference's receive time onto the odometry timeline through odometry's own
    receive -> header correspondence.
"""
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message

from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleAttitudeSetpoint

ODOM = "/uav_0/state_estimator/local_position/odom"
REF = "/uav_0/fsc_autopilot_ros2/position_controller/reference"
SC = "/uav_0/fmu/out/sensor_combined"
ATT = "/uav_0/fmu/out/vehicle_attitude"
SP = "/uav_0/fmu/in/vehicle_attitude_setpoint"


def rows(cur, tid):
    return cur.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (tid,)).fetchall()


def hdr_s(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main(bagdir, out):
    db3 = [f for f in os.listdir(bagdir) if f.endswith(".db3")][0]
    con = sqlite3.connect(os.path.join(bagdir, db3))
    cur = con.cursor()
    tid = {r[1]: r[0] for r in cur.execute("SELECT id,name FROM topics")}

    # ---- odometry: the controller feedback, ENU ----
    od = rows(cur, tid[ODOM])
    o_recv = np.array([t for t, _ in od], dtype=np.float64)
    msgs = [deserialize_message(bytes(d), Odometry) for _, d in od]
    o_hdr = np.array([hdr_s(m) for m in msgs], dtype=np.float64)
    pos = np.array([[m.pose.pose.position.x, m.pose.pose.position.y,
                     m.pose.pose.position.z] for m in msgs])
    quat = np.array([[m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                      m.pose.pose.orientation.y, m.pose.pose.orientation.z]
                     for m in msgs])
    vel = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y,
                     m.twist.twist.linear.z] for m in msgs])
    omg = np.array([[m.twist.twist.angular.x, m.twist.twist.angular.y,
                     m.twist.twist.angular.z] for m in msgs])

    # ---- reference: receive time mapped onto the odometry header timeline ----
    rf = rows(cur, tid[REF])
    r_recv = np.array([t for t, _ in rf], dtype=np.float64)
    rmsgs = [deserialize_message(bytes(d), PositionControllerReference) for _, d in rf]
    order = np.argsort(o_recv)
    r_t = np.interp(r_recv, o_recv[order], o_hdr[order])
    r_pos = np.array([[m.position.x, m.position.y, m.position.z] for m in rmsgs])
    r_yaw = np.array([m.yaw for m in rmsgs], dtype=np.float64)
    r_unit = np.array([m.yaw_unit for m in rmsgs], dtype=np.int32)

    data = dict(
        odom_t=o_hdr, odom_recv=o_recv, odom_pos=pos, odom_quat=quat,
        odom_vel=vel, odom_omega=omg,
        ref_t=r_t, ref_recv=r_recv, ref_pos=r_pos, ref_yaw=r_yaw, ref_yaw_unit=r_unit,
    )

    # ---- PX4 streams, for continuity with the earlier study ----
    for key, topic, cls in (("sc", SC, SensorCombined), ("att", ATT, VehicleAttitude),
                            ("sp", SP, VehicleAttitudeSetpoint)):
        if topic not in tid:
            continue
        rr = rows(cur, tid[topic])
        mm = [deserialize_message(bytes(d), cls) for _, d in rr]
        data[f"{key}_ts"] = np.array([m.timestamp for m in mm], dtype=np.int64)
        data[f"{key}_recv"] = np.array([t for t, _ in rr], dtype=np.float64)
        if key == "sc":
            data["sc_gyro"] = np.array([m.gyro_rad for m in mm])
            data["sc_accel"] = np.array([m.accelerometer_m_s2 for m in mm])
        elif key == "att":
            data["att_q"] = np.array([m.q for m in mm])
        else:
            data["sp_q_d"] = np.array([m.q_d for m in mm])
            data["sp_thrust"] = np.array([m.thrust_body for m in mm])
    con.close()

    np.savez_compressed(out, **data)
    dt = np.diff(o_hdr)
    print(f"wrote {out}")
    print(f"  odom      n={len(o_hdr)}  span {o_hdr[-1]-o_hdr[0]:.2f}s  "
          f"header dt: med {np.median(dt)*1e3:.2f} p1 {np.percentile(dt,1)*1e3:.2f} "
          f"p99 {np.percentile(dt,99)*1e3:.2f} ms")
    drecv = np.diff(np.sort(o_recv)) * 1e-9
    print(f"  (receive dt for contrast: med {np.median(drecv)*1e3:.2f} "
          f"max {drecv.max()*1e3:.1f} ms -- why header.stamp is used)")
    print(f"  reference n={len(r_t)}")
    t0 = o_hdr[0]
    for i in range(len(r_t)):
        print(f"    {r_t[i]-t0:8.2f}s  pos=({r_pos[i,0]:6.2f},{r_pos[i,1]:6.2f},"
              f"{r_pos[i,2]:5.2f})  yaw={r_yaw[i]:7.2f} unit={r_unit[i]}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
