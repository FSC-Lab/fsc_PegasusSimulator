#!/usr/bin/env python3
"""Extract a DIRECT-actuation flight bag to npz.

Same timing discipline as extract_odom_bag.py (which this is modelled on):
  * rosbag2's `messages.timestamp` is the RECEIVE time and is bursty -- never a time axis.
    Odometry's `header.stamp` is clean 100 Hz and is the timeline everything maps onto.
  * Topics that carry no header (std_msgs/String mode + controller_type) only have a
    receive time, so they are mapped onto the odometry timeline through odometry's own
    receive -> header correspondence, exactly as the reference topic is.

Extra over the baseline extractor: the two mode topics, so DIRECT segments can be
separated from BASELINE ones, plus the inner-loop streams (motor commands, rate-control
debug) that only exist in direct actuation.
"""
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message

from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
from nav_msgs.msg import Odometry
from px4_msgs.msg import (ActuatorMotors, SensorCombined, VehicleAttitude,
                          VehicleAttitudeSetpoint, VehicleTorqueSetpoint)
from std_msgs.msg import Float32MultiArray, String

P = "/uav_0"
ODOM = f"{P}/state_estimator/local_position/odom"
REF = f"{P}/fsc_autopilot_ros2/position_controller/reference"
MODE = f"{P}/fsc_autopilot_ros2/direct_actuation/mode"
CTYPE = f"{P}/fsc_autopilot_ros2/controller_type"
SC = f"{P}/fmu/out/sensor_combined"
ATT = f"{P}/fmu/out/vehicle_attitude"
SP = f"{P}/fsc_autopilot_ros2/attitude_setpoint_debug"
MOT = f"{P}/fmu/in/actuator_motors"
TRQ = f"{P}/fmu/in/vehicle_torque_setpoint"
RCD = f"{P}/fsc_autopilot_ros2/direct_actuation/rate_control_debug"


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

    # ---- odometry: controller feedback, ENU. The master timeline. ----
    od = rows(cur, tid[ODOM])
    o_recv = np.array([t for t, _ in od], dtype=np.float64)
    msgs = [deserialize_message(bytes(d), Odometry) for _, d in od]
    o_hdr = np.array([hdr_s(m) for m in msgs], dtype=np.float64)
    order = np.argsort(o_recv)

    def to_odom_time(recv):
        """Map a receive-time array onto the odometry header timeline."""
        return np.interp(recv, o_recv[order], o_hdr[order])

    data = dict(
        odom_t=o_hdr, odom_recv=o_recv,
        odom_pos=np.array([[m.pose.pose.position.x, m.pose.pose.position.y,
                            m.pose.pose.position.z] for m in msgs]),
        odom_quat=np.array([[m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                             m.pose.pose.orientation.y, m.pose.pose.orientation.z]
                            for m in msgs]),
        odom_vel=np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y,
                            m.twist.twist.linear.z] for m in msgs]),
        odom_omega=np.array([[m.twist.twist.angular.x, m.twist.twist.angular.y,
                              m.twist.twist.angular.z] for m in msgs]),
    )

    # ---- reference ----
    rf = rows(cur, tid[REF])
    rmsgs = [deserialize_message(bytes(d), PositionControllerReference) for _, d in rf]
    data["ref_t"] = to_odom_time(np.array([t for t, _ in rf], dtype=np.float64))
    data["ref_pos"] = np.array([[m.position.x, m.position.y, m.position.z]
                                for m in rmsgs])
    data["ref_yaw"] = np.array([m.yaw for m in rmsgs], dtype=np.float64)
    data["ref_yaw_unit"] = np.array([m.yaw_unit for m in rmsgs], dtype=np.int32)

    # ---- mode topics: headerless, so receive-time mapped ----
    for key, topic in (("mode", MODE), ("ctype", CTYPE)):
        if topic not in tid:
            continue
        rr = rows(cur, tid[topic])
        data[f"{key}_t"] = to_odom_time(np.array([t for t, _ in rr], dtype=np.float64))
        data[f"{key}_v"] = np.array(
            [deserialize_message(bytes(d), String).data for _, d in rr])

    # ---- PX4 / inner-loop streams ----
    for key, topic, cls in (("sc", SC, SensorCombined),
                            ("att", ATT, VehicleAttitude),
                            ("sp", SP, VehicleAttitudeSetpoint),
                            ("mot", MOT, ActuatorMotors),
                            ("trq", TRQ, VehicleTorqueSetpoint)):
        if topic not in tid:
            continue
        rr = rows(cur, tid[topic])
        mm = [deserialize_message(bytes(d), cls) for _, d in rr]
        recv = np.array([t for t, _ in rr], dtype=np.float64)
        data[f"{key}_recv"] = recv
        data[f"{key}_t"] = to_odom_time(recv)
        data[f"{key}_ts"] = np.array([m.timestamp for m in mm], dtype=np.int64)
        if key == "sc":
            data["sc_gyro"] = np.array([m.gyro_rad for m in mm])
            data["sc_accel"] = np.array([m.accelerometer_m_s2 for m in mm])
        elif key == "att":
            data["att_q"] = np.array([m.q for m in mm])
        elif key == "sp":
            data["sp_q_d"] = np.array([m.q_d for m in mm])
            data["sp_thrust"] = np.array([m.thrust_body for m in mm])
        elif key == "mot":
            data["mot_ctrl"] = np.array([m.control[:4] for m in mm])
        elif key == "trq":
            data["trq_xyz"] = np.array([m.xyz for m in mm])

    if RCD in tid:
        rr = rows(cur, tid[RCD])
        mm = [deserialize_message(bytes(d), Float32MultiArray) for _, d in rr]
        n = min(len(m.data) for m in mm) if mm else 0
        data["rcd_t"] = to_odom_time(np.array([t for t, _ in rr], dtype=np.float64))
        data["rcd"] = np.array([list(m.data[:n]) for m in mm], dtype=np.float64)

    con.close()
    np.savez_compressed(out, **data)

    t0 = o_hdr[0]
    dt = np.diff(o_hdr)
    print(f"wrote {out}")
    print(f"  odom n={len(o_hdr)}  span {o_hdr[-1]-t0:.2f}s  "
          f"header dt med {np.median(dt)*1e3:.2f} ms")
    if "mode_v" in data:
        print("  --- direct_actuation/mode transitions ---")
        mv, mt = data["mode_v"], data["mode_t"]
        for i in range(len(mv)):
            if i == 0 or mv[i] != mv[i - 1]:
                print(f"    {mt[i]-t0:8.2f}s  {mv[i]}")
    if "ctype_v" in data:
        print("  --- controller_type transitions ---")
        cv, ct = data["ctype_v"], data["ctype_t"]
        for i in range(len(cv)):
            if i == 0 or cv[i] != cv[i - 1]:
                print(f"    {ct[i]-t0:8.2f}s  {cv[i]}")
    print(f"  --- reference n={len(data['ref_t'])} ---")
    for i in range(len(data["ref_t"])):
        rp, ry = data["ref_pos"][i], data["ref_yaw"][i]
        print(f"    {data['ref_t'][i]-t0:8.2f}s  pos=({rp[0]:6.2f},{rp[1]:6.2f},"
              f"{rp[2]:5.2f})  yaw={ry:7.2f} unit={data['ref_yaw_unit'][i]}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
