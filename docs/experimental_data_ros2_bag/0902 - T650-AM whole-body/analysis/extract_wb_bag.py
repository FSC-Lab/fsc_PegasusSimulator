#!/usr/bin/env python3
"""Extract every WB-relevant topic of the 0902 bags to one npz per bag.

Time base: bag receive timestamp minus bag t0 (hardware: one clock). Header stamps
are also stored where they exist (suffix _hdr) for rate/jitter checks.
"""
import os, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray, Float64MultiArray, String
from geometry_msgs.msg import PoseStamped, Vector3
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from px4_msgs.msg import (ActuatorMotors, VehicleThrustSetpoint, VehicleTorqueSetpoint,
                          VehicleStatus, VehicleAttitude, VehicleOdometry)
from fsc_autopilot_ros2_msgs.msg import Mocap, WholeBodyReference, UDEState

NS = "/uav_0"

def extract(bagdir, out):
    db3 = [f for f in os.listdir(bagdir) if f.endswith(".db3")][0]
    con = sqlite3.connect(os.path.join(bagdir, db3))
    cur = con.cursor()
    tid = {r[1]: r[0] for r in cur.execute("SELECT id,name FROM topics")}
    t0 = cur.execute("SELECT MIN(timestamp) FROM messages").fetchone()[0]

    def rows(name):
        if name not in tid:
            return []
        return cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                           (tid[name],)).fetchall()

    def T(rr):
        return np.array([(t - t0) * 1e-9 for t, _ in rr])

    d = {"t0_ns": t0}

    # --- wb_control_debug (57 f32) ---
    rr = rows(NS + "/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug")
    d["wb_t"] = T(rr)
    d["wb"] = np.array([deserialize_message(bytes(b), Float32MultiArray).data for _, b in rr])

    # --- arm-side law_debug (18 f32) ---
    rr = rows(NS + "/fsc_open_manipulator/external_torque_controller/law_debug")
    d["law_t"] = T(rr)
    d["law"] = np.array([deserialize_message(bytes(b), Float32MultiArray).data for _, b in rr])

    # --- WholeBodyReference stream ---
    rr = rows(NS + "/fsc_autopilot_ros2/whole_body_direct_actuation/reference")
    ms = [deserialize_message(bytes(b), WholeBodyReference) for _, b in rr]
    d["ref_t"] = T(rr)
    v3 = lambda v: [v.x, v.y, v.z]
    for f in ["x_cd", "x_cd_dot", "x_cd_ddot", "r_ed", "r_ed_dot", "b1_d", "b1_de"]:
        d["ref_" + f] = np.array([v3(getattr(m, f)) for m in ms])
    d["ref_q_d"] = np.array([list(m.q_d) for m in ms])
    d["ref_qdot_d"] = np.array([list(m.qdot_d) for m in ms])

    # --- odometry (estimator feedback) ---
    rr = rows(NS + "/state_estimator/local_position/odom")
    ms = [deserialize_message(bytes(b), Odometry) for _, b in rr]
    d["od_t"] = T(rr)
    d["od_pos"] = np.array([[m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z] for m in ms])
    d["od_quat"] = np.array([[m.pose.pose.orientation.w, m.pose.pose.orientation.x,
                              m.pose.pose.orientation.y, m.pose.pose.orientation.z] for m in ms])
    d["od_vel"] = np.array([[m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z] for m in ms])
    d["od_omg"] = np.array([[m.twist.twist.angular.x, m.twist.twist.angular.y, m.twist.twist.angular.z] for m in ms])

    # --- mocap ---
    rr = rows(NS + "/mocap")
    ms = [deserialize_message(bytes(b), Mocap) for _, b in rr]
    d["mc_t"] = T(rr)
    d["mc_pos"] = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in ms])
    d["mc_quat"] = np.array([[m.pose.orientation.w, m.pose.orientation.x,
                              m.pose.orientation.y, m.pose.orientation.z] for m in ms])

    # --- euler topics ---
    for key, name in [("px_eul", "/state_estimator/local_position/pixhawk_euler"),
                      ("ot_eul", "/state_estimator/local_position/optitrack_euler")]:
        rr = rows(NS + name)
        ms = [deserialize_message(bytes(b), Vector3) for _, b in rr]
        d[key + "_t"] = T(rr)
        d[key] = np.array([[m.x, m.y, m.z] for m in ms])

    # --- arm joint states (real servo feedback) ---
    rr = rows(NS + "/fsc_open_manipulator/joint_states")
    ms = [deserialize_message(bytes(b), JointState) for _, b in rr]
    d["js_t"] = T(rr)
    d["js_names"] = np.array(ms[0].name)
    # reorder to joint1..4 if needed
    order = [ms[0].name.index(n) for n in sorted(ms[0].name)]
    d["js_pos"] = np.array([np.array(m.position)[order] for m in ms])
    d["js_vel"] = np.array([np.array(m.velocity)[order] if len(m.velocity) else [np.nan]*len(order) for m in ms])
    d["js_eff"] = np.array([np.array(m.effort)[order] if len(m.effort) else [np.nan]*len(order) for m in ms])
    d["js_order"] = np.array(sorted(ms[0].name))

    # --- flight-node joint torque command ---
    rr = rows(NS + "/fsc_open_manipulator/external_torque_controller/joint_torque_command")
    ms = [deserialize_message(bytes(b), JointState) for _, b in rr]
    d["jtc_t"] = T(rr)
    d["jtc_names"] = np.array(ms[0].name)
    d["jtc_eff"] = np.array([m.effort for m in ms])

    # --- arm-side observer + smoothed reference ---
    for key, name in [("vo", "/fsc_open_manipulator/external_torque_controller/velocity_observer"),
                      ("srt", "/fsc_open_manipulator/external_torque_controller/smoothed_reference_joint_trajectory")]:
        rr = rows(NS + name)
        ms = [deserialize_message(bytes(b), JointState) for _, b in rr]
        d[key + "_t"] = T(rr)
        d[key + "_pos"] = np.array([m.position for m in ms])
        d[key + "_vel"] = np.array([m.velocity if len(m.velocity) else [np.nan]*4 for m in ms])
        d[key + "_eff"] = np.array([m.effort if len(m.effort) else [np.nan]*4 for m in ms])
        if ms:
            d[key + "_names"] = np.array(ms[0].name)

    # --- motors ---
    for key, name in [("mot", "/fmu/in/actuator_motors"),
                      ("motdbg", "/fsc_autopilot_ros2/whole_body_direct_actuation/motors_debug")]:
        rr = rows(NS + name)
        ms = [deserialize_message(bytes(b), ActuatorMotors) for _, b in rr]
        d[key + "_t"] = T(rr)
        d[key] = np.array([list(m.control[:4]) for m in ms])

    # --- thrust/torque setpoints to PX4 ---
    rr = rows(NS + "/fmu/in/vehicle_thrust_setpoint")
    ms = [deserialize_message(bytes(b), VehicleThrustSetpoint) for _, b in rr]
    d["thr_t"] = T(rr); d["thr"] = np.array([list(m.xyz) for m in ms])
    rr = rows(NS + "/fmu/in/vehicle_torque_setpoint")
    ms = [deserialize_message(bytes(b), VehicleTorqueSetpoint) for _, b in rr]
    d["trq_t"] = T(rr); d["trq"] = np.array([list(m.xyz) for m in ms])

    # --- UDE ---
    rr = rows(NS + "/fsc_autopilot_ros2/position_controller/ude")
    ms = [deserialize_message(bytes(b), UDEState) for _, b in rr]
    d["ude_t"] = T(rr)
    d["ude_dist"] = np.array([[m.disturbance_estimate.x, m.disturbance_estimate.y, m.disturbance_estimate.z] for m in ms])

    # --- governor poses ---
    for key, name in [("gee", "/whole_body_governor/current_ee"),
                      ("geeb", "/whole_body_governor/current_ee_body"),
                      ("gpb", "/whole_body_governor/pending_base"),
                      ("geet", "/whole_body_governor/ee_target")]:
        rr = rows(NS + name)
        ms = [deserialize_message(bytes(b), PoseStamped) for _, b in rr]
        d[key + "_t"] = T(rr)
        d[key + "_pos"] = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in ms]) if ms else np.zeros((0, 3))
        d[key + "_quat"] = np.array([[m.pose.orientation.w, m.pose.orientation.x,
                                      m.pose.orientation.y, m.pose.orientation.z] for m in ms]) if ms else np.zeros((0, 4))

    # --- target joints ---
    rr = rows(NS + "/whole_body_governor/target_joints")
    d["gtj_t"] = T(rr)
    d["gtj"] = np.array([list(deserialize_message(bytes(b), Float64MultiArray).data) for _, b in rr]) if rr else np.zeros((0, 4))
    rr = rows(NS + "/fsc_open_manipulator/external_torque_controller/target_joint_setpoint")
    d["tjs_t"] = T(rr)
    d["tjs"] = np.array([list(deserialize_message(bytes(b), Float64MultiArray).data) for _, b in rr]) if rr else np.zeros((0, 4))

    # --- strings (timeline) ---
    for key, name in [("mode", "/fsc_autopilot_ros2/whole_body_direct_actuation/mode"),
                      ("alaw", "/fsc_open_manipulator/external_torque_controller/active_law"),
                      ("gstat", "/whole_body_governor/status")]:
        rr = rows(NS + name)
        d[key + "_t"] = T(rr)
        d[key + "_s"] = np.array([deserialize_message(bytes(b), String).data for _, b in rr])

    # --- vehicle status (arming) ---
    rr = rows(NS + "/fmu/out/vehicle_status_v1")
    ms = [deserialize_message(bytes(b), VehicleStatus) for _, b in rr]
    d["vs_t"] = T(rr)
    d["vs_arm"] = np.array([m.arming_state for m in ms])
    d["vs_nav"] = np.array([m.nav_state for m in ms])

    np.savez_compressed(out, **d)
    print(f"{os.path.basename(bagdir)} -> {out}  ({len(d)} keys)")

base = "/home/fsc-jupiter/Source/fsc_PegasusSimulator/docs/experimental_data_ros2_bag/0902 - T650-AM whole-body"
outdir = os.path.join(base, "analysis")
os.makedirs(outdir, exist_ok=True)
for b in sorted(os.listdir(base)):
    if b.startswith("wb_tracking"):
        extract(os.path.join(base, b), os.path.join(outdir, b + ".npz"))
