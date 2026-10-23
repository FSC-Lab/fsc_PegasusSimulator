#!/usr/bin/env python3
"""Generic rosbag2 (sqlite3) -> npz extractor: every topic flattened by field name.

Per topic <key>: <key>__recv (receive time, s), <key>__hdr (header stamp if present),
and <key>__<field.path> numeric arrays. Variable-length arrays (Float32MultiArray.data)
are NaN-padded to the max length (READ BY INDEX). Strings kept as object arrays.
"""
import os, sqlite3, sys, numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

P = "/uav_0"
TOPICS = {
 "wb":       f"{P}/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug",
 "wbref":    f"{P}/fsc_autopilot_ros2/whole_body_direct_actuation/reference",
 "wbmode":   f"{P}/fsc_autopilot_ros2/whole_body_direct_actuation/mode",
 "motors":   f"{P}/fsc_autopilot_ros2/whole_body_direct_actuation/motors_debug",
 "ctype":    f"{P}/fsc_autopilot_ros2/controller_type",
 "vinfo":    f"{P}/fsc_autopilot_ros2/vehicle_info",
 "ude":      f"{P}/fsc_autopilot_ros2/position_controller/ude",
 "pcstate":  f"{P}/fsc_autopilot_ros2/position_controller/state",
 "pcref":    f"{P}/fsc_autopilot_ros2/position_controller/reference",
 "attsp":    f"{P}/fsc_autopilot_ros2/attitude_setpoint_debug",
 "odom":     f"{P}/state_estimator/local_position/odom",
 "opti_eul": f"{P}/state_estimator/local_position/optitrack_euler",
 "px_eul":   f"{P}/state_estimator/local_position/pixhawk_euler",
 "imu":      f"{P}/state_estimator/enu/imu/data",
 "mocap":    f"{P}/mocap",
 "mocap_status": f"{P}/mocap_status",
 "js":       f"{P}/fsc_open_manipulator/joint_states",
 "law":      f"{P}/fsc_open_manipulator/external_torque_controller/law_debug",
 "tcmd":     f"{P}/fsc_open_manipulator/external_torque_controller/joint_torque_command",
 "armref":   f"{P}/fsc_open_manipulator/external_torque_controller/reference_joint_trajectory",
 "smooth":   f"{P}/fsc_open_manipulator/external_torque_controller/smoothed_reference_joint_trajectory",
 "velobs":   f"{P}/fsc_open_manipulator/external_torque_controller/velocity_observer",
 "actlaw":   f"{P}/fsc_open_manipulator/external_torque_controller/active_law",
 "dxl":      f"{P}/fsc_open_manipulator/dynamixel_hardware_interface/dxl_state",
 "pl_status": f"{P}/whole_body_planner/status",
 "pl_ee_target": f"{P}/whole_body_planner/ee_target",
 "pl_pending": f"{P}/whole_body_planner/pending_base",
 "pl_target_joints": f"{P}/whole_body_planner/target_joints",
 "pl_current_ee": f"{P}/whole_body_planner/current_ee",
 "pl_viz_path": f"{P}/whole_body_planner/viz_path",
 "pl_viz_pose": f"{P}/whole_body_planner/viz_pose",
 "vodom_px": f"{P}/fmu/out/vehicle_odometry",
 "vvo": f"{P}/fmu/in/vehicle_visual_odometry",
 "tsync": f"{P}/fmu/out/timesync_status",
 "attsp_in": f"{P}/fmu/in/vehicle_attitude_setpoint",
 "thr_in": f"{P}/fmu/in/vehicle_thrust_setpoint",
 "act_motors": f"{P}/fmu/in/actuator_motors",
 "vstatus":  f"{P}/fmu/out/vehicle_status_v1",
 "batt":     f"{P}/fmu/out/battery_status",
 "vatt":     f"{P}/fmu/out/vehicle_attitude",
 "vodom":    f"{P}/fmu/out/vehicle_odometry",
 "sc":       f"{P}/fmu/out/sensor_combined",
 "rosout":   "/rosout",
}

def flatten(msg, prefix, out):
    for name, ftype in zip(msg.get_fields_and_field_types().keys(), msg.get_fields_and_field_types().values()):
        v = getattr(msg, name)
        key = f"{prefix}{name}"
        if hasattr(v, "get_fields_and_field_types"):
            flatten(v, key + ".", out)
        elif hasattr(v, "__len__") and not isinstance(v, (str, bytes)):
            if len(v) and hasattr(v[0], "get_fields_and_field_types"):
                for i, e in enumerate(v):
                    flatten(e, f"{key}[{i}].", out)
            else:
                out[key] = np.asarray(list(v), dtype=object) if (len(v) and isinstance(v[0], str)) else np.asarray(list(v), dtype=float)
        else:
            out[key] = v

def main(bagdir, outpath):
    db3 = [f for f in os.listdir(bagdir) if f.endswith(".db3")][0]
    con = sqlite3.connect(os.path.join(bagdir, db3)); cur = con.cursor()
    tinfo = {name: (tid, typ) for tid, name, typ in cur.execute("SELECT id,name,type FROM topics")}
    data = {}
    for key, topic in TOPICS.items():
        if topic not in tinfo:
            print("missing", topic); continue
        tid, typ = tinfo[topic]
        cls = get_message(typ)
        rows = cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)).fetchall()
        if not rows:
            print("empty", topic); continue
        recv = np.array([r[0] for r in rows], dtype=np.float64) * 1e-9
        fields = {}
        for _, blob in rows:
            m = deserialize_message(bytes(blob), cls)
            f = {}
            flatten(m, "", f)
            for k, v in f.items():
                fields.setdefault(k, []).append(v)
        data[f"{key}__recv"] = recv
        for k, vs in fields.items():
            if k.startswith("header.stamp"):
                continue
            if isinstance(vs[0], np.ndarray) and vs[0].dtype == object:
                data[f"{key}__{k}"] = np.array([list(x) for x in vs], dtype=object); continue
            if isinstance(vs[0], np.ndarray):
                L = max(len(x) for x in vs)
                a = np.full((len(vs), L), np.nan)
                for i, x in enumerate(vs): a[i, :len(x)] = x
                data[f"{key}__{k}"] = a
            elif isinstance(vs[0], str):
                data[f"{key}__{k}"] = np.array(vs, dtype=object)
            else:
                data[f"{key}__{k}"] = np.array(vs, dtype=float)
        if "header.stamp.sec" in fields:
            data[f"{key}__hdr"] = np.array(fields["header.stamp.sec"], float) + np.array(fields["header.stamp.nanosec"], float) * 1e-9
        print(f"{key:14s} n={len(rows):6d}  fields={len(fields)}")
    np.savez_compressed(outpath, **data)
    print("wrote", outpath)

if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
