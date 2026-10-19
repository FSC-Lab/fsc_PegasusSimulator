import sys, numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag = sys.argv[1]; out = sys.argv[2]
T0 = 1792013326048378036
conv = {
 '/vrpn_mocap/uav_0/pose': lambda m: [m.pose.position.x,m.pose.position.y,m.pose.position.z,m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w],
 '/vrpn_mocap/obj_0/pose': lambda m: [m.pose.position.x,m.pose.position.y,m.pose.position.z,m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w],
 '/uav_0/mocap': lambda m: [m.pose.position.x,m.pose.position.y,m.pose.position.z,m.twist.linear.x,m.twist.linear.y,m.twist.linear.z],
 '/obj_0/mocap': lambda m: [m.pose.position.x,m.pose.position.y,m.pose.position.z],
 '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug': lambda m: list(m.data)+[np.nan]*(115-len(m.data)),
 '/uav_0/fmu/in/actuator_motors': lambda m: list(m.control[:4]),
 '/uav_0/whole_body_planner/current_ee': lambda m: [m.pose.position.x,m.pose.position.y,m.pose.position.z],
 '/uav_0/whole_body_planner/current_base': lambda m: [m.pose.position.x,m.pose.position.y,m.pose.position.z],
 '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/reference': lambda m: [m.x_cd.x,m.x_cd.y,m.x_cd.z,m.x_cd_dot.x,m.x_cd_dot.y,m.x_cd_dot.z,m.x_cd_ddot.x,m.x_cd_ddot.y,m.x_cd_ddot.z],
 '/uav_0/fmu/out/battery_status': lambda m: [m.voltage_v, m.current_a],
 '/uav_0/state_estimator/enu/imu/data': lambda m: [m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
 '/uav_0/fmu/out/vehicle_attitude': lambda m: list(m.q),
 '/uav_0/state_estimator/local_position/optitrack_euler': lambda m: [m.x,m.y,m.z],
 '/uav_0/fsc_open_manipulator/joint_states': lambda m: list(m.position[:6]) + [np.nan]*(6-len(m.position[:6])),
 '/uav_0/fsc_autopilot_ros2/position_controller/ude': lambda m: [m.disturbance_estimate.x,m.disturbance_estimate.y,m.disturbance_estimate.z, float(m.is_active)],
}
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr','cdr'))
tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
topics = list(conv)
r.set_filter(rosbag2_py.StorageFilter(topics=topics))
types = {t: get_message(tmap[t]) for t in topics}
data = {t: ([],[]) for t in topics}
names = {}
while r.has_next():
    t, raw, ts = r.read_next()
    m = deserialize_message(raw, types[t])
    if t == '/uav_0/fsc_open_manipulator/joint_states' and t not in names: names[t] = list(m.name)
    data[t][0].append((ts-T0)/1e9); data[t][1].append(conv[t](m))
res = {}
for t,(ts,v) in data.items():
    k = t.strip('/').replace('/','__')
    res[k+'__t'] = np.array(ts); res[k] = np.array(v, dtype=float)
    print(k, res[k].shape)
print(names)
np.savez_compressed(out, **res)
