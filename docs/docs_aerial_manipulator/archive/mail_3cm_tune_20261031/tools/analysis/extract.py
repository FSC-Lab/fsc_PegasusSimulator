"""Extract the mail-delivery bag signals into an npz (absolute bag ns timestamps)."""
import sys, numpy as np, rosbag2_py
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
bag, out = sys.argv[1], sys.argv[2]
P = lambda p: [p.x, p.y, p.z]
Q = lambda q: [q.w, q.x, q.y, q.z]
conv = {
 '/vrpn_mocap/uav_0/pose': lambda m: P(m.pose.position) + Q(m.pose.orientation),
 '/vrpn_mocap/obj_0/pose': lambda m: P(m.pose.position) + Q(m.pose.orientation),
 '/uav_0/mocap': lambda m: P(m.pose.position) + [m.twist.linear.x, m.twist.linear.y, m.twist.linear.z],
 '/uav_0/state_estimator/local_position/odom': lambda m: P(m.pose.pose.position) + Q(m.pose.pose.orientation) + P(m.twist.twist.linear),
 '/uav_0/whole_body_planner/current_ee': lambda m: P(m.pose.position) + Q(m.pose.orientation),
 '/uav_0/whole_body_planner/current_base': lambda m: P(m.pose.position) + Q(m.pose.orientation),
 '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/reference': lambda m: P(m.x_cd) + P(m.x_cd_dot),
 '/uav_0/fsc_open_manipulator/joint_states': lambda m: (list(m.position) + [np.nan] * 6)[:6] + (list(m.effort) + [np.nan] * 6)[:6],
 '/uav_0/fmu/in/actuator_motors': lambda m: list(m.control[:4]),
 '/uav_0/fmu/out/battery_status': lambda m: [m.voltage_v, m.current_a],
 '/uav_0/fmu/out/vehicle_attitude': lambda m: list(m.q),   # [w, x, y, z]; the tilt source when there is no VRPN (sim)
 '/uav_0/whole_body_planner/pick_place/arrival_error': lambda m: (list(m.data) + [np.nan] * 8)[:8],
 '/uav_0/whole_body_planner/pick_place/info': lambda m: (list(m.data) + [np.nan] * 88)[:88],
}
store = get_typestore(Stores.ROS2_HUMBLE)
add = {}
for f in (Path.home() / 'Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2_msgs/msg').glob('*.msg'):
    add.update(get_types_from_msg(f.read_text(), f'fsc_autopilot_ros2_msgs/msg/{f.stem}'))
store.register(add)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
topics = [t for t in conv if t in tmap]
r.set_filter(rosbag2_py.StorageFilter(topics=topics))
RB = {t for t in topics if tmap[t].startswith('fsc_autopilot_ros2_msgs/')}
types = {t: (None if t in RB else get_message(tmap[t])) for t in topics}
data = {t: ([], []) for t in topics}; names = None
while r.has_next():
    t, raw, ts = r.read_next()
    m = store.deserialize_cdr(raw, tmap[t]) if t in RB else deserialize_message(raw, types[t])
    if t.endswith('joint_states') and names is None: names = list(m.name)
    data[t][0].append(ts); data[t][1].append(conv[t](m))
res = {'joint_names': np.array(names or [])}
for t, (ts, v) in data.items():
    k = t.strip('/').split('/')[-1] if 'pick_place' not in t else 'pp_' + t.split('/')[-1]
    if t.startswith('/vrpn_mocap'): k = 'vrpn_' + t.split('/')[2]
    if t == '/uav_0/mocap': k = 'mocap'
    if t == '/uav_0/fmu/out/vehicle_attitude': k = 'att'
    if t == '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/reference': k = 'wb_ref'
    res[k + '_t'] = np.array(ts, dtype=np.int64); res[k] = np.array(v, dtype=float)
np.savez_compressed(out, **res)
print(out, {k: v.shape for k, v in res.items() if not k.endswith('_t')})
