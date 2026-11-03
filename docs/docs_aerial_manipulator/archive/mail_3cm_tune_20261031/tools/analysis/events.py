import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag = sys.argv[1]
TOP = ['/uav_0/whole_body_planner/pick_place/status', '/uav_0/whole_body_planner/status',
       '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/mode', '/uav_0/fsc_autopilot_ros2/controller_type',
       '/uav_0/fsc_open_manipulator/external_torque_controller/active_law', '/rosout',
       '/uav_0/fsc_open_manipulator/gripper_controller/gripper_cmd/_action/status',
       '/uav_0/whole_body_planner/target_joints', '/uav_0/whole_body_planner/pending_base',
       '/parameter_events', '/uav_0/fmu/out/vehicle_status_v1']
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
TOP = [t for t in TOP if t in tmap]
r.set_filter(rosbag2_py.StorageFilter(topics=TOP))
types = {t: get_message(tmap[t]) for t in TOP}
t0 = None; last = {}
while r.has_next():
    t, raw, ts = r.read_next()
    if t0 is None: t0 = ts
    m = deserialize_message(raw, types[t]); s = (ts - t0) / 1e9
    short = t.split('/')[-1] if 'pick_place' not in t else 'pp/' + t.split('/')[-1]
    if t == '/rosout':
        print(f"{s:7.2f} ROSOUT[{m.level}] {m.name}: {m.msg[:240]}")
    elif t.endswith('/_action/status'):
        print(f"{s:7.2f} GRIPPER_ACTION " + ", ".join(str(x.status) for x in m.status_list))
    elif t == '/parameter_events':
        print(f"{s:7.2f} PARAM {m.node} changed={[p.name for p in m.changed_parameters]}")
    elif t.endswith('vehicle_status_v1'):
        key = (m.arming_state, m.nav_state)
        if last.get(t) != key: print(f"{s:7.2f} PX4 arming={m.arming_state} nav={m.nav_state}"); last[t] = key
    elif hasattr(m, 'data') and isinstance(m.data, str):
        if last.get(t) != m.data: print(f"{s:7.2f} {short}: {m.data[:300]}"); last[t] = m.data
    elif hasattr(m, 'pose'):
        p = m.pose.position; print(f"{s:7.2f} {short}: ({p.x:.3f},{p.y:.3f},{p.z:.3f})")
    elif hasattr(m, 'data'):
        d = list(m.data)
        if last.get(t) != d: print(f"{s:7.2f} {short}[{len(d)}]: {[round(x,3) for x in d[:24]]}"); last[t] = d
print("t0_ns", t0)
