"""Extract a box-pushing bag (sqlite, no metadata needed) into an npz of the series this check needs.

Times are receive stamps relative to the bag's first message [s].
"""
import os, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bagdir, out = sys.argv[1], sys.argv[2]
db = os.path.join(bagdir, [x for x in os.listdir(bagdir) if x.endswith('.db3')][0])
con = sqlite3.connect(db)
topics = {r[1]: (r[0], r[2]) for r in con.execute('select id,name,type from topics')}
t0 = con.execute('select min(timestamp) from messages').fetchone()[0]


def read(name, f):
    if name not in topics:
        return np.zeros((0, 1))
    tid, typ = topics[name]
    cls = get_message(typ)
    rows = []
    for ts, data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        v = f(deserialize_message(data, cls))
        if v is None:
            continue
        rows.append([(ts - t0) * 1e-9] + list(v))
    return np.array(rows, dtype=float) if rows else np.zeros((0, 1))


def pose(m):
    p, q = m.pose.position, m.pose.orientation
    return [p.x, p.y, p.z, q.w, q.x, q.y, q.z]


def mocap(m):
    # fsc Mocap: try the usual field names
    for a in ('pose',):
        if hasattr(m, a):
            return pose(getattr(m, a)) if hasattr(getattr(m, a), 'pose') else \
                [m.pose.position.x, m.pose.position.y, m.pose.position.z,
                 m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z]
    return None


d = {}
d['arr'] = read('/uav_0/whole_body_planner/push_pull/arrival_error', lambda m: list(m.data)[:30] + [np.nan] * (30 - len(m.data)))
d['dbg'] = read('/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug',
                lambda m: list(m.data)[:118] if len(m.data) >= 118 else None)
d['vrpn_box'] = read('/vrpn_mocap/obj_0/pose', pose)
d['vrpn_uav'] = read('/vrpn_mocap/uav_0/pose', pose)
d['obj'] = read('/obj_0/mocap', mocap)
d['odom'] = read('/uav_0/state_estimator/local_position/odom',
                 lambda m: pose(m.pose) + [m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z])
d['att'] = read('/uav_0/fmu/out/vehicle_attitude', lambda m: list(m.q))
d['ee'] = read('/uav_0/whole_body_planner/current_ee', pose)
d['base'] = read('/uav_0/whole_body_planner/current_base', pose)
d['js'] = read('/uav_0/fsc_open_manipulator/joint_states', lambda m: list(m.position)[:5] + [0] * (5 - len(m.position)))
d['jsn'] = np.array([0])
d['mot'] = read('/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/motors_debug', lambda m: list(m.control)[:4])
d['bat'] = read('/uav_0/fmu/out/battery_status', lambda m: [m.voltage_v, m.current_a])
d['guard'] = read('/uav_0/state_estimator/mocap_guard', lambda m: list(m.data))
names = None
tid = topics['/uav_0/fsc_open_manipulator/joint_states'][0]
data = con.execute('select data from messages where topic_id=? limit 1', (tid,)).fetchone()[0]
names = list(deserialize_message(data, get_message('sensor_msgs/msg/JointState')).name)
status = []
tid = topics.get('/uav_0/whole_body_planner/push_pull/status', topics.get('/uav_0/whole_body_planner/status'))[0]
for ts, data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
    status.append(((ts - t0) * 1e-9, deserialize_message(data, get_message('std_msgs/msg/String')).data))
np.savez(out, js_names=np.array(names), status_t=np.array([s[0] for s in status]),
         status=np.array([s[1] for s in status]), **d)
print(out, {k: v.shape for k, v in d.items()}, names)
