"""Version fingerprints from a 10-27 box-pushing bag: array lengths, rosout text, planner info."""
import sqlite3, sys, glob, os, collections
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bagdir = sys.argv[1]
out = sys.argv[2]
db = os.path.join(bagdir, [x for x in os.listdir(bagdir) if x.endswith('.db3')][0])
con = sqlite3.connect(db)
topics = {r[0]: (r[1], r[2]) for r in con.execute('select id,name,type from topics')}
name2id = {v[0]: k for k, v in topics.items()}

def msgs(name):
    if name not in name2id:
        return
    tid = name2id[name]
    cls = get_message(topics[tid][1])
    for ts, data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        yield ts, deserialize_message(data, cls)

t0 = con.execute('select min(timestamp) from messages').fetchone()[0]
f = open(out, 'w')
def p(*a):
    print(*a, file=f)

p('== array lengths (Float*MultiArray topics)')
for tid, (name, typ) in sorted(topics.items(), key=lambda x: x[1][0]):
    if typ.endswith('MultiArray'):
        c = collections.Counter(len(m.data) for _, m in msgs(name))
        p(f'{name}: {dict(c)}')

p('\n== /rosout')
for ts, m in msgs('/rosout'):
    p(f'{(ts-t0)/1e9:8.2f} [{m.name}] {m.msg}')

p('\n== /parameter_events')
for ts, m in msgs('/parameter_events'):
    p(m)

for name in ['/uav_0/fsc_open_manipulator/external_torque_controller/active_law',
             '/uav_0/fsc_autopilot_ros2/vehicle_info',
             '/uav_0/fsc_autopilot_ros2/controller_type']:
    p(f'\n== {name}')
    seen = []
    for ts, m in msgs(name):
        s = str(m)
        if s not in seen:
            seen.append(s)
            p(f'{(ts-t0)/1e9:8.2f}', s)

p('\n== push_pull/info (every distinct message)')
last = None
for ts, m in msgs('/uav_0/whole_body_planner/push_pull/info'):
    d = list(m.data)
    if d != last:
        p(f'{(ts-t0)/1e9:8.2f} n={len(d)}', [round(x, 6) for x in d])
        last = d

p('\n== push_pull/status (distinct)')
last = None
for ts, m in msgs('/uav_0/whole_body_planner/push_pull/status'):
    if m.data != last:
        p(f'{(ts-t0)/1e9:8.2f}', m.data)
        last = m.data

p('\n== robot_description (length + a hash)')
import hashlib
for ts, m in msgs('/uav_0/fsc_open_manipulator/robot_description'):
    p(len(m.data), hashlib.sha1(m.data.encode()).hexdigest())
    open(out + '.urdf', 'w').write(m.data)
