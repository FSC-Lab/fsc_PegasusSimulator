import sys, pickle
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag = sys.argv[1]; out = sys.argv[2]
topics = sys.argv[3:]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr','cdr'))
tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=topics))
types = {t: get_message(tmap[t]) for t in topics}
data = {t: [] for t in topics}
while r.has_next():
    t, raw, ts = r.read_next()
    data[t].append((ts, deserialize_message(raw, types[t])))
pickle.dump(data, open(out,'wb'))
for t in topics: print(t, len(data[t]))
