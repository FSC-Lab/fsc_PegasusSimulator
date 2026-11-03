import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
bag, out = sys.argv[1], sys.argv[2]
TOP = '/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug'
IDX = [0] + list(range(41, 52)) + [55] + list(range(97, 105))   # mode, motors(4), x_cd(3), x_c(3), n_sat, unalloc thrust, F_hat(4), w_hat_q(4)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=[TOP]))
ts, rows = [], []
while r.has_next():
    t, raw, stamp = r.read_next()
    d = deserialize_message(raw, Float32MultiArray).data
    rows.append([d[i] if i < len(d) else np.nan for i in IDX]); ts.append(stamp)
np.savez_compressed(out, t=np.array(ts, dtype=np.int64), x=np.array(rows, dtype=float), idx=np.array(IDX))
print(out, len(ts))
