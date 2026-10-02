"""Pull the 4-D attribution's wrench reading out of the hardware bags (0918, 0924).
[0] mode flag, [24..27] e_y, [72..77] w_e = S_e^T F_f (the FILTERED reading at the flown
omega_x, EE frame), [97..100] F_raw (UNFILTERED), [101..104] w_q_hat, [105] chi_free."""
import glob, os, sqlite3, sys, numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float32MultiArray
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "experimental_data_ros2_bag"))
OUT = os.path.join(os.path.dirname(__file__), "..", "data")
T = "/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug"
bags = sorted(glob.glob(os.path.join(ROOT, "0918*", "*", "*", "*.db3")) + glob.glob(os.path.join(ROOT, "0924*", "*", "*", "*.db3")))
for db in bags:
    nm = os.path.basename(os.path.dirname(db))
    con = sqlite3.connect(db)
    tid = con.execute("select id from topics where name=?", (T,)).fetchone()
    if tid is None:
        print(nm, "no wb debug"); continue
    rows = con.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[0],)).fetchall()
    t = np.array([r[0] for r in rows]) * 1e-9
    L = []
    for _, b in rows:
        d = deserialize_message(b, Float32MultiArray).data
        v = np.full(115, np.nan); v[:min(len(d), 115)] = d[:115]; L.append(v)
    A = np.array(L)
    np.savez_compressed(os.path.join(OUT, f"hw_{nm}.npz"), t=t - t[0], A=A)
    direct = A[:, 0] == 1
    print(f"{nm}: {len(t)} msgs, rate {1/np.median(np.diff(t)):.0f} Hz, DIRECT {direct.sum()} samples, len {np.nanmax(np.sum(np.isfinite(A),1))}")
