"""Save the recorded position-controller reference waypoints on the SAME replay-time axis
as the attitude setpoints (t=0 at the first attitude setpoint), using bag receive times."""
import sqlite3, os, sys, numpy as np
from rclpy.serialization import deserialize_message
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference

def go(bag, exp_npz, out):
    db3 = [f for f in os.listdir(bag) if f.endswith(".db3")][0]
    con = sqlite3.connect(os.path.join(bag, db3)); cur = con.cursor()
    tid = {r[1]: r[0] for r in cur.execute("SELECT id,name FROM topics")}
    rows = cur.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
                       (tid["/uav_0/fsc_autopilot_ros2/position_controller/reference"],)).fetchall()
    con.close()
    d = np.load(exp_npz)
    t0_bag = int(d["sp_bag_t"][0])           # first attitude setpoint = replay t=0

    t_rel, pos, vel, acc, thr, yaw, yunit = [], [], [], [], [], [], []
    for t, data in rows:
        m = deserialize_message(bytes(data), PositionControllerReference)
        t_rel.append((t - t0_bag) * 1e-9)
        pos.append([m.position.x, m.position.y, m.position.z])
        vel.append([m.velocity.x, m.velocity.y, m.velocity.z])
        acc.append([m.acceleration.x, m.acceleration.y, m.acceleration.z])
        thr.append([m.thrust.x, m.thrust.y, m.thrust.z])
        yaw.append(m.yaw); yunit.append(m.yaw_unit)
    np.savez(out, t_rel=np.array(t_rel), position=np.array(pos), velocity=np.array(vel),
             acceleration=np.array(acc), thrust=np.array(thr),
             yaw=np.array(yaw), yaw_unit=np.array(yunit, dtype=np.int32))
    print(f"{out}: {len(t_rel)} refs, t_rel {t_rel[0]:.2f}..{t_rel[-1]:.2f}s")
    for i in range(len(t_rel)):
        print(f"   {t_rel[i]:8.2f}s  pos={pos[i]}  yaw={yaw[i]} unit={yunit[i]}")

B="/home/fsc-jupiter/Source/fsc_PegasusSimulator/docs/experimental_data_ros2_bag"
go(f"{B}/debug_recording_20260806_134620", sys.argv[1]+"/exp_A.npz", sys.argv[1]+"/ref_A.npz")
go(f"{B}/debug_recording_20260806_140303", sys.argv[1]+"/exp_B.npz", sys.argv[1]+"/ref_B.npz")
