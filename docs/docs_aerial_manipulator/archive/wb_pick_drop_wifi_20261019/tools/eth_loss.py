#!/usr/bin/env python3
"""Frame loss on the PX4 -> Orin link while it is UP, 10-14..10-19 bags: holes in /fmu/out/sensor_combined
(400 Hz, PX4 timestamps) and whether the Orin's own republished copy (state_estimator/enu/imu/data) has the
same hole -- if it does, the samples were lost before the Orin's ROS side (Ethernet / PX4 / Agent), not on
WiFi to the laptop recorder.  Run from docs/experimental_data_ros2_bag with ROS 2 sourced, /usr/bin/python3."""
import sqlite3, glob, sys, numpy as np
from rclpy.serialization import deserialize_message
from px4_msgs.msg import SensorCombined
from sensor_msgs.msg import Imu
def load(c, name, cls, field):
    r=c.execute("SELECT id FROM topics WHERE name=?", (name,)).fetchone()
    if not r: return None
    out=[]
    for (b,) in c.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",(r[0],)):
        m=deserialize_message(bytes(b),cls)
        out.append(m.timestamp*1000 if field=="px4" else m.header.stamp.sec*10**9+m.header.stamp.nanosec)
    return np.array(out,dtype=np.int64)
for db in sorted(glob.glob("**/*.db3",recursive=True)):
    if db.split("/")[0][:4] not in ("1014","1015","1016","1019"): continue
    c=sqlite3.connect(db).cursor()
    sc=load(c,"/uav_0/fmu/out/sensor_combined",SensorCombined,"px4"); im=load(c,"/uav_0/state_estimator/enu/imu/data",Imu,"hdr")
    if sc is None or im is None or len(sc)<1000: continue
    d=np.diff(sc)/1e6; per=np.median(d)
    ok=d<1000   # drop the long outages, count single-frame holes only
    holes=np.where(ok&(d>1.5*per))[0]
    miss_px4=int(np.sum(np.round(d[holes]/per)-1))
    # holes the Orin's own copy also has (+-1 period) -> lost before the Orin (Ethernet / PX4 / Agent)
    both=0
    for i in holes:
        a,b=sc[i],sc[i+1]
        j=np.searchsorted(im,a); k=np.searchsorted(im,b+int(0.6*per*1e6))
        seg=im[max(j-1,0):k+1]
        if len(seg)>=2 and np.max(np.diff(seg))/1e6>1.5*per: both+=1
    n=len(sc)
    print(f"{db.split('/')[-2][:52]:52s} period {per:.2f} ms  samples {n:6d}  missing {miss_px4:5d} ({100*miss_px4/n:.3f}%)  holes {len(holes):4d}  also missing on the Orin copy {both:4d}")
