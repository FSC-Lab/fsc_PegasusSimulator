import numpy as np, sys
d = np.load(sys.argv[1])
sc = d["sc_ts"].astype(np.float64)*1e-6
print(f"sensor_combined: n={len(sc)} span={sc[-1]-sc[0]:.1f}s -> {len(sc)/(sc[-1]-sc[0]):.1f} Hz")
dt = np.diff(sc)
print(f"  dt ms: med {np.median(dt)*1e3:.2f} p1 {np.percentile(dt,1)*1e3:.2f} "
      f"p99 {np.percentile(dt,99)*1e3:.2f} max {dt.max()*1e3:.1f}")
ts = d["timesync"]
if len(ts):
    # columns: driver_wall_us, px4_msg_timestamp, remote_timestamp, observed_offset, estimated_offset
    w = ts[:,0]*1e-6; w -= w[0]
    est = ts[:,4]*1e-6
    A = np.stack([w, np.ones_like(w)],1)
    slope = np.linalg.lstsq(A, est, rcond=None)[0][0]
    print(f"timesync estimated_offset drift: {slope*1e6:+.1f} us/s  -> "
          f"sim/wall clock ratio = {1.0+slope:.5f}")
    print(f"  offset range {est.min():.4f}..{est.max():.4f} s over {w[-1]:.1f}s")
rl = d["reflog"]
print(f"reflog: {len(rl)} switches; wall vs px4 stamp columns available")
if len(rl) > 1:
    wall = rl[:,0]*1e-6; px4 = rl[:,1]*1e-6
    print(f"  wall span {wall[-1]-wall[0]:.2f}s   px4-stamp span {px4[-1]-px4[0]:.2f}s   "
          f"ratio {(px4[-1]-px4[0])/(wall[-1]-wall[0]):.5f}")
od = d["odom"]
print(f"odom n={len(od)}  final ENU pos {od[-1,2:5]}")
