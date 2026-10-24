#!/usr/bin/env python3
"""Quick link-health check of one pick-and-drop bag, from extract_timing.py's npz (10-19 tool).

Per link, by the laptop recorder's RECEIVE time (and PX4's own timestamps for the PX4 streams):
largest gaps, count of gaps > 50 / 100 / 300 ms, rate. Also the timesync RTT, the node's mode
switches, PX4 nav/arming/failsafe changes and every /rosout line that mentions feedback/stale/lost.
usage: link_check.py <pd.npz>     (/usr/bin/python3, numpy only)"""
import sys
import numpy as np

d = np.load(sys.argv[1], allow_pickle=True)
t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
T1 = max(d[k][-1] for k in d.files if k.endswith("__recv") and len(d[k]))
print(f"bag span {(T1 - t0) / 1e9:.1f} s")

def gaps(name, use="recv", nom=None):
    k = name + ("__recv" if use == "recv" else "__px4ts")
    if k not in d.files or len(d[k]) < 3:
        print(f"  {name:70s} MISSING"); return
    t = d[k].astype(np.float64) / (1e9 if use == "recv" else 1e6)
    dt = np.diff(t)
    i = np.argsort(dt)[::-1][:3]
    tr = (d[name + "__recv"].astype(np.float64) - t0) / 1e9
    top = ", ".join(f"{dt[j]*1e3:.0f} ms @{tr[j]:.1f}s" for j in i)
    print(f"  {name[:62]:62s} {use:5s} n {len(t):6d} {len(t)/(t[-1]-t[0]):6.1f} Hz  "
          f">50ms {np.sum(dt>0.05):3d} >100 {np.sum(dt>0.1):3d} >300 {np.sum(dt>0.3):2d} | {top}")

print("\nPX4 -> Agent (Ethernet) -> Orin ROS -> WiFi -> laptop")
for n in ("uav_0__fmu__out__sensor_combined", "uav_0__fmu__out__vehicle_attitude",
          "uav_0__fmu__out__vehicle_odometry", "uav_0__fmu__out__vehicle_status_v1",
          "uav_0__fmu__out__timesync_status", "uav_0__fmu__out__battery_status"):
    gaps(n, "recv"); gaps(n, "px4")
print("\nOrin-published (Orin -> laptop WiFi only)")
for n in ("uav_0__fsc_open_manipulator__joint_states", "uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__wb_control_debug",
          "uav_0__fmu__in__actuator_motors", "uav_0__state_estimator__enu__imu__data",
          "uav_0__state_estimator__local_position__odom"):
    gaps(n)
print("\nlaptop -> Orin WiFi (mocap relayed by the Orin estimator to PX4)")
for n in ("uav_0__mocap", "vrpn_mocap__uav_0__pose", "uav_0__fmu__in__vehicle_visual_odometry"):
    gaps(n)

# the Orin's own IMU copy vs PX4 sensor_combined: holes before the Orin's ROS side
sc = d["uav_0__fmu__out__sensor_combined__px4ts"].astype(np.int64)
per = np.median(np.diff(sc))
holes = np.where(np.diff(sc) > 1.5 * per)[0]
miss = int(np.sum(np.round(np.diff(sc)[holes] / per) - 1))
print(f"\nsensor_combined (PX4 clock): period {per/1e3:.2f} ms, missing samples {miss} ({100*miss/len(sc):.3f} %), holes {len(holes)}, "
      f"largest {np.diff(sc).max()/1e3:.1f} ms")

ts = "uav_0__fmu__out__timesync_status"
rtt = d[ts + "__rtt"].astype(float)
print(f"timesync RTT us: median {np.median(rtt):.0f}  p99 {np.percentile(rtt,99):.0f}  max {rtt.max():.0f}; "
      f"est offset span {np.ptp(d[ts+'__est_off'].astype(float))/1e3:.2f} ms")

vs = "uav_0__fmu__out__vehicle_status_v1"
tr = (d[vs + "__recv"] - t0) / 1e9
prev = None
print("\nPX4 status changes (nav_state, arming, failsafe, offboard_lost):")
for i in range(len(tr)):
    cur = (int(d[vs + "__nav_state"][i]), int(d[vs + "__arming_state"][i]), int(d[vs + "__failsafe"][i]), int(d[vs + "__offboard_lost"][i]))
    if cur != prev:
        print(f"  {tr[i]:7.2f} s  nav {cur[0]:2d} arm {cur[1]} failsafe {cur[2]} offboard_lost {cur[3]}"); prev = cur
md = "uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__mode"
tr = (d[md + "__recv"] - t0) / 1e9; prev = None
print("\nwhole-body node mode:")
for i in range(len(tr)):
    if d[md + "__data"][i] != prev:
        print(f"  {tr[i]:7.2f} s  {d[md+'__data'][i]}"); prev = d[md + "__data"][i]
st = "uav_0__whole_body_planner__pick_place__status"
tr = (d[st + "__recv"] - t0) / 1e9; prev = None
print("\npick-place status:")
for i in range(len(tr)):
    if d[st + "__data"][i] != prev:
        print(f"  {tr[i]:7.2f} s  {d[st+'__data'][i][:110]}"); prev = d[st + "__data"][i]
ro = "rosout"
tr = (d[ro + "__recv"] - t0) / 1e9
print("\n/rosout warnings/errors + feedback/stale/lost/link lines:")
for i in range(len(tr)):
    m = str(d[ro + "__msg"][i]); lv = int(d[ro + "__level"][i])
    if lv >= 30 or any(w in m.lower() for w in ("feedback", "stale", "lost", "timeout", "watchdog", "link")):
        print(f"  {tr[i]:7.2f} s  L{lv} {d[ro+'__name'][i]}: {m[:150]}")
