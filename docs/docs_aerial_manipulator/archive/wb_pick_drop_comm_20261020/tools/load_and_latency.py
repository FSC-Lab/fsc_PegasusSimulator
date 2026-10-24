#!/usr/bin/env python3
"""Battery current/power at the basket lift (10-19 cut vs 10-20), Orin->laptop WiFi delay spread, and what the
flight node streamed after the pilot took STAB.  usage: load_and_latency.py   (/usr/bin/python3, numpy)"""
import os, numpy as np
H = os.path.dirname(os.path.abspath(__file__))
for tag, f in (("10-19 PD", os.path.join(H, "../../wb_pick_drop_wifi_20261019/npz/pd.npz")),
               ("10-20", os.path.join(H, "../npz/pd.npz"))):
    d = np.load(f, allow_pickle=True)
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
    b = "uav_0__fmu__out__battery_status"
    tb = (d[b + "__recv"] - t0) / 1e9; I = d[b + "__current"].astype(float); V = d[b + "__voltage"].astype(float)
    st = "uav_0__whole_body_planner__pick_place__status"
    ts = (d[st + "__recv"] - t0) / 1e9; s = d[st + "__data"]
    lift = [ts[i] for i in range(len(s)) if str(s[i]).startswith("FLYING exit_pick")][0]
    w = (tb > lift - 1) & (tb < lift + 6)
    fl = (tb > 20) & (tb < tb[-1] - 3)
    print(f"{tag}: exit_pick at {lift:.1f} s; battery current lift window max {I[w].max():.1f} A "
          f"(mean {I[w].mean():.1f}), whole flight max {I[fl].max():.1f} A, power max {(I*V)[fl].max():.0f} W, V {V[fl].min():.2f}-{V[fl].max():.2f}")
    js = "uav_0__fsc_open_manipulator__joint_states"
    lat = (d[js + "__recv"] - d[js + "__stamp"]) / 1e6
    lat -= np.median(lat)
    print(f"      Orin->laptop delay (joint_states recv-stamp, median removed): p99 {np.percentile(lat,99):.1f} ms, max {lat.max():.1f} ms")
    am = "uav_0__fmu__in__actuator_motors"
    ta = (d[am + "__recv"] - t0) / 1e9
    md = "uav_0__fsc_autopilot_ros2__whole_body_direct_actuation__mode"
    tm = (d[md + "__recv"] - t0) / 1e9; m = d[md + "__data"]
    off = [tm[i] for i in range(1, len(m)) if m[i] == "SAFETY" and m[i - 1] == "DIRECT"]
    if off:
        print(f"      DIRECT->SAFETY at {off[0]:.2f} s; actuator_motors messages after it: {int(np.sum(ta > off[0] + 0.01))}")
