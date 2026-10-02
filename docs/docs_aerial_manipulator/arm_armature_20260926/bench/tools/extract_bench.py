#!/usr/bin/env python3
"""Arm bench calibration bags (2026-09-09 / 2026-09-11) -> one npz per bag.

    source /opt/ros/humble/setup.bash; source ~/ros2_ws/rosdeps/local_setup.bash
    PYTHONNOUSERSITE=1 /usr/bin/python3 extract_bench.py [name-regex ...]

The Google-Drive download split each session over several folders (-1-001, -002,
-003) and a bag's .db3 is not always beside its metadata.yaml, so bags are found by
their .db3 and read straight from sqlite (the topics table carries the types).

Every array is in SLOT order j1..j4 (the joint_state_broadcaster publishes
[joint2, joint3, joint1, joint4]; it is re-ordered by name here, once).
    js_t    header stamp [s]      js_q  position [rad]    js_v  Present Velocity [rad/s]
    js_i    Present Current [counts, 2.69 mA]             js_grip gripper_left_joint [m]
    ref_*   smoothed_reference_joint_trajectory (position/velocity/effort = command)
    io_*    pwm_state_broadcaster gpio (pwm, cur, vel, pos, vin, temp)
"""
import glob, os, re, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = "/home/shiqi/fsc_PegasusSimulator/docs/experimental_data_ros2_bag"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
SESS = {"2026-09-09 (ID 12 & 13)": "s0909", "2026-09-11 (ID 11 & 14)": "s0911"}
J = ("joint1", "joint2", "joint3", "joint4")


def find_bags():
    out = {}
    for db in sorted(glob.glob(os.path.join(ROOT, "2026-09-*", "*", "*", "*.db3"))):
        sess = next(v for k, v in SESS.items() if k in db)
        out[(sess, os.path.basename(os.path.dirname(db)))] = db
    return out


def read(db):
    c = sqlite3.connect(db)
    tops = {n: (i, t) for i, n, t in c.execute("select id,name,type from topics")}
    res = {}
    for name, (tid, typ) in tops.items():
        key = ("js" if name.endswith("/joint_states") else
               "ref" if name.endswith("smoothed_reference_joint_trajectory") else
               "io" if name.endswith("gpio_states") else
               "vo" if name.endswith("velocity_observer") else
               "cl" if name.endswith("current_loop_debug") else None)
        if key is None:
            continue
        M = get_message(typ)
        rows = c.execute("select timestamp,data from messages where topic_id=? order by timestamp", (tid,)).fetchall()
        if not rows:
            continue
        tr = np.array([r[0] for r in rows]) * 1e-9
        msgs = [deserialize_message(r[1], M) for r in rows]
        if key in ("js", "ref", "vo"):
            names = list(msgs[0].name); ix = [names.index(j) for j in J]
            th = np.array([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in msgs])
            res[key + "_t"] = th; res[key + "_recv"] = tr
            for fld, s in (("position", "q"), ("velocity", "v"), ("effort", "i")):
                a = [list(getattr(m, fld)) for m in msgs]
                if a and len(a[0]) >= 4:
                    res[f"{key}_{s}"] = np.array([[r[k] for k in ix] for r in a])
            if key == "js" and "gripper_left_joint" in names:
                g = names.index("gripper_left_joint")
                res["js_grip"] = np.array([m.position[g] for m in msgs])
        elif key == "io":
            th = np.array([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec for m in msgs])
            res["io_t"] = th
            grp = list(msgs[0].interface_groups); ix = [grp.index(f"dxl{k}") for k in (1, 2, 3, 4)]
            iname = list(msgs[0].interface_values[0].interface_names)
            for nm, s in (("Present PWM", "pwm"), ("Present Current", "cur"), ("Present Velocity", "vel"),
                          ("Present Position", "pos"), ("Present Input Voltage", "vin"),
                          ("Present Temperature", "temp")):
                if nm in iname:
                    p = iname.index(nm)
                    res["io_" + s] = np.array([[m.interface_values[k].values[p] for k in ix] for m in msgs])
        elif key == "cl":
            L = max(len(m.data) for m in msgs)
            res["cl_t"] = tr
            res["cl"] = np.array([list(m.data) + [np.nan] * (L - len(m.data)) for m in msgs])
    return res


if __name__ == "__main__":
    pats = [re.compile(p) for p in sys.argv[1:]] or [re.compile(".")]
    os.makedirs(OUT, exist_ok=True)
    for (sess, name), db in find_bags().items():
        if not any(p.search(name) for p in pats):
            continue
        fn = os.path.join(OUT, f"{sess}__{name}.npz")
        if os.path.exists(fn):
            continue
        d = read(db)
        np.savez_compressed(fn, **d)
        n = len(d.get("js_t", []))
        print(f"{sess} {name}: js {n} ({(d['js_t'][-1]-d['js_t'][0]) if n else 0:.1f} s), "
              f"keys {sorted(k for k in d if not k.endswith('_t'))}", flush=True)
