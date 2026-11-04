#!/usr/bin/env python3
"""Replay a HARDWARE whole-body flight bag through TODAY's flight node (the whole-body 4-D L1 law),
open loop, and compare what it commands with what the flight commanded (2026-11-03, the
"no surprises" check after the mail route: the arm torques, the four motors, the law's debug).

    replay_bag_law.py BAG --yaml Y --out OUT [--domain 92]

Every topic the node READS is played from the bag (`ros2 bag play`, real time, private domain,
localhost only) -- odometry, PX4 attitude / status / odometry / EKF flags, the arm's joints,
velocity observer and reference, the planner's whole-body stream -- and none it WRITES (fmu/in/*,
its own debug, the arm torque command). DIRECT is requested at the bag time the recorded node
logged "ENTERING WHOLE-BODY DIRECT" and released where it left it. The recorded vehicle is the
plant, so a law unchanged since the flight commands the same torques up to the timing of its
250 Hz tick against the inputs: run it TWICE to measure that floor (the repeat), and read any
difference from the recording against it.

Starts the flight node: NEVER beside an Isaac campaign or a live stack (stale-node guards).
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

import numpy as np

NS = "/uav_0/"
OUT_TOPICS = {   # what the node writes: never played, recorded from the replay
    "tau": NS + "fsc_open_manipulator/external_torque_controller/joint_torque_command",
    "motors": NS + "fmu/in/actuator_motors",
    "dbg": NS + "fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug",
}
SKIP_PREFIX = (NS + "fmu/in/", NS + "fsc_autopilot_ros2/", "/rosout", "/parameter_events")
KEEP = (NS + "fsc_autopilot_ros2/whole_body_direct_actuation/reference",   # the planner's stream: an INPUT
        NS + "fsc_autopilot_ros2/position_controller/reference")
DBG = {"u1": [17], "tau_body": [18, 19, 20], "e_y": [24, 25, 26, 27], "d_t_hat": [31, 32, 33],
       "d_r_hat": [34, 35, 36], "d_q_hat": [37, 38, 39, 40], "tau_joint": [13, 14, 15, 16]}
JOINTS = ["joint1", "joint2", "joint3", "joint4"]


def bag_topics(path):
    import yaml
    m = yaml.safe_load(open(os.path.join(path, "metadata.yaml")))["rosbag2_bagfile_information"]
    return [t["topic_metadata"]["name"] for t in m["topics_with_message_count"] if t["message_count"] > 0]


def read_outputs(path, topics=None):
    """(t, values) per output from a bag: tau [4 by name], motors [4], dbg fields; DIRECT on/off times."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    from px4_msgs.msg import ActuatorMotors
    from std_msgs.msg import Float32MultiArray
    from rcl_interfaces.msg import Log
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=list(OUT_TOPICS.values()) + ["/rosout"]))
    t0 = r.get_metadata().starting_time.nanoseconds
    out = {"tau": [], "motors": [], "dbg": []}
    on = off = None
    while r.has_next():
        tp, data, t = r.read_next()
        ts = (t - t0) * 1e-9
        if tp == OUT_TOPICS["tau"]:
            m = deserialize_message(data, JointState)
            if all(j in m.name for j in JOINTS) and len(m.effort) == len(m.name):
                out["tau"].append([ts] + [m.effort[m.name.index(j)] for j in JOINTS])
        elif tp == OUT_TOPICS["motors"]:
            out["motors"].append([ts] + list(deserialize_message(data, ActuatorMotors).control[:4]))
        elif tp == OUT_TOPICS["dbg"]:
            d = deserialize_message(data, Float32MultiArray).data
            if len(d) > 40:
                out["dbg"].append([ts] + [d[i] for f in DBG.values() for i in f])
        else:
            m = deserialize_message(data, Log)
            if "ENTERING WHOLE-BODY DIRECT" in m.msg and on is None:
                on = ts
            if on is not None and off is None and ts > on + 1.0 and re.search(r"SAFETY guard|LEAVING WHOLE-BODY|-> SAFETY", m.msg):
                off = ts
    return {k: np.array(v) for k, v in out.items()}, on, off, t0


def gs_reference(path, before):
    """The drone ground station's position reference the SAFETY controller held before DIRECT:
    published before the recording started (0 messages in the 10-31 bags), but every
    position_controller/state carries it as position_reference."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from fsc_autopilot_ros2_msgs.msg import PositionControllerState
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=[NS + "fsc_autopilot_ros2/position_controller/state"]))
    t0 = r.get_metadata().starting_time.nanoseconds
    ref = None
    while r.has_next():
        _, data, t = r.read_next()
        if (t - t0) * 1e-9 > before:
            break
        p = deserialize_message(data, PositionControllerState).position_reference
        ref = (p.x, p.y, p.z)
    return ref


def diff(A, B, a, b):
    """Over the DIRECT window: per column, rms(A-B) / rms(A) and max |A-B|, B interpolated onto A."""
    A = A[(A[:, 0] >= a) & (A[:, 0] <= b)]
    res = []
    for k in range(1, A.shape[1]):
        Bk = np.interp(A[:, 0], B[:, 0], B[:, k])
        e = A[:, k] - Bk
        res.append((float(np.sqrt(np.nanmean(e ** 2))), float(np.sqrt(np.nanmean(A[:, k] ** 2))), float(np.nanmax(np.abs(e)))))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--domain", type=int, default=92)
    a = ap.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(a.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import SetBool
    from sensor_msgs.msg import JointState
    from px4_msgs.msg import ActuatorMotors
    from std_msgs.msg import Float32MultiArray
    from fsc_autopilot_ros2_msgs.msg import PositionControllerReference

    rec, on, off, bag_t0 = read_outputs(a.bag)
    if on is None:
        sys.exit("no DIRECT entry in the recorded log")
    off = off or rec["dbg"][-1, 0]
    gs_ref = gs_reference(a.bag, on)
    print(f"the ground station's reference before DIRECT: {gs_ref} (yaw 0)")
    play_topics = [t for t in bag_topics(a.bag)
                   if t not in OUT_TOPICS.values() and (t in KEEP or not t.startswith(SKIP_PREFIX))]
    print(f"recorded DIRECT {on:.2f} .. {off:.2f} s; playing {len(play_topics)} input topics")
    log = open(a.out + ".node.log", "w")
    node = subprocess.Popen(["ros2", "launch", "fsc_autopilot_ros2", "single_aerial_manipulator_whole_body_l1_direct_actuation_launch.py",
                             "uav_prefix:=uav_0", f"params_file:={a.yaml}"], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    play = None
    try:
        rclpy.init()
        n = Node("bag_replay_law_driver")
        clock = {"t": None}
        rep = {"tau": [], "motors": [], "dbg": []}

        def bt():
            return None if clock["t"] is None else (clock["t"] - bag_t0) * 1e-9
        n.create_subscription(Clock, "/clock", lambda m: clock.__setitem__("t", m.clock.sec * 10**9 + m.clock.nanosec),
                              qos_profile_sensor_data)   # `ros2 bag play --clock` publishes it BEST EFFORT

        def on_tau(m):
            if bt() and all(j in m.name for j in JOINTS) and len(m.effort) == len(m.name):
                rep["tau"].append([bt()] + [m.effort[m.name.index(j)] for j in JOINTS])

        def on_mot(m):
            if bt():
                rep["motors"].append([bt()] + list(m.control[:4]))

        def on_dbg(m):
            if bt() and len(m.data) > 40:
                rep["dbg"].append([bt()] + [m.data[i] for f in DBG.values() for i in f])
        n.create_subscription(JointState, OUT_TOPICS["tau"], on_tau, 500)
        n.create_subscription(ActuatorMotors, OUT_TOPICS["motors"], on_mot, qos_profile_sensor_data)
        n.create_subscription(Float32MultiArray, OUT_TOPICS["dbg"], on_dbg, 500)
        gs_pub = n.create_publisher(PositionControllerReference, NS + "fsc_autopilot_ros2/position_controller/reference", 10)
        gs_on = {"pub": gs_ref is not None}

        def gs_tick():
            if gs_on["pub"]:
                m = PositionControllerReference()
                m.header.stamp = n.get_clock().now().to_msg()
                m.position.x, m.position.y, m.position.z = map(float, gs_ref)
                m.yaw, m.yaw_unit = 0.0, PositionControllerReference.DEGREES
                gs_pub.publish(m)
        n.create_timer(0.5, gs_tick)
        ex = MultiThreadedExecutor(4)
        ex.add_node(n)
        threading.Thread(target=ex.spin, daemon=True).start()
        calls = []
        cli = n.create_client(SetBool, NS + "fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode")
        if not cli.wait_for_service(timeout_sec=40.0):
            sys.exit("the flight node did not come up (see " + a.out + ".node.log)")
        time.sleep(3.0)
        play = subprocess.Popen(["ros2", "bag", "play", a.bag, "--clock", "200", "--topics"] + play_topics,
                                stdout=open(a.out + ".play.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
        t_wait = time.time()
        while bt() is None:
            if time.time() - t_wait > 15.0 or play.poll() is not None:
                sys.exit("no /clock from ros2 bag play within 15 s -- see " + a.out + ".play.log")
            time.sleep(0.01)
        for t_call, val in ((on - 0.05, True), (off - 0.05, False)):
            while (bt() is None or bt() < t_call) and play.poll() is None:
                time.sleep(0.002)
            ok, msg = False, ""
            for _ in range(20):   # the entry gates may need a tick of fresh inputs
                fut = cli.call_async(SetBool.Request(data=val))
                t1 = time.time()
                while not fut.done() and time.time() - t1 < 3.0:
                    time.sleep(0.002)
                if fut.done():
                    ok, msg = fut.result().success, fut.result().message
                    if ok:
                        break
                time.sleep(0.05)
            if val and ok:
                gs_on["pub"] = False   # the ground station sends nothing in DIRECT (the planner owns the reference)
            calls.append((round(bt() or -1, 2), "DIRECT" if val else "SAFETY", ok, msg[:200]))
            print(f"  [{bt() or -1:6.2f}s] set_direct_mode {val}: {'OK' if ok else 'REFUSED'} {msg[:200]}")
        play.wait(timeout=300)
        time.sleep(1.0)
    finally:
        if play and play.poll() is None:
            os.killpg(play.pid, signal.SIGINT)
        os.killpg(node.pid, signal.SIGINT)
        try:
            node.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(node.pid, signal.SIGKILL)
        for f in (lambda: ex.shutdown(), rclpy.shutdown):   # either may not exist after a setup error
            try:
                f()
            except Exception:
                pass
    rep = {k: np.array(v) for k, v in rep.items()}
    a0, b0 = on + 1.0, off - 0.5   # past the entry tick, before the release
    summary = {"calls": calls, "window": [a0, b0]}
    names = {"tau": [f"tau_j{j + 1}" for j in range(4)], "motors": [f"motor{j + 1}" for j in range(4)],
             "dbg": [f"{f}[{i}]" for f, ix in DBG.items() for i in ix]}
    print(f"\n{'signal':16s} {'rms(rec)':>10s} {'rms(rec-rep)':>13s} {'ratio':>7s} {'max|rec-rep|':>13s}")
    for k in ("tau", "motors", "dbg"):
        if len(rep[k]) < 10 or len(rec[k]) < 10:
            print(f"{k}: NO DATA (rec {len(rec[k])}, replay {len(rep[k])})")
            continue
        for nm, (e, s, mx) in zip(names[k], diff(rec[k], rep[k], a0, b0)):
            summary[nm] = {"rms_rec": s, "rms_diff": e, "max_diff": mx}
            print(f"{nm:16s} {s:10.4f} {e:13.5f} {e / s if s > 0 else float('nan'):7.3f} {mx:13.5f}")
    np.savez(a.out + ".npz", **{"rec_" + k: v for k, v in rec.items()}, **{"rep_" + k: v for k, v in rep.items()})
    json.dump(summary, open(a.out + ".json", "w"), indent=1)
    print(f"wrote {a.out}.{{json,npz,node.log,play.log}}")


if __name__ == "__main__":
    main()
