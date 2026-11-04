#!/usr/bin/env python3
"""Replay a HARDWARE mail-delivery bag through TODAY's planner, open loop, and compare the
whole-body reference it streams with the one the flight recorded (2026-11-03, user: "run it
with previous mail box delivery rosbag to ensure no changes ... I do not want any surprises").

    replay_bag_planner.py BAG --yaml Y --out OUT [--domain 91]

The bag's MEASUREMENTS (odometry, PX4 attitude, arm joints, the flight node's mode, obj_0) are
played into a fresh planner (`ros2 bag play`, real time, private ROS domain, localhost only);
the planner's state that was set BEFORE the recording started (Adjust's shift, the Gets) is
written as parameters; and every button the operator pressed is pressed at the bag time the
recorded planner log shows it ("planning <leg> from the current hold", "descend_pick:",
"exit_pick:", "exit_place:", "planned:"). The recorded vehicle is the plant, so every leg is
re-planned from the same hold the flight's planner had. Then, per leg, the replay's
streamed reference (x_cd, r_ed, q_d, b1_d, b1_de) is time-aligned to the recorded one and the
largest differences are reported -- with the yaml that flew, every number should be ~0.

Starts a planner node: NEVER run it beside an Isaac campaign (stale-node guards).
"""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import numpy as np

def yaml_planner_value(yaml_path, key):
    """A planner-section value of the yaml (flow lists / scalars only)."""
    import yaml
    d = yaml.safe_load(open(yaml_path))
    return d["/**/whole_body_trajectory_planner"]["ros__parameters"][key]


def derive_state(bag, yaml_path):
    """The planner state set BEFORE the recording started, from the bag itself: Adjust's shift
    (pick_place/info [0..1]) on the yaml's nominal Start / Land Start / Land, the pick capture
    ([60..62] + its yaw [80]), the place point (execute_place's planned claw goal minus
    Rz(place yaw) * pick_place_ee_offset) and the GS's Mailbox Start write (/parameter_events)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import Float64MultiArray
    from rcl_interfaces.msg import ParameterEvent
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=[PP + "info", "/parameter_events"]))
    info, extra = None, {}
    while r.has_next():
        tp, data, _ = r.read_next()
        if tp == "/parameter_events":
            for p in deserialize_message(data, ParameterEvent).changed_parameters:
                if p.name == "pick_place_place_start":
                    extra[p.name] = list(p.value.double_array_value)
        elif info is None:
            dd = deserialize_message(data, Float64MultiArray).data
            if dd[4] == 1.0:
                info = list(dd)
    if info is None:
        sys.exit(f"{bag}: no planned pick_place/info (Plan before the recording?)")
    sh = info[0:2]
    base = {}
    for k in ("pick_place_start", "pick_place_land_start", "pick_place_land"):
        v = [float(x) for x in yaml_planner_value(yaml_path, k)]
        base[k] = [v[0] + sh[0], v[1] + sh[1], v[2], v[3]]
    off = [float(x) for x in yaml_planner_value(yaml_path, "pick_place_ee_offset")]
    leg3 = info[14 + 7 * 3: 14 + 7 * 4]
    y = math.radians(leg3[3])
    c, s_ = math.cos(y), math.sin(y)
    place = [leg3[4] - (c * off[0] - s_ * off[1]), leg3[5] - (s_ * off[0] + c * off[1]), leg3[6] - off[2], leg3[3]]
    params = dict(base)
    params["pick_place_pick_point"] = info[60:63] + [info[80]]
    params["pick_place_place_point"] = place
    params.update(extra)
    params["pick_place_place_drop"] = False
    params["pick_place_place_insert"] = True
    return params


INPUTS = [
    "/uav_0/state_estimator/local_position/odom",
    "/uav_0/fmu/out/vehicle_attitude",
    "/uav_0/fsc_open_manipulator/joint_states",
    "/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/mode",
    "/obj_0/mocap",
    "/uav_0/fsc_open_manipulator/external_torque_controller/smoothed_reference_joint_trajectory",
]
REF = "/uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/reference"
ARM_REF = "/uav_0/fsc_open_manipulator/external_torque_controller/reference_joint_trajectory"
STATUS = "/uav_0/whole_body_planner/pick_place/status"
PP = "/uav_0/whole_body_planner/pick_place/"
LEGS = ["go_to_start", "execute_pick", "go_to_place_start", "execute_place", "go_to_land_start", "execute_land"]
BUTTONS = [   # recorded planner log line -> the service the operator / GS called
    (re.compile(r"pick-and-place planned:"), "plan", 0.15),
    (re.compile(r"planning (\w+) from the current hold"), None, 0.05),
    (re.compile(r"descend_pick: descending"), "descend_pick", 0.05),
    (re.compile(r"exit_pick: climbing"), "exit_pick", 0.05),
    # the Insert PRESS (10-31: the service handler logs this; the automatic insert logs "AUTO INSERT")
    (re.compile(r"descend_place: sliding"), "descend_place", 0.05),
    (re.compile(r"exit_place: from where"), "exit_place", 0.05),
]


def ref_row(m):
    v = lambda a: [a.x, a.y, a.z]
    return v(m.x_cd) + v(m.r_ed) + list(m.q_d) + v(m.b1_d) + v(m.b1_de)   # 3+3+4+3+3 = 16


def read_bag(path):
    """The recorded reference, arm reference, statuses and the operator's button times."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from fsc_autopilot_ros2_msgs.msg import WholeBodyReference
    from trajectory_msgs.msg import JointTrajectory
    from std_msgs.msg import String
    from rcl_interfaces.msg import Log
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    r.set_filter(rosbag2_py.StorageFilter(topics=[REF, ARM_REF, STATUS, "/rosout"]))
    t0 = r.get_metadata().starting_time.nanoseconds
    ref, arm, status, calls = [], [], [], []
    while r.has_next():
        tp, data, t = r.read_next()
        ts = (t - t0) * 1e-9
        if tp == REF:
            ref.append([ts] + ref_row(deserialize_message(data, WholeBodyReference)))
        elif tp == ARM_REF:
            m = deserialize_message(data, JointTrajectory)
            if m.points:
                arm.append([ts] + list(m.points[0].positions[:4]))
        elif tp == STATUS:
            status.append((ts, deserialize_message(data, String).data))
        else:
            m = deserialize_message(data, Log)
            if m.name.endswith("whole_body_trajectory_planner"):
                for rx, srv, lead in BUTTONS:
                    g = rx.search(m.msg)
                    if g:
                        calls.append((ts - lead, srv or g.group(1)))
    return np.array(ref), np.array(arm), status, sorted(calls), t0


def leg_windows(status):
    """[(leg, t_start, t_end)] from the FLYING <leg> ... DONE/INCOMPLETE edges."""
    out, cur = [], None
    for ts, s in status:
        w = s.split()
        if not w:
            continue
        if w[0] == "FLYING" and len(w) > 1 and (cur is None or cur[0] != w[1]):
            if cur:
                out.append((cur[0], cur[1], ts))
            cur = (w[1], ts)
        elif w[0] in ("DONE", "INCOMPLETE:", "ABORTING", "ABORTED", "COMPLETE", "IDLE", "NOT") and cur:
            out.append((cur[0], cur[1], ts))
            cur = None
    if cur:
        out.append((cur[0], cur[1], status[-1][0]))
    return out


def compare(rec, rep, wrec, wrep):
    """Per recorded leg: the replay's same leg, best time shift within +-0.15 s, largest differences."""
    rows = []
    used = set()
    for leg, a, b in wrec:
        cand = [i for i, (l2, _, _) in enumerate(wrep) if l2 == leg and i not in used]
        if not cand:
            rows.append({"leg": leg, "t": round(a, 2), "missing_in_replay": True})
            continue
        i = cand[0]
        used.add(i)
        _, a2, b2 = wrep[i]
        R = rec[(rec[:, 0] >= a) & (rec[:, 0] <= b)]
        if len(R) < 5:
            continue
        best = None
        for sh in np.arange(-0.15, 0.1501, 0.002):
            tq = R[:, 0] - a + a2 + sh
            ok = (tq >= rep[0, 0]) & (tq <= rep[-1, 0])
            if ok.sum() < 0.9 * len(R):
                continue
            P = np.column_stack([np.interp(tq[ok], rep[:, 0], rep[:, k]) for k in range(1, rep.shape[1])])
            e = np.sqrt(np.mean(np.sum((P[:, 0:3] - R[ok, 1:4]) ** 2, axis=1)))
            if best is None or e < best[0]:
                best = (e, sh, P, ok)
        if best is None:
            rows.append({"leg": leg, "t": round(a, 2), "no_overlap": True})
            continue
        _, sh, P, ok = best
        D = P - R[ok, 1:]
        ang = lambda u, v: np.degrees(np.arccos(np.clip(np.sum(u * v, 1) /
                                                       (np.linalg.norm(u, axis=1) * np.linalg.norm(v, axis=1) + 1e-12), -1, 1)))
        rows.append({
            "leg": leg, "t": round(a, 2), "T_rec": round(b - a, 2), "T_rep": round(b2 - a2, 2), "shift_ms": round(1e3 * (a2 - a + sh), 1),
            "x_cd_mm": round(1e3 * np.max(np.linalg.norm(D[:, 0:3], axis=1)), 2),
            "r_ed_mm": round(1e3 * np.max(np.linalg.norm(D[:, 3:6], axis=1)), 2),
            "q_d_deg": [round(float(np.degrees(np.max(np.abs(D[:, 6 + j])))), 3) for j in range(4)],
            "b1_d_deg": round(float(np.max(ang(P[:, 10:13], R[ok, 11:14]))), 3),
            "b1_de_deg": round(float(np.max(ang(P[:, 13:16], R[ok, 14:17]))), 3),
            "q4_end_deg_rec": round(float(np.degrees(R[-1, 10])), 2), "q4_end_deg_rep": round(float(np.degrees(P[-1, 9])), 2),
        })
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--domain", type=int, default=91)
    a = ap.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(a.domain)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, Log
    from fsc_autopilot_ros2_msgs.msg import WholeBodyReference
    from trajectory_msgs.msg import JointTrajectory
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

    rec_ref, rec_arm, rec_status, calls, bag_t0 = read_bag(a.bag)
    lay = {"params": derive_state(a.bag, a.yaml)}
    for k, v in lay["params"].items():
        print(f"  state {k}: {[round(x, 5) for x in v] if isinstance(v, list) else v}")
    print(f"recorded: {len(rec_ref)} reference samples, {len(rec_status)} statuses, buttons {[(round(t, 1), s) for t, s in calls]}")
    log = open(a.out + ".planner.log", "w")
    planner = subprocess.Popen(
        ["ros2", "launch", "fsc_trajectory_planner", "whole_body_trajectory_planner_launch.py", "uav_prefix:=uav_0",
         f"params_file:={a.yaml}", "base_com:=[0.0,-0.017854,0.0]", "arm_joint_sign:=[-1.0,1.0,1.0,-1.0]",
         "hold_ee_world:=false"], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    play = None
    rclpy.init()
    n = Node("bag_replay_driver")
    clock = {"t": None}
    rep_ref, rep_arm, rep_status, rep_log = [], [], [], []
    lock = threading.Lock()

    def bag_time():
        return None if clock["t"] is None else (clock["t"] - bag_t0) * 1e-9

    n.create_subscription(Clock, "/clock", lambda m: clock.__setitem__("t", m.clock.sec * 10**9 + m.clock.nanosec),
                          qos_profile_sensor_data)   # `ros2 bag play --clock` publishes it BEST EFFORT
    n.create_subscription(WholeBodyReference, REF, lambda m: rep_ref.append([bag_time()] + ref_row(m)) if bag_time() else None, 200)

    def on_arm(m):
        if m.points and bag_time():
            rep_arm.append([bag_time()] + list(m.points[0].positions[:4]))
    n.create_subscription(JointTrajectory, ARM_REF, on_arm, 200)
    latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(String, STATUS, lambda m: rep_status.append((bag_time() or -1.0, m.data)), latched)
    n.create_subscription(Log, "/rosout", lambda m: rep_log.append((bag_time() or -1.0, m.msg))
                          if m.name.endswith("whole_body_trajectory_planner") else None, 100)
    ex = MultiThreadedExecutor(4)
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        cli = n.create_client(SetParameters, "/uav_0/whole_body_trajectory_planner/set_parameters")
        if not cli.wait_for_service(timeout_sec=30.0):
            sys.exit("the planner did not come up (see " + a.out + ".planner.log)")
        time.sleep(2.0)
        ps = []
        for k, v in lay["params"].items():
            pv = ParameterValue()
            if isinstance(v, bool):
                pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, v
            else:
                pv.type, pv.double_array_value = ParameterType.PARAMETER_DOUBLE_ARRAY, [float(x) for x in v]
            ps.append(Parameter(name=k, value=pv))
        for p in ps:   # one at a time: a refusal names its own key
            req = SetParameters.Request(parameters=[p])
            fut = cli.call_async(req)
            t1 = time.time()
            while not fut.done() and time.time() - t1 < 5.0:
                time.sleep(0.01)
            r = fut.result().results[0] if fut.done() else None
            print(f"  set {p.name}: {'ok' if r and r.successful else 'REFUSED ' + (r.reason if r else 'no answer')}")
        clients = {}
        for s in set(s for _, s in calls):
            clients[s] = n.create_client(Trigger, PP + s)
        play = subprocess.Popen(["ros2", "bag", "play", a.bag, "--clock", "200", "--topics"] + INPUTS,
                                stdout=open(a.out + ".play.log", "w"), stderr=subprocess.STDOUT, start_new_session=True)
        t_wait = time.time()
        while bag_time() is None:
            if time.time() - t_wait > 15.0 or play.poll() is not None:
                sys.exit("no /clock from ros2 bag play within 15 s -- see " + a.out + ".play.log")
            time.sleep(0.01)
        results = []
        for t_call, s in calls:
            while bag_time() is None or bag_time() < t_call:
                time.sleep(0.002)
                if play.poll() is not None:
                    break
            ok, msg = False, "not tried"
            for _ in range(20):   # a press the planner is not ready for yet (a leg finishing): retry for ~1 s
                fut = clients[s].call_async(Trigger.Request())
                t1 = time.time()
                while not fut.done() and time.time() - t1 < 3.0:
                    time.sleep(0.002)
                if fut.done():
                    ok, msg = fut.result().success, fut.result().message
                    if ok:
                        break
                time.sleep(0.05)
            results.append((round(bag_time() or -1, 2), s, ok, msg[:140]))
            print(f"  [{bag_time() or -1:6.2f}s] {s}: {'OK' if ok else 'REFUSED'} {msg[:140]}")
        play.wait(timeout=200)
        time.sleep(1.0)
    finally:
        if play and play.poll() is None:
            os.killpg(play.pid, signal.SIGINT)
        os.killpg(planner.pid, signal.SIGINT)
        try:
            planner.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(planner.pid, signal.SIGKILL)
        ex.shutdown()
        rclpy.shutdown()
    rep = np.array([r for r in rep_ref if r[0] is not None])
    wrec, wrep = leg_windows(rec_status), leg_windows([(t, s) for t, s in rep_status if t >= 0])
    rows = compare(rec_ref, rep, wrec, wrep)
    np.savez(a.out + ".npz", rec_ref=rec_ref, rep_ref=rep, rec_arm=rec_arm, rep_arm=np.array(rep_arm))
    seq_rec = [s.split(" -- ")[0][:60] for _, s in rec_status]
    seq_rep = [s.split(" -- ")[0][:60] for t, s in rep_status if t >= 0]
    json.dump({"yaml": a.yaml, "bag": a.bag, "calls": results, "legs": rows, "status_rec": seq_rec, "status_rep": seq_rep,
               "planner_log": [(round(t, 2), m) for t, m in rep_log]}, open(a.out + ".json", "w"), indent=1)
    print(f"\n{'leg':18s} {'t':>6s} {'T rec/rep':>11s} {'shift':>7s} {'x_cd':>7s} {'r_ed':>7s} {'q_d max diff [deg] j1..j4':>28s} {'b1_d':>6s} {'b1_de':>6s}  q4 end rec/rep")
    for r in rows:
        if "x_cd_mm" not in r:
            print(f"{r['leg']:18s} {r['t']:6.1f}  {'MISSING IN REPLAY' if r.get('missing_in_replay') else 'no overlap'}")
            continue
        print(f"{r['leg']:18s} {r['t']:6.1f} {r['T_rec']:5.2f}/{r['T_rep']:5.2f} {r['shift_ms']:6.0f}ms {r['x_cd_mm']:5.2f}mm {r['r_ed_mm']:5.2f}mm "
              f"{str(r['q_d_deg']):>28s} {r['b1_d_deg']:5.2f}° {r['b1_de_deg']:5.2f}°  {r['q4_end_deg_rec']:7.2f}/{r['q4_end_deg_rep']:7.2f}")
    print(f"wrote {a.out}.{{json,npz,planner.log,play.log}}")


if __name__ == "__main__":
    main()
