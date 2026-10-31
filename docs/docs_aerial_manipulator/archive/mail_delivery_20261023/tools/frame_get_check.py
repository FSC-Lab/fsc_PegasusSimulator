#!/usr/bin/env python3
"""The LOCALIZATION FRAME's Get on the Mail Envelope tab (2026-10-29), WITHOUT
Isaac: the REAL planner on the HARDWARE mail yaml (pick_place_*_locator set)
and the REAL tab (gs_harness/gs_mail_snapshot, offscreen). obj_0 is the frame's
marker centroid; the tab's own Get buttons are pressed with the frame on the
pick hat and then on the mailbox top, and the planner's points are checked
against the frame + the locator turned by the frame's yaw (the place yaw taken
from the frame, nearest the typed one). The tab is photographed after each Get.

    frame_get_check.py <params yaml> <out dir>      (run through frame_get_check.sh)
"""
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from fsc_autopilot_ros2_msgs.msg import Mocap

NS = "/uav_mailshot"
HERE = os.path.dirname(os.path.abspath(__file__))
FRAME_JSON = os.path.join(HERE, "..", "cad", "localization_frame.json")
FAILS = []


def check(ok, what):
    print(f"  [{'PASS' if ok else 'FAIL'}] {what}")
    if not ok:
        FAILS.append(what)


def through(c, yaw, d):
    co, si = math.cos(yaw), math.sin(yaw)
    return np.array([c[0] + co * d[0] - si * d[1], c[1] + si * d[0] + co * d[1], c[2] + d[2]])


class Rig(Node):
    def __init__(self):
        super().__init__("frame_get_rig", namespace=NS)
        self.obj = self.create_publisher(Mocap, "/obj_0/mocap", 10)
        self.cmd = self.create_publisher(String, "/pndshot/cmd", 10)
        self.acks = []
        self.create_subscription(String, "/pndshot/ack", lambda m: self.acks.append(m.data), 10)
        self.getp = self.create_client(GetParameters, NS + "/whole_body_trajectory_planner/get_parameters")
        self.xyz, self.yaw = np.zeros(3), 0.0
        self.rng = np.random.default_rng(1)
        self.create_timer(0.0083, self.feed)        # the lab's 120 Hz

    def feed(self):
        m = Mocap()
        p = self.xyz + self.rng.normal(0.0, 0.0004, 3)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
        m.pose.orientation.z, m.pose.orientation.w = math.sin(0.5 * self.yaw), math.cos(0.5 * self.yaw)
        self.obj.publish(m)


def spin_for(rig, s):
    t = time.monotonic() + s
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.01)


def gs(rig, cmd):
    n = len(rig.acks)
    m = String()
    m.data = cmd
    rig.cmd.publish(m)
    t = time.monotonic() + 10.0
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.01)
        for a in rig.acks[n:]:
            if a.endswith(cmd) or a.startswith(("ok " + cmd, "err " + cmd)):
                return a
    return "timeout " + cmd


def param(rig, name):
    rq = GetParameters.Request()
    rq.names = [name]
    f = rig.getp.call_async(rq)
    t = time.monotonic() + 5.0
    while not f.done() and time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.01)
    return np.array(f.result().values[0].double_array_value) if f.done() else None


def label_text(out, snap):
    with open(os.path.join(out, snap + ".json")) as f:
        w = json.load(f)["widgets"]
    return [x.get("text", "") for x in w if "locator" in x.get("text", "").lower()]


def main():
    params_yaml, out = sys.argv[1], sys.argv[2]
    with open(params_yaml) as f:
        doc = yaml.safe_load(f)
    pl = next(v["ros__parameters"] for k, v in doc.items() if "whole_body_trajectory_planner" in k)
    d_pick, d_place = pl["pick_place_pick_locator"], pl["pick_place_place_locator"]
    typed_place = pl["pick_place_place_point"]
    fr = json.load(open(FRAME_JSON))
    check(np.allclose(d_pick, fr["locator"]["pick"]) and np.allclose(d_place, fr["locator"]["place"]),
          f"the yaml's locators = the frame design's ({fr['locator']})")
    rclpy.init()
    rig = Rig()
    rig.getp.wait_for_service(timeout_sec=20.0)
    spin_for(rig, 3.0)
    print("[1] the tab's frame-locator line")
    gs(rig, "snap:frame_0_loaded")
    t = label_text(out, "frame_0_loaded")
    check(any("Frame locator ON" in x for x in t), f"the line reads ON: {t}")

    print("[2] the frame on the pick hat, yaw 27 deg -> Get (Envelope row)")
    hat = np.array([1.0, 1.0, 0.77])                 # the pick hat's top centre
    C = np.array(fr["markers"]["centroid"]) / 1e3    # the marker centroid from O, frame axes
    rig.yaw = math.radians(27.0)
    rig.xyz = through(hat, rig.yaw, C)
    spin_for(rig, 1.0)
    a = gs(rig, "click:Get#0")
    spin_for(rig, 1.5)
    p = param(rig, "pick_place_pick_point")
    want = through(rig.xyz, rig.yaw, d_pick)
    check(a.startswith("ok") and p is not None and p.size == 4 and np.linalg.norm(p[:3] - want) < 1.5e-3
          and abs(p[3] - 27.0) < 0.2, f"pick point {None if p is None else np.round(p, 4).tolist()} = frame + "
          f"locator {np.round(want, 4).tolist()} at yaw 27 ({a})")
    rel = 1e3 * through(np.zeros(3), -rig.yaw, p[:3] - hat)
    check(np.linalg.norm(rel - np.array(fr["offsets"]["pick"])) < 1.5,
          f"the envelope centre from the hat centre, frame axes, {np.round(rel, 1).tolist()} mm = the slot's "
          f"{fr['offsets']['pick']}")
    gs(rig, "snap:frame_1_pick_get")

    print(f"[3] the frame on the mailbox top, yaw 4 deg (typed place yaw {typed_place[3]}) -> Get (Mailbox row)")
    rig.xyz, rig.yaw = np.array([1.70, 0.25, 1.15]), math.radians(4.0)
    spin_for(rig, 1.0)
    a = gs(rig, "click:Get#1")
    spin_for(rig, 1.5)
    q = param(rig, "pick_place_place_point")
    want = through(rig.xyz, rig.yaw, d_place)
    yaw_want = typed_place[3] + math.degrees(math.remainder(math.radians(4.0 - typed_place[3]), 2 * math.pi))
    check(a.startswith("ok") and q is not None and q.size == 4 and np.linalg.norm(q[:3] - want) < 1.5e-3
          and abs(q[3] - yaw_want) < 0.2, f"place point {None if q is None else np.round(q, 4).tolist()} = frame + "
          f"locator {np.round(want, 4).tolist()}, yaw {yaw_want:.1f} from the frame ({a})")
    gs(rig, "snap:frame_2_place_get")
    gs(rig, "quit")
    rig.destroy_node()
    rclpy.try_shutdown()
    print("PASS" if not FAILS else f"FAIL: {FAILS}")
    sys.exit(1 if FAILS else 0)


if __name__ == "__main__":
    main()
