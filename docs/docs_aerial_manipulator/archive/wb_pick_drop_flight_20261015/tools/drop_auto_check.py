#!/usr/bin/env python3
"""The AUTONOMOUS DROP's gates (pick_place_drop_auto, 2026-10-16) against the REAL planner, on the
carry_plan_check.py loopback rig: fly the pick-and-drop mission to the drop hold, then hold each gate
failing in turn and require NO release, then make every gate pass and require exactly ONE release
(pick_place/drop_now) no sooner than the dwell.

    drop_auto_check.py <params yaml>      (run through drop_auto_check.sh: private ROS domain 77)

Scenarios at the drop hold, each a few seconds: the first 2 s (the wait gate) / the payload 40 mm off
the drop point / the wrist roll q4 5 deg off / the payload's mocap FROZEN / FLIPPED (a 180 deg solve) /
SWINGING +-40 mm at 0.5 Hz / the claw MOVING (+-8 mm at 1 Hz, still inside its 20 mm) / the claw 30 mm
off / everything good -> one release.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from std_msgs.msg import String
from fsc_autopilot_ros2_msgs.msg import Mocap

import carry_plan_check as C


class DropRig(C.Rig):
    def __init__(self, place_xy, drop_z):
        super().__init__()
        self.place = np.array([place_xy[0], place_xy[1], drop_z])
        self.obj_mode = "pick"            # pick | centre | off40 | frozen | flipped | swing
        self.claw_wiggle = False
        self.base_hold = None
        self.drops = []
        self.status_log = []
        self.create_subscription(String, "whole_body_planner/pick_place/drop_now",
                                 lambda m: self.drops.append((time.monotonic(), m.data)), 10)

    def feed(self):
        if self.claw_wiggle and self.base_hold is not None:
            self.xyz = self.base_hold + np.array([0.008 * math.sin(2 * math.pi * time.monotonic()), 0.0, 0.0])
        super().feed()

    def mocap_pose(self):
        """Override of the payload position: obj_mode decides where obj_0 is."""
        if self.obj_mode == "pick":
            return C.OBJ + self.rng.normal(0.0, 0.0005, 3), False
        if self.obj_mode == "frozen":
            return self.place.copy(), False
        p = self.place + self.rng.normal(0.0, 0.0005, 3)
        if self.obj_mode == "off40":
            p = p + np.array([0.0, 0.040, 0.0])
        elif self.obj_mode == "swing":
            p = p + np.array([0.0, 0.040 * math.sin(math.pi * time.monotonic()), 0.0])
        return p, self.obj_mode == "flipped"


def patch_feed(rig):
    """carry_plan_check.Rig.feed publishes obj_0 at C.OBJ; replace that part with mocap_pose()."""
    pub = rig.obj

    class Shim:
        def publish(self, m):
            p, flipped = rig.mocap_pose()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
            if flipped:
                m.pose.orientation.w, m.pose.orientation.x = 0.0, 1.0   # 180 deg about x: tilt 180 deg
            pub.publish(m)
    rig.obj = Shim()


def status_during(rig, seconds):
    seen = []
    t = time.monotonic() + seconds
    while time.monotonic() < t:
        rclpy.spin_once(rig, timeout_sec=0.02)
        s = rig.pp_status or ""
        if not seen or seen[-1] != s:
            seen.append(s)
    return seen


def main():
    params = yaml.safe_load(open(sys.argv[1]))["/**/whole_body_trajectory_planner"]["ros__parameters"]
    place = params["pick_place_place_point"]
    drop_z = place[2] + params.get("pick_place_drop_height", 0.0)
    rclpy.init()
    rig = DropRig(place[:2], drop_z)
    patch_feed(rig)
    m = String(); m.data = "DIRECT"
    rig.mode.publish(m)
    C.spin_for(rig, 3.0)
    C.call(rig, "capture_pick")
    C.call(rig, "plan")
    C.wait_status(rig, ["READY"], 20)
    # the mission to the drop hold (as carry_plan_check.py)
    for k, leg in enumerate(C.LEGS[:4]):
        if not C.call(rig, leg):
            sys.exit(1)
        if leg == "execute_pick":
            C.wait_status(rig, ["WAITING execute_pick"], 60)
            rig.teleport(1)
            C.spin_for(rig, 0.6)
            rig.xyz = rig.xyz + rig.arr[3:6]
            C.spin_for(rig, 6.5)
            C.call(rig, "descend_pick")
            C.wait_status(rig, ["DONE execute_pick"], 30)
            rig.teleport(1)
            C.call(rig, "exit_pick")
            C.wait_status(rig, ["DONE exit_pick"], 30)
            rig.xyz = rig.xyz + np.array([0.0, 0.0, 0.20])
            C.spin_for(rig, 0.6)
        elif leg == "execute_place":
            rig.obj_mode = "off40"                    # the payload hangs 40 mm off from the start
            C.wait_status(rig, ["DONE execute_place"], 60)
            rig.teleport(3)
        else:
            C.wait_status(rig, [f"DONE {leg}"], 60)
            rig.teleport(k)
    rig.base_hold = rig.xyz.copy()
    results = []

    def check(name, seconds, must_contain, setup):
        setup()
        seen = status_during(rig, seconds)
        tail = [s.split(" -- auto drop: ")[-1] for s in seen if "auto drop:" in s]
        ok = (len(rig.drops) == 0 and any(must_contain in t for t in tail))
        results.append((name, ok, tail[-1] if tail else "(no auto-drop text)"))

    check("wait gate (first 1.6 s)", 1.6, "waiting", lambda: None)
    check("payload 40 mm off", 2.5, "mm from the drop point", lambda: None)
    check("wrist q4 5 deg off", 2.5, "wrist q4 5.0",
          lambda: (setattr(rig, "obj_mode", "centre"), rig.q.__setitem__(3, math.radians(5.0))))
    check("payload frozen", 2.5, "frozen",
          lambda: (rig.q.__setitem__(3, 0.0), setattr(rig, "obj_mode", "frozen")))
    check("payload flipped solve", 2.5, "not tracked", lambda: setattr(rig, "obj_mode", "flipped"))
    check("payload swinging +-40 mm", 4.0, "payload", lambda: setattr(rig, "obj_mode", "swing"))
    check("claw moving +-8 mm at 1 Hz", 3.0, "claw moving",
          lambda: (setattr(rig, "obj_mode", "centre"), setattr(rig, "claw_wiggle", True)))
    check("claw 30 mm off", 2.5, "claw 30",
          lambda: (setattr(rig, "claw_wiggle", False),
                   setattr(rig, "xyz", rig.base_hold + np.array([0.030, 0.0, 0.0]))))
    # everything good: one release, after the dwell
    rig.xyz = rig.base_hold.copy()
    t_good = time.monotonic()
    seen = status_during(rig, 3.0)
    n = len(rig.drops)
    dt = rig.drops[0][0] - t_good if n else float("nan")
    released = [s for s in seen if "RELEASE signalled" in s]
    # (2026-10-30) the claw's speed is measured over the last 0.5 s (fit + spread), so a
    # claw that has just moved 30 mm must first be still for that window, then the 0.5 s
    # dwell: 1.0 s plus a few 10 Hz ticks
    ok = n == 1 and 0.45 <= dt <= 1.7 and bool(released)
    results.append(("all gates good -> ONE release after the dwell", ok,
                    f"{n} drop_now, {dt:.2f} s after the gates passed: {rig.drops[0][1] if n else ''}"))
    print("\n  AUTONOMOUS DROP CHECK")
    for name, ok_, txt in results:
        print(f"  {'PASS' if ok_ else 'FAIL'}  {name:40s} {txt[:150]}")
    rig.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(r[1] for r in results) else 1)


if __name__ == "__main__":
    main()
