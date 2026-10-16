#!/usr/bin/env python3
"""AIM THE DROP AT THE BASKET (pick_place_drop_aim_payload, 2026-10-16) against the REAL planner, on the
carry_plan_check.py loopback rig.

    drop_aim_check.py <params yaml>      (run through drop_aim_check.sh: private ROS domain 77)

After the grasp the basket HANGS from the claw the way it did on 10-15 D-1: [+3, -17, -160] mm in the
vehicle frame, where the ee_offset model puts it at [+20, 0, -160]. The rig hovers at Drop Start long
for the settle plus the aim window, then:
  - Ready To Drop must say "aimed at the basket" and move the drop rest by exactly the measured
    difference, Rz(place yaw) * (-(ee_offset + hang))_xy -- read back from pick_place/info;
  - at the drop hold, with the claw on its (shifted) target, the hanging basket must sit on the drop point;
  - the claw 30 mm off (the basket hanging with it): the status must name the PAYLOAD gate, not the claw
    gate -- aimed, the claw has no position gate of its own;
  - the claw moving +-8 mm at 1 Hz: the claw's stillness gate still holds it;
  - back on target: exactly ONE release, no sooner than the dwell, with the payload ~0 mm off.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import carry_plan_check as C

HANG = np.array([0.003, -0.017, -0.160])          # basket - claw, vehicle frame (x nose, y left), 10-15 D-1


def rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class HangRig(C.Rig):
    def __init__(self):
        super().__init__()
        self.obj_mode = "pick"            # pick: on the pick hat | hang: from the claw
        self.claw_body = None             # claw - base, vehicle frame, at the carry/drop arm pose
        self.claw_wiggle = False
        self.base_hold = None
        self.drops = []
        self.create_subscription(String, "whole_body_planner/pick_place/drop_now",
                                 lambda m: self.drops.append((time.monotonic(), m.data)), 10)

    def claw(self):
        return self.xyz + rz(self.yaw) @ self.claw_body

    def feed(self):
        if self.claw_wiggle and self.base_hold is not None:
            self.xyz = self.base_hold + np.array([0.008 * math.sin(2 * math.pi * time.monotonic()), 0.0, 0.0])
        super().feed()


def patch_feed(rig):
    """carry_plan_check.Rig.feed publishes obj_0 at C.OBJ; in hang mode it hangs from the claw instead."""
    pub = rig.obj

    class Shim:
        def publish(self, m):
            if rig.obj_mode == "hang":
                p = rig.claw() + rz(rig.yaw) @ HANG + rig.rng.normal(0.0, 0.0005, 3)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, p)
            pub.publish(m)
    rig.obj = Shim()


def call_full(rig, name):
    c = rig.cli[name]
    if not c.wait_for_service(timeout_sec=10.0):
        sys.exit(f"service {name} not available")
    f = c.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(rig, f, timeout_sec=10.0)
    r = f.result()
    print(f"  {name}: {'OK' if r.success else 'REFUSED'} -- {r.message}")
    return r


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
    assert params.get("pick_place_place_drop") and params.get("pick_place_drop_aim_payload"), \
        "the yaml must have the drop place and the basket aim on"
    place = np.array(params["pick_place_place_point"][:3], dtype=float)
    place_yaw = math.radians(params["pick_place_place_point"][3])
    ee_off = np.array(params["pick_place_ee_offset"], dtype=float)
    window = params.get("pick_place_drop_aim_window_s", 4.0)
    settle = params.get("pick_place_drop_aim_settle_s", 2.0)
    rclpy.init()
    rig = HangRig()
    patch_feed(rig)
    m = String(); m.data = "DIRECT"
    rig.mode.publish(m)
    C.spin_for(rig, 3.0)
    C.call(rig, "capture_pick")
    C.call(rig, "plan")
    C.wait_status(rig, ["READY"], 20)
    C.spin_for(rig, 0.5)
    xb3, yaw3, ee3 = rig.info[14 + 21:17 + 21], math.radians(rig.info[17 + 21]), rig.info[18 + 21:21 + 21]
    rig.claw_body = rz(yaw3).T @ (np.asarray(ee3) - np.asarray(xb3))
    goal3_before = np.array(rig.info[35:42])
    results = []
    # the mission to Drop Start (as drop_auto_check.py); the basket hangs from the claw after the grasp
    for k, leg in enumerate(C.LEGS[:3]):
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
        else:
            C.wait_status(rig, [f"DONE {leg}"], 60)
            rig.teleport(k)
        if leg == "execute_pick":
            rig.obj_mode = "hang"
    # hover at Drop Start for the aim window: the planner measures the basket - claw offset
    q_ok = np.abs(rig.q[:3] - np.radians(params["pick_place_place_pose_deg"][:3])).max() < math.radians(1.0)
    results.append(("arm at the drop pose while hovering at Drop Start", q_ok,
                    f"q = {np.round(np.degrees(rig.q), 2)} deg"))
    C.spin_for(rig, settle + window + 1.0)     # the turn's swing is skipped, then a full window
    r = call_full(rig, "execute_place")
    C.spin_for(rig, 0.3)
    shift = np.array(rig.info[35:38]) - goal3_before[0:3]
    shift_ee = np.array(rig.info[39:42]) - goal3_before[4:7]
    want = np.zeros(3)
    want[:2] = (-(rz(place_yaw) @ (ee_off + HANG)))[:2]
    ok = (r.success and "aimed at the basket" in r.message and "NOT aimed" not in r.message
          and np.abs(shift - want).max() < 0.001 and np.abs(shift_ee - want).max() < 0.001)
    results.append(("Ready To Drop aims at the basket", ok,
                    f"rest moved {np.round(shift * 1e3, 1)} mm, claw target {np.round(shift_ee * 1e3, 1)} mm, "
                    f"expected {np.round(want * 1e3, 1)} mm"))
    C.wait_status(rig, ["DONE execute_place"], 60)
    rig.teleport(3)
    rig.base_hold = rig.xyz.copy()
    basket = rig.claw() + rz(rig.yaw) @ HANG
    err = np.linalg.norm(basket[:2] - place[:2])
    results.append(("hanging basket on the drop point at the drop hold", err < 0.002,
                    f"basket {err * 1e3:.1f} mm from the drop point, claw {np.linalg.norm(rig.arr[3:6]) * 1e3:.1f} mm "
                    f"from its target (the model target: {np.linalg.norm(rig.claw()[:2] - (place + rz(place_yaw) @ ee_off)[:2]) * 1e3:.1f} mm)"))

    def check(name, seconds, must, must_not, setup):
        setup()
        seen = status_during(rig, seconds)
        tail = [s.split(" -- auto drop: ")[-1] for s in seen if "auto drop:" in s]
        ok_ = (len(rig.drops) == 0 and any(must in t for t in tail)
               and not any(must_not in t for t in tail if must_not))
        results.append((name, ok_, tail[-1] if tail else "(no auto-drop text)"))

    check("wait gate (first 1.6 s)", 1.6, "waiting", "", lambda: None)
    check("claw 30 mm off, basket with it -> the PAYLOAD gate", 2.5, "mm from the drop point",
          "from the drop target",
          lambda: setattr(rig, "xyz", rig.base_hold + np.array([0.030, 0.0, 0.0])))
    check("claw moving +-8 mm at 1 Hz -> its stillness gate", 3.0, "claw moving", "",
          lambda: setattr(rig, "claw_wiggle", True))
    rig.claw_wiggle = False
    rig.xyz = rig.base_hold.copy()
    t_good = time.monotonic()
    # a LIVE set of a drop gate while holding must leave the plan valid (review
    # finding 2026-10-16: those keys were missing from the "only gates" list)
    cli = rig.create_client(SetParameters, C.NS + "/whole_body_trajectory_planner/set_parameters")
    cli.wait_for_service(timeout_sec=5.0)
    req = SetParameters.Request()
    for name in ("pick_place_drop_auto_payload_tol", "pick_place_drop_aim_window_s"):
        v = float(params.get(name, 0.025 if "tol" in name else 4.0))
        req.parameters.append(Parameter(name=name, value=ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=v)))
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(rig, f, timeout_sec=5.0)
    set_ok = f.result() is not None and all(r_.successful for r_ in f.result().results)
    seen = status_during(rig, 1.0)
    stale = [x for x in seen if x.startswith("NOT PLANNED") or "stale" in x]
    results.append(("live set of a drop gate leaves the plan valid", set_ok and not stale,
                    f"set {'accepted' if set_ok else 'REFUSED'}; status {seen[-1][:60] if seen else '?'}"))
    seen += status_during(rig, 2.0)
    n = len(rig.drops)
    dt = rig.drops[0][0] - t_good if n else float("nan")
    ok = n == 1 and 0.45 <= dt <= 1.2 and any("RELEASE signalled" in s for s in seen)
    results.append(("back on target -> ONE release after the dwell", ok,
                    f"{n} drop_now, {dt:.2f} s after: {rig.drops[0][1] if n else ''}"))
    print("\n  DROP AIM CHECK")
    for name, ok_, txt in results:
        print(f"  {'PASS' if ok_ else 'FAIL'}  {name:52s} {txt[:170]}")
    rig.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all(r_[1] for r_ in results) else 1)


if __name__ == "__main__":
    main()
