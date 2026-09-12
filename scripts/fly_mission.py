#!/usr/bin/env python3
"""Drive a completion-gated waypoint mission on the SAFE hover controller, with telemetry.

    scripts/fly_mission.py                          # the default demo mission
    scripts/fly_mission.py --wp 0,0,8 --wp 40,0,8 --wp 40,0,0.4
    scripts/fly_mission.py --hold 20                # single hover-hold soak

Each leg streams its waypoint until the vehicle is inside `--tol` and slower than
`--vtol`, then advances -- the same rule the ground station's Fly-to-point uses, so what
this proves on the command line is what the GS button does. Prints a live table and a
per-leg summary (arrival error, worst altitude deviation, peak speed, attitude extremes).

Run with the SYSTEM python from a ROS-sourced shell (scripts/rosenv.sh).
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference


def rpy(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class Mission(Node):
    def __init__(self, ns, wps, tol, vtol, leg_timeout, quiet):
        super().__init__("fly_mission")
        self.pub = self.create_publisher(
            PositionControllerReference,
            f"/{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.create_subscription(Mocap, f"/{ns}/mocap", self.on_mocap, qos_profile_sensor_data)
        self.p = self.v = None
        self.att = (0.0, 0.0, 0.0)
        self.wps = wps
        self.tol, self.vtol, self.leg_timeout, self.quiet = tol, vtol, leg_timeout, quiet
        self.i = 0
        self.t_leg = time.time()
        self.samples = []
        self.results = []
        self.create_timer(0.1, self.step)
        self.create_timer(1.0, self.report)
        self.done = False

    def on_mocap(self, m):
        self.p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
        self.att = rpy(m.pose.orientation)

    def send(self, wp):
        r = PositionControllerReference()
        r.position.x, r.position.y, r.position.z = float(wp[0]), float(wp[1]), float(wp[2])
        r.yaw = float(wp[3])
        try:
            r.yaw_unit = PositionControllerReference.DEGREES
        except Exception:
            pass
        self.pub.publish(r)

    def step(self):
        if self.done or self.p is None:
            return
        wp = self.wps[self.i]
        self.send(wp)
        d = float(np.linalg.norm(self.p - np.array(wp[:3])))
        sp = float(np.linalg.norm(self.v))
        self.samples.append((self.p.copy(), sp, self.att, d))
        el = time.time() - self.t_leg
        if (d < self.tol and sp < self.vtol) or el > self.leg_timeout:
            arr = np.array([s[0] for s in self.samples])
            spd = np.array([s[1] for s in self.samples])
            at = np.array([s[2] for s in self.samples])
            self.results.append(dict(
                wp=wp, t=el, err=d, timeout=el > self.leg_timeout,
                dz=float(np.max(np.abs(arr[:, 2] - wp[2]))) if len(arr) else 0.0,
                vmax=float(np.max(spd)) if len(spd) else 0.0,
                roll=float(np.max(np.abs(at[:, 0]))) if len(at) else 0.0,
                pitch=float(np.max(np.abs(at[:, 1]))) if len(at) else 0.0))
            r = self.results[-1]
            print(f"  LEG {self.i+1} -> ({wp[0]:.1f},{wp[1]:.1f},{wp[2]:.1f}) yaw {wp[3]:.0f}: "
                  f"{'TIMEOUT ' if r['timeout'] else 'arrived '}in {el:5.1f}s  "
                  f"err {d:.3f} m  peak {r['vmax']:.2f} m/s  "
                  f"|roll|max {r['roll']:.1f} |pitch|max {r['pitch']:.1f} deg", flush=True)
            self.samples = []
            self.i += 1
            self.t_leg = time.time()
            if self.i >= len(self.wps):
                self.done = True

    def report(self):
        if self.p is None or self.done or self.quiet:
            return
        wp = self.wps[self.i]
        d = float(np.linalg.norm(self.p - np.array(wp[:3])))
        print(f"    t={time.time()-self.t_leg:5.1f} p=({self.p[0]:7.2f},{self.p[1]:6.2f},"
              f"{self.p[2]:6.2f}) |v|={np.linalg.norm(self.v):4.2f} "
              f"rpy=({self.att[0]:+5.1f},{self.att[1]:+5.1f},{self.att[2]:+6.1f}) "
              f"-> wp{self.i+1} {d:6.2f} m", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="uav_0")
    ap.add_argument("--wp", action="append", default=[],
                    help="x,y,z[,yaw_deg] -- repeatable, flown in order")
    ap.add_argument("--tol", type=float, default=0.30)
    ap.add_argument("--vtol", type=float, default=0.35)
    ap.add_argument("--leg-timeout", type=float, default=90.0)
    ap.add_argument("--hold", type=float, default=0.0, help="hover-hold soak [s] at the first wp")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()

    if a.wp:
        wps = []
        for w in a.wp:
            f = [float(x) for x in w.split(",")]
            wps.append((f + [0.0])[:4] if len(f) < 4 else f[:4])
    else:                                   # default demo mission
        wps = [(0, 0, 8, 0), (20, 0, 8, 0), (20, 10, 8, 90),
               (20, 10, 3, 90), (20, 10, 0.35, 90)]
    print(f"mission: {len(wps)} legs, tol {a.tol} m / {a.vtol} m/s")
    rclpy.init()
    n = Mission(a.ns, wps, a.tol, a.vtol, a.leg_timeout, a.quiet)
    t0 = time.time()
    try:
        while rclpy.ok() and not n.done and time.time() - t0 < 600:
            rclpy.spin_once(n, timeout_sec=0.05)
        if a.hold > 0:
            print(f"  HOLD {a.hold:.0f}s at the last waypoint")
            th = time.time(); pos = []
            while rclpy.ok() and time.time() - th < a.hold:
                rclpy.spin_once(n, timeout_sec=0.05)
                n.send(wps[-1])
                if n.p is not None:
                    pos.append(n.p.copy())
            pos = np.array(pos[len(pos) // 4:])
            e = np.linalg.norm(pos - np.array(wps[-1][:3]), axis=1)
            print(f"  HOLD: mean err {e.mean():.3f} m, max {e.max():.3f} m, "
                  f"drift {np.ptp(pos, axis=0)}")
    except KeyboardInterrupt:
        pass
    ok = all(not r["timeout"] for r in n.results) and len(n.results) == len(wps)
    print(f"\nMISSION {'COMPLETE' if ok else 'INCOMPLETE'} in {time.time()-t0:.0f}s")
    for r in n.results:
        print(f"   ({r['wp'][0]:6.1f},{r['wp'][1]:6.1f},{r['wp'][2]:5.1f}) "
              f"err {r['err']:.3f} m  alt dev {r['dz']:.2f} m  vmax {r['vmax']:.2f} m/s")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
