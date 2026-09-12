#!/usr/bin/env python3
"""
Step-response tool for tuning the VTOL under fsc_autopilot control.

Holds the vehicle's current pose, applies a STEP on one axis of the position
reference (yaw / x / y / z), records the actual response from /uav_0/state/pose,
and reports the standard metrics so gains can be tuned from data instead of by
eye:  rise time (10->90%), overshoot %, settling time (±band), steady-state error.

Run it with the stack up and the vehicle ARMED + OFFBOARD + hovering
(e.g. after `fsc_ref ... ; fsc_offboard ; fsc_arm`). Use system python with ROS
+ both workspaces sourced (same env as the fsc_offboard_helpers pane):

  python3 step_response.py --axis yaw --to 90            # yaw 0->90 deg
  python3 step_response.py --axis z   --to 2.0 --hold 3 --record 12
  python3 step_response.py --axis x   --to 1.0 --csv /tmp/step_x.csv

Metrics + an ASCII plot print at the end; --csv also dumps (t, ref, actual).
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from fsc_autopilot_ros2_msgs.msg import PositionControllerReference


def quat_to_yaw_deg(x, y, z, w):
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


class StepResponse(Node):
    def __init__(self, a):
        super().__init__("vtol_step_response")
        self.a = a
        ns = a.ns
        self.ref_pub = self.create_publisher(
            PositionControllerReference,
            f"/{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.create_subscription(PoseStamped, f"/{ns}/state/pose",
                                 self.on_pose, qos_profile_sensor_data)
        self.pose = None            # latest (x,y,z,yaw)
        self.base = None            # held pose captured at start
        self.frm = None             # value of the stepped axis before the step
        self.t_start = time.time()
        self.t_step = None
        self.samples = []           # (t_since_step, actual, ref)
        self.timer = self.create_timer(1 / 30.0, self.tick)

    def on_pose(self, msg):
        p = msg.pose
        self.pose = {
            "x": p.position.x, "y": p.position.y, "z": p.position.z,
            "yaw": quat_to_yaw_deg(p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w),
        }

    def tick(self):
        if self.pose is None:
            return
        if self.base is None:                      # capture hold pose once
            self.base = dict(self.pose)
            self.frm = self.a.frm if self.a.frm is not None else self.base[self.a.axis]
            self.get_logger().info(
                f"holding {self.base} ; will step {self.a.axis} {self.frm:.2f} -> {self.a.to:.2f}")
        el = time.time() - self.t_start
        stepped = el >= self.a.hold
        if stepped and self.t_step is None:
            self.t_step = time.time()
            self.get_logger().info(f"STEP now (t={el:.1f}s)")

        ref = dict(self.base)
        ref[self.a.axis] = self.a.to if stepped else self.frm
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.position.x, m.position.y, m.position.z = (
            float(ref["x"]), float(ref["y"]), float(ref["z"]))
        m.yaw = float(ref["yaw"]); m.yaw_unit = 0  # DEGREES
        self.ref_pub.publish(m)

        if self.t_step is not None:
            t = time.time() - self.t_step
            self.samples.append((t, self.pose[self.a.axis], self.a.to))
            if t >= self.a.record:
                raise KeyboardInterrupt

    # ---- metrics ----
    def report(self):
        s = [(t, v) for (t, v, _) in self.samples if t >= 0]
        if len(s) < 5:
            print("Not enough samples."); return
        frm, to = self.frm, self.a.to
        step = to - frm
        final = sum(v for _, v in s[-max(3, len(s) // 5):]) / max(3, len(s) // 5)
        ss_err = to - final
        # overshoot (past target, in step direction)
        peak = max(v for _, v in s) if step >= 0 else min(v for _, v in s)
        overshoot = (peak - to) / step * 100 if step else 0.0
        # rise time 10->90%
        def cross(frac):
            tgt = frm + frac * step
            for t, v in s:
                if (step >= 0 and v >= tgt) or (step < 0 and v <= tgt):
                    return t
            return None
        t10, t90 = cross(0.10), cross(0.90)
        rise = (t90 - t10) if (t10 is not None and t90 is not None) else None
        # settling time (last exit of ±band% of |step| around target)
        band = self.a.band / 100.0 * abs(step)
        settle = 0.0
        for t, v in s:
            if abs(v - to) > band:
                settle = t
        unit = "deg" if self.a.axis == "yaw" else "m"
        print("\n================ STEP RESPONSE ================")
        print(f" axis={self.a.axis}  {frm:.3f} -> {to:.3f} {unit}   (step {step:+.3f})")
        print(f" final value      : {final:.3f} {unit}")
        print(f" steady-state err : {ss_err:+.3f} {unit}   ({abs(ss_err)/abs(step)*100 if step else 0:.1f}% of step)")
        print(f" overshoot        : {overshoot:.1f} %")
        print(f" rise time 10-90% : {('%.2f s' % rise) if rise is not None else 'n/a'}")
        print(f" settling (±{self.a.band:.0f}%): {settle:.2f} s")
        self._ascii_plot(s, frm, to)
        if self.a.csv:
            with open(self.a.csv, "w") as f:
                f.write("t,ref,actual\n")
                for (t, v, r) in self.samples:
                    f.write(f"{t:.3f},{r:.4f},{v:.4f}\n")
            print(f" wrote {self.a.csv}")
        print("===============================================")

    @staticmethod
    def _ascii_plot(s, frm, to, h=10, w=60):
        vals = [v for _, v in s]
        lo, hi = min(vals + [frm, to]), max(vals + [frm, to])
        if hi - lo < 1e-6:
            hi += 1.0
        tmax = s[-1][0]
        grid = [[" "] * w for _ in range(h)]
        def row(v): return h - 1 - int((v - lo) / (hi - lo) * (h - 1))
        tr = row(to)
        for c in range(w):
            grid[tr][c] = "-"                       # target line
        for t, v in s:
            c = min(w - 1, int(t / tmax * (w - 1))) if tmax > 0 else 0
            grid[row(v)][c] = "*"
        print(f"\n {hi:7.2f} " + "".join(grid[0]))
        for r in range(1, h - 1):
            print(" " * 9 + "".join(grid[r]))
        print(f" {lo:7.2f} " + "".join(grid[-1]) + f"\n           0{' ' * (w - 6)}{tmax:.1f}s   (--- = target)")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--ns", default="uav_0")
    p.add_argument("--axis", choices=["yaw", "x", "y", "z"], default="yaw")
    p.add_argument("--from", dest="frm", type=float, default=None,
                   help="value before the step (default: current)")
    p.add_argument("--to", type=float, required=True, help="value after the step")
    p.add_argument("--hold", type=float, default=3.0, help="s to hold before stepping")
    p.add_argument("--record", type=float, default=12.0, help="s to record after step")
    p.add_argument("--band", type=float, default=5.0, help="settling band [%% of step]")
    p.add_argument("--csv", default=None)
    a = p.parse_args()

    rclpy.init()
    node = StepResponse(a)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
