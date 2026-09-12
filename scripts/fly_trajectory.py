#!/usr/bin/env python3
"""Headless NMPC trajectory flight: load a reference trajectory, arm, play it, land.

    scripts/fly_trajectory.py full_mission_transition.csv --land
    scripts/fly_trajectory.py /abs/path/traj.csv --controller nmpc_full --scale 1 --rate 50

The scripted twin of the GS "Fly + Land" tab's Play button, for the streamlined
trajectory->SMC->NMPC pipeline (2026-09-08):

    this script --(raw reference)--> smc_outer --(reference_smc)--> nmpc_full --> PX4

  * Bare trajectory names resolve against the GS trajectories/ folder; CSV columns are
    t,x,y,z,yaw_deg[,vx,vy,vz] (the GS TrajectoryData loader is imported, not copied).
  * Delta mode (default): the shape is anchored at the vehicle's CURRENT position/yaw,
    exactly like the GS "Start at current position" checkbox. --absolute disables.
  * Arming is RETRIED until PX4 accepts (EKF convergence after a stack restart rejects
    the first attempts with "Arming denied" -- that is normal, not a failure).
  * The geometric controller is NOT part of the flown path, but it stays the abort
    target: Ctrl-C hands the vehicle to Safe Hover -- staged through the NMPC's
    back-transition (decel_now) if wing-borne, because an instant handover at cruise
    speed departs the aircraft (HIL bag full_tr30, 65.7 m excursion).
  * --land polls for touchdown at the trajectory's end and force-disarms (21196) once
    settled -- a plain disarm is refused while the controller still holds altitude.

Run with the SYSTEM python from a ROS-sourced shell (scripts/rosenv.sh), stack up via
start_vtol_smc_hil.sh with CONTROLLERS="geometric nmpc_full" (or after restart_hil.sh).
"""
import argparse
import importlib.util
import math
import os
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference
from px4_msgs.msg import VehicleCommand, VehicleStatus

GS_ROOT = os.path.expanduser("~/ros2_ws/src/ros2_ground_station_gui")
TRAJ_DIR = os.path.join(GS_ROOT, "trajectories")
GEO_NODE = "/geometric_hover_control"
NMPC_NODE = "/nmpc_full_smc_control"


def _load_trajectory_data_cls():
    """Import the GS's TrajectoryData (the single source of truth for the file format)."""
    path = os.path.join(GS_ROOT, "src", "ROS_Node", "trajectory_player.py")
    spec = importlib.util.spec_from_file_location("gs_trajectory_player", path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["gs_trajectory_player"] = mod
    spec.loader.exec_module(mod)          # needs PyQt5 importable (GUI is never created)
    return mod.TrajectoryData


def rpy_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class TrajFlight(Node):
    def __init__(self, ns, controller, rate):
        super().__init__("fly_trajectory")
        self.ns = ns
        self.controller = controller
        self.rate = rate
        self.pos = None
        self.vel = np.zeros(3)
        self.yaw = 0.0
        self.armed = False
        self.offboard = False
        self.ref_pub = self.create_publisher(
            PositionControllerReference,
            f"/{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.ctrl_pub = self.create_publisher(
            String, f"/{ns}/ground_station/active_controller", 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, f"/{ns}/fmu/in/vehicle_command", 10)
        self.create_subscription(Mocap, f"/{ns}/mocap", self._on_mocap,
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1",
                                 self._on_status, qos_profile_sensor_data)
        self._last_ctrl_pub = 0.0

    # -- callbacks ---------------------------------------------------------
    def _on_mocap(self, m):
        self.pos = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        v = m.twist.linear
        self.vel = np.array([v.x, v.y, v.z])
        self.yaw = rpy_yaw(m.pose.orientation)

    def _on_status(self, m):
        self.armed = (m.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (m.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    # -- primitives --------------------------------------------------------
    def spin_for(self, secs):
        end = time.time() + secs
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def speed(self):
        return float(np.linalg.norm(self.vel))

    def publish_ref(self, pos, yaw_rad, vel):
        m = PositionControllerReference()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "ground"
        m.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        m.velocity = Vector3(x=float(vel[0]), y=float(vel[1]), z=float(vel[2]))
        m.yaw = float(math.degrees(yaw_rad))
        m.yaw_unit = PositionControllerReference.DEGREES
        self.ref_pub.publish(m)

    def select_controller(self, name, force=False):
        """Publish the active-controller id; re-published ~1 Hz by callers (the String is
        volatile QoS -- a restarted node misses a single publish; GS lesson)."""
        now = time.time()
        if force or now - self._last_ctrl_pub >= 1.0:
            self.ctrl_pub.publish(String(data=name))
            self._last_ctrl_pub = now

    def send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = int(command)
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(m)

    def set_node_param(self, node, key, val):
        try:
            r = subprocess.run(["ros2", "param", "set", node, key, str(val)],
                               capture_output=True, timeout=6)
            return r.returncode == 0 and b"successful" in r.stdout.lower()
        except Exception:
            return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("trajectory", help="CSV/NPZ file, or a bare name under the GS trajectories/ dir")
    ap.add_argument("--ns", default="uav_0")
    ap.add_argument("--controller", default="nmpc_full",
                    choices=["nmpc_full", "nmpc6_cruise", "nmpc_cruise"])
    ap.add_argument("--scale", type=float, default=1.0, help="spatial scale")
    ap.add_argument("--tscale", type=float, default=1.0, help="time scale (>1 = faster)")
    ap.add_argument("--rate", type=float, default=50.0, help="reference publish rate Hz")
    ap.add_argument("--lead", type=float, default=10.0,
                    help="pace the reference on tracking error: the trajectory clock FREEZES "
                         "while |err| exceeds this [m] (0 = pure wall clock). Measured need "
                         "(flight1 2026-09-08): a clocked reference ran 260 m ahead of the "
                         "NMPC's own transition schedule; the clamp keeps the carrot reachable.")
    ap.add_argument("--stall-timeout", type=float, default=180.0,
                    help="abort to Safe Hover if the paced clock makes no progress this long [s]")
    ap.add_argument("--absolute", action="store_true",
                    help="play the file's absolute coordinates (default: anchor at current pose)")
    ap.add_argument("--land", action="store_true",
                    help="poll for touchdown at trajectory end and force-disarm once settled")
    ap.add_argument("--no-arm", action="store_true",
                    help="skip arming (vehicle already armed / arming handled elsewhere)")
    ap.add_argument("--arm-timeout", type=float, default=90.0)
    args = ap.parse_args()

    path = args.trajectory
    if not os.path.exists(path):
        cand = os.path.join(TRAJ_DIR, path)
        if os.path.exists(cand):
            path = cand
        else:
            sys.exit(f"trajectory not found: {args.trajectory} (also tried {cand})")
    TrajectoryData = _load_trajectory_data_cls()
    traj = TrajectoryData.load(path)
    print(f"[traj] {traj.name}: {traj.duration:.1f} s, extent {np.round(traj.max_extent(), 1)} m")

    rclpy.init()
    n = TrajFlight(args.ns, args.controller, args.rate)

    # 1. pose ---------------------------------------------------------------
    t0 = time.time()
    while n.pos is None:
        rclpy.spin_once(n, timeout_sec=0.1)
        if time.time() - t0 > 10.0:
            sys.exit("[traj] no mocap pose after 10 s -- is the stack up?")
    p0, yaw0 = traj.start_point()
    if args.absolute:
        offset, yaw_off = np.zeros(3), 0.0
    else:
        offset = n.pos - args.scale * p0
        yaw_off = n.yaw - yaw0
    print(f"[traj] anchored at ({n.pos[0]:.2f},{n.pos[1]:.2f},{n.pos[2]:.2f})"
          f"{' (absolute)' if args.absolute else ''}")

    def ref_at(t_traj):
        p, yw, v = traj.sample(t_traj)
        return offset + args.scale * p, yw + yaw_off, args.scale * args.tscale * v

    # 2. engage the NMPC + pre-stream the start reference -------------------
    n.select_controller(args.controller, force=True)
    end = time.time() + 2.0
    while time.time() < end:
        pos, yw, _ = ref_at(0.0)
        n.publish_ref(pos, yw, np.zeros(3))
        n.select_controller(args.controller)
        n.spin_for(1.0 / args.rate)
    print(f"[traj] controller = {args.controller} (geometric stays as the STOP fallback)")

    # 3. arm + offboard, retried until PX4 accepts ---------------------------
    if not args.no_arm:
        for node in (GEO_NODE, NMPC_NODE):
            n.set_node_param(node, "auto_arm", "true")
        t0, last_try = time.time(), 0.0
        while not (n.armed and n.offboard):
            pos, yw, _ = ref_at(0.0)
            n.publish_ref(pos, yw, np.zeros(3))
            n.select_controller(args.controller)
            if time.time() - last_try > 2.0:
                n.send_cmd(176, 1.0, 6.0)     # DO_SET_MODE -> OFFBOARD
                n.send_cmd(400, 1.0)          # ARM
                last_try = time.time()
                print(f"[arm] armed={n.armed} offboard={n.offboard} (retrying; EKF may still be settling)")
            n.spin_for(1.0 / args.rate)
            if time.time() - t0 > args.arm_timeout:
                sys.exit("[arm] PX4 refused arming past the timeout -- check window 1-px4")
        print("[arm] armed + offboard")

    # 4. play (state-paced: the clock only advances while the vehicle is keeping up) ------
    err = []
    dt = 1.0 / args.rate
    t_traj = 0.0
    frozen_ticks = total_ticks = 0
    t_progress = time.time()
    aborted = False
    last_stat = -1.0
    try:
        while True:
            pos, yw, vel = ref_at(t_traj)
            n.publish_ref(pos, yw, vel)
            n.select_controller(args.controller)
            d = np.linalg.norm(n.pos - pos) if n.pos is not None else float("inf")
            if n.pos is not None:
                err.append(n.pos - pos)
            total_ticks += 1
            if args.lead <= 0.0 or d < args.lead:
                t_traj += dt * args.tscale
                t_progress = time.time()
            else:
                frozen_ticks += 1
                if time.time() - t_progress > args.stall_timeout:
                    aborted = True
                    print(f"\n[abort] no clock progress for {args.stall_timeout:.0f} s "
                          f"(|err|={d:.1f} m) -- handing to Safe Hover", flush=True)
                    break
            if t_traj >= traj.duration:
                break
            if t_traj - last_stat >= 0.5:                                # ~2 Hz status
                last_stat = t_traj
                pace = 1.0 - frozen_ticks / max(total_ticks, 1)
                print(f"\r[fly] t={t_traj:6.1f}/{traj.duration:.1f} s  V={n.speed():5.1f} m/s"
                      f"  |err|={d:5.2f} m  pace={pace:4.2f}   ", end="", flush=True)
            n.spin_for(dt)
    except KeyboardInterrupt:
        aborted = True
        print("\n[abort] Ctrl-C -- handing to Safe Hover", flush=True)
    if total_ticks:
        print(f"\n[fly] pace: clock ran {1.0 - frozen_ticks / total_ticks:.0%} of wall time"
              f" (froze {frozen_ticks} of {total_ticks} ticks at lead {args.lead:.0f} m)")

    # 5. abort path: staged handover to the geometric fallback ----------------
    if aborted:
        if args.controller == "nmpc_full" and n.speed() > 5.0:
            print(f"[abort] wing-borne at {n.speed():.1f} m/s -- back-transition first "
                  "(instant handover departs the aircraft)")
            n.set_node_param(NMPC_NODE, "decel_now", "1.0")
            t0 = time.time()
            while n.speed() > 3.0 and time.time() - t0 < 60.0:
                p = n.pos.copy()
                n.publish_ref(p, n.yaw, np.zeros(3))
                n.select_controller(args.controller)
                n.spin_for(0.1)
            n.set_node_param(NMPC_NODE, "decel_now", "0.0")
        hold = n.pos.copy()
        hold[2] = max(hold[2], 8.0)
        n.select_controller("geometric", force=True)
        print(f"[abort] Safe Hover at ({hold[0]:.1f},{hold[1]:.1f},{hold[2]:.1f}) -- "
              "streaming hold; Ctrl-C again to exit (controller keeps the last target)")
        try:
            while True:
                n.publish_ref(hold, n.yaw, np.zeros(3))
                n.select_controller("geometric")
                n.spin_for(0.1)
        except KeyboardInterrupt:
            pass
    else:
        # 6. trajectory complete: hold the final point; optionally land -------
        pos_f, yaw_f, _ = ref_at(traj.duration)
        print(f"\n[fly] trajectory complete -- holding ({pos_f[0]:.1f},{pos_f[1]:.1f},{pos_f[2]:.1f})")
        if args.land and pos_f[2] < 0.6:
            # Touchdown belongs to the GEOMETRIC controller (the GS LAND button's split):
            # nmpc_full holds its own cruise-altitude band and will hover at ~cruise_alt over
            # the target forever (measured, flight1 2026-09-08: parked at z=21 m over the
            # z=0.1 reference). Hand over for the descent -- by trajectory end the vehicle is
            # slow, so this is the validated hover-envelope handover.
            n.set_node_param(GEO_NODE, "radius_max", "8000.0")
            n.set_node_param(GEO_NODE, "radius_local", "2000.0")
            n.select_controller("geometric", force=True)
            settled = False
            t0 = time.time()
            while time.time() - t0 < 120.0:
                n.publish_ref(pos_f, yaw_f, np.zeros(3))
                n.select_controller("geometric")
                n.spin_for(0.1)
                if n.pos is not None and n.pos[2] < 0.25 and n.speed() < 0.4:
                    settled = True
                    break
            if settled:
                # FORCE the disarm (21196): the controller still holds the z target, so PX4's
                # land detector never sees low throttle -- same rationale as the GS land poll.
                for node in (GEO_NODE, NMPC_NODE):
                    n.set_node_param(node, "auto_arm", "false")
                n.send_cmd(400, 0.0, 21196.0)
                print(f"[land] touchdown at z={n.pos[2]:.2f} m -- disarmed")
            else:
                # NEVER disarm airborne (a forced disarm at altitude drops the vehicle).
                z = n.pos[2] if n.pos is not None else float("nan")
                print(f"[land] NOT settled after 120 s (z={z:.2f} m, V={n.speed():.1f}) -- "
                      "staying armed, holding the final point; Ctrl-C to exit")
                try:
                    while True:
                        n.publish_ref(pos_f, yaw_f, np.zeros(3))
                        n.select_controller(args.controller)
                        n.spin_for(0.1)
                except KeyboardInterrupt:
                    pass
        elif args.land:
            print(f"[land] final reference z={pos_f[2]:.1f} m is not near ground -- not disarming")
        else:
            print("[fly] holding final point; Ctrl-C to exit (controller keeps the last target)")
            try:
                while True:
                    n.publish_ref(pos_f, yaw_f, np.zeros(3))
                    n.select_controller(args.controller)
                    n.spin_for(0.1)
            except KeyboardInterrupt:
                pass

    # 7. summary --------------------------------------------------------------
    if err:
        e = np.array(err)
        rms = np.sqrt((e ** 2).mean(axis=0))
        mx = np.abs(e).max(axis=0)
        print(f"[stats] tracking error  rms x/y/z = {rms[0]:.2f}/{rms[1]:.2f}/{rms[2]:.2f} m"
              f"   max = {mx[0]:.2f}/{mx[1]:.2f}/{mx[2]:.2f} m   ({len(e)} samples)")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
