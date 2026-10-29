#!/usr/bin/env python3
"""Fly the C++ FSC autopilot (scripts/start_vtol_fsc_hil.sh) through a scripted multicopter mission and score it.

    source scripts/rosenv.sh; python3 scripts/fly_fsc_cpp.py --cfg ~/flight_bags/fsc_cpp/config_airframe_plan_A
    python3 scripts/fly_fsc_cpp.py --cfg DIR --alt 5 --wp 10,0 --wp 0,0 --hold 15

REFUSES to arm unless (1) DIR/px4_params.verified exists and is newer than DIR/px4_params.txt (the launcher's
8-params window read every derived PX4 parameter back from the console), (2) Isaac's log for this launch
carries the PX4 tilt map line (`[VTOL] tilt map:`; without it PX4's upright tilt is the nacelles' aft stop)
and, when Isaac printed one, the same airframe digest the config was derived from, (3) the autopilot node is
up, (4) the vehicle sits on the ground, upright and still (a crashed vehicle is resurrected by Isaac --
CLAUDE.md "sim resurrects the vehicle").
Then: stream a RATE-LIMITED reference (the autopilot's own position loop is not a trajectory generator),
OFFBOARD, ARM, climb, the legs (each held `--hold` s once inside --tol and slower than --vtol), descend on the
start point, disarm. Aborts on |roll| or |pitch| > --att_abort or if PX4 leaves OFFBOARD (a failsafe).
An abort that leaves the aircraft airborne, armed and in OFFBOARD with a live mocap LANDS IT IN PLACE (the
driver is the only reference streamer: exiting would leave it hovering on the autopilot's last reference --
2026-10-28, the CAD airframe's first flight). `--land_here` does only that, for an aircraft already up.
Prints a 1 Hz table and a summary: per hold the mean / sd / worst position error, the attitude and yaw
extremes, the climb overshoot, the landing error. Exit 0 = landed within 0.5 m, 1 = flew but did not,
2 = refused, 3 = aborted.
"""
import argparse
import glob
import math
import os
import re
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference
from px4_msgs.msg import VehicleCommand, VehicleStatus


def rpy(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def preflight(cfg, isaac_log):
    bad = []
    ver = os.path.join(cfg, "px4_params.verified"); txt = os.path.join(cfg, "px4_params.txt")
    if not os.path.exists(ver) or os.path.getmtime(ver) < os.path.getmtime(txt):
        bad.append(f"no {ver} newer than {txt}: the PX4 read-back did not pass (see {cfg}/px4_console.txt)")
    head = open(txt).readline()
    m = re.search(r"af=([0-9a-f]+)", head); af = m.group(1) if m else None
    if isaac_log is None:
        logs = sorted(glob.glob(os.path.join(os.path.dirname(cfg.rstrip("/")), "isaac_*.log")), key=os.path.getmtime)
        isaac_log = logs[-1] if logs else None
    if isaac_log is None:
        bad.append("no Isaac log (isaac_*.log beside the config dir)")
    else:
        s = open(isaac_log, errors="replace").read()
        if "[VTOL] tilt map:" not in s:
            bad.append(f"{isaac_log}: no '[VTOL] tilt map:' line -- Isaac did not get VTOL_TILT_PX4CA")
        afs = set(re.findall(r"af=([0-9a-f]{10})", s))
        if afs and af not in afs:
            bad.append(f"{isaac_log}: Isaac built af={sorted(afs)}, the config was derived from af={af}")
    return bad, af, isaac_log


class Flight(Node):
    def __init__(self, a):
        super().__init__("fly_fsc_cpp")
        ns = a.ns
        self.a = a
        self.ref_pub = self.create_publisher(PositionControllerReference,
                                             f"/{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        qos_px4 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
        self.cmd_pub = self.create_publisher(VehicleCommand, f"/{ns}/fmu/in/vehicle_command", 10)
        self.create_subscription(Mocap, f"/{ns}/mocap", self.on_mocap, qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, f"/{ns}/fmu/out/vehicle_status_v1", self.on_status, qos_px4)
        self.p = None; self.v = None; self.att = None; self.t_state = 0.0
        self.armed = False; self.nav = None
        self.ref = None                      # the streamed (rate-limited) reference
        self.goal = None
        self.create_timer(1.0 / 30.0, self.tick)

    def on_mocap(self, m):
        self.p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
        self.att = rpy(m.pose.orientation)
        self.t_state = time.monotonic()

    def on_status(self, s):
        self.armed = s.arming_state == 2
        self.nav = int(s.nav_state)

    def tick(self):
        if self.ref is None or self.goal is None:
            return
        dt = 1.0 / 30.0
        d = self.goal - self.ref
        dh = d[:2]; nh = float(np.linalg.norm(dh))
        if nh > 1e-9:
            self.ref[:2] += dh * min(1.0, self.a.vh * dt / nh)
        vz = self.a.vz_up if d[2] > 0 else self.a.vz_down
        self.ref[2] += math.copysign(min(abs(d[2]), vz * dt), d[2])
        r = PositionControllerReference()
        r.header.stamp = self.get_clock().now().to_msg()
        r.position.x, r.position.y, r.position.z = (float(v) for v in self.ref)
        r.yaw = 0.0; r.yaw_unit = 0
        self.ref_pub.publish(r)

    def vcmd(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        c.command = int(command); c.param1 = float(p1); c.param2 = float(p2)
        c.target_system = 1; c.target_component = 1; c.source_system = 1; c.source_component = 1
        c.from_external = True
        self.cmd_pub.publish(c)


def land_in_place(n, spin, z_ground, timeout):
    """Descend on the aircraft's current x, y and disarm once it is down and still (the descend leg's rule:
    2 s down -> disarm, 6 s -> forced). True when PX4 reports disarmed."""
    n.goal = np.array([n.p[0], n.p[1], z_ground - 0.3])
    t0 = time.monotonic(); t_in = None; last = 0.0
    while time.monotonic() - t0 < timeout:
        spin(0.02)
        now = time.monotonic()
        if not n.armed:
            return True
        if now - n.t_state > 0.5:
            return False
        if now - last > 1.0:
            last = now
            print(f"  land       p ({n.p[0]:+6.2f},{n.p[1]:+6.2f},{n.p[2]:5.2f}) |v| {np.linalg.norm(n.v):4.2f}", flush=True)
        if n.p[2] - z_ground < 0.15 and float(np.linalg.norm(n.v)) < 0.2:
            if t_in is None:
                t_in = now
            if now - t_in > 2.0:
                n.vcmd(400, 0.0)
                if now - t_in > 6.0:
                    n.vcmd(400, 0.0, 21196.0)
        else:
            t_in = None
    return not n.armed


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--cfg", help="the launcher's FSC_CFG_DIR (required unless --land_here)")
    ap.add_argument("--land_here", action="store_true",
                    help="land an aircraft already armed in OFFBOARD at its current x, y; nothing else")
    ap.add_argument("--ground_z", type=float, default=0.15, help="--land_here: the resting height [m]")
    ap.add_argument("--isaac_log", default=None)
    ap.add_argument("--ns", default="uav_0")
    ap.add_argument("--alt", type=float, default=5.0)
    ap.add_argument("--wp", action="append", default=None, help="x,y relative to the start, at --alt (repeatable)")
    ap.add_argument("--hold", type=float, default=15.0)
    ap.add_argument("--tol", type=float, default=0.3); ap.add_argument("--vtol", type=float, default=0.3)
    ap.add_argument("--vh", type=float, default=1.5); ap.add_argument("--vz_up", type=float, default=0.8)
    ap.add_argument("--vz_down", type=float, default=0.5)
    ap.add_argument("--leg_timeout", type=float, default=60.0)
    ap.add_argument("--att_abort", type=float, default=45.0)
    ap.add_argument("--skip_preflight", action="store_true")
    a = ap.parse_args()
    wps = [tuple(float(v) for v in w.split(",")) for w in (a.wp or ["10,0", "0,0"])]
    if a.cfg is None and not a.land_here:
        ap.error("--cfg is required")

    if not (a.skip_preflight or a.land_here):
        bad, af, ilog = preflight(a.cfg, a.isaac_log)
        print(f"[fly_fsc_cpp] config {a.cfg} (af={af}), Isaac log {ilog}")
        for b in bad:
            print(f"[fly_fsc_cpp] REFUSED: {b}")
        if bad:
            return 2
    rclpy.init()
    n = Flight(a)
    spin = lambda s: rclpy.spin_once(n, timeout_sec=s)   # noqa: E731
    t_end = time.monotonic() + 30.0
    while time.monotonic() < t_end and (n.p is None or n.nav is None):
        spin(0.1)
    if n.p is None or n.nav is None:
        print(f"[fly_fsc_cpp] REFUSED: no mocap ({n.p is not None}) or no vehicle_status ({n.nav is not None})")
        return 2
    t_end = time.monotonic() + 10.0                     # DDS discovery of a node a second old is not instant
    names = n.get_node_names()
    while "autopilot_sv_baseline_node" not in names and time.monotonic() < t_end:
        spin(0.2); names = n.get_node_names()
    if "autopilot_sv_baseline_node" not in names:
        print(f"[fly_fsc_cpp] REFUSED: autopilot_sv_baseline_node is not running ({sorted(names)[:12]})")
        return 2
    if a.land_here:
        if not (n.armed and n.nav == 14):
            print(f"[fly_fsc_cpp] REFUSED: --land_here needs an aircraft armed in OFFBOARD (armed {n.armed}, nav {n.nav})")
            return 2
        print(f"[fly_fsc_cpp] landing in place at ({n.p[0]:+.2f}, {n.p[1]:+.2f}) from z {n.p[2]:.2f}", flush=True)
        n.ref = n.p.copy()
        ok = land_in_place(n, spin, a.ground_z, 120.0)
        print(f"[fly_fsc_cpp] {'LANDED and disarmed' if ok else 'NOT disarmed'} at ({n.p[0]:+.2f}, {n.p[1]:+.2f}, {n.p[2]:.2f})")
        rclpy.shutdown()
        return 0 if ok else 1
    roll, pitch, _ = n.att
    if not (n.p[2] < 0.6 and abs(roll) < 10 and abs(pitch) < 10 and float(np.linalg.norm(n.v)) < 0.3):
        print(f"[fly_fsc_cpp] REFUSED: vehicle not grounded/upright/still: p {np.round(n.p, 2)} att {np.round(n.att, 1)}"
              f" |v| {np.linalg.norm(n.v):.2f}")
        return 2
    home = n.p.copy()
    print(f"[fly_fsc_cpp] home {np.round(home, 2)}; streaming the reference, then OFFBOARD + ARM")
    n_armed0 = 0 if a.skip_preflight else sum(1 for ln in open(ilog, errors="replace") if "[VTOL_HIL" in ln and "armed=True" in ln)
    n.ref = home.copy(); n.ref[2] = home[2]; n.goal = home.copy()
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:
        spin(0.05)
    for attempt in range(15):
        if n.nav != 14:
            n.vcmd(176, 1.0, 6.0)
        elif not n.armed:
            n.vcmd(400, 1.0)
        else:
            break
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.0:
            spin(0.05)
    if not (n.armed and n.nav == 14):
        print(f"[fly_fsc_cpp] REFUSED by PX4: armed={n.armed} nav_state={n.nav} (14 = OFFBOARD)")
        return 2
    print("[fly_fsc_cpp] armed in OFFBOARD")
    # ★ THE NACELLES, ON THE GROUND, BEFORE THE CLIMB (2026-10-28: the first flight of this path took off on
    # the aileron's channel -- the nacelles at 45 deg, PX4's tilt on the elevator -- and slid 200 m along the
    # ground). Isaac's HIL debug line (PEGASUS_HIL_DEBUG=1) carries the mapped channel dict: both tilts must
    # sit within 5 deg of upright once the spool-up is over.
    if not a.skip_preflight:
        # the log arrives through a pipe: wait for NEW armed lines past the spool-up window (<= 10 s), do not
        # read it once (2026-10-28: a 3 s single read found none and refused a healthy stack)
        t1 = time.monotonic(); lines = []
        while time.monotonic() - t1 < 10.0:
            spin(0.1)
            s = open(ilog, errors="replace").read()
            armed_lines = [ln for ln in s.splitlines() if "[VTOL_HIL" in ln and "armed=True" in ln]
            if len(armed_lines) - n_armed0 >= 8 and time.monotonic() - t1 > 3.0:
                lines = armed_lines; break
        m = re.findall(r"tilt map: .*?\(tilt_max ([\d.]+)", s)
        tmax = float(m[-1]) if m else None
        tl = re.search(r"'tilt_left': (-?[\d.e+-]+)", lines[-1]) if lines else None
        tr = re.search(r"'tilt_right': (-?[\d.e+-]+)", lines[-1]) if lines else None
        if tmax is None or tl is None or tr is None:
            n.vcmd(400, 0.0, 21196.0)
            print(f"[fly_fsc_cpp] REFUSED: no armed HIL debug line with the tilt channels in {ilog} "
                  f"(launch with PEGASUS_HIL_DEBUG=1) -- disarmed")
            return 2
        ang = (float(tl.group(1)) * tmax, float(tr.group(1)) * tmax)
        print(f"[fly_fsc_cpp] nacelles on the ground, armed: left {ang[0]:+.1f} deg, right {ang[1]:+.1f} deg")
        # PX4 holds the hover collective on the tilts' upright stop (MINA) and yaws by tilting ONE nacelle
        # forward: a yaw correction on the ground is a differential with one nacelle still upright. A routing
        # or mapping error moves BOTH (the first flight: 45 / 45). So: one within 2 deg, neither past 30.
        if min(abs(ang[0]), abs(ang[1])) > 2.0 or max(abs(ang[0]), abs(ang[1])) > 30.0:
            n.vcmd(400, 0.0, 21196.0)
            print("[fly_fsc_cpp] REFUSED: the nacelles are not upright under PX4's allocation -- disarmed")
            return 2

    legs = [("climb", np.array([home[0], home[1], home[2] + a.alt]))]
    legs += [(f"wp({x:g},{y:g})", np.array([home[0] + x, home[1] + y, home[2] + a.alt])) for x, y in wps]
    legs += [("descend", np.array([home[0], home[1], home[2] - 0.3]))]
    rows = []; holds = []; t_fly = time.monotonic(); last_print = 0.0; aborted = None
    worst_att = 0.0; yaw0 = n.att[2]; worst_yaw = 0.0; climb_over = 0.0
    for name, goal in legs:
        n.goal = goal.copy()
        t_leg = time.monotonic(); t_in = None; hold_err = []
        while True:
            spin(0.02)
            now = time.monotonic(); t = now - t_fly
            if now - n.t_state > 0.5:
                aborted = "mocap stale"; break
            roll, pitch, yaw = n.att
            worst_att = max(worst_att, abs(roll), abs(pitch))
            worst_yaw = max(worst_yaw, abs((yaw - yaw0 + 180) % 360 - 180))
            if name == "climb":
                climb_over = max(climb_over, n.p[2] - goal[2])
            if max(abs(roll), abs(pitch)) > a.att_abort:
                aborted = f"attitude {roll:+.0f}/{pitch:+.0f}"; break
            if n.nav != 14 or not n.armed:
                if name == "descend" and not n.armed:
                    break
                aborted = f"PX4 left OFFBOARD/armed (nav {n.nav}, armed {n.armed})"; break
            err = float(np.linalg.norm(n.p - goal)); spd = float(np.linalg.norm(n.v))
            if now - last_print > 1.0:
                last_print = now
                print(f"  t {t:6.1f} {name:10s} p ({n.p[0]:+6.2f},{n.p[1]:+6.2f},{n.p[2]:5.2f}) ref ({n.ref[0]:+6.2f},"
                      f"{n.ref[1]:+6.2f},{n.ref[2]:5.2f}) err {err:5.2f} |v| {spd:4.2f} rpy ({roll:+5.1f},{pitch:+5.1f},"
                      f"{yaw:+6.1f})", flush=True)
            if name == "descend":
                if n.p[2] - home[2] < 0.15 and spd < 0.2:
                    if t_in is None:
                        t_in = now
                    if now - t_in > 2.0:
                        n.vcmd(400, 0.0)
                        if now - t_in > 6.0:
                            n.vcmd(400, 0.0, 21196.0)     # force, if PX4 will not disarm on its own
                else:
                    t_in = None
                if now - t_leg > a.leg_timeout + 30:
                    aborted = "descend timeout"; break
                continue
            arrived = err < a.tol and spd < a.vtol and float(np.linalg.norm(n.ref - goal)) < 1e-6
            if arrived and t_in is None:
                t_in = now
            if t_in is not None:
                hold_err.append(n.p - goal)
                if now - t_in >= a.hold:
                    e = np.array(hold_err); en = np.linalg.norm(e, axis=1)
                    holds.append((name, float(en.mean()), float(e.std(axis=0).max()), float(en.max())))
                    break
            if now - t_leg > a.leg_timeout:
                aborted = f"{name}: not inside {a.tol} m within {a.leg_timeout:g} s (err {err:.2f})"; break
        if aborted:
            break
    if aborted and n.armed and n.nav == 14 and not aborted.startswith("mocap") and n.p[2] - home[2] > 0.15:
        print(f"[fly_fsc_cpp] ABORTED ({aborted}): landing in place at ({n.p[0]:+.2f}, {n.p[1]:+.2f})", flush=True)
        land_in_place(n, spin, home[2], a.leg_timeout + 30)
    land_err = float(np.linalg.norm((n.p - home)[:2]))
    if aborted:
        n.vcmd(400, 0.0)
    print("\n[fly_fsc_cpp] SUMMARY")
    for name, m, sd, w in holds:
        print(f"  hold {name:10s} mean err {m:.3f} m  sd {sd:.3f}  worst {w:.3f}")
    print(f"  climb overshoot {climb_over:+.2f} m   worst |roll|/|pitch| {worst_att:.1f} deg   worst yaw drift {worst_yaw:.1f} deg")
    print(f"  landing {land_err:.2f} m from home, armed={n.armed}" + (f"   ABORTED: {aborted}" if aborted else ""))
    rclpy.shutdown()
    if aborted:
        return 3
    return 0 if land_err < 0.5 and not n.armed else 1


if __name__ == "__main__":
    sys.exit(main())
