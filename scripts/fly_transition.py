#!/usr/bin/env python3
"""Drive and instrument a hover -> transition -> cruise -> hover mission.

    scripts/fly_transition.py --alt 20 --dist 300 --controller nmpc_full

Phases, each held until its exit condition (or its timeout):
  1 CLIMB     on the SAFE controller, up to --alt
  2 ENGAGE    hand over to --controller, confirm it is driving
  3 CRUISE    release a far target; the controller accelerates, tilts and (if it can)
              goes wing-borne. Runs until the target is reached or --cruise-time elapses.
  4 RECOVER   command a hover at the current position; wait for it to slow down, then hand
              back to the safe controller.

Prints the numbers that decide whether it is REAL fixed-wing flight rather than a fast
multicopter: tilt angle, angle of attack, and above all the WING SHARE -- how much of the
aircraft's weight the wing is carrying, computed from the simulator's own aero model at the
measured state. Rotors at ~100% of weight with tilt 0 is a quadrotor, whatever its speed.
"""
import argparse
import importlib.util
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy,
                       QoSHistoryPolicy, qos_profile_sensor_data)
from std_msgs.msg import String
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference
from px4_msgs.msg import ActuatorMotors, ActuatorServos

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "vtol_aerodynamics",
    os.path.join(_HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
                 "logic", "dynamics", "vtol_aerodynamics.py"))
_aero_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_aero_mod)
AERO = _aero_mod.VTOLAerodynamics()
WEIGHT = 3.45 * 9.81
MAX_THRUST = 18.0


class _St:
    def __init__(self, vb, wb):
        self.linear_body_velocity = vb
        self.angular_velocity = wb


def quat_R(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Flight(Node):
    def __init__(self, ns, cid="cascade"):
        super().__init__("fly_transition")
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.ref = self.create_publisher(
            PositionControllerReference,
            f"/{ns}/fsc_autopilot_ros2/position_controller/reference", 10)
        self.ctl = self.create_publisher(String, f"/{ns}/ground_station/active_controller", 10)
        # the cascade's guidance protocol: "wp X Y ...", "loiter X Y R [dir]", "rth", "hold" --
        # on the ACTIVE cascade's own topic (the adaptive cascade listens on cascade_adaptive/cmd)
        gtop = cid if str(cid).startswith("cascade") else "cascade"
        self.cmd = self.create_publisher(String, f"/{ns}/{gtop}/cmd", 10)
        # the cascade's status feed (5 Hz): `stopped=1` is the node saying its back-transition has
        # run the speed reference to zero with the rotors upright. In WIND its ground speed after
        # the stop is the crosswind's drift, so "V < 3 m/s" never comes (piece 4, 2026-09-15).
        self.casc_status = ""
        self.create_subscription(String, f"/{ns}/{gtop}/status", self.on_casc_status, 10)
        self.create_subscription(Mocap, f"/{ns}/mocap", self.on_mocap, qos_profile_sensor_data)
        self.create_subscription(ActuatorMotors, f"/{ns}/fmu/in/actuator_motors", self.on_mot, qos)
        self.create_subscription(ActuatorServos, f"/{ns}/fmu/in/actuator_servos", self.on_srv, qos)
        self.p = self.v = None
        self.R = np.eye(3)
        self.w = np.zeros(3)
        self.mot = np.zeros(4)
        self.srv = np.zeros(5)
        self.log = []

    def on_mocap(self, m):
        self.p = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        self.v = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
        self.w = np.array([m.twist.angular.x, m.twist.angular.y, m.twist.angular.z])
        self.R = quat_R(m.pose.orientation)

    def on_mot(self, m):
        c = [x for x in list(m.control[:4]) if not math.isnan(x)]
        if len(c) == 4:
            self.mot = np.array(c)

    def on_srv(self, m):
        c = [x for x in list(m.control[:5]) if not math.isnan(x)]
        if len(c) == 5:
            self.srv = np.array(c)

    def set_ctrl(self, name):
        self.ctl.publish(String(data=name))

    def send(self, xyz, yaw_deg=0.0):
        r = PositionControllerReference()
        r.position.x, r.position.y, r.position.z = [float(a) for a in xyz]
        r.yaw = float(yaw_deg)
        try:
            r.yaw_unit = PositionControllerReference.DEGREES
        except Exception:
            pass
        self.ref.publish(r)

    def on_casc_status(self, m):
        self.casc_status = str(m.data)

    def casc_stopped(self):
        return "stopped=1" in self.casc_status

    def telem(self):
        """Everything needed to tell fixed-wing flight from a fast multicopter."""
        vb = self.R.T @ self.v
        F, _ = AERO.update(_St(vb, self.w), None)
        lift_world_z = float(self.R[2] @ F)
        V = float(np.linalg.norm(self.v))
        roll = math.degrees(math.atan2(self.R[2, 1], self.R[2, 2]))
        pitch = math.degrees(math.asin(float(np.clip(self.R[2, 0], -1, 1))))   # nose-up +
        yaw = math.degrees(math.atan2(self.R[1, 0], self.R[0, 0]))
        return dict(t=time.time(), p=self.p.copy(), V=V, vz=float(self.v[2]),
                    aoa=math.degrees(AERO.last_state.alpha) if V > 1.0 else 0.0,
                    tilt=math.degrees(float(self.srv[0]) * math.pi / 2),
                    thrust=float(self.mot.sum()) * MAX_THRUST,
                    wing=100.0 * lift_world_z / WEIGHT,
                    roll=roll, pitch=pitch, yaw=yaw)

    def line(self, phase, tag=""):
        d = self.telem()
        self.log.append(d)
        print(f"  {phase:8s} t={d['t']-self.t0:6.1f} p=({d['p'][0]:7.1f},{d['p'][1]:6.1f},"
              f"{d['p'][2]:5.1f}) V={d['V']:5.2f} tilt={d['tilt']:5.1f} AoA={d['aoa']:+6.1f} "
              f"wing={d['wing']:+6.1f}% rotors={d['thrust']:5.1f}N "
              f"rpy=({d['roll']:+5.0f},{d['pitch']:+5.0f},{d['yaw']:+5.0f}) {tag}", flush=True)


def spin(n, secs, fn=None, hz=5.0):
    t = time.time()
    last = 0.0
    while rclpy.ok() and time.time() - t < secs:
        rclpy.spin_once(n, timeout_sec=0.02)
        if fn and time.time() - last > 1.0 / hz:
            last = time.time()
            if fn() is True:
                return True
    return False



# Which ROS node implements each controller_id. The back-transition is triggered by a param on the
# ACTIVE controller's node, and hardcoding one of them meant --controller silently drove the wrong
# node's parameter (setting decel_now on nmpc_full while the cascade was flying).
CTRL_NODE = {
    "nmpc_full": "/nmpc_full_smc_control",
    "cascade": "/nmpc_cascade_control",
    "cascade_adaptive": "/nmpc_cascade_control_adaptive",   # STESO + slosh predictor (2026-09-14)
    "nmpc6_cruise": "/nmpc6_cruise_control",
}


def ctrl_node(cid):
    return CTRL_NODE.get(cid, "/nmpc_full_smc_control")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ns", default="uav_0")
    ap.add_argument("--controller", default="nmpc_full")
    ap.add_argument("--alt", type=float, default=20.0)
    ap.add_argument("--dist", type=float, default=300.0)
    ap.add_argument("--climb-time", type=float, default=45.0)
    ap.add_argument("--cruise-time", type=float, default=70.0)
    ap.add_argument("--recover-time", type=float, default=45.0)
    ap.add_argument("--back-transition", action="store_true",
                    help="decelerate on the trim schedule with a nose-up flare before handing over")
    ap.add_argument("--bt-time", type=float, default=60.0)
    ap.add_argument("--land", action="store_true",
                    help="after recovering, fly to --land-x/--land-y and touch down there")
    ap.add_argument("--land-x", type=float, default=None)
    ap.add_argument("--land-y", type=float, default=None)
    ap.add_argument("--land-time", type=float, default=90.0)
    ap.add_argument("--guide", default="",
                    help='guidance plan "T:cmd;T:cmd" (T seconds after the cruise starts), e.g. '
                         '"10:wp 400 0 400 300 0 300;90:rth". With an rth the node brakes itself: '
                         'the cruise ends when it has stopped and the driver skips its own '
                         'back-transition; land at --land-x/y (home).')
    a = ap.parse_args()

    rclpy.init()
    n = Flight(a.ns, a.controller)
    n.t0 = time.time()
    spin(n, 3.0)
    if n.p is None:
        print("no mocap -- is the stack up?"); sys.exit(2)
    start = n.p.copy()
    print(f"start at ({start[0]:.1f},{start[1]:.1f},{start[2]:.1f})")

    print("\n== 1 CLIMB (safe controller) ==")
    n.set_ctrl("geometric")
    def climb():
        n.send([start[0], start[1], a.alt]); n.set_ctrl("geometric")
        n.line("climb")
        return abs(n.p[2] - a.alt) < 0.5 and np.linalg.norm(n.v) < 0.5
    if not spin(n, a.climb_time, climb, hz=1.0):
        print("  climb did not settle -- continuing anyway")

    print(f"\n== 2 ENGAGE {a.controller} ==")
    base = n.p.copy()
    for _ in range(12):
        n.set_ctrl(a.controller); spin(n, 0.25)
    n.line("engage")

    print(f"\n== 3 CRUISE: target {a.dist:.0f} m ahead ==")
    tgt = [base[0] + a.dist, base[1], a.alt]
    peak = {"V": 0.0, "tilt": 0.0, "wing": -1e9}
    plan = []
    for part in a.guide.split(";"):
        if ":" in part:
            tt, cmd = part.split(":", 1); plan.append((float(tt), cmd.strip()))
    plan.sort()
    guided = {"t0": time.time(), "rth": False, "stopped": False}
    if plan:
        print("  guidance plan: " + "; ".join(f"t+{tt:.0f}: {c}" for tt, c in plan))
        # ★ THE CRUISE WINDOW MUST OUTLAST THE PLAN. --cruise-time defaults to 70 s, and when it
        # expires before the guidance has stopped the aircraft this driver forces decel_now
        # itself -- bag slosh_20260913_224823: a plan with rth at t+150 was cut off at t+55, in
        # a 26 deg bank in the second fly-by turn (it survived; the flight was lost). A plan that
        # ends in rth gets its own window: the last command's time plus 150 s for the leg home,
        # the braking and the stop (the loop returns early the moment the rth stop is seen).
        need = plan[-1][0] + 150.0
        if a.cruise_time < need:
            print(f"  cruise window {a.cruise_time:.0f} s is shorter than the plan: raised to {need:.0f} s")
            a.cruise_time = need
    def cruise():
        n.send(tgt); n.set_ctrl(a.controller)
        d = n.telem()
        now = time.time() - guided["t0"]
        while plan and now >= plan[0][0]:
            tt, cmd = plan.pop(0)
            n.cmd.publish(String(data=cmd))
            print(f"  >> GUIDE t+{now:.0f}: '{cmd}'")
            if cmd.split()[0].lower() == "rth":
                guided["rth"] = True
        if guided["rth"] and ((d["V"] < 3.0 and d["tilt"] < 15.0) or n.casc_stopped()):
            guided["stopped"] = True
            print(f"  RTH complete: stopped at ({d['p'][0]:.1f}, {d['p'][1]:.1f}), V={d['V']:.2f}"
                  + ("  (the cascade reports stopped=1; the ground speed is the wind's drift)" if d["V"] >= 3.0 else ""))
            return True
        peak["V"] = max(peak["V"], d["V"]); peak["tilt"] = max(peak["tilt"], d["tilt"])
        peak["wing"] = max(peak["wing"], d["wing"])
        n.line("cruise")
        if d["p"][2] < 2.0:
            print("  !! ALTITUDE LOST -- aborting to the safe controller"); return True
        if abs(d["roll"]) > 80 or abs(d["pitch"]) > 60:
            print("  !! ATTITUDE DEPARTURE -- aborting to the safe controller"); return True
        return float(np.linalg.norm(d["p"][:2] - np.array(tgt[:2]))) < 15.0
    spin(n, a.cruise_time, cruise, hz=2.0)

    # ---- 3b BACK-TRANSITION: decelerate on the schedule BEFORE handing over -------------
    # Bag full_tr30: without this the harness hands a 16 m/s wing-borne aircraft straight to the
    # multicopter controller, which writes tilt to literal ZERO. Lift doubles, it balloons 10 m,
    # stalls at AoA +78 deg and departs into a dive -- a 65.7 m excursion. The vehicle has to be
    # SLOWED DOWN first, nose-up, on the same trim schedule that took it out.
    if a.back_transition and guided["stopped"]:
        print("\n== 3b BACK-TRANSITION: done by the guidance (rth) -- skipped ==")
    elif a.back_transition:
        print("\n== 3b BACK-TRANSITION (decelerate on the schedule, nose-up flare) ==")
        import subprocess
        subprocess.run(["ros2","param","set",ctrl_node(a.controller),"decel_now","1.0"],
                       capture_output=True, timeout=10)
        bt = {"zmin": 1e9, "zmax": -1e9, "aoa": 0.0, "vzmax": 0.0, "n": 0, "frozen": 0,
              "last": None}
        # ★ LATCH the stop point ONCE.  Recomputing "40 m ahead of the CURRENT heading" every
        # tick is a moving carrot: the vehicle always has somewhere to go, so it never has a
        # reason to stop.  Bag full_tr34 chased it for 70 s in a slow turn (yaw 32 -> 20 deg)
        # and only bled 16.3 -> 12.1 m/s.  The braking distance is ~V^2/(2a); 120 m is generous
        # at 16 m/s and 1.5 m/s^2 so the reference never asks it to stop SHORT.
        hd0 = math.atan2(n.R[1, 0], n.R[0, 0])
        stop_pt = [n.p[0] + 120.0 * math.cos(hd0), n.p[1] + 120.0 * math.sin(hd0), a.alt]
        def back():
            n.send(stop_pt)
            d = n.telem(); n.line("backtrans")
            bt["zmin"] = min(bt["zmin"], d["p"][2]); bt["zmax"] = max(bt["zmax"], d["p"][2])
            bt["aoa"] = max(bt["aoa"], abs(d["aoa"])); bt["vzmax"] = max(bt["vzmax"], abs(d["vz"]))
            # FROZEN-SIM GUARD.  A dead HIL link (or a killed PX4) leaves mocap repeating the last
            # sample forever: bag full_tr33 reported "slowed below 3 m/s: True" while airspeed sat
            # at 16.60 for the whole window, because the state simply stopped changing.  A result
            # from a frozen simulator must never be reported as a manoeuvre.
            bt["n"] += 1
            key = (round(d["p"][0], 4), round(d["p"][1], 4), round(d["p"][2], 4), round(d["V"], 4))
            bt["frozen"] = bt["frozen"] + 1 if key == bt["last"] else 0
            bt["last"] = key
            return d["V"] < 3.0 and bt["frozen"] < 4
        stopped = spin(n, a.bt_time, back, hz=2.0)
        if bt["frozen"] >= 4:
            print(f"  !! SIMULATION FROZEN — state unchanged for {bt['frozen']} samples "
                  f"(dead HIL link or PX4 killed). THIS RUN IS INVALID.")
        print(f"  slowed below 3 m/s: {stopped}")
        print(f"  altitude {bt['zmin']:.1f}-{bt['zmax']:.1f} m (excursion {bt['zmax']-bt['zmin']:.1f} m), "
              f"peak |AoA| {bt['aoa']:.0f} deg, peak |vz| {bt['vzmax']:.1f} m/s")
        subprocess.run(["ros2","param","set",ctrl_node(a.controller),"decel_now","0.0"],
                       capture_output=True, timeout=10)

    print("\n== 4 RECOVER to hover ==")
    hold = [n.p[0] + 0.0, n.p[1], max(a.alt, 8.0)]
    def rec():
        n.send(hold); n.line("recover")
        return float(np.linalg.norm(n.v)) < 2.0
    slow = spin(n, a.recover_time, rec, hz=1.0)
    print(f"  slowed below 2 m/s: {slow}; handing back to the safe controller")
    for _ in range(12):
        n.set_ctrl("geometric"); n.send([n.p[0], n.p[1], max(a.alt, 8.0)]); spin(n, 0.25)
    spin(n, 12.0, lambda: (n.send([hold[0], hold[1], hold[2]]), n.line("hover"))[1], hz=1.0)

    # ---- 5 LAND at a precise coordinate -------------------------------------------------
    # The mission is only finished when the aircraft is ON THE GROUND WHERE IT WAS ASKED TO BE.
    # Everything before this is judged in the air, and a run that cruises beautifully and then
    # arrives 40 m off target is not a completed mission -- the whole point of killing the
    # forward velocity with flare + aft tilt is to be able to stop somewhere specific.
    # Flown on the SAFE controller: it owns fly-to-point and holds a commanded x,y,z (validated
    # to 6 mm hold error, 0.02 m arrival), so the landing does not depend on the NMPC at all.
    land_err = None
    if a.land:
        tx = a.land_x if a.land_x is not None else n.p[0]
        ty = a.land_y if a.land_y is not None else n.p[1]
        print(f"\n== 5 LAND at ({tx:.1f}, {ty:.1f}) ==")
        # ★ A LANDING ONLY COUNTS IF THE AIRCRAFT WAS FLYING.
        # Trial bt_flare25 reported "LANDING ERROR 0.00 m" after DEPARTING and hitting the
        # ground at 10.6 m/s -- it tumbled to a stop near the target, and the landing phase then
        # commanded a point it was already lying on. Measuring position after a crash is not a
        # landing accuracy measurement. Require real altitude and a controlled attitude first.
        R0 = n.R if hasattr(n, "R") else None
        if n.p[2] < 5.0:
            print(f"  LANDING INVALID -- not airborne (z={n.p[2]:.2f} m); "
                  f"the aircraft is on the ground before the landing phase began")
            land_err = None
            a.land = False
    if a.land:
        n.set_ctrl("geometric")
        # close in at altitude first, then descend -- a diagonal to the ground lands short
        def approach():
            n.send([tx, ty, max(a.alt, 8.0)]); n.line("approach")
            return math.hypot(n.p[0] - tx, n.p[1] - ty) < 1.0
        arrived = spin(n, a.land_time, approach, hz=1.0)
        print(f"  over the point: {arrived} (err {math.hypot(n.p[0]-tx, n.p[1]-ty):.2f} m)")
        def descend():
            n.send([tx, ty, 0.15]); n.line("land")
            return n.p[2] < 0.35
        down = spin(n, a.land_time, descend, hz=1.0)
        land_err = math.hypot(n.p[0] - tx, n.p[1] - ty)
        print(f"  touchdown: {down}  at ({n.p[0]:.2f}, {n.p[1]:.2f}, {n.p[2]:.2f})  "
              f"error {land_err:.2f} m from ({tx:.1f}, {ty:.1f})")

    print("\n===== SUMMARY =====")
    print(f"  peak speed        {peak['V']:.2f} m/s   (stall ~14.1, wing-borne cruise ~15.6)")
    print(f"  peak tilt         {peak['tilt']:.1f} deg  (90 = rotors fully forward)")
    print(f"  peak wing share   {peak['wing']:.1f} % of weight")
    # VERDICT ON SUSTAINED, LEVEL FLIGHT ONLY.
    # Peaks lie. Flight full_tr11 hit 15.8 m/s / 120% wing share and this function called it
    # "TRUE FIXED-WING" -- it was a DIVE, trading 17 m of altitude for speed on the way to
    # touching the ground at 0.33 m. Anything measured while descending is gravity, not lift,
    # so the verdict now only counts samples that are (a) roughly level and (b) held.
    # LEVEL IS NOT ENOUGH -- the sample must also be FLOWN.  When a solve fails the node
    # falls back to safe_hover(), which commands equal thrust ~= weight (33.8 N) and tilt 0;
    # the vehicle then GLIDES, and a glide at speed is level, fast and wing-loaded.  Bags
    # full_tr13 and full_tr15 both reported spectacular headline numbers (80.3% wing share,
    # then "15.31 m/s / 120.7%") that were recorded with the controller already dropped out.
    # Requiring the rotors to be UNLOADED separates flying from falling with style.
    # NOTE these are per-column maxima over the selected samples and need NOT be simultaneous;
    # the single best simultaneous sample is printed separately below.
    HOVER_N = 32.0                   # >= this is the safe_hover fallback, not controlled flight
    lg = [d for d in n.log if abs(d["vz"]) < 1.5 and d["V"] > 3.0]
    cg = [d for d in lg if d["thrust"] < HOVER_N]
    n_lvl, n_ctl = len(lg), len(cg)
    lvl_wing = max((d["wing"] for d in cg), default=0.0)
    lvl_tilt = max((d["tilt"] for d in cg), default=0.0)
    lvl_V = max((d["V"] for d in cg), default=0.0)
    # ★ INSTANTANEOUS LEVEL IS NOT STEADY FLIGHT (bag full_tr20).  At the BOTTOM of a phugoid
    # `vz` passes through zero, so a porpoising vehicle -- altitude cycling 63-90 m, speed 7-20
    # m/s, vz +-7.5 m/s -- scatters samples that each pass |vz| < 1.5 and it scored
    # "TRUE FIXED-WING (sustained, level, controlled)".  Steady flight means CONSECUTIVE level
    # samples, so require the longest unbroken RUN, not the total count.
    run = best_run = 0
    for d in n.log:
        lvl = abs(d["vz"]) < 1.5 and d["V"] > 3.0 and d["thrust"] < HOVER_N
        run = run + 1 if lvl else 0
        best_run = max(best_run, run)
    print(f"  longest UNBROKEN controlled+level run: {best_run} samples "
          f"(~{best_run / 2.0:.1f} s at the 2 Hz cruise sampling)")
    sustained = best_run >= 10       # ~5 s of CONSECUTIVE level flight
    print(f"  LEVEL samples (|vz| < 1.5 m/s): {n_lvl}; of those CONTROLLED "
          f"(rotors < {HOVER_N:.0f} N, i.e. not the hover fallback): {n_ctl}")
    print(f"  CONTROLLED + LEVEL maxima (not necessarily simultaneous): "
          f"V {lvl_V:.2f} m/s, tilt {lvl_tilt:.1f} deg, wing {lvl_wing:.1f}%")
    if cg:
        b = max(cg, key=lambda d: d["V"])
        print(f"  fastest CONTROLLED sample: V {b['V']:.2f} m/s, tilt {b['tilt']:.1f} deg, "
              f"wing {b['wing']:.1f}%, rotors {b['thrust']:.1f} N, z {b['p'][2]:.1f} m")
    if not sustained:
        verdict = "INCONCLUSIVE -- too little CONTROLLED level flight to judge"
    elif lvl_wing > 70 and lvl_tilt > 60 and lvl_V > 13:
        verdict = "TRUE FIXED-WING (sustained, level, controlled)"
    elif lvl_wing > 30:
        verdict = "partial wing-borne (level, controlled)"
    else:
        verdict = "MULTICOPTER (the wing did nothing)"
    lost = max((d0["p"][2] - d1["p"][2]) for d0 in n.log[:1] for d1 in n.log) if n.log else 0.0
    if lost > 5.0:
        verdict += f"   [!! lost {lost:.0f} m of altitude -- check for a dive]"
    print(f"  verdict           {verdict}")
    if land_err is not None:
        print(f"  LANDING ERROR     {land_err:.2f} m from the commanded point")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
