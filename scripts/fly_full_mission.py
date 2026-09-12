#!/usr/bin/env python3
"""ONE-COMMAND wing-borne mission: takeoff -> transition -> cruise -> back-transition -> land.

    scripts/fly_full_mission.py                 # 60 m climb, 1 km cruise, land downrange
    scripts/fly_full_mission.py --dist 1000 --alt 60

The autonomous twin of the GS "Full Auto Mission" button. It SEQUENCES the flight-validated
primitives (it does NOT play a position trajectory -- the transition is speed-scheduled inside
the NMPC, so a position CSV fights it; measured 2026-09-08, a clocked reference ran 260 m ahead
of the vehicle). Phases, each completion- or timeout-gated:

  1 CLIMB          arm + OFFBOARD, climb straight up to --alt on the SAFE geometric controller
  2 ENGAGE         apply the validated WB params (ttr40/flare15/aft-tilt20 -- each proven by a
                   HIL bag) to nmpc_full, switch to it
  3 CRUISE         release a target --dist ahead; the NMPC accelerates, tilts to 90 deg and
                   goes wing-borne on its own schedule. Aborts to safe hover on altitude loss
                   or attitude departure.
  4 BACK-TRANS     decel_now: decelerate on the trim schedule (nose-up flare + aft rotor tilt)
                   to a LATCHED stop point, until below 3 m/s. Without this the vehicle is
                   handed to the multicopter controller at cruise speed, tilt snaps to 0, lift
                   doubles and it departs (bag full_tr30: 65.7 m excursion).
  5 LAND           hand to geometric, descend to the ground where it stopped (~--dist downrange),
                   force-disarm once settled (a plain disarm is refused while the controller
                   still holds altitude).

Reuses fly_transition.Flight (telemetry + aero wing-share) so what this proves is what that
validated harness proved. Run with the SYSTEM/isaac_env python from a ROS-sourced shell
(scripts/rosenv.sh); stack up via start_vtol_smc_hil.sh CONTROLLERS="geometric nmpc_full"
(smc_outer must be running -- it bridges the raw reference to nmpc_full).
"""
import argparse
import math
import subprocess
import sys
import time

import numpy as np
import rclpy
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, VehicleStatus

import fly_transition as ft           # Flight, spin, AERO -- the validated harness

NMPC_NODE = "/nmpc_full_smc_control"
GEO_NODE = "/geometric_hover_control"

# The flight-validated gentle-transition configuration (mirrors the GS Wing-borne button's
# WB_PARAMS; every value established by a HIL bag -- see docs/wing_borne_90_runbook.md).
WB_PARAMS = {
    "ttr": 40.0, "trans_sink": 0.8, "v_lead": 2.5, "vcruise": 16.0,
    "wingborne_v": 14.5, "descent_budget": 45.0, "cruise_alt_band": 60.0,
    "tilt_margin_deg": 8.0, "decel_now": 0.0, "decel_rate": 1.5,
    "flare_deg": 15.0, "beta_aft_deg": 20.0,
}


def set_param(node, key, val):
    r = subprocess.run(["ros2", "param", "set", node, key, str(val)],
                       capture_output=True, timeout=8)
    return r.returncode == 0 and b"successful" in r.stdout.lower()


def send_cmd(n, command, p1=0.0, p2=0.0):
    m = VehicleCommand()
    m.command = int(command); m.param1 = float(p1); m.param2 = float(p2)
    m.target_system = m.target_component = m.source_system = m.source_component = 1
    m.from_external = True
    m.timestamp = int(n.get_clock().now().nanoseconds / 1000)
    n.cmd_pub.publish(m)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--ns", default="uav_0")
    ap.add_argument("--alt", type=float, default=60.0, help="cruise altitude [m] (WB needs >=60)")
    ap.add_argument("--dist", type=float, default=1000.0, help="wing-borne cruise distance [m]")
    ap.add_argument("--climb-time", type=float, default=90.0)
    ap.add_argument("--arm-timeout", type=float, default=180.0,
                    help="ARM retries this long: the EKF can take >90 s to settle after a "
                         "fresh stack restart (PX4 rejects with 'Resolve system health failures')")
    ap.add_argument("--cruise-time", type=float, default=180.0)
    ap.add_argument("--bt-time", type=float, default=90.0)
    ap.add_argument("--land-time", type=float, default=120.0)
    ap.add_argument("--no-arm", action="store_true")
    a = ap.parse_args()

    rclpy.init()
    n = ft.Flight(a.ns)
    n.cmd_pub = n.create_publisher(VehicleCommand, f"/{a.ns}/fmu/in/vehicle_command", 10)
    n._armed = False
    n._offb = False
    n.create_subscription(VehicleStatus, f"/{a.ns}/fmu/out/vehicle_status_v1",
                          lambda m: (setattr(n, "_armed", m.arming_state == 2),
                                     setattr(n, "_offb", m.nav_state == 14)),
                          ft.qos_profile_sensor_data)
    n.t0 = time.time()
    ft.spin(n, 3.0)
    if n.p is None:
        print("no mocap -- is the stack up?"); sys.exit(2)
    start = n.p.copy()
    print(f"start at ({start[0]:.1f},{start[1]:.1f},{start[2]:.1f}); "
          f"mission: climb {a.alt:.0f} m -> cruise {a.dist:.0f} m -> back-transition -> land")

    # ---- 1 CLIMB: arm + offboard + climb on the safe controller -------------------------
    print("\n== 1 CLIMB (arm + offboard, safe controller) ==")
    n.set_ctrl("geometric")
    if not a.no_arm:
        for node in (GEO_NODE, NMPC_NODE):
            set_param(node, "auto_arm", "true")
        t0, last = time.time(), 0.0
        while not (n._armed and n._offb):
            n.send([start[0], start[1], a.alt]); n.set_ctrl("geometric")
            if time.time() - last > 2.0:
                send_cmd(n, 176, 1.0, 6.0); send_cmd(n, 400, 1.0)
                last = time.time()
                print(f"  arming: armed={n._armed} offboard={n._offb} (EKF may still be settling)")
            ft.spin(n, 0.2)
            if time.time() - t0 > a.arm_timeout:
                print("  !! arming timed out -- check window 1-px4"); sys.exit(3)
        print("  armed + offboard")

    def climb():
        n.send([start[0], start[1], a.alt]); n.set_ctrl("geometric")
        n.line("climb")
        return abs(n.p[2] - a.alt) < 1.0 and float(np.linalg.norm(n.v)) < 0.7
    if not ft.spin(n, a.climb_time, climb, hz=1.0):
        print("  climb did not settle -- continuing anyway")

    # ---- 2 ENGAGE: apply WB params, switch to nmpc_full ---------------------------------
    print("\n== 2 ENGAGE nmpc_full (validated wing-borne params) ==")
    if n.p[2] < 55.0:
        print(f"  !! only {n.p[2]:.0f} m -- the transition needs ~60 m; aborting")
        _abort_to_hover(n, a); return
    bad = [k for k, v in WB_PARAMS.items() if not set_param(NMPC_NODE, k, v)]
    set_param(GEO_NODE, "radius_max", "8000.0"); set_param(GEO_NODE, "radius_local", "2000.0")
    if bad:
        print(f"  !! WB params FAILED: {bad} -- is nmpc_full running? aborting")
        _abort_to_hover(n, a); return
    base = n.p.copy()
    for _ in range(12):
        n.set_ctrl("nmpc_full"); ft.spin(n, 0.25)
    n.line("engage")

    # ---- 3 CRUISE: release the far target, let the NMPC transition + cruise --------------
    print(f"\n== 3 CRUISE: target {a.dist:.0f} m ahead ==")
    tgt = [base[0] + a.dist, base[1], a.alt]
    peak = {"V": 0.0, "tilt": 0.0, "wing": -1e9}
    aborted = {"why": None}
    def cruise():
        n.send(tgt); n.set_ctrl("nmpc_full")
        d = n.telem()
        peak["V"] = max(peak["V"], d["V"]); peak["tilt"] = max(peak["tilt"], d["tilt"])
        peak["wing"] = max(peak["wing"], d["wing"]); n.line("cruise")
        if d["p"][2] < 2.0:
            aborted["why"] = "ALTITUDE LOST"; return True
        if abs(d["roll"]) > 80 or abs(d["pitch"]) > 60:
            aborted["why"] = "ATTITUDE DEPARTURE"; return True
        return float(np.linalg.norm(d["p"][:2] - np.array(tgt[:2]))) < 20.0
    ft.spin(n, a.cruise_time, cruise, hz=2.0)
    if aborted["why"]:
        print(f"  !! {aborted['why']} -- aborting to safe hover")
        _abort_to_hover(n, a); return
    print(f"  reached cruise target region; peak V={peak['V']:.1f} tilt={peak['tilt']:.0f} "
          f"wing={peak['wing']:.0f}%")

    # ---- 4 BACK-TRANSITION: decelerate on the schedule to a latched stop point -----------
    print("\n== 4 BACK-TRANSITION (decelerate: nose-up flare + aft tilt) ==")
    set_param(NMPC_NODE, "decel_now", "1.0")
    hd0 = math.atan2(n.R[1, 0], n.R[0, 0])
    stop_pt = [n.p[0] + 120.0 * math.cos(hd0), n.p[1] + 120.0 * math.sin(hd0), a.alt]
    bt = {"n": 0, "frozen": 0, "last": None}
    def back():
        n.send(stop_pt); d = n.telem(); n.line("backtrans")
        key = (round(d["p"][0], 4), round(d["p"][1], 4), round(d["p"][2], 4), round(d["V"], 4))
        bt["frozen"] = bt["frozen"] + 1 if key == bt["last"] else 0
        bt["last"] = key
        return d["V"] < 3.0 and bt["frozen"] < 4
    slowed = ft.spin(n, a.bt_time, back, hz=2.0)
    set_param(NMPC_NODE, "decel_now", "0.0")
    if bt["frozen"] >= 4:
        print("  !! SIMULATION FROZEN (dead HIL link / PX4 killed) -- INVALID; not landing")
        return
    print(f"  slowed below 3 m/s: {slowed}")

    # ---- 5 LAND: hand to geometric, descend to ground downrange, disarm ------------------
    print("\n== 5 LAND (safe controller, descend where it stopped) ==")
    land_xy = (float(n.p[0]), float(n.p[1]))
    for _ in range(12):
        n.set_ctrl("geometric"); n.send([land_xy[0], land_xy[1], max(a.alt, 8.0)]); ft.spin(n, 0.25)
    # settle to a low hover first, then descend to the ground
    t0 = time.time()
    while time.time() - t0 < 20.0 and float(np.linalg.norm(n.v)) > 2.0:
        n.send([land_xy[0], land_xy[1], 8.0]); n.set_ctrl("geometric"); n.line("settle"); ft.spin(n, 0.5)
    settled = False
    t0 = time.time()
    while time.time() - t0 < a.land_time:
        n.send([land_xy[0], land_xy[1], 0.05]); n.set_ctrl("geometric"); n.line("land")
        ft.spin(n, 0.4)
        if n.p[2] < 0.25 and float(np.linalg.norm(n.v)) < 0.4:
            settled = True; break
    if settled:
        for node in (GEO_NODE, NMPC_NODE):
            set_param(node, "auto_arm", "false")
        send_cmd(n, 400, 0.0, 21196.0)          # FORCED disarm (held z target hides low throttle)
        print(f"  landed at ({land_xy[0]:.1f},{land_xy[1]:.1f}), z={n.p[2]:.2f} m -- disarmed")
    else:
        print(f"  NOT settled after {a.land_time:.0f} s (z={n.p[2]:.2f} m) -- staying armed, holding")

    print("\n===== SUMMARY =====")
    print(f"  cruise distance   {a.dist:.0f} m")
    print(f"  peak speed        {peak['V']:.2f} m/s   (stall ~14.1, wing-borne cruise ~15.6)")
    print(f"  peak tilt         {peak['tilt']:.1f} deg  (90 = rotors fully forward)")
    print(f"  peak wing share   {peak['wing']:.1f} % of weight")
    n.destroy_node(); rclpy.shutdown()


def _abort_to_hover(n, a):
    """Safety fallback used on any phase failure: stage a back-transition if fast, then
    hand to the safe controller and hold."""
    if n._armed and float(np.linalg.norm(n.v)) > 5.0:
        set_param(NMPC_NODE, "decel_now", "1.0")
        t0 = time.time()
        while time.time() - t0 < 60.0 and float(np.linalg.norm(n.v)) > 3.0:
            n.send([n.p[0], n.p[1], max(a.alt, 8.0)]); ft.spin(n, 0.2)
        set_param(NMPC_NODE, "decel_now", "0.0")
    for _ in range(12):
        n.set_ctrl("geometric"); n.send([n.p[0], n.p[1], max(float(n.p[2]), 8.0)]); ft.spin(n, 0.25)
    print("  aborted -- safe hover; Ctrl-C to exit")
    try:
        while rclpy.ok():
            n.send([n.p[0], n.p[1], max(float(n.p[2]), 8.0)]); n.set_ctrl("geometric"); ft.spin(n, 0.2)
    except KeyboardInterrupt:
        pass
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
