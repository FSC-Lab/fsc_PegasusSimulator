#!/usr/bin/env python3
"""LIVE end-to-end test of the ground-station Vehicle + Wing-borne controls.

Drives the REAL Trajectory tab against a RUNNING HIL stack: it builds the widget with a real
rclpy node (so ARM / OFFBOARD / TAKEOFF actually reach PX4) and then clicks the buttons in the
order an operator would, checking the vehicle responds.

The offscreen smoke test (test_gs_trajectory_tab.py) checks the ROUTING rules with a stubbed ROS.
This one checks that the buttons FLY THE AIRCRAFT.  Run it after `scripts/restart_hil.sh`:

    /usr/bin/python3 tests/test_gs_live_flight.py
"""
import os, sys, time, math
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/ros2_ground_station_gui/src"))
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from PyQt5 import QtWidgets
import types

FAIL = []
def check(name, ok, detail=""):
    print(f"  {'OK  ' if ok else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}")
    if not ok:
        FAIL.append(name)

class LiveRos(Node):
    """Minimal stand-in for the GS ros object: real pub/sub, stubbed data_struct."""
    def __init__(self):
        super().__init__("gs_live_test")
        self.data_struct = types.SimpleNamespace()
        from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
        self.position_com_pub = self.create_publisher(
            PositionControllerReference,
            "/uav_0/fsc_autopilot_ros2/position_controller/reference", 10)

def spin(ros, tab, secs, app):
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(ros, timeout_sec=0.02)
        app.processEvents()

def main():
    rclpy.init()
    app = QtWidgets.QApplication([])
    ros = LiveRos()
    from ROS_Node.trajectory_player import TrajectoryPlayerTab
    tab = TrajectoryPlayerTab(ros)
    print("tab built against a live ROS node")

    print("\n[1] mocap reaching the tab")
    spin(ros, tab, 5.0, app)
    check("mocap received", tab._mocap_pos is not None,
          f"pos={None if tab._mocap_pos is None else np.round(tab._mocap_pos,2)}")
    if tab._mocap_pos is None:
        print("  !! no mocap — is the stack up? aborting"); return 1
    z0 = float(tab._mocap_pos[2])

    print("\n[2] px4_msgs wiring (ARM/OFFBOARD need it)")
    check("VehicleCommand publisher", tab._cmd_pub is not None)
    check("VehicleStatus subscription", tab._status_sub is not None)
    spin(ros, tab, 3.0, app)
    check("VehicleStatus arriving", tab._status is not None)

    print("\n[3] guards refuse before a controller is engaged")
    tab._controller = None
    tab._on_arm();      check("ARM refused with no controller", "engage a controller" in tab.lbl_veh.text())
    tab._on_offboard(); check("OFFBOARD refused with no controller", "engage a controller" in tab.lbl_veh.text())

    print("\n[4] TAKEOFF: offboard + arm + climb")
    tab.spin_pz.setValue(z0 + 12.0)
    tab._on_takeoff()
    spin(ros, tab, 30.0, app)
    z = float(tab._mocap_pos[2])
    check("armed", tab._armed() is True, f"arming_state={getattr(tab._status,'arming_state','?')}")
    check("climbed", z > z0 + 6.0, f"z {z0:.1f} -> {z:.1f} m")

    print("\n[5] wing-borne button refuses too low, accepts high enough")
    tab._on_wingborne()
    low_ok = "climb to" in tab.lbl_wb.text()
    check("refuses below 60 m", low_ok, tab.lbl_wb.text()[:60])
    tab._mocap_pos = np.array([tab._mocap_pos[0], tab._mocap_pos[1], 105.0])   # pretend at altitude
    tab._on_wingborne(); spin(ros, tab, 2.0, app)
    check("applies params + engages", tab._controller == "nmpc_full", tab.lbl_wb.text()[:70])

    print("\n[6] STOP from cruise runs the BACK-TRANSITION, not a handover")
    tab._mocap_vel = np.array([16.0, 0.0, 0.0])          # pretend wing-borne
    tab._on_stop()
    check("STOP -> back-transition", "back-transition" in tab.lbl_status.text().lower(),
          tab.lbl_status.text()[:70])
    tab._bt_timer.stop()

    print("\n[6b] REFERENCE: load every bundled trajectory, then play one")
    import os as _os
    tdir = tab._traj_dir()
    files = sorted(f for f in _os.listdir(tdir) if f.lower().endswith((".csv", ".npz")))
    check("dropdown populated", tab.combo_ref.count() > 0, f"{tab.combo_ref.count()} entries")
    for f in files:
        tab.traj = None
        tab._load_path(_os.path.join(tdir, f))
        ok = tab.traj is not None and len(getattr(tab.traj, "t", [])) > 1
        n = len(getattr(tab.traj, "t", [])) if tab.traj is not None else 0
        check(f"load {f}", ok, f"{n} samples")
    # play one on an NMPC controller (playing on `geometric` is refused by design)
    tab._load_path(_os.path.join(tdir, "cruise_fly_and_land.csv"))
    tab._mocap_pos = None; spin(ros, tab, 2.0, app)          # real pose back
    tab.chk_delta.setChecked(True)                             # anchor at the vehicle
    tab._set_controller("nmpc_full")
    n_before = len(getattr(ros.position_com_pub, "msgs", []) or [])
    tab._on_play()
    spin(ros, tab, 6.0, app)
    check("play advances the clock", tab._t_play > 0.5, f"t_play={tab._t_play:.1f} s")
    check("mode is PLAYING", tab._mode == "playing", f"mode={tab._mode}")
    tab._on_pause(); spin(ros, tab, 1.0, app)
    t_paused = tab._t_play
    spin(ros, tab, 2.0, app)
    check("pause freezes the clock", abs(tab._t_play - t_paused) < 0.2,
          f"{t_paused:.2f} -> {tab._t_play:.2f}")
    tab._on_stop(); spin(ros, tab, 1.0, app)
    check("stop resets the clock", tab._t_play == 0.0, f"t_play={tab._t_play}")

    print("\n[7] DISARM refuses while airborne and moving")
    tab._mocap_pos = np.array([0.0, 0.0, 40.0]); tab._mocap_vel = np.array([8.0, 0.0, 0.0])
    tab._on_disarm(); check("DISARM refused airborne", "refused" in tab.lbl_veh.text().lower(),
                            tab.lbl_veh.text()[:60])

    print("\n[8] LAND descends and disarms")
    tab._mocap_pos = None                                  # let real mocap take over again
    spin(ros, tab, 2.0, app)
    tab._on_land()
    spin(ros, tab, 60.0, app)
    z = float(tab._mocap_pos[2])
    check("descended", z < 1.5, f"z = {z:.2f} m")
    check("disarmed", tab._armed() is False, f"arming_state={getattr(tab._status,'arming_state','?')}")

    print("\n" + ("ALL LIVE GS CHECKS PASSED" if not FAIL else f"FAILED: {', '.join(FAIL)}"))
    ros.destroy_node(); rclpy.shutdown()
    return 1 if FAIL else 0

if __name__ == "__main__":
    sys.exit(main())
