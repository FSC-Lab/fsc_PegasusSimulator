#!/usr/bin/env python3
"""LIVE end-to-end test of the ground station's GUIDANCE row: the buttons fly the aircraft.

Builds the REAL Trajectory tab (offscreen) against a RUNNING HIL stack with a real rclpy node
and PRESSES THE BUTTONS an operator would, in order:

    ▲ TAKEOFF  ->  🚀 Run Full Auto Mission (ctrl = Cascade, alt 30)  ->  wait for CRUISE
    ->  ↱ Fly waypoints (the text box)  ->  legs + loiter on the last waypoint
    ->  🏠 Return to home  ->  the mission's RTH phase brakes, settles and LANDS AT HOME.

Until 2026-09-13 evening the ↱ button had never been pressed in flight: the same protocol string
had been flown from the scripted driver, and ◯ Loiter / 🏠 had been pressed. This presses it
(QPushButton.click() -> the clicked signal -> _on_guide_wps -> the String on cascade/cmd),
and checks the aircraft flew what the button said: the ack in the status feed, a loiter
reached, the RTH stop, the landing at home. The bag the launcher opened records it all.

Run after `VTOL_TANK=slosh VTOL_TANK_KG=0.502 bash scripts/start_slosh_hil.sh` (the launcher's
own GS window is closed first: two ground stations re-publishing active_controller would fight
at 1 Hz -- the same reason the scripted driver closes it):

    /usr/bin/python3 tests/test_gs_guidance_live.py [--wps "500 0 500 400 100 400"] [--loiter_s 60]

Exit 0 = every check passed. The bag is closed (SIGINT) at the end; score it with
scripts/ana_guided.py. The stack is left up: tear down with scripts/kill_slosh_hil.sh.
"""
import os, sys, time, math, subprocess
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/ros2_ground_station_gui/src"))
import numpy as np, rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets
import types

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SESSION = os.environ.get("HIL_SESSION", "vtol_smc_hil")
FAIL = []


def check(name, ok, detail=""):
    print(f"  {'OK  ' if ok else 'FAIL'}  {name}{('  — ' + detail) if detail else ''}", flush=True)
    if not ok:
        FAIL.append(name)


def arg(name, default):
    return sys.argv[sys.argv.index(name) + 1] if name in sys.argv else default


class LiveRos(Node):
    """Minimal stand-in for the GS ros object: real pub/sub, stubbed data_struct."""
    def __init__(self):
        super().__init__("gs_guidance_live_test")
        self.data_struct = types.SimpleNamespace()
        from fsc_autopilot_ros2_msgs.msg import PositionControllerReference
        self.position_com_pub = self.create_publisher(
            PositionControllerReference,
            "/uav_0/fsc_autopilot_ros2/position_controller/reference", 10)


def spin(ros, app, secs):
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(ros, timeout_sec=0.02)
        app.processEvents()


def spin_until(ros, app, pred, secs, every=5.0, say=None):
    """Spin until pred() is true or secs elapse; returns pred()'s final value."""
    t0 = time.time(); t_say = t0
    while time.time() - t0 < secs:
        rclpy.spin_once(ros, timeout_sec=0.02)
        app.processEvents()
        if pred():
            return True
        if say and time.time() - t_say > every:
            t_say = time.time(); print(f"      t+{time.time() - t0:4.0f} s: {say()}", flush=True)
    return pred()


def close_launcher_gs():
    """The launcher's GS window re-publishes its selection at 1 Hz forever; ONE ground station."""
    sh = os.path.join(os.environ.get("TMPDIR", "/tmp"), "gs_live_close_gs.sh")
    with open(sh, "w") as f:
        f.write("tmux kill-window -t %s:9-groundstn 2>/dev/null\n"
                "for p in $(pgrep -f single_drone_ground_control 2>/dev/null); do kill -9 $p 2>/dev/null; done\n"
                "exit 0\n" % SESSION)
    subprocess.run(["bash", sh], check=False)


def main():
    wps = arg("--wps", "500 0 500 400 100 400")
    loiter_s = float(arg("--loiter_s", "60"))
    alt = float(arg("--alt", "30"))

    print("[0] vehicle fit to fly?")
    vs = subprocess.run(["/usr/bin/python3", os.path.join(ROOT, "scripts", "vehicle_state.py")],
                        capture_output=True, text=True, timeout=90).stdout.strip().splitlines()
    vs = vs[-1] if vs else ""
    check("vehicle at the origin, upright, grounded, stopped", vs.startswith("ok"), vs)
    if not vs.startswith("ok"):
        print("  !! not flying a wreck -- restart the stack"); return 1
    close_launcher_gs()

    rclpy.init()
    app = QtWidgets.QApplication([])
    ros = LiveRos()
    from ROS_Node.trajectory_player import TrajectoryPlayerTab
    tab = TrajectoryPlayerTab(ros)
    print("tab built against a live ROS node (the launcher's GS window closed)")

    print("\n[1] mocap + PX4 status reaching the tab")
    spin(ros, app, 6.0)
    check("mocap received", tab._mocap_pos is not None,
          f"pos={None if tab._mocap_pos is None else np.round(tab._mocap_pos, 2)}")
    if tab._mocap_pos is None:
        print("  !! no mocap — is the stack up? aborting"); return 1
    check("VehicleStatus arriving", tab._status is not None)
    z0 = float(tab._mocap_pos[2]); home = np.array(tab._mocap_pos[:2], float)

    print(f"\n[2] ▲ TAKEOFF to {alt:.0f} m (offboard + arm + climb on Safe Hover)")
    tab.spin_pz.setValue(alt)
    tab.btn_takeoff.click() if hasattr(tab, "btn_takeoff") else tab._on_takeoff()
    ok = spin_until(ros, app, lambda: tab._armed() is True and float(tab._mocap_pos[2]) > alt - 2.0, 90.0,
                    say=lambda: f"z={float(tab._mocap_pos[2]):.1f} armed={tab._armed()}")
    check("armed and at altitude", ok, f"z {z0:.1f} -> {float(tab._mocap_pos[2]):.1f} m, armed={tab._armed()}")
    if not ok:
        return 1
    spin(ros, app, 5.0)

    print("\n[3] 🚀 Run Full Auto Mission with ctrl = Cascade")
    i = tab.cmb_wb.findData("cascade"); tab.cmb_wb.setCurrentIndex(i)
    check("mission controller = cascade", tab._wb_choice == "cascade", tab._wb_choice)
    tab.spin_fm_alt.setValue(alt); tab.spin_fm_dist.setValue(1000.0)
    tab.btn_fm.click()
    ok = spin_until(ros, app, lambda: getattr(tab, "_fm", None) is not None
                    and tab._fm.get("phase") == "cruise", 120.0,
                    say=lambda: tab.lbl_fm.text()[:90])
    check("mission reached CRUISE on the cascade", ok, tab.lbl_fm.text()[:90])
    if not ok:
        return 1
    V = lambda: float(np.linalg.norm(getattr(tab, "_mocap_vel", np.zeros(3))[:2]))
    ok = spin_until(ros, app, lambda: V() > 15.0, 60.0, say=lambda: f"V={V():.1f} {tab.lbl_fm.text()[:60]}")
    check("wing-borne (V > 15)", ok, f"V={V():.1f}")
    spin(ros, app, 5.0)

    print(f"\n[4] ↱ Fly waypoints: '{wps}'")
    tab.edit_wps.setText(wps)
    tab.btn_wps.click()
    check("button sent the wp command", tab.lbl_guide.text().startswith("sent: wp"), tab.lbl_guide.text()[:80])
    check("mission marked guided", bool(tab._fm.get("guided")), str(tab._fm.get("guided")))
    src = getattr(tab, "_hud_src", None)
    st = lambda: (getattr(src, "casc_status", "") or "") if src is not None else ""
    ok = spin_until(ros, app, lambda: "mode=leg" in st(), 15.0, say=lambda: st()[:80])
    check("node acked: mode=leg in the status feed", ok, st()[:80])
    # the legs end in a loiter on the last waypoint
    ok = spin_until(ros, app, lambda: "mode=loiter" in st(), 150.0, every=10.0,
                    say=lambda: f"pos=({tab._mocap_pos[0]:.0f},{tab._mocap_pos[1]:.0f}) V={V():.1f} {st()[:60]}")
    check("legs flown, loiter reached", ok, st()[:80])
    z_lo = [] ; rho = []
    last = wps.split()[-2:]; cx, cy = float(last[0]), float(last[1])
    t0 = time.time()
    while time.time() - t0 < loiter_s:
        spin(ros, app, 1.0)
        z_lo.append(float(tab._mocap_pos[2])); rho.append(math.hypot(tab._mocap_pos[0] - cx, tab._mocap_pos[1] - cy))
    check(f"loiter held {loiter_s:.0f} s at altitude", min(z_lo) > alt - 3.0 and max(z_lo) < alt + 3.0,
          f"z {min(z_lo):.1f}-{max(z_lo):.1f}; rho last 30 s {np.mean(rho[-30:]):.1f} +- {np.std(rho[-30:]):.1f} m")

    print("\n[5] 🏠 Return to home")
    tab.btn_rth.click()
    check("rth sent, mission in RTH phase", tab._fm is not None and tab._fm.get("phase") == "rth",
          tab.lbl_fm.text()[:80])
    ok = spin_until(ros, app, lambda: tab._fm is None, 400.0, every=10.0,
                    say=lambda: f"d_home={np.linalg.norm(np.array(tab._mocap_pos[:2]) - home):.0f} V={V():.1f} {tab.lbl_fm.text()[:60]}")
    txt = tab.lbl_fm.text()
    check("mission ended", ok, txt[:90])
    # the forced disarm goes out in the same tick the label changes; VehicleStatus takes a
    # moment to say so (flight 5, 2026-09-13: read as armed=True, PX4 was disarmed 2 s later)
    disarmed = spin_until(ros, app, lambda: tab._armed() is False, 10.0)
    check("LANDED (disarmed)", "LANDED" in txt and disarmed, f"armed={tab._armed()}")
    err = float(np.linalg.norm(np.array(tab._mocap_pos[:2]) - home))
    check("landed at home", err < 2.0, f"{err:.2f} m from the takeoff point, z={float(tab._mocap_pos[2]):.2f}")

    # close the launcher's bag (SIGINT; a -9'd bag has no metadata.yaml)
    subprocess.run(["tmux", "send-keys", "-t", f"{SESSION}:10-bag", "C-c"], check=False)
    time.sleep(5)
    print("\n" + ("ALL LIVE GUIDANCE CHECKS PASSED" if not FAIL else f"FAILED: {', '.join(FAIL)}"))
    ros.destroy_node(); rclpy.shutdown()
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
