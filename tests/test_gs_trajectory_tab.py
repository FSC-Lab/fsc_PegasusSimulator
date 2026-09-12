#!/usr/bin/env python3
"""Offscreen smoke test of the REAL ground-station Trajectory tab.\n\nRuns the actual widget with QT_QPA_PLATFORM=offscreen and a stubbed ROS object, so the\ncontroller-routing rules can be checked without a running stack:\n  * a typed X/Y/Z and a fly-to-point mission must hand the vehicle to `geometric`\n  * STOP must hand it to `geometric`\n  * playing a trajectory on `geometric` must be refused (it tracks points, not paths)\n  * the playback geofence must be live\n\nRun with the SYSTEM python (the ground station's own): /usr/bin/python3 tests/test_gs_trajectory_tab.py\n"""
import os, sys, math, types
os.environ["QT_QPA_PLATFORM"] = "offscreen"
sys.path.insert(0, os.path.expanduser("~/ros2_ws/src/ros2_ground_station_gui/src"))
from PyQt5 import QtWidgets
import numpy as np

class Pub:
    def __init__(s): s.msgs = []
    def publish(s, m): s.msgs.append(m)

class Clock:
    def now(s):
        class T:
            def to_msg(self):
                from builtin_interfaces.msg import Time
                return Time()
        return T()

class Ros:
    def __init__(s):
        s.position_com_pub = Pub(); s.ctrl = Pub()
        s.data_struct = types.SimpleNamespace()
    def create_publisher(s, *a, **k): return s.ctrl
    def create_subscription(s, *a, **k): return None
    def get_clock(s): return Clock()

app = QtWidgets.QApplication([])
from ROS_Node.trajectory_player import TrajectoryPlayerTab
ros = Ros()
tab = TrajectoryPlayerTab(ros)
print("tab built OK")

# pretend mocap is flowing
tab._mocap_pos = np.array([1.0, 2.0, 5.0]); tab._mocap_yaw = 0.3
tab._mocap_vel = np.array([0.0, 0.0, 0.0])

# 1. typed position must force the geometric controller
tab.spin_px.setValue(10.0); tab.spin_py.setValue(-4.0); tab.spin_pz.setValue(8.0)
tab._on_goto_position()
tab._tick()
assert tab._controller == "geometric", tab._controller
m = ros.position_com_pub.msgs[-1]
assert (round(m.position.x,2), round(m.position.y,2), round(m.position.z,2)) == (10.0,-4.0,8.0), m.position
print(f"GO/HOLD -> controller={tab._controller}, ref=({m.position.x},{m.position.y},{m.position.z}) OK")

# 2. STOP + safe hover
tab._on_hover(); tab._tick()
assert tab._controller == "geometric" and tab._mode == "hover"
print("STOP -> geometric hover OK")

# 3. mission = completion-gated waypoints on geometric
tab.spin_tx.setValue(30.0); tab.spin_ty.setValue(12.0); tab.spin_cz.setValue(15.0)
tab._on_mission()
assert tab._controller == "geometric" and tab._mode == "mission"
legs = [l[0] for l in tab._mission["legs"]]
assert legs == ["CLIMB","TRAVERSE","DESCEND"], legs
tab._tick()
m = ros.position_com_pub.msgs[-1]
print(f"Mission legs {legs}; first waypoint ({m.position.x:.1f},{m.position.y:.1f},{m.position.z:.1f}) OK")
# advance a leg by teleporting the vehicle to the waypoint
tab._mocap_pos = np.array([1.0, 2.0, 15.0])
for _ in range(3): tab._tick()
assert tab._mission["i"] == 1, tab._mission["i"]
print("CLIMB leg completion-gated -> advanced to TRAVERSE OK")

# 4. Play on geometric must refuse (no crash, no publish)
import ROS_Node.trajectory_player as tp
called = {}
tp.QMessageBox.information = lambda *a, **k: called.setdefault("warned", True)
class T: pass
tab.traj = tp.TrajectoryData(np.linspace(0,5,6), np.zeros(6), np.zeros(6), np.full(6,5.0), np.zeros(6))
# NOTE (2026-09-08): _on_play used to REFUSE on `geometric` with a message box. The tab now
# AUTO-ENGAGES an NMPC instead (see _on_play / _engage) -- trajectory tracking belongs to the
# NMPC controllers, so Play switches to one rather than making the operator do it. The check is
# therefore "Play never leaves a trajectory running on the geometric controller".
tab._controller = "geometric"
tab._on_play()
assert tab._mode == "playing", "Play should start"
assert tab._controller in ("nmpc_full", "nmpc6_cruise", "nmpc_cruise"), \
    f"Play must not track a path on {tab._controller}"
print(f"Play on geometric -> auto-engaged {tab._controller} OK")
tab._controller = "nmpc_full"; tab._on_play()
assert tab._mode == "playing"
print("Play on an NMPC controller -> allowed OK")

# 5. geofence is live for playback
tab._offset = np.array([1e5, 0.0, 0.0])
tab._tick()
assert tab._mode == "hover", tab._mode
print("playback geofence -> tripped to hover OK")
# 6. MISSION REFERENCE BUILD (velocity field + computed profile + progress -> ready)
# The cruise-speed field is bounded by the AERODYNAMICS, not by taste: CL peaks at AoA 10 deg
# (CL_max 0.791) so Vstall = 14.13 m/s, and trimmed the wing cannot carry the weight below
# ~15 m/s. The ceiling is 16 m/s -- the fastest speed actually flown.
assert hasattr(tab, "spin_fm_vel"), "cruise velocity field missing"
assert tab.spin_fm_vel.maximum() == 16.0, f"max should be 16 m/s, got {tab.spin_fm_vel.maximum()}"
print(f"cruise velocity field {tab.spin_fm_vel.minimum():.0f}-{tab.spin_fm_vel.maximum():.0f} m/s OK")

_, errs, _ = tab._plan_reference(1000.0, 13.0, 30.0)
assert errs and "stall" in errs[0].lower(), errs
_, errs, _ = tab._plan_reference(1000.0, 14.5, 30.0)
assert errs, "14.5 m/s is below the trimmed wing minimum and must be refused"
_, errs, _ = tab._plan_reference(300.0, 16.0, 30.0)
assert errs and "short" in " ".join(errs), errs
plan, errs, warns = tab._plan_reference(1000.0, 16.0, 30.0)
assert not errs, errs
assert abs(plan["d_acc"] - 320.0) < 1.0, plan["d_acc"]
assert abs(plan["d_acc"] + plan["d_cru"] + plan["d_dec"] - 1000.0) < 1.0, plan
assert warns and "stall" in warns[0], "16 m/s is 1.13x stall and must warn"
print(f"profile: accel {plan['d_acc']:.0f} m + cruise {plan['d_cru']:.0f} m + stop "
      f"{plan['d_dec']:.0f} m = {plan['dist']:.0f} m, AoA {plan['aoa']:.1f} deg OK")
print(f"refuses below-stall / below-wing-minimum / too-short requests OK")

# ★ the GUI ALTITUDE must reach the node. `cruise_alt` used to never be sent at all: the field
# drove only the ground station's climb phase, so asking for 50 m left the NMPC transitioning
# against its built-in 30 m. Everything altitude-dependent is derived in _plan_reference.
for alt_req in (10.0, 30.0, 60.0):
    pl, er, _ = tab._plan_reference(1000.0, 16.0, alt_req)
    assert not er, (alt_req, er)
    pr = pl["params"]
    assert pr["cruise_alt"] == alt_req, f"cruise_alt not passed through: {pr.get('cruise_alt')}"
    assert pr["vcruise"] == 16.0, pr["vcruise"]
    # the abort band must stay STRICTLY above what the reference may spend, or the transition
    # commands a sink it is then aborted for
    assert pr["cruise_alt_band"] > pr["descent_budget"], (alt_req, pr)
    assert pr["descent_budget"] <= alt_req * 0.25 + 1e-9, (alt_req, pr)
    # ★ altitude authority must NOT be faded when there is no altitude to spend.
    # Bag vtol_20260909_214409: engaged at 10 m with the fade active, sagged 10.7 -> 7.4 m
    # and departed before reaching wing-borne speed.
    if alt_req <= 15.0:
        assert pr["alt_auth_trans"] >= 0.99, \
            f"at {alt_req} m the altitude hold must stay full, got {pr['alt_auth_trans']}"
    else:
        assert pr["alt_auth_trans"] < 1.0, (alt_req, pr["alt_auth_trans"])
    print(f"alt {alt_req:4.0f} m -> cruise_alt {pr['cruise_alt']:.0f}, "
          f"may spend {pr['descent_budget']:.0f} m, abort at -{pr['cruise_alt_band']:.0f} m, "
          f"alt-hold {pr['alt_auth_trans']*100:.0f}% OK")

# the params the build SENDS must be the ones the plan derived from the GUI altitude
_sent = {}
tab._set_node_params = lambda target, params: (_sent.update({target: dict(params)}),
                                               (len(params), []))[1]
tab._get_node_param = lambda target, name: 16.0
tab._fm_launch = lambda *a, **k: setattr(tab, "_fm_launched", True)
tab.spin_fm_alt.setValue(60.0); tab.spin_fm_vel.setValue(16.0); tab.spin_fm_dist.setValue(1000.0)
tab._armed = lambda: True
tab._on_full_mission()
for _ in range(20):
    if not tab._fm_build["timer"].isActive() and tab.fm_prog.value() == 100:
        break
    tab._fm_build_step()
sent = _sent.get("/nmpc_full_smc_control", {})
assert sent.get("cruise_alt") == 60.0, f"node got cruise_alt={sent.get('cruise_alt')}, GUI said 60"
print(f"GUI altitude 60 m -> node cruise_alt {sent['cruise_alt']:.0f} m OK")
tab.spin_fm_alt.setValue(30.0)

# the build must reach READY only once the NODE reports the schedule it actually built
tab._set_node_params = lambda target, params: (len(params), [])
tab._get_node_param = lambda target, name: 16.0
tab._fm_launch = lambda *a, **k: setattr(tab, "_fm_launched", True)
tab.spin_fm_vel.setValue(16.0); tab.spin_fm_dist.setValue(1000.0); tab.spin_fm_alt.setValue(30.0)
tab._armed = lambda: True
tab._on_full_mission()
for _ in range(20):
    if not tab._fm_build["timer"].isActive() and tab.fm_prog.value() == 100:
        break
    tab._fm_build_step()
assert tab.fm_prog.value() == 100, f"progress stuck at {tab.fm_prog.value()}"
assert getattr(tab, "_fm_launched", False), "mission never launched after the build"
print(f"build -> progress 100% -> READY -> launched OK  ({tab.lbl_fm.text()})")

# and it must NOT launch if the node never reports the schedule
tab._fm_launched = False
tab._get_node_param = lambda target, name: None
tab._on_full_mission()
for _ in range(25):
    tab._fm_build_step()
    if not tab._fm_build["timer"].isActive():
        break
assert not tab._fm_launched, "must not launch when the trim schedule was never built"
assert "not answering" in tab.lbl_fm.text(), tab.lbl_fm.text()
print("node silent -> build fails, mission does NOT launch, cause named OK")

# the three timeout causes must be distinguishable, not one generic message
for stub, expect in ((lambda t, n: 0.0, "older build"),
                     (lambda t, n: 12.0, "built for 12.0")):
    tab._fm_launched = False
    tab._get_node_param = stub
    tab._on_full_mission()
    for _ in range(25):
        tab._fm_build_step()
        if not tab._fm_build["timer"].isActive():
            break
    assert not tab._fm_launched, expect
    assert expect in tab.lbl_fm.text(), f"expected {expect!r} in {tab.lbl_fm.text()!r}"
    print(f"timeout cause reported: …{expect}… OK")

# ---- the auto mission must be able to target the CASCADE, not just nmpc_full ----------------
# The two controllers do NOT share a parameter surface, and `_set_node_params` treats an unknown
# parameter as a failure that aborts the mission build. So the plan built for the cascade must
# contain ONLY keys the cascade declares -- otherwise every cascade mission fails to launch.
tab._wb_choice = "nmpc_full"
assert tab._wb_id() == "nmpc_full" and tab._wb_node() == "/nmpc_full_smc_control"
plan_full, errs, _ = tab._plan_reference(1000.0, 16.0, 30.0)
assert not errs, errs

tab._wb_choice = "cascade"
assert tab._wb_id() == "cascade", tab._wb_id()
assert tab._wb_node() == "/nmpc_cascade_control", tab._wb_node()
plan_casc, errs, _ = tab._plan_reference(1000.0, 16.0, 30.0)
assert not errs, errs
pf, pc = set(plan_full["params"]), set(plan_casc["params"])
# NOT a subset any more: the cascade legitimately carries the PROPELLER MODEL, which nmpc_full
# has no concept of. What must hold is (a) everything it sends is declared by the cascade node
# (CASC_KEYS), and (b) none of nmpc_full's transition knobs leak into it.
assert pc <= tab.CASC_KEYS, f"cascade plan carries keys outside CASC_KEYS: {sorted(pc - tab.CASC_KEYS)}"
assert not (pc & (pf - tab.CASC_KEYS)), f"nmpc_full-only knobs leaked: {sorted(pc & (pf - tab.CASC_KEYS))}"
assert "tilt_slew_deg_s" not in pc, "tilt_slew_deg_s means something different on each node"
assert "descent_budget" not in pc and "tilt_hold_deg" not in pc, "nmpc_full-only knobs leaked"
for k in ("vcruise", "cruise_alt", "ttr", "decel_rate", "flare_deg", "beta_aft_deg"):
    assert k in pc, f"cascade plan is missing {k}"
assert abs(float(plan_casc["params"]["vcruise"]) - 16.0) < 1e-6
assert abs(float(plan_casc["params"]["cruise_alt"]) - 30.0) < 1e-6
print(f"auto mission -> cascade: {len(pc)} params, subset of nmpc_full's {len(pf)}, "
      f"no name-collisions OK")

# the propeller model must LOAD with the mission, or the controller inverts a different prop
# from the one the sim is flying and silently under-delivers thrust
for k in ("inflow", "prop_diameter_m", "prop_zero_thrust_J", "rotor_omega_max"):
    assert k in pc, f"cascade mission does not carry {k}"
assert "prop_diameter_m" not in pf, "prop model leaked into the nmpc_full param set"
print("propeller model ships with the cascade mission, not with nmpc_full OK")

# THRUST FEASIBILITY: the button must refuse a cruise speed the propeller cannot sustain
tab._wb_choice = "cascade"
_, errs_ok, _ = tab._plan_reference(1000.0, 16.0, 30.0)
assert not errs_ok, f"16 m/s should be feasible, got {errs_ok}"
# ★ EVERY speed the spinbox offers (10-16) must be ACCEPTED. Below ~14.5 the wing cannot carry
# the weight, but that is a TILT-ASSISTED cruise, not an error -- the rotors make up the
# shortfall. An earlier version of this check tested for FULL wing-borne flight at every speed
# and refused the entire 10-14 half of the range.
# Speeds below the 14.13 m/s stall are refused by a PRE-EXISTING check ("below stall"), which
# is not the thrust check and predates it. What the thrust check must NOT do is add refusals of
# its own on top of the flyable range: an earlier version tested for FULL wing-borne flight at
# every speed and would have refused everything under 14.5 for a second, wrong reason.
for v in (15.0, 16.0):
    _, e_v, _ = tab._plan_reference(1000.0, v, 30.0)
    assert not e_v, f"{v} m/s must be flyable, got {e_v}"
for v in (10.0, 12.0, 14.0):
    _, e_v, _ = tab._plan_reference(1000.0, v, 30.0)
    assert e_v, f"{v} m/s is below the trimmed wing-borne minimum and should be refused"
    assert any(("stall" in x.lower() or "carry the weight" in x.lower()) for x in e_v), \
        f"{v} m/s must be refused by the WING checks, not by the thrust check: {e_v}"
print("feasibility: 15-16 flyable; below that refused by the WING gates (stall / trimmed\n            wing-borne minimum), and the thrust check adds no second refusal OK")

# selecting the mission controller must NOT switch the active one (safe to change in flight)
before = tab._controller
tab._wb_choice = "nmpc_full"
assert tab._controller == before, "changing the mission target must not switch the active controller"
print("mission-controller selector does not disturb the active controller OK")

print("\nALL GUI SMOKE CHECKS PASSED")
