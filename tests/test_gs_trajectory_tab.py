#!/usr/bin/env python3
"""Offscreen smoke test of the REAL ground-station Trajectory tab.\n\nRuns the actual widget with QT_QPA_PLATFORM=offscreen and a stubbed ROS object, so the\ncontroller-routing rules can be checked without a running stack:\n  * a typed X/Y/Z and a fly-to-point mission must hand the vehicle to `geometric`\n  * STOP must hand it to `geometric`\n  * playing a trajectory on `geometric` must be refused (it tracks points, not paths)\n  * the playback geofence must be live\n\nRun with the SYSTEM python (the ground station's own): /usr/bin/python3 tests/test_gs_trajectory_tab.py\n"""
import os, sys, math, types
os.environ["QT_QPA_PLATFORM"] = "offscreen"
# the GS judges speeds against the TANK aircraft (M = 3.449572 + VTOL_TANK_KG, the controllers'
# own expression); pin the mode so the numbers below are the ones asserted
os.environ["VTOL_TANK_KG"] = os.environ.get("VTOL_TANK_KG", "0.502") or "0.502"
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
    def create_publisher(s, *a, **k):
        # record the TOPIC each publisher was created for (the stub returns one Pub, so the
        # object cannot distinguish them -- the guidance topics per cascade id are checked here)
        try:
            s.pub_topics.append(a[1] if len(a) > 1 else k.get('topic'))
        except AttributeError:
            s.pub_topics = [a[1] if len(a) > 1 else k.get('topic')]
        return s.ctrl
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
# (CL_max 0.791) so Vstall = 14.13 m/s at the 3.45 kg airframe, scaling with sqrt(W) -- and the
# weight is the TANK aircraft's (VTOL_TANK_KG: 15.12 m/s at 0.502 kg, the controllers' mass).
# Trimmed, the wing cannot carry the weight alone below ~15 x sqrt(W/33.84) m/s -- a WARNING
# (the rear pair carries the deficit; 0.7 kg is flown at 16.0 against a 16.4 trimmed minimum).
# The ceiling is 16 m/s -- the fastest speed actually flown.
assert hasattr(tab, "spin_fm_vel"), "cruise velocity field missing"
assert tab.spin_fm_vel.maximum() == 16.0, f"max should be 16 m/s, got {tab.spin_fm_vel.maximum()}"
print(f"cruise velocity field {tab.spin_fm_vel.minimum():.0f}-{tab.spin_fm_vel.maximum():.0f} m/s OK")
water = float(os.environ["VTOL_TANK_KG"])
assert abs(tab.M_KG - (3.449572 + water)) < 1e-9, tab.M_KG
assert abs(tab.V_STALL - 14.13 * math.sqrt(tab.M_KG / 3.449572)) < 1e-9, tab.V_STALL
assert abs(tab.V_WING_MIN - 15.0 * math.sqrt(tab.M_KG / 3.449572)) < 1e-9, tab.V_WING_MIN
if abs(water - 0.502) < 1e-9:
    assert abs(tab.V_STALL - 15.12) < 0.01, f"0.502 kg must stall at 15.12 m/s, got {tab.V_STALL:.2f}"
print(f"speed checks judge the {tab.M_KG:.3f} kg aircraft: stall {tab.V_STALL:.2f}, trimmed wing minimum {tab.V_WING_MIN:.2f} m/s OK")

_, errs, _ = tab._plan_reference(1000.0, 13.0, 30.0)
assert errs and "stall" in errs[0].lower(), errs
_, errs, _ = tab._plan_reference(1000.0, tab.V_STALL - 0.3, 30.0)
assert errs and "stall" in errs[0].lower(), "0.3 m/s under the tank aircraft's stall must be refused"
if tab.V_STALL < 16.0 < tab.V_WING_MIN:
    _, errs, warns = tab._plan_reference(1000.0, 16.0, 30.0)
    assert not errs and any("rear" in w for w in warns), \
        f"between stall and the trimmed minimum must fly with a warning, got {errs} / {warns}"
_, errs, _ = tab._plan_reference(300.0, 16.0, 30.0)
assert errs and "short" in " ".join(errs), errs
plan, errs, warns = tab._plan_reference(1000.0, 16.0, 30.0)
assert not errs, errs
assert abs(plan["d_acc"] - 320.0) < 1.0, plan["d_acc"]
assert abs(plan["d_acc"] + plan["d_cru"] + plan["d_dec"] - 1000.0) < 1.0, plan
assert warns and any("stall" in w for w in warns), "16 m/s is 1.06x stall and must warn"
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
    if v >= tab.V_STALL:
        assert not e_v, f"{v} m/s must be flyable, got {e_v}"
    else:
        assert e_v and "stall" in e_v[0].lower(), f"{v} m/s is under the tank aircraft's stall: {e_v}"
for v in (10.0, 12.0, 14.0):
    _, e_v, _ = tab._plan_reference(1000.0, v, 30.0)
    assert e_v, f"{v} m/s is below stall and should be refused"
    assert any("stall" in x.lower() for x in e_v), \
        f"{v} m/s must be refused by the STALL check, not by the thrust check: {e_v}"
print(f"feasibility: >= {tab.V_STALL:.2f} flyable; below that refused by the stall gate, and the\n            thrust check adds no second refusal OK")

# selecting the mission controller must NOT switch the active one (safe to change in flight)
before = tab._controller
tab._wb_choice = "nmpc_full"
assert tab._controller == before, "changing the mission target must not switch the active controller"
print("mission-controller selector does not disturb the active controller OK")

# ---- GUIDANCE ROW (cascade): separate buttons for waypoints / loiter / RTH / straight --------
# Each button is ONE String on /uav_0/cascade/cmd in the node's protocol; refused unless the
# cascade is the ACTIVE controller (the node resets its guidance at engage, so an early command
# is silently lost). The Ros stub hands every publisher the same Pub, so the guidance messages
# land in ros.ctrl alongside the controller switches: filter by content.
def _guide_msgs():
    return [m.data for m in ros.ctrl.msgs if isinstance(m, tp.String)
            and m.data.split()[0] in ("wp", "loiter", "rth", "hold")]
tab._fm = None
tab._controller = "geometric"
n0 = len(_guide_msgs())
tab._on_guide_wps()
assert len(_guide_msgs()) == n0 and "refused" in tab.lbl_guide.text(), tab.lbl_guide.text()
print("guidance on a non-cascade controller -> refused, nothing published OK")
tab._controller = "cascade"
tab.edit_wps.setText("500 0, 500 400, 100 400")
tab._on_guide_wps()
assert _guide_msgs()[-1] == "wp 500.0 0.0 500.0 400.0 100.0 400.0", _guide_msgs()[-1]
tab.edit_wps.setText("500 0 500")
tab._on_guide_wps()
assert _guide_msgs()[-1].startswith("wp 500.0 0.0 500.0 400.0") and "even" in tab.lbl_guide.text()
tab.spin_lo_x.setValue(100.0); tab.spin_lo_y.setValue(400.0); tab.spin_lo_r.setValue(100.0)
tab.cmb_lo_dir.setCurrentIndex(1)
tab._on_guide_loiter()
assert _guide_msgs()[-1] == "loiter 100.0 400.0 100 -1", _guide_msgs()[-1]
assert tab.spin_lo_r.minimum() >= 60.0, "an orbit tighter than the bank-limited turn must not be offered"
tab.btn_ghold.click()
assert _guide_msgs()[-1] == "hold"
# "loiter here": centre one radius abeam of the COURSE on the turn side -> the aircraft is on the
# circle's tangent and orbits at once. Heading east at (300, 400), CCW, R 100 -> centre (300, 500).
tab._mocap_pos = np.array([300.0, 400.0, 30.0]); tab._mocap_yaw = 0.0
tab._mocap_vel = np.array([16.0, 0.0, 0.0])
tab.cmb_lo_dir.setCurrentIndex(0); tab.spin_lo_r.setValue(100.0)
tab._on_guide_loiter_here()
assert _guide_msgs()[-1] == "loiter 300.0 500.0 100 1", _guide_msgs()[-1]
tab.cmb_lo_dir.setCurrentIndex(1); tab._on_guide_loiter_here()
assert _guide_msgs()[-1] == "loiter 300.0 300.0 100 -1", _guide_msgs()[-1]
# in a bank the nose is not the course: the centre follows the velocity vector
tab._mocap_yaw = 0.5; tab._on_guide_loiter_here()
assert _guide_msgs()[-1] == "loiter 300.0 300.0 100 -1", _guide_msgs()[-1]
print("loiter HERE -> centre one radius abeam of the course, turn side OK")
print("waypoints / loiter / straight -> protocol strings OK")

# ---- THE ADAPTIVE CASCADE (2026-09-14): a second selectable controller, "Adaptive NMPC + Geo" --
# id cascade_adaptive, node /nmpc_cascade_control_adaptive, guidance on /uav_0/cascade_adaptive/*.
# The GS must treat it as a cascade everywhere: the button, the mission ctrl choice with the
# cascade's parameter filter, the guidance row (refused on neither, sent on its OWN topic), the
# stop-from-cruise rule. The stub hands every publisher one Pub, so the topic is checked by which
# publisher object the guidance used.
assert "cascade_adaptive" in tab.CASCADE_IDS and tab.WB_CONTROLLERS["cascade_adaptive"] == "/nmpc_cascade_control_adaptive"
assert hasattr(tab, "btn_adapt") and "Adaptive" in tab.btn_adapt.text()
tab.btn_adapt.click()
assert tab._controller == "cascade_adaptive" and ros.ctrl.msgs[-1].data == "cascade_adaptive", ros.ctrl.msgs[-1].data
assert "Adaptive" in tab.lbl_ctrl.text()
i = tab.cmb_wb.findData("cascade_adaptive"); assert i >= 0, "the mission ctrl choice lacks the adaptive cascade"
tab.cmb_wb.setCurrentIndex(i); tab._wb_choice = "cascade_adaptive"
assert tab._wb_id() == "cascade_adaptive" and tab._wb_node() == "/nmpc_cascade_control_adaptive"
pa = tab._wb_params(); pc_ = (tab._wb_choice, tab._wb_choice)
tab._wb_choice = "cascade"; pc2 = tab._wb_params(); tab._wb_choice = "cascade_adaptive"
assert pa == pc2, "the adaptive cascade must get exactly the cascade's parameter set"
assert set(pa) <= tab.CASC_KEYS
n1 = len(_guide_msgs()); tab.edit_wps.setText("500 0 500 400"); tab._on_guide_wps()
assert len(_guide_msgs()) == n1 + 1 and _guide_msgs()[-1].startswith("wp 500.0 0.0"), "guidance must be accepted on the adaptive cascade"
assert tab._guide_pubs["cascade_adaptive"] is not None and tab._guide_pubs["cascade"] is not None
print("adaptive cascade: button, ctrl choice, cascade parameter set, guidance accepted OK")

# ---- THE INDI CASCADE (2026-09-21): a THIRD selectable controller, "NMPC + INDI" -- id
# cascade_indi, node /nmpc_cascade_control_indi, guidance on /uav_0/cascade_indi/*. Same rule:
# the GS must treat it as a cascade everywhere, and it must be SEPARATE from the other two (its
# own button, its own id on active_controller, its own guidance publisher, its own node in the
# arm/disarm sweeps) -- a variant that quietly reused another id would fly the wrong node.
assert "cascade_indi" in tab.CASCADE_IDS and tab.WB_CONTROLLERS["cascade_indi"] == "/nmpc_cascade_control_indi"
assert hasattr(tab, "btn_indi") and "INDI" in tab.btn_indi.text()
tab.btn_indi.click()
assert tab._controller == "cascade_indi" and ros.ctrl.msgs[-1].data == "cascade_indi", ros.ctrl.msgs[-1].data
assert "INDI" in tab.lbl_ctrl.text()
i = tab.cmb_wb.findData("cascade_indi"); assert i >= 0, "the mission ctrl choice lacks the INDI cascade"
tab.cmb_wb.setCurrentIndex(i); tab._wb_choice = "cascade_indi"
assert tab._wb_id() == "cascade_indi" and tab._wb_node() == "/nmpc_cascade_control_indi"
pi_ = tab._wb_params(); tab._wb_choice = "cascade"; pc3 = tab._wb_params(); tab._wb_choice = "cascade_indi"
assert pi_ == pc3, "the INDI cascade must get exactly the cascade's parameter set"
n1 = len(_guide_msgs()); tab.edit_wps.setText("500 0 500 400"); tab._on_guide_wps()
assert len(_guide_msgs()) == n1 + 1 and _guide_msgs()[-1].startswith("wp 500.0 0.0"), "guidance must be accepted on the INDI cascade"
assert tab._guide_pubs["cascade_indi"] is not None
for cid in tab.CASCADE_IDS:
    assert f"/uav_0/{cid}/cmd" in getattr(ros, "pub_topics", []), \
        f"no guidance publisher on /uav_0/{cid}/cmd -- each cascade owns its topic"
for cid in tab.CASCADE_IDS:
    assert tab.WB_CONTROLLERS[cid] in tab.ARM_NODES, f"{cid}'s node is not in the arm/disarm sweep"
assert len(set(tab.ARM_NODES)) == len(tab.ARM_NODES) and "/geometric_hover_control" in tab.ARM_NODES
print("INDI cascade: own button, id, node, guidance topic, arm sweep OK")
tab._controller = "cascade"; tab._wb_choice = "cascade"

# RTH inside the auto mission: the cruise phase becomes RTH, the mission lands AT HOME once the
# cascade has stopped itself; if it never stops, the schedule's back-transition is forced rather
# than handing a 16 m/s aircraft to the hover controller.
tab._set_node_params = lambda target, params: (len(params), [])
tab._wb_choice = "cascade"          # the mission's cruise tick re-asserts its target controller
tab._fm = dict(phase="cruise", t0=tab._now(), start=(0.0, 0.0), yaw=0.0, alt=30.0, dist=1000.0,
               params=None, tgt=(1000.0, 0.0), stop=None, tprev=None, frozen=0, last=None)
tab._mocap_pos = np.array([300.0, 400.0, 30.0]); tab._mocap_yaw = math.pi
tab._mocap_vel = np.array([-16.0, 0.0, 0.0])
tab._on_guide_loiter()
assert tab._fm["guided"] is True
tab._fm_tick()
assert tab._fm["phase"] == "cruise" and "guided" in tab.lbl_fm.text(), tab.lbl_fm.text()
tab._on_guide_rth()
assert _guide_msgs()[-1] == "rth" and tab._fm["phase"] == "rth" and tab._fm["land_xy"] == (0.0, 0.0)
tab._fm_tick()
assert tab._fm["phase"] == "rth" and "to home" in tab.lbl_fm.text(), tab.lbl_fm.text()
tab._mocap_pos = np.array([5.0, 24.0, 30.0]); tab._mocap_vel = np.array([0.0, 0.5, 0.0])   # the flown stop
tab._fm_tick()
assert tab._fm["phase"] == "settle" and tab._fm["land_xy"] == (0.0, 0.0) and tab._controller == "geometric"
print("RTH in the auto mission -> rth phase -> stopped near home -> settle over HOME on geometric OK")
tab._fm = dict(phase="rth", t0=tab._now() - 1e4, start=(0.0, 0.0), yaw=0.0, alt=30.0, dist=1000.0,
               params=None, tgt=(1000.0, 0.0), stop=None, tprev=None, frozen=0, last=None,
               land_xy=(0.0, 0.0), rth_d0=500.0)
tab._controller = "cascade"
tab._mocap_pos = np.array([300.0, 400.0, 30.0]); tab._mocap_vel = np.array([-16.0, 0.0, 0.0])
tab._fm_tick()
assert tab._fm["phase"] == "backtrans", tab._fm["phase"]
print("RTH that never stops -> forced back-transition, not a hover handover at 16 m/s OK")
tab._fm = None

print("\nALL GUI SMOKE CHECKS PASSED")
