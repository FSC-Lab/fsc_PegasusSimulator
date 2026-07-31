#!/usr/bin/env python
"""
02_aerial_manipulator_track.py

Author: Ben Natra (send2ben123@gmail.com)
Author: Shiqi Gao (shiqi.gao907@gmail.com)

IN-PROCESS aerial-manipulator demo running controller_track.py (the full COUPLED
feedback-linearizing law) inside a physics-step callback — no ROS2 round-trip,
true 250 Hz control at the physics dt.

Two-phase flight, selected by the MODE knob below (mirrors the MATLAB harness):
  Phase 1 TAKEOFF  climb to TAKEOFF_ALTITUDE with the arm software-locked.
  Phase 2 TRACK    arm switches to torque-control impedance; the MODE trajectory
                   runs, re-anchored at the takeoff setpoint (zero initial
                   position/orientation error) with its own clock (t=0 at handover).
Set MODE = "hover" (hold the takeoff point) or "circle" (nominal MATLAB circle).

Each physics step (world.add_physics_callback → _control_step):
  1. read body pose/twist (DC) + arm joint states (core Articulation API)
  2. build X, generate the takeoff / trajectory reference  (controller_track.py)
  3. dynamics() + MatlabController()  → thrust, tau_body, tau_joint
  4. mixer  → 4 rotor speeds → backend.input_ref
  5. tau_joint → art.set_joint_efforts (arm dofs in true effort mode);
     software PD hold + gravity comp during the takeoff phase
  6. gripper_joint position target from the trajectory's grip flag

Run with:  scripts/start_aerial_manipulator_track.sh <config_name>
(no WSL controller needed)
"""

import os
import sys
import math
import json

# ── Sweep / batch overrides ──────────────────────────────────────────────────
# utils/gain_sweep.py launches this demo repeatedly with the AM_SWEEP env var
# holding a JSON dict. Recognized keys:
#   headless (bool)         run without a window
#   t_end (s)               auto-stop + save after this sim time
#   log_path (str)          npz destination (default log/track_log.npz)
#   traj_type (str)         override controller TRAJ_TYPE
#   k_x k_v k_R k_w         scalars
#   K_y D_y K_i             4-lists (diagonal)
#   M_r_d                   3-list (diagonal)
#   POSTURE_KP POSTURE_KD   4-lists
#   USE_POSTURE_ANCHOR      bool (True = paper law + extra u3 PD; False = paper law only)
#   DLS_LAMBDA TAU_MAX      scalars
# A crashed run (on the ground / flipped after takeoff) ends early and the npz
# carries crashed=True.
SWEEP = json.loads(os.environ.get("AM_SWEEP", "{}"))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": bool(SWEEP.get("headless", False))})

import numpy as np
import carb
import omni.usd
import omni.timeline
from omni.isaac.core.world import World
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
for sub in ("utils", "geometric_controller"):
    p = os.path.join(SCRIPT_DIR, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicle_manager import VehicleManager

from fsc_aerial_manipulation.utils import add_dome_lighting
from fsc_aerial_manipulation.rotorcraft.x650_rotorcraft_utils import (
    spawn_rotorcraft_with_mavlink,
    print_mass_inertia_properties,
    print_rotor_positions,
)
from fsc_aerial_manipulation.robotic_arm import controller_track as C   # the tuned controller (in-process)

# Data logs (.npz) are written to a `log/` folder inside the controller package,
# parallel to plot_results.py's `images/` output dir (resolved from the installed
# package so it holds regardless of CWD / editable-install location).
LOG_DIR = os.path.join(os.path.dirname(C.__file__), "log")

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  CONFIG  (matches the ROS2 demo so the same fixed model is used)         ║
# ╚══════════════════════════════════════════════════════════════════════════╝

# ── Flight mode (the knob you switch) ─────────────────────────────────────────
# Selects what the vehicle does AFTER the takeoff phase. EVERY mode runs the same
# two-phase sequence, mirroring the MATLAB test harness:
#   Phase 1  TAKEOFF — climb to TAKEOFF_ALTITUDE at CLIMB_RATE with the arm
#            software-LOCKED (PD + gravity comp through the effort path); the
#            coupled impedance law is not applied while on/near the ground.
#   Phase 2  TRACK   — the arm switches to the full torque-control impedance law
#            and the selected trajectory runs. At the phase-1→2 handover the
#            trajectory is RE-ANCHORED at the current CoM / EE offset / heading,
#            so it STARTS at the takeoff setpoint with zero initial position AND
#            orientation error, and its clock starts at 0 (not sim t=0).
# Modes (map 1:1 onto controller_track.TRAJ_TYPE / TRAJ_CONFIG):
#   "hover"  — hold the takeoff setpoint (the previous default test).
#   "circle" — track ONE rest-to-rest circle in the horizontal plane at the
#              takeoff altitude (radius/period from TRAJ_CONFIG["circle"],
#              default r=1.0 m, T=20 s). The base heading follows the tangent and
#              the EE tracks the compatible circle with all joint angles held at
#              zero — the nominal MATLAB circle test.
#   "circle_bent" — DIAGNOSTIC: the same circle, but the arm is held at a better-
#              conditioned pose (q_hold = [0,0.8,0.8,0]; cond(J_3y) ~35 vs ~96 at
#              q=0) instead of q=0. Isolates whether the posture anchor is only
#              needed because of J_3y conditioning. Run it, then comment out the
#              POSTURE_KP/KD term in controller_track.u3 and rerun: if the arm now
#              holds, the fold was conditioning; if it still folds, it's the
#              gravity-residual slope (independent of conditioning).
# Override per-run without editing this file: AM_SWEEP='{"traj_type":"circle"}'.
MODE = "circle"

# Vehicle USD — resolved from the installed fsc_aerial_manipulation package so
# the path holds regardless of CWD (same asset the ROS2 demo and the paired
# 01_aerial_manipulator_inproc.py use).
import fsc_aerial_manipulation.rotorcraft as _fsc_rotorcraft
ASSETS_DIR = os.path.join(os.path.dirname(_fsc_rotorcraft.__file__), "assets")
USD_FILE   = os.path.join(ASSETS_DIR, "AM_realign.usda")
USD_PRIM_PATH    = "/gripper_bat"
BODY_PATH        = "/body"
ROTOR_PATHS      = ["/rotor0", "/rotor1", "/rotor2", "/rotor3"]
ROTOR_JOINT_NAMES = ["joint0", "joint1", "joint2", "joint3"]
ARM_JOINT_NAMES  = ["manip_joint1", "manip_joint2", "manip_joint3", "manip_joint4"]
# Arm state/effort I/O goes through the core Articulation API (tensor-backed):
# raw q/qdot/tau are the model coordinates directly — no sign corrections.

SPAWN_POS   = (0.0, 0.0, 0.5)
SPAWN_EULER = (0.0, 0.0, 0.0)
VEHICLE_ID  = 0

CLIMB_RATE  = 0.4   # [m/s] takeoff altitude ramp (matches controller node)

# Rotor model — must match controller_track.py RotorMixerParams and the spawn thrust curve.
ROTOR_K_THRUST = 1.03e-5
ROTOR_K_TORQUE = 1.0e-6
ROTOR_DIR      = [-1, -1, 1, 1]

# Reflected rotor inertia — must equal controller_track.py make_params J_arm.
ARM_ARMATURE   = 353.5 ** 2 * 1.6e-7   # ≈ 0.02 kg·m²

# Safety clamps so a runaway control command can't blow up the physics / crash the
# sim. Normal operation is well inside these; they only cap pathological spikes.
# TAU_MAX sized to the joints: gravity scale is ~0.25 N·m, normal impedance ~0.5;
# the old 5.0 let a transient dump 250 rad/s² into a 0.02 kg·m² joint — with
# true effort control (no hidden drive damping) that flailed the arm at spawn
# and its reactions (>> the ~3.7 N·m the rotors can differential) tumbled the body.
TAU_MAX   = 1.5      # [N·m]   max arm joint torque (normal ~0.5)
OMEGA_MAX = 2000.0   # [rad/s] max rotor speed (hover ~850)

# Software arm hold during takeoff: the coupled feedback-linearized law is only
# valid AIRBORNE (ground contact is not in the model), so until the takeoff
# phase ends the arm gets a gentle PD-to-zero + gravity compensation THROUGH
# THE SAME effort path — drive gains are never switched mid-sim (mid-sim gain
# writes went stale in PhysX before).
ARM_HOLD_KP = 3.0    # [N·m/rad]   ωn = sqrt(3/0.02) ≈ 12 rad/s per joint
ARM_HOLD_KD = 0.25   # [N·m·s/rad] ζ ≈ 0.5

# Gripper: the fingers are actuated by ROTATING THE HUB via gripper_joint
# (manip_base ↔ hub revolute; the prismatic + lg_rev/rg_rev linkage converts hub
# rotation into finger travel). AM_realign re-zeroed the joint at the old open
# rest pose (the asset is saved with jaws open, limits ±50°), and flipped it to
# parent-first (body0=manip_base) — so the sign convention INVERTED vs the old
# asset: closing is now NEGATIVE hub rotation (old: rest −15.4° open, +35°
# closed = +50.4° of travel; new: 0° open, ≈−50° closed, right at the authored
# lower limit). Direction derived from the linkage geometry (hub pivots at
# ±y0.015 push the x-aligned links outward for +q) — VERIFY on the first run
# via the npz grip/grip_q traces + visual, flip GRIPPER_CLOSED to +50° if wrong.
GRIPPER_JOINT      = "gripper_joint"
GRIPPER_REST_DEG   = 0.0                              # authored rest target (open)
GRIPPER_OPEN       = math.radians(GRIPPER_REST_DEG)   # [rad] jaws open (= rest)
GRIPPER_CLOSED     = math.radians(-50.0)              # [rad] jaws closed (grasp)
GRIPPER_RATE       = 0.5                              # [rad/s] open/close ramp

# ── pick-and-place props ─────────────────────────────────────────────────────
# Spawned automatically when TRAJ_TYPE == "pickplace": a pedestal + graspable
# cube at the PICK grasp point and an empty pedestal at the PLACE point.
# Positions come from the same FK the trajectory uses, plus (a) the finger-pad
# midpoint (0.0494 m beyond the EE origin along its z, from the USD geometry)
# and (b) the MEASURED steady EE offset at the grasp pose (combo_pickplace run:
# e_y ≈ [−0.010, +0.044, +0.004] — the residual-CoM droop), so the cube sits
# between the open fingers when the gripper closes. The cube's world position
# is recorded in the npz ("obj_p") — the carry is measurable, not just visual.
PICKPLACE_PROPS = True
OBJ_SIZE   = 0.03    # [m] cube edge (pad gap ~7 cm open, ~3.5 cm closed)
OBJ_MASS   = 0.03    # [kg] deliberately unmodeled payload — part of the test
EE_SAG     = np.array([+0.0075, -0.0467, +0.0131])  # measured grasp offset [m]
                     # ^ measured from the 2026-07-14 AM_realign pickplace log:
                     # steady (r_e − r_ed) at grasp [+0.0089, −0.0442, +0.0042]
                     # PLUS the pad-vs-cube delta [−0.002, −0.0026, +0.009] from
                     # the arm settling at q≈−0.58 vs q_d=−0.498 (the posture/
                     # impedance equilibrium overshoots the reference, rotating
                     # the pad direction ~9° beyond the placement math). In that
                     # run the pads sat 9 mm above the cube center, caught only
                     # its top edge and dropped it on retract. Re-measure if
                     # gains or q_target change.
PAD_OFF_EE = np.array([0.0, 0.0, -0.0494])        # pad midpoint, EE frame [m]
STAND_XY   = 0.02    # [m] pedestal cross-section — MUST stay < OBJ_SIZE so the
                     # closing pads wrap the cube's faces, not the pedestal

# select the trajectory from the in-script MODE knob; AM_SWEEP's traj_type (batch
# runs) overrides it when present.
C.TRAJ_TYPE = str(SWEEP.get("traj_type", MODE))

# module-level sweep overrides (see AM_SWEEP block at the top; gain overrides
# on the controller instance are applied in InProcessDemoV2.__init__)
if "POSTURE_KP" in SWEEP:
    C.POSTURE_KP = np.asarray(SWEEP["POSTURE_KP"], float)
if "POSTURE_KD" in SWEEP:
    C.POSTURE_KD = np.asarray(SWEEP["POSTURE_KD"], float)
if "USE_POSTURE_ANCHOR" in SWEEP:   # False = strictly the paper's EE law (no extra u3 PD)
    C.USE_POSTURE_ANCHOR = bool(SWEEP["USE_POSTURE_ANCHOR"])
if "DLS_LAMBDA" in SWEEP:
    C.DLS_LAMBDA = float(SWEEP["DLS_LAMBDA"])
if "TAU_MAX" in SWEEP:
    TAU_MAX = float(SWEEP["TAU_MAX"])


class InProcessDemoV2:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # --- Pegasus world + environment ------------------------------------
        self.pg        = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world     = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        self._raise_physx_gpu_capacity()

        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(stage=stage, dome_path="/World/DomeLight",
                          intensity=2500.0, exposure=0.0, color=(1.0, 1.0, 1.0))

        # --- Spawn (ROS2 backend present but driven IN-PROCESS via input_ref) -
        self.drone_path = spawn_rotorcraft_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=VEHICLE_ID,
            spawn_pos=SPAWN_POS, spawn_euler=SPAWN_EULER,
            usd_file=USD_FILE, usd_prim_path=USD_PRIM_PATH,
            articulation_path="", body_path=BODY_PATH,
            rotor_paths=ROTOR_PATHS, rotor_joint_names=ROTOR_JOINT_NAMES,
            thrust_config={
                "rotor_constant": [ROTOR_K_THRUST] * 4,
                "rolling_moment_coefficient": [ROTOR_K_TORQUE] * 4,
                "rot_dir": ROTOR_DIR,
            },
            enable_lockstep=False,
            px4_primary=False,
            include_px4=False,        # no PX4 SITL in-process → skip MAVLink backend
        )

        self._wait_for_prim(self.drone_path, max_frames=300)
        # --- Model/physics fixes (same as the ROS2 demo) --------------------
        self._dedupe_physics_scenes()     # one PhysicsScene only
        self._disable_self_collisions()   # adjacent-link hulls overlap → thrash
        self._disable_rotor_colliders()   # props must not collide
        self._zero_arm_joint_states()     # start arm at 0, not the baked state
        self._setup_gripper_drive()       # stiff position drive + OPEN target (bakes pre-play)
        self._obj = None
        self._spawn_pickplace_props()     # pedestals + graspable cube (pickplace only)
        self.world.reset()
        self.stage = omni.usd.get_context().get_stage()

        print_mass_inertia_properties(self.stage, self.drone_path)
        print_rotor_positions(self.stage, self.drone_path, ROTOR_PATHS)

        # --- DC handles (body pose/twist only; the arm goes through the core
        # Articulation API below) -------------------------------------------
        self._dc     = _dynamic_control.acquire_dynamic_control_interface()
        self._body   = self._dc.get_rigid_body(self.drone_path + BODY_PATH)

        # --- Core Articulation API for the ARM (required). dynamic_control is
        # deprecated and its dof layer is broken here (wrong velocities, silent
        # gain writes); the tensor-backed Articulation API is the supported path
        # for dof state reads, effort writes and runtime gain changes.
        from omni.isaac.core.articulations import Articulation
        self._art = Articulation(prim_path=self.drone_path)
        self._art.initialize()
        names = list(self._art.dof_names)
        self._arm_idx = [names.index(nm) for nm in ARM_JOINT_NAMES]
        self._grip_idx = names.index(GRIPPER_JOINT) if GRIPPER_JOINT in names else None
        print(f"[InProc] core Articulation API active; arm dof indices {self._arm_idx}"
              f", gripper dof {self._grip_idx}", flush=True)

        # --- Pegasus backend (write rotor input_ref directly) ---------------
        self._vehicle = VehicleManager.get_vehicle_manager().get_vehicle(self.drone_path)
        self._backend = self._vehicle._backends[0]

        # --- Controller + mixer (in-process, controller_track.py) -----------------
        self.params = C.make_params()
        self.ctrl   = C.MatlabController(self.params)
        self.ctrl.hold = True            # takeoff hold until airborne
        print(f"[InProc] EE law: {'paper + extra u3 PD (posture anchor ON)' if C.USE_POSTURE_ANCHOR else 'strictly the paper (posture anchor OFF)'}",
              flush=True)
        # sweep gain overrides on the controller instance (AM_SWEEP)
        g = self.ctrl.g
        for k in ("k_x", "k_v", "k_R", "k_w"):
            if k in SWEEP:
                setattr(g, k, float(SWEEP[k]))
        for k in ("K_y", "D_y", "K_i"):
            if k in SWEEP:
                setattr(g, k, np.diag(np.asarray(SWEEP[k], float)))
        if "M_r_d" in SWEEP:
            g.M_r_d = np.diag(np.asarray(SWEEP["M_r_d"], float))
        if SWEEP:
            print(f"[InProc] AM_SWEEP overrides: {SWEEP}", flush=True)
        # auto-stop + crash bookkeeping (sweep runs)
        self._t_end = float(SWEEP["t_end"]) if "t_end" in SWEEP else None
        self._crashed = False
        self._done = False
        self.mixer  = C.RotorMixer(C.RotorMixerParams())

        # --- Arm actuation: TRUE effort control from the start --------------
        # switch_dof_control_mode("effort") zeros the drive gains — no hidden
        # drive damping (the model assumes an undamped joint; task damping comes
        # from D_y in the controller). The takeoff hold is a SOFTWARE hold
        # through this same effort path — drive gains are never touched mid-sim.
        self._set_arm_armature()
        actrl = self._art.get_articulation_controller()
        for i in self._arm_idx:
            actrl.switch_dof_control_mode(dof_index=i, mode="effort")
        print("[InProc] arm dofs in effort mode", flush=True)

        self._tr = None
        self._climb_z = 0.0
        self._t = 0.0
        self._log = 0
        self._grip_cmd = GRIPPER_OPEN            # rate-limited gripper command [rad];
        self._grip_q = 0.0                       # starts at the authored rest (open)
        if not hasattr(self, "_grip_idx"):
            self._grip_idx = None
        # history buffers for MATLAB-style plotting (saved on exit → plot script)
        self._hist = {k: [] for k in
                      ("t", "p", "v0", "omega0", "e_R", "e_y", "q", "qdot",
                       "thrust", "tau_j", "u3",
                       "t_thrust", "t_imp", "t_int", "t_cpl", "tau_b", "tau_b_arm",
                       "grip", "grip_q",
                       "x_c", "x_cd", "r_e", "r_ed", "q_d", "obj_p")}
        carb.log_info("[InProc] ready")

    def _record(self, p0, v0, omega0, q, qdot, tau_applied, res, ref, x_c, r_e):
        h = self._hist
        h["t"].append(self._t)
        h["p"].append(np.asarray(p0, float).copy())
        h["v0"].append(np.asarray(v0, float).copy())
        h["omega0"].append(np.asarray(omega0, float).copy())
        h["e_R"].append(np.asarray(res["e_R"], float).copy())
        h["e_y"].append(np.asarray(res["e_y"], float).copy())
        h["q"].append(np.asarray(q, float).copy())
        h["qdot"].append(np.asarray(qdot, float).copy())
        h["thrust"].append(float(res["thrust"]))
        h["tau_j"].append(np.asarray(tau_applied, float).copy())   # APPLIED torque (hold or task law)
        h["u3"].append(np.asarray(res["u3"], float).copy())
        h["tau_b"].append(np.asarray(res["tau_body"], float).copy())
        h["tau_b_arm"].append(np.asarray(res["tau_body_arm"], float).copy())
        h["grip"].append(float(getattr(self, "_grip_cmd", 0.0)))
        h["grip_q"].append(float(getattr(self, "_grip_q", 0.0)))
        # actual vs reference (CoM / EE / arm) — for the MATLAB-style plots
        h["x_c"].append(np.asarray(x_c, float).copy())
        h["x_cd"].append(np.asarray(ref["x_cd"], float).copy())
        h["r_e"].append(np.asarray(r_e, float).copy())
        h["r_ed"].append(np.asarray(ref["r_ed"], float).copy())
        h["q_d"].append(np.asarray(ref.get("q_d", np.zeros(4)), float).copy())
        # object world position (pickplace props) — proves/disproves the carry
        if self._obj is not None:
            try:
                pos, _ = self._obj.get_world_pose()
                h["obj_p"].append(np.asarray(pos, float).copy())
            except Exception:
                h["obj_p"].append(np.zeros(3))
        else:
            h["obj_p"].append(np.zeros(3))
        for k in ("t_thrust", "t_imp", "t_int", "t_cpl"):
            h[k].append(np.asarray(res[k], float).copy())

    def _save_history(self):
        h = self._hist
        if not h["t"]:
            return
        os.makedirs(LOG_DIR, exist_ok=True)
        path = SWEEP.get("log_path") or os.path.join(LOG_DIR, "track_log.npz")
        np.savez(path, crashed=np.array(self._crashed),
                 **{k: np.array(v) for k, v in h.items()})
        plot_script = os.path.join(os.path.dirname(C.__file__), "plot_results.py")
        print(f"[InProc] saved {len(h['t'])} samples -> {path} "
              f"(crashed={self._crashed})\n"
              f"         plot with the Isaac python (ISAAC_PY), e.g.:\n"
              f"           ~/isaacsim/python_r_fsc.sh {plot_script}", flush=True)

    # ── physics/model fixes (ported from the ROS2 demo) ─────────────────────

    def _raise_physx_gpu_capacity(self):
        stage = omni.usd.get_context().get_stage()
        scene = next((p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)), None)
        if scene is None:
            return
        api = PhysxSchema.PhysxSceneAPI.Apply(scene)
        api.CreateGpuFoundLostAggregatePairsCapacityAttr().Set(256 * 1024)
        api.CreateGpuTotalAggregatePairsCapacityAttr().Set(256 * 1024)
        api.CreateGpuFoundLostPairsCapacityAttr().Set(1024 * 1024)
        api.CreateGpuMaxRigidContactCountAttr().Set(2 * 1024 * 1024)
        api.CreateGpuMaxRigidPatchCountAttr().Set(256 * 1024)

    def _wait_for_prim(self, prim_path, max_frames=300):
        stage = omni.usd.get_context().get_stage()
        for _ in range(max_frames):
            p = stage.GetPrimAtPath(prim_path)
            if p and p.IsValid():
                return p
            self.world.step(render=False)
        carb.log_error(f"[InProc] prim never appeared: {prim_path}")
        return None

    def _dedupe_physics_scenes(self):
        stage = omni.usd.get_context().get_stage()
        scenes = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if len(scenes) <= 1:
            return
        keep = next((p for p in scenes
                     if not p.GetPath().pathString.startswith(self.drone_path)), scenes[0])
        for p in scenes:
            if p.GetPath() == keep.GetPath():
                continue
            path = p.GetPath()
            stage.RemovePrim(path)
            still = stage.GetPrimAtPath(path)
            if still and still.IsValid():
                still.SetActive(False)
        print(f"[InProc] kept one PhysicsScene: {keep.GetPath()}", flush=True)

    def _disable_self_collisions(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        target = next((pr for pr in Usd.PrimRange(root)
                       if pr.HasAPI(UsdPhysics.ArticulationRootAPI)), root)
        PhysxSchema.PhysxArticulationAPI.Apply(target).CreateEnabledSelfCollisionsAttr().Set(False)
        print(f"[InProc] self-collision disabled on {target.GetPath()}", flush=True)

    def _disable_rotor_colliders(self):
        stage = omni.usd.get_context().get_stage()
        n = 0
        for rp in ROTOR_PATHS:
            root = stage.GetPrimAtPath(self.drone_path + rp)
            if not root or not root.IsValid():
                continue
            for prim in Usd.PrimRange(root):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
                    n += 1
        print(f"[InProc] disabled {n} rotor collider(s).", flush=True)

    def _zero_arm_joint_states(self):
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        targets = set(ARM_JOINT_NAMES)
        n = 0
        for prim in Usd.PrimRange(root):
            if prim.GetName() in targets and prim.IsA(UsdPhysics.RevoluteJoint):
                for attr in ("state:angular:physics:position",
                             "state:angular:physics:velocity"):
                    a = prim.GetAttribute(attr)
                    if a and a.HasValue():
                        a.Set(0.0)
                n += 1
        print(f"[InProc] zeroed baked state on {n} arm joint(s).", flush=True)

    def _setup_gripper_drive(self):
        """Stiff position drive on gripper_joint, initial target OPEN. Runs before
        world.reset() so the gains bake into PhysX (USD target attr is in DEGREES;
        gains proven in gripper_drone_demo.py). Runtime targets go through the
        core Articulation API in _apply_gripper."""
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(self.drone_path)
        prim = next((p for p in Usd.PrimRange(root)
                     if p.GetName() == GRIPPER_JOINT and p.IsA(UsdPhysics.RevoluteJoint)), None)
        if prim is None:
            print(f"[InProc] WARNING: gripper joint '{GRIPPER_JOINT}' not found — "
                  f"grip commands will be ignored", flush=True)
            return
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.GetStiffnessAttr().Set(1e6)
        drive.GetDampingAttr().Set(1e4)
        drive.GetMaxForceAttr().Set(1000.0)
        # keep the AUTHORED rest target — no hub motion at spawn; the model's
        # link-4 lump was extracted at this pose
        drive.GetTargetPositionAttr().Set(GRIPPER_REST_DEG)
        print(f"[InProc] gripper drive stiffened on {prim.GetPath()} "
              f"(target = authored rest {GRIPPER_REST_DEG:.1f} deg)", flush=True)

    def _spawn_pickplace_props(self):
        """Pedestal + cube at the pick grasp point, empty pedestal at the place
        point (see the PICKPLACE_PROPS block for how positions are derived).
        Runs before world.reset() so physics picks the props up cleanly."""
        if not PICKPLACE_PROPS or C.TRAJ_TYPE != "pickplace":
            return
        try:
            from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
            cfg = C.TRAJ_CONFIG["pickplace"]
            qT = np.array(cfg["q_target"], float)
            _, r_c0, _, _ = C._arm_fk(np.zeros(4))
            r_eT, r_cT, _, _ = C._arm_fk(qT)
            R_e = np.eye(3)
            # (self.params doesn't exist yet — this runs pre-reset; params are cheap)
            for h, qq in zip(C.make_params()["h_i_im1"], qT):
                R_e = R_e @ C.joint_rotation(h, qq)
            anchor = np.array([SPAWN_POS[0] + r_c0[0], SPAWN_POS[1] + r_c0[1],
                               C.TAKEOFF_ALTITUDE])
            grasp = {}
            for tag in ("pick", "place"):
                wp = np.array(cfg[f"{tag}_wp"], float)
                grasp[tag] = anchor + wp + (r_eT - r_cT) + R_e @ PAD_OFF_EE + EE_SAG
            for tag, g in grasp.items():
                h = float(g[2] - OBJ_SIZE / 2.0)
                FixedCuboid(prim_path=f"/World/{tag}_stand", name=f"{tag}_stand",
                            position=np.array([g[0], g[1], h / 2.0]),
                            scale=np.array([STAND_XY, STAND_XY, h]),
                            color=np.array([0.35, 0.35, 0.4]))
            self._obj = DynamicCuboid(
                prim_path="/World/pick_object", name="pick_object",
                position=grasp["pick"], scale=np.array([OBJ_SIZE] * 3),
                color=np.array([0.9, 0.15, 0.15]), mass=OBJ_MASS)
            try:
                from omni.isaac.core.materials import PhysicsMaterial
                mat = PhysicsMaterial("/World/Physics/obj_material",
                                      static_friction=1.2, dynamic_friction=1.0,
                                      restitution=0.0)
                self._obj.apply_physics_material(mat)
            except Exception as exc:
                print(f"[InProc] object friction material skipped ({exc})", flush=True)
            print(f"[InProc] pickplace props: cube at {np.round(grasp['pick'], 3)}, "
                  f"place stand at {np.round(grasp['place'], 3)}", flush=True)
        except Exception as exc:
            self._obj = None
            print(f"[InProc] WARNING: pickplace props failed ({exc}) — "
                  f"flying without objects", flush=True)

    def _apply_gripper(self, grip_closed, dt):
        """Rate-limited gripper command from the trajectory's grip flag."""
        if self._grip_idx is None:
            return
        target = GRIPPER_CLOSED if grip_closed >= 0.5 else GRIPPER_OPEN
        step = max(-GRIPPER_RATE * dt, min(GRIPPER_RATE * dt, target - self._grip_cmd))
        self._grip_cmd += step
        from omni.isaac.core.utils.types import ArticulationAction
        self._art.get_articulation_controller().apply_action(ArticulationAction(
            joint_positions=np.array([self._grip_cmd]),
            joint_indices=np.array([self._grip_idx])))

    def _arm_prims(self):
        root = self.stage.GetPrimAtPath(self.drone_path)
        prims = {p.GetName(): p for p in Usd.PrimRange(root)
                 if p.IsA(UsdPhysics.RevoluteJoint)}
        return [(n, prims[n]) for n in ARM_JOINT_NAMES if n in prims]

    def _set_arm_armature(self):
        for name, prim in self._arm_prims():
            PhysxSchema.PhysxJointAPI(prim).CreateArmatureAttr().Set(float(ARM_ARMATURE))
        print(f"[InProc] arm armature = {ARM_ARMATURE:.4f} kg·m²", flush=True)

    # ── state ───────────────────────────────────────────────────────────────

    def _read_state(self):
        pose = self._dc.get_rigid_body_pose(self._body)
        lin  = self._dc.get_rigid_body_linear_velocity(self._body)
        ang  = self._dc.get_rigid_body_angular_velocity(self._body)
        p0 = np.array([pose.p.x, pose.p.y, pose.p.z])
        R0 = C.quat_to_rot(pose.r.w, pose.r.x, pose.r.y, pose.r.z)
        v0 = R0.T @ np.array([lin.x, lin.y, lin.z])     # world → body
        omega0 = R0.T @ np.array([ang.x, ang.y, ang.z]) # world → body
        q_all  = np.asarray(self._art.get_joint_positions(), float)
        qd_all = np.asarray(self._art.get_joint_velocities(), float)
        q    = q_all[self._arm_idx]
        qdot = qd_all[self._arm_idx]
        if self._grip_idx is not None:
            self._grip_q = float(q_all[self._grip_idx])   # measured hub angle
        return R0, p0, v0, omega0, q, qdot

    def _build_X(self, R0, p0, q, v0, omega0, qdot):
        return np.concatenate([p0, R0.flatten(order="F"), q, v0, omega0, qdot])

    # ── control step (physics callback) ─────────────────────────────────────
    # Registered via world.add_physics_callback → runs once per PHYSICS step
    # with the true step_size (world.step(render=True) advances ~4 physics
    # steps per call, so render-loop control would run 4x slow with a 4x-wrong dt).

    def _control_step(self, step_size):
        try:
            self._control_step_inner(step_size)
        except Exception as exc:
            carb.log_error(f"[InProc] control step: {exc}")

    def _control_step_inner(self, dt):
        R0, p0, v0, omega0, q, qdot = self._read_state()

        if self._tr is None:
            # first-pose setup (anchor the trajectory) — mirrors the ROS2 node
            X0   = self._build_X(R0, p0, q, v0, omega0, qdot)
            dyn0 = C.dynamics(X0, self.params)
            x_c  = p0 + R0 @ dyn0["r_0c_0"]
            self._tr = C.build_traj(np.array([x_c[0], x_c[1], C.TAKEOFF_ALTITUDE]),
                                    R0 @ (dyn0["r_0e_0"] - dyn0["r_0c_0"]),
                                    R0 @ np.array([1.0, 0, 0]))
            self._climb_z = x_c[2]
            self._t = 0.0
            print(f"[InProc] first p0={p0.round(3)} traj='{self._tr['type']}' "
                  f"climb {x_c[2]:.2f}->{C.TAKEOFF_ALTITUDE:.2f} m  dt={dt:.4f}",
                  flush=True)
            return

        self._t += dt
        # sweep bookkeeping: auto-stop at t_end; flag + stop on a crash
        # (on the ground after the spin-up, or flipped past ~78 deg)
        if self._t_end is not None and self._t >= self._t_end:
            self._done = True
        if self._t > 1.0 and (p0[2] < 0.05 or R0[2, 2] < 0.2):
            if not self._crashed:
                print(f"[InProc] CRASH detected at t={self._t:.2f} "
                      f"(z={p0[2]:.3f}, R33={R0[2,2]:.2f})", flush=True)
            self._crashed = True
            self._done = True
        # Takeoff hold: the coupled law is only valid AIRBORNE (ground contact
        # is not in the model), so the arm is held by a software PD + gravity
        # comp through the same effort path until the climb window ends.
        hold = self._t < C.TAKEOFF_TIME
        if hold != self.ctrl.hold:
            self.ctrl.hold = hold
            print(f"[InProc] arm hold -> {hold} (t={self._t:.1f}s)", flush=True)
            if not hold:
                # RE-ANCHOR at the hold→impedance handover. The integrator-free
                # loops hover with a steady tilt+offset (e_R~0.03 roll → thrust
                # tilts → the position loop settles ~0.1 m off in y, and e_y
                # inherits it). Anchoring here starts e_x=e_y=0; the vehicle
                # holds its actual equilibrium instead of the spawn xy.
                dyn_a = C.dynamics(self._build_X(R0, p0, q, v0, omega0, qdot), self.params)
                x_c_now = p0 + R0 @ dyn_a["r_0c_0"]
                self._tr = C.build_traj(x_c_now, R0 @ (dyn_a["r_0e_0"] - dyn_a["r_0c_0"]),
                                        R0 @ np.array([1.0, 0, 0]))
                print(f"[InProc] re-anchored ref at x_c={x_c_now.round(3)}", flush=True)

        if hold:
            dz = C.TAKEOFF_ALTITUDE - self._climb_z
            step = max(-CLIMB_RATE * dt, min(CLIMB_RATE * dt, dz))
            self._climb_z += step
            vz = step / dt if dt > 0 else 0.0
            z3 = np.zeros(3)
            ref = {"x_cd": np.array([self._tr["x_c0"][0], self._tr["x_c0"][1], self._climb_z]),
                   "x_cd_dot": np.array([0.0, 0.0, vz]), "x_cd_ddot": z3,
                   "x_cd_d3": z3, "x_cd_d4": z3}
            ref = C._fixed_ee(ref, self._tr)
        else:
            ref = C.generate_reference(self._t - C.TAKEOFF_TIME, self._tr)

        X   = self._build_X(R0, p0, q, v0, omega0, qdot)
        dyn = C.dynamics(X, self.params)
        res = self.ctrl(X, dyn, ref, dt)

        # rotors → input_ref (clamped: a runaway command can't blow up the sim)
        omega = np.clip(self.mixer.mix(res["thrust"], res["tau_body"]), 0.0, OMEGA_MAX)
        for i in range(min(4, len(self._backend.input_ref))):
            self._backend.input_ref[i] = float(omega[i])

        # arm efforts EVERY step: software hold during takeoff, full impedance
        # law after. One actuation path, one clamp.
        if hold:
            g_arm = dyn["g"][6:]
            # hold to the trajectory's q_hold (zeros for every mode except
            # circle_bent) so the arm reaches the commanded pose BEFORE the
            # trajectory starts → zero initial joint/EE error at the handover,
            # and circle_bent starts already at its non-singular pose.
            q_hold = self._tr.get("q_hold", None)
            q_ref = q_hold if q_hold is not None else np.zeros_like(q)
            tau_j = -ARM_HOLD_KP * (q - q_ref) - ARM_HOLD_KD * qdot + g_arm
        else:
            tau_j = res["tau_joint"]
        tau_j = np.clip(tau_j, -TAU_MAX, TAU_MAX)
        self._art.set_joint_efforts(np.asarray(tau_j, float),
                                    joint_indices=self._arm_idx)

        # gripper: trajectory grip flag. Refs without one (takeoff/hover) default
        # to OPEN = the authored rest pose → the hub never moves until the
        # trajectory commands the grasp.
        self._apply_gripper(ref.get("grip", 0.0), dt)

        self._record(p0, v0, omega0, q, qdot, tau_j, res, ref,
                     x_c=p0 + R0 @ dyn["r_0c_0"], r_e=p0 + R0 @ dyn["r_0e_0"])

        self._log += 1
        if self._log >= 250:
            self._log = 0
            drive = (res["t_thrust"] + res["t_cpl"]).round(3)     # pushes arm off
            correct = (res["t_imp"] + res["t_int"]).round(3)       # fights to fix it
            print(f"[InProc] t={self._t:5.1f} z={p0[2]:.3f} thr={res['thrust']:.1f} "
                  f"e_R={res['e_R'].round(3)}\n"
                  f"          q={q.round(3)} qdot={qdot.round(3)} "
                  f"tau_j={tau_j.round(3)}\n"
                  f"          u2={np.asarray(res['u2']).round(3)} "
                  f"u3={np.asarray(res['u3']).round(3)}\n"
                  f"          tau_body={res['tau_body'].round(3)} "
                  f"N1ᵀu3={res['tau_body_arm'].round(3)} omega0={omega0.round(3)}\n"
                  f"          arm err e_y={res['e_y'].round(3)}  "
                  f"drive={drive}  correct={correct}\n"
                  f"          q_d={np.asarray(ref.get('q_d', np.zeros(4))).round(3)} "
                  f"grip cmd={math.degrees(self._grip_cmd):.1f}deg "
                  f"meas={math.degrees(self._grip_q):.1f}deg", flush=True)

    # ── main loop ───────────────────────────────────────────────────────────

    def run(self):
        self.world.add_physics_callback("am_control", self._control_step)
        self.timeline.play()
        render = not bool(SWEEP.get("headless", False))   # headless: skip rendering
        while simulation_app.is_running() and not self._done:
            self.world.step(render=render)   # control runs inside the physics steps
        self._save_history()
        self.timeline.stop()
        simulation_app.close()


def main():
    InProcessDemoV2().run()


if __name__ == "__main__":
    main()
