# PegasusSimulator — Developer Checklist

## Repository layout

This is the FSC Lab fork of [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) (NVIDIA Isaac Sim framework for PX4/ArduPilot multirotor simulation).

- `extensions/pegasus.simulator/` — core simulator (upstream): vehicles, backends, sensors, dynamics, thrusters, graphs, UI
  - `pegasus/simulator/logic/vehicles/` — vehicle base class (`vehicle.py`), `multirotor.py`, and concrete models under `multirotors/` (e.g. `iris.py`, `ideal_quadrotor.py`)
  - `pegasus/simulator/logic/backends/` — control/telemetry backends (PX4 mavlink, ArduPilot mavlink, ROS 2)
- `extensions/fsc_aerial_manipulation/` — FSC Lab's own library: `aerodynamics/`, `constraints/`, `robotic_arm/`, `rotorcraft/`, `slung_load/`, `utils/`
- `application/` — runnable example scripts, one subfolder per scenario: `ideal_quadrotor/`, `px4_base/`, `slungload/`, `robotic_arm/` (X650 quadrotor + arm, see below). `aerial_manipulation/` and `VTOL/` still exist as empty placeholders
- `examples/`, `docs/`, `scripts/`, `tools/` — upstream examples, docs, and Isaac Sim tooling

## Aerial manipulator (X650 quadrotor + robotic arm)

Merged from teammate Shiqi Gao's `dev_robotic_arm` branch (`380d297`, 2026-07-10) — a quadrotor
("X650" airframe) with a 4-DOF robotic arm rigidly coupled to it, controlled by a Python port of
a validated MATLAB whole-body geometric+impedance controller. Its original/default `direct`
mode is pure ROS 2. The launcher now also provides a `px4-offboard` mode where the external
controller streams PX4 `OffboardControlMode(direct_actuator=True)` + `ActuatorMotors`, PX4
applies its Offboard/arming/saturation gate, and the resulting `HIL_ACTUATOR_CONTROLS` drives
Isaac through Pegasus's primary MAVLink backend.

**Related paths — treat these three as one unit for future work on this scenario:**
- `application/robotic_arm/`
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/`
- `scripts/indoor_sim/start_aerial_manipulator.sh`

- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_vehicle/x650_vehicle.py` —
  `VehicleMod(Vehicle)`, thin subclass adding `usd_prim_path`/configurable `body_path` for a
  custom USD asset.
- `.../robotic_arm/utils_vehicle/x650_multirotor.py` — `MultirotorMod(BaseMultirotor, VehicleMod)`, adds
  configurable rotor prim paths/joint names. Compliant with this file's vehicle-model checklist
  below (`config=None` resolved inside `__init__`, no mutable defaults).
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/x650_rotorcraft_utils.py`
  — spawn helper (`spawn_rotorcraft_with_mavlink` with `px4_primary`/`include_px4` toggles for
  ROS2-only vs PX4-primary operation) plus asset-authoring helpers used to extract USD-derived
  mass/inertia/rotor-position params for the controller.
- `.../robotic_arm/controller.py` (1008 lines) — `MatlabController` (thrust + body torque +
  joint torques from the full coupled airframe+arm dynamics, not the arm alone) wrapped in a
  ROS2 `MatlabControllerNode`: subscribes `state/pose`/`state/twist*`/`joint_states`, publishes
  `rotor_velocity_command`/`joint_torque_cmd`. Run standalone (`python3 controller.py --ros-args
  -r __ns:=/uav_0`), not launched by the Isaac Sim script itself.
- `.../robotic_arm/utils_model/postprocessor.py` (1227 lines) — **not** a runtime component. A one-shot
  OnShape→USD asset-cleanup pipeline (reparent/rename/realign frames/set mass+inertia/add
  colliders/apply joint drives/export), meant to be pasted into Isaac Sim's Script Editor after
  CAD import, not imported by anything else.
- `.../robotic_arm/__init__.py` is empty — import submodules directly
  (`fsc_aerial_manipulation.robotic_arm.utils_vehicle.x650_vehicle`, etc.), not via the package.
- `application/robotic_arm/01_aerial_manipulator_hover.py` (761 lines) — the working demo
  entrypoint: DC-interface joint control, ROS2 pub/sub for torque/rotor commands, arm
  position-hold→effort-control handoff.
- `application/robotic_arm/22_x650_with_manip_minimal.py` (109 lines) — **currently broken as
  committed**: inserts `application/robotic_arm/utils/` onto `sys.path` to import
  `x650_rotorcraft_utils`, but that `utils/` directory doesn't exist in this repo (line 30), and
  it hardcodes an absolute USD path under `/home/fsc-jupiter/...` (line 73) rather than
  resolving relative to `FSC_PEGASUS_ROOT`. Don't use as a template until fixed — `01` is the
  correct reference.
- `scripts/indoor_sim/start_aerial_manipulator.sh` + `scripts/config/shiqi_machine.conf` — follows the
  same `common_config.sh`/`terminal_utils.sh`/per-machine-config convention as the slung-load
  launch scripts. `direct` starts the original two panes (Isaac + controller).
  `px4-offboard` starts PX4 SITL, Micro XRCE-DDS Agent, Isaac, and the controller; it sources a
  configurable `PX4_MSGS_SETUP`, applies `UXRCE_DDS_SYNCT=0` plus disabled HIL auto-disarm for
  the run, prestreams for one second, confirms Offboard, then auto-arms. The controller runs
  under `--ros-args -r __ns:=/uav_0`, matching `PX4_UXRCE_DDS_NS=uav_0`. The actual Isaac
  entrypoint is `01_aerial_manipulator_hover.py`.
- `.../robotic_arm/utils_planner/compatible_trajectory.py` (new, 2026-08-06; moved into
  `utils_planner/` 2026-08-08) — offline planner for the
  dynamically-compatible EE+CoM trajectory, Python port of `MATLAB
  Code/utils/utils_planning/plan_compatible_trajectory.m`: prescribes the EE task (3-axis
  position span+arc, orientation sweep yaw/tilt/wrist) plus an independent platform heading,
  and SOLVES the CoM reference x_cd(t) (deg-16 polynomial, Picard fixed point over thrust-dir ↔
  arm-kinematics coupling). Pure numpy, testable without Isaac. Two deliberate deviations from
  the MATLAB original: (1) **z-x-z, not z-y-z** — the AM_realign asset's joints 2/3 rotate
  about +x (MATLAB model: +y), so R_e = Rz·Rx·Rz and the joint recovery is a z-x-z
  decomposition; (2) planned once in a **canonical frame** (EE start at origin, zero heading)
  and cached, then anchored by translation+yaw in `build_traj` — exact since gravity is along
  z. Uses the exact `com_i` link CoMs from `make_params()`, not the MATLAB's l_i/2
  approximation. Wired into `controller_free.py` as `TRAJ_TYPE = "compatible"`
  (`TRAJ_CONFIG["compatible"]` holds the task/planner knobs; angle amplitudes sized to this
  asset's authored joint limits — q1 ±35°, q2/q3 −90..+50°, checked by the planner's
  `q_lim_deg` warning) and into `02_aerial_manipulator_free.py` as `MODE = "compatible"` (now
  the default; EE_FORCE_ENABLE set False for the plain tracking test). The plan's t=0 arm pose
  `q0 = [0, b0/2, b0/2, 0]` is exported as `tr["q_hold"]` so the existing takeoff hold
  pre-positions the arm — run with `TAKEOFF_ARM_HOLD = True`. Offline validation (2026-08-06,
  system python): Picard converges in 7 iters, dynamic defect 3.7e-6 m, cond(J_3y) ≈ 15 the
  whole plan (vs ~96 at q=0), all reference derivative chains FD-consistent to ~1e-10, ref
  eval ~0.2 ms/call at 250 Hz. Only wired into the GMO controller/demo, not
  `controller_track.py` (its generate_reference has no "compatible" branch).
  **4-phase showcase flight added same day** (`TRAJ_TYPE="compatible_showcase"`, now the
  demo's default MODE): takeoff with the arm **FOLDED at the singularity-certified HOME pose
  β=80° (q_hold=[0,40°,40°,0])** → fly-in forward (**along world +y**, see `path_yaw_deg`
  below) while UNFOLDING (β→50°) → PINNED phase (EE position+heading exactly fixed — wrist
  stays 0 so b1_de is β-independent — while the platform yaws ±15° and the arm folds/unfolds
  50→75→50°) → fly-out to above the landing spot while FOLDING BACK to home →
  **pure vertical descent to a LANDING, arm at home throughout**. HOME POSE (2026-08-06,
  user's request, derived from `refs_matlab/utils/utils_singularity/singularity_sweep.m`
  mapped onto this asset): the elbow/yaw singular branches live at **NEGATIVE q2/q3 in this
  asset's sign convention** (MATLAB's +y joint axes flip sign vs the asset's +x; deepest
  valley q3≈−52°, and the previously measured cond≈880 at q2=q3=−43° IS the elbow branch),
  while on the positive fold line nondimensional σ_min(J_3y0/Lchar) RISES monotonically with
  β (0.38 at q=0 → 0.52 at 80° = 5.2× the analysis' 0.10 keep-out margin, worst-case over
  q1 ±35°/q4 ±180°; task envelope worst 0.48). The planner now computes/prints this
  certification per plan (`diag min_sigma_nd`, warns <0.10). Arm fold direction (measured,
  easy to get backwards): β=q2+q3 and LARGER β tucks the EE UP toward the body (base→EE reach
  0.279 m at β=0, 0.205 at 50°, 0.16 at 80°) — folded home = compact + ground-safe (~0.26 m
  EE clearance at rest) + best-conditioned. `land_drop` resized 1.03→1.17 m (same home pose
  at both ends ⇒ EE drop = CoM drop; the old value belonged to the asymmetric schedule).
  **TASK SHAPE reworked 2026-08-07 (user request)**: (a) the pinned point sits 0.35 m BELOW
  both the fly-in start and fly-out end, each transit arcing ~0.15 m over first, so the SIDE
  view shows climb-over→dip→climb-over→descend (EE z 1.48→1.62→1.13→1.62→1.48→land) while
  `Ay=0` keeps both transits DEAD STRAIGHT in bird view (measured 0.0 mm lateral deviation);
  (b) the pinned phase is a TRUE GIMBAL — EE position AND heading frozen inertially while the
  platform sweeps a FULL sine cycle 0→+30°→0→−30°→0 (new `_scal_sincycle` helper, min-snap
  phased so endpoint rates vanish), returning to forward so fly-out continues straight;
  `yaw_work_deg` dropped to 0 to free all of q1 for this. **TWO KINEMATIC BOUNDS worth
  knowing**: (1) a FULL 360° platform spin is IMPOSSIBLE with the EE yaw locked — R_e^0 =
  R_0ᵀR_e forces q1 = −ψ and q1 is limited to ±35°; q4 cannot help because the two z-rotations
  in Rz(q1)Rx(β)Rz(q4) only trade at β=0 (the wrist singularity). Dropping the yaw lock would
  allow a full circle. (2) the drone barely TRANSLATES while pinned — q1 counter-rotating the
  yaw also counter-rotates the arm's offset, so R_0·r_0e is invariant and a pure yaw moves the
  base NOT AT ALL; all pinned translation comes from the arm's FOLD changing r_0e, only
  ~0.03 m horizontally / ~0.06 m vertically over β 50–80° (measured CoM span 0.059 m). This is
  a workspace bound, not a tuning knob.
  **Takeoff-hold target is SLEWED (`ARM_HOLD_RATE`=0.5 rad/s), not stepped**: the raw 0→40°
  spawn step overshot the ζ≈0.5 PD into q2's +50° hard stop (measured 50.0° at t=0.35 s);
  the slew caps the transient at 26° and peak hold torque 2.62→0.86 N·m.
  Ground contact is NOT in the control model, so `02_aerial_manipulator_free.py` gained a
  `LAND_*` block: below `LAND_HOLD_Z`=1.0 m the arm reverts to the software PD hold **tracking
  the stowed pose** (NOT the takeoff q_hold — that would swing the arm down at touchdown) and
  the GMO freezes; after the plan ends the rotors ramp off over `LAND_DISARM_TIME`=2 s.
  **The vehicle's resting body height is 0.305 m / CoM 0.290 m — MEASURED from a landing run,
  not the 0.613 m spawn height** (the drone lifts off at t=0 and is never seen resting at
  spawn; sizing `land_drop` off the spawn value left the reference 27 cm high and the rotor
  ramp turned it into a drop). Planner
  extensions this required, all in `compatible_trajectory.py`: (a) segmented prescribed task
  (`_prescribed_task_showcase`, selected by the `T_fly_in` cfg key); (b) **min-snap (septic)
  segment phases** — the compatible CoM's velocity depends on the prescribed JERK via the
  thrust-direction map, so min-jerk phases (jerk jumps at joins) would make the solved CoM
  velocity discontinuous at phase boundaries; (c) **piecewise polynomial p_c, one per phase,
  with constrained fits** (value pinned + zero vel/acc at both segment ends) — a single
  global polynomial fits move–hold–move poorly (4 mm defect, shifted q0), and unconstrained
  per-segment fits RING in their endpoint derivatives, which the Picard loop amplifies
  through p_c'' (deg 20 constrained even diverges — deg 12 is the sweet spot). The classic
  single-segment mode keeps the plain unconstrained fit (flight-validated, ~40x lower
  defect). Headless-validated (2026-08-06, 34 s run): pinned-phase EE held to 0.28 mm / yaw
  err 1e-5 while the base flew an 11.7 cm path; per-phase EE error ≤0.7 mm; touchdown settles
  3.9 cm at ≤0.34 m/s onto the resting height with final thrust back at hover (32.7 N ≈ mg)
  and zero drift after; joints peak q1 30°, q2/q3 44° (limit 50°).
  **`path_yaw_deg` (added 2026-08-06, default 90° for the showcase)** — decouples TRAVEL
  direction from HEADING: `_eval_task` rotates only the prescribed EE *position* about z,
  leaving R_e/b1_de/b1_d untouched. Needed because the controller's b1_d steers BODY +x but
  this asset's mechanical front (arm side) is BODY +y, so the demo would otherwise fly
  sideways-on. At 90° the vehicle travels along world +y — face-first — while the attitude
  reference stays on +x (it never yaws to follow the path; the only yaw is the ±15° pinned
  counterpoint). Valid because the canonical EE start is the origin and gravity is
  z-invariant. **TEMPORARY**: the user plans to re-author the USDA so the front is body +x,
  after which this goes back to 0.0. A USDA-only rotation is NOT sufficient — `make_params`
  (both controllers), `RotorMixer.rotor_positions`, `M_r_d`, and the planner's z-x-z joint
  recovery all encode the body frame and must change together, and the measured pickplace
  `EE_SAG` would go stale. Re-validated headless at 90°: identical quality to the +x version
  (pinned EE 0.28 mm, touchdown 3.9 cm settle, no crash).
  **Spawn-at-home (2026-08-07, user request off the singularity-sweep Fig 3)**: the arm now
  SPAWNS at the folded home pose (`02`'s `Q_SPAWN` = the plan's `q0` = [0,40°,46°,0], resolved
  from the cached pure-numpy `get_plan` pre-Isaac; zeros for hover/circle, `q_hold` for
  circle_bent) instead of hanging at q=0 — at q=0 the gripper pad reaches 0.282 m below the
  base, only 2.3 cm off the floor at the 0.305 m resting height, and the old zero-then-slew
  start swept it near the ground through the first ~1.5 s of climb; at home the lowest arm
  point (the elbow) keeps 0.14 m clearance. The arm now holds home continuously
  spawn→takeoff→fly-in, and the landing hold already tracked home through touchdown. The home
  POSE itself is unchanged (the user's sketched link2-horizontal fold needs q2≈79° vs the +50°
  stop; β=96 would raise the elbow only 1.5 cm while erasing the stop margin the measured
  ~7.5° q2 handover overshoot needs — β=86 is the feasible optimum, σ_nd 0.529). Two
  hard-won implementation facts: (a) authoring `PhysicsJointStateAPI` state attrs pre-reset
  does NOT take effect in 02's startup sequence (the asset authors no state values; the
  pre-reset `world.step()` calls in `_wait_for_prim` let PhysX capture its reset-snapshot at
  q=0 and `world.reset()` restores that over the authored attrs — measured q(t0)=0), so the
  pose is written post-`_art.initialize()` via `set_joint_positions`; (b) **PhysX roots the
  AM_realign articulation tree at an ARM link**, so that teleport swings the BODY around the
  arm — it came out rolled −(q2+q3) = −86° about x and 0.21 m displaced, the anchored
  reference chased the sideways thrust axis and the vehicle tumbled at t=1 s. The write is
  therefore followed by a RIGID RE-SEAT (measure body pose before/after, left-apply the world
  correction to the articulation root via `set_world_pose`; exact no-op on a body-rooted
  tree). The old `_zero_arm_joint_states` was deleted — the asset authors zero state anyway,
  it was always a no-op. Headless-validated (34 s, 2026-08-07): arm within 2.0° of home for
  the whole climb (vs the old 64° hanging→folded swing), q2 peak 47.2° at handover, pinned EE
  0.43 mm mean / 1.17 mm max, touchdown at exactly 0.305 m, zero final drift, final q = home.
  **TRAJECTORY CATALOGUE RENAMED + EXTENDED (2026-08-08, user request — supersedes every
  older mode name above)**: `utils_planner` now names every free-flight mode `<shape>_<who>`
  (`<shape>` hover/circle/figure8/poly; `<who>` `drone` = arm LOCKED at one pose, `whole` =
  whole body moves). Renames: `hover`→`hover_drone`, `circle`→`circle_drone` (old
  `circle_bent` became its optional `q_hold` config knob), `compatible_showcase`→`poly_whole`
  (remark: showcase_end_effector_gimbal); the classic single-phase `compatible` entry was
  DELETED from the catalogue (the offline planner in `compatible_trajectory.py` is untouched
  — it is poly_whole's engine, and its classic-task branch still evaluates for legacy cfgs).
  FOUR NEW modes, implemented in `trajectory_library.py` + new `utils_planner/arm_sweep.py`:
  (a) `hover_whole` (remark: showcase_drone_gimbal) — the base HOVERS pinned at the takeoff
  setpoint while a prescribed EE command sweeps the arm (fold β 80→50→80 two full min-snap
  cycles + arm-yaw q1 ±25° in quadrature → the EE traces a closed loop; measured span 12 cm
  lateral / 5 cm vertical / ±25° EE yaw). Dynamically EXACT, not an approximation: arm motion
  is internal, so a fixed system-CoM reference plus a moving EE reference is fully compatible
  — the visible "drone gimbal" is the base counter-moving a few cm about the held CoM.
  (b) `circle_whole` / `figure8_whole` — the _drone base path plus a β 50..80° fold sweep
  (q1 = 0 keeps b1_de exactly on the tangent); the EE reference is the FK-exact swept offset
  (arm_sweep.py: central-difference Jacobian ε=1e-6 + directional second difference ε=1e-4
  on the exact `_arm_kin` chain, anchored as d(t)=Rz(Δψ)(d0+Rz(ψ0)[u(q(t))−u(q0)]) so d(0)
  EQUALS the measured anchor → zero initial EE error; ~0.4 ms/eval vs the 4 ms 250 Hz budget).
  (c) `figure8_drone` — Gerono lemniscate x=A·sinθ, y=(B/2)·sin2θ (A=1.2 m, B=0.7, T=24 s),
  θ = 2π·minjerk, same Faà-di-Bruno 4-derivative chain as the circle; tangent heading is
  parameterized by θ (defined at the zero-speed rest endpoints), and |de/dθ| ≥ B > 0 so the
  tangent never degenerates. Unlike the circle, the yaw RATE REVERSES at the crossing.
  (d) `poly_drone` — the showcase waypoints flown by the BASE alone: min-snap septic segments
  (new `minsnap()` with 4 derivatives — jerk-continuous joins, one order smoother than the
  circle's minjerk) fly-in→hold→fly-out over the same target/land_wp/timings as poly_whole,
  the same ±30° mid-phase platform-yaw full sine cycle, arm locked at the folded home
  [0,40°,46°,0]; ends hovering at land_wp and LANDS (`LAND_ENABLE` is now poly_whole OR
  poly_drone). The intended A/B: with the arm locked the yaw cycle SWINGS the EE around the
  base — nothing can be pinned. New API `P.initial_arm_pose(traj_type, params=None)` replaces
  02's `_takeoff_arm_pose()` for Q_SPAWN; EVERY mode with a non-zero arm pose exports
  `tr["q_hold"]`. Consumers updated: `02_aerial_manipulator_free.py` (MODE default
  "poly_whole"; waypoint markers + npz phase table know poly_drone),
  `px4_direct_actuator_aerial_manipulator_gmo.py` (MODE="hover_drone"),
  `utils/px4_gmo_gain_sweep.py`, controller_free's self-test, `Command.md`,
  `Flight State Machine.md`. `controller_track`'s EMBEDDED catalogue (used by
  01_track) keeps its OWN old names — it never imported utils_planner, so the rename does
  not touch it. **SUPERSEDED 2026-08-09 for pick/push**: those two controllers were merged
  away (see the CONTROLLER MERGE entry below) and their catalogues now live in
  utils_planner alongside everything else. Offline-validated (system
  python, 2026-08-08): FD-consistency of every derivative chain in all 8 modes (max interior
  err ≤2e-4, FD-limited), poly_drone joins continuous through JERK, sweeps inside joint
  limits (q2/q3 peak 40° vs 50 limit, q1 25 vs 35, peak rates ≤57°/s), headings unit-norm,
  all endpoints at rest, controller_free self-test passes (hover thrust = mg exactly).
  NOT yet flown in Isaac.

**CONTROLLER MERGE (2026-08-09) — one law, per-scenario YAML.** `controller_free.py`,
`controller_pick.py` and `controller_push.py` were three copies of the SAME law (measured
ignoring comments: `make_params`, `dynamics`, `Gains` and the mixer byte-identical;
`MatlabController` differed only in two features `free` had — `gmo_inhibit` and the
saturation-consistent coupling — and the copies had silently drifted apart on
`DLS_LAMBDA`, 0.3 vs `0#.3`). They are now:
- `utils_controller/controller.py` — THE law, no gain defaults in code.
- `utils_controller/control_params.py` + `robotic_arm/config/<scenario>.yaml` —
  every gain and law knob (`free`, `pick`, `push`, `px4_direct`). A missing key is a
  startup error, so a run's parameters always trace to one file. The free demo passes its
  TRAJECTORY TYPE as a `variant`, so a mode can carry its own section.
- the pickplace*/push_home MISSION PLANS moved to `utils_planner` (they are trajectories);
  `build_traj` gained `land_in_plan` (default False — the demo owns the landing).
- `controller_track.py` (posture-anchor baseline) and `controller_hover.py` (ROS2 demo)
  are NOT part of this and stay as they are.
Both moves were proven behaviour-preserving before deleting anything: old-vs-new law over
4 operating points x hold on/off x 3 scenarios, and old-vs-new references over the full
plans at 250 Hz — max difference exactly 0.0 in every case. The px4 rig's `TUNED_GAINS`
dict became `config/px4_direct.yaml` (renamed `px4_direct_free.yaml` 2026-08-10). NOT yet
flown in Isaac.

**PX4 DIRECT-ACTUATOR RIG BECAME THE FREE-FLIGHT PARALLEL OF 02 (2026-08-10, user
request).** `px4_direct_actuator_aerial_manipulator_gmo.py` →
`application/robotic_arm/03_px4_direct_aerial_manipulator_free.py`; launcher →
`scripts/start_px4_direct_aerial_manipulator_free.sh` (tmux session `am_free_px4`, optional
2nd arg = trajectory, forwarded INLINE on the pane command line — an exported var does not
reliably reach an already-running tmux server); gains →
`config/px4_direct_free.yaml`. It is no longer a two-phase hover rig: it now runs 02's FULL
flight machine — setpoint takeoff at the plan's own start point, hover-error-gated handover
(no clock gating), tracked task, hover-gated setpoint landing, rotor ramp-off, auto-close —
plus 02's Q_SPAWN arm-spawn + rigid re-seat + ground-seat, two-panel terminal frame (with an
extra ROTOR cmd/real row), waypoint markers, flight-volume report and npz phase table. Kept
PX4-specific: PX4-primary spawn, `LaggedQuadraticThrustCurve` (the MOTOR DELAY MODEL,
lambda = 10.51), omega published to the external bridge, the armed+OFFBOARD engagement hold
(phase 0), the 3.416 kg body override mirrored into the control model, and the
`omega_cmd`/`omega_plant`/`omega_real` diagnostics. Dropped to match 02's scope: the
EE-force disturbance block, the debug-draw arrows, `ARM_ALWAYS_PD_HOLD`.
- **TUNING RESULT — the failure was the BODY ATTITUDE LOOP, not the arm gains** (20 headless
  full-stack flights, `utils/px4_gmo_gain_sweep.py`, now `--traj`-aware and scoring per
  PHASE off the npz phase table). Every crash was the same ~2.7 Hz mode, in all four joints
  AND the body attitude at once, growing from the moment the law took the arm with a MOVING
  reference. Cause: with `M_r_d = I0` the attitude loop's `wn = sqrt(k_R/0.065) = 11.1 rad/s`
  at `k_R = 8` sits ON the rotor-lag pole (10.51), so the moment commanded to cancel the
  arm's reaction arrives ~90 deg out of phase and pumps the mode. At `k_R = 8` poly_whole
  crashed at `K_y` = 8, 24, 60, 120, 150 AND 300, and the single `K_y = 200` completion did
  NOT repeat — that was a marginal system's run-to-run scatter (the stack is wall-clock PX4
  + DDS, not deterministic), which is why every candidate is now repeat-tested. `k_R` 8→4,
  `k_w` 2→1.5 (wn 7.85, same zeta) flew at `K_y` 8, 60 AND 200 — a 25x range — and repeats
  3/3 within 3%. Because this is a PLANT property, k_R/k_w live in `default`, and they
  **supersede the 2026-07-31 hover tuning's k_R = 8** — that was validated with the arm
  hanging at q=0 and a climb-ramp takeoff; in this demo's configuration hover_drone at
  k_R = 8 diverges ~4 s after the handover, while k_R = 4 holds 90 s at 0.5 mm CoM rms /
  1.0 mm EE. `default` also gets `tau_max` 4.1→3.0 so the law and the demo clamp at the
  same number. The `poly_whole` section then carries ONE deviation, a tracking choice:
  K_y 200 (free.yaml's stiffness; 60 → 0.6 mm and 8 → 2.5 mm pinned EE also fly 3/3) with
  **D_y 24**, which is NOT optional — free.yaml's D_y 6 crashes and 12 crashes, 24 and 48
  fly. `K_o` stays 0.5 on all ten channels (raising the six BODY channels to 1.0 crashes:
  the GMO books the rotor lag as a phantom disturbance). Measured on the shipped config,
  4/4 complete flights: CoM rms 7 mm, EE rms 0.1 mm, pinned-phase EE 0.2 mm, |e_R| max
  0.009, peak arm torque 1.4 of 3.0 N·m, touchdown at exactly 0.305 m, arm back at home.
- **Trajectory de-aggression for this plant** via a new planner hook,
  `utils_planner.traj_config` + `cfg_overrides=` on `build_traj`/`initial_arm_pose` (unknown
  key = error, except documented-optional `q_hold`): the demo's `TRAJ_CFG_OVERRIDE` holds
  `pin_mode "combo"→"yaw"`, phases 6→9 s (peak |v| 0.75→0.50 m/s, |a| 0.48→0.21 m/s²) and
  `beta_home` 86→80 deg. The catalogue is UNTOUCHED, so 02 keeps its validated numbers.
  beta_home is a JOINT-STOP fix: at 86 the law's ~3.5 deg overshoot folding back to home
  put q3 at 49.4-49.7 deg against the +50 hard stop on every flight; 80 leaves ~4 deg.
- **CATALOGUE DEFECT FOUND, NOT FIXED (02 flies it today)**: `TRAJ_CONFIG["poly_whole"]`'s
  `pin_mode: combo` + `pin_phase_deg: 90` starts the pinned fold at `beta_work + 30` while
  the fly-in ENDS at `beta_work`, so the plan is DISCONTINUOUS at BOTH pinned joins — a
  measured 61 mm step in `x_cd` and 16 deg in the recovered arm pose, twice per flight. The
  EE reference is continuous across it, which is why an EE-error metric never showed it.
  `pin_mode: yaw` measures 0.0 mm / 0.0 deg. Fixing the catalogue needs its own 02 flight.
- **A ground-seated vehicle needs a FOLDED arm.** `hover_drone` has no `q_hold`, so the arm
  hangs at q = 0, which reaches 0.282 m below the base — below the legs at the 0.305 m
  resting height. Seated, the vehicle rests on its gripper and tips over during the
  pre-engagement wait (measured: rolled 74 deg, 0.57 m off, before PX4 armed). The old
  midair-spawn version never met this. Fixed by giving hover_drone the same folded home
  through `TRAJ_CFG_OVERRIDE`; the demo also WARNS at startup when the spawn fold
  `beta = q2+q3 < 40 deg`.

**Known checklist deviations, not yet fixed** (see "Checklist for adding a new vehicle model"
below — `utils_vehicle/x650_vehicle.py`/`x650_multirotor.py` themselves are compliant, only `controller.py`
deviates):
- `controller.py:774` — ROS2 node name is hardcoded (`"matlab_aerial_manipulator_controller"`),
  not parameterized by vehicle/uav id. Fine for today's single-vehicle demo; will collide if two
  arm instances ever run in the same ROS2 domain.
- `controller.py:961` — `rclpy.init(args=args)` is called bare, not wrapped in try/except per
  this repo's "already initialised is not an error" rule; will raise if `rclpy` is already
  initialized by a host process.

## X650 airframe reused standalone (no arm) for a stock PX4 slung-load scenario

`application/slungload/px4_single_drone_payload_x650.py` (new) — a **different, simpler** use
of the X650 platform than the aerial-manipulator scenario above: the bare X650 airframe (no
robotic arm) flown as a plain PX4 SITL quadrotor via the **stock** `pegasus.simulator`
`Multirotor`/`MultirotorConfig` (no `VehicleMod`/`MultirotorMod` subclassing), carrying a
fixed-length slung-load payload — structurally the same pattern as
`application/slungload/01_px4_single_drone_payload.py`, just with the X650 asset swapped in for
Iris. This works unmodified because the new asset follows the stock naming convention the
`Multirotor` class hardcodes (`/body`, `/rotor0`..`/rotor3`, `joint0`..`joint3`).

- Active asset: `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/x650_new.usd`
  (~54MB) — **not** the same file as the aerial-manipulator's
  `AM_realign.usda` (~255MB) in that same `assets/` directory; `x650_new.usd` is the bare frame only
  (stock rotor/body naming, no arm links), while `AM_realign.usda` is the full quadrotor+arm
  asset with the custom prim structure `VehicleMod`/`MultirotorMod` are built to reference. Both
  are covered by the `extensions/fsc_aerial_manipulation/**/assets/` `.gitignore` rule (large
  binaries, distributed out-of-band via Google Drive, not committed). The legacy bare-frame file
  has the wrong frame direction and is retained only as historical data; no code, launcher,
  environment variable, or recovery procedure may select it.
- PX4 is the primary backend here (motor authority), with a ROS2 backend for state-publishing
  only (`sub_control: False`) — same division as every other `application/slungload/` scenario,
  and the opposite of the aerial-manipulator scenario (pure ROS2, no PX4).
- Note the filename breaks the `NN_px4_...` numbering convention every other file in
  `application/slungload/` follows (`01_`..`06_`) — not fixed here since it wasn't asked for,
  just worth knowing if you're looking for it by number.

**MN4014+15x5" thrust-curve calibration applied (2026-07-13)** — the legacy asset's structure was
inspected first (see
`docs/propeller_testing/` below for the bench data this uses): naming/articulation pattern
matches the known-working `iris.usd` exactly (`ArticulationRootAPI` on the non-rigid-body root
Xform, `RigidBodyAPI` on `/body` and each `/rotorN`). Its frame direction and motor/propeller assignment
were later found to be wrong; `x650_new.usd` corrects both and is now the sole source of geometry
and rotor ordering. The corrected joint anchors place each motor at approximately
`(±0.22990663, ±0.22990663)` m, giving a 0.325137 m arm length; the pure-Python allocation model
uses those values. The original calibration's vehicle mass was
then set to 3.5kg total (`/x650/body/body`'s authored mass 1.467kg → 3.416kg, so the 4 rotors'
existing ~0.021kg each bring the total to exactly 3.5kg; diagonal inertia scaled by the same
mass ratio, ×2.328, assuming similar mass distribution). A backup
was left alongside the legacy asset for historical comparison, not as a valid model to restore.

**Inertia halved again (2026-07-13), mass unchanged** — user's own follow-up correction: scaling
inertia by the same ratio as mass assumed the added ~2kg is distributed like the rest of the
body, but if it's more centrally concentrated (e.g. batteries mounted near the CG rather than
spread over the frame's full extent), actual inertia wouldn't scale as much as mass did. Diagonal
inertia halved from the mass-ratio-scaled value: `(0.1345, 0.1492, 0.1513) → (0.0673, 0.0746,
0.0757)` kg·m² (mass stays 3.416kg). Historical snapshots of these intermediate values must not
be restored as flight assets. This was part of diagnosing the
rotor-lag-induced PX4 oscillation (see below): testing whether the earlier proportional-scaling
assumption on inertia, not just the actuator lag itself, was contributing to the instability,
before considering retuning PX4's rate/attitude gains directly.

The bench-fit throttle→RPM/thrust/torque polynomials (`docs/propeller_testing/` below) were
then converted into the sim's actual thrust-curve interface:
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/x650_bare_frame_utils.py`
  (new) — factored `spawn_x650_with_mavlink(...)` (bare X650, stock `Multirotor`) out of
  `px4_single_drone_payload_x650.py` into a shared module so the calibration constants live in
  exactly one place; that script now just imports it. Distinct from the arm-scenario's
  `x650_rotorcraft_utils.py` (spawns the *arm-equipped* X650 on `MultirotorMod`/`VehicleMod` —
  different USD asset, different vehicle classes).
- Calibration derivation: `QuadraticThrustCurve` takes rotor angular velocity **ω (rad/s)** and
  computes `force = rotor_constant·ω²`, yaw torque via `rolling_moment_coefficient·ω²·rot_dir` —
  an instantaneous model, **no rotor spin-up lag** (`quadratic_thrust_curve.py`'s own comment:
  "no delay introduced"). Only the Step 1 throttle→RPM/thrust/torque polynomials were used at
  first; **the report's Step 3 λ (spin-up bandwidth) model was added later** (2026-07-13, user
  report: it "significantly affects the dynamics") — see below.
  - `rpm(x) = 70.2593x + 781.49` (x = throttle 0–100%) is linear, and so is PX4MavlinkBackend's
    own `omega = (controls + input_offset)·input_scaling + zero_position_armed` map (`controls`
    = PX4's normalized 0–1 motor command) — so the two were matched **exactly** (not just at the
    endpoints): `zero_position_armed = rpm(0)·2π/60 = 81.8374 rad/s`,
    `input_scaling = (rpm(100) − rpm(0))·2π/60 = 735.7537 rad/s`, replacing the stock
    Iris-tuned defaults (`input_scaling=1000`, `zero_position_armed=100`).
  - `thrust(x)`/`torque(x)` are quadratic/cubic in throttle, but `QuadraticThrustCurve` only
    supports a pure `k·ω²` form (through the origin) — so `rotor_constant`/
    `rolling_moment_coefficient` were derived via an **unweighted zero-intercept least-squares
    fit**, sampling the bench polynomials across throttle 0–100% and regressing against ω²
    (no raw CSV data available in this repo to fit directly against ω² — see the propeller
    bench-test section below). Fit quality: thrust RMSE 0.31N (range 0–30N, ~1%), torque RMSE
    0.0136 N·m (range 0.01–0.52 N·m, ~2.6%) — the fixed-through-origin sim model can't capture
    the bench fit's small nonzero throttle=0 intercept, but tracks it closely elsewhere.
  - **Bug found on first live test (2026-07-13) and fixed**: props were visibly spinning on the
    ground before arming. Root cause: `QuadraticThrustCurve.update()` applies `min_rotor_velocity`
    as an **unconditional floor** (`velocity = np.maximum(min_rotor_velocity, ...)`), regardless
    of armed state — unlike `zero_position_armed` (`PX4MavlinkBackend`), which is correctly
    armed-gated (`handle_control()` calls `zero_input_reference()`, forcing true zero, whenever
    PX4 reports disarmed). The first version of this calibration reused the same 81.8374 rad/s
    idle value for *both* — forcing rotors to spin at that floor even disarmed. Fixed by
    splitting them: `zero_position_armed=81.8374` (unchanged, armed-idle-throttle behavior is
    physically correct and desired) but `min_rotor_velocity=0.0` (matches stock Iris's own
    default, restores the sim's existing disarmed→true-zero gating). Not a curve-fitting
    problem — the bench-measured nonzero idle RPM at 0% throttle is real hardware behavior and
    didn't need to be altered; it was being applied through the wrong one of two similar-looking
    but differently-gated parameters.
  - Resulting total max thrust (4 rotors) ≈118.8N against the new 3.5kg vehicle weight (34.3N)
    → thrust/weight ≈3.46:1, a physically sensible margin for this motor class.
- **Rotor spin-up lag added (2026-07-13)** —
  `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/lagged_thrust_curve.py`
  (new): `LaggedQuadraticThrustCurve` subclasses the stock `QuadraticThrustCurve` (not modified —
  Iris and every other vehicle stay on the instantaneous stock class) and replaces its
  `velocity = clip(input_reference, min, max)` instantaneous jump with a first-order lag,
  `omega_dot = rotor_lambda*(omega_cmd - omega)`, directly implementing the bench report's Step 3
  model (`RPM_rate = λ·(RPM_cmd − RPM)`), previously left unapplied. Force is always computed
  from the lagged `omega`, never from the raw commanded/target velocity directly.
  - **Per-step update is the exact discrete (zero-order-hold) solution, not Euler integration**
    (user-requested fix, 2026-07-14): `omega[k+1] = omega_cmd[k] + (omega[k] -
    omega_cmd[k])·exp(-rotor_lambda[k]·dt)`. Euler integration is only *conditionally* stable
    (diverges into growing oscillation once `rotor_lambda·dt` exceeds ~1–2 — confirmed
    numerically: at `rotor_lambda·dt=2.4`, Euler oscillates ±4500 rad/s around an 800 rad/s
    target while the exact update converges smoothly); the exact form is unconditionally stable
    for any `rotor_lambda, dt > 0` and stays exact even if `rotor_lambda` later varies with
    operating point (as long as it's treated as constant within one step — the usual
    zero-order-hold assumption). Not just theoretical here: the exact update also matches the
    true continuous-time analytic solution much more closely at normal operating `rotor_lambda`
    (confirmed numerically: ~3.4x smaller error than Euler after the same number of steps).
  - `rotor_lambda` default `10.51 (1/s)` is the mean of the report's `lag_all`/`lag_filtered`
    variants (10.519/10.501) — not `direct_all`/`direct_filtered` (noisier, up to ~24%
    unphysical negative λ values, NRMSE>1 — see the propeller bench-test section below).
    `τ=1/λ≈95ms`, inside the report's own physically-expected 70–200ms range.
  - Wired into `x650_bare_frame_utils.py` in place of `QuadraticThrustCurve`
    (`X650_ROTOR_LAMBDA=10.51`), so all three X650 scenarios that share this spawn helper (bare
    drone, fixed-length-cable payload, variable-length-cable payload) pick it up automatically.
  - **Resolved (2026-07-14): the drone couldn't fly normally after this was added — root cause
    confirmed as PX4's untouched generic `none_iris` rate/attitude gains, not a bug in the lag
    model itself** (separately verified correct via standalone step-response checks, and the
    Euler-vs-exact-discrete-update fix below). User reported diverging rate/attitude
    oscillation at the bench-measured `λ=10.51`.
    - **Per-step lag update switched from Euler to the exact discrete (zero-order-hold)
      solution** (user-requested, 2026-07-14): `omega[k+1] = omega_cmd + (omega[k] -
      omega_cmd)·exp(-λ·dt)`, unconditionally stable for any `λ,dt>0` (Euler integration is
      only conditionally stable — confirmed numerically: at `λ·dt=2.4`, Euler diverges into a
      growing ±4500 rad/s oscillation around an 800 rad/s target while the exact update
      converges smoothly). Applied in `lagged_thrust_curve.py`; force is still always computed
      from the lagged `omega`, never from the raw commanded velocity directly.
    - **Diagnostic λ sweep** (against PX4's then-still-untouched default gains,
      `MC_*RATE_K=1.0`): `10.51` (bench value) diverged, `12.0` crashed outright, `13.5`/`14.5`
      both oscillated (marginal), `15.0`/`15.51` flew cleanly. This confirmed a lag-vs-
      control-loop-tuning mismatch, not a model bug — but `15.51` isn't the physically accurate
      value, just a stability-driven workaround.
    - **X650 CAD inertia correction**: the user provided the actual CAD (OnShape) "Mass and
      section properties" panel for the X650 frame part. Its `Lxx/Lyy/Lzz` (kg·mm², ÷1e6 for
      kg·m²) — `(0.057775, 0.065004, 0.064082)` — matched the legacy asset's **original, pre-edit**
      authored inertia `(0.057775, 0.064082, 0.065005)` for `(Ixx,Iyy,Izz)` almost exactly (Y/Z
      swapped between CAD's and USD's axis labeling) — confirming the asset's original inertia
      was already correctly CAD-sourced, and the earlier `×2.328` mass-ratio scaling (done when
      total mass was set to 3.5kg) was the wrong move, not a refinement. A same-ratio "halve the
      scaled inertia" diagnostic (an intermediate, still-not-CAD-accurate value) was tried first
      and made flight *worse*, which in hindsight makes sense — it wasn't testing the real CAD
      value. Fixed by setting `/x650/body/body`'s diagonal inertia to the exact original
      CAD-derived values (`0.05777498, 0.06408172, 0.065004565`) while **keeping mass at
      3.416kg** (ignoring CAD's own 1.467kg reference mass, per explicit user instruction — the
      extra ~2kg is assumed not to significantly change the rotational inertia distribution). A
      historical backup files are diagnostic records only and must not be restored into an
      active scenario because their model frame is wrong.
    - **X650 PX4 gain tuning — final working configuration**, confirmed live against the
      corrected CAD inertia and the true bench λ=10.51:
      | Parameter | Stock default | X650 tuned value |
      |---|---|---|
      | `MC_ROLLRATE_K` / `MC_PITCHRATE_K` / `MC_YAWRATE_K` | 1.0 | **0.3** |
      | `MC_ROLL_P` / `MC_PITCH_P` | 6.5 | **3.25** |
      | `MC_YAW_P` | 2.8 | **1.4** |

      Reasoning: `*RATE_K` is an overall multiplier on that axis's whole rate-loop P/I/D/FF
      stack — the cleanest lever to uniformly soften the loop without rebalancing P/I/D ratios
      individually, needed because the added actuator lag reduces the loop's available phase
      margin. The attitude-loop `P` gains needed reducing too, found empirically after rate-only
      softening (down to `0.2`) stopped helping — with the inner (rate) loop now much slower,
      the outer (attitude) loop's unchanged gain was commanding rate corrections faster than the
      softened inner loop could actually deliver, a classic cascade-control mismatch (outer loop
      bandwidth must stay meaningfully below inner-loop bandwidth). Halving both attitude `P`
      gains alongside the rate gains resolved this.
      - `scripts/apply_x650_px4_gains.sh` (new) — applies and `param save`s this exact
        configuration to a running PX4 SITL instance via `tmux send-keys`, so it doesn't have
        to be re-typed by hand every session. Called (backgrounded, `&`) from all three X650
        launch scripts (`start_x650_single_drone.sh`,
        `start_single_drone_sitl_payload_x650.sh`,
        `start_single_drone_sitl_payload_variable_cable_x650.sh`) a fixed 8s after PX4 is
        expected to have booted to its `pxh>` prompt — `tmux send-keys` has no way to detect
        readiness, so keys sent too early are silently dropped.
- `application/px4_base/03_px4_single_drone_x650.py` (new) — bare X650 (no payload), mirrors
  `01_px4_single_drone.py`'s exact structure with Iris swapped for the calibrated X650 via
  `spawn_x650_with_mavlink`. **`01_px4_single_drone.py`/Iris itself was not touched** by any of
  this work — verified via `git status` on the upstream `extensions/pegasus.simulator/` tree.
- `scripts/outdoor_sim/start_x650_single_drone.sh` (new) — mirrors `start_single_drone_sitl.sh` exactly,
  pointing at the new bare-drone script above; now also calls `apply_x650_px4_gains.sh` (see
  "X650 PX4 gain tuning" above) 8s after launch.
- `scripts/indoor_sim/start_single_drone_x650.sh` (added 2026-07-25) runs the same calibrated
  bare-X650 plant and measured `lambda=10.51 1/s` rotor lag with PX4-owned control, but combines
  the validated X650 gains with the indoor external-vision estimator settings before EKF2 starts.
  It keeps those parameters in the independent `rootfs_fsc_indoor_x650/` work directory and
  verifies Isaac pose and inertial-velocity ROS 2 topics after startup.
- `rotorcraft/assets/x650_new.usd` became the active bare-X650 asset on 2026-07-25. It corrects
  the model rotation and PX4 Quad-X motor/propeller order: rotors 0/1 are the front-right/rear-left
  counter-clockwise pair and rotors 2/3 are the front-left/rear-right clockwise pair. The new file
  authors only 1.467368 kg on its body mesh, so `x650_bare_frame_utils.py` overrides that mesh at
  spawn time with the established 3.416079 kg body mass and
  `(0.05777498, 0.06408172, 0.065004565) kg*m^2` diagonal inertia. Together with the four authored
  rotor masses, total simulated vehicle mass remains 3.5 kg. Do not copy the new asset's authored
  body mass into controller or calibration constants.
- Note: `scripts/indoor_sim/start_single_drone_sitl_payload_x650.sh` already existed (pre-dates this
  session's work, launches the payload variant) but lacks the tmux kill-pane cleanup pattern the
  other launch scripts in this repo have — not fixed here since it wasn't asked for. It does now
  also call `apply_x650_px4_gains.sh`, same as the other two X650 launch scripts.

**X650 variable-length cable variant added and confirmed working (2026-07-13)** —
`application/slungload/px4_single_drone_payload_variable_length_cable_x650.py` (new): same
winch mechanism as `02_px4_single_drone_payload_variable_length_cable.py` (rod_a/rod_b +
prismatic joint + `ROS2CableWinchBackend`), with Iris swapped for the calibrated X650 via
`spawn_x650_with_mavlink` — same swap pattern as the fixed-length-cable variant above. Payload
mass (`rod_b_mass+payload_mass=0.565kg`) is unchanged from the Iris version — that constraint
comes from `cable_torque_ctrl_node`'s deployed mass param (AK40-10 side), not the carrying
vehicle's lift capacity, so it doesn't change just because X650 has far more thrust margin.
`scripts/indoor_sim/start_single_drone_sitl_payload_variable_cable_x650.sh` (new) mirrors
`start_single_drone_sitl_payload_variable_cable.sh` exactly, pointing at the new script, and also
calls `apply_x650_px4_gains.sh`. Neither the Iris variant nor its launch script were touched
(verified via `git status`).

## Propeller bench test data (`docs/propeller_testing/`)

Real motor+propeller bench-test reports — reference data, not code. `MN_4014_15x5` is wired into
the X650 vehicle's thrust curve and `MN_4010_15x5` into the T650's (see below);
`MT_2216_10x4.5` is not yet connected to any vehicle config.

- `MT_2216_10x4.5_report.pdf` (generated 2026-06-05), `MN_4014_15x5_report.pdf` (generated
  2026-07-12) and `MN_4010_15x5_report.pdf` (generated 2026-07-15) — one report per motor+prop
  combo tested.
- Each report gives: **Step 1** polynomial fits of throttle → RPM/Thrust/Torque (with R², all
  ≥0.996 in both reports); **Step 2** throttle→RPM response lag lookup tables; **Step 3** a
  first-order RPM-response bandwidth model, `RPM_rate = λ·(RPM_cmd − RPM)`, comparing 4
  estimation variants (`direct_all`/`direct_filtered`: λ from the model equation at each sample;
  `lag_all`/`lag_filtered`: λ = 1/τ from the measured step-response lag directly — a
  system-ID estimate independent of the model equation).
- The methodology page lists companion CSV outputs (`global_fit_coeffs.csv`,
  `lag_vs_throttle_rate.csv`, `lambda_lookup_<variant>.csv`, etc.) — **only the summary PDF
  reports are currently in this repo, not the underlying CSVs.**
- **`MN_4014_15x5` is now fully connected**: its Step 1 throttle→RPM/thrust/torque polynomials
  *and* its Step 3 spin-up-lag (λ) model are both applied to the X650 vehicle — see the X650
  section above for the full derivation, `x650_bare_frame_utils.py`'s calibration, and
  `lagged_thrust_curve.py`'s `LaggedQuadraticThrustCurve`. `MT_2216_10x4.5` remains unconnected
  to any vehicle config as of this writing.
- **`MN_4010_15x5` is fully connected too, to the T650** (2026-08-05; constants moved into
  `t650_params.py` 2026-08-06) — see the T650 section below. **The fit procedure was validated
  before use**, twice: re-running it against the MN4014 polynomials reproduces that motor's
  committed `k`/`c` to **+0.004% / +0.015%**, and the MN4010's to **+0.003% / +0.018%**, so the
  motor sets are directly comparable and not fitted by different methods. Do the same before
  adding a third. The fit is an unweighted zero-intercept least-squares of the report's Step 1
  polynomials sampled uniformly across throttle 0–100%, projected onto the simulator's fixed
  `force = k·ω²` / `torque = c·ω²` form.
- **The T650's shipped `k`/`c` are no longer the bench values** — both carry a named empirical
  fit factor (T650 section). `BENCH_ROTOR_CONSTANT` / `BENCH_ROLLING_MOMENT_COEFFICIENT` in
  `t650_params.py` are the unmodified report-derived numbers.

## T650 (Tarot 650): X650 geometry + MN4010 motors (added 2026-08-05, restructured 2026-08-06)

A second vehicle sharing the bare-X650 asset, added for the FSC Lab Tarot 650 build. The USD
geometry, rotor ordering and arm length are inherited unchanged — there is no separate T650
asset — but as of 2026-08-06 **the T650 has its own parameter and spawn modules** rather than
borrowing the X650's.

| | MN4014 (X650) | MN4010 (T650) |
|---|---|---|
| ω idle → max | 81.8374 → 817.5911 rad/s | **64.0603 → 730.0507 rad/s** |
| `rotor_constant` k (bench) | 4.536223e-05 | **4.540431e-05** → tuned **4.679931e-05** |
| `rolling_moment_coefficient` c (bench) | 8.366000e-07 | **8.247173e-07** → tuned **2.474152e-06** |
| λ (spin-up bandwidth) | 10.51 (τ 95.1 ms) | **10.0265 (τ 99.7 ms)** |
| body / total mass | 3.416079 / 3.5 kg | **2.95 / 3.033921 kg** |
| hover command | 0.4800 | **0.5025** |

Three things that look like bugs but are not:

- **The T650 hovers ~22 points higher on the stick** (0.5025 vs 0.4800). Its top rotor speed is
  lower at a nearly identical thrust constant, so the same weight sits higher. Static
  thrust-to-weight is still 3.35.
- **The MN4010 is the SLOWER rotor**, λ 10.0265 vs 10.51 — ~4.8% more lag. Modest, but it costs
  phase margin, so the softened X650 PX4 gains (`MC_*RATE_K=0.3`, `MC_ROLL_P/MC_PITCH_P=3.25`,
  `MC_YAW_P=1.4`) are if anything more necessary here. Do **not** raise λ to buy stability —
  that was tried on the MN4014 (15.51) and is a workaround for untuned gains, not a physical
  value.
- **k and c are no longer the bench values.** Both carry an empirical fit factor (below). The
  bench numbers are still in the file as `BENCH_*`; the tuning is one named factor each.

### 2.95 kg is now the BODY mass, not the total

Until 2026-08-06, `T650_MASS = 2.95` was the vehicle's **total** (body 2.866079 + four authored
rotor bodies 0.083921). On instruction it is now the **body** mass, so the total the solver sees
is **3.033921 kg**. That is a real change to the plant, and it is what forced the thrust-constant
fit below: at the old total the bench k reproduced the measured hover to +0.14%, and the
0.503188 hover command was confirmed four separate times (two in simulation, two against
hardware). If the intent was ever 2.95 kg total, set `BODY_MASS = 2.95 - ROTOR_MASS_TOTAL` and
`THRUST_FIT_FACTOR = 1.0`; every derived quantity recomputes automatically.

### Empirical tuning against flight C (2026-08-06)

Fitted to `docs/experimental_data_ros2_bag/debug_recording_20260806_164742` replayed through the
same `fsc_autopilot_ros2` stack. **Parameters only — no modelling was changed, and mass was not
touched.** Full method, before/after metrics and limitations: `docs/sim_to_real_t650/TUNING_t650.md`.

| factor | value | what it does |
|---|---|---|
| `THRUST_FIT_FACTOR` | ×1.030724 on k | restores the measured hover command (0.5025) at the now-fixed 3.033921 kg |
| `YAW_TORQUE_FIT_FACTOR` | ×3.0 on c | stands in for a **missing model term** (below); yaw shape RMSE 4.109° → 1.268° |
| — | ×1.030724 on Ixx, Iyy | bookkeeping only: roll/pitch authority ∝ k/I, so this holds that (already good) fit exactly where it was. Izz deliberately NOT scaled — yaw is corrected through c, scaling both would double-count |

**`c` is an effective yaw-torque coefficient, not a drag coefficient.** The simulated yaw axis is
under-powered because the thrust curve applies only the steady drag torque `c·ω²` and never the
reaction to the rotors' own angular acceleration, `I_rotor·ω̇`, which during a yaw command adds
across all four rotors and is 2–4× larger than the modelled torque. Inflating c is a stand-in.
**Revert c to `BENCH_ROLLING_MOMENT_COEFFICIENT` if `LaggedQuadraticThrustCurve` ever gains that
term** — that is the real fix, and ×3.0 measures how much torque is missing. Only the ratio
c/Izz sets yaw dynamics, so this could equally have been done by lowering Izz; c was chosen
because Izz has CAD support and matching by inertia alone would need Izz ≈ 0.029 kg·m², under
half any physical estimate.

Where things live:

- `extensions/.../rotorcraft/t650_params.py` — **new**, parallel to `x650_params.py` and
  importing nothing from it, so a change to one vehicle cannot silently move the other. Holds the
  MN4010 calibration (with its derivation and the fit validation), the mass model, the shared
  rotor geometry, and hover/scaling values **computed from those constants rather than hard-coded**
  so they cannot go stale. Pure Python, no Isaac imports, same as `x650_params.py`.
  Inertia is copied from the X650 CAD diagonal on the "mass difference is centrally concentrated"
  assumption. It IS an assumption; replace it if the T650 gets its own CAD.
- `extensions/.../rotorcraft/t650_bare_frame_utils.py` — **new**, `spawn_t650_with_mavlink()`.
  Standalone; shares only the USD asset (`assets/x650_new.usd`, referenced by its own
  `T650_USD` constant) and overrides the asset's authored 1.467 kg body mass at spawn.
- `application/px4_base/05_px4_single_drone_t650.py` — the T650 app. Calls the helper with **no**
  mass/motor arguments, so every T650 number lives in exactly one place.
- `scripts/indoor_sim/start_single_drone_t650.sh` — thin wrapper over the X650 launcher.
- `scripts/indoor_sim/start_t650_direct_actuator_sitl.sh` — direct-actuation variant, lockstep
  disabled and pushed with `tmux setenv -g` (same fix as the X650 version).

`x650_bare_frame_utils.py`'s `MOTOR_CALIBRATIONS["MN4010"]` entry and the `motor=` argument are
now **unused** — the T650 no longer routes through them. They were left in place rather than
removed, so be aware there are two spawn paths for the same vehicle and they will drift; prefer
the `t650_*` modules and delete the table entry when convenient.

`start_single_drone_x650.sh` is now the shared orchestration for both vehicles, via three
env hooks whose defaults reproduce the X650 exactly: `INDOOR_SIM_PEGASUS_SCRIPT`,
`INDOOR_SIM_VEHICLE_LABEL`, `INDOOR_SIM_PX4_PROFILE`. The last is a directory **name**, not a
path, so a variant wrapper does not need `PX4_DIR` resolved before it runs (it isn't —
`common_config.sh` only defines it inside `load_machine_config`, which the base launcher calls).
**Each vehicle must keep its own PX4 profile**: PX4 `param save`s into it, so sharing one would
silently carry one airframe's tune into the other.

**`SYS_HAS_MAG=0` was missing from the indoor launcher until 2026-08-05** and is now set for
both vehicles. The indoor scenario already ran `EKF2_MAG_TYPE=5` (None), so declaring no
magnetometer is the consistent setting; it is also on the required indoor list in
`fsc_drone_state_estimator_ros2/docs/px4_indoor_paramters.md`. It is a boot-time parameter — a
live `param set` will not take effect until PX4 restarts.

**Validation, 2026-08-05** (against the then-current 2.95 kg **total** mass — historical, see the
mass note above). Two independent paths, both agreeing with the calibration:

| path | measured hover | predicted 0.503188 |
|---|---|---|
| QGC/PX4 mixer (`commander takeoff`, GPS profile) | 0.503341 | 0.030% |
| `fsc_autopilot_ros2` baseline node (indoor EV, full stack) | 0.50320 | 0.002% |

The first flew a full takeoff → 2.5 m hover (vz 0.0024 m/s, roll 1.3°, pitch 0.1°, body rates
~0.002-0.004 rad/s) → land. The second held a commanded 1.0 m at 0.9963 m (0.37 cm error,
0.59 cm spread) with lateral hold inside 1.5 cm. Real hardware later agreed too: flight B's
command averages 0.5033 over its first 5 s and flight C's 0.5025 over t=5–15 s, both before
battery sag accumulates.

Two things learned while running it, both easy to lose an hour to:

- **`ekf2 stop` on a running lockstep sim deadlocks the PX4↔Isaac HIL link** (`ERROR
  [simulator_mavlink] poll timeout 0, 111`). Do not restart the estimator in place to pick up
  EKF2 parameter changes — relaunch the sim instead.
- **Isaac Sim crashes at ~800 ms under `PEGASUS_HEADLESS=1`** on this machine (breakpad,
  `std::__throw_system_error` out of `pthread_create`), well before any scenario code runs. Run
  with the display; `DISPLAY=:0` and an X11 session are available.

## Sim-to-real validation, T650 indoor flights (`docs/sim_to_real_t650/`, 2026-08-06)

Three recorded indoor T650 flights replayed through the **same** `fsc_autopilot_ros2` baseline
stack in IsaacSim and compared against hardware. Bags live in
`docs/experimental_data_ros2_bag/` (not committed-friendly: ~48 MB total).

| bag | what it supports |
|---|---|
| `debug_recording_20260806_134620` (A, 117 s) | `sensor_combined` only — no measured position |
| `debug_recording_20260806_140303` (B, 169 s) | same |
| `debug_recording_20260806_164742` (**C**, 211 s) | **also records `state_estimator/local_position/odom`** → the real position/velocity/attitude comparison |

Read `REPORT_flightC.md` + `TUNING_t650.md` first; `REPORT.md` (flights A/B) is marked stale for
its step numbers but remains the reference for vibration and battery sag.

**Method that makes it like-for-like.** The recorded `position_controller/reference` waypoints are
replayed into the live stack and `fsc_autopilot_ros2` regenerates its own attitude setpoints —
the real flights' setpoints are never injected, they are the thing being predicted. Simulation
takes off to the sequence's first waypoint and settles before t=0 (2.0–2.8 cm error), so the
initial condition matches the mid-flight bags.

**Findings that are about the simulator, not this campaign:**

- **Yaw is the one axis the plant gets materially wrong** — before tuning: overshoot 44.9% vs
  9.2%, rise 0.840 s vs 0.555 s, shape RMSE 4.11° (20% of a 20° step). Root cause is the missing
  `I_rotor·ω̇` term (see the T650 tuning section). Independent corroboration: **PX4's own autotune
  cannot identify the yaw axis** — it returns 5.6× less response per unit excitation than
  roll/pitch and times out at 20 s, while roll and pitch converge in ~5 s each
  (`figures/autotune_diagnostic.png`). This is the third independent confirmation, after the X650
  campaign, that yaw gains tuned in simulation must not be carried to hardware.
- **Vibration is not modelled at all.** Real airframe accelerometer RMS above 2 Hz is 1.3–1.7
  m/s²; simulated is 0.036 m/s², i.e. **sim is 2–6% of real**. Anything depending on IMU noise —
  filter/notch tuning, vibration failsafes, EKF innovation gating — cannot be developed here.
  Below 2 Hz the rigid-body motion agrees (gyro within 1.07–1.53×).
- **Battery sag is now demonstrated, not hypothesised.** Same controller, same maneuver: the real
  hover command climbs +0.80 %-pts over 95 s (flight A) while simulation is flat to −0.01 %-pts.
  The X650 fidelity doc listed this as "a hypothesis, not a result". Consequence: **fit thrust
  constants to the EARLY-flight hover**, before sag accumulates.
- **Both vehicles hover tilted, in opposite directions** — real roll/pitch −1.31°/+1.07°,
  simulated +0.98°/−1.58°, similar magnitude but 161° apart; on flight C it shows as a constant
  +2.58°/+2.46° offset with correlations still 0.74/0.84 (transients align, datum does not). A
  vehicle holding position with persistent tilt means the attitude estimate's "level" is offset
  from where the thrust axis produces no horizontal force — on hardware IMU/mocap levelling, in
  simulation the USD asset's body frame vs its rotor plane. **Not chased yet**; ~2.5° of built-in
  tilt matters for any trim, disturbance-estimation or UDE study.
- **Position agreement is good and partly luck.** Flight C: shape RMSE 5.5 cm on 1 m steps,
  settling bias +0.004 s. But flights A/B show the *acceleration* that produces it is off by
  +22.6% in peak and +92.7% in rise — two errors that partly cancel when integrated. Do not read
  trajectory-level agreement as inner-loop fidelity.
- **Run-to-run noise floor: position shape RMSE varies ±0.008 m** between two runs of an
  identical configuration. Differences below ~0.01 m are not resolvable from single runs.

### Traps, all of which silently corrupt results

- **The simulator does not run in real time** — 0.84–0.90× measured. A reference schedule issued
  on wall clock is compressed by that factor *in simulation time* (a 90 s sequence became 69.3 s,
  every hold 23% short). `tools/stack_driver.py::sim_now` drives the schedule from
  `sensor_combined.timestamp` instead; achieved spans then match the recorded ones to 23 ms
  over 183 s.
- **In simulation the recorded topics span two clocks.** `sensor_combined` / `vehicle_attitude`
  are PX4-stamped (simulation time); the controller's attitude setpoints are stamped on ROS wall
  clock. Mixing them misplaces every commanded trace by tens of seconds. The odometry record
  carries both per sample; interpolate, do **not** affine-fit (the ratio varies through the run —
  59 ms RMS residual). On hardware this does not arise; PX4 and ROS share a clock there.
- **rosbag2 `messages.timestamp` is the receive time and is bursty** — 50.8 ms jitter against
  odometry's clean 10.00 ms `header.stamp`. Use header stamps. The reference topic's own header
  stamps are on a *different epoch* from odometry's, so reference onsets are recovered by mapping
  reference receive time through odometry's receive→header correspondence (3–4 ms residual).
- **PX4 autosaves parameters a few seconds after a `param set`** — "live-only" tuning is not
  live-only. A yaw gain sweep leaked its last gain set into `rootfs_fsc_indoor_t650`, and because
  `MC_YAWRATE_P/I/D` are **not** in the launcher's `PX4_PARAM_` override list they would have
  survived into every later run (with `MC_YAWRATE_K` silently dropping to PX4's 1.0 default,
  since the swept value matched it and PX4 does not save defaults). Add those three to the
  launcher's override list, or restore explicitly after any sweep.
- **`start_single_drone_x650.sh:77` clones the PX4 rootfs with `cp -a` for any new profile**, and
  that rootfs is **39 GB** of accumulated logs. Creating a scratch profile cost 39 GB of disk.
  Exclude `log/` — or create only the `etc`/`test_data` symlinks plus `eeprom/`, which is all PX4
  needs to boot.
- **`virtual_remote` is service-driven, not interactive.** `/uav_0/rc/{arm,disarm,offboard,rtl}`
  as `std_srvs/Trigger`; there is no `_wait_for_arm_trigger()` and no "press Enter" gate. The
  safety comment at `start_baseline_t650_stack_fused.sh:31-34` is **stale and overstates the
  guarantee** — the pane arms whenever anyone calls the service, script included.
- **PX4 autotune needs `MC_AT_EN=1` set at BOOT** (`rc.mc_apps:23` only starts the module if the
  parameter is already >0), and there is no `MC_AT_AXES` in v1.16 — it sequences roll→pitch→yaw
  and rewrites all three. Use `MC_AT_APPLY=0` to identify without writing. Excitation is injected
  at `mc_att_control_main.cpp:353`, inside the attitude-control block, so it works in OFFBOARD
  too. The FSC Pixhawk firmware is modified indoor-only — **do not run autotune on hardware.**
- **Run long jobs under `tmux`, not `nohup`/`setsid`** — a killed tool call otherwise takes the
  whole process group with it.

### Tooling (`docs/sim_to_real_t650/tools/`)

`extract_odom_bag.py` (bag → npz, header-stamp based) · `stack_driver.py` (drives the live stack
through a recorded reference sequence on the simulation clock; optional `PARAM_SCHEDULE_FILE` env
var fires PX4 console commands at set simulation times, used for gain sweeps) ·
`run_stack_case.sh` (one closed-loop case end to end; **auto-arms**, simulation-only) ·
`plot_odom.py` / `metrics_odom.py` (flight C figures and metrics) · `plot_tuning.py`
(before/after) · `plot_autotune.py` · `extract_bag.py` / `steps.py` / `plot_steps.py` /
`metrics.py` / `sensor_compare.py` (flights A/B, `sensor_combined`-based).

Analysis runs on the **system** `python3` (numpy 2.x + matplotlib); bag extraction needs ROS 2 and
`px4_msgs` sourced.

## Normalized swing state (r, v) — JGCD paper data feed

`extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/utils/swing_state_backend_utils.py`
(new) — `ROS2SwingStateBackend`, publishes the normalized swing state `r`/`v` used by the JGCD
paper at `~/source/JGCD-paper-longhao/main.tex` ("Robust Neural Contraction Slung Load
Manipulation With Active Cable Length Control"), for that paper's still-empty "Simulation
Verification" section.

- **Definitions** (derived from `main.tex`, not guessed): cable vector
  `l_c = x_q - x_p` (quadrotor position minus payload position, main.tex:793), cable length
  `l = ||l_c||`, unit direction vector `n = l_c/l = [r; sqrt(1-r^T r)]` (`eq:def_cable_vector`) —
  so **`r` is the horizontal (x,y) position difference between drone and payload, divided by the
  total 3-D cable length `l`** (`r = (x_q-x_p)_{xy} / l`, i.e. the horizontal components of the
  unit cable-direction vector `n`, equivalently) — normalized by the actual straight-line cable
  length, **not** by the horizontal distance itself or any fixed reference length. Matches
  main.tex's own description: "the 2-D offset of the quadrotor when the cable is 1m long"
  (main.tex:130); dimensionless, bounded by `||r||<=1`. **Not** the winch's own
  extension/extension-rate (already published separately by `ROS2CableWinchBackend` as
  `cable_winch_0/state/cable`) — a different physical quantity (swing geometry vs. cable length
  itself).
- `v = rdot` is computed **analytically**, not by finite-differencing `r` — deliberately, per
  this repo's own established lesson (differentiating a noisy/irregularly-sampled quantity
  amplifies noise; see the AK40-10 emulator's `on_state_cable()` timing bug in the WIP section
  below) and because the closed form is directly available here: `l_dot = n·(v_q-v_p)`,
  `n_dot = ((v_q-v_p) - n*l_dot)/l` (`eq: B_property`'s `B` matrix, whose top block is `I_2`, so
  `n_dot[:2] == rdot` by construction), `v = n_dot[:2]`. `v_q`/`v_p` come directly from
  `dynamic_control.get_rigid_body_linear_velocity` (drone body + payload), never differentiated.
  Verified numerically (standalone, no Isaac Sim needed): analytic `v` matches a finite-
  difference of `r` at `dt=1e-6` to ~1e-7 agreement.
- Publishes two `geometry_msgs/Vector3Stamped` topics (`z` always 0, `r`/`v` are 2-D):
  `<topic_prefix>r` and `<topic_prefix>v`, default prefix `swing_state_{swing_id}/`. Follows the
  same per-instance-unique-node-name pattern as `ROS2CableWinchBackend`
  (`simulator_swing_state_{swing_id}`).
- Wired into **both** variable-length-cable scenarios as `swing_state_0/{r,v}`:
  `application/slungload/02_px4_single_drone_payload_variable_length_cable.py` (Iris) and
  `application/slungload/px4_single_drone_payload_variable_length_cable_x650.py` (X650) — not
  wired into the fixed-length-cable scenarios, since this is specifically paper-verification data
  for the active-cable-length-control system.
- **Bug found on first live test (2026-07-14) and fixed**: `TypeError: Can't instantiate abstract
  class ROS2SwingStateBackend with abstract method update_state`. `Backend`'s ABC only checks
  that a method *named* `update_state` exists on the subclass (not its signature - e.g.
  `ROS2CableWinchBackend`'s own `update_state(self, extension, extension_vel)` has a completely
  different signature from the base class's `update_state(self, state)` and still satisfies it
  fine), and this class simply never defined one. Fixed by adding a no-op `update_state(self,
  state)` — this backend doesn't need it since it independently reads both the drone's and
  payload's rigid bodies via `dynamic_control` in its own `update_sim_state()` physics callback,
  the same pattern `ROS2CableWinchBackend` uses. Affects both scenarios it's wired into (single
  shared class, one fix covers both).
- **Known simplification, flagged to the user, not yet confirmed**: `x_q`/`v_q` are read from the
  drone's own rigid-body prim (`uav_path`, the same one the spherical joint attaches to), i.e.
  the drone's own COM — the small `uav_hook_local` cable-attachment offset from that COM is
  ignored. This matches `main.tex`'s idealized model (`x_q` is just "quadrotor inertial
  position," no hook offset anywhere in the math), so it's the more faithful choice of the two
  options, not an arbitrary shortcut — but if the paper's verification actually wants the real
  physical attachment point instead, this needs revisiting.

## Work in progress: variable-length slung-load cable + AK40-10 Isaac Sim emulator

Design doc: `docs/design_requirements/design_requirements.txt`. Full implementation plan (context,
rationale, verification steps): `/home/longhao/.claude/plans/gentle-crunching-treehouse.md`.

Goal: a slung-load variant whose cable has variable length (2 rigid rods + a prismatic joint,
joint force = cable tension), bridged over ROS 2 to the real AK40-10 winch-actuator stack in
`fsc_autopilot_ws` (`AK40-10-ROS2-Bridge` driver + `fsc_ak_actuator_ground_control` GUI) via a new
"emulator" ROS 2 package that impersonates the real driver's `ak_motor_cable_control_node` so
those two packages work against it **unmodified**.

**Status — Part 1 (Isaac Sim side, this repo) built and force→extension response CONFIRMED WORKING under manual flight test:**
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/constraints/constraints_utils.py` — added `create_prismatic_joint` (no DriveAPI, pure kinematic constraint)
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/slung_load/variable_length_cable_utils.py` (new) — `create_prismatic_rod_pair`, `setup_variable_length_cable_geometry`
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/utils/cable_winch_backend_utils.py` (new) — `ROS2CableWinchBackend`: publishes `cable_winch_0/state/cable` (extension, extension velocity) + `state/force`, subscribes `cable_winch_0/command/force`, applies it as a force pair each physics step
- `application/slungload/02_px4_single_drone_payload_variable_length_cable.py` — populated (was an empty placeholder): drone ↔(spherical)↔ rod_a ↔(prismatic)↔ rod_b ↔(spherical)↔ payload, wired to the new backend
- `scripts/indoor_sim/start_single_drone_sitl_payload_variable_cable.sh` (new) — launch script for this scenario, same tmux/PX4 pattern as `start_single_drone_sitl_payload.sh`

**Runtime test log:**
1. Full stack launches cleanly: PX4 SITL connects over mavlink, Isaac Sim runs with no exceptions/tracebacks, visually the rods/payload behaved correctly (confirmed by user).
2. ROS2 topics (`/cable_winch_0/{state/cable,state/force,command/force}`) all present and publishing; `ros2 topic info --verbose` confirms the subscriber is connected.
3. **Bug found** (ground/static test, no pilot input): commanding a force via `ros2 topic pub .../command/force` had **no effect on extension** — `state/force` correctly echoed the commanded value (proves the ROS2 subscribe/callback path works), but `state/cable` position stayed pinned at ~0.004 regardless of force magnitude/sign/duration (tested sustained pubs up to 8s). Isolated to the physics application step, not ROS2.
4. Ruled out: joint limits sitting exactly on the flush starting position (`lower_limit=0.0` == initial translation) — widened to `[-max_cable_extension, +max_cable_extension]` as a diagnostic, no change.
5. Applied fix: `update_sim_state` was calling `dc.apply_body_force(..., global=True)` with a world-frame force vector — the only other precedent in this codebase (`vehicle.py`'s `apply_force`/`apply_torque`) always uses `global=False` (local frame). Changed to apply the force in each rod's **local** frame (`global=False`) using `self._axis_local` directly (valid since rod_a/rod_b never rotate relative to each other). Also added a throttled debug `print(...)` every 60 physics steps in `update_sim_state` showing `rb_a`/`rb_b` handles, `pos_a`/`pos_b`, `axis_world`, `extension`, and the applied local force.
6. **Re-verified with the sim still on the ground/no pilot input (office machine, `fsc_lab_machine` config)**: still no measurable response to a sustained ±1 N force over 6s+ (`state/cable` position drift ~1e-6 m, well below the ~meters of displacement 1 N should produce on the 0.02 kg rod). Looked like the fix hadn't worked.
7. **Root cause of steps 3/6, resolved**: it was never a frame/API bug — the drone was sitting static/on the ground (not actually hovering), so there was no real gravity-loaded cable, and the test forces (±1 N) were below the weight the joint needs to overcome to move at all. With the drone actually hovering (piloted via QGroundControl) and the payload hanging freely, **contracting the cable requires the commanded force to exceed the combined weight of everything below the joint**: `(rod_b_mass + payload_mass) × g = (0.02 + 0.3) × 9.81 ≈ 3.14 N` with default masses. Manually tested: 1 N and 3 N → no contraction; 3.14 N and 3.5 N → contracts. The cutover landed almost exactly on the computed equilibrium value, confirming this is real physics (gravity/tension equilibrium), not an engine artifact — **force→extension path is working correctly**. `rod_a`'s weight doesn't factor in (drone-side segment, reacted through its own spherical joint).

**Part 1 cleanup: done** — removed the throttled debug `print(...)` from `update_sim_state` (`cable_winch_backend_utils.py`), and reverted the prismatic joint's limits from the symmetric `±max_cable_extension` diagnostic back to the physically-correct `lower_limit=0.0` (can't retract past flush) / `upper_limit=max_cable_extension` in `application/slungload/02_px4_single_drone_payload_variable_length_cable.py`, now that the diagnostic is confirmed to have been a red herring (real cause was the static/grounded-drone test setup, see step 7 above).

**Status — Part 2 (`AK40-10-Isaac-Sim-Emulator`, own repo at `fsc_autopilot_ws/src/AK40-10-Isaac-Sim-Emulator`, pushed to `github.com/FSC-Lab/AK40-10-Isaac-Sim-Emulator`) built, SPEED mode + EXTERNAL mode + requirement #5 all verified working; requirement #6 next:**
- Package scaffolded (own git repo, cloned the layout of `isaacsim_optitrack_ros2_emulator`): `CMakeLists.txt` (+ `src/CMakeLists.txt`), `package.xml` (ROS2 package name `ak_motor_cable_control_emulator`), `config/cable_control_emulator_params.yaml`, `launch/cable_control_emulator.launch.py`, `src/ak_motor_cable_control_node.cpp` (node name `ak_motor_cable_control_node`, the drop-in name the real GUI/`cable_torque_ctrl_node` expect), `README.md`, `CLAUDE.md`. Ported from `AK40-10-ROS2-Bridge/src/ak_motor_cable_control_node.cpp`: full ROS2 interface (all topics/services/params except CAN/MIT-protocol-specific ones), external-mode 3-state machine + authority hierarchy, command/heartbeat watchdog *detection* (fallback behavior changed, see below).
- **SPEED mode required real debugging, not a direct port** — three iterations, in order:
  1. First attempt kept the real driver's convention (`~/command.velocity` = rad/s, pure damper `force=kd_speed*(v_des-v)`, `kd_speed=0.5` via `effective_radius=0.036`). Produced ~0.009 N of force for a typical command — three orders of magnitude below the ~3.14 N gravity load on the default payload. No visible response ("no response in the cable" — user report).
  2. Switched `~/command.velocity` to m/s directly (dropped `effective_radius` from this path) and added genuine PD against a position setpoint integrated from the velocity command, so a `kp` term could reject the constant gravity disturbance (a pure damper fundamentally cannot). First gain guess (`kp_speed=2000, kd_speed=50`) implied a ~79 rad/s natural frequency — too stiff for the ROS2↔Isaac-Sim (GUI-rendering, sub-100Hz) control loop, destabilized the physics solver ("payload moves very fast, simulation hit a glitch" — user report).
  3. Added an explicit `gravity_feedforward_n` term (mirrors `cable_torque_ctrl_node`'s own `tau_p=-m*g*r` feedforward) so `kp_speed`/`kd_speed` only handle tracking dynamics, not disturbance rejection — dropped gains ~100x (`kp_speed=10, kd_speed=4`, ~5.6 rad/s). **User confirmed this works.**
  4. One more bug found after that: the real driver's `kd_watchdog` fallback (weak pure damper, no feedforward) is safe on real hardware because the winch's mechanical friction prevents free-fall — this sim's cable has none, so a stale command (e.g. the GUI's Stop button publishes one command then stops republishing) let the payload drop to the ground ~500ms later. Fixed by removing `kd_watchdog` entirely: any time SPEED isn't actively tracking a fresh command, the same PD+feedforward law now freezes the setpoint at the current position and holds it — "stale" means hold, not fall back to passive damping.
- Full design rationale, the sign-convention table, and all current parameter defaults are documented in the emulator's own `CLAUDE.md` (`fsc_autopilot_ws/src/AK40-10-Isaac-Sim-Emulator/CLAUDE.md`) — read that before touching SPEED-mode gains or units.
- **EXTERNAL mode: verified working, including requirement #5.** Getting here took two real bug fixes (both user-caught from live plots):
  1. `~/arm`-ing `cable_torque_ctrl_node` (no `~/reference`, its own default gravity-hold-at-arm-position law) with the drone hovering produced growing oscillation and repeated torque-limit clamping — even plain SPEED-mode hold was fine, isolating this to the EXTERNAL-mode path. **Fixed**: the emulator's shaft-dynamics low-pass filter (`omega_filtered_retract_`) was built for its own internal force computation but never applied to what actually gets *published* — `~/cable_state`/`~/joint_state` were still sending raw, noisier Isaac Sim velocity straight to `cable_torque_ctrl_node`'s sliding-mode/L1-adaptive law, which is tuned for real hardware's much cleaner encoder feedback and chattered on the noise. Both topics now publish the same filtered value.
  2. `cable_torque_ctrl_params.yaml`'s deployed `mass: 0.565` kg (real hardware's payload, baked into gravity feedforward and the L1 adaptive law's nominal init at startup, no override path or live-update support) didn't match this scenario's actual hanging load (`rod_b_mass+payload_mass` was `0.32` kg — nearly half). **Fixed** by changing the sim's own default masses to match (`application/slungload/02_px4_single_drone_payload_variable_length_cable.py`'s `payload_mass` default `0.3→0.545`, so `rod_b_mass(0.02)+payload_mass(0.545)=0.565` exactly), and updating the emulator's `gravity_feedforward_n` default to match (`3.14→5.54` N, SPEED mode only).
  3. Also checked and ruled out as contributors: Iris lift capacity (1.5 kg body, ~41 N max thrust vs ~20.5 N loaded weight — ~2:1 margin, confirmed by directly querying the USD asset's mass) and PX4 attitude-gain mismatch (SPEED mode, which bypasses `cable_torque_ctrl_node` entirely, stayed clean with the same heavier payload).
  - **After both fixes**: `cable_square_ref_node` + `cable_torque_ctrl_node` (both unmodified) correctly track a real commanded reference against the sim cable — **requirement #5 confirmed working**. A small bounded oscillation (chatter) persists, traced to the controller's own `smc_phi=0.15` sliding-mode boundary layer width interacting with this sim's noise profile — not a bug, not growing, effort stays within limits; not chased further since the controller can't be modified. Full write-up in the emulator's own `CLAUDE.md`.

**Open investigation (2026-07-11): why does the SMC controller chatter in sim but run cleanly on real hardware?** User reports `cable_torque_ctrl_node` (BLSMC/L1) works perfectly on the real AK40-10 rig; in this sim it's the bounded chatter described above. Working theory: Isaac Sim isn't running this scenario in true real-time, so the controller's fixed-100Hz wall-clock polling sometimes sees stale/jittery feedback that a real CAN-bus-fed control loop wouldn't. Findings so far:
- Directly measured `cable_winch_0/state/cable`'s actual wall-clock publish rate (`ros2 topic hz`) against the nominal `physics_dt=1/250s`: **~150–190 Hz actual, both with rendering on and in headless mode (`PEGASUS_HEADLESS=1`, added to `02_px4_..._variable_length_cable.py` this session)** — only ~60–76% of nominal, and headless gave **no meaningful improvement** over rendered, which was initially surprising.
- Root cause of that non-result: the comparison wasn't controlled for GPU contention. `nvidia-smi` showed the GPU (RTX 3080) at 90% utilization, shared across many concurrent desktop processes (Xorg, kwin_x11, several VSCode/Chrome renderer processes, QGroundControl, `qgroundnode`) — Isaac Sim was the largest single consumer (2865MiB) but far from the only one, and **Isaac Sim uses the GPU heavily even in headless mode** (no visible window ≠ no GPU usage — likely still doing offscreen RTX/Hydra passes). CPU was not the bottleneck (load average 4.55 of 20 cores, plenty of headroom).
- Also found, while trying to get `qgroundnode`'s 3D view working as a lightweight (non-Isaac-Sim) way to watch the drone during headless runs: PX4's uXRCE-DDS bridge (`/fmu/out/*` topics) shows **Publisher count 0** even after starting `MicroXRCEAgent udp4 -p 8888` and seeing a full connection handshake in its log — classic symptom of the agent needing to be listening *before* PX4 boots, not attached after. Not yet fixed (deprioritized — orthogonal to the real-time investigation; Pegasus's own `/uav_0/state/*` topics are unaffected, they come from a separate rclpy backend, not PX4's DDS bridge).
- **Controlled re-test, same day**: closed Chrome and Slack (the two identifiable unrelated GPU consumers) and re-measured headless. Result: GPU utilization dropped only 90%→77%, **Isaac Sim's own GPU footprint was unchanged (2865 MiB both times)**, and the measured rate was unchanged too (~140–190 Hz). This ruled out contention from unrelated desktop apps as the cause — but the real cause turned out to be simpler and fixable (see below), so treat the "not rendering" framing at this point in the investigation as superseded.
- **Root cause found via direct profiling**: added `PEGASUS_PROFILE=1` (wall-clock timing around `world.step()` and `ROS2CableWinchBackend.update_sim_state()`, printed every 250 steps) to `02_px4_..._variable_length_cable.py`. With rendering on (even headless — the script was still unconditionally calling `world.step(render=True)`): outer loop only ~41–47 Hz (~22ms/call, max ~44–49ms); our own callback averaged under 1ms (~0.8–0.9ms, max ~3ms) — clearly not the bottleneck. The `ros2 topic hz`-measured ~140–190 Hz was reconciled as ~4 physics substeps firing per outer `world.step()` call (`physics_dt=1/250s` vs `rendering_dt=1/60s` ≈ 4.2:1 ratio) — i.e. **Isaac Sim was still running a full render pass every call even with no window to show** (consistent with the GPU-usage finding above: headless ≠ no rendering work).
- **Fix**: made the step loop's `render` argument conditional — `render=False` whenever `PEGASUS_HEADLESS=1` (there's nothing to display anyway), instead of always `render=True`. Result: outer loop jumped to **~163–267 Hz** (often at or above the nominal 250 Hz), max per-step time dropped from ~44–49ms to ~10–18ms, and the actual `cable_winch_0/state/cable` rate (`ros2 topic hz`) rose to **~195–242 Hz** (78–97% of nominal, up from ~140–190 Hz) with much tighter jitter (max ~11–15ms, down from ~21–42ms). **This is now the permanent default** (tied to `PEGASUS_HEADLESS`, not gated behind the profiling flag) — headless runs skip rendering unconditionally.
- **Residual gap** (250 Hz nominal vs. ~195–242 Hz actual, plus the still-present ~11–15ms jitter) is smaller now but not zero — a genuinely faster/dedicated machine (the shelved remote-server-over-ZeroTier test) would be the next lever if this still matters for the `smc_phi` boundary-layer chatter accepted above. Not pursued further for now given the size of the improvement already achieved. Real-time factor plateaued at **~80%** after the `render=False` fix (measured again after the SMC-cause discussion below, unchanged) — the remaining gap is attributed to PhysX's own CPU solve cost plus the PX4 lockstep mavlink/TCP round-trip each physics step, both effectively fixed costs for this scene on this hardware, not something with an obvious further toggle.
- **Conclusion on what caused the chatter**: after both bug fixes (velocity filtering, mass matching) *and* the real-time improvement above, torque/external-mode chatter still persisted essentially unchanged. Since improving sim timing didn't reduce it, **`cable_torque_ctrl_node`'s own sliding-mode boundary-layer behavior (`smc_phi`) is the dominant cause**, not sim timing — sliding-mode chattering near the boundary layer is a well-known, textbook SMC characteristic in any discretized loop, not a sim-specific bug. Likely compounded somewhat by this sim's simulated plant being *less damped* than real hardware (the shaft-dynamics model has viscous drag only, no Coulomb/static friction, a deliberate scope simplification — see the emulator's own `CLAUDE.md`), since real mechanical friction would naturally suppress chattering amplitude more than our simplified model does.

**Methodological conclusion — when to use this sim vs. a custom solver** (recorded 2026-07-11, useful for scoping future work in this repo): this Isaac Sim + emulator setup is **not** the right tool for verifying a controller's theoretical properties (stability margins, exact tracking-error bounds, chattering amplitude as a tuning metric) — for that, a simplified deterministic solver (e.g. a Python RK4 integrator against the nominal plant model) is the right approach, since it's free of rendering cost, PX4 lockstep IPC, and OS scheduling jitter entirely. What this sim setup *is* good for: integration-testing the actual unmodified controller binary against a realistic 3D physics scene and the real ROS2 topology it will face on real hardware, before real experiments — exactly how the two genuine bugs this session (unfiltered velocity feeding the SMC/L1 law, the payload-mass mismatch) were caught, neither of which a pure math simulation would ever have surfaced. This class of validation is well-scoped to **slow-dynamics applications** (aerial manipulation, slung loads — pendulum/cable timescales of seconds), where the measured ~10–20ms timing jitter is 2+ orders of magnitude smaller than the dynamics being validated; it would matter far more for fast/high-bandwidth control problems (attitude loops, flexible-structure vibration) where the sim's own timing noise could rival the phenomena under study.

**Lightweight visualization tooling added this session** (`scripts/`, for watching the drone/payload during headless runs without Isaac Sim's own render cost):
- `view_drone_3d.sh` — opens RViz2 (`config/drone_view.rviz`) with a Grid + TF display (drone, via Pegasus's existing `map`→`uav__base_link` broadcast) + a Pose display for the payload (`/payload/state/pose`, published by a `ROS2RigidBodyBackend` newly wired onto the payload in `02_px4_..._variable_length_cable.py` — it had no pose output before). Also starts `cable_line_marker_node.py` in the background (kills it on exit via an `EXIT` trap) — a small standalone rclpy node with no Isaac Sim dependency that subscribes to both poses and publishes a `visualization_msgs/Marker` line between them, since RViz has no built-in way to connect two independently-published poses.
- Tuning notes if the framing ever looks off again: RViz2's Pose display has *separate* size properties per `Shape` — `Shaft Length`/`Head Length` only apply to `Shape: Arrow`; `Shape: Axes` (what's used here) needs `Axes Length`/`Axes Radius` instead, silently ignoring the Arrow-only properties if set instead (this was a real bug hit this session — don't reintroduce it).
- Separately, `qgroundnode`'s own PX4-telemetry-based 3D view was **not** fixed (deprioritized, see above) — the tooling above is the reliable path since it doesn't depend on PX4's uXRCE-DDS bridge at all.

**PD+L1 chatter A/B test node (2026-07-11, in `fsc_autopilot_ws`'s `AK40-10-Isaac-Sim-Emulator` package, not this repo)**: to empirically test the `smc_phi`-boundary-layer conclusion above, built a second controller node, `cable_torque_ctrl_pd_l1_node`, as a near-verbatim copy of `cable_torque_ctrl_node.cpp` with the same public ROS2 interface (drop-in swap in `cable_torque_ctrl.launch.py`) but with the BLSMC sliding-surface/switching term (`s`, `tau_sw`, `smc_eta`, `smc_phi`) replaced by a plain PD term (new `kp_pd`/`kd_pd` params) on top of the same feedforward. The L1 adaptive estimator and UDE monitor are unchanged — valid because the L1 predictor/adaptation law only reads back the previously *applied* torque, not how it was computed.

**First live test surfaced a bigger, unrelated bug**: holding position with zero reference produced a sustained **~23cm limit-cycle oscillation** (confirmed via the node's own `~/debug` topic: `e_p` amplitude was 0.228m in the first quarter of a 12s window and 0.227m in the last — not decaying, a true limit cycle, not settling ringing). Traced to the `coulomb_friction` feedforward term, copied unchanged from `cable_torque_ctrl_node`'s deployed value (real hardware's calibrated static friction) — but this sim's shaft-dynamics model has **no Coulomb friction at all** (viscous drag only, an explicit scope decision). At the oscillation's velocities, `tanh(omega/deadband)` saturates to essentially `sign(velocity)`, i.e. a relay force always in phase with velocity that does net positive work every cycle since there's no real friction for it to cancel — a textbook friction-overcompensation limit cycle. **Fixed** by removing the Coulomb term from this node entirely (not zeroing it — no physical friction here for it to ever compensate), keeping viscous drag only. **User confirmed live: controller is smoother, no chattering, holding position cleanly.**

**Second bug, found getting the live square-wave reference test running**: `cable_square_ref.launch.py` (real driver package, must stay unmodified) hardcodes its reference remap to `/cable_torque_ctrl_node/reference`, not parameterized the way `cable_torque_ctrl.launch.py` is — so it never reached `cable_torque_ctrl_pd_l1_node`. Fixed by running the same unmodified `cable_square_ref_node` executable directly (`ros2 run`, not the launch file) with a manual remap. **Third, self-inflicted bug** on the first attempt at that fix: an unquoted `-r ~/reference:=...` gets tilde-expanded by bash itself into the user's home directory path before `ros2` ever sees it, so the remap silently failed with no error. Fixed by quoting the whole remap argument.

With all three fixed, the live A/B test (real square-wave reference driving `cable_torque_ctrl_pd_l1_node`) is now running. **This meaningfully updates the smc_phi conclusion above**: the Coulomb-friction-feedforward mismatch (present in the real `cable_torque_ctrl_node` too, against this same frictionless-in-Coulomb-terms sim plant) is at least a major contributor to the originally observed chatter — the SMC switching term may have been masking/absorbing this effect as small bounded chatter rather than being its root cause. Not yet isolated exactly how much of the original chatter was `smc_phi` vs. this friction mismatch — full write-up (including a proposed follow-up test: re-running the original `cable_torque_ctrl_node` with `coulomb_friction` diagnostically overridden to 0) in the emulator package's own `CLAUDE.md`.

**Next steps (in order — see the plan file for full detail):**
1. Optional follow-up to fully separate the two chatter causes: re-run the original `cable_torque_ctrl_node` (BLSMC) with `coulomb_friction` overridden to `0` via a command-line param override (diagnostic only, not a permanent change to its deployed config) and see whether its chatter also disappears — would show how much of the original chatter was `smc_phi` vs. the friction mismatch.
2. Validate requirement #6: bring up the `ak_motor_ground_station` GUI against the same running stack (emulator + `cable_torque_ctrl_node`, external mode active) and confirm Stop/Emergency-Stop/Exit-External correctly override/interrupt the active external controller.
3. Optionally: explicitly verify the `off→standby→running` state transitions and `RUNNING→STANDBY` fallback in isolation (implicitly exercised during the above testing, but not checked step-by-step on their own).
4. Optionally: fix the uXRCE-DDS agent/PX4 ordering issue so `qgroundnode`'s 3D view works (restart PX4 with the agent already running first) — lower priority now that `view_drone_3d.sh` covers the same need without that dependency.

## Checklist for adding a new vehicle model

When creating a new vehicle (config class + vehicle class), verify each item below.

**First decide how much to share.** If the airframe reuses an existing asset and differs only
in motors and/or mass — as the T650 does from the X650 — there are two routes, and the repo
now contains one of each:

- **Shared module, selected by name.** Add a `MOTOR_CALIBRATIONS` entry in
  `x650_bare_frame_utils.py` and pass `motor=` / `body_mass=`. Least duplication; fine while
  the vehicles genuinely differ only in a few numbers.
- **Parallel modules** (`t650_params.py` + `t650_bare_frame_utils.py`, added 2026-08-06 on
  instruction). Importing nothing from the X650 pair means a change to one vehicle cannot
  silently move the other — which matters once a vehicle starts carrying its own empirical
  fit factors, as the T650 now does. Cost is duplication that will drift.

Do not do both for the same vehicle: the T650 currently has a live `t650_*` path and a
now-dead `MOTOR_CALIBRATIONS["MN4010"]` entry, which is exactly the drift risk to avoid.

Whichever route you take, re-run the fit procedure against an already-committed motor first
and confirm it reproduces that motor's constants before trusting it on the new one (see
"Propeller bench test data"). Compute hover/scaling values **from** the constants rather than
hard-coding them, so they cannot go stale when a constant changes.

### Config class
- [ ] All fields are **pure data** — no live resources (ROS nodes, sockets, processes) instantiated in `__init__`
- [ ] `self.backends = []` or `self.backends = None` — never `[SomeBackend(...)]`
- [ ] Same rule applies to `self.sensors`, `self.graphical_sensors`, `self.graphs` if they carry live state

### Vehicle class `__init__` signature
- [ ] Default `config` argument is `None`, **not** `config=MyConfig()` — Python evaluates default arguments once at import time, creating shared mutable state across all calls
  ```python
  # Wrong
  def __init__(self, ..., config=MyVehicleConfig()):

  # Correct
  def __init__(self, ..., config=None):
      if config is None:
          config = MyVehicleConfig()
  ```
- [ ] Default backend (if any) is created **inside** `__init__` after resolving `None`:
  ```python
  if config.backends is None:
      config.backends = [MyDefaultBackend()]
  ```

### ROS 2 backend
- [ ] The backend creates **exactly one** `rclpy` node per instance — verify the node name is unique (include `vehicle_id`)
- [ ] `rclpy.init()` is wrapped in `try/except` (already initialised is not an error)
- [ ] Publisher/subscriber QoS matches what the counterpart node expects (`qos_profile_sensor_data` = BEST_EFFORT)

### Single-link floating articulation (no external USD)
- [ ] `ArticulationRootAPI` and `RigidBodyAPI` are applied to the **same** prim — PhysX 5 does not form a valid articulation when they are on separate prims
- [ ] Override `update_state` to read from `self._stage_prefix` (not `+ "/body"`)
- [ ] Call `apply_force` / `apply_torque` with `body_part=""`
