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

**AM-T650: THE AERIAL MANIPULATOR UNDER THE fsc_autopilot_ros2 DIRECT-ACTUATION STACK
(2026-08-10, user request — the safe-fallback integration step for the whole-body
controller).** T650 is the airframe the real AM will be built on, so the AM plant now has a
variant carrying the T650 parameters, flown by the external quadrotor-only DIRECT law while
the arm is parked. `application/robotic_arm/04_px4_direct_t650_aerial_manipulator_hold.py` spawns
`AM_realign.usda` PX4-primary (inline spawn, 03's pattern) with `t650_params`' MN4010
calibration (ω 64.06..730.05, k/c incl. both fit factors, λ = 10.0265 via
`LaggedQuadraticThrustCurve`) and re-authors `/body` to the T650 2.95 kg + T650 inertia on
the live stage → **TOTAL 3.746 kg** (AM rotors 4×0.039887 + arm 0.636624 add on top — NOT
x650_new.usd's lighter rotors). In-process it does exactly one thing: the flight-validated
02/03 arm hold (PD KP 3.0/KD 0.25 + `dynamics()` gravity comp, clamp 3.0 N·m, slewed ref)
at the user-specified home `q_home = [0, 40°, 40°, 0]`, hardcoded — no planner dependency —
running unconditionally (ground/SAFETY/DIRECT). Keeps 02/03's spawn-at-home + rigid re-seat
+ ground-seat, effort-mode arm DOFs, armature, gripper-drive pin, collider/scene fixes; drops
the whole-body law, mixer, bridge, flight machine, npz. Reads `PEGASUS_PX4_LOCKSTEP`
(indoor-launcher convention; 03 hardcodes False instead) and does NOT start paused (the
mocap emulator needs `/uav_0/state/*` flowing). Launcher
`scripts/indoor_sim/start_t650_aerial_manipulator_direct_actuator_sitl.sh` = the T650 DIRECT wrapper pattern
(agent pre-check, lockstep export + tmux-server push, offboard param script) but setting the
`INDOOR_SIM_*` hooks itself (04's script, label AM-T650, own PX4 profile
`rootfs_fsc_indoor_am_t650`) and exec'ing the x650 base launcher directly. Pairs with
fsc_autopilot_ros2's `start_direct_actuation_t650_aerial_manipulator_stack.sh` +
`config/params_single_drone_direct_actuation_t650_aerial_manipulator.yaml` (dev_CCM): a copy of the T650
tune whose ONLY value changes are the four `vehicle_*` numbers — mass 3.746172,
thrust_scaling/idle_thrust RE-DERIVED about the heavier hover point (0.036206/0.236457,
hover ≈ 0.569 vs bare 0.503; the +23% mass is far off the tangent-fit anchor, so copying
was not an option) — verified: key set identical, exactly 4 value diffs, rounded values
reproduce hover to −2.3e-06. **The yaml's `vehicle_mass` must equal the TOTAL the spawn
prints; the printout is the truth.** Known-accepted: alloc px/py stay geometric while the
folded arm offsets the CoM (standing trim expected — if the rate integrator parks near
i_max 0.09, seed `ratectl_trim_*` from `rate_control_debug[3..5]`); rate gains are the
T650/X650 carry-over on a heavier, higher-inertia plant (watch yaw first — the 2.45×
sim-yaw-gain caveat applies unchanged). Commands: Command.md §7.7.
**FIRST FLIGHT FLIPPED ON LIFT-OFF (2026-08-10) — ROOT CAUSE: AM_realign's rotor CHANNEL
INDEXING is mirrored vs x650_new.usd** (ch0 front-LEFT, ch1 rear-RIGHT, ch2 front-RIGHT,
ch3 rear-LEFT — read off the live stage), which EXACTLY NEGATES THE ROLL ROW of any
standard Quad-X allocation while leaving pitch/yaw correct — and the flip happened in
SAFETY mode, where PX4's OWN mixer does the allocation, so no yaml-side fix could reach it
(observed ω=[427,467,466,427]: the left pair commanded low while PX4 believed it was the
right pair). This plus the front-on-+y quirk is fixed at the ASSET level, per the user's
standing plan to re-author the front onto body +x: **`AM_xfwd.usda`**, generated by
`utils_model/make_x_forward_asset.py` (AM_realign untouched, still serves all whole-body
demos). The transform exploits the postprocessed layout (every link/joint frame authored at
identity): rotate every non-body link's pose by Rz(−90) (frame moves WITH content — nothing
inside needs touching), keep /body's frame while rotating its mesh child + CoM and swapping
Ixx↔Iyy, and rotate the body-side localPos0/localRot0 of the 5 body-anchored joints
(joint0..3, manip_joint1). Result: arm on body +x; rotors r0 FR/CCW, r1 RL/CCW, r2 RR/CW,
r3 FL/CW — x650_new's convention with CHANNELS 2↔3 SWAPPED, so 04 remaps
ROTOR_PATHS=[rotor0,rotor1,rotor3,rotor2] and the plant presents exact PX4 Quad-X. 04 also
gained a gravity-comp FRAME ADAPTER (R0_model = R0_actual·Rz(−90) — make_params/dynamics
still describe the OLD frame; exact for any attitude since only the frame label changed,
verified identical at level attitude). The whole-body stack (make_params, RotorMixer,
planner z-x-z, EE_SAG, path_yaw_deg=90) is NOT migrated — it keeps flying AM_realign.
**SIM-VALIDATED same day (headless full stack, flown end-to-end by Claude):** SAFETY
takeoff on a streamed z=1.2 reference (SAFETY holds the ground position until a reference
arrives — stream it at ≥20 Hz, ONE publisher only), hover 4 mm, DIRECT entered in-flight
and held 50+ s zero-drift with no watchdog trip, SAFETY reversion, 0.2 m/s descent to
touchdown; hover command 0.5687 vs 0.569 predicted; arm ≤0.6° from home throughout; the
forward-arm CoM shows as the predicted standing pitch trim (front pair ~+37 rad/s),
absorbed cleanly. PX4 denies disarm until the land detector sees low thrust — land by
reference, then disarm. Rendered confirmation run still pending.
**ARM FEEDFORWARD (2026-08-10, user-reported symptom: X underdamped/slow while Y clean).**
Cause: the arm folded at home puts the CoM **19.5 mm FORWARD** (model at q_home; independently
19.3 mm by back-solving the flown hover's rotor commands 59.7/54.1%), a constant **+0.72 N·m**
nose-down pitch moment. `alloc_rotor*_px/py` are geometric, and per rate_controller.hpp ONLY the
rate integrator can absorb a torque bias (attitude loop is pure P; the UDE compensates force) —
so every engagement re-learned it while the vehicle tilted. dy = −0.1 mm is why ROLL/Y was clean:
the symptom's asymmetry is the arm's asymmetry. Fixed CONFIG-ONLY with the purpose-built
`ratectl_trim_y: -0.040` in `params_..._t650_aerial_manipulator.yaml` (dev_CCM) — **no control-law change**.
Value COMPUTED, not tuned: the normalized FLU pitch torque whose allocated commands give zero
physical moment about the true CoM at hover collective (normalized pitch 1.0 = 11.12 N·m; uses
44% of i_max), predicting 59.7/54.1/59.7/54.1% against the flown 59.7/54.1/59.6/54.0%.
**Do NOT instead CoM-reference `alloc_*_px`** (design_decisions.md's action item): the
effectiveness matrix assumes thrust ∝ command but the real curve has a large affine offset, so
that OVER-corrects (+0.72 → −0.50 N·m), verified through this repo's own NormalizedMix.
Matched A/B flown (arm→climb→engage DIRECT, trim 0 vs −0.040): X peak excursion **97.7 → 2.1 cm**,
settling **23.1 → 4.1 s**, both settling to the same I_y ≈ −0.0394. Two testing traps, both hit:
(a) there is NO on-set-parameters callback, so `ros2 param set` changes what `param get` reports
while the controller keeps the YAML value — relaunch with `params_file:=` to A/B; (b) `reset()`
(which applies the trim) fires on the **ARMING edge**, not the mode switch — SAFETY↔DIRECT uses
`resetDerivative()` and deliberately PRESERVES the integrator — so land/disarm/re-arm between
trials. Full write-up: `docs/docs_aerial_manipulator/Feedforward Compensation for Home-Pose Arm.md`.
Still open: SAFETY mode has the same bias and PX4's own integrator re-learns it (unreachable from
this config); a true q-dependent feedforward belongs to the whole-body controller, not this rig.
**BOTH CONTROL PATHS NOW EXIST, AND THE SLOW-STEP DIAGNOSIS (2026-08-11, user request).** The AM
rig gained a BASELINE (SAFETY-only) counterpart so the arm's compensation exists on both paths:
new `scripts/indoor_sim/start_t650_aerial_manipulator_baseline_sitl.sh` (the direct launcher minus the lockstep
disable — baseline WANTS lockstep, and it now asserts `PEGASUS_PX4_LOCKSTEP=1` + clears the tmux
global so a stale 0 from a DIRECT run cannot leak), pairing with fsc_autopilot_ros2's
`start_baseline_t650_aerial_manipulator_stack_fused.sh` + `config/params_single_vehicle_baseline_t650_aerial_manipulator.yaml`
(dev_CCM). Both Isaac launchers share `04_px4_direct_t650_aerial_manipulator_hold.py` (control-agnostic — it only
spawns the plant and holds the arm) and the PX4 profile `rootfs_fsc_indoor_am_t650`.
**The arm's FORCE is fed forward on both paths via `vehicle_mass` 3.746170**, read independently by
robust_controller's gravity term and the UDE; the one-number check is
`position_controller/ude.disturbance_estimate.z`, measured **+0.05 N** on both (it would be ≈ −8.3 N
if the bare-T650 yaml's 2.9 kg were loaded — a 29% error). The arm's TORQUE stays DIRECT-only:
the baseline node builds no rate controller, and **PX4 v1.16 has no multicopter torque trim**
(`TRIM_PITCH` exists only in fw_rate_control/rc_calibration, zero refs under control_allocator or
mc_rate_control) — verified, not assumed.
**SLOW x/y/z STEP CONVERGENCE WAS NOT MISSING FEEDFORWARD.** robust_controller.cpp ALREADY
implements velocity/acceleration/thrust feedforward and `PositionControllerReference` already
carries all three fields — but a STEP has zero velocity and zero acceleration, so every FF term is
zero by construction and no feedforward can speed up a step. The cause is the poles: `nested` form
is `k_vel*(vel_err + k_pos*pos_err)` in NEWTONS with **no mass normalisation**, so the arm's +23%
mass alone cost 10% of both wn and zeta. Fixed per path — baseline `k_vel` 3.0→**3.70** (pure
mass-restore of the hardware-validated T650 response; deliberately NOT higher, because
docs/sim_to_real_fidelity.md measured sim overstating the settling benefit of raising k_vel by
**5.8×**, which is how a sim-tuned 8.0 was walked back to 3.0), DIRECT 3.0→**7.0** (the X650 DIRECT
value, hardware-validated at a near-identical 3.5 kg, where higher k_vel is stabilising against the
~120 ms attitude lag). Measured DIRECT A/B: x/y rise 2.0-2.5→1.6-1.7 s, overshoot 42-53→25-30%,
final err ±52-100→±2-14 mm; z overshoot 14-19→0-0.5%, settle 11.0→5.8 s. **The second-order formula
describes the Z AXIS ONLY** (z drives collective directly; x/y close through the attitude loop) —
it predicted 4.3 s/5.3% at 7.0, z measured 5.8 s/0% but x/y 15.7-17.8 s/25-30%. That residual x/y
underdamping is the attitude lag. **x/y RINGING FIXED by LOWERING k_pos 1.0→0.6** (DIRECT only;
zeta = sqrt(k_vel/(4*k_pos*m)), so lowering k_pos raises damping WITHOUT asking the inner loop for
bandwidth — the cascade remedy when the inner lag is fixed): x/y overshoot **25-30% → 0.0-0.4%**,
settling **15.7-18.9 → 9.6-10.1 s**, final err ±2-14 → ±0.1-3.0 mm, at the cost of rise 1.7 → 3.3 s.
k_pos_z stays 1.0 (z never rang). Safe to adopt from sim because it LOWERS gain — the 5.8x
sim-overstatement warning applies to raising. **Two hypotheses measured and REJECTED first, do not
retry**: (a) more torque feedforward — the trim already compensates it AND a pitch-only bias cannot
ring ROLL, yet y overshot 29.5% vs x 25.5% while dy = -0.1 mm; (b) inertia-scaled rate gains — the
arm really does raise inertia x1.53 roll/x1.86 pitch/x2.02 yaw (coupled mass matrix at q_home vs
bare T650), but scaling ratectl_kp/ki/kd by that ratio gave NO improvement (23.5-29.5% vs 25-30%),
so the rate loop is not the limiter, the 99.7 ms rotor lag is. Full write-up incl. the per-path
table: `docs/docs_aerial_manipulator/Feedforward Compensation for Home-Pose Arm.md` §6-7.1;
commands: Command.md §7.8. Baseline path still at k_pos 1.0 with 42-50% x/y overshoot — the same
fix would likely help but is UNMEASURED there.
**TRUE ARM TORQUE FEEDFORWARD (2026-08-11, user request — the first fsc_autopilot_ros2
control-law change).** The trim was the hover-only, arming-edge approximation; the arm is a
KNOWN wrench, so the DIRECT node now cancels it exactly every tick: new optional
`system_ff_tau_{x,y}_per_coll/_const` params (default 0.0 = off, other vehicles bit-identical)
add `per_coll*collective + const` to the commanded torque before allocation. Closed form
`tau_ff_y(c) = -dx/(√2·L)·(c + b/a) = -0.060c - 0.0058` — evaluated at hover it reproduces the
flight-calibrated trim (-0.0399 vs -0.040) BEFORE any code was written. AM yaml sets it and
zeroes `ratectl_trim_y`. Flown: settled I_y **-0.0394 → +0.0005** (integrator fully freed),
step metrics unchanged-or-better. Exact at every thrust level (climbs/z-steps no longer dump
0.060·Δc onto the integrator); q-dependent dx(q) is the whole-body controller's opening move.
Write-up: `Feedforward Compensation for Home-Pose Arm.md` §8.
**RESTRUCTURED same evening (user request): the feedforward no longer touches the shared
node.** fsc_autopilot_ros2's `single_drone_direct_actuation_client.{hpp,cpp}` were reverted
byte-identical to upstream, and the feedforward now lives in a deliberate PARALLEL FORK,
`fsc_autopilot_ros2_node/single_aerial_manipulator_direct_actuation/`
(`autopilot_aerial_manipulator_direct_actuation_node`, launched by
`single_aerial_manipulator_direct_actuation_launch.py`) — same node name/topics/services as
the parent, so this repo's side cannot tell them apart; bug fixes do NOT auto-propagate
between the two clients. In the same pass every `am_t650` file was renamed to
`t650_aerial_manipulator` in both repos (launchers, yamls, the 04 hold script) — EXCEPT the
PX4 profile directory `rootfs_fsc_indoor_am_t650`, kept on purpose: renaming it would
re-clone a fresh 39 GB rootfs and orphan the saved AM params.

**ROS2 POSITION-MODE ARM STACK (fsc_open_manipulator) INTEGRATED (2026-08-13, user
request — first step toward external arm control; torque mode later).** The AM-T650
DIRECT rig gained a sibling where the arm is commanded OVER ROS2 by the real
OM-X position-mode stack instead of 04's in-process hold. New, all incremental (04 and
its launcher untouched): `application/robotic_arm/05_px4_direct_t650_aerial_manipulator_ros2_arm_hold.py`
(04's plant verbatim; the hold law's REFERENCE now comes from
`/uav_0/isaacsim_manipulator/position_commands`, joint states/efforts go out on
`/uav_0/isaacsim_manipulator/joint_states` under controller-side names joint1..4 — the PD+gravity+clamp
hold is unchanged and acts as the Dynamixel-servo emulation, AK40-10-emulator pattern;
reference LATCHES if the stack dies, so solo it behaves exactly like 04) +
`scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuator_sitl.sh` (the 04 direct
launcher plus a detached-helper tmux window `arm` with the ros2_control stack — gated on the
Isaac topic — and the ARM GROUND STATION `joint_plot_inverted`; two ground stations total
with the drone one; same PX4 profile as 04 on purpose, same plant). The arm side lives in
`~/ros2_ws/src/fsc_open_manipulator` (workspace repo + Gao907/open_manipulator fork on
omx-torque-control, cloned 2026-08-13): new package `open_manipulator_x_isaac_bridge`
(topic-based `hardware_interface::SystemInterface` plugin `IsaacTopicSystem` — read()
latches JointState by name, write() publishes commands; on_activate BLOCKS for the first
Isaac state so the controller never snapshots NaN; stub URDF carries only the ros2_control
block; gripper-less copy of the AERIAL config — inverted-only per the user, home
[0,40°,40°,0] = the plant's spawn pose so activation homing is a no-op) running the REAL
`PositionController` plugin. Machine facts (shiqi-desktop): no system ros2_control/pinocchio
and no sudo → 30 debs extracted root-lessly to `~/ros2_ws/rosdeps/` (its `local_setup.bash`
documents the future `sudo apt install` replacement; octomap runtime libs symlinked, 2
non-relocatable cmake paths patched); Qt 6.8.3+Charts via aqtinstall to `~/Qt` (custom_gui's
expected location); **one fork change**: `open_manipulator_x_custom_controller/CMakeLists.txt`
additionally links `pinocchio::pinocchio` — ros-humble-pinocchio 4.0's Boost.MPL sizing
defines and deprecated-header include dir only travel on the modern target, and
`ament_target_dependencies` drops them (harmless on the bench machine's older pinocchio).
The workspace builds STANDALONE inside its repo (`colcon build --base-paths src`, a
gitignored `COLCON_IGNORE` at its root keeps `~/ros2_ws`-level builds out). Config hooks in
`shiqi_machine.conf`: `FSC_OM_ARM_WS`, `FSC_OM_ARM_ROSDEPS_SETUP`. VALIDATED 2026-08-13
headless (ground-seated, no PX4 — drone path is 04's, not re-flown): real plant + real stack
hold home, track a 4-joint target and go_home through the real interfaces, all <0.05° steady
error, hold torque ≈0.7 N·m; the user then ran the full workflow live. **NAMESPACED under
/uav_0 same day (user request)**: the launch takes `namespace` (default uav_0, nodes +
spawners), the yaml nests its node keys under `/uav_0:` (launch namespace and yaml nesting
must change TOGETHER — a mismatch means params silently don't apply and the controller fails
on empty `joints`), and the controller's logging topics went namespace-relative — so the
whole arm interface sits beside the drone's: /uav_0/controller_manager,
/uav_0/arm_position_controller/…, /uav_0/{joint_states,joint_desired_states,joint_measured_states}.
This required making custom_gui's ROS names NAMESPACE-RELATIVE (they were absolute
"/..."; bare launches still resolve to the root, so the bench workflow is unchanged) — the
sim's station runs with `-r __ns:=/uav_0`. Re-validated loopback (hold/move/go_home PASS)
and on the wire: the namespaced GUI publishes/subscribes the /uav_0 target topic, follows
/uav_0/joint_states, and adopts the working range via the namespaced parameter service.
**SUPERSEDED 2026-08-15 by the OWNER-PREFIX topic scheme** (user request; full convention in
`docs/docs_aerial_manipulator/Arm Topic Naming.md`): every topic now sits under the namespace
of the PACKAGE that owns it, mirroring the flight stack's own /uav_0/fsc_autopilot_ros2/… —
so the real arm stack moved to **/uav_0/fsc_open_manipulator/**{joint_states,
joint_desired_states,dynamic_joint_states,arm_position_controller/…,controller_manager} and
the Isaac servo emulation (sim-only, absent on hardware) to
**/uav_0/isaacsim_manipulator/**{joint_states,position_commands}. The whole first group comes
from ONE launch argument (`namespace:=uav_0/fsc_open_manipulator`) because the yaml keys were
flattened to the `/**/<node>:` wildcard the same day — so the "launch namespace and yaml
nesting must change TOGETHER" trap above is GONE. **The wildcard must be FLAT**: `/**:` with
the node nested under it parses and matches nothing (verified live at two namespaces). The
GS `__ns` and the launcher's stack-readiness gate both derive from the launcher's single
`ARM_NS`, and fsc_autopilot_ros2's `armff_joint_topic` became the relative
`fsc_open_manipulator/joint_states`. NOT flown yet.
Commands: Command.md §7.9. **ARM-REPO CONSOLIDATION same day (user request — supersedes
this entry's workspace paths)**: the two arm repos (workspace repo + ROBOTIS fork) were
merged into ONE, fork-rooted, histories preserved; on GitHub the fork takes the
`fsc_open_manipulator` name and the old workspace repo is archived. New layout on this
machine: `~/colcon_ws/src/fsc_open_manipulator` (packages at repo root — custom_gui,
open_manipulator_x_isaac_bridge, the controller, the ROBOTIS set) with the three pinned
Dynamixel deps vcs-imported beside it; `~/ros2_ws/src/fsc_open_manipulator` is GONE
(deleted after the merge; the rosdeps overlay at `~/ros2_ws/rosdeps` is a separate path
and STAYS). `FSC_OM_ARM_WS` now points at the workspace `~/colcon_ws` (launcher derives
`$FSC_OM_ARM_WS/install/setup.bash`); builds are plain `colcon build --packages-select`
from the workspace root, no `--base-paths`/COLCON_IGNORE machinery. ROBOTIS releases
merge straight in (`git merge upstream/humble`). Rebuilt + loopback re-validated after
the move (hold/move/go_home PASS).

**GEOMETRIC + L1-ADAPTIVE STACK (2026-08-20, user request — the controller of Cai et al.,
"An experiment study for unmanned aerial manipulator systems with L1 adaptive augmentation
of geometric control", CEP 164 (2025) 106418; PDF in `docs/docs_aerial_manipulator/`).**
A THIRD external controller for the §7.9 rig, implemented as a parallel fork in
fsc_autopilot_ros2 (`single_aerial_manipulator_geometric_l1_direct_actuation`, node
`autopilot_geometric_l1_direct_actuation_node`, params
`params_single_aerial_manipulator_geometric_l1_direct_actuation_t650.yaml`, stack script
`start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh`). DIRECT mode runs the
paper's **constant-position/constant-yaw specialization** at 250 Hz straight from the
position reference (the message has no jerk/snap/yaw-rate fields, so ω_d = ω̇_d = 0; do not
claim full time-varying-trajectory equivalence): u_b = geometric SE(3)
(position+attitude in N/N·m, with the arm's measurable wrench compensated through the LIVE
r_os from the arm encoders — at home it reproduces the flown −0.0195 N·m/N to the reported
precision) plus
u_L1 = L1 augmentation (predictor on [v;ω], piecewise-constant adaptation with Ḡ inverted
once per distinct feedback capture stamp (4 ms fixed physics step in Isaac, stamp delta on
hardware), LPF; matched channel only; predictor fed the allocator's
ACHIEVED wrench as anti-windup; injection clamped ±6 N / ±1.5/0.8 N·m). No attitude
integral, no UDE in DIRECT — L1 replaces both; SAFETY is the unchanged baseline for
takeoff/abort. Attitude gains carried from §7.10.4's sim-validated 1.0/0.55 (the paper's
KR 1.92/Kω 0.3 gives ζ≈0.2 against the MN4010 rotor-lag pole — do not "restore" them on
this plant); position pair is the paper's mass-scaled and softened (Kp 4/4/8, Kv 6/6/10);
L1 poles/bandwidth are a conservative first-flight tune below the paper's 65/80/ωc 6, not
an RTF correction. **FLOWN 2026-08-24 — see the campaign entry below.** Before that it
was only **static-harness checked 2026-08-20**: synthetic harness
(§7.10's pre-flight standard) reproduced the hover split 0.597/0.540 exactly, u_L1 ≡ 0 at
a model-consistent hover, and the adaptation converged to the numerically correct matched
estimate (γ_f + f_applied = mg ± 0.8 N) against a frozen plant. First campaign = hover
test. Commands + debug-array layout: Command.md §7.12.
**DERIVATION + IMPLEMENTATION VERIFIED 2026-08-20 (Command.md §7.12.2).** The paper's
law re-implemented in its own NED/FRD frame agrees with this ENU/FLU implementation to
4e-16 N·m over 400 random states with ω up to 1.5 rad/s; the closed loop reproduces the
paper's eq (27) `I ė_ω = −K_R e_R − K_ω e_ω` exactly (the r_os feedforward cancels d_im1
and ω×Iω); and the LIVE node matches an independent Python reference to 3e-8 N·m at
non-trivial states — **including ω = 1.1 rad/s, which the hover bench could never test
because every centripetal term is identically zero at hover**. Three defects were found
and fixed in the pass: (a) the UDE was integrating the ROBUST controller's command in
DIRECT, a command this node never applies, corrupting the SAFETY abort — now fed the
achieved collective; (b) the L1 predictor used forward EULER, measured to diverge at the
dt gate's 50 ms ceiling with the paper's own A_s = −65/−80 — now the exact zero-order-hold
update, the same fix `lagged_thrust_curve.py` records for the rotor lag; (c) **the law and
the predictor used different gravity constants** (9.80665 vs 9.81), which a 30 s hover soak
exposed as u_L1 settling at −0.119 N instead of zero — the two form one loop, so a constant
model disagreement is indistinguishable from a real disturbance and is amplified by
≈1/(|A_s|·T), 10x here. Every constant shared by the law and the predictor must come from
one place. NOT a defect but needed to read logs: the PWC estimate under-reads a constant
disturbance by e^{A_s·T} (10% in sim, ~50% at the paper's hardware poles).
**POST-REVIEW CORRECTIONS, 2026-08-20:** restored the intended T650 body mass to 2.95 kg
(the accidental `2.95 - rotor_mass` made the live AM plant 3.662249 kg against the YAML's
3.746170 kg); the L1 launcher now enforces that total in Isaac; adaptation advances only on
new feedback samples rather than wall-timer repeats; and arm FK subtracts the bare-airframe
CoM `[0.00001019,-0.00030900,0.04178889]` m in model axes so r_os has the paper's O→S
definition. The corrected live-node harness was rerun over hover and three non-trivial
states, agreeing with the independent reference to 1.34e-6 N, 3.14e-8 N·m, and 1.79e-9 m
in arm FK; its measured LPF coefficient implies the configured 4.000 ms sample period.
The earlier claim that the paper printed a negative r_os block in G(R) was a
typesetting misread: the minus belongs to the exponent in I^-1; paper and code agree after
the NED/FRD→ENU/FLU thrust-sign conversion.

**AM-T650 GEOMETRIC + L1 FIRST FLIGHTS — +20% kf AND +10 mm CoM AT ONCE (2026-08-24,
user request).** The §7.12 node flew for the first time, twice, matched, with both
robustness injections active and **no gain touched**. `armff_mismatch_x` 0.0085 →
**0.010 m** (the round top of the 8–14 mm band the 0819 hardware flights bracket, not
its mid-point); `alloc_thrust_coeff` unchanged at 5.6159172e-05 = **+20%** of the plant's
4.679931202e-05. Mission = §7.13.2's, so the numbers sit beside the bare-T650 campaign's;
driven by the SAME `l1_payload_campaign_driver.py` with `--hover-z 1.2 --land-z 0.35`
(no fork needed — it is namespace- and debug-length-agnostic), scored by a NEW
`docs/sim_to_real_t650/tools/am_l1_robustness_metrics.py` that knows this plant and reads
the AM-only debug fields. Data: `docs/sim_to_real_t650/am_l1_robustness_20260824/`.
Runs A/B agree to the third decimal: u_L1 thrust **7.303 N** (predicted 7.350, **73% of
the 10 N clamp, 0 samples railed**), f_b 36.796 = mg, commanded 44.099 N →
44.099×(4.679931/5.6159172) = **36.75 N delivered = exactly hover** (so the mismatch is
demonstrably active and L1 absorbs all of it — the baseline never notices), motors
0.597/0.540, |e_R| ≤ 0.0073, arm at HOME and **debug [39] = 2 (live encoders) for 100% of
samples**, touchdown 0.339 m at 0.18°.
- **THE ACCOUNTING TRAP, and it inverts the verdict.** Scoring the CoM injection as
  `dx_err*f_b` = 0.368 N·m makes L1's +0.2354 N·m look like a **64%** cancellation, i.e.
  a failure. It is not: the moment channel carries the **+20% allocation error too**, and
  there the two injections partly OFFSET. What the plant needs commanded is
  `-dx_true*T*(kf_c/kf_p)` = −0.8622; the baseline commands −1.0837; so the required u_L1
  pitch is **+0.2217** and the measured +0.2354 is **106% of it**. **Never score one
  injection without the other when both are on.** I published the 64% number before
  catching this — the metrics script now does the full arithmetic.
- **The residual 0.011 N·m is why position error exists, and it is the paper's STRUCTURE,
  not a tune.** Attitude loop is pure P → parks at e_R ≈ 0.0039 rad; position loop is pure
  P/D → parks at `T·e_R/Kp` ≈ 3.5 cm. Measured settled X offset 3.4–4.9 cm (was ~3 cm at
  the old 8.5 mm injection — it scales). §7.13's finding (b) mechanism, different source.
- **Do NOT read the X-STEP "final error" as settled.** X has a slow residual mode of tens
  of seconds (still moving 0.04 m per 6 s at the end of the 15 s window), so the −80/−99 mm
  on the +0.5 m X step is a WINDOW ARTIFACT. Z does not close through the attitude loop and
  does settle: −7.1 to −7.6 mm, both runs, both directions.
- **PX4 never disarms this rig** (`Disarming denied: not landed`, forever, even seated at
  exactly 0.305 m with the reference pushed below it). Identical to the bare-T650 campaign
  at `--land-z 0.28`, so it is NOT the AM and NOT the land target — do not "fix" it by
  lowering the land target (0.12 tipped the whole-body rig). Full step-0 clean and relaunch
  between missions; the driver already refuses to start against an armed vehicle.
- Expected, not a fault: the DIRECT→SAFETY abort balloons ~560 mm (§7.13 finding (c) —
  the SAFETY UDE books the DIRECT-only mismatch as a disturbance). Sim-injection artifact.
- **Doc drift fixed:** Command.md said `l1_control_debug` is 41 elements; the code has
  published **44** since 2026-08-20 ([41..43] = the injected mismatch, which is what lets
  an analysis recover the TRUE r_os as [36..38] − [41..43]).
- **GROUND-STATION PANE NEVER STARTED ON THIS MACHINE, and it was the INTERPRETER.**
  `python3` first on PATH is the Isaac env's 3.11, which has neither PyQt5 nor an rclpy
  built for it, so the `gui` pane of every stack script died at import while every other
  pane was fine. `start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh` now
  PROBES for an interpreter that can import both (override `FSC_GS_PYTHON`), non-fatally —
  it picked `/usr/bin/python3` and the station came up. **The other 12 stack scripts still
  have the bare `python3`.** Verified live in DIRECT: drone GS shows Controller
  "Geometric+L1 Direct Actuation", Vehicle "AM-T650-L1", and **N/T pies M1 59.6 / M2 54.0 /
  M3 59.7 / M4 54.2%** — the ros2_ground_station_gui uncommitted `geometric_l1_direct_actuation`
  motors_debug entry working (`ros2 topic info` on that topic reads Publisher 1 /
  Subscription 1); the arm GS shows POSITION mode, the arm at HOME, live joint limits
  adopted from the controller, and today's new EE Whole-Body tab. Screenshots in the
  campaign directory. **The monitor is DPMS-off on this machine — screenshots come back
  all-black until `xset dpms force on`.**
- Dead code worth knowing before someone trusts it: the GS's `direct_mode_client` is
  hardcoded to the `direct_actuation` namespace, so it would NOT reach this fork — but
  nothing calls it. The Controller tab uses the fork-agnostic
  `fsc_autopilot_ros2/list_controllers` + `activate_controller` pair, which is why the tab
  works on every fork.
Full campaign: Command.md §7.12.5.

**BARE-T650 GEOMETRIC + L1 (2026-08-21, user request — the no-arm parallel of the §7.12
rig).** fsc_autopilot_ros2 (dev_CCM) gained a second L1 fork,
`single_drone_geometric_l1_direct_actuation` (executable
`autopilot_drone_geometric_l1_direct_actuation_node` — decorated because the AM fork owns
the plain name. **The decoration is INFIXED** ("autopilot_" + "drone_" + the rest), so
neither name is a substring of the other and `pgrep -f` on either matches exactly ONE fork
— every stale-node guard must therefore list BOTH explicitly, which is a defect found and
fixed 2026-08-21 after all three L1 scripts had been shipped with the decorated name
missing, i.e. unable to detect a stale instance of their own controller),
namespace `nodelib::single_drone_l1`: the AM L1 client minus the whole arm model — r_os ≡ 0
on a bare airframe (the paper's O→S with O = bare CoM), no `armff_*` keys, no joint-state
subscription, debug array 37 elements ([0..35] = the AM layout, [36] = L1 active). Same
frozen mode-service name `fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode`.
Yaml `params_single_drone_geometric_l1_direct_actuation_t650.yaml` = bare-T650 plant/SAFETY
numbers (mass 3.033921, posctl 1.0/3.0, ude_gain 2.0) + the AM campaign's L1 tune (A_s 2/2,
ωc 6, 10 N thrust bound) + the SAME deliberate +20% `alloc_thrust_coeff` (5.6159172e-05;
plant truth 4.679931e-05) — the §7.12.4 robustness protocol minus the r_os injection (no
arm model to corrupt; the thrust mismatch is the whole test). Scripts: Pegasus
`scripts/indoor_sim/start_t650_geometric_L1_adaptive_direct_actuation_sitl.sh` (plant twin of the
geometric launcher; gates on the DECORATED node name, which is what lets it refuse the AM
stack flying this plant), stack
`scripts/isaacsim/start_geometric_l1_direct_actuation_t650_stack.sh`, hardware
`scripts/indoor_exp/start_geometric_l1_direct_actuation_stack_t650.sh` (greps the yaml and
warns loudly while the +20% coefficient ships — restore the bench motor pair before any
hardware flight). The older geometric stack/exp scripts' stale-controller lists gained the
missing fork names. Commands + full campaign results: Command.md §7.13. **SIM-SWEPT
2026-08-21, three findings that will bite again:** (a) the geometric sibling's attitude
pair kr 2.4/kΩ 0.73 is NOT safe on the L1 node — its rate-loop crossover (9.5–11.4 rad/s)
sits on the MN4010 lag pole and the L1 torque channel's ωc=6 LPF + one-sample delay eat
the remaining margin; measured as a ~2.4 Hz mode that decayed in one run and grew +3%/s to
a flip in the repeat (marginal, scatter-decided — repeat-test everything). Shipped kr
1.2/kΩ 0.45, 3/3 clean with the step transient decaying within 4 s every run. (b) A
constant ~4–5 cm −x/−y hover offset is the PLANT: γ̂_um reads a real ~0.20 N lateral force
(x650_new.usd's built-in ~0.4° thrust-axis/body-frame misalignment), and the paper's
integrator-less pure-P position loop parks exactly at F_lat/Kp (Kp·e_p = γ̂_um measured to
a few percent, 3/3). (c) The
DIRECT→SAFETY abort carries a one-off ~6 N vertical surge (~0.5 m balloon): the SAFETY-path
UDE books the DIRECT-only mismatch as a disturbance and hands it to a SAFETY loop whose
thrust model is honest — an artifact of the ASYMMETRIC sim injection that does not transfer
to hardware (a real kf error lives in both models). Settled hover: u_L1 thrust +5.92 N in
all three runs (predicted +5.95 — the mismatch closed), motors 0.502 symmetric, u_L1
torque ≤0.053 N·m.

**BARE-T650 L1 CARRYING A 769 g PAYLOAD (2026-08-24, user request — the loaded
counterpart of the §7.13 campaign).** Last week's hardware loaded flight saturated the L1
thrust clamp; the report `docs/docs_aerial_manipulator/L1 Against Battery Fade.pdf`
(artifact "L1 Against Battery Fade") showed **two thirds of that demand was bookkeeping**
— `vehicle_mass` left at the bare 3.033921 kg while the vehicle weighed ~3.88 kg cost
8.28 N of the 12.8–15.3 N asked for; only 4.4–6.8 N was the battery fade being tested.
Both of the report's recommendations are now applied, **no gain touched**:
- **Isaac plant carries the payload.** New `PEGASUS_PAYLOAD_MASS` hook (kg, default 0.0 =
  every existing rig bit-identical) read by `application/px4_base/05_px4_single_drone_t650.py`,
  plumbed through `start_single_drone_x650.sh`'s Isaac pane command line (baked in, NOT
  exported — the tmux-server-env trap that already bites `PEGASUS_PX4_LOCKSTEP`), and
  defaulted to **0.769** by `start_t650_geometric_L1_adaptive_direct_actuation_sitl.sh`.
  Total **3.802921 kg**. Payload is mass at the body CoM, **inertia UNCHANGED** — so the
  sim does NOT reproduce the hardware payload's ~4 mm forward CoM shift (0.5→0.8 N·m
  standing pitch moment) and its u_L1 torque reads far smaller than hardware's.
- **yaml (`params_single_drone_geometric_l1_direct_actuation_t650.yaml`, dev_CCM):**
  `vehicle_mass` 3.033921→**3.802921**, `l1adapt_max_thrust_n` 10→**18**, and
  `vehicle_thrust_scaling`/`vehicle_idle_thrust` 0.040232/0.203169→**0.035935/0.238967**
  — the SAFETY tangent linearization **re-derived, not copied** (+25.3% mass moves the
  hover point off the bare anchor, the AM-T650 +23% lesson). Hover command 0.5025→0.5741.
- **`alloc_thrust_coeff` stays 5.6159172e-05.** The report's finding that the REAL k_f is
  the MN4010 bench value 4.540431e-05 (i.e. `THRUST_FIT_FACTOR` 1.030724 is a sim-only
  hover-command match) is a **hardware**-yaml item: it wired nothing into the simulator, so
  Isaac's plant truth is still `t650_params.ROTOR_CONSTANT` = 4.679931e-05 and +20% of it
  is unchanged. THE PLANT-TRUTH NUMBER IS WHATEVER ISAAC APPLIES.
**FLOWN 2026-08-24, 2/2 matched full-mission SITL runs** (§7.13.2's mission), agreeing to
the third decimal: u_L1 thrust settles **7.416 N** (predicted 7.461) = **0 of 28750 DIRECT
samples on the rail**, whole-DIRECT range 6.79–8.23 N;
f_b = 37.35 N = mg exactly while the commanded collective is 44.77 N (**the baseline law
never notices the kf error — L1 absorbs all of it**, the report's Flight-A signature);
44.769×(4.679931/5.6159172) = 37.307 N delivered = exactly hover, so the mismatch is
demonstrably active; motors 0.5741 symmetric; u_L1 torque 0.047 of 1.5 N·m; soak tilt
0.15° p-p; steps within the bare campaign's numbers; touchdown 0.305 m.
**Run C — last week's config on the loaded plant CANNOT TAKE OFF**, a stronger result than
the expected sag: commanded collective **32.1 N against 37.31 N of weight** (bare-mass
gravity FF 29.76 + ~2.3 position term), and the UDE that would supply the missing 5.2 N is
gated off by `ude_height_threshold: 0.4` below 0.4 m — which the vehicle cannot reach
without it. Ground deadlock. With the mass wrong the demand is 15.005 N, so the raised
bound alone would have been consumed exactly: **both changes were needed, not either one.**
**Two traps worth more than the numbers:** (a) `PEGASUS_PAYLOAD_MASS` and `vehicle_mass`
must move together — launcher bare + yaml loaded gives the controller a 769 g phantom;
(b) **never fly two missions off one launch.** PX4's land detector never sees this rig land
(`Disarming denied: not landed` forever) so the vehicle stays armed, and the next run finds
PX4 latched "in flight" AND the SAFETY UDE integrating the ground reaction — the takeoff
then produces **zero lift with no error message** (measured: 60 s at ref z=1.0, altitude
constant to 3 decimals). Full step-0 clean between flights; the campaign driver now refuses
to start against an armed vehicle. Land to z = 0.28 (resting height 0.305 m, not the 0.07 m
spawn). Tooling + data: `docs/sim_to_real_t650/tools/l1_payload_campaign_{driver,metrics}.py`
and `docs/sim_to_real_t650/l1_payload_campaign_20260824/`; both need `/usr/bin/python3`
(the default python3 here is a uv 3.11, rclpy is built for 3.10). NOT covered: battery fade
(the sim has none — this demand is constant where hardware's grew 4.4→6.8 N over 2.5 min)
and the payload CoM offset; budget for both on top of the measured 7.5 N.
Full campaign: Command.md §7.13.3.
**BOUND 15→18 N and the GS ROTOR DISPLAY, same day (user request).** 15 N covers the
sim's kf mismatch alone (measured 7.42 N); a real loaded sortie also carries **battery
fade, 4.4→6.8 N growing over 2.5 min** — worst-case sum **14.2 N**, so 18 N leaves ~21%
margin, and even railed the commanded collective is 62.8 N against the allocator's 99.8 N
ceiling. The matched estimate tracks it automatically (`2.0*max_thrust_n`), so there is no
separate estimate bound. **The 15.3 N hardware peak is NOT the sizing number** — 8.28 N of
it was the un-updated `vehicle_mass`; with the mass fixed that flight asks ~7.0 N, so
**u_L1 heading for 15 N with the mass correct means the mass or kf is wrong, not the
bound.** Re-flown (run D, 90 s soak): 7.415 N, identical to A/B, 41.2% of clamp, no rail.
**GS N/T rotor pies read 0.0% for this rig — the SAME per-namespace subscription-list
defect as the whole-body fork's on 2026-08-23**, in
`ros2_ground_station_gui/src/ROS_Node/ros_single_drone_control.py` (shared repo,
`dev_robotic_arm`, left UNCOMMITTED). There are exactly **four** direct-actuation
namespaces across the seven forks and forks SHARE them in pairs (AM + bare-drone):
`direct_actuation`, `geometric_direct_actuation`, `geometric_l1_direct_actuation`
(the missing one — both L1 forks), `whole_body_direct_actuation`. All four now listed;
**add the namespace whenever a fork is added.** Diagnosis is one command:
`ros2 topic info .../<ns>/motors_debug` read `Publisher 1 / Subscription 0`, and `1 / 1`
after. The other two gates were already fine (controller-name is a SUBSTRING match on
"Direct Actuation"; `connected` is PX4 `pre_flight_checks_pass`). **Needs a GS restart.**
Verified live in DIRECT: M1 57.6 / M2 57.3 / M3 57.4 / M4 57.4% against the 0.5741 hover
command. SEPARATE, NOT FIXED: on this machine the GS pane of all 13 stack scripts never
starts — `python3` is the Isaac env's 3.11 and PyQt5 exists only for `/usr/bin/python3`;
launch it by hand with that interpreter (environment issue, not a script bug).

**HARDWARE/SIM CONFIG SPLIT FOR THE BARE-T650 L1 RIG (2026-08-24, user request).**
`params_single_drone_geometric_l1_direct_actuation_t650.yaml` (dev_CCM) is now the
**HARDWARE** config; the simulation one is preserved verbatim as **`..._t650_sim.yaml`**
(the file 7.13.3's three SITL runs flew) and the isaacsim stack script points there. The
indoor_exp script keeps the unsuffixed name. **Motivated by the 2026-08-19 AM geometric
first hardware flight, which took off on the unmodified SIM yaml and diverged in 1.3 s** —
see [[am-geometric-first-hardware-flight]]; two of its three measured causes were the two
allocator constants. Exactly **10 keys differ**, all 81 present in both, **every gain
identical**: `alloc_thrust_coeff` 5.6159172e-05→**4.540431e-05** (bench — the 2026-08-21
hover balance put the real coefficient on the bench value to −0.08%, so the sim number is
3.0% optimistic fresh and 10.3% by end of sortie); `alloc_rotor*_km` ±0.052867→**±0.018164**
(bench, = c/kf; the sim value carries t650_params' ×3.0 stand-in for the unmodelled
`I_rotor·ω̇` — **2.91×, and NOT inert**: measured yaw response 0.35× commanded on 0819);
`l1adapt_fixed_sample_time_s` 0.004→**0.0** (measured stamp deltas; 0.0 is what actually
flew, recovered from the LPF recursion as 3.44/13.05/16.62 ms); `vehicle_mass`
3.802921→**3.878000**; `vehicle_thrust_scaling`/`idle_thrust`→**0.036128/0.247419**
(re-derived at bench kf AND that mass); `vehicle_name`.
**`vehicle_mass` 3.878 is INFERRED from the hover balance, NOT WEIGHED** — 75 g over
bare+payload, likely mounting hardware. Preferred over the arithmetic 3.802921 because
75 g = 0.74 N the augmentation must find, the exact error class being removed; but weigh
it. Alternative pair for 3.802921 is 0.036483/0.244076, written into the yaml.
**Kept on purpose:** A_s 2/2 + ωc 6 **are the hardware values** (the report read A_s = −2
out of the 0821 logs; 94–95% delivered, flat 0.09–0.17 N residual, measured under-read
2.5–3% vs the e^{A_s·T} prediction 1.6%) — the paper's 65/80 chases the last 5% but is its
own step; `l1adapt_max_thrust_n` 18 (with bench kf the allocator error mostly goes, leaving
battery fade 4.4→6.8 N — expect u_L1 in a **4–7 N band walking upward**, and 15 N means the
mass or kf is wrong); `l1geo_kr/kΩ` 1.2/0.45 — **the one block carried over UNTESTED**
(the geometric sibling says "hardware start 1.2/0.75" since its extra softening was an RTF
artifact, but this node closes the L1 torque channel through an ωc=6 LPF + one-sample delay
that the geometric node never pays: at 2.4/0.73 in sim a ~2.4 Hz mode decayed once and grew
+3%/s to a flip on the repeat). **Never raise kΩ against the lag pole.**
The indoor_exp guard was widened from the one +20% string to all five sim values and now
PRINTS POSITIVE CONFIRMATION of the three that must be right plus the mass — a silent pass
used to be indistinguishable from a grep that stopped matching. **Trap: the `0.0` check
must be anchored to end-of-line, or it matches the sim's `0.004` as a substring and reports
OK on the wrong file.** Verified by launching the node on the hardware yaml under
`uav_test` and reading every changed key back. NOT FLOWN in this form; geometric+L1 remains
unflown on hardware on any airframe. Details: Command.md §7.13.4.

**T650 PLANT THRUST CONSTANT RE-ANCHORED ON MEASURED FLIGHT DATA, AND THE AM-T650
GEOMETRIC+L1 SIM RETUNE (2026-08-25, user request).** The simulated T650 was
**15.8% stronger than the real vehicle**, and that surplus was exactly the standing
thrust compensation the L1/UDE augmentation was always seen carrying in sim.
`t650_params.THRUST_FIT_FACTOR` **1.030724 -> 0.890066** (`ROTOR_CONSTANT`
4.679931e-05 -> **4.041283e-05**); previous values, the derived block and the full
derivation are kept in-file for reversion.
- **MEASURED, not fitted to a hover command.** `kf_eff = m(g-a_z)/(cos(theta)*sum
  omega_i^2)` back-solved from the 0820 stepped-payload ulogs, **log_252** (+719 g,
  the ARM-EQUIVALENT payload: 3.753 kg vs the AM's 3.746, hovering at the same 0.6159
  collective). All seven flights sit **below** bench (0.883-0.954x) and the spread is
  NOT payload — logs 249->250 hover at the same collective yet differ **7.8%** because
  a battery pack was swapped. Regression over all seven: `ln kf = -16.767 +
  2.149*lnV - 0.264*u`, R^2 = 0.997, i.e. **kf_eff ~ V^2**. The old factor was a
  hover-match to flight C's fresh pack, so it encoded ~25.3 V as a propeller property.
- **The correction is folded into kf, NOT the omega-map, on purpose.** The physically
  faithful fix is a voltage anchor on `ZERO_POSITION_ARMED`/`MAX_ROTOR_VEL`, but the
  DIRECT allocator carries its own copy of that map (`alloc_omega_idle/max`), so
  scaling only the plant's would inject a second undeclared mismatch and a robustness
  campaign needs ONE knob. Cost, stated in-file: sim yaw-torque-to-thrust runs ~12%
  high. `YAW_TORQUE_FIT_FACTOR` left at 3.0 (its fit was against yaw SHAPE).
- **`alloc_rotor*_km` MUST MOVE WITH kf** — it is `c_plant/kf_plant`, so the allocator
  commands `tau_z = km*f` and this ratio is what makes the deliberate
  `alloc_thrust_coeff` injection scale thrust AND yaw by the same factor. Missing it
  left yaw delivered at 0.965 of command while thrust was at 0.833. Now **0.0612219**;
  confirmed by the startup print's yaw scale returning to 3.14041 N.m.
- **`vehicle_thrust_scaling`/`idle_thrust` are a function of plant kf AND mass** and had
  to be re-derived (AM: 0.036206/0.236457 -> **0.038962/0.261777**). SAFETY then holds
  z to -3.5 +- 4.8 mm. **THE OTHER SEVEN T650 SIM YAMLS ARE NOW STALE** on this pair
  (and on `alloc_thrust_coeff`/`alloc_rotor*_km`); only the AM L1 sim yaml was updated.
- **HARDWARE IS DELIBERATELY UNCHANGED** at bench `4.540431e-05`. The re-anchor makes
  the SIMULATED plant match the real vehicle; on hardware the plant IS the real
  vehicle. 4.041283e-05 is a battery state, not a motor property, and baking one
  sortie's state in would make the allocator **over-deliver ~7% on a fresh pack** —
  over-thrust at lift-off, the wrong direction. On a 4.20 V/cell pack bench is only
  1.8% optimistic (u_L1 ~0.7 N); the 0819 flight's 0.876 loop gain becomes **0.982**,
  and neither the bench-kf swap (0.903) nor the charge (0.953) gets there alone.

**AM-T650 GEOMETRIC+L1 `_sim.yaml` RETUNED at the new plant** (both injections live),
5 flights, harness `docs/sim_to_real_t650/tools/am_l1_tune_cycle.sh`, scorer
`am_l1_transition_metrics.py`, data `am_l1_tuning_20260824/`. Shipped: `l1geo_kr_x/y`
1.0->**1.5**, `l1geo_komega_x/y` 0.55->**0.70**, `ude_disturbance_lbz/ubz` +-10->**+-2**,
`l1adapt_omega_c` stays **6**. Abort balloon **560 -> 138 mm**, entry tilt 6.43 ->
3.81 deg, entry settle 18.7 -> 8.9 s, X-step overshoot 15.6 -> 2.6%.
- **The entry transient is the ATTITUDE LOOP, not a slow L1** (misdiagnosed first):
  u_L1 reaches 7.1 of its 7.35 N within **0.73 s**. What tilts the vehicle is the
  +10 mm r_os over-compensation, ~0.37 N.m of pitch on a `kr = 1.0` loop soft enough
  to park at 0.37 rad. The 511 mm xy excursion is purely downstream of that tilt.
- **The abort balloon is a UDE PHANTOM.** In DIRECT the UDE is fed the allocator's
  BELIEVED achieved collective (44.1 N vs 36.75 delivered), converges to a fictitious
  -7.35 N and dumps it into SAFETY. On hardware this is CORRECT (a real kf error lives
  in the SAFETY model too); in sim SAFETY's model is honest, so it is fiction. Capping
  the z bound caps the balloon: predicted 560*(2/7.35) = 152 mm, measured 136 and 138.
- **`l1adapt_omega_c` 6 -> 8 DESTABILIZES** (with kr 1.9): a growing **0.62 Hz** mode,
  steady tilt 0.25 -> 8.81 deg, abort tilt 38.6 deg. The AUGMENTATION drives it —
  `u_L1 tau_y` rms rose **14x** while attitude error rose 1.75x. Keep omega_c at 6.
  `kr 1.9` at `omega_c 6` is UNTESTED and its entry numbers looked better.
- **Steady position hold is ~22-29 mm, structurally**: `T*e_R_y/Kp` against the
  uncancelled CoM pitch residual. `e_R_y` and `u_L1 tau_y` are identical to 3 decimals
  across runs; one 10.6 mm run was a lucky outlier. Do not read a single run.
- **YAW CLAMPS sized against the ALLOCATOR** (`l1geo_max_torque_z` 2.0->**0.40**,
  `l1adapt_max_torque_z` 0.8->**0.25**, sum 0.65 <= the 0.654 N.m FREE budget). With
  bench km the yaw ceiling is 0.8724 and the allocator desaturates airmode-style past
  0.654 — preserving torque but TRADING COLLECTIVE, so a railed yaw channel appears as
  an **altitude kick** (+8 N at 0.8, +12 N at 1.0). RAISING these is not a safety move:
  above 0.8724 they deliver nothing. Sim-validated: peak yaw demand 0.0216 N.m = 3% of
  budget, 0.00% railed. **A CoM offset produces ZERO yaw moment** (`r_os x (0,0,T)` has
  no z component) — the 8-9 mm dx uncertainty loads `l1adapt_max_torque_xy` (1.5,
  untouched, ~5x margin).
- **HARNESS TRAP: never gate arming on `vehicle_status.pre_flight_checks_pass`.** It is
  **false on this rig while it is armed and flying** (EKF `cs_yaw_align`/`cs_ev_yaw`/
  `cs_ev_pos` all true, commander "Ready for takeoff!"). Gate on the
  `estimator_status_flags`. The same false flag is what pinned the ground station's N/T
  rotor pies at 0.0% for a whole DIRECT flight — fixed in `ros2_ground_station_gui` by
  gating the display on motors_debug FRESHNESS instead of on `connected`.
- Score the abort balloon in a window ENDING AT LAND_WAIT; a fixed 25 s window scores
  the intended descent as balloon (557 mm on a run whose real balloon is 136).

**WHOLE-BODY DIRECT ACTUATION — THE COUPLED LAW IN fsc_autopilot_ros2, ARM IN TORQUE MODE
(2026-08-22, user request — controller.py ported to C++, both ground stations live).** A fourth
AM fork, `single_aerial_manipulator_whole_body_direct_actuation` (node
`autopilot_whole_body_direct_actuation_node`, substring-safe vs all siblings), runs THE law
(impedance + GMO + DLS + saturation-consistent coupling) at 250 Hz in DIRECT, commanding the
4 rotors (ActuatorMotors) AND the 4 arm joint torques over ROS2. Everything is NEW FILES
(user's incrementality rule): `wb_model/wb_controller` (byte-faithful Eigen port, MODEL kept in
AM_realign's y-forward frame + one `frame_adapter.hpp` boundary), `wb_reference_builder`
(drone GS position/yaw → x_cd chain via a CoM anchor captured at DIRECT entry; arm GS joint
target → slewed q_d → FK-implied EE reference on the law's own model), client with the L1
fork's SAFETY/watchdog/failsafe plumbing, GATED DIRECT entry (pos/vel/arm-err + freshness —
the sequencing rule made mechanical), and the arm-hold torques streamed in SAFETY too (one
continuous torque source across the switch). Arm transport: new `ExternalTorqueController`
(fsc_open_manipulator, derives TorqueControllerBase: pass-through + clamp + position-limit
pull-back + stale-PD-fallback with re-arm hysteresis + zero-on-deactivate) → new
`IsaacTopicEffortSystem` (effort-only bridge twin; zero-seed on activate, NaN = don't publish)
→ `isaacsim_manipulator/effort_commands` (the name Arm Topic Naming.md had reserved) → new
`06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py` (05's plant; fresh efforts applied
clipped ±3.0, stale → PD+gravity hold at the LATCHED pose — a torque stream must never latch).
Launchers: Pegasus `scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh`
(06 + torque arm stack + arm GS `controller:=external_torque_controller simulation:=true`) +
stack `scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh`;
all existing stale-node guards + stop script + the drone GS motors_debug gained the fork.
**PARITY-LOCKED**: `generate_wb_truth.py` (Pegasus, utils/) builds the T650 model variant with
the CORRECT model-frame I0 = [0.07475072, 0.08616121, 0.083401015], Ixy −0.0089 (computed, NOT
05's un-rotated `_body_dI`; base mass 3.1095500) and dumps 220 cases + GMO rollouts; the gtest
`WbParityTest` (fixture `tests/data/wb_truth_t650.json`) holds the C++ law to ≤1e-8. Model in
code (`t650Defaults`), gains in yaml — control_params.py's split. **FIRST DIRECT ENGAGEMENT
GREW A RATE OSCILLATION (2026-08-22): the in-process-validated attitude tune k_R=4/k_w=1.5/
M_r_d=AM_realign is NOT valid over the external DDS loop** (transport latency 03 never had) —
tripped the 360 dps watchdog 20 s in, at 14° tilt. Shipped tune: k_R=2.0/k_w=1.1 and M_r_d =
the ACTUAL T650 M_r diag at home [0.077681, 0.090738, 0.083401] (the old numbers under-stated
roll/pitch inertia 20–40%, hidden extra gain via M_r·M_r_d⁻¹); wn 6.9→4.9 rad/s vs the
10.0 rad/s rotor-lag pole. **SIM-VALIDATED same day, 2 flights / 3 engagements after the
retune, 179 s of DIRECT**: EE rms 2.36 mm, CoM rms 3.2 mm, u1 = mg exactly, peak joint torque
0.87/3.0, zero saturations, |e_R| ≤ 0.037, no resonant mode; in-flight arm GS step tracked <1°
in 4 s with the base held to 5 mm; abort bumpless, re-entry gated, touchdown 0.307/0.311 m.
tau_max = 3.0 appears in THREE places (wb yaml / ExternalTorqueController / 06) — keep one
number. Commands: Command.md §7.14. Small trap found live: `pgrep -f` stale-node guards also
match a SHELL whose command line merely contains the node name — don't monitor a launch from a
same-name-carrying foreground shell.

**WHOLE-BODY +20% KF ROBUSTNESS TEST FAILED (2026-08-22, user-requested injection).** The
controller allocator used `5.6159172e-05` while 06 kept the calibrated
plant truth `4.679931e-05` and measured rotor pole `10.0265 1/s`. A guarded constant-hover
run entered DIRECT with zero anchored CoM error but was aborted after 7.68 s at 30.45 deg
tilt: altitude 1.148 -> 0.796 -> 1.385 m, CoM error 1.504 m, `d_hat_z` -9.12 N, `u1` peak
49.06 N, and two arm joints at the 3.0 N.m clamp for 52.9% of DIRECT samples. Rotor
unallocated thrust remained numerical zero, all arm/DDS streams stayed live, and the plant
and controller printed their intended distinct coefficients, so this is the present GMO
tune failing the multiplicative actuator mismatch rather than a transport or injection
fault. Full protocol and numbers: Command.md §7.14.2.

**WHOLE-BODY +15% KF HOVER PASSED AFTER RETUNE (2026-08-22).** Current yaml belief is
`5.38192065e-05` against the unchanged `4.679931e-05` plant and `10.0265 1/s` motor pole.
The hover-only tune uses k_v=12, k_w=1.5, EE Ky/Dy=2/4, GMO Ko=0.5/0.1/0.1, and an optional
joint-posture PID (Kp/Kd/Ki=2/0.25/0.05, integral torque clamp 0.8 N.m) to select the
controller-smoothed home branch without the instability of the rejected Kp=8 cases
(**that PID no longer exists — DELETED 2026-09-05, see the entry below**). Two
independent clean 90 s runs had zero saturation, <=1.55 deg tilt, 2.0-3.3 mm steady CoM RMS,
u1=42.262 N and d_hat_z=-5.512 N. Final arm q was within 2.8 deg of [0,40,40,0] and still
converging. This validates constant hover only, not position or EE steps. Restore
`4.679931e-05` for matched-model or hardware-oriented use. Details: Command.md §7.14.2.

**ONE ARM REFERENCE, ONE PLANNER PER MODE — THE TORQUE CONTROLLER NOW ONLY
TRACKS (2026-09-05, user design).** The min-jerk generator lived inside
`TorqueControllerBase`, so in whole-body DIRECT it ran IN SERIES with the
whole-body planner's B-spline plan: the whole-body planner poked `target_joint_setpoint` and the
controller re-profiled it. The 0902/0903 flights measured the arm REPLAYING the
whole-body planner's move **3-4 s late** with endpoints agreeing exactly. Fixed
structurally, all new/additive:
- **NEW node `arm_planner`** (`open_manipulator_x_custom_controller/
  src/arm_planner_node.cpp`, ~320 lines, C++). Owns the arm reference
  in **SAFETY only** — including the ground station's
  `<ctrl>/target_joint_setpoint` + `<ctrl>/go_home` interface at the SAME names,
  because the controller no longer advertises them — runs the SAME min-jerk
  profile (moved verbatim, so the tuned motion is preserved), and streams
  `<ctrl>/reference_joint_trajectory` (`trajectory_msgs/JointTrajectory`) at
  100 Hz. In DIRECT it goes **SILENT** and REFUSES targets/go_home rather than
  queueing them; it also tracks the measured joints while silent, so a SAFETY
  revert resumes from where the arm IS, not a stale goal.
- **`whole_body_planner`** publishes the same message on the same
  topic in DIRECT, every stream tick (100 Hz, was a separate 20 Hz
  `arm_sync_rate` timer — retired), carrying the plan's own `q_d`/`qdot_d`.
  The `target_joint_setpoint` poke and the one-shot `_publish_arm_sync` sites
  are gone: ONE publish site.
- **`TorqueControllerBase`** gained `external_reference_topic` (default `""`)
  and `external_reference_timeout` (0.2 s). Non-empty = PURE TRACKING: the
  internal generator is switched off AT THE INTERFACE (`~/target_joint_setpoint`,
  `~/sine_reference_trajectory`, `~/go_home` are NOT advertised), no activation
  homing (homing is a plan), and a stale stream **FREEZES** the last reference
  rather than inventing a fallback trajectory. **The empty default keeps
  `ComputedTorqueController`, `PositionController` and every bench/hardware
  workflow byte-identical** — only `torque_controller_isaac_aerial.yaml` opts in.
- **The reference FANS OUT, it does not loop through the controller.** The WB
  node subscribes `reference_joint_trajectory` directly (type changed
  JointState -> JointTrajectory, `wb_arm_reference_topic` repointed, builder
  setter renamed `setSmoothedReference` -> `setPlannerReference`). It used to
  read the controller's `smoothed_reference_joint_trajectory` — a CONTROLLER
  emitting a reference and the flight node consuming a controller's output, the
  wrong direction, and what hid the second generator. That topic keeps its name
  and type and is now TELEMETRY ONLY: `effort` = the commanded torque the arm GS
  plots, position/velocity = an echo. Measured live: reference 2 pub / 2 sub,
  echo 1 pub / 1 sub (the plot). `comparison_driver.py`'s wb path publishes
  JointTrajectory now too.
Loopback-validated (`open_manipulator_x_custom_controller/test/
test_arm_planner.py`, fake joint states + fake mode, no Isaac):
9/9 — 100 Hz in SAFETY, seeds/homes on the measured pose, tracks a GS joint
target (max inter-sample step 3.75 mrad), 0 messages in DIRECT, target and
go_home both refused there, resumes on the measured pose after revert. The
whole-body planner's own three tests pass, with `arm syncs 550` against 546 reference
samples (the full stream rate, was 20 Hz). **NOT yet flown.**
Topic contract: `docs/docs_aerial_manipulator/Arm Topic Naming.md`.

**THE JOINT-POSTURE PID IS LOAD-BEARING — REMOVED, FLOWN, REVERTED (2026-09-05,
user request; the term is not in the published law but this rig needs it).**
`wb_controller.cpp` adds a clamped joint-space PID on `(q_d - q, qdot_d - qdot)`
to `u3`. It was deleted (term, `WbGains` fields, all five
`wb_posture_kp/_kd/_ki/_i_max/_ramp_s`, keys in all three whole-body yamls),
flown, and PUT BACK. **Seven 75-90 s DIRECT soaks on the 7.14.1 sequence, one
variable at a time:** +15% kf WITH the PID = stable (CoM 2.9 mm, EE 6.5 mm,
tilt 0.04 deg, |e_R| 0.066, peak tau 0.74, 0% clamp); +15% WITHOUT = DIVERGES,
abort 32 s, **51% of samples on the 3.0 N.m joint clamp**, tilt 32 deg, CoM
error 1.49 m; +7.5% WITHOUT diverges (abort 15 s); **MATCHED kf WITHOUT is
1 PASS IN 3** (75 s, then abort 17 s, then abort 3.7 s — same config all three,
the 2026-08-10 run-to-run-scatter lesson; a single completed run proves
nothing, and reading the first one alone briefly shipped an unstable config).
**MECHANISM** (first 5 s without it): the arm leaves home at once — q2 spans
-10.8..50.0 deg, q3 -77.6..50.0, hits the +50 stop at t = 0.8 s and crosses
into NEGATIVE q2/q3, this asset's **elbow-singular branch**; J_3y degrades, the
DLS solve demands torque the servos cannot give, all four rail, and the
reaction takes the base. With the PID the same window keeps q2 31-43, q3 39-43.
**THE EE TASK CANNOT SUBSTITUTE AND STIFFENING IT IS WORSE**: at `K_y` = 2 a
200 mm task error is 0.4 N of restoring force, and that softening happened in
the SAME +15% campaign that added the PID — **the two are a compensating pair,
only ever validated together** (every `controller.py` config that flies with no
posture anchor uses `K_y` 8-200) — but undoing both, `K_y` 2->20 / `D_y` 4->9,
was the WORST of the seven runs, abort at 3.8 s / 36 deg tilt. A stiff task on
this laggy plant is its own instability. **TREE STATE: term restored, +15%
injection restored, both yamls carry a DO-NOT-DELETE note at `wb_dls_lambda`
and the sim yaml's `alloc_thrust_coeff` block records that the two must move
together.** Parity 4/4 and WbReferenceBuilderTest 2/2 both before and after, so
the C++/Python law agreement is unaffected either way. What removal actually
needs is a RETUNE with mismatch margin (M_r_d, GMO bandwidths, K_y/D_y, a
gentler DIRECT entry), not a one-line edit. Declared-deviation note for papers:
in steady flight the term is 0.004 of 0.77 N.m (0.5%) at the flown 0.11 deg arm
error, added straight to `u3` with NO null-space projection (there is none —
the 4-DOF task has isolated but non-unique solutions, `[0,40,40,0]` and
`[0,128.7,-108.4,0]` deg agreeing on the EE pose to 4.7e-16). Campaign driver +
per-run metrics: `docs/docs_aerial_manipulator/posture_removal_20260905/`;
tables: Command.md 7.14.6.

**L1 ADAPTIVE AUGMENTED DISTURBANCE OBSERVER — IMPLEMENTED, PARITY-LOCKED AND
SIM-VALIDATED (2026-09-06, user request).** The working note "Decompose the lumped
disturbances into end-effector and orthogonal components" (2026-08-27, in the manuscript
repo's `disturbance observer design/`) is implemented as an ALTERNATIVE ESTIMATOR for the
whole-body law and flown against the GMO on the same plant. Everything is NEW FILES; the
GMO path is byte-identical and `WbParityTest` passes 4/4 before and after.
- **What it replaces.** The GMO's `d_hat = K_o (p - p_hat)` closed on its own predictor
  IS `K_o/(s+K_o)` applied to the true residual, so ONE gain buys both estimate accuracy
  and loop robustness — which is why this rig's `K_o` is pinned at 0.5/0.1/0.1 (1.0 on the
  body channels crashed it). The L1 law splits them: a DEADBEAT piecewise-constant
  inversion whose accuracy is the sample period alone, then an explicit
  `C(s) = omega_c/(s+omega_c)` that alone decides robustness. It also ATTRIBUTES the task
  force: a momentum residual measures only `d + d_e`, so the GMO's lumped
  `(J_y^#)^T d_hat` hands `u3` the internal disturbance as a PHANTOM CONTACT FORCE, and
  the repair is the four directions no wrench can reach (`N(J_e)`, the arm self-motions,
  where `Z_0^T J_e^T = 0` exactly — no contact flag anywhere).
- **Where it lives.** `wb_l1_observer.{hpp,cpp}` in the whole-body fork + the Python
  reference `extensions/.../utils_controller/l1_observer.py` (run it for its self-test);
  selected by `wb_l1_observer_type` (default `"gmo"`, so every existing yaml is
  unaffected); second executable `autopilot_whole_body_l1_direct_actuation_node` over the
  SAME client — the law is identical, only the update rule changes, so a fork would have
  been a copy that could drift. **Shared ROS namespace on purpose**: the drone GS, the
  whole-body planner and the stop scripts need no changes, and the two rigs are mutually
  exclusive anyway; `vehicle_name` tells them apart. Pegasus launcher
  `scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_direct_actuation_sitl.sh`,
  stack `scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh`,
  yaml `..._whole_body_l1_..._t650_sim.yaml` = its GMO twin plus one block (DIFF THEM
  before every campaign). Every stale-node guard in both repos gained the new name. The
  comparison harness gained a third case `wb_l1`.
- **TWO PLACES THE NOTE LEAVES A CHOICE, both measured, both changed the implementation.**
  (a) **`Phi` is not the deadbeat gain of the predictor that can actually be run.** The
  `(h+u)` term must be plain Euler (the plant integrates it exactly; any other weight
  leaves a `(weight-dt)(h+u)` bias, ~0.15 N on z at hover), so the exact gain is
  `Phi_d = dt(I-e^{A_s Ts})(I-e^{A_s dt})^-1`, which equals the note's `Phi` in the
  fine-sub-step limit and `dt` at one-tick adaptation. Using `Phi` measures a REAL DC
  error of `a*Ts`: 0.8% at `A_s=2`, **6.9% at `A_s=20`**; with `Phi_d` it is 2.5e-13.
  **Corollary: at one-tick adaptation `A_s` drops out of the estimate entirely** — it only
  weights the average within an interval, so `wb_l1_a_*` and `wb_l1_adapt_period_s` are
  ONE trade, not two, and `A_s` is inert in noise-free simulation.
  (b) **The note's minimum-norm `L_c = B_a G^+` makes the phantom force WORSE on this
  plant.** The collective's coupling into the wrench-free rows is
  `a_3 = [0, -0.002, -0.103, -0.001]` against the joints' exact `I_4`, so minimum norm
  compares NEWTONS with NEWTON-METRES and books a 5.5 N thrust deficit as joint torque:
  measured 0.04 N of 5.5 found and the attributed `|F_hat_y|` **4.5x worse** than not
  decomposing (0.89 -> 4.00). Generalized to `L_c = B_a W G^T (G W G^T)^-1` with a
  diagonal prior variance — every property the note's proof uses survives for ANY `W > 0`
  (`Z_0^T L_c = I_4`, range still in `R(B_a)` so the lateral rows stay frozen, Lyapunov in
  the `W^-1` metric). `W = 1,1,1` is the note's own choice and the CODE DEFAULT; the yaml
  ships `(100, 0.25, 0.0025)` and finds 4.90 N of 5.5.
  (c) **The note's persistency-of-excitation remedy is useless here**, measured: a
  vigorous 60 s arm sweep gives an averaged projector with eigenvalues down to 4e-4, and
  **400 s of sweeping moved the collective estimate from -0.04 to -0.30 N** of a true
  -5.5. The metric in (b) is what actually fixes it; `omega_i` changes only how fast the
  fixed point is reached, not where (identical from 0.2 to 10 rad/s).
- **PARITY-LOCKED.** `generate_wb_l1_truth.py` -> `wb_l1_truth_t650.json` (5 gain sets x
  40 steps, covering N=1 and N=5 adaptation, unequal `A_s`, both `L_c` metrics, decompose
  on/off, and a set whose bounds clamp); `WbL1ParityTest` holds the C++ to **1e-8**, the
  same gate as the law's own. Also verified: `J_e Z_0 = 0` to 4e-17, `rank J_e = 6`
  everywhere, `J_y = S_e Jbar_e` to 3e-16, and `w_hat` IDENTICAL to 2.7e-15 with no
  contact wrench, a 2.6 N one and a 26 N one — the property that makes the scheme
  contact-flag-free.
- **FLOWN, 4 x 110 s DIRECT, none aborted** (`docs/docs_aerial_manipulator/
  l1_observer_20260906/`, harness `application/robotic_arm/utils/wb_l1_*`). The `_sim`
  plant carries all three factors the user asked about: **thrust loss** (+15% allocator
  kf), **model uncertainty** (mass and inertia x1.10, CoM shift 10/10/5 mm) and **motor
  delay** (rotor lag lambda = 10.0265 1/s) — together 10.8 N the observer must find, and
  `u1` = 47.559 N / `d_hat_z` = -10.809 N are identical to 3 decimals across runs, which
  is the check that the plant really was the same in each.

  | omega_c | rise90 | entry sag | recovery | steady CoM | \|e_R\| | tilt pp | **phantom F_y** |
  |---|---|---|---|---|---|---|---|
  | GMO K_o .5/.1/.1 | 2.86 s | 463 mm | 49.3 s | 4.59 mm | .0885 | 0.108 | **1.884 N** |
  | L1, = K_o | 3.28 s | 428 mm | 49.2 s | 3.17 mm | .0934 | 0.337 | **0.065 N** |
  | **L1, 2/.5/.5** | 1.38 s | 187 mm | 21.0 s | **1.65 mm** | .0059 | 0.564 | **0.055 N** |
  | L1, 6/2/1 | 0.73 s | 69 mm | never | 52.0 mm | .2615 | 22.24 | 2.045 N |

  **At matched bandwidth the two are indistinguishable and must be** (both reach `f_d`
  and `u_2` through the same first-order filter) — that is the sanity check. **The
  phantom force is a separate axis and collapses 29x at zero cost**, and three numbers
  show it is the mechanism: `w_hat_thrust` = -10.753 of -10.81 (the null channel booked
  **99.5%** of the residual to the COLLECTIVE), `w_e_hat` = 0.074 N (correct — nothing is
  touching the arm), and `sigma(d_hat^c)` 0.657 N vs `sigma(d_hat_f)` 0.0021 N (the
  deadbeat estimate really is a noisy 250 Hz momentum difference and the filter really
  does remove it). **omega_c = 2/0.5/0.5 is the shipped winner**, 4-5x the GMO's ceiling,
  better on every metric with zero saturation. **6/2/1 is past the limit** and fails as
  predicted: best ENTRY of the four, then a 22 deg p-p limit cycle with `d_hat_z` swinging
  -5..-20 N onto its own bound and 10.4% of samples on the joint clamp — 6 rad/s sits on
  the 10.03 rad/s rotor-lag pole.
- **NOT covered, state it rather than imply it:** no contact in any flight (so the TRUE
  wrench estimate is untested — `w_e_hat` ~ 0 when the answer is 0 is necessary, not
  sufficient); no arm motion; `sim_arm_backemf_enable` false, so **`lc_var_q = 0.0025` is
  the one shipped number the campaign did not test** and it must be RAISED once the droop
  is on; ONE flight per configuration, which near a stability boundary on this rig proves
  nothing (the 2026-08-10 scatter lesson); and the 6/2/1 run raised all three channel
  groups together, so which one binds is unresolved. NOT flown on hardware.
- **THE RIG'S STANDARD TEST ALSO FLIES (2026-09-06, user request): 1 m hover, x/y/yaw
  steps, compatible trajectory.** Five attempts, four completed; run E flew every leg.
  Steps x/y ±0.5 m peak 203-268 mm and settle 50-63 mm; yaw ±30 deg peaks 30/9.4 mm and
  settles 6.7/0.9 mm; **the COMPATIBLE-TRAJECTORY leg (EE −6 cm) is the best-tracked
  motion in the mission — 9.8 mm peak, 0.74 mm settled, 0.17 deg tilt** while the arm
  moves and the base counter-moves, which is the expected ordering because it is the only
  leg whose reference is dynamically consistent by construction. Zero saturation,
  `u1`/`d_hat_z`/`w_hat_thrust` 47.57/−10.814/−10.738 N across runs C/D/E to 3 decimals.
  The ~55 mm settled translation residual is the structural `T*e_R/K_p` offset, not a
  tune. **One attempt in five aborted 8.4 s after DIRECT entry** (growing lateral
  oscillation, arm on the 3.0 N.m clamp 21.4% of samples) — runs B-E flew the same 1 m,
  so it is the run-to-run scatter 7.14.6 records, NOT the observer or the altitude.
- **A STEP IS NOT A REFERENCE PUBLISH in whole-body DIRECT**, and two traps in driving the
  handshake both produce a plausible log with the steps silently not happening (each cost
  a flight): (a) the planner's unchanged-target guard is inactive while it is HOLDing, so
  a full-rate `position_controller/reference` stream drags it back out of HOLD every tick
  — publish on CHANGE in DIRECT; (b) its status carries suffixes (`PLANNED T=5.2s`,
  `INFEASIBLE: <reason>`), so a state check must take the FIRST TOKEN, splitting on
  whitespace AND ':' — stripping only the colon never matches `PLANNED` and times every
  leg out while the planner reports a good plan. Also require `EXECUTING` before `HOLD`
  counts as completion, since HOLD is where a leg starts.
- **THE EE WORKSPACE AT THE FOLDED HOME IS SMALL AND LOPSIDED, and both obvious step
  directions are outside it** (measured, each refused in ~0.1 s with a readable reason):
  reaching OUT 0.15 m gives `IK did not converge` — the gripper already sits 0.26 m from
  the body origin, its own reach (0.16 m wrist at beta = 80 deg + the 0.108 m gripper
  offset); retracting IN 0.05 m gives `joint limits: q3 = 63.0 vs 50` because retracting
  FOLDS it further and q3 is already at 40. **DOWN is the direction with room** (it
  unfolds): 0.06 m down solves to q = [0, 27.1, 38.8, 0] deg and flies. Map it with
  `transition_planner.ik_position_azimuth` before guessing; the planner publishes the
  true reachable set on `whole_body_planner/workspace_rz`.
- Traps found while building the harness: **`vehicle_status` never publishes on this PX4
  v1.16 / px4_msgs release/1.16 pairing — it is `vehicle_status_v1`**, and subscribing to
  the un-suffixed name strands a driver in its WAIT phase with no error at all; the
  system `python3` has numpy 2.2.6 in `~/.local` against an apt matplotlib built for
  numpy 1.x, so plotting needs `PYTHONNOUSERSITE=1`; and **`pkill -f "[w]b_..."` still
  kills your own shell if the SAME command line later mentions the target in plain text**
  — the bracket trick protects the pattern, not the rest of the line.
  Full write-up: `docs/docs_aerial_manipulator/L1 Augmented Disturbance Observer.md`;
  commands: Command.md 7.15.

**THE L1 OBSERVER'S FOLLOW-UP CAMPAIGN — LAW AUDIT, POSTURE, BACK-EMF
(2026-09-06, user request).** Three questions, 12 flights, data
`docs/docs_aerial_manipulator/l1_final_20260906/`, tables Command.md 7.15.7,
write-up `L1 Augmented Disturbance Observer.md` 6.
- **THE LAW CARRIES EXACTLY ONE TERM THE MANUSCRIPT DOES NOT**, audited line by
  line: the clamped joint-space PID on `u3`. The DLS regularisation and the
  saturation-consistent rebuild are numerics and actuator bookkeeping, not force
  terms. The node now PRINTS which way each run is configured (`LAW CHECK` in
  the controller pane, green when the four `wb_posture_*` are 0) — a silent
  pass is indistinguishable from a check that stopped matching.
- **REMOVING IT ON THE L1 PATH FAILS TOO: 2 aborts of 2** (7.7 s / 8.0 s,
  32-47% of samples on the 3.0 N.m joint clamp), so the L1's attribution does
  NOT make the posture anchor unnecessary — and WHY is the useful part.
  **The two observers fail for different reasons and only one is an observer
  problem.** The GMO's is attribution (lumped residual arrives in `u3` as a
  phantom contact force, measured 1.35-1.89 N; the task yields). The L1 fixes
  exactly that — phantom force **0.044-0.132 N, 13-40x smaller** — and still
  cannot select an IK BRANCH, because nothing in a task-space law can: the
  4-DOF EE task has isolated but non-unique solutions. Under the entry
  transient q3 hits its +50 stop at t = 1 s then crosses into NEGATIVE q3, the
  elbow-singular branch, q1 rails at -35, `J_3y` degrades and the DLS solve
  demands torque the servos cannot give. **Keep it, declare it** (0.5% of `u3`
  in steady flight); the real fix is an explicit branch guard, not a gain.
- **THE ARM'S BACK-EMF DROOP DEFEATS BOTH OBSERVERS, 4 runs of 4.** With
  `sim_arm_backemf_enable: true` (`b = [0, 0.9337, 1.4934, 0]` N.m per rad/s)
  the GMO aborts at 9.7/10.8 s and the L1 at 5.8/5.8 s; with it off both fly
  the full 8-leg mission twice each. Repeats agree to a few percent — not
  scatter. **Neither should be expected to compensate it**: (a) `-b*qdot` is a
  GAIN error, a feedback path, and `b/I ~ 75 1/s` against the observers' arm
  channel at 0.5 (L1) / 0.1 (GMO) rad/s is 150-750x too fast; (b) the damage
  lands on the BASE, because `tau = T^T u` makes the rotors pre-compensate the
  arm reaction `N1^T u3` that never arrives; (c) it closes a POSITIVE LOOP
  around the (normal, large) DIRECT-entry transient — measured droop 0.09 N.m
  at t = 1 s with tilt already 5.5 deg, then **1.70 N.m** by t = 7 s. The two
  fail differently, which is diagnostic: the GMO takes 10 s and RAILS the clamp
  (4.9-9.2%) before drifting 2.1-2.4 m; the L1 takes 5.8 s, reaches a HIGHER
  tilt (31-32 deg) and never touches the clamp — its faster arm channel
  responds sooner and drives the base harder. NOT concluded: that the law
  cannot fly a PWM-mode arm. Untried: a gentler DIRECT entry, an explicit
  `+b*qdot` arm feedforward (one line, `b` identified to +-5%), and on hardware
  the real fix, current-control Mode 0, where `b -> 0` and this model should be
  DELETED not compensated.
- **COMPARISON, same plant/mission/driver, ideal arm, 16 s hold per leg**
  (figure `compare_gmo_l1.png`, scorer `utils/wb_compare_metrics.py`, CoM error
  mm as peak/rms/settled): step x +-0.5 m GMO 296/96/6.8 and 251/80/5.0 vs L1
  **193/61/2.6** and **209/66/3.0**; step y GMO 267/86/2.5, 268/85/4.9 vs L1
  238/76/5.1, 222/70/4.9; yaw +-30 deg indistinguishable (<10 mm either way);
  **compatible trajectory GMO 64/39/14.7 and 57/33/18.9 vs L1 47/18/2.6 and
  35/13/1.4** — 2.4-2.9x rms and 6-13x settled, the expected ordering because
  it is the leg where arm and base move together and a LUMPED estimate is most
  wrong. **The most reproducible difference is the DIRECT-ENTRY transient**
  (3 runs each): peak 1273/1273/1305 mm recovering to <50 mm in 37.9/38.6/39.3 s
  vs **796/800/854 mm in 7.1/7.6/7.7 s** — 1.6x peak, **5x recovery**, the
  `omega_c`-vs-`K_o` bandwidth split doing what it exists for.
- **A HOLD-LENGTH TRAP THAT INVERTS THE VERDICT, and it cost a pair of runs.**
  At the driver's default 6 s hold the L1 scored 65-72 mm "settled" against the
  GMO's 6-21 mm, i.e. WORSE. Artifact: this rig has a slow, lightly damped
  position mode, so at 6 s neither has settled and the 2 s average lands at an
  arbitrary phase. At 16 s the ordering inverts. **Never compare settled errors
  across runs with different `--hold-between`.**
- **THE COMPATIBLE-TRAJECTORY LEG NOW MOVES ALL FOUR JOINTS** (user request).
  Down alone was a pure fold — q1 = q4 = 0 throughout, so the arm-yaw and wrist
  channels were never exercised. Now lateral 0.08 m + down 0.07 m + 60 deg EE
  heading (`--ee-lat` / `--ee-yaw`), mapped offline with
  `transition_planner.ik_position_azimuth` BEFORE flying: q = [17.6, 31.0, 29.8,
  61.9] deg, every joint moving >=9 deg, sigma_nd 0.297 vs the 0.10 keep-out.
  **THE LATERAL AXIS IS WORLD Y, NOT X** — measured, one refused leg: the frame
  adapter's `R0_model = R0_actual*Rz(-90)` puts model +y (ALONG the arm,
  radially OUT) on world +x, so a world-X lateral step is a reach outward and
  the planner refuses it in 0.13 s with "IK did not converge". Confirmed live by
  the driver's own mission anchor print (base [0.26, 0.203, 0.982], EE [0.517,
  0.204, 0.888] — the arm on world +x at yaw 0). The world-Y SIGN does not
  matter: the workspace is symmetric across the arm (sigma_nd 0.295 vs 0.297)
  and 60 deg of heading moves q4 a lot either way.
- **The joint_state_broadcaster's order on this rig is `[q2, q3, q1, q4]`**, not
  joint1..4 — at the ground pose it reads (40, 40, 0, 0) for the home
  [0, 40, 40, 0] deg, and in flight column 2 pins at exactly -35 (q1's stop)
  while column 3 reaches 87 (only q4 has that range). Mislabelling it makes an
  elbow excursion look like a wrist one.

**WHOLE-BODY L1 TUNING — TWO PARAMETERS, TWO USER-REPORTED SYMPTOMS FIXED
(2026-09-06).** 16 flights, data `docs/docs_aerial_manipulator/l1_tune_20260906/`,
tables Command.md 7.15.8. All three disturbance sources stayed ACTIVE (+15% kf,
mass/inertia x1.10 + 10/10/5 mm CoM, rotor lag); back-EMF off per the request.
Changed: `ude_height_threshold` 0.4 -> **0.35**, `wb_ky_psi`/`wb_dy_psi` 1.0 ->
**0.3**. Nothing else.
- **THE SLOW SAFETY TAKEOFF IS A UDE GATE SITTING ABOVE THE RESTING HEIGHT.**
  `sim_plant_mass_scale 1.10` makes the plant 4.120787 kg while the controller
  keeps `vehicle_mass 3.746170` — the gravity feedforward is **3.67 N short**,
  only the UDE can supply a constant force error, the gate disabled it below
  0.40 m, and the vehicle **RESTS at 0.305 m**. Measured: creeps at 0.003 m/s
  for **22 s**, crosses 0.40, then climbs normally. The bare-T650 deadlock
  (7.13.3 run C) in slow motion; the yaml comment already warned about it, the
  value had just never been checked against this rig's resting height. Fixed:
  0.95 m in **9.2/9.6 s vs 28.2 s**. NOT fixed by correcting the mass — that
  deletes the disturbance under test.
  **THE OBVIOUS VALUE IS UNSAFE AND ONE RUN WOULD HAVE SHIPPED IT.** 0.32
  flies, but across **33 recorded runs the seated peak reaches 0.3338 m** (the
  arming transient), so 0.32 is crossed while still on the ground in **29 of
  33** and the UDE would integrate the ground reaction. 0.35 clears it by
  16 mm, 0 of 33. The gate is on MEASURED altitude and dropping below HOLDS the
  integral rather than resetting it (`ude_base.cpp:47`), so a bounce costs
  nothing — only the seated case matters.
- **THE J1/J4 TORQUE RIPPLE IS THE EE HEADING CHANNEL'S LOOP GAIN.** q1 and q4
  carry ~0 MEAN torque (q2/q3 hold gravity) but ALL the ripple, ~70% of it in a
  **3.90 Hz** line; the EE heading task row oscillates at exactly that
  frequency and heading IS the (q1,q4) pair. **On the FULL mission the shipped
  gains drove q4 across its entire +-3.0 N.m clamp (6.00 N.m p-p)** — invisible
  in a hover-only test, because only the trajectory leg's 60 deg heading sweep
  exercises the channel. At 0.3: **0.062 N.m p-p (96x), peak arm torque
  3.00 -> 0.85, 0.0% clamped**, and heading tracking IMPROVED (e_psi std
  1.17e-3 -> 5.6e-4 rad) — a self-excited oscillation, not useful stiffness.
  **THE RIPPLE IS L1-SPECIFIC AND THE GMO YAML DOES NOT CARRY THIS CHANGE** —
  measured both ways: the GMO's own q4 ripple is 0.0042 N.m std at `ky/dy_psi`
  1.0 against the L1's 0.4833, and 0.3 moves it to 0.0046, i.e. nothing. What
  differs is what feeds that task row: the GMO's `F_hat_y` is the smooth lumped
  `(J_y^#)^T d_hat` off a heavily filtered estimate, the L1's is the attributed
  `Lam_y S_e Lam_e^-1 w_hat_e` rebuilt every tick from a DEADBEAT one, and the
  heading row is the least-filtered path it takes. **`ude_height_threshold`
  0.35 IS mirrored into the GMO yaml** — it helps both (GMO takeoff
  29.7 -> 15.5 s), since the mass mismatch and resting height are shared.
  **NOT an anomalous `M_Y_psi`**: reading 0.05 against x/y/z's 1.0 as a 20x
  amplification is a UNITS ERROR (kg.m^2 vs kg); the real per-row amplification
  is `Lam_y*M_y^-1 = [0.485, 1.320, 0.778, 0.404]` and the heading row is
  unremarkable. Four routes all work (`my_psi` 0.3 or 1.0, `dls_lambda` 0.6,
  `ky/dy_psi` 0.3); the last measured best on the trajectory legs and tilt.
- **OBSERVER BANDWIDTH IS NOT THE LEVER, AND IT IS THE ROTATIONAL CHANNEL THAT
  BINDS.** The earlier sweep moved all three `omega_c` groups at once and left
  "which one binds" open. One group at a time: rotational 0.5 -> 1.5 degrades
  badly (tail 2.7 -> 279 mm, tilt 34 deg, 4.5% clamped) and 0.5 -> 3.0 **aborts
  in 12 s**; translational 2 -> 4 and arm 0.5 -> 1.5 change essentially
  nothing. So 0.5 is already the rotational ceiling, and **the DIRECT-entry
  transient is not observer-limited** — no `omega_c` will fix it. It is the
  SAFETY->DIRECT handover (SAFETY's UDE has the ~10.8 N, DIRECT's observer
  restarts at zero); closing it means SEEDING the L1 from the UDE at the mode
  switch, which is a code change, not a gain.
- Tooling: `utils/wb_tune_score.py` (entry transient + dominant-mode amplitude
  and FREQUENCY — a tune that moves the frequency changed which element
  dominates, one that moves only the amplitude did not);
  `wb_l1_set_gains.py` now also takes LAW gains by their full `wb_` name.

**WHOLE-BODY WHOLE-BODY PLANNER — PAPER-FAITHFUL DIRECT COMMANDING (2026-08-23, user
design).** In whole-body DIRECT the law now always receives the paper's FULL reference set
(CoM chain through snap, base heading, EE position+heading chains, consistent q_d) instead
of GS steps, streamed as the NEW `fsc_autopilot_ros2_msgs/WholeBodyReference` (MODEL-frame
by contract; the msgs CMake GLOBs so the message was a pure file-add). Everything is
ADDITIVE — no original file's behaviour changed:
- **Pegasus side (this repo): `utils_planner/transition_planner.py`** — compatible
  SETPOINT-TO-SETPOINT transitions. Every task channel A→B on ONE shared MIN-SNAP phase
  (min-snap REQUIRED, not min-jerk: the solved CoM's VELOCITY depends on the prescribed
  jerk through the thrust-direction map, so nonzero endpoint jerk would step the CoM
  velocity at the hold↔transition joins), CoM solved by the same Picard fixed point as
  `compatible_trajectory.py` (imported, untouched), with a time-varying β split and the
  rest-point identity (α,β,γ) = (φ+q1, q2+q3, q4) making both endpoints ALGEBRAICALLY
  exact on the holds. Ships `make_params_t650()` (generate_wb_truth's correct model-frame
  body override), a damped-Newton 4-DOF IK (position + heading azimuth, FD Jacobian on the
  exact chain, multi-seed) and `rest_ref()` for static holds. Offline-validated (system
  python3, run the file): defect 4.4e-8 m, hold handover 0.028 mm / ~1e-5 m/s, FD-consistency
  5e-11, IK round-trip 1e-14 rad, σ_nd ≥ 0.51 on the test transit, wrist-singular and
  out-of-range goals refused with operator-readable reasons.
- **2026-09-14: the whole-body planner is now a C++ (rclcpp) node in its OWN
  ROS 2 package, `fsc_trajectory_planner`** (`~/Workspaces/fsc_autopilot_ws/src/
  fsc_trajectory_planner`, node `whole_body_trajectory_planner`, namespaced per
  vehicle by the launch file's `uav_prefix`). It links the flight stack's
  exported `fsc_autopilot_ros2::wb_law` for the whole-body model and the flat
  B-spline planner, and carries its own C++ IK and straight-line Picard
  transition planner — **it does not import this repo's `utils_planner`**,
  so the `pegasus_root` / `FSC_PEGASUS_ROOT` coupling described in the next
  bullet is gone from every stack script. Interface unchanged (same
  `whole_body_planner/*` topics/services, same states), so the arm GS tab, the
  Isaac `viz_path`/`viz_pose` drawing in 06 and `wb_l1_campaign_driver.py` work
  as before. Plans in ~3-4 ms (was 45-260 ms). Parity against
  `transition_planner.py` / `flat_bspline_planner.py` is locked by gtests with
  fixtures generated from THIS repo's Python (`scripts/dump_python_fixtures.py`
  there; `dump_flat_reference.py` here), which is now the only remaining
  relation between the two: a change to `utils_planner`'s maths must be
  mirrored there or the fixtures diverge. The vehicle model and the trajectory
  backends sit behind registries (`vehicleFactories()`, `plannerFactories()`)
  so a second airframe or a figure-8/circle shape is one factory each. Full
  §7.15.1 sim flight with it: Command.md §7.15.11. The Python
  `whole_body_planner.py` and its `planner/` directory were DELETED from the
  autopilot repo the same day (its rig tests moved to the new package). This
  repo's `utils_planner` stays: the 02/03 demos, comparison/plot tools,
  `l1_observer.py`'s self-test and the C++ truth generators
  (`generate_wb_truth.py`, `generate_wb_l1_truth.py`, `dump_flat_reference.py`)
  import it, and no flight-stack process does. The bullet below describes
  the Python node as it was.
- **2026-09-15: END-EFFECTOR TRAJECTORY MODE in fsc_trajectory_planner.** Circle /
  figure-8 EE trajectories (yaw along the tangent) planned as compatible whole-body
  runs following `~/Downloads/Task-space Planner`'s `main_redundant_zyxx.m` adapted
  to the z-x-x-z OM-X (q1 fixed at 0, q2 an assigned sinusoid, drone yaw = the first
  z angle; the fmincon feasibility residual solved as a relaxed Picard fixed point).
  Note the MATLAB script's `Ref_TrajGen.m` sweeps ALL THREE EE Euler angles linearly
  and does not align yaw to the tangent — tangent yaw with zero extra roll/pitch is
  the user's specification, and a literally level EE is this arm's wrist singularity
  (refused). Flown from the arm GS's new "EE trajectory" tab (Sine Test and Demos
  tabs removed). Sim record: Command.md §7.15.12.
- **fsc_autopilot_ros2 (dev_CCM): the whole-body planner**
  (`.../single_aerial_manipulator_whole_body_direct_actuation/planner/
  whole_body_planner.py`, plain rclpy script, no build; own tmux WINDOW
  `whole-body planner` in the stack launcher — SIX PANES is that row's ceiling, a 7th split fails
  with `no space for new pane` in the 80-col detached window and `set -e` then kills the
  launcher before the vrc pane/titles/attach; imports the planner via `pegasus_root` param /
  `FSC_PEGASUS_ROOT`). State machine: silent in SAFETY (takeoff byte-identical) → on
  DIRECT (from the node's latched `/mode` topic) captures the rest hold from odom + the
  smoothed arm reference and streams at 100 Hz → drone-GS reference = PENDING base (never
  executed; republished latched on `whole_body_planner/pending_base`, auto ride-along
  plan) → arm-GS EE target (`whole_body_planner/ee_target`, world PoseStamped) = replan
  (worker thread, `_plan_gen` supersedence; status latched on `whole_body_planner/status`:
  HOLD/PENDING/CALCULATING/PLANNED/INFEASIBLE:<reason>/EXECUTING) → Trigger
  `whole_body_planner/send` executes; completion re-holds at the goal and one-shot-seats
  the arm smoother there. While EXECUTING it also streams q_d to the
  ExternalTorqueController's target_joint_setpoint (20 Hz) so a SAFETY abort lands on the
  CURRENT pose. Frame boundary mirrors frame_adapter: φ_model = ψ_actual − π/2,
  R0_model = R0_actual·Rz(−90). Uses threading.RLock — the plain Lock self-deadlocked when
  the infeasible branch published status under the lock (found in loopback). Loopback-
  validated end-to-end (fake mode/odom/arm rig): 100 Hz hold with exact home FK (x_cd
  shows the known 19.5 mm forward CoM), PENDING→PLANNED (ride-along T=5.4 s),
  EE replan, Send → 552 samples, max inter-sample EE step 2.46 mm, terminal EE exactly on
  target, new hold = IK pose, SAFETY revert silences instantly.
- **WB node hook (C++, the fork):** new optional params `wb_streamed_ref_timeout_s`
  (0.25 s) / `wb_streamed_ref_topic`; a fresh streamed reference IS the law's reference
  verbatim (copied, no frame math — the message is model-frame), stale → the internal
  builder (hover-campaign behaviour preserved; existing yamls run unchanged). Malformed
  samples (non-finite / degenerate headings) are dropped, edges logged; debug[56] =
  stream-fresh. Parity suite still 4/4 — the law itself untouched.
- **Arm GS: NEW second tab "EE Whole-Body"** (`custom_gui/src/ee_wholebody_panel.{hpp,cpp}`,
  reusing tab 1's WorkspaceView/PlanView untouched + a NEW third HeadingDial for the
  inertial EE heading with the pending-base heading as a rim tick). Inertial world X/Y/Z +
  heading boxes; visualization base-relative about the PENDING base; lamp orange
  Calculating → green Planned / red Infeasible+reason; Assign(plan)/Send(execute)/Clear.
  Guidance verdict from tab 1's raster only — the whole-body planner is authoritative (the raster is
  mount-plate-anchored, the whole-body planner body-origin-anchored: cm-level display bias near the
  boundary, stated in the tooltip). Screenshot-validated on the live pipe (frame math
  confirmed: base (0.5,0.2,1.4,15°) + pick → world boxes exact). One trap: this Qt style's
  QPalette::Mid is near-white — the dial uses hard-coded grays.
  In whole-body DIRECT tab 1's joint/EE commands are superseded (the law ignores the
  smoothed joint path while the stream is fresh); tab 2 is the EE surface.
Full operator flow + debug layout: Command.md §7.14.3.
**FIRST WHOLE-BODY DIRECT FLIGHT, 2026-08-23 — two GROUND-STATION wiring bugs found and
fixed (neither in the law, the whole-body planner or the planner; both were topic plumbing):**
(1) **Drone GS N/T rotor pies stuck at 0.0%** through the whole DIRECT flight. Its
motors_debug subscription list is PER-FORK and held only `direct_actuation` +
`geometric_direct_actuation`; this fork publishes `whole_body_direct_actuation/motors_debug`.
Diagnosed in one command — `ros2 topic info` on that topic showed `Publisher count: 1,
Subscription count: 0`. Fixed by adding the name to that list in
`ros2_ground_station_gui/src/ROS_Node/ros_single_drone_control.py` (a shared repo on main —
the change is one line into an existing extension-point list, left UNCOMMITTED). Both of
the GUI's other gates were already fine: the controller-name test is a SUBSTRING match on
"Direct Actuation" (so "Whole-Body Direct Actuation" passes), and `connected` was true.
NOTE the §7.14 text claiming the `connected` gate had been removed was WRONG — it is still
there (it is really PX4 `pre_flight_checks_pass`); Command.md now says so.
(2) **Arm GS "EE Whole-Body" tab dead** — lamp stuck on "Not in whole-body DIRECT",
"waiting for the whole-body planner", buttons greyed, while the whole-body planner was actually at
`PLANNED T=10.3s`. NAMESPACE MISMATCH: the arm station runs at
`/uav_0/fsc_open_manipulator` (owner-prefix convention) but the whole-body planner is a FLIGHT-STACK
node at `/uav_0`, so the panel's plain relative names resolved to
`/uav_0/fsc_open_manipulator/whole_body_planner/*` and matched nothing (both sets of
topics existed side by side — the giveaway in `ros2 topic list`). The panel now resolves
against the vehicle namespace by stripping its own last namespace component (root and
single-component namespaces handled; `planner_namespace` parameter overrides).
Verified isolated from the flying vehicle under `/uav_test/fsc_open_manipulator` →
subscribes `/uav_test/whole_body_planner/*`, and screenshot-confirmed the panel then
activates (green PLANNED, base target populated, all three buttons enabled).
BOTH FIXES NEED A GROUND-STATION RESTART to take effect.
**Arm-GS panel then RESTRUCTURED to the user's spec (same day)**: joint-limit grid
(Setpoints styling, green/red) fed by a new whole-body planner topic
`whole_body_planner/target_joints` — published on EVERY plan attempt including
infeasible ones, since that is exactly when the row earns its place, and defaulting to
the hold pose · read-only drone-target row (amber while pending) · the three selectors ·
the EE target as TWO editable rows, relative-to-drone and inertial, each rewriting the
other through the anchor (heading stored inertially so it does not swing with base yaw) ·
buttons `Trajectory Planning` / `Send Compatible Trajectory` / `Clear` · whole-body planner status
strip moved to the BOTTOM. Screenshot-verified against a fake whole-body planner at /uav_test.
Full layout: Command.md §7.14.3 step 3.
**EE = THE GRIPPER, and the tab now draws the RIGHT workspace (2026-08-23,
user's definition + "most setpoints are infeasible" report).** Two findings,
both measured:
(a) `make_params()` deliberately ends the chain at the WRIST
(`ee_pos = joint_pos[3]  # EE = the wrist (manip_joint4)`, l_i[3] = 0). The
task variable y = [r_e; b_1e] is meant to be the GRIPPER, so the T650
whole-body variant now extends l_i[3] by the measured pad midpoint
`GRIPPER_OFF_WRIST = [0, 0, -0.0494]` (= 01_track's `PAD_OFF_EE`, the number
that demo adds by hand BECAUSE the model's EE is the wrist). Applied ONLY in
`transition_planner.make_params_t650()` + the C++ `t650Defaults`, never in the
shared `make_params()` — the legacy demos are flight-validated on the wrist
convention and would double-count. The offset is ~COAXIAL with joint 4 (the
asset's gripper CoM is 39.6 mm along that axis, 5.7 mm off it), which is what
preserves the 4-DOF split: q4 still cannot move r_e (verified identical r_0e at
q4 = 0/90), so position stays a (q1,q2,q3) problem and q4 buys the heading,
exactly as the z-x-z recovery assumes. Only l_i[3] moves — com_i[3] is measured
from O_chain[3] = manip_joint4, so no mass property changes. Fixture
regenerated, **WbParityTest 4/4**, planner self-test passes (defect 4.8e-8 m,
sigma_nd 0.404), whole-body planner loopback passes (hold r_ed 0.1465 -> 0.1952).
`generate_wb_truth.py`'s duplicate `make_params_t650` was DELETED — it now
imports the extension's, so fixture and whole-body planner cannot describe different
robots. **K_y/D_y were tuned against the wrist EE and have not been re-flown.**
(b) The tab was drawing the Setpoints raster, which is wrong here twice over:
that model has a PITCH wrist with a 126 mm bracket while the asset has the
coaxial ROLL wrist (~10 cm out), and it ignores the fold guard — of the region
it drew, only **2.6%** could ever be planned, which is exactly the "most
setpoints are infeasible" the user hit. The whole-body planner now publishes the true
usable set (`whole_body_planner/workspace_rz`, filled 192x192 (r,z) occupancy
grid, latched, one (q2,q3) sweep: r/z are EXACTLY q1-independent because the
chain is left-multiplied by Rz(q1)) and the panel draws that. Measured
afterwards: **0/40 points outside it plan** (a correct necessary bound), ~55%
of points inside plan at a fixed heading — the remainder fail on the WRIST
ROLL q4 needing 92-99 deg against its ±90 limit, which the joint row shows in
red. The guidance line says exactly this.
**RE-PLAN OSCILLATION — THREE COMPOUNDING DEFECTS, all fixed 2026-08-23** (user: "if I
click trajectory planning then send, the aerial manipulator will oscillate"). The
reference itself was never at fault — both plans are smooth offline (peak 17 deg/s, no
overshoot, defect 1e-8). The chain was:
1. **The whole-body planner's Picard solve holds the GIL for ~0.26 s in ONE block**, against the
   node's `wb_streamed_ref_timeout_s` of **0.25 s** — so the streamed reference went
   stale during EVERY solve. Fixed with a `yield_hook`/`yield_every` in
   `plan_transition`, called every 32 grid samples in BOTH the Picard and diagnostics
   loops (once per ITERATION is not enough — one iteration alone is ~26 ms of solid
   GIL). Measured after: the 100 Hz stream survives a re-plan at **101 Hz, worst gap
   14 ms**.
2. **On staleness the node fell back to its internal builder**, whose reference tracks
   `outer_ref_` — the RAW ground-station setpoint, a METRE away once the operator had
   sent a new drone target. Every stall therefore flicked the law onto a distant
   reference and back: the lurch. Now a `streamed_ref_seen_` latch makes the builder
   fallback available ONLY before the whole-body planner's first sample of an engagement (cleared
   in `switchMode`); after that a stale stream FREEZES the last streamed setpoint with
   all derivatives zeroed. That is also the *correct* reference during a solve, since
   the whole-body planner streams a static hold while it plans.
3. **Every repeated ground-station message restarted the solve** — a second click or a
   periodic republish flickered CALCULATING <-> PLANNED so a stable PLANNED could not be
   caught (this is the "Status automatically updates another trajectory" the user saw),
   and it fired defect 1 over and over. Both target callbacks now IGNORE an unchanged
   target (1 mm / 2 mrad).
Also fixed while chasing it: the planning worker caught only `ValueError`/`LinAlgError`,
so any other error killed the thread silently and left the GUI on an orange
"Calculating…" for ever — it now catches everything, reports INFEASIBLE with the message
and logs a traceback. Regression: `planner/test_replan_stream.py` (stream rate
+ worst gap across a re-plan, setpoint immobility while solving, and the repeated-target
case).
**FLOWN END-TO-END 2026-08-23 (§7.14.1, three matched runs): the three reference defects
ARE fixed, and the SWING IS A SEPARATE, PRE-EXISTING CONTROL PROBLEM.** In every run the
streamed reference was static to **0.0 mm** with `wb_control_debug[56] = 1` throughout and
ONE plan event (no storm) — yet the vehicle still limit-cycles at **0.88 Hz**, exactly the
attitude loop's `sqrt(k_R/I) = sqrt(2.0/0.065) = 5.55 rad/s`, with the ARM swinging ±10°
against a CONSTANT `q_d`. Closed-loop, not commanded. A/B/C/D: gripper EE + the shipped +15%
kf = bounded ±9° cycle; gripper EE + the CALIBRATED kf = DIVERGED (roll ±176°, watchdog →
SAFETY in ~30 s); WRIST EE + calibrated kf = worse still (flew 34 m at 18 m/s); and **run D,
the whole-body planner KILLED so the node runs its own internal builder (`Publisher count: 0` on the
reference topic), diverges IDENTICALLY** — pitch ±0.9° at t=8-16 s, ±2.1 at 16-20, ±8.6 at
20-26, flipped by 26: a growing unstable mode with the whole-body planner not in the loop at all.
That is what rules the reference path out; without run D it could not have been. So the
`r_e` = gripper change is NOT the trigger, and **the +15% "stress injection" is the only
reason this configuration flies** — a higher believed kf makes the allocator command ~13%
LESS motor per unit torque, i.e. a quiet loop-gain reduction. The yaml is LEFT at
5.38192065e-05 with that recorded in it; §7.14.2's reading of that number as robustness
margin is misleading.
**FIXED the same day — it was `M_r_d`, not k_R/k_w.** `wb_mrd_x/y/z` was the T650 **I0**
diagonal `[0.077681, 0.090738, 0.083401]`, the BODY-ONLY inertia, while the COUPLED
rotational inertia with the arm is `~[0.132, 0.112, 0.116]` (straight off `dynamics()`'s
M). `M_r_d` is the inertia the law IMPOSES, so the old value asked the vehicle to rotate as
if it were half its real weight — a ~2× bandwidth demand the 99.7 ms rotor lag + DDS/HIL
transport delay cannot honour, and the +15% kf injection was the only thing holding it up.
**Fix: `M_r_d` → 1.5× I0 = `[0.116522, 0.136107, 0.125102]`, `alloc_thrust_coeff` back to
the CALIBRATED 4.679931e-05, injection REMOVED.** k_R/k_w/K_y/D_y/GMO/model untouched.
Screened offline with the NEW `application/robotic_arm/utils/wb_hover_stability.py` (exact
law + exact model + allocator with the believed-vs-true kf split + rotor lag + a
transport-delay FIFO, ~5 s/candidate) scored on **DELAY MARGIN**, which is what binds:
1.0× I0 = 24 ms, 1.25× = 28, **1.5–2.0× = 32**, 2.5× = 28 (turns over). k_R barely moves it
and raising k_w makes it WORSE; K_y/D_y do not affect it. **That tool is NOT validated in
absolute terms** — it calls the flown +15% run unstable when the flight held it — so trust
only the RELATIVE ordering and fly the winner.
FLOWN (matched kf, whole-body planner streaming): pitch limit cycle **±9.2° → 0.14° p-p**, CoM error
**18 → 2.6 mm** mean, EE error **53 → 5.1 mm**, arm-vs-reference **±10° → 0.11°**, thrust
36.74–36.76 N against a 36.75 N hover, no saturation — and at the calibrated kf the OLD
tune diverged outright. The whole-body planner cycle (drone-GS target → Trajectory Planning → Send)
then executed for the FIRST time: pitch within ±0.07°, CoM error ≤6 mm, arrived at
x = 0.480 for a 0.5 m CoM command (the ~20 mm is the forward arm's CoM-vs-base offset) and
returned to HOLD. ONE flight each — repeat-test before hardware (the 2026-08-10 lesson),
though a decaying transient vs a divergence is far outside run-to-run scatter.
**ROBUSTNESS CONFIRMED — the retune carries the +15% kf mismatch with NO further tuning** (user kept the injection on purpose: kf is never exact on hardware and ~15% is near worst case, so a tune that only works at zero mismatch is not a result). Flown, continuous 53 s steady state with the allocator believing the wrong kf: pitch **18.4° → 0.197° p-p** (~93× less attitude motion than the same mismatch before the retune), roll 0.157°, CoM error **18 → 2.85 mm**, EE error **53 → 6.1 mm**, position spread 8×12×0.5 mm, no saturation — within a whisker of the zero-mismatch case (0.14°/2.6 mm). The mismatch is demonstrably ACTIVE: u1 = 42.26 N and 42.26×(4.679931/5.38192) = 36.74 N delivered = exactly hover. Expect a larger ENTRY transient though — CoM error starts ~119 mm and decays 119→26→5 mm over ~40 s as the observer learns the ~13% thrust deficit; that is the estimator working, not a fault. Shipped yaml = retuned M_r_d + 5.38192065e-05. Tables: Command.md §7.14.4 (diagnosis) and §7.14.5 (fix + robustness).
Operating traps from the campaign: land to **z ≥ 0.35** (commanding 0.12, below the
0.305 m resting height, pushed the vehicle into the ground and tipped it), and after any
crash do a FULL step-0 clean — a stale armed PX4 makes the next spawn tip and then refuse
to arm on "Preflight Fail: Attitude failure (roll)".
(c) **q4 WIDENED to ±120° (2026-08-23, user)** — the wrist roll WAS the binding
constraint, so this is the fix for (b)'s residual: in-region feasibility at a
fixed heading went **60% -> 98%** (measured, same 120 samples; the 2 remaining
are IK non-convergence right at the envelope edge), and nothing outside the
region plans either way (0/40, unchanged). The asset authors manip_joint4 at
**±180**, so ±120 is well inside the physical stop. THREE definitions must stay
equal — `transition_planner.Q_MIN/Q_MAX`, the C++ `WbReferenceBuilder::kQMin/
kQMax`, and `torque_controller_isaac_aerial.yaml`'s `min/max_position` (the GS
adopts that one LIVE, and the whole-body launcher now also passes matching
`fallback_*_deg` so the joint row is right before the controller answers).
The hardware/gazebo arm configs kept ±90 until **2026-09-12, when the user made
the cabling/mechanical call and they were widened to ±120 as well** — see the
q4-unification entry at the end of this file. There are now SIX definitions, not
three: add the xacro's `joint4_min/max_deg` (which writes the servo EEPROM) and
`dxl_servo_ids.py`'s `EXPECTED_LIMITS[14]`. Verified after the change: parity 4/4,
planner self-test, whole-body planner loopback, and the GS rendering J4 [-120, 120] with
105° GREEN (it would have been red before). NOT yet flown. The flight itself reached DIRECT
with the whole-body planner live and planning, so the law/whole-body planner/planner path is sound; a full
Assign→Send→execute cycle has still NOT been flown.

**THE ARM IS NOT AN IDEAL TORQUE SOURCE, AND NOW THE SIM KNOWS (2026-09-03, user
request off the 0902/0903 applied-torque analysis).**
**SUPERSEDED 2026-09-09 — the back-EMF droop is GONE from the simulator, because
the arm controller's new software CURRENT LOOP removes it on the real arm. The
entry below is kept for the identification and for the traps it records, but
`servo_model.py` no longer contains `b`, `PEGASUS_ARM_SERVO_MODEL` no longer
accepts `pwm`/`pwm_0903`, and the yaml keys are `sim_arm_current_noise_*`. See
"THE ARM'S CURRENT LOOP" at the end of this file.** The OM-X servos fly in Dynamixel
**Operating Mode 16 (PWM)**, so what the arm controller writes is a DUTY, i.e. a VOLTAGE.
The winding sees `V - Ke*qd` across `R`, so the delivered torque falls by a fixed amount per
unit of joint speed whatever was asked for:
`tau_app = clip(tau_cmd, ±tau_cap) − b*(qd − s_emf*qd_ref)`, **`b = Kt²/R =
[1.102, 0.934, 1.493, 1.102] N·m per rad/s`**. Isaac applied the commanded effort exactly,
which is the mechanical reason it never reproduced the hardware behaviour. New
`extensions/.../robotic_arm/servo_model.py` (`DynamixelPwmServo`, pure numpy, self-test
replays both flights) wired into `06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py`
through **`PEGASUS_ARM_SERVO_MODEL`** = `pwm` (DEFAULT — droop on joints 2/3, ceiling 3.0) ·
`pwm_0903` (adds the 2/3 Sep per-joint ceilings [0.370, 2.160, 1.535, 0.370] N·m, which
reproduces those bags and DELIBERATELY breaks the one-tau_max rule) · `ideal` (the old
behaviour, for an A/B). **This changes 06's default behaviour on purpose.**
**THE SWITCH IS A LAUNCHER KNOB, NOT A CONTROLLER PARAMETER (2026-09-04, user request).**
`start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh` sets
`PEGASUS_ARM_SERVO_MODEL` (default `pwm`); the base launcher `start_single_drone_x650.sh`
VALIDATES it against `pwm|pwm_0903|ideal` and BAKES it into the Isaac pane's command line (the
tmux-server-env trap, same as `PEGASUS_PX4_LOCKSTEP`/`PEGASUS_PAYLOAD_MASS`), and the launcher
prints the active mode in colour. It deliberately does NOT go in
`params_single_aerial_manipulator_whole_body_direct_actuation_t650_sim.yaml`: the droop is a
PLANT property the controller neither applies nor models — that mismatch IS the effect under
test — so a parameter there would be a knob wired to nothing. That yaml now carries a header
block saying exactly this and pointing at the real switch. **Trap fixed while wiring it:** the
base launcher bakes the variable unconditionally, so an unset knob arrives as the EMPTY STRING;
06 now reads `(os.environ.get(...) or "pwm")` rather than a `get()` default, which would have
raised SystemExit on every launcher that does not set it.
**MOVED INTO THE CONTROLLER YAML, 2026-09-04 (user asked three times — their call).** The
sim yaml now owns it: `sim_arm_backemf_enable: true` plus `sim_arm_backemf_b_j1..j4`
(`[0.0, 0.9337, 1.4934, 0.0]` N·m per rad/s) in its `1. REALITY MODEL` section. The
whole-body NODE never declares those keys — rclcpp ignores them, verified by launching a
node named `fsc_autopilot_ros2` against the file (101 params delivered, the five `sim_*`
among them, every gain unaffected) — so the WB **launcher** greps them
(`$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/…_t650_sim.yaml`, path from the machine
conf) and forwards `PEGASUS_ARM_SERVO_MODEL` + the new `PEGASUS_ARM_SERVO_B` to Isaac.
**Precedence: environment > yaml > built-in.** `sim_` prefix on purpose: nothing in the
control law reads them. Cost accepted knowingly: a key in a controller config consumed by a
shell script, and a bare `ros2 launch` ignores it — only the §7.14.1 launcher path applies it.
Validated: rcl parser accepts the file; the launcher resolves all five cases (yaml true/false,
env override of mode and of b, key absent → default + NOTE); two headless Isaac runs confirm
the coefficients arrive.
- **IDENTIFIED, not assumed.** The command chain `duty = clip_±885[clip_±max_effort(tau·N·rho)
  + s_emf*(885*Ke/Vs)*qd_ref]`, `rho = 885*R*I_LSB/Vs`, reproduces the logged Goal PWM to
  **≤0.23 duty counts (1.8 mN·m) over 32 700 samples**, so the commanded side is exact and the
  whole gap is the motor. At rest j2/j3 deliver 99.4/94.0% (3 Sep) and 100.1/97.6% (2 Sep).
  In motion the drop is linear and symmetric with slope 0.950 (j2) and 1.409/1.476 (j3)
  against `Kt²/R` = 0.934/1.493 with NO fitted parameter (`Kt = Ke` in SI for any DC machine).
  Model error on the recorded flights 0.247 → 0.134 (3 Sep j2) and 0.222 → 0.067 (j3) N·m rms.
- **THE LOSS IS ABSOLUTE, NOT A PERCENTAGE.** 0.93/1.49 N·m per rad/s whatever the command,
  so delivery ≥90% needs `|tau_cmd| ≥ 10*b*|qd|` and the torque REVERSES at `|qd| = tau/b`.
  A 0.5 N·m command on j3 is faithful only to 0.033 rad/s (1.9 °/s). Never score this with a
  percentage alone — the commanded torque carries a large static gravity-hold component that
  the motor does deliver, so eta can read 104% in a phase whose mean error is already 0.04 N·m.
- **Joints 1 and 4 are deliberately left at b = 0.** Their command (0.03–0.04 N·m rms) never
  left the current sensor's noise floor in either flight and j1's fit (0.457 vs theory 1.102,
  R² 0.50) disagrees with theory; `backemf_joints=range(4)` enables them if wanted.
- **No double-count:** `switch_dof_control_mode(mode="effort")` zeroes the drive stiffness AND
  damping, so the asset's authored `drive:angular:physics:damping` 1.5/0.7 on manip_joint2/3 is
  NOT active — before this change the simulated joints had no damping at all. The droop is
  applied EXPLICITLY as a torque; `b*dt/I` = 0.19/0.30 at the 0.0200 kg·m² armature and 250 Hz
  (stable below 2). If the physics step ever grows, move it into the PhysX drive damping, which
  is implicit.
- `joint_states.effort` out of 06 now carries the **APPLIED** torque, matching what the hardware
  backend reports there (Present Current); before the servo model existed the two were one number.
- **NOT modelled and it matters:** gearbox friction. `tau_app` is Kt × Present Current, the
  ELECTROMAGNETIC torque; the 353.5:1 gearbox's losses sit downstream and the flight data cannot
  separate them (no torque sensor). A breakaway torque needs a bench measurement.
  **The fix on the real arm is current-control mode (Mode 0)** — the servo then closes its own
  current loop, R and Ke drop out and b goes to zero; this model should then be DELETED, not
  retuned. Also: the controller's configured `back_emf_ke[2] = 2.86` contradicts j2's calibrated
  `Kt = 2.139` by 34% (it did nothing in these flights because qd_ref ≈ 0, but it is wrong —
  the calibration implies `[2.323, 2.139, 2.534, 2.323]`).
- **EXPECT THE WHOLE-BODY TUNE TO CHANGE.** The law's `N1^T u3` arm-reaction pre-compensation is
  exact only if the arm delivers the modelled torque; with the droop in, the sim now carries the
  same error hardware does, which is the point. Validated offline against both flights and
  smoke-tested in Isaac (banner, `b*dt/I`, `|tau|max cmd/app` in the status line, clean 4000-step
  run); **NOT yet flown in a full whole-body SITL flight.** Report:
  `docs/experimental_data_ros2_bag/0903 - T650-AM whole-body/analysis/` (extraction +
  `applied_torque_analysis.py` + `wb_divergence_report.html`).
- **`utils/wb_hover_stability.py` also runs the servo now** (`--servo pwm` default,
  `ideal` for the A/B) — the offline screener had the same ideal-arm idealisation. It did
  NOT repair the tool's absolute verdict: it still calls the flown +15 % kf run divergent in
  every combination tried (servo on/off x old/new M_r_d), so keep trusting the ORDERING only.
  **Three STALE constants found there while checking, flagged in-file and deliberately NOT
  changed** (changing them re-bases every sweep recorded against the tool): `KF_TRUE`
  4.679931e-05 should be `t650_params.ROTOR_CONSTANT` = 4.041283e-05 since the 2026-08-25
  re-anchor (15.8 % high — the tool flies the pre-re-anchor, too-strong plant); `KF_SHIPPED`
  5.38192065e-05 should be the sim yaml's `alloc_thrust_coeff` 4.6474755e-05 (the +15 % RATIO
  is right, the absolute loop gain is not); and `make_gains()`'s `mrd` is the PRE-retune
  M_r_d against a yaml that has carried (0.116522, 0.136107, 0.125102) since 2026-08-23.
- **THE TWO ARM REFERENCES ARE NOT SYNCHRONISED — the arm controller REPLAYS the whole-body planner's
  move 3-4 s LATE (2026-09-03, measured on both 0902 and 0903).** `q_d` (the whole-body planner's
  `whole_body_direct_actuation/reference`, MODEL convention) and `q_cmd`
  (`external_torque_controller/smoothed_reference_joint_trajectory`) run the SAME move one after
  the other: 3 Sep joint 3 whole-body planner 24.81-27.81 s / arm 28.71-33.05 s (**lag +3.90 s**), 2 Sep
  joint 3 +3.04 s and joint 2 +4.06 s. **Endpoints agree EXACTLY** (8.28/8.28, 40.00/40.00), so it
  is a TIMING fault, not wiring or convention, and the lag equals the length of the whole-body planner's move
  — a goal delivered ONCE AT COMPLETION rather than streamed. The measured joint tracks `q_d` and
  ignores `q_cmd` (correct: the law owns the torque in passthrough), so **every compensation term,
  the stale-fallback PD hold and the DIRECT-entry gate are all computed against a trajectory
  nothing is flying.** This is UPSTREAM of the qd_ref-vs-qd_meas choice — feeding the back-EMF term
  measured velocity fixes what it compensates, not the 3-4 s stale picture.
  **`external_torque_controller/target_joint_setpoint` recorded ZERO messages in both flights**
  despite the recorder being subscribed and the whole-body planner spending 3.9 s / 11.4 s in EXECUTING;
  **CORRECTED 2026-09-04 by a live capture:** `ros2 topic info` on that topic reads **Publisher
  count: 2** (the whole-body planner's arm-sync AND the arm GS), so the publisher IS in the graph — the
  earlier inference from rosbag2's `offered_qos_profiles` (one TRANSIENT_LOCAL profile ⇒ the
  whole-body planner's VOLATILE publisher absent) was WRONG; that field is not a reliable publisher count.
  What stands is the ZERO MESSAGES: the whole-body planner advertised and did not publish during either
  flight, so the goal reaches the arm controller by a path the recording does not show.
  **Check `ros2 topic info -v` + `ros2 topic hz` on that topic during EXECUTING before
  instrumenting anything else.** Charts, the per-joint lag table and the six-term decomposition are
  in the report artifact; the analysis is `applied_torque_analysis.py`'s `sync` / `ff` payload.

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

**THE JOINT-POSTURE TERM: MECHANISM FOUND, TWO PAPER-CONSISTENT FIXES FLOWN 3/3
WITHOUT IT (2026-09-09, user request — the three questions: which disturbance,
can gains fix it, what is the minimal provable change).** Full write-up
`docs/docs_aerial_manipulator/Removing the Joint-Posture Term.md`; data + drivers
`docs/docs_aerial_manipulator/posture_ablation_20260909/`; offline tool
`application/robotic_arm/utils/wb_entry_sim.py` (exact law + L1 observer + plant
injections + joint stops + rotor lag; reproduces every flight to the mm — the +5%
model flight parked the arm at q = [−35, 11, 24, 30] deg with e_y 214 mm, the sim
said [−35, 9, 26, 31] and 214 mm; the anchor-only fix flew 102.7 mm, the sim said
102.7). **Neither failure is a gain problem; both are the manuscript modelling `d_e`
as the CONTACT wrench only:**
- **Mechanism 1 — entry transient vs. workspace.** At the SAFETY→DIRECT switch the
  observer restarts at zero, the unlearned 10.8 N / 0.4 N·m drive a 0.8–1.3 m base
  excursion, and the WORLD-fixed EE reference is unreachable (arm workspace ~5 cm at
  the folded home). `e_y` tracks `e_x` 1:1, q3 hits +50 deg at 1 s, then crosses into
  the elbow-singular branch. With the PID on the same 0.8 m transient happens and the
  arm simply never moves — the 2 N·m/rad PID overrides the 2 N/m task, which is why
  `K_y` had to drop to 2 when the PID was added: a compensating pair.
- **Mechanism 2 — the internal disturbance is a permanent phantom contact force.**
  `(J_y^#)^T` maps 0.22 N of task force per N of vertical deficit at home; the
  impedance renders it, so the arm walks off home by F/K_y: 5% model → 0.41 N →
  214 mm at K_y 2 (measured 214–218), then a stop, then the branch. The L1
  attribution does NOT remove it (closed loop = F̂_ext + M_y Λ^-1 (F − F̂)).
- **Ablation flights, PID off, plant-side kf, matched allocator** (hover soak):
  delay-only 3/3 clean; +5% model 2/2 vehicle ok but ARM parked off-branch (214 mm);
  +5% kf 1 abort / 1 ok with the arm on the negative branch; +5%+5% ok, elbow branch,
  clamped. **Plant-side kf ≤0.90 CANNOT TAKE OFF** (SAFETY gravity FF short + UDE
  height gate = 7.13.3 run C deadlock) — those rows are invalid, so the fix flights
  used the allocator-side +17.6% (the 09-06 configuration). Yesterday's 17-flight
  in-law sweep (`l1_inlaw_20260909`) and today's offline sweep of EVERY gain group
  (k_x/k_v, k_R/k_w, K_y/D_y 0.5–200, M_y, DLS, ω_c, seeds): nothing survives.
- **The fix, two options, both default-off in the C++ (parity 9/9 unchanged):**
  `wb_ee_anchor_com` (free-flight EE reference = planned EE-minus-CoM offset held
  about the ACTUAL CoM: r_ed + e_x, derivatives + e_vx/+e_ax, no differentiation;
  proof = change of error variable, u3 untouched) and `wb_u3_internal_ff`
  (coupling feedforward carries u + d̂, i.e. `inner += (J_y^#)^T d̂_e`; proof = one
  exponentially decaying observer-error input, the same treatment u1/u2 already get).
  Flown at the full injection, posture 0, K_y 20/D_y 12: **3/3 completed, peak e_y
  29–31 mm through a 0.85–0.92 m base transient, settled 1.2 mm, arm back at
  [0,40,41,−1], 0% clamp** (K_y 50/20: 15 mm / 0.5 mm). Anchor alone: flies with a
  102.7 mm steady offset (= 2.4 N phantom / 20). Feedforward alone at K_y 2: arm at
  the stops, 68% clamped. **The shipped yaml is UNCHANGED** (posture on, K_y 2, both
  keys false) — adopting = keys true, K_y 20 / D_y 12, posture 0. Provable joint
  potential, if a branch selector is still wanted: `K_y e_y + J_3y^{-T} K_q e_q`
  (a potential in y through the local diffeomorphism), NOT the flown `K_p e_q` on u3.
- **THE ARM CHANNEL WAS THE ONLY ONE WITHOUT A CANCELLATION, and that is the whole of
  mechanism 2.** `f_d` carries `−d̂_t` and `u2` carries `−d̂_r`; `u3`'s only estimate is
  `F̂_y`, which enters through the inertia-shaping `(Λ_y M_y^-1 − I)` and is a READING of
  the contact force, not a compensation — and `d̂_ρ` is computed and injected NOWHERE.
  The L1 decomposition fixes the reading (`F̂_y` → 0 correctly, no contact) and changes
  nothing about the force: `(J_y^#)^T d` still acts on the task, with UNIT gain at
  natural inertia. Written into the design note 2026-09-09 as the paragraph "The arm
  channel needs its own cancellation" + eqs. (ee_dyn_int), (u3_int), (ee_closed_int) in
  `disturbance observer design/disturbance_observer_draft.tex` (PDF rebuilt, 0 errors).
- **THE TERM HAS THREE BLOCKS AND THE ARM BLOCK IS THE LEAST OF THEM** (asked 2026-09-09,
  measured): the addition is ONE term `−J̄_3y^-1 (J_y^#)^T d̂`, three only because
  `(J_y^#)^T = [J̄_1y J̄_2y J̄_3y]`. The two BASE blocks are NOT a second copy of the
  `−d̂_t`/`−d̂_r` in `f_d`/`u2` — those are in the COMMAND, `d` is in the PLANT, and `u3`
  pre-compensates their SUM (the arm is bolted to a base a thrust deficit accelerates).
  Flown-estimate split at home: `J̄_1y d̂_t` 2.380 N, `J̄_2y d̂_r` 0.498, `J̄_3y d̂_ρ` 0.653,
  total 2.127 (the arm block OPPOSES the base blocks). So carrying `d̂_ρ` ALONE is WORSE
  THAN NOTHING — offline 102.7 → 148.3 mm; base blocks alone 36.3; all three 1.3.
- **TWO FEEDFORWARD SOURCES, both flown.** `wb_u3_internal_ff` feeds the filtered LUMPED
  `d̂_Σ,f`; `wb_u3_internal_ff_use_w_hat` (added 2026-09-09) switches to the note's
  INTERNAL `T^-T ŵ` (Step 2), the form eq. (u3_int) states. **In free flight the lumped
  one is BETTER and exact** (`d_e = 0` ⇒ it IS `d`); the internal one is the only one
  admissible under CONTACT. Flown 2/2: settled EE 3.1/3.4 mm and peak arm torque
  1.34/1.28 N·m vs the lumped 1.2 mm / 0.8 N·m, arm dipping to q3 19° vs 38° mid-entry.
  That cost is `ŵ` being PARTIAL — a stationary arm gives α = 0 in the note's PE
  condition, only 4 of 10 directions identified — and it works here only because `L_c`
  books **99.3% of the residual to the collective** (measured `ŵ_z` = −10.73 of −10.81 N).
- **MISMATCH GRID, 9 flights, all completed** (2026-09-09): thrust {10,15}% x model
  {5,10}%, compensation on, posture off, K_y 20/D_y 12. Settled EE error 0.7/0.9/1.0/1.2 mm
  against d̂_z −6.12/−8.65/−8.17/−10.81 N — **linear at ~0.11 mm/N**, i.e. residual observer
  error, not phantom force; 0% clamp everywhere, arm ends at home in all nine, repeats agree
  to a decimal. Base transient grows 345 → 919 mm (that is the handover, 7.15.8, untouched).
  Thrust MUST be allocator-side: plant-side above ~5% never takes off (7.13.3 run C deadlock).
- **`u3_internal_ff` REQUIRES AN ATTRIBUTING OBSERVER, and nothing enforces it** (found
  2026-09-09 while checking the law against the note): the red d̂ terms are the INTERNAL
  disturbance and the blue F̂_y is the CONTACT force, two different objects. On the L1 path
  with decompose=true they are (attributed F̂_y ≈ 0.06 N measured); on the GMO path, or L1
  with decompose=false, F̂_y IS the same lumped vector as d̂, so it enters twice and the
  effective coefficient is 2I − Λ_y M_y^-1 ≈ diag(1.52, 0.68, 1.22, 1.60) instead of I.
  NOT flown that way, but the key is declared in all three whole-body yamls. A startup
  refusal was offered and NOT added.
- **FULL 8-LEG MISSION FLOWN TWICE WITH THE PID AT ZERO, AND THE KEYS THEN DELETED**
  (2026-09-09): 314 s / 276 s of DIRECT, 9/10 and 10/10 legs, **0.00% clamp both**, peak
  arm torque 0.79 N.m of 3.0, **q3 NEVER negative** (0.00 s on the elbow-singular branch
  the term existed to prevent); q2 touched its +50 stop 0.31 s during the first step's
  entry transient only. Run A's missing leg is a DRIVER RACE, not a control failure: the
  planner went PENDING->CALCULATING->PLANNED inside one 10 ms sample, the driver's `send`
  then hit CALCULATING again because the second target restarted the solve, and it waited
  out its 45 s exec timeout while the vehicle held at 1.5 mm EE error / 0% clamp. Fix
  belongs in wb_l1_campaign_driver.py, NOT the law.
  **The five `wb_posture_*` keys are now DELETED from the L1 sim yaml** — they are
  kOptional/default 0.0, so absence makes the term UNREACHABLE from that file, not merely
  off, and the green LAW CHECK line proves it per launch. The `alloc_thrust_coeff` block's
  "THIS AND wb_posture_* ARE COUPLED" warning was rewritten: that coupling is gone.
  **SCOPE: that one file.** The C++ is untouched and the three GMO whole-body yamls still
  need the term.
- **POSTURE-TERM WATCH, printed once a second in DIRECT** (2026-09-09, user request):
  `WbCommand::t_posture` carries the joint PID's ACTUAL applied contribution to u3 (zero
  while `hold` owns the arm), and the client prints it — GREEN `posture PID: u3_posture =
  [0 0 0 0] N.m (max|.| = 0) -- kp/kd/ki/i_max = 0/0/0/0` while it is zero, YELLOW with the
  four values, max, %-of-|u3| and the gains the moment it is not. Verified in a live flight,
  not just built. Complements the startup LAW CHECK: the banner says how it is CONFIGURED,
  this says what is APPLIED. Parity 9/9 after the change.
- **ENTRY-TRANSIENT TUNE, NO UDE SEED (2026-09-09): `wb_k_x` 16 -> 32, `wb_k_v` 12 -> 20.**
  Peak base error at the SAFETY->DIRECT switch **919 -> 411 mm**, under 100 mm in
  **6.4 vs 12.4 s**, peak tilt 10.0 -> 6.3 deg, EE behaviour unchanged (peak 30.4,
  settled 1.18 mm), 0% clamp, full 8-leg mission 10/10. Mechanism: the peak error is
  HORIZONTAL -- the vehicle sags ~220 mm, the unlearned CoM moment tilts it, the tilt
  converts thrust to sideways drift -- so the position loop AND the rotational observer
  are both levers. wn 2.07 -> 2.92 rad/s, zeta 0.78 -> 0.91, still far under the 10.03
  rotor pole.
  **`wb_l1_omega_c_r` 2.0 ABORTS and that result is load-bearing**: best ENTRY of any
  candidate (303 mm) then a slow rotational oscillation from t = 8 s to a 36 deg abort at
  15 s (|e_R| 0.04->0.38, d_hat_r swinging +-2 N.m). 7.15.8's ceiling on that gain STANDS;
  the compensation did not lift it. 1.0 and 1.5 are stable (545 / 388 mm) but 1.5 is only
  1.33x below a measured instability, so the POSITION-LOOP lever was shipped instead --
  same peak, better recovery, observer untouched.
  **`wb_entry_sim.py`'s delay-margin column is NOT a stability certificate for
  `omega_c_r`** -- it gave the 2.0 case 32 ms against a 16 ms nominal and the rig failed at
  nominal. Trust it for ORDERING only, as its own header says.
  Data + drivers: `posture_ablation_20260909/run_entry_tune{,2}.sh`, `entry_score.py`.
- **Open:** controller.py (the Python source of truth) does not carry the hooks —
  the parity fixture locks them OFF only; **NO CONTACT has been simulated on any run,
  which is the entire reason `ŵ` is preferred over the lumped estimate** (06 applies no
  EE wrench; that block existed in 02 and was dropped from 03 onward, so `w_e ≡ 0`
  everywhere and the source distinction is derivation, not measurement); steps/trajectory
  legs not flown with them; interaction phase must keep the WORLD anchor (planner switch
  not implemented); GMO variant offline only (34 mm settled); the 0.9 m entry transient
  itself is the handover (7.15.8), untouched.

**THE ARM'S CURRENT LOOP — THE DROOP IS GONE, THE RESIDUAL IS THE MODEL
(2026-09-09, user request).** `fsc_open_manipulator` now closes a **software
current loop** around Dynamixel Mode 16 (design + bench validation:
`fsc_open_manipulator/doc/Current Loop Design.md`, figure
`doc/current_error_all.png`), calibrated on J2/J3. It is deliberately SLOW —
`wc` = 1.5 Hz, 6.6x below the stability limit — because rejecting the back-EMF
disturbance IS deleting the machine's own damping `Kt^2/R`, and that damping is
the scarce resource on an arm whose velocity signal has 48 ms of group delay.
The simulator's plant model was rebuilt around its OUTCOME:
- **REMOVED: the back-EMF droop.** `servo_model.py` no longer contains `b`,
  `B_BACKEMF`, `backemf_joints`, `back_emf_ff_scale`, `implied_b_true` or
  `explicit_stability_ratio`; `applied()` lost its `qdot`/`qdot_ref` arguments
  and takes `dt` instead. `PEGASUS_ARM_SERVO_MODEL` is now **`current|ideal`**;
  `pwm` and `pwm_0903` are REFUSED WITH A MESSAGE rather than reinterpreted
  (`pwm_0903` existed to replay the 0902/0903 droop-era bags and cannot without
  `b`). Yaml keys `sim_arm_backemf_*` -> **`sim_arm_current_noise_{enable,a,
  bw_hz,seed}`** in BOTH whole-body `_sim` yamls; `PEGASUS_ARM_SERVO_B` ->
  `PEGASUS_ARM_CURRENT_NOISE_{A,BW_HZ,SEED}`, validated and baked into the Isaac
  pane by `start_single_drone_x650.sh` (the tmux-server-env trap, unchanged).
- **ADDED: the residual.** `tau_app = clip(tau_cmd, +-tau_cap) + Kt * i_err`,
  `i_err` **zero-mean, 7 mA rms, first-order band-limited at 5 Hz**, four
  independent channels, seeded. 7 mA is **J3** — the worse of the two calibrated
  joints (bench, loop closed: j2 4.2-4.6, j3 6.7-6.9 mA rms) — and per the
  user's instruction the same magnitude goes on all four; J1/J4 carry commands
  near the current sensor's floor and are uncharacterised, so that is a
  documented assumption. Through each `Kt` it is
  **[16.3, 15.0, 17.7, 16.3] mN.m rms**, ~2% of the arm's 0.7 N.m hold torque.
  **The SHAPE is the point, not the size**: loop off, j2/j3 sat on a persistent
  one-sided ~-20 mA offset and never crossed zero (delivery 93.3/87.5%); loop
  closed, the error is centred (99.4/91.8%). A bias corrupts a torque-controlled
  flight, scatter does not — and the 2026-09-03 model was all bias.
- **Generated as an EXACT-DISCRETIZATION AR(1)**, `a = exp(-2*pi*fc*dt)`,
  `i <- a*i + sqrt(1-a^2)*sigma*randn(4)`, so the stationary rms is `sigma` at
  ANY step size. An Euler form's variance scales with `dt` and would silently
  retune the plant if the physics step ever changed — the same lesson
  `lagged_thrust_curve.py` records for the rotor lag. Self-test measures the rms
  dt-invariant to 0.4% at 4 and 16 ms and recovers `fc` = 5.0 Hz from the lag-1
  autocorrelation.
- **The software loop is NOT run in simulation, on purpose, and there is a real
  foot-gun here.** The plant models its outcome; the residual is already
  post-correction. But `IsaacTopicEffortSystem` DOES export an effort state
  interface (06 republishes the applied torque), so setting
  `current_loop_bandwidth_hz > 0` in `torque_controller_isaac_aerial.yaml` would
  close a SECOND loop around a signal that is already post-loop and cancel the
  disturbance under test. It defaults to 0.0; that file now carries a comment
  saying why it must stay there.
- **The controller cannot see it**, which is what makes the test fair: the
  whole-body node never subscribes to `joint_states.effort` (grep-verified — it
  only publishes its own commanded torque there). The residual reaches the law
  only through the physics.
- **`wb_l1_lc_var_q` 0.0025 was flagged as "must be RAISED once the droop is
  on". That instruction is now VOID** — a droop is a systematic joint-torque
  error the observer should be allowed to find; zero-mean noise is one it
  should NOT chase, so a low prior is right. Noted in the yaml.
- **FLOWN, 3 missions, 0 aborts, no gain touched.** Plant = the shipped `_sim`
  config (+15% allocator kf, mass/inertia x1.10, 10/10/5 mm CoM shift, MN4010
  rotor lag); law = the 2026-09-09 config (posture term absent,
  `wb_ee_anchor_com` + `wb_u3_internal_ff`, K_y 20/D_y 12, k_x 32/k_v 20).
  Run B against the matched ideal-arm baseline `l1_mission_kx32.npz`
  (same mission, same 16 s holds):

  | | noise ON | ideal arm |
  |---|---|---|
  | legs completed / aborted | 10/10, no | 10/10, no |
  | DIRECT | 276.2 s | 276.9 s |
  | CoM err mean / late | 21.0 / 9.6 mm | 18.8 / 5.0 mm |
  | EE err mean / max | 3.5 / 33.4 mm | 2.9 / 30.4 mm |
  | tilt p-p, \|e_R\| max | 3.08 deg, 0.181 | 2.78 deg, 0.170 |
  | peak arm torque | 0.798 of 3.0 N.m | 0.751 |
  | joint clamp / rotor sat | **0.00% / 0.00%** | 0.00% / 0.00% |
  | q2/q3 abs err | 0.81 / 1.21 deg | 0.58 / 0.97 deg |
  | phantom \|F_hat_y\| | 0.123 N | 0.077 N |

  Run C repeats B to within a few percent on every row (CoM mean 16.9 mm, EE
  3.2, tilt 2.95 deg, tau 0.785, 0.00% clamp), so this is not one lucky run.
  Per-leg: every leg completed, peaks within 8% of the baseline, settled errors
  1.5-6x larger in a range that is 3-8 mm.
  **THE MECHANISM IS IN THE ARM CHANNELS OF THE OBSERVER, and it reproduces
  3/3** — std over the quiet soak hold, N.m:

  | | d_hat^c UNFILTERED (deadbeat) q1/q2/q3/q4 | d_hat FILTERED (what the law sees) |
  |---|---|---|
  | noise, 3 runs | .0247-.0254 / .0389-.0496 / .0229-.0266 / .0199-.0206 | .0029 / .0031-.0050 / .0039-.0042 / .0038-.0040 |
  | ideal arm | .0138 / .0330 / .0143 / .0108 | .0003 / .0047 / .0025 / .0006 |

  The unfiltered deadbeat estimate picks the injection up almost exactly — the
  quadrature excess over the ideal run is **16.7 mN.m on q4 and 17.9 on q3
  against the 16.3 / 17.7 injected** (q1/q2 read ~21 because the law's own
  reaction adds there) — and the L1's explicit `C(s) = omega_c/(s+omega_c)`
  then cuts it ~5x, to 3-5 mN.m against the arm's 700 mN.m hold torque. That
  bandwidth/accuracy split is exactly what the observer exists for, and it is
  why 7 mA costs single-digit millimetres.
  **DO NOT read this off the COLLECTIVE channel.** `dc_z_std` is 0.73 / 0.23 /
  0.29 N on noise-B / noise-C / the IDEAL run — the arm's joint-space noise is
  not what dominates it, and its run-to-run spread swamps the effect. Scoring
  it that way once made the noise look 2.5x worse than ideal on one run and
  identical on the next.
- **Measured end-to-end from flight telemetry, not just from the model**: over a
  33 s quiet hold, `applied - commanded` has mean **0.1-0.8 mN.m** (the bias is
  really gone) at 21-24 mN.m rms against the 15-18 injected — the excess is the
  law REACTING, its own commanded torque carrying 13-19 mN.m of ripple at 250 Hz
  in the same window, plus the asynchronous 50 Hz log sampling of two streams.
  **The applied-torque log columns are in MODEL order** (verified by
  cross-correlation), NOT the broadcaster order `LOG_Q_OF_JOINT` applies to the
  POSITION columns — mixing them up scores joint 2 against joint 1.
- Tooling: `wb_hover_stability.py`'s `--servo` is now `current|ideal`, seeded;
  read its delay-margin ORDERING with `ideal`, since the noise is scatter.
  Campaign: `docs/docs_aerial_manipulator/arm_current_noise_20260909/`
  (`run_noise.sh`, README). NOT flown on the GMO rig, though the plant change
  reaches it (its yaml has the same new keys); NOT flown on hardware.

**WHOLE-BODY SAFETY GUARD — every DIRECT exit hovers in place and folds the arm
home (2026-09-11, user request).** Three small changes across three repos, and
nothing in any control law:
- **L1 sim yaml:** `system_wd_max_tilt_deg` 40 → **20**. The watchdog's tilt is
  `acos(R22)` ≥ max(|roll|, |pitch|), so it trips on either. The GMO twin still
  has 40.
- **fsc_open_manipulator `arm_planner`:** new `home_on_safety_revert` (node
  default false; the Isaac aerial yaml sets it true) plus
  `home_on_revert_delay_s` 1.0. On the DIRECT → SAFETY edge of the mode topic it
  holds the measured pose for 1 s, then folds home on its min-jerk profile. Any
  arm command during the delay cancels the fold.
  One edge covers all four exits: the watchdog, the feedback-loss failsafe, the
  drone GS "Return to baseline" (`activate_controller`) and `set_direct_mode
  false`.
- **WB client:** `switchMode` publishes `/mode` immediately rather than on the
  500 ms info tick.

Hover-in-place needed no change — `switchMode(kSafety)` already re-seeds
`outer_ref_` to the current pose, and the drone GS publishes a setpoint only when
the operator presses send, never as a stream.

Flown 3× in Isaac with NO reference published after the revert (Command.md
§7.15.1):
- Arm home in 2.40 / 2.44 s, final error ≈ 0.4°.
- Settled within 28–30 mm of the revert point.
- Nominal DIRECT peaked at 6.9–7.3° of tilt.

**Two excursions, measured apart with a no-fold control flight:**
- The ~0.45 m VERTICAL climb after the revert is the pre-existing handover
  surge. It is identical without the fold. In DIRECT the UDE is fed the
  allocator's believed collective, and this yaml's allocator believes a kf
  17.6% too high. It is sim-only.
- The ~0.4 m SIDEWAYS walk IS the fold: 72 → 394 mm, starting on a +6° roll
  spike. Un-yawing the arm moves the CoM, and PX4's integrator re-learns the
  trim.
- Knobs, neither flown: the fold delay and the planner's `max_velocity`.

Loopback test (`test_arm_planner.py`, 12 checks) also covers the fold. Its
teardown no longer aborts with rc 134. **Trap:** that test drives the REAL
`/uav_0/.../mode` topic, so a live stack still republishing `SAFETY` makes the
four DIRECT checks fail — tear the sim down first. NOT flown on the GMO rig or
on hardware.

**SIDEWAYS-DRIFT TRIP + THE L1 HARDWARE PAIR (2026-09-11, same request).**
- **New `system_wd_max_drift_m`** (node default 0.0 = off, so the GMO and
  hardware GMO yamls are untouched), **0.75 m in the L1 sim yaml**. It measures
  `|x_c − x_cd|_xy`, the law's own HORIZONTAL CoM tracking error against **the
  reference the law is flying** — not a fixed point, not the raw GS setpoint —
  which is the only version that survives planned motion: the planner moves the
  base 0.5 m on a step leg and none of it counts. `driftTripped()` runs in the
  DIRECT tick after reference selection, shares the tilt/rate watchdog's latch,
  and is evaluated BEFORE the law step so a tripped tick commands nothing.
  Measured peaks: 410 mm at the DIRECT entry, 349 mm on the legs, so 0.75 is
  ~1.8x the worst nominal. Flown: the full 10-leg mission with **no false trip**
  (170 s DIRECT, 0 saturations), and a forced trip at a temporary 0.15 m limit
  that fired 1.44 s into DIRECT **at 7.0° tilt** (so tilt cannot explain it),
  hovered to 21 mm and folded the arm home in 2.40 s.
- **Arm fold for a stack with NO `arm_planner`** (i.e. hardware): new
  `system_arm_go_home_service` + `system_arm_go_home_delay_s` (1.0 s) call the
  arm controller's own `go_home` after the revert; re-entering DIRECT cancels a
  pending fold. EMPTY in the sim yaml on purpose — `arm_planner` owns that edge
  there, and two mechanisms would command the arm twice.
- **NEW L1 HARDWARE PAIR, never flown:**
  `config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650.yaml`
  + `scripts/indoor_exp/start_whole_body_l1_direct_actuation_stack_t650_aerial_manipulator.sh`.
  PLANT/ALLOCATOR/SAFETY map from the GMO **hardware** file; LAW/OBSERVER from
  the L1 **sim** file; every `sim_plant_*`/`sim_arm_*` key and the kf injection
  dropped; guard at 20° / 0.75 m. **Trap I hit: the copied launcher must KEEP
  the shared `fsc_indoor_autopilot_stack` session name** — renaming it per fork
  would stop it killing the GMO stack, and both publish the same PX4 setpoints.
  Validated by loading the yaml on the node at `/uav_test` (guard params,
  identity, L1 banner, green posture-OFF check, mass cross-check).
- **PRE-EXISTING HARDWARE BLOCKER found while doing this, NOT fixed:** the
  hardware `wb_arm_reference_topic` is `reference_joint_trajectory`, which only
  `arm_planner` publishes, and the hardware arm launch
  (`external_torque_hardware.launch.py`) does not start it — the controller
  keeps its own generator and publishes `~/smoothed_reference_joint_trajectory`.
  The DIRECT-entry gate checks that topic, so hardware DIRECT entry should be
  REFUSED today, on the GMO hardware stack too. Left for a decision: run
  `arm_planner` alongside the hardware arm stack (set
  `external_reference_topic`, the sim topology, which also supplies the fold) or
  repoint the key. Recorded in both new headers.

**ARM CALIBRATION + PER-JOINT CURRENT LOOP ALIGNED INTO THE SIM AND BOTH WB-L1
LAUNCHERS — AND THIS MACHINE CANNOT VALIDATE THEM (2026-09-11, user request).**
Pulled `fsc_open_manipulator` (6 commits, `bf80b51..06e598c`: the Mode-16 current
loop, the motor-constant campaign, the j1/j4 unit swap) and aligned the plant and
the two whole-body L1 launchers to it. Full write-up + all eight flights:
`docs/docs_aerial_manipulator/arm_calib_20260911/`.
- **THE CALIBRATION** (`servo_model.py`): `NM_TO_COUNTS`
  `[160.0, 173.8, 146.7, 160.0] -> [162.4, 154.0, 150.5, 153.4]` counts/N.m,
  `RESISTANCE_OHM` `[4.90, 4.90, 4.30, 4.90] -> [5.26, 4.90, 4.53, 4.88]`. The
  j1/j4 UNITS were calibrated by swapping them into the j2/j3 brackets, where
  gravity gives them a lever. **Two checks now ASSERT in the self-test and they
  are what makes the set trustworthy**: the derived `NM_TO_DUTY` comes out
  `[169.47, 149.70, 135.25, 148.51]`, exactly what the arm controller LOGGED on
  hardware, and `eta = Kt/Ke` against the measured Ke is `[0.943, 0.943, 0.942,
  0.943]` — four motors agreeing to 0.1% on a sensible gearbox loss, where the
  superseded set implied the impossible `eta > 1` on j2. Believed to **+-2%**:
  the campaign found kappa conditioning-limited, not measurement-limited.
- **THE RESIDUAL IS PER JOINT, AND THE UNTRIMMED JOINTS ARE THE NOISY ONES.**
  `current_loop_bandwidth_hz_joints` ships `[0.0, 1.5, 1.5, 0.0]` — the trim
  helps the loaded joints and HURTS j1/j4, whose commanded current is mostly
  noise. So `CURRENT_NOISE_A` `0.007` uniform -> `[0.0156, 0.0062, 0.0096,
  0.0156]` A, i.e. torque noise `[16.3, 15.0, 17.7, 16.3] -> [35.7, 15.0, 23.7,
  37.8]` mN.m. The 2026-09-09 note that j1/j4 were "uncharacterised, a
  documented assumption" is superseded.
- **TWO JUDGEMENTS THE MEASUREMENT DOES NOT SETTLE, both flagged in-file rather
  than quietly decided.** (a) THE SIGMA CONVENTION: the bench scores the 5 Hz
  CONTENT, an IN-BAND rms, while the model wants a TOTAL. `total = sqrt(2) x
  in-band` is right only if the real spectrum keeps falling like the model's
  above 5 Hz, which `servo_model.py` says is "not characterised" — a 41%
  inflation either way. The superseded model used `sigma = in-band` directly.
  Both were flown; neither caused the failures. (b) **IS j1/j4's RESIDUAL EVEN
  TORQUE?** At the steady hold the COMMANDED current is **0.2 / 108 / 36 / 0.0
  LSB** (2.69 mA quantum) while the measured residual is 4.1 / 1.6 / 2.5 / 4.1.
  On j2 it is unambiguously a real torque error; on j1/j4 the command is ZERO
  and the "error" is ~4 sensor quanta, which is why the arm repo says the trim
  there chases noise. Real ripple (torque the law must reject) and current-sense
  noise (no torque at all) are indistinguishable in that data and have OPPOSITE
  plant consequences. **A bench run at a NON-ZERO j1/j4 load separates them** —
  a real error scales with command, sensor noise does not.
- **PLUMBING**: the sim yaml still carried `sim_arm_backemf_*`, deleted from the
  plant on 2026-09-09, so the launcher fell back to built-ins and printed a NOTE
  every launch. Now per-joint `sim_arm_current_noise_a_j1..j4`; `06` and
  `start_single_drone_x650.sh` accept `"a1,a2,a3,a4"`. `wb_arm_torque_plot.py`'s
  hard-coded `NM_TO_DUTY` now IMPORTS from `servo_model.py` so a re-calibration
  cannot leave it scoring the previous arm.
- **HARDWARE LAUNCHER**: `max_effort` is unchanged in COUNTS but its N.m value
  moved with the calibration, `[0.38, 2.19, 1.36, 0.38] -> [0.34, 2.44, 1.42,
  0.39]` — j2 gained 13%, j1/j4 lost ~9%, and these STILL BIND below the law's
  `wb_tau_max` 3.0. New non-fatal ARM-SIDE CONFIG CHECK verifies kappa, the
  per-joint bandwidth, Ke and R against the arm repo at startup
  (`FSC_OM_ARM_REPO` to point it). **Two hardware items re-checked, not
  assumed**: `external_reference_topic` is STILL unset and the bring-up still
  starts no `arm_planner`, so the DIRECT-entry blocker stands; and
  `back_emf_velocity_source: observer` means the flight law must supply ALL arm
  damping — never flown, ramp `back_emf_ff_scale` 0 -> 1.
**THE VALIDATION DID NOT PASS, AND IT DOES NOT PASS WITH AN IDEAL ARM EITHER.**
Eight flights, seven aborts. **`ideal` — an exact torque source, the plant
bit-identical to the 09-09 baseline that flew clean — aborted 2/2**, and the
ordering is not even monotonic in noise (`ideal` aborted EARLIEST). Same
signature everywhere: step peaks 290-325 mm (shiqi baseline 182-226), tilt p-p
18.7-20.1 deg, a growing ~0.6 Hz mode in q2/q3.
- **CAUSE, MEASURED: this box runs the sim at RTF 0.336 and the feedback is a
  STAIRCASE.** The plant makes a new state at 85 Hz wall (250 Hz sim x 0.34)
  while the estimator->controller chain runs on the WALL CLOCK at 235-250 Hz:
  **65.2% of consecutive odometry samples are byte-identical, each pose
  republished ~2.87 times = 1/RTF**, so two of every three control ticks act on
  stale data while the law's observers assume a 4 ms step. **NOTE THE
  DIRECTION** — the obvious guess is backwards: a slow sim makes transport
  delays SMALLER in simulated time; the damage is OVERSAMPLING a slow plant.
- **Headless does NOT fix it** (RTF 0.336 -> 0.351): Isaac is CPU-bound in
  PHYSICS at ~400%, GPU at 40%. `PEGASUS_HEADLESS` is now baked into the Isaac
  pane by `start_single_drone_x650.sh` (it never reached it before — the
  tmux-server-env trap), which is worth having but is not the remedy.
- Ruled out by measurement, each cheaper than the flight that would have found
  it: loop stalls (clean 250 Hz, p99 4.9 ms); the `bspline` planner backend (it
  came in with the L1 observer commit, so the 09-09 baseline had it too); arm
  pass-through fidelity (applied - commanded mean 0.0-2.0 mN.m); rendering.
- **The 09-09 baseline ran on `shiqi_machine`; every run here was on
  `fsc_lab_machine`. CROSS-MACHINE ABSOLUTE NUMBERS ARE NOT COMPARABLE** on this
  rig and must not be quoted as a baseline — `wb_l1_metrics` output carries no
  machine field, so record it beside the run. **CHECK RTF FIRST** on any new box
  before reading a whole-body result: the rig's attitude tune was retuned once
  already for transport latency (2026-08-22) and `M_r_d` was picked on a ~32 ms
  delay margin; a 2.87x feedback staircase is far outside all of it.
- **Re-run this validation on a box holding RTF near 1.** Nothing in the flight
  set implicates the alignment, and nothing in it validates the alignment either.

**J4 FOLLOW-UP: THE SIM WAS MODELLING j1/j4's CURRENT SENSOR, NOT THEIR MOTORS
(2026-09-11 evening, user request).** Pulled `fsc_open_manipulator` `612775a`
"joint4: friction_ff belongs to the unit, and its peaks are the sensor floor".
- **NEITHER CHANGED PARAMETER REACHES THE WHOLE-BODY FLIGHT PATH**, verified in
  code, not assumed: `friction_ff[3]` 21.2203 -> 7.7776 sits behind
  `auxiliary_terms_enabled() {return !passthrough_;}`, so it is identically zero
  while the law streams and acts only in the local PD+ hold; and
  `current_loop_bandwidth_hz_joints` is UNCHANGED at `[0.0, 1.5, 1.5, 0.0]` (the
  commit only records the measurement behind it: j4 trim ON halves the bias,
  mean 8.1 -> 3.4 mA, but more than doubles the variance, sd 7.8 -> 16.9, total
  rms 17.2 vs 11.2 OFF). The sim models no gearbox friction and its arm config
  carries no motor model, so nothing in the sim consumes either.
- **BUT THE FINDING THAT CAME WITH THEM INVALIDATED A NUMBER I HAD SHIPPED.**
  j1/j4's 11 mA is NOT a torque: at stalled samples, where `I = V/R` must hold
  exactly, **j4's measured current correlates 0.58 with its applied duty against
  0.97 on j2** (7.6 counts of residual against a signal of similar size; j4 runs
  at a few current counts, j2 at 100+). The arm repo's own verdict is "stop
  quoting a metric below its own noise floor" — and it explains three failed
  fixes there: the trim made j4 WORSE, the dither did nothing, and only ~20% of
  the "reversal excess" was ever friction. **This is exactly the ambiguity the
  morning's entry flagged as needing "a bench run at a NON-ZERO j1/j4 load", and
  it is now settled in the direction that says do not model it as torque.**
  `CURRENT_NOISE_A` `[0.0156, 0.0062, 0.0096, 0.0156] -> [0.0096, 0.0062,
  0.0096, 0.0096]` A: j1/j4 take j3's figure, the worse of the two joints whose
  residual IS real — restoring, for a better reason, what this model did before
  that morning. Conservative on purpose: a duty/R error scales with duty and
  j1/j4 command ~500x less of it. Torque noise is now `[22.0, 15.0, 23.7, 23.3]`
  mN.m, verified in the live Isaac banner.
- **KNOWN AND DELIBERATELY NOT MODELLED**: j4's remaining error is a
  VELOCITY-DEPENDENT BIAS (-2.6 mA at rest, +6..+11 moving, ~8 mA mean) that the
  arm repo attributes to a `back_emf_ke` ~20% high — slot 4's Ke was never
  measured in that slot, it is the j3 unit's value from another slot and day.
  That IS a real torque error on the flight path (the pass-through compensates
  back-EMF with it) and it is structurally a DROOP, not noise, so it would not
  be captured by raising any sigma. Left out because the 20% is an inference
  from an unmeasured constant; fixing it needs J4STEEP + the 80 mm bracket.
- **None of this changes the 8-flight campaign's conclusion**, which was never
  sensitive to the arm residual: `ideal`, zero noise on all four joints, aborted
  2/2. Re-measured this evening on an otherwise idle box: **RTF 0.347, odom
  stale-repeat 65.2%, each pose republished 2.88x** — identical to the morning,
  so the machine limit is reproducible and `fsc_lab_machine` still cannot
  validate this rig.
- Two traps hit again while doing it, both already in this file and both worth
  the reminder: `scripts/kill_stale_sim_processes.sh -y` KILLS ITS OWN INVOKING
  SHELL (exit 144), so anything launched in the same Bash call never starts —
  run it in a call of its own; and `pgrep -f "<node name>"` matched MY OWN SHELL
  and reported a controller that was not running.

**q4 UNIFIED AT ±120° ACROSS SIM AND HARDWARE (2026-09-12, user's call).** The
2026-08-23 widening stopped at the sim: planner / C++ / Isaac ran ±120 while every
HARDWARE arm config stayed ±90, and the shared planner is expected to use 90-120°
(that range is why q4 was widened — in-region feasibility 60% -> 98%). So a target
the planner accepted would drive q4 past the hardware controller's guard, where
`external_torque_controller` REPLACES the whole-body law's streamed torque with its
kp/kd pull-back — on the joint carrying EE heading — and the law is never told. The
C++ header's own comment already required these to be equal.
- **SIX definitions now, and they must stay equal** (was three): the three sim ones
  (`transition_planner.Q_MIN/Q_MAX`, `WbReferenceBuilder::kQMin/kQMax`,
  `torque_controller_isaac_aerial.yaml`) plus the xacro's `joint4_min/max_deg`
  (writes servo EEPROM) and `dxl_servo_ids.py`'s `EXPECTED_LIMITS[14]`. The sim
  three were ALREADY ±120 and were not touched; only the hardware side moved.
- **Changed**: 9 aerial arm configs (the flight one is
  `external_torque_controller_hardware_aerial_pwm.yaml`) ±1.570796 -> ±2.0943951
  rad; xacro `joint4_min/max_deg` -90/90 -> -120/120; `EXPECTED_LIMITS[14]`
  `(1024, 3072, -90, 90)` -> **`(683, 3413, -120, 120)`**. The counts follow the
  xacro's own ceil/floor conversion, and a regenerated URDF writes exactly
  683/3413 with joints 1-3 unchanged — that round-trip is the check.
- **NOT changed, on purpose**: the four NON-aerial configs (`position_controller_
  hardware.yaml`, `position_controller_gazebo.yaml`, `computed_torque_controller_
  gazebo.yaml`, `computed_torque_controller_hardware.yaml`). Those are the UPRIGHT
  ground rig, whose cabling is a different question; their software guard still
  clamps ±90 even though the EEPROM is now ±120, which is the safe direction.
- **THE EEPROM IS STALE UNTIL A POSITION-MODE BRING-UP RUNS.** The limits live in
  the servo, not the config, so joint4 still holds 1024/3072 until one bring-up
  writes them. `doc/Joint Limits.md`'s table is a record of a past hardware read
  and was deliberately left alone; `check_servo_limits.py` will read 683/3413 only
  after the rewrite. In torque mode this changes nothing (Mode 16 ignores EEPROM
  limits — the yaml's `min/max_position` is the only guard and it moved), but a
  position-mode session would still stop at 90°.
- Verified: all 10 aerial yamls parse and read ±120.0°; the URDF round-trip above;
  `dxl_servo_ids.py` parses and agrees with the xacro; the planner self-test passes
  (IK round-trip 3.2e-13 rad, defect 5.5e-08 m). The C++ `kQMin/kQMax` were
  untouched, so the parity fixtures cannot be affected. NOT flown.

**HARDWARE ADOPTS THE SIM ARM TOPOLOGY — WHOLE-BODY DIRECT ENTRY UNBLOCKED
(2026-09-12, user's call).** The DIRECT-entry gate requires `wb_arm_reference_topic`
fresh, and **it is evaluated while still in SAFETY, where `whole_body_planner` is
silent by design** (`_stream_tick` returns unless `_mode_direct`; its only arm-reference
publish is reached from inside that tick). So the SAFETY owner, `arm_planner`, must be
running for the gate to open — and the hardware bring-up never launched it. The builder
is explicit that nothing else counts: `seed()` "deliberately does NOT count as an
external reference". **In DIRECT the arm does track the whole-body planner's compatible
trajectory only** — that half was always right; the gap was purely the SAFETY-side
bootstrap.
- `external_torque_hardware.launch.py` now starts `arm_planner`, mirroring
  `torque_control_isaac.launch.py`; `external_torque_controller_hardware_aerial_pwm.yaml`
  gains `external_reference_topic`/`_timeout` and a `/**/arm_planner:` section.
  **THE PAIR IS THE TOPOLOGY — never set the key without launching the node**: the key
  is what puts the controller into pure tracking and un-advertises
  `~/target_joint_setpoint`, `~/sine_reference_trajectory`, `~/go_home`; `arm_planner`
  re-advertises all three AT THE SAME NAMES, so the GS and the flight node's
  `system_arm_go_home_service` are unaffected. Alone, the key leaves the arm with no
  reference at all (a stale stream FREEZES, it does not fall back).
- **The move profile is HARDWARE'S OWN, not the sim's**: 10 deg/s (`0.17453`, the
  2026-09-08 operational bound) against Isaac's 0.2 rad/s — the controller's former
  values verbatim, so the tuned motion is preserved.
- **`home_on_safety_revert: false` on hardware, and that is not a gap.** The flight yaml
  already points `system_arm_go_home_service` at this node's `go_home`, so the flight
  node commands the fold 1.0 s after a revert. Sim does the opposite (key true, service
  empty). **EXACTLY ONE of the two may be active** — both would command the arm twice on
  every revert.
- **`mode_topic` is ABSOLUTE and hard-codes `uav_0`** (the flight node is a flight-stack
  node at the vehicle namespace, this one lives under the arm package's). That matches
  both defaults. FLYING UNDER A DIFFERENT PREFIX MEANS EDITING THAT LINE — otherwise the
  node never sees DIRECT, keeps streaming its SAFETY reference, and fights the
  whole-body planner for the arm.
- **Stale refusal text fixed**: the gate said "controller smoothed arm reference never
  received", naming the pre-2026-09-05 topic. It now names the actual
  `wb_arm_reference_topic` and asks "is arm_planner running?" — the old wording sent an
  operator to `~/smoothed_reference_joint_trajectory`, which is telemetry and publishes
  happily either way.
- **Validated** (`test_arm_planner.py`, 12 checks, no Isaac/PX4): SIM config **12/12 ALL
  PASS**; the NEW HARDWARE config **11/12**, the single failure being exactly
  `folds HOME after a SAFETY revert` — the key deliberately false there. Same node, same
  binary, difference is only the configured fold owner. Also: launch file evaluates
  (`--show-args` rc=0), `arm_planner_node` is an installed executable, both yamls parse,
  `fsc_autopilot_ros2` rebuilds. **q4 re-verified ±120° across all six definitions after
  these edits.** NOT flown on hardware.
- Trap worth keeping: `kill` on the `ros2 run` WRAPPER leaves the node child alive. Two
  planners then publish the same topic and the loopback test fails on "reference is
  smooth" for reasons that have nothing to do with the config under test.

**GROUND TEST: THE WHOLE-BODY + L1 RIG WITH INERT PROPS (2026-09-12, user
request — "what happens if I switch to whole-body DIRECT on the ground and move
the arm").** A parallel of
`start_t650_aerial_manipulator_whole_body_L1_adaptive_direct_actuation_sitl.sh`
that is byte-identical in every respect except the rotor wrench. New launcher
`scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_ground_test_sitl.sh`;
the fsc_autopilot_ros2 side is **completely unchanged** — same stack script,
same node, same yaml, same gains — so this is purely a PLANT variant and the two
rigs cannot drift apart on the controller side.
- **`PEGASUS_PLANT_KM_SCALE` is NEW** (06 + `start_single_drone_x650.sh`,
  default 1.0 → every existing rig bit-identical), the yaw-channel twin of
  `PEGASUS_PLANT_KF_SCALE`. **BOTH are needed and this is easy to get wrong**:
  `Multirotor.update()` applies `k_f*w^2` as a per-rotor force and
  `k_m*w^2*rot_dir` as one summed yaw moment on `/body` — two different call
  sites. Measured at a yaw-differential command [600,600,400,400] rad/s on the
  shipped T650 constants: nominal 42.029 N / −0.98966 N·m; **`k_f=0` alone
  0.000 N but still −0.98966 N·m** (props that torque the airframe while
  lifting nothing — not an airframe); both zero 0.000 / 0.00000. The rotors
  still reach the commanded speed in every case, so the allocator, the ω map
  and the lag model are untouched; only the physics changes. 06 WARNS if only
  one is zero.
- **THE DIRECT GATE IS WHY THIS NEEDED MORE THAN AN ENV VAR.**
  `directEntryAllowed()` refuses unless `|state − outer_ref_| <= wb_gate_pos_m`
  (0.15 m). The flying rig satisfies that by taking off to 1.2 m and settling;
  on the ground `outer_ref_` is still its DEFAULT (0,0,0) against a vehicle
  seated at ~0.305 m, so DIRECT is refused for a reason unrelated to the test.
  New `application/robotic_arm/utils/wb_ground_reference_hold.py` supplies that
  one handshake: it republishes a reference AT THE VEHICLE'S OWN MEASURED POSE
  while the node reports SAFETY (datum-agnostic by construction — it never
  hard-codes 0.305), and **GOES SILENT IN DIRECT**, because a full-rate
  `position_controller/reference` stream drags the whole-body planner out of
  HOLD every tick (the 2026-08-23 re-plan oscillation). On a DIRECT→SAFETY edge
  it RE-CAPTURES, since `switchMode(kSafety)` has just re-seeded `outer_ref_`
  itself and republishing the stale pre-DIRECT pose would fight it. Runs in its
  own tmux window `ground`; the pane PROBES for an interpreter that imports
  both `rclpy` and `fsc_autopilot_ros2_msgs` (`FSC_GROUND_HOLD_PYTHON`
  overrides) — the bare-`python3`-is-Isaac's-3.11 trap, confirmed live here.
  **NEVER send a takeoff setpoint on this rig**: it opens a ~0.9 m error the
  gate then refuses and no thrust can close.
- **NOT a flight rig.** The controller believes it is flying, so nothing it
  reports about thrust, disturbance or altitude means what it means in the air.
  Read the ARM and the ATTITUDE.
**FLOWN 2026-09-12, 2 runs, NEITHER ABORTED (77 s and 359 s of DIRECT, two arm
legs each). NOTHING HAPPENS, and the reason is exact.** Driver
`utils/wb_ground_test_driver.py`, scorer + data
`docs/docs_aerial_manipulator/ground_test_20260912/`. Tables: Command.md 7.16.1.
- **THE LAW SITS IN A SELF-CONSISTENT HOVER EQUILIBRIUM AND NEVER FINDS OUT.**
  One number carries the whole result: **`u1 + d_hat_z` = 36.755 N, constant to
  9-12 mN across both runs**, against the controller's own
  `m*g = 3.746170*9.80665 = 36.737 N`; `corr(u1, -d_hat_z) = 0.9992`. The
  momentum observer measures `dp/dt`; seated, `dp/dt = 0` and the commanded
  `h + u` is already ~0, so THERE IS NO RESIDUAL TO FIND. The floor supplies
  the reaction and nothing in the loop can distinguish that from flight.
- **MY PREDICTION IN THIS ENTRY WAS WRONG AND IS CORRECTED.** It said `u1`
  would grow and `d_hat_z` would walk to its bound with `debug[88]` railing. It
  does not: `|d_hat_z|` reached **0.90 N of its 20 N bound** in six minutes and
  **0.00%** of samples were on a bound, 0.00% rotor saturation, 0.00% joint
  clamp.
- **BOTH INJECTIONS GO UNDETECTED.** The plant is 4.120787 kg (**40.41 N**) and
  the allocator believes a kf 15% high; the law commands **36.75 N** — the
  BELIEVED weight — and never learns otherwise. In FLIGHT the same config has
  `d_hat_z = -10.81 N` within seconds (7.15). That contrast is the cleanest
  statement of what a ground engagement is.
- **The vehicle is immobile to the MICRON**: x/y/z p-p **0.5-0.7 um**, tilt p-p
  **138 udeg**, slide 0.0 mm, over both runs. Compliant, not rigid — the tilt
  is non-zero — but the arm's 0.8 N.m cannot lever a 40 N vehicle off its legs.
  So the 20 deg TILT watchdog, which I expected to be the interesting one,
  never came close.
- **THE ARM IS UNAFFECTED and works normally.** Both compatible-trajectory legs
  planned (1.7-2.1 s) and executed (T = 6.8/6.9 s); EE settled **0.9-2.1 mm**,
  peak torque 0.82 of 3.0 N.m, 0% clamp, joints back at home. Motors ran
  mean 0.567 with the per-motor split [0.592, 0.544, 0.597, 0.535] — the
  standing PITCH trim of the forward arm CoM, front pair high, exactly as on
  the flying rigs.
- **THE SLOW DRIFT IS REAL BUT NOT CHARACTERISED — do not quote a time to the
  bound.** `d_hat_z` walks at **-9 mN/s (run A) vs -2.7 mN/s (run B)**, a 3x
  spread on one configuration, so the RATE is not reproducible. An exponential
  fit to run B settles at -2.24 N with tau = 660 s and beats a line (SSE 181 vs
  241), but **tau exceeds the 359 s window**, so "converging" is an
  extrapolation. Measured: 0.90 N in six minutes.
- **SAFETY-RELEVANT, and the reason not to do this with props on.** From the
  instant DIRECT is entered on the ground the law commands **~57% on all four
  motors and ~hover collective**, continuously, with no error signal telling it
  to stop. Here that is 9% under the true weight so it would not lift — but
  that margin is an accident of the mass injection, not a property of the law.
- **PX4 STAYS ARMED, so ONE LAUNCH IS ONE RUN.** After run A `arming_state = 2`
  though the vehicle never left the ground: the land detector needs low thrust
  and the law commands 57%. Full clean + relaunch between runs; the driver
  refuses to start against an armed vehicle.
- **A DEFECT IN THE HOLD NODE, FOUND ON THE FIRST RUN AND FIXED.** It latched
  the hold after N odometry samples, and **the estimator publishes (0, 0, 0)
  before it has a fix** — the count was satisfied instantly, the hold was taken
  at the origin, and the gate then read 0.305 m against its own 0.15 m limit,
  i.e. exactly the refusal the node exists to prevent. Its own log line is what
  caught it. It now **TRACKS the measured pose in SAFETY** rather than latching
  one: no startup ordering to get wrong, nothing to chase on a seated vehicle,
  and the SAFETY-revert re-seed becomes automatic. Re-verified on the relaunch.
- **NOT covered:** one configuration only (the shipped `_sim` plant); no
  attempt to move the BASE while seated (only EE legs); the drift beyond 6 min;
  and nothing about what a REAL vehicle does, since the whole point of the rig
  is that the props are inert. Commands: Command.md 7.16.
- **CONFIRMED ON HARDWARE the same day (user's report, qualitative).** Run on
  the real AM with the MOTORS TURNING AND NO PROPS FITTED, the end-effector
  "basically holds in a fixed setpoint" — the sim's result and, per the user,
  for the sim's reason: the drone does not move. The KINEMATIC half of 7.16.1
  therefore transfers: an arm-only compatible trajectory at the folded home
  asks for ~7 mm of system-CoM motion, the arm's own mass redistribution
  supplies all of it, and the base is never required to translate. **What does
  NOT transfer is the thrust channel.** The sim's tidy hover equilibrium
  (`u1 + d_hat_z` = the believed `m*g`, observer finding nothing) is a
  statement about a plant whose props produce zero; on hardware the same law
  commands **~57% on all four motors** from the instant DIRECT is entered, and
  that is live thrust the moment props are fitted. **Never enter whole-body
  DIRECT on a grounded vehicle with props on**, and treat "it held nicely on
  the bench" as evidence about the ARM only.
  NOT established by this: any hardware number (the report is qualitative), the
  heading channel (sim lags 21 deg mid-move), or a base-translating leg.

**THE PD+ CALIBRATION CORRECTIONS NOW ACT ON THE STREAMED u3 — ARM SIDE,
LAW AND OBSERVER UNTOUCHED (2026-09-14, user request).** The whole-body L1
rig (§7.15) streams `u3` in N·m to `fsc_open_manipulator`'s
`ExternalTorqueController`, whose shared base already ran the PD+ campaign's
command-level pipeline (report §2.2: mapping → friction → dither → clamp →
back-EMF + current trim) but gated the friction/dither/integral OFF on the
pass-through. The pass-through now treats the streamed torque as a second
NOMINAL law and applies the same bottom-level correction: three LIVE keys in
`external_torque_controller_hardware_aerial_pwm.yaml`, all default false so
every existing config is byte-identical — `passthrough_auxiliary_terms: true`
(friction + dither, reference-driven, exactly zero at a hold),
`passthrough_gravity_correction: true` (`(S−1)·g(q)`, VALID because
`wb_arm_model_check` puts the whole-body model's `g(q)` within 0.0016 N·m of
the URDF's, so the law's `g` IS `g_urdf`; −0.072 N·m on j2 = the handover step
the observer used to absorb), `passthrough_integral: false` (DELIBERATE: the
L1 arm channel + `wb_u3_internal_ff` already integrate the joint residual; a
second integrator would corrupt `ŵ`). The friction load term is the corrected
streamed torque (eq. 5's `µ|τ_load|`). Nothing in fsc_autopilot_ros2's law,
observer or node changed; the Isaac yaml leaves the keys unset on purpose
(the sim plant has no gearbox friction). `law_debug` 18 → 28 append-only
(`[18..21]` aux added, `[22..25]` gravity correction, `[26]/[27]` flags); the
"`[3]` = complement of `[0]`" invariant now holds only with the aux key false.
**Validated on mock hardware, 24/24** (`test/run_passthrough_corrections_test.sh`:
real controller on `fake_components/GenericSystem`, real yaml, every stage
scored against the yaml's own constants — κτ to 0.01 counts, friction to
0.02, dither rms = A/√2 exact, exactly 0 at rest). Found and fixed on the way:
`current_loop_bandwidth_hz_joints` was live in PD+ but not in the pass-through's
copy of the callback. Trap: mock hardware mirrors the written duty into Present
Current, so the trim must be OFF (scalar and list) in any loopback. The
hardware stack script's arm-side check prints the two keys either way and
reds out a true integral. Commands + watch list: Command.md §7.15.13.
**NOT flown.**

**GEARBOX FRICTION + A 5 % ARM GRAVITY MISMATCH IN THE PLANT, AND THE
COMPENSATION IN SIM — 3 FLIGHTS, STABLE (2026-09-14, user request off the
calibration report).** `servo_model.py` gained the report's eq. (5) gearbox
friction `[fc + µ|τ|]·tanh(q̇/w)` (fc [17.1, 31.4, 57.5, 52.4] mN·m = the
yaml's duty counts / κ_pwm, µ [0, .246, .161, 0], w 0.015), applied on the
MEASURED velocity downstream of the electromagnetic torque and
**momentum-clamped** to `I_min·|q̇|/dt` (armature 0.02) so an explicit Coulomb
term at 250 Hz can never reverse a joint (self-test: spin-down at the 3 N·m
clamp never reverses). `06` gained `PEGASUS_ARM_FRICTION_SCALE` (0 = off, the
old plant; 1.05 = 5 % more than compensated), `_FRICTION_WIDTH`, and
`PEGASUS_ARM_MASS_SCALE` (arm links AND inertias ×, every model nominal —
distinct from `sim_plant_mass_scale`, which moves the BODY only and leaves the
arm exact); both WB launchers forward `sim_arm_friction_scale/_width` and
`sim_arm_mass_scale` from the `_sim` yamls (both twins now ship 1.05/0.015/1.05)
and pass `passthrough_corrections:=true|false` to `torque_control_isaac.launch.py`
so the arm-side friction feed-forward (now in `torque_controller_isaac_aerial.yaml`,
same fc/µ in N·m, `passthrough_auxiliary_terms: true`, dither 0) is on iff the
plant has friction (`PEGASUS_ARM_FRICTION_COMP=0` = the uncompensated A/B). No
back-EMF, on purpose — the current loop owns it. **Flown 3× (A mismatch,
B matched, C uncompensated) vs the ideal-arm 16 s-hold baseline: all complete,
0 % clamp, 0 % saturation, peak τ ≤ 1.06 of 3 N·m, tilt ≤ 3.6° p-p — it
stabilizes.** The 5 % itself is cheap (A vs B: j3 err 0.6→2.6°, CoM wander
16→25 mm std; `d̂_z` finds the 0.31 N of extra arm weight, `τ_q2` the +5 %).
**`late_com` is a WANDER not an offset** (per-axis means 1–7 mm; the std of
`e_com,y` is 16–25 mm at ~0.1 Hz; EE error 1–2 mm std). **Compensation buys
the standing accuracy (C: joints park short 1.5/3.0°, phantom bias −0.28 N,
`late_com` 36 mm) and costs a metastable 0.88 Hz j2-torque cycle at 0.105 N·m
rms** = the attitude loop's `√(k_R/I)` mode (2026-08-23): the friction
feed-forward is exactly zero at a hold (streamed `q_d` bit-constant — checked),
so it cannot drive it there; it RE-EXCITES it at each leg end (FF off on `q̇_d`,
plant friction off on the lagging `q̇`); in C the same mode is kicked by the
EE leg and DECAYS (97→53→13→6 %); in A/B it locks in. Damping is
model-dependent (the clamp is an effective stick below ~0.04 rad/s); the mode is
the airframe's. Per-hold scorer: `arm_mismatch_20260914/hold_chatter.py`
(`wb_l1_metrics`' `late_com` cannot tell offset from wander). Not implicated:
`wb_l1_lc_var_q`. Not tried: repeat C, damp the 0.88 Hz mode (k_w/M_r_d), a
rate-limited FF switch-off. Tables: Command.md §7.15.14. Three traps hit
again and worth the reminder: `pkill -f` kills the invoking shell when the
SAME command line mentions the pattern in plain text — even inside a heredoc;
`tmux capture-pane` wraps at the pane width, so multi-word greps need `-J`; a
metric that averages |e| reads an oscillation as a standing error.

**CONFIG A MADE THE STANDING §7.15 SIM PLANT (2026-09-14, user request — "use A
for the current simulation ... to simulate this minor mismatching case for
robustness validation before the flight test").** No code change was needed:
both `_sim` twins already shipped `sim_arm_friction_scale: 1.05` /
`_width: 0.015` / `sim_arm_mass_scale: 1.05`, the launcher derives
`passthrough_corrections:=true` from friction > 0, and `wb_l1_set_gains.py`
preserves the new keys (it re.subs only the observer keys it knows) — all three
VERIFIED, not assumed. What was missing was DISCOVERABILITY, now fixed in the
three places that describe the plant: both yamls' section-1 FACTOR table gained
the two rows and now say SIX factors, not four; the L1 isaacsim stack script's
header listed a plant that was stale twice over (it still promised the
2026-09-09-deleted back-EMF droop and knew nothing of the friction); and
Command.md §7.15 now opens with the change, with a "CONFIRM THE PLANT TOO"
banner check beside the observer one in §7.15.1. **The consequence to carry:
every flight number in §7.15.4–§7.15.12 (incl. the C++-planner and EE-trajectory
flights, 2026-09-14/15) was flown on a FRICTIONLESS arm and is NOT
comparable with a run from here on** — the frictionless baseline is
`posture_ablation_20260909/l1_mission_kx32.npz`, and
`PEGASUS_ARM_FRICTION_SCALE=0 PEGASUS_ARM_MASS_SCALE=1.0` reproduces that plant
for an A/B. The GMO twin (§7.14) carries the same values on purpose: the repo's
own rule is that the two sim yamls must not describe different plants.

**THE FOUR-DIMENSIONAL ATTRIBUTION — THE NOTE'S SEPTEMBER REVISION, IMPLEMENTED,
PARITY-LOCKED AND FLOWN AGAINST THE 6-D DESIGN ON THE SAME PLANT (2026-09-16/17,
user request).** `disturbance_observer_draft.tex` ("the four-dimensional interaction
wrench"; the 6-D design is archived in `_backup.tex`) replaces the metric +
projector + identifier of the 2026-09-06 observer with an ALGEBRAIC attribution on
the JOINT ROWS: `w_e = S_e^T F + k` with `k` joint-invisible, `[w_Σ]_q = w_q +
J_yq^T F` (a platform wrench has `[w]_q = 0` EXACTLY), `F̂ = J_yq^-T([ŵ_Σ]_q − ŵ_q)`,
`ŵ_q` a slow trim (`ω_q`) in free flight and frozen in contact, `F̂_y = (1−χ) C_x F̂`
(ZERO in free flight by construction), `u3`'s feedforward `d_int^f = C_x T^-T ŵ_int`.
Layer 1 untouched. Everything additive: the `four_d` branch of `wb_l1_observer.cpp`
+ `l1_observer.py` (self-test section 6: exact attribution in contact to 2e-15 where
the 6-D identifier had 0.59 N of error), keys `wb_l1_four_d / omega_q / contact /
collision_threshold_n / omega_x_{t,r,q}`, debug [97..100] raw F̂ / [101..104] ŵ_q /
[105] χ, a 4D banner + per-second watch line, `WbL1ParityTest` 9 rollouts, **same
executable** — the yaml `..._whole_body_l1_4d_..._sim.yaml` is the switch; new
stack script `start_whole_body_l1_4d_..._stack.sh` and Pegasus launcher
`start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh`,
which READS `wb_l1_four_d` OFF THE RUNNING NODE (`ros2 param get`) and refuses the
6-D stack, since a process-name gate cannot tell them apart. Tooling:
`wb_l1_tune_cycle.sh l1_4d`, `wb_l1_set_gains.py --four-d`, `phantom_Fraw_*` in
`wb_l1_metrics.py`, `|Fraw|` in `wb_compare_metrics.py`; campaign
`docs/docs_aerial_manipulator/l1_4d_20260916/` (`run_4d.sh`, `summarize_4d.py`,
`ripple_4d.py`, `plot_4d.py`). Commands + every table: Command.md §7.17.
- **THE ONE TUNING KNOB IS `ω_x`, AND THE ANSWER IS 0.25 rad/s — the note's own scalar
  form, shipped.** In free flight the 4-D law differs from the flown 6-D one in exactly
  two places: `F̂_y ≡ 0`, and `u3`'s feedforward is `C_x` of the WHOLE residual at ONE
  bandwidth where the 6-D fed the per-group `d_f` (2/0.5/0.5). The 6-D yaml's `ω_x = 20`
  (a wrench-reading smoother there) is NOT a starting point. Sweep, 10 flights, none
  aborted, 0 % clamp/saturation: 6 → 339 mN·m of j2 ripple at the airframe's 0.94 Hz
  attitude mode (§7.15.14's metastable cycle) and 14° tilt p-p; 2 → 259; 0.5 → 196;
  **0.25 → 5.0 mN·m, tilt ripple 0.075°, below BOTH 6-D baselines (7.8 / 70), repeated
  3/3.** NOT a bandwidth mismatch: per-block `Ω_x` at the 6-D's own 2/0.5/0.5 still
  rippled 167, 2/0.25/0.25 rippled 182 — the TRANSLATIONAL block of the feedforward at
  2 rad/s excites the mode and the boundary is between 0.25 and 0.5. What the 6-D had
  and the 4-D removes on purpose is the rendered `−(Λ_y M_y⁻¹ − I) F̂_y`, a ~0.3–0.4 N
  force off a 20 rad/s filter acting as fast momentum feedback into `u3`, i.e. damping
  that mode by accident.
- **WHAT THE ATTRIBUTION BUYS ON THE CONFIG-A PLANT, SCORED PER STATE** (friction
  ×1.05 + arm mass ×1.05 are JOINT-ROW disturbances, the class a 6-D metric can never
  keep off the task). **EVERY LEG IS A COMPATIBLE TRAJECTORY**, steps included, so
  every state has a reference; `traj_errors.py` scores each channel in its own unit,
  peak/rms/settled-rms per leg (`traj_errors.txt`/`.csv`). Pooled over the mission,
  6-D → 4-D settled: CoM position 30 → 13 mm (2.3×), CoM velocity 22 → 17 mm/s,
  attitude 1.29 → 0.50° (2.6×), EE POSITION 5.5 → 3.0 mm (1.8×), **EE HEADING
  9.57 → 1.33° (7.2×)**, **joints 9.68 → 1.88° (5.1×)**. **THE PEAKS ARE IDENTICAL ON
  EVERY CHANNEL** (247 vs 241 mm, 9.4 vs 9.3°, 32 vs 35 mm, 26.3 vs 26.9°) — peak is
  the planner's transition, which both designs fly the same way; the attribution
  changes only what is left after arrival, and it changes ORIENTATION far more than
  translation. A translation-only metric calls these two designs nearly equivalent.
  **THE 6-D HEADING ERROR IS A SLOW OSCILLATION, NOT AN OFFSET**: 0 → 13° p-p at
  **0.06–0.11 Hz**, continuous through hover and the settled window of every leg
  (4-D: 0.8° p-p); the joint column mirrors it because the EE heading is carried by
  the `(q1,q4)` pair. The 4-D's one HIGHER peak, 17–25° on the three trajectory legs,
  is tracking lag on a commanded 60° heading sweep that decays to 0.9–2.4° inside the
  hold, where the 6-D's comparable peak never decays; both lag it because
  `K_y,ψ = 0.3` is deliberately soft. Entry transient and `d̂_z` (−11.18 N) identical
  (same Layer 1). Raw F̂ (collision reading, gated) 0.15–0.24 N late with `ŵ_q` at
  0.05–0.13 N·m still converging at `ω_q = 0.2`; instantaneously it is a 250 Hz
  momentum difference through `J_yq⁻ᵀ` and noisy (~1 N std) — a collision threshold
  sits on the filtered value.
- **A UNIT DEFECT IN THE PER-LEG TOOL, FOUND 2026-09-17 AND FIXED.**
  `wb_compare_metrics.py` reported an "EE error" that was `norm(e_y)` over ALL FOUR
  components — three in METRES plus `sin(heading error)`, DIMENSIONLESS. An 8.7°
  heading error contributes 0.151 to that norm and was printed as **"151 mm" of EE
  position error**; the first published version of the §7.17.4 table and the artifact
  carried 110–233 mm "EE" figures that were mostly heading. **EE position was never
  more than ~7 mm settled on either design.** The tool now reports `peakEE`/`settEE`
  (mm) and `pkHead`/`setHead` (deg) separately, and the campaign's numbers are
  recomputed per channel by `traj_errors.py` and cross-checked against it. **Never
  norm a task vector whose components carry different units**; and note `‖e_R‖` and
  `e_y[3]` are SINES, inverted through `asin`, not radians. Every analysis window
  must also be masked to DIRECT: the last leg runs past the abort into the SAFETY
  descent, where the node publishes the shorter debug prefix and the recorder
  zero-pads it — which reads as a 0 mm CoM "error" and a 65° joint "error".
- **Traps hit:** `wb_l1_set_gains.py` wrote `omega_x_t=0` as an INTEGER and rclcpp
  refused the double at startup — the node died before its first log line and the
  cycle reported "never came up" (the setter now writes a float when the file holds
  one); a running `run_4d.sh` must NOT be edited (bash reads scripts by byte offset —
  revert to the original bytes if you did); the campaign's EXIT trap restores the
  yaml from a backup taken at START, so a mid-campaign yaml edit needs the backup
  refreshed or is lost; `setsid nohup … &`'s "Done" is the shell reaping the wrapper,
  not the campaign; `pgrep -f run_4d.sh` matches the checking shell itself; the
  session's scratchpad does not survive a restart — keep drafts in the repo.
- **NOT covered:** contact (the `χ = 0` branch is parity-locked and self-tested, never
  flown — no contact exists in Isaac); `ω_q` as a flight knob; hardware. `wb_l1_omega_x`
  on the 6-D yaml is unchanged at 20 (there it filters only the wrench reading).

**THE 4-D RIG ON THE C++ TRAJECTORY PLANNER, THE THREE-REPO SYNC, AND A HARDWARE
PAIR (2026-09-17, user request).** Local work in all three repos was committed and
the remotes merged in (nothing pushed): Pegasus `dev_robotic_arm`,
fsc_autopilot_ros2 `dev_CCM` (the Python planner deleted, `ee_traj_*` keys), and
`fsc_trajectory_planner` cloned at **`~/ros2_ws/src/fsc_trajectory_planner`** on
this desktop. The incoming notes say `~/Workspaces/fsc_autopilot_ws`; that is the
other machine. Full record: Command.md §7.17.5.
- **The arm GS's circle / figure-8 "EE trajectory" tab was ALREADY in the local
  arm repo** (`omx-torque-control`, Longhao's 5d22f50/537a5c0/2abcd3d, and in the
  Sep-14 GS build). There is **no `dev_robotic_arm` branch** on either arm remote
  (`origin` = Gao907/fsc_open_manipulator; a separate FSC-Lab/fsc_open_manipulator
  exists and its `omx-torque-control` is BEHIND the local checkout).
- **Command.md renumbering:** the incoming §7.15.11 (C++ planner flight) and
  §7.15.12 (EE circle) kept their numbers; the local PD+ section is now
  **§7.15.13** and the friction-mismatch flights **§7.15.14**.
- **4-D sim pair re-derived, not hand-edited:** merged 6-D twin + the saved
  6-D->4-D patch, so the 4-D yaml carries `/**/whole_body_trajectory_planner`
  (bspline, `ee_traj_*`) and the 4-D stack launches the C++ node. The planner
  LINKS `wb_law`, which compiles `wb_l1_observer.cpp`: rebuild it after any
  law/observer change (`colcon build --packages-select fsc_autopilot_ros2
  fsc_trajectory_planner`). Verified: WbParityTest 4/4, WbReferenceBuilderTest
  2/2, WbL1ParityTest 4/4, flat-planner parity, planner gtests 14/14, and both of
  the planner's rig loopbacks (transition; EE circle, 8890 samples within 3 mm of
  the radius) with `params_file:=` the 4-D sim yaml.
- **`wb_l1_tune_cycle.sh` gained `WB_L1_MISSION=ee_circle|ee_figure8`** (default
  `standard`, unchanged). It runs the planner package's installed
  `ee_trajectory_sim_driver.py` on any of gmo/l1/l1_4d; that package's own
  `ee_trajectory_sim_cycle.sh` hard-codes the 6-D stack and the other machine's
  paths. The EE driver's npz has no `wb_control_debug`; score it with
  `docs/docs_aerial_manipulator/l1_4d_planner_20260917/ee_run_score.py` (the
  §7.15.12 metrics).
- **FLOWN in Isaac on this desktop, both EE shapes on the 4-D rig** (config-A
  plant), each end to end with no abort, refusal or INFEASIBLE:
  - circle (s 0.905/1.131): raw EE error 124/178 mm, lag 1.05 s, residual
    55 mm, radius 0.450 of 0.500 m.
  - figure-8 (s 0.286/0.358): 50/106 mm, lag 1.37 s, residual 27 mm, extent
    0.970 × 0.464 of 1.0 × 0.5 m. **The first recorded figure-8 flight on any
    rig.**

  The 4-D watch line showed `chi=free` and the J2 trim `w_hat_q` at about
  −0.1 N·m. **NOT comparable with §7.15.12's 6-D circle**: that flight ran on
  another machine at RTF 0.34, on a frictionless arm. The matched 6-D run is
  `WB_L1_MISSION=ee_circle ... wb_l1_tune_cycle.sh l1 <tag>`. Data:
  `docs/docs_aerial_manipulator/l1_4d_planner_20260917/`.
- **EE-run arm split retuned 2026-09-17 (user): q2 30 +- 10 deg at a 60 deg
  fold, 48 s period**, both 4-D yamls (the 6-D twins keep 80 / 40 +- 8). **The
  fold and the centre are one choice** — at rest `q3 = fold - q2`, so 30 +- 10
  at the old 80 deg fold is REFUSED (q3 -> 60 vs the +50 stop; the planner names
  the only admissible centre, 40). At fold 60 both joints sit in [20, 40], 10 deg
  of margin. The period must DIVIDE `laps * lap_time` or the run does not end on
  its start pose. Peak joint rate is `amp * 2pi * s / period` and the SLIDER sets
  the worst case: 1.79 deg/s at the circle's s_max, 0.62 at the figure-8's.
  Costs: EE 0.135 m below the base (was 0.061), sigma_nd 0.306 -> 0.258, peak
  joint torque 0.82 of 3.0 N.m; s_max unchanged (it is the yaw rate). Check any
  `ee_traj_*` edit with `l1_4d_planner_20260917/ee_plan_probe.py` before flying.
  NOT re-flown at this split.
- **SIM AND EXPERIMENT PLAN IDENTICALLY (2026-09-17, user request).** The
  `planner` backend was the ONLY difference between each whole-body yaml and its
  `_sim` twin (all three pairs, 33 keys, one diff); all three hardware files now
  select **`bspline`**, so the planner sections are byte-identical. Precondition
  met: bspline flew the 2026-09-14 §7.15.1 sim mission 10/10 and planned every
  Go-to-start of the 09-17 EE flights, and it enforces rotor force + joint torque
  which straight_line does NOT (its refusals are a superset). Verified on the 4-D
  hardware yaml through the planner's rig loopback. **`straight_line` is KEPT**:
  one key away, the only backend with HARDWARE time (0912), the parity anchor to
  `utils_planner/transition_planner.py`, and the only one that holds the EE to a
  straight line — an operational property near a net/person/object. Delete it
  only after bspline has flown on hardware.
- **4-D HARDWARE PAIR, never flown:**
  `params_..._whole_body_l1_4d_direct_actuation_t650.yaml` (the 6-D hardware yaml
  + the identical parameter delta, identity `AM-T650-WB-L1-4D-HW`) and
  `fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh`,
  with a FATAL design gate (`wb_l1_four_d: true`, not overridable). **It carries
  the 6-D hardware file's 2026-09-12 experiment kf 4.260431e-05 on purpose**;
  restore the bench value in both files together. Both L1 hardware launchers
  now expect `current_loop_bandwidth_hz_joints [0.0, 1.5, 1.5, 0.2]` (the arm's
  TEMPORARY j4 trim); before, they printed a red MISMATCH on every start.
