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
a validated MATLAB whole-body geometric+impedance controller. **Pure ROS 2, no PX4 SITL** (a
different architecture from every scenario under `application/slungload/`, which are all PX4
SITL + mavlink).

- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/x650_vehicle.py` —
  `VehicleMod(Vehicle)`, thin subclass adding `usd_prim_path`/configurable `body_path` for a
  custom USD asset.
- `.../robotic_arm/x650_multirotor.py` — `MultirotorMod(BaseMultirotor, VehicleMod)`, adds
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
- `.../robotic_arm/postprocessor.py` (1227 lines) — **not** a runtime component. A one-shot
  OnShape→USD asset-cleanup pipeline (reparent/rename/realign frames/set mass+inertia/add
  colliders/apply joint drives/export), meant to be pasted into Isaac Sim's Script Editor after
  CAD import, not imported by anything else.
- `.../robotic_arm/__init__.py` is empty — import submodules directly
  (`fsc_aerial_manipulation.robotic_arm.x650_vehicle`, etc.), not via the package.
- `application/robotic_arm/01_aerial_manipulator_hover.py` (761 lines) — the working demo
  entrypoint: DC-interface joint control, ROS2 pub/sub for torque/rotor commands, arm
  position-hold→effort-control handoff.
- `application/robotic_arm/22_x650_with_manip_minimal.py` (109 lines) — **currently broken as
  committed**: inserts `application/robotic_arm/utils/` onto `sys.path` to import
  `x650_rotorcraft_utils`, but that `utils/` directory doesn't exist in this repo (line 30), and
  it hardcodes an absolute USD path under `/home/fsc-jupiter/...` (line 73) rather than
  resolving relative to `FSC_PEGASUS_ROOT`. Don't use as a template until fixed — `01` is the
  correct reference.
- `scripts/start_aerial_manipulator.sh` + `scripts/config/shiqi_machine.conf` — follows the
  same `common_config.sh`/`terminal_utils.sh`/per-machine-config convention as the slung-load
  launch scripts. 2-pane tmux (`aerial_manip` session): pane 0 runs the Isaac Sim demo, pane 1
  sources ROS 2 and runs `controller.py` under `--ros-args -r __ns:=/uav_0` after an 8s delay
  (long enough for Isaac to spawn the vehicle first). Both panes tee to `/tmp/aerial_manip_isaac.log`
  / `/tmp/aerial_manip_controller.log` for post-mortem grepping. Note the script's own header
  comment still references an old filename (`aerial_manipulator_ee_impedance.py`) that no longer
  exists — the actual entrypoint it launches is `01_aerial_manipulator_hover.py`; the comment is
  just stale, not a real path.

**Known checklist deviations, not yet fixed** (see "Checklist for adding a new vehicle model"
below — `x650_vehicle.py`/`x650_multirotor.py` themselves are compliant, only `controller.py`
deviates):
- `controller.py:774` — ROS2 node name is hardcoded (`"matlab_aerial_manipulator_controller"`),
  not parameterized by vehicle/uav id. Fine for today's single-vehicle demo; will collide if two
  arm instances ever run in the same ROS2 domain.
- `controller.py:961` — `rclpy.init(args=args)` is called bare, not wrapped in try/except per
  this repo's "already initialised is not an error" rule; will raise if `rclpy` is already
  initialized by a host process.

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
- `scripts/start_single_drone_sitl_payload_variable_cable.sh` (new) — launch script for this scenario, same tmux/PX4 pattern as `start_single_drone_sitl_payload.sh`

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
