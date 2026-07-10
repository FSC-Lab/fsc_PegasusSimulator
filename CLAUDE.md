# PegasusSimulator — Developer Checklist

## Repository layout

This is the FSC Lab fork of [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) (NVIDIA Isaac Sim framework for PX4/ArduPilot multirotor simulation).

- `extensions/pegasus.simulator/` — core simulator (upstream): vehicles, backends, sensors, dynamics, thrusters, graphs, UI
  - `pegasus/simulator/logic/vehicles/` — vehicle base class (`vehicle.py`), `multirotor.py`, and concrete models under `multirotors/` (e.g. `iris.py`, `ideal_quadrotor.py`)
  - `pegasus/simulator/logic/backends/` — control/telemetry backends (PX4 mavlink, ArduPilot mavlink, ROS 2)
- `extensions/fsc_aerial_manipulation/` — FSC Lab's own library: `aerodynamics/`, `constraints/`, `robotic_arm/`, `rotorcraft/`, `slung_load/`, `utils/`
- `application/` — runnable example scripts, one subfolder per scenario: `ideal_quadrotor/`, `px4_base/`, `slungload/` (`aerial_manipulation/` and `VTOL/` exist but are currently empty placeholders)
- `examples/`, `docs/`, `scripts/`, `tools/` — upstream examples, docs, and Isaac Sim tooling

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

**Remaining cleanup (not yet done):**
- Remove the throttled debug `print(...)` in `update_sim_state` (`cable_winch_backend_utils.py`) added during step 5.
- Revisit `lower_limit`/`upper_limit` on the prismatic joint (`application/slungload/02_px4_single_drone_payload_variable_length_cable.py`, currently symmetric `±max_cable_extension` as a diagnostic) — original intent was "can't retract past flush" (`lower_limit=0.0`), but exactly 0 may need a small safety margin instead. Now that force response is confirmed working, this can be revisited without the diagnostic confounding it.

**Next steps (in order — see the plan file for full detail):**
1. Do the cleanup above.
2. Part 2 (in `fsc_autopilot_ws`, separate repo): scaffold new C++/ament_cmake package `AK40-10-issac-sim-emulator`, cloning the layout of `isaacsim_optitrack_ros2_emulator`. Node executable/name must be `ak_motor_cable_control_node` (drop-in for the real driver). Port `AkMotorCableControlNode` (`AK40-10-ROS2-Bridge/src/ak_motor_cable_control_node.cpp`) with full parity (SPEED/TORQUE/POSITION modes, external-mode 3-state machine, both watchdogs, torque clamping) — only its CAN I/O layer is replaced with the two new topics above.
3. Validate requirement #5 (`ros2 launch ak_motor_driver cable_torque_ctrl.launch.py`, unmodified, controls the sim cable in external mode) and #6 (the `ak_motor_ground_station` GUI's Stop/Emergency-Stop/Exit-External interrupts correctly override the emulator).

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
