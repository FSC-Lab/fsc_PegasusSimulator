# AGENTS.md

## Scope

These instructions apply to the entire repository. This is the FSC Lab fork of
Pegasus Simulator, built on NVIDIA Isaac Sim for PX4, ArduPilot, Python, and ROS 2
multirotor simulation. Prefer a focused change in the FSC extension over modifying
upstream Pegasus code when either approach is possible.

`CLAUDE.md` contains detailed FSC development history, calibration rationale, and
known limitations. Consult the relevant section before changing X650, slung-load,
variable-cable, or aerial-manipulator behavior; do not copy its historical notes
into source code unless they explain a current invariant.

## Repository map

- `extensions/pegasus.simulator/`: upstream simulator extension. Core vehicle,
  backend, sensor, dynamics, thrust, graph, and UI code lives under
  `pegasus/simulator/logic/`.
- `extensions/fsc_aerial_manipulation/`: FSC-owned Python package. Put FSC-specific
  aerodynamics, constraints, rotorcraft, slung-load, robotic-arm, and shared utility
  code here.
- `application/`: FSC runnable scenarios grouped by `px4_base/`, `slungload/`,
  `robotic_arm/`, and `ideal_quadrotor/`.
- `examples/`: upstream standalone examples and tutorials.
- `scripts/`: shared machine configuration, parameter helpers, visualization,
  cleanup, and launch orchestration support.
- `scripts/indoor_sim/`: OptiTrack/ROS 2, aerial-manipulator, slung-load,
  multi-drone, and direct-actuator indoor launchers.
- `scripts/outdoor_sim/`: standard PX4-owned Iris and calibrated X650 outdoor
  launchers.
- `scripts/config/`: per-machine `.conf` files defining `PX4_DIR`, `ISAAC_PY`, and
  `FSC_PEGASUS_ROOT`.
- `docs/`: Sphinx documentation and FSC lab guides. FSC-specific change history is
  in `docs/fsc_log/CHANGELOG_FSC.md`.

## Environment and setup

- The supported baseline is Isaac Sim 5.1.0, Python 3.11 for the FSC package, and
  PX4-Autopilot 1.14.3 on Ubuntu.
- Isaac Sim modules (`isaacsim`, `omni`, `carb`, `pxr`) are not available in a
  normal system Python. Run simulator applications with `isaac_run <script.py>` or
  the machine's `ISAAC_PY` wrapper.
- Set `ISAACSIM_PATH` before installing either extension. The canonical setup is in
  `docs/source/setup/installation.rst`.
- For extension-mode development, link the repository to Isaac Sim with:

  ```bash
  ./link_app.sh --path "$ISAACSIM_PATH"
  ```

- Run a configured PX4/Isaac scenario as shown below. The argument may include or
  omit `.conf`:

  ```bash
  ./scripts/outdoor_sim/start_single_drone_sitl.sh longhao_machine
  ```

- New launch scripts should live in `scripts/indoor_sim/` or
  `scripts/outdoor_sim/`, source both parent helpers `scripts/common_config.sh`
  and `scripts/terminal_utils.sh`, accept one machine config, use paths relative
  to `FSC_PEGASUS_ROOT`, validate entrypoints, and clean up sibling tmux panes
  when one process exits. Do not introduce user-specific absolute paths.

## Standalone application rules

- Import `SimulationApp` and instantiate it immediately, before importing other
  `omni` or Pegasus modules. Isaac Sim can crash if extension imports precede app
  startup. Follow `application/px4_base/01_px4_single_drone.py`.
- Build the world through `PegasusInterface`, reset it after adding vehicles and
  articulations, then play the timeline and step the world.
- Stop the timeline and close `SimulationApp` during normal cleanup.
- Resolve packaged assets relative to `__file__`; never hardcode a developer's home
  directory.

## Architecture invariants

### Vehicle and configuration objects

- Config classes contain data only. Do not create ROS nodes, sockets, subprocesses,
  or other live resources in a config constructor.
- Avoid shared mutable defaults. Use `config=None`, construct the config inside the
  vehicle constructor, and create default backends only after resolving `None`.
- Apply the same rule to backend, sensor, graphical-sensor, and graph lists.
- For a single-link floating articulation authored in code, apply
  `ArticulationRootAPI` and `RigidBodyAPI` to the same prim. Override state reads to
  use `self._stage_prefix`, and use `body_part=""` for force/torque application.

### Backend ownership

- Backend order matters: `config.backends[0]` is the primary motor-command source.
- In PX4 scenarios, keep PX4 first. A ROS 2 backend used only for telemetry must set
  `sub_control=False` so it cannot compete for motor authority.
- A ROS 2 backend should own exactly one uniquely named node per vehicle. Include
  the vehicle ID in the node name, tolerate an already-initialized `rclpy`, and
  match counterpart QoS (sensor streams generally use
  `qos_profile_sensor_data`/BEST_EFFORT).
- Keep vehicle IDs, MAVLink ports, ROS namespaces, and PX4 instances consistent in
  multi-vehicle scenarios.

### Scenario boundaries

- Treat these aerial-manipulator paths as one unit:
  `application/robotic_arm/`,
  `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/`, and
  `scripts/indoor_sim/start_aerial_manipulator.sh`. Its default `direct` mode is pure ROS 2;
  its optional `px4-offboard` mode routes normalized `ActuatorMotors` through
  PX4's arm/saturation gate and back to Isaac over HIL.
- Use `application/robotic_arm/01_aerial_manipulator_hover.py` as the working arm
  reference. `22_x650_with_manip_minimal.py` contains stale/missing paths and is not
  a valid template.
- The bare X650 uses stock Pegasus `Multirotor` with corrected `x650_new.usd`;
  its spawn helper overrides the asset's body mass/inertia so the established
  3.5 kg total and CAD-derived inertia remain unchanged. The arm-equipped
  X650 uses custom `VehicleMod`/`MultirotorMod` with `AM_realign.usda`. Do not swap
  their helpers or assets. The legacy bare-X650 USD has the wrong frame
  direction and must never be selected, including through configuration or an
  environment override.
- X650 calibration constants have physical provenance. Before changing mass,
  inertia, rotor speed mapping, thrust/torque coefficients, rotor lag, or PX4 gains,
  read the corresponding sections of `CLAUDE.md` and the reports under
  `docs/propeller_testing/`. Preserve the disarmed zero-speed behavior and the exact
  discrete rotor-lag update.

### X650 direct-actuator validation workflow

The current ROS 2 direct-actuator work is deliberately staged. Do not begin by
tuning the free-flight controller when motor order, signs, or torque response are
in doubt.

1. Run `scripts/indoor_sim/start_x650_pinned_direct_actuator_test.sh`. It launches PX4,
   Micro XRCE-DDS Agent, the temporary external ROS pulse node, and
   `application/px4_base/04_x650_pinned_direct_actuator_test.py`.
2. The pinned fixture clamps translation in a physics callback but never resets
   orientation or angular velocity. Spherical and nominal translation-only D6
   joints both locked this articulation's attitude in live tests and must not be
   reintroduced for the torque fixture.
3. Use the calibrated static MN4014 + 15x5 model without rotor delay in this
   diagnostic only. Compare the force-at-rotor predicted torque against measured
   angular acceleration in `/tmp/x650_pinned_torque.csv`.
4. Only after all six roll, pitch, and yaw pulses have matching torque/acceleration
   signs, run `scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh` for free flight. The
   hover plant uses the real `10.51 1/s` rotor lag.

The pulse node is external at
`/home/longhao/source/fsc_autopilot_ws/src/x650_direct_actuator_test_node.py`.
It prestreams zero, requests OFFBOARD, confirms OFFBOARD, requests arming,
confirms arming, and only then sends nonzero motors. After the sequence it holds
zero and uses PX4's force-disarm code because a rotating pinned vehicle may still
be classified as airborne.

For the free-flight test, `scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh` launches
the external `apl20_ros/autopilot_node` with
`/home/longhao/source/fsc_autopilot_ws/src/apl20/apl20_ros/config/x650.yaml`.
The controller owns position, velocity, attitude, rate, and control-allocation
loops; PX4 provides OFFBOARD/arming/saturation gating and relays the resulting
actuators over HIL.

External wall-clock DDS control must use `enable_lockstep=False`. A DDS OFFBOARD
transition can pause a PX4 actuator cycle, so Pegasus lockstep can deadlock while
waiting for a reply. `spawn_x650_with_mavlink` keeps `enable_lockstep=True` as its
backward-compatible default; the ROS hover launcher overrides it through
`PEGASUS_PX4_LOCKSTEP=0`. Do not globally disable lockstep for PX4-owned examples.

Current validated constants and results:

- authored X650 mass: `3.5 kg` total;
- computed steady hover motor command: `0.4800039`;
- rotor moment ratio `kq/kf`: `0.018442656`;
- settled motor means:
  `[0.48031, 0.47983, 0.48109, 0.47906]`;
- final 10 s PX4-local altitude: mean `1.50084 m`, standard deviation
  `0.00510 m`;
- mean PX4-local 3D tracking error: `0.0152 m`;
- Isaac ground-truth altitude: mean `1.81577 m`, standard deviation
  `0.00458 m`.

The approximately `+0.314 m` Isaac-vs-PX4 altitude difference is an unresolved
HIL vertical-estimator datum offset. Preserve it as an explicit caveat; do not
compensate for it in hover thrust, position gains, or the commanded altitude.

The hover launcher writes `/tmp/x650_ros_hover_track.csv` and component logs with
the `/tmp/x650_ros_hover_*` prefix. It accepts `X650_HOVER_X`, `X650_HOVER_Y`,
`X650_HOVER_ALT`, and `X650_HOVER_TEST_DETACHED=1`. The pinned fixture uses the
corresponding `/tmp/x650_pinned_*` logs.

The launcher also starts `isaacsim_optitrack_ros2_emulator` for `uav_0`. Isaac
publishes ground-truth pose/quaternion on `/uav_0/state/pose`, ENU linear velocity
on `/uav_0/state/twist_inertial`, and FLU body velocity/angular rate on
`/uav_0/state/twist`. The emulator combines them into
`fsc_autopilot_ros2_msgs/msg/Mocap` on `/uav_0/mocap` at 250 Hz. The source topics
were measured at approximately 263 Hz in the 2026-07-24 live test.

### Operational launch sequences

The external controller workspace on the FSC Jupiter machine is
`$HOME/Workspaces/fsc_autopilot_ws`. APL20 requires CMake 3.28 or newer; keep it
below CMake 4 for compatibility with the other workspace projects. Build APL20
against the existing `px4_msgs` installation rather than using
`--packages-up-to`, which needlessly rebuilds `px4_msgs`:

```bash
cd "$HOME/Workspaces/fsc_autopilot_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select apl20 apl20_ros \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select apl20 apl20_ros
colcon test-result --verbose
```

Before starting Isaac Sim, run `nvidia-smi`. A driver/library mismatch observed
after an NVIDIA package update was resolved by rebooting; do not begin by purging
NVIDIA packages.

Run the APL20 X650 directional-thrust test with the GUI as follows; append
`headless` to run without the GUI:

```bash
cd "$HOME/Source/fsc_PegasusSimulator"
FSC_AUTOPILOT_WS="$HOME/Workspaces/fsc_autopilot_ws" \
X650_HOVER_X=1 X650_HOVER_Y=-1 X650_HOVER_ALT=1.5 \
./scripts/indoor_sim/start_x650_ros_offboard_hover_test.sh fsc_lab_machine
```

The target uses ENU coordinates. The verified `(1, -1, 1.5)` m run converged to
about 1.5 cm 3D PX4-local tracking error. Stop it with
`tmux kill-session -t x650_ros_hover`.

Use the following standard launchers when PX4, rather than APL20, should own the
complete flight-control and motor-command path:

```bash
# Standard Iris
./scripts/outdoor_sim/start_single_drone_sitl.sh fsc_lab_machine

# Calibrated bare X650 with X650-specific PX4 gains
./scripts/outdoor_sim/start_x650_single_drone.sh fsc_lab_machine
```

Do not run `apl20_ros/autopilot_node` or publish to
`/uav_0/fmu/in/actuator_motors` alongside these standard launchers. Both use the
`px4_isaac` tmux session, so they cannot run simultaneously without separate PX4
instances, ports, vehicle IDs, and session names. Stop either with
`tmux kill-session -t px4_isaac`.

Focused runbooks are in `docs/single_drone_standard_sequence.md`,
`docs/X650_APL20_START.md`, and `docs/X650_NORMALIZED_THROTTLE_ROS2.md`.

For a standard indoor Iris with a parameter database isolated from outdoor
simulation, use
`scripts/indoor_sim/start_single_drone_iris.sh`. It stores parameters under
PX4's `build/px4_sitl_default/rootfs_fsc_indoor/` while retaining PX4 instance 0,
TCP port 4560, MAVLink system ID 1, and namespace `/uav_0`.

For the equivalent bare X650 with the measured `10.51 1/s` motor lag, use
`scripts/indoor_sim/start_single_drone_x650.sh`. Its independent
`rootfs_fsc_indoor_x650/` profile combines the indoor external-vision parameters
with the validated X650 attitude/rate gains. Both indoor launchers verify Isaac
ground truth on `/uav_0/state/pose` and `/uav_0/state/twist_inertial`.

## Code style

- Follow PEP 8, Google-style docstrings, and the existing local style. Black is
  configured for a 120-character line length.
- Keep imports explicit and use absolute package imports. Do not hide missing
  dependencies with broad `try/except` blocks.
- Shell scripts use Bash, begin with `set -euo pipefail`, quote expansions, and
  validate required files and directories. Use `shellcheck` when available.
- Preserve BSD-3-Clause headers on files that already use them; add appropriate
  attribution to new substantial source files.
- Update the nearest README/Sphinx page when behavior, setup, launch commands, or
  public APIs change. Record meaningful FSC changes in
  `docs/fsc_log/CHANGELOG_FSC.md` when appropriate.

## Validation

Choose the narrowest checks that exercise the changed behavior and state clearly
when Isaac Sim, PX4, ROS 2, a GPU, or an out-of-band asset prevents a live test.

```bash
# Repository formatting and static checks
pre-commit run --all-files

# Documentation
python -m pip install -r docs/requirements.txt
make -C docs html

# Representative standalone smoke run (requires Isaac Sim)
isaac_run examples/0_template_app.py
```

The extension test under
`extensions/pegasus.simulator/pegasus/simulator/tests/` uses `omni.kit.test`; it is
an Isaac Sim/Kit test, not a plain `pytest` suite. For physics or controller changes,
also run the affected application through its launch script and inspect both Isaac
and PX4/ROS output. Do not claim runtime verification based only on compilation or
import checks in system Python.

For Python files that do not import Isaac modules at module load, a narrow syntax
check is useful:

```bash
python -m py_compile path/to/changed_file.py
```

## Assets and generated files

- FSC USD assets under `extensions/fsc_aerial_manipulation/**/assets/` are large,
  gitignored, and distributed out of band. Their absence from `git status` does not
  mean they are unimportant; check filesystem state before diagnosing a missing
  asset.
- Never add local asset backups, simulator output, logs, ROS `build/install/log`,
  Sphinx `_build`, caches, or machine-specific config to commits.
- Do not edit binary USD files casually. Keep units explicit (SI in simulation code),
  verify prim paths and applied PhysX APIs, and document the source of mass/inertia
  or calibration changes.

## Change discipline

- Inspect nearby code and current worktree changes before editing. Preserve user
  changes and avoid unrelated cleanup in focused patches.
- Prefer extending FSC modules over duplicating scenario logic or patching upstream
  classes. Factor shared spawn/configuration behavior into the FSC package when two
  or more FSC applications need it.
- Keep public behavior backward compatible unless the task explicitly calls for a
  breaking change.
- Use concise commit subjects with the repository's conventional prefixes:
  `feat:`, `fix:`, `rem:`, `doc:`, or `chore:`.
