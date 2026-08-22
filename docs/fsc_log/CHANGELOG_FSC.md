# FSC Lab Fork Changelog

## Added
- Controller-neutral X650 direct-actuator SITL launcher at
  `scripts/indoor_sim/start_x650_direct_actuator_sitl.sh`; it reuses the calibrated indoor X650
  stack, disables Pegasus lockstep, and applies the per-run PX4 wall-clock DDS settings while
  leaving Micro XRCE-DDS and motor-command ownership to the external controller.
- Indoor PX4-controlled X650 launcher at
  `scripts/indoor_sim/start_single_drone_x650.sh`.
- Isolated persistent indoor X650 PX4 profile combining OptiTrack estimator
  settings with the validated X650 controller gains.
- Automatic ROS 2 ground-truth topic verification for the indoor X650.
- Corrected `x650_new.usd` rotation and PX4 motor ordering for bare-X650
  scenarios, with runtime overrides preserving the established 3.5 kg mass and
  CAD-derived inertia.
- Retired the legacy wrong-frame bare-X650 asset from all runtime selection;
  corrected the standalone allocation model to the new asset's PX4 Quad-X
  rotor-to-corner order.
- Bare-T650 geometric+L1 direct-actuator SITL launcher at
  `scripts/indoor_sim/start_t650_geometric_L1_adaptive_direct_actuation_sitl.sh`
  — the no-arm parallel of the AM geometric+L1 rig: identical bare-T650 plant
  to the geometric launcher (lockstep off), requiring the paired
  fsc_autopilot_ros2 single-drone L1 stack (which carries the same +20%
  controller-side thrust-coefficient robustness mismatch) by its decorated
  node name. Run sequence and campaign results: Command.md §7.13.

## Modified
- Retuned the AM-T650 whole-body controller for a simulation-only +15% thrust-
  coefficient mismatch. Added an optional, reaction-consistent joint-posture
  PID objective using the controller-smoothed reference, with clamped integral
  torque to keep the arm on the `[0, 40, 40, 0]` branch. Two clean 90 s hover
  runs passed with zero saturation, at most 1.55 degrees tilt, and 2.0-3.3 mm
  steady CoM RMS. This is a hover-only validation; +20% still fails.
- Injected a simulation-only +20% controller-side thrust-coefficient mismatch
  into the AM-T650 whole-body DIRECT configuration while preserving the
  measured 99.7 ms rotor lag and true Isaac coefficient. A guarded hover showed
  the current GMO tune does not reject it: the run was aborted at 30.5 degrees
  tilt after 7.7 s with 1.50 m CoM error and two arm joints saturated. The
  failure, restore value, and telemetry interpretation are in Command.md
  §7.14.2.
- Completed the AM-T650 whole-body direct-actuation operator path: the drone
  ground station now recognizes the whole-body controller and derives total
  normalized throttle from its four live motor commands without suppressing
  them when PX4's unrelated `pre_flight_checks_pass` flag is false; the arm ground station
  exposes `WB-TORQUE`, follows `external_torque_controller`, and routes joint or
  end-effector targets through that controller's minimum-jerk generator. The
  coupled law now consumes the exact published smoothed position/velocity
  reference, fixing activation homing, both Home buttons, and end-effector
  trajectory tracking. The home pose is `[0, 40, 40, 0]` degrees. Documented the
  XM430 torque-off requirement for Operating Mode changes and kept the whole-body
  arm in torque mode for the complete bring-up.
- Tuned the AM-T650 geometric+L1 simulation controller against a deliberate
  +20% controller-side thrust-coefficient mismatch: raised the adaptive thrust
  bound to 10 N, selected 2 1/s predictor poles and a 6 rad/s filter bandwidth,
  retained the stable geometric position gains, and documented the rejected
  higher-gain candidate and full X/Z validation metrics.
- Corrected the AM-T650 geometric+L1 plant/controller contract: restored the intended
  2.95 kg body mass, added a launch-time total-mass guard, advanced adaptation only on
  distinct feedback samples, and referenced the live arm CoM offset to the bare-airframe
  CoM required by the paper.
- Documented separate indoor Iris, indoor X650, and outdoor launch sequences.
- Documented that X650 free-flight direct-actuation control and allocation live in the external
  APL20 workspace, including the ROS 2 → PX4 → MAVLink HIL → Pegasus path and the distinction
  from the pinned motor-pulse diagnostic node.
- Corrected the normalized-throttle guide's motor-position labels to match the validated X650
  PX4 FRD rotor order.

## Notes
- Upstream project: https://github.com/PegasusSimulator/PegasusSimulator
