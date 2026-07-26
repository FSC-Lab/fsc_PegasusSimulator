# FSC Lab Fork Changelog

## Added
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

## Modified
- Documented separate indoor Iris, indoor X650, and outdoor launch sequences.
- Documented that X650 free-flight direct-actuation control and allocation live in the external
  APL20 workspace, including the ROS 2 → PX4 → MAVLink HIL → Pegasus path and the distinction
  from the pinned motor-pulse diagnostic node.
- Corrected the normalized-throttle guide's motor-position labels to match the validated X650
  PX4 FRD rotor order.

## Notes
- Upstream project: https://github.com/PegasusSimulator/PegasusSimulator
