# Raw-mocap guard: the velocity after a pause or a flip (report item 8, 2026-10-21)

The processor's twist is a one-sample finite difference: zero on a republished frozen pose,
spiked for ~3 samples (~25 ms) after a VRPN pause or a drone flip. On 10-21 those spikes passed
the guard into the law's odometry: 3.75 m/s after the flip (1-2 ticks of +61 N), 0.5-2 m/s after
the pauses.

## The change (fsc_drone_state_estimator_ros2, dev_robotic_arm, raw executable only)

`MocapOdomBridge`: new parameter `mocap_guard_vel_holdoff_s` (0.1 s; 0 = off). For that long
after any bad or bit-identical (repeated) mocap sample, and while the blend back runs, a published
mocap sample carries EKF2's velocity (when fresh) instead of the processor's. Position, attitude
and the bridge are unchanged; the fused executable is unchanged.

## Checks

- `test_mocap_guard` 10/10 (unchanged logic).
- `tools/replay_check.py`: the recorded 10-21 mocap + EKF2 around the flip and two pauses replayed
  through the BUILT `indoor_mocap_feedback_node` (private domain). Processor max 3.76 / 1.22 /
  0.55 m/s; published max 0.25 (the processor's normal noise) / 0.04 / 0.13 m/s (EKF2's). The
  flight's own odometry had carried 3.75 / 1.21 m/s.
- The raw hardware stack refuses an estimator without `mocap_guard_vel_holdoff_s`.
- NOT flown.
