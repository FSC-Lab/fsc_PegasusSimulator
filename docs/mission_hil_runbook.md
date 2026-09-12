# Full-mission HIL runbook (SO(3) node, `mission_mode`)

One controller flies the whole mission: **climb → hover → transition → forward cruise →
back-transition → settle → land at a target point**. Self-contained (builds its own
`FSCMissionProfile` at the engage pose; no SMC/GS reference needed).

**Offline-validated envelope:** cruise speed **≤ 14 m/s** (tilt-assisted cruise, rotors still
share lift) → lands within **0.4 m** of the target. Genuine 90° wing-borne cruise is NOT in this
envelope (the rotor→wing handoff pitch instability — see `docs/fixed_wing_control_analysis.md` and
the 5-state `nmpc_cruise` track).

## Launch

```bash
# from a clean shell; opens the tmux HIL stack. Mission on the 6e-so3 window.
SO3_MISSION=1.0 MISSION_VC=13.0 MISSION_CAP=55.0 MISSION_TX=200.0 MISSION_TY=0.0 MISSION_ZC=15.0 \
  bash scripts/start_vtol_smc_hil.sh --here pieper_machine
```
Env knobs (all forced to doubles for rclpy): `SO3_MISSION` (1.0 on), `MISSION_TX/TY` (land target,
world m), `MISSION_VC` (cruise speed, ≤14 validated), `MISSION_CAP` (cruise tilt cap deg),
`MISSION_ZC` (cruise altitude m).

The node builds the NMPC with the mission-validated weights automatically
(`mission_w_att=220, mission_w_beta=200, mission_w_om=18`) — the strong-attitude tune that makes
it use TILT (not body-lean) for forward motion. This triggers a one-time JIT build if not cached.

## Fly it

The mission is the SO(3) node in STANDBY until activated. Two ways to engage:

1. **Standalone from the ground** (mission takes off itself): once the stack is up and the node is
   publishing, activate it —
   ```bash
   ros2 topic pub -1 /uav_0/ground_station/active_controller std_msgs/msg/String "{data: so3_transition}"
   ```
   The node auto-arms, builds the mission at the current (ground) pose, and flies climb→…→land.

2. **After a takeoff** (recommended for the first runs): take off on `geometric` to a low hover,
   then activate `so3_transition`. Engage **low** (near the takeoff altitude) for a full climb+land;
   engage at cruise altitude for a transition-only test (climb/land phases become no-ops since
   `z_ground = engage altitude`).

Watch the node window: it logs `MISSION t=.. /tf <phase> p=(..) V=.. tilt=.. aoa=.. ok=..`.

## Record + verify

```bash
/tmp/recbag.sh mission_hil    # /uav_0/mocap + actuators + reference + active_controller
/tmp/anabag.py mission_hil    # sim-exact AoA/tilt/altitude time series
```
Expect: altitude held through the transition, AoA staying below the 13° stall, tilt ramping with
speed, and a landing near `(MISSION_TX, MISSION_TY)`.

## Tuning knobs (live `ros2 param set /nmpc_so3_transition_control ...`)
- `mission_vcruise` — cruise speed (raise toward 14; >14 diverges at the cruise in sim).
- `mission_cap_deg` — cruise tilt cap.
- `kpitch` (default 3) — elevator pitch-attitude hold (the sole pitch actuator near 90°).
- `mission_speed_lead` (0.4) — how far the reference leads achieved speed through the transition.
- `mission_errhold` (3 m) — freeze the mission clock if position lag exceeds this (climb/hover/land).

## Notes / gotchas
- The legacy s-anchored transition path (SMC-fed, `mission_mode=0`) is UNTOUCHED — the mission is a
  self-contained branch (`_mission_step`).
- If a run inverts the vehicle, restart PX4 (fresh EKF) BEFORE Isaac, then relaunch — see the main
  runbook's reset procedure.
- The mission reference is dumped/validated offline: `fsc_mission_ref.py --tx 200 --check`, and the
  NMPC tracking offline: `nmpc_so3_transition_node.py --mission --tx 200 --vc 13 --cap 55 --amax 6
  --tout 16 --tback 12 --watt 220 --wom 18 --wbeta 200 --ksasp 0.6`.
