# Rosbag reference — 6-state wing-borne NMPC (`nmpc6_cruise`)

How to record + read a bag for the 90° wing-borne controller
([nmpc6_cruise_node.py](../ros2/px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc6_cruise_node.py)).
Flight/launch details are in [wing_borne_90_runbook.md](wing_borne_90_runbook.md).

## Record

```bash
# start the recording BEFORE you engage the controller (from a clean shell)
bash scripts/recbag_nmpc6.sh myrun         # -> /tmp/bags/myrun   (Ctrl-C to stop)
```
Then take off on Safe Hover, engage **"✈ 6-state Wing-borne (90°)"**, let it fly (or crash), Ctrl-C.

## Topics recorded + what each carries

| topic | msg type | what it tells you |
|---|---|---|
| `/uav_0/mocap` | `fsc_autopilot_ros2_msgs/Mocap` | **the truth state.** `pose.position` (ENU world), `pose.orientation` (FLU-body→ENU quat), `twist.linear` (WORLD velocity), `twist.angular` (**BODY** rates p,q,r). Everything the node feeds the NMPC comes from here. |
| `/uav_0/fmu/in/actuator_motors` | `px4_msgs/ActuatorMotors` | **commanded motor thrusts** `control[0..3]` = FL, FR, RL, RR, normalized 0..1 (×18 N). Front pair vs rear pair = the pitch/thrust split; rear→0 at wing-borne cruise. |
| `/uav_0/fmu/in/actuator_servos` | `px4_msgs/ActuatorServos` | **commanded servos** `control[0..4]` = **tilt_L, tilt_R, aileron, elevator, rudder** (normalized −1..1; tilt 0..1). tilt→1.0 (=90°) is full pusher. |
| `/uav_0/ground_station/active_controller` | `std_msgs/String` | which controller is driving PX4 — look for `nmpc6_cruise` to mark engage time. |
| `/uav_0/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` | arming + nav state (armed? offboard?). |
| `/uav_0/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | PX4's OWN attitude estimate (NED-FRD quat) — cross-check vs mocap. |
| `/uav_0/fmu/out/vehicle_local_position` | `px4_msgs/VehicleLocalPosition` | PX4's OWN position/velocity estimate (NED) — cross-check vs mocap. |

The node is **self-contained** (it builds its own hover→90° reference internally) — there is no
external reference topic to record, unlike the SMC/GS-fed controllers.

## Analyze

```bash
~/envs/isaac_env/bin/python3 /tmp/anabag.py myrun          # downsampled time series + summary
# columns: t, pos, vel, body-pitch, AoA (computed sim-exact: v_body=Rᵀv, α=atan2(w,u)),
#          motors, tilt servo, elevator ; summary = altitude excursion / AoA range / tilt range
```
`anabag.py` reads `/uav_0/mocap` + the two actuator streams directly (it imports the msg classes;
the GS python has no `yaml` for `get_message`). It ignores the reference topics (absent here) — fine.

The node ALSO prints a live attitude readout every 0.5 s (watch the `6f-nmpc6` window):
```
NMPC6 fly V=16 tilt=90 | pitch=+6 AoA=+6 roll=+0 yaw=+0 | z=20 ok=1 9ms T=[4,0] de=-0.11 mot=[..]
```

## What a HEALTHY wing-borne run looks like

- **tilt servo** ramps 0→1.0 (0→90°) as **V** builds 0→16 m/s, altitude **z held flat**.
- **pitch ≈ AoA ≈ +6°** in cruise (they must AGREE; the mocap pitch is validated == the sim AoA).
- **roll ≈ 0, yaw ≈ 0** throughout (small transient, recovers).
- **motors**: front pair unloads to ~0.1 (≈3.8 N total), **rear pair → 0** at cruise (wing carries weight).
- `ok=1` on the solves; solve time ~9 ms.

## Failure signatures (how to tell what broke)

| what you see in the bag/log | diagnosis |
|---|---|
| **pitch drops toward 0 → vz strongly negative** at tilt≈90 | pitch not held. At 90° the pitch is razor-sensitive (θ=+8→climb, θ=0→−8 m/s² dive). Usually a *symptom* of the lateral divergence below; if roll/yaw are clean, it's the elevator/pitch loop. |
| **roll or yaw grows** as tilt passes ~60–90° | lateral authority. The tilt-aware lateral hands roll/yaw to the aileron/rudder at cruise; if they still diverge, raise `k_ail`/`k_rud` or `kR_roll`/`kR_yaw`. |
| **motors all ~0 / NaN, vehicle just lands** right after engage | node loop crashed before publishing (e.g. an undefined attr / exception) — check the `6f-nmpc6` window for a traceback. |
| **tilt never leaves 0**, V never builds | not engaged (no `nmpc6_cruise` on active_controller) or still in the 2 s settle, or the solve is failing (`ok=0`). |
| **pitch ≠ AoA** in steady cruise | mocap attitude convention mismatch (should NOT happen — verified consistent; if it does, the frame/quat handling changed). |
| **mocap vs vehicle_attitude disagree** | EKF/mocap divergence — but nmpc6 uses MOCAP only, so it's immune unless the mocap itself is bad. |

## Params to tune live (`ros2 param set /nmpc6_cruise_control <p> <v>`)
`vcruise`, `cap_deg` (try 80 before 90), `ttr` (transition time), `latency_s` (match loop latency),
`kR_roll kW_roll kR_yaw kW_yaw k_ail k_rud` (lateral).

## Gotchas
- Record from a CLEAN shell (the script unsets the isaac_env/conda pollution).
- The node survives teardown — add `nmpc6_cruise` to `/tmp/kill_hil.sh`'s `pgrep -f` pattern.
- Set `PX4_NS=uav_0` (default) if your namespace differs.
