# Yaw swing during the SAFETY (baseline) takeoff — 2026-10-16

User report: in the pick-and-place / pick-and-drop / box-pushing sims the vehicle
"always yaws a little and rotates back" during the baseline takeoff.

## What it is

Every run on the mirror-derived plant (35 of 35 checked: `pick_place_drop_20261014`,
`lift_lurch_gains_20261016`, `j4_heading_stuck_20261016`) does the same thing:

| | yaw |
|---|---|
| on the ground | 0.0° (the commanded yaw) |
| starts turning | as the skids unload, ~0.5 s before 3 cm of lift |
| peak | **−29° (clockwise)**, ~2.4 s after it starts, z ≈ 0.55 m |
| back through 0° | ~6.5 s after it starts |
| overshoot | ~+2.5°, settled before DIRECT entry (~12 s) |

Run-to-run spread is ±1° (`tools/yaw_takeoff.py`, `tools/yaw_timeline.py`).

## Cause

The mirror plant's **standing yaw torque**, `sim_plant_torque_bias_z: -0.095` N·m
(`..._l1_4d_..._sim.yaml` section 1.2; every derived pick-and-place / pick-and-drop /
box-pushing yaml inherits it). 06 applies it body-fixed from the first physics step.

1. On the ground the skid friction holds the vehicle against it.
2. PX4 resets its rate integrators while the land detector says landed, so at lift-off
   the yaw integrator starts at zero and only the P terms push back.
3. The swing is large because the same plant scales the yaw drag coefficient to
   0.2333× (`sim_plant_km_scale`, 0.70× bench). SAFETY's yaw loop is PX4's, and the
   SITL profile's softened gains (`MC_YAWRATE_K` 0.3, `MC_YAW_P` 1.4, `MC_YAWRATE_P/I`
   0.2/0.1 defaults) were matched to the 3×-bench plant (`TUNING_t650.md`). PX4 now
   has ~1/4.3 of that authority, so the integrator takes ~5 s to learn the torque.

## Evidence

- **A/B in Isaac** (`runs/nobias1`, same pick-and-drop bring-up, only
  `PEGASUS_PLANT_TORQUE_BIAS=0.0,0.0,0.0`; banner confirms torque 0 and k_m ×0.2333):
  peak **+1.7°** vs −29°. Flown to DIRECT entry only, then interrupted.
- **Model** (`tools/yaw_model.py`: PX4 yaw cascade + rotor lag, step torque at lift-off,
  integrator from zero, Izz 0.12): −41° at 2.3 s, small positive overshoot — the
  flown shape and order of size. Same torque at `km_scale` 1.0: −8°. Not flown.
- **PX4 gains** read from `rootfs_fsc_indoor_am_t650/parameters.bson`
  (`tools/bson_params.py`): only the X650 softened set is non-default.

## Hardware does not show it

10-14..10-16 flights (mocap): ±5° of twisting on the floor during spin-up, then
1–7° after lift-off in **either** direction, back within ~3 s. So the sim exaggerates
the hardware. The hardware Pixhawk's yaw gains are not recorded in this repo.

## Not changed

Nothing in any yaml, PX4 profile or code. Possible fixes (user's call):

- leave it — it is over before DIRECT entry and does not touch the DIRECT law;
- give the AM-T650 PX4 profile yaw gains matched to the mirror's yaw authority
  (e.g. `MC_YAWRATE_K` 0.3 → ~1.0), in that profile only;
- drop the torque bias from the scenario yamls (the DIRECT observer then has one
  disturbance fewer to find).
