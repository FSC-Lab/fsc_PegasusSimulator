# 2026-09-29 — first hardware flight of PS4 real-time teleoperation (whole-body 4-D L1, T650-AM)

Bag: `docs/experimental_data_ros2_bag/0929 - T650-AM whole-body-L1-4D Gamepad-*/…/flight_wb_l1_4d_ps4_20260929_160027`
(133 s, one flight). Stack: fused feedback, hardware yaml
`params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml` (vehicle `AM-T650-WB-L1-4D-HW`,
no `teleop_*` keys, so the planner's defaults flew: 0.20 m/s xy, 0.15 m/s z, 20 °/s yaw, 4 cm/s EE, time scale 1.0).
Design and interface: Command.md §7.21. Every number below comes from `tools/analyze.py` → `analysis/metrics.txt`.

## Verdict

**Stable, and the teleop chain worked end to end on hardware. It is not yet validated for the pick-and-place
operations that matter most**: altitude (△/✕), yaw (□/○), the arm from the pad home, and sustained presses were
not exercised in this flight. Two things to fix or work around first: arm work began at the stowed home, which the
pad cannot leave except downward, and every DIRECT → SAFETY revert drops the vehicle about 20 cm.

## What was flown

| | |
|---|---|
| DIRECT | 10.65–118.90 s (108.2 s); exit = operator "Return to baseline", no watchdog or abort |
| TELEOP | 14.50–112.95 s (98.4 s), released cleanly to HOLD |
| inputs actually used | 7.5 s of 98.4 s, all short taps (0.11–0.49 s): D-pad fwd/back 10 taps, left/right 13, L-stick EE forward 6, R-stick EE down 5, PS 1, L1 / R1 gripper |
| never used | △/✕ altitude, □/○ yaw, EE left/right, EE up, wrist roll, any press longer than 0.5 s, the leash, the geofence |
| base moved | 0.33 m in x, 0.23 m in y, 0 in z, 0° yaw |
| arm moved | EE 74 mm down, 15 mm forward relative to the airframe, then PS to the pad home [0, 30, 30, 0] |

## Stability (all of DIRECT)

| check | result |
|---|---|
| law loop | 250.1 Hz, max gap 16.9 ms |
| WholeBodyReference stream | 100 Hz, max gap 23.7 ms, fresh 99.996 % of ticks |
| pad feed `rc/input` | 25 Hz, max gap 45 ms (timeout 300 ms), never stale, never disarmed |
| rotor saturation / unallocated wrench | 0 ticks / 0 |
| motor commands | 0.36–0.79 |
| arm torque | max 1.04 N·m (j2) of the 3.0 clamp, 0 clamped ticks |
| attitude | `|e_R|` rms 0.023, max 0.075; tilt max 2.4°; body rate max 18.6 °/s |
| arm velocity source | velocity observer on 100 % of ticks |
| attribution | χ free 100 %; consumed `F̂_y` exactly 0 |
| feedback | VRPN 124 Hz, max gap 28.5 ms, no frozen poses; EKF fused EV position/velocity/yaw throughout |

## Tracking

Errors are the law's own (`wb_control_debug`). The absolute EE error is e_y + e_x, because the EE reference is
anchored to the CoM.

| phase | CoM rms / max | EE task rms | EE heading rms | EE absolute rms / max | joints rms (j1..j4) | tilt max |
|---|---|---|---|---|---|---|
| hold before teleop (3.4 s) | 10.0 / 16.8 mm | 3.6 mm | 0.63° | 12.4 / 20.4 mm | 1.65 / 0.36 / 0.09 / 0.08° | 1.5° |
| teleop (98.4 s) | **18.3 / 49.3 mm** | 2.7 mm | 0.70° | **18.8 / 50.6 mm** | 0.26 / 1.80 / 1.18 / 0.11° | 2.4° |
| hold after teleop (5.9 s) | 13.2 / 27.4 mm | 3.4 mm | 0.70° | 13.7 / 28.3 mm | 0.42 / 2.32 / 2.46 / 0.17° | 1.6° |

- **The position error is the airframe's slow wander, not the teleop.** The CoM error peaks at 0.125 Hz, and
  most of it sits in the 0.05–0.3 Hz band (12.4 mm of x). The largest excursions, 40–49 mm at 41, 73 and 77 s,
  happen with the reference static. This is the same ±15–35 mm, ~0.1 Hz mode seen in every 4-D flight since 0918.
  The 0928 circle flights' static-reference windows read 22.4 / 53.1 mm.
- **Pad → vehicle.** The pad target reaches the reference after **0.6 s** (the 5-stage CoM smoother, by design).
  The vehicle follows the reference with no added lag (cross-correlation 0.00 s), and measured minus reference is
  10.8 / 9.8 / 3.2 mm rms. One 0.15 s tap at 0.20 m/s moved the target 30 mm, and the reference arrived in about
  1.2 s with no overshoot (fig 2).
- **Arm.** The EE task error stays at 1–4 mm. The joints park 1–2.5° off the reference after each move: q2 38.3
  vs 40.1 at 62 s, and 27.6–27.7 / 30.5–32.7 vs 30 / 30 at 112–118 s. This is the known j2/j3 friction stick. The
  pass-through has no joint integral, and at K_y the parking error costs ~3–5 mm at the gripper (fig 3).
- **Gripper.** L1 opened it at 107.6 s and R1 closed it at 109.5 s (it holds closed on nothing at effort ~35).
  There was no visible effect on the vehicle: CoM error ≤ 36 mm, the same as the wander.
- **Battery.** 24.10 → 23.47 V over DIRECT. The L1 thrust estimate `d̂_z` walked +0.82 → −1.35 N with it and
  absorbed the fade.

## Findings that matter for pick-and-place

1. **The arm work started from the stowed home, where only "down" is free.** The pad's joint box is the range ×
   0.8, so q2 ≤ 37°, while the stowed hold has q2 = 40°. Six EE-forward pushes produced 14 mm in total and raised
   the wall note "joint 2 at the pad's inner bound" 8 times (fig 3). Forward moved at all only after "down" had
   lowered q2. Command.md §7.21 already says "Engage, press PS, then fly the arm". The PS press here came after the
   arm work, at 92.3 s.

   From the pad home [0, 30, 30, 0] the pure-axis reach is ±9 cm sideways, 10 cm down, 5 cm up, and only
   **±1.5 cm fore/aft**. Fore/aft positioning over an object has to come from the D-pad.
2. **Every DIRECT → SAFETY revert drops the vehicle ~20 cm.** Six of six airborne reverts on the 4-D hardware
   rig dropped 186–225 mm, 2–3 s after the switch (fig 4). This includes 0921 #1, where the arm barely moved, so
   the arm fold is not the cause and neither is teleop.

   The hardware yaml carries two thrust models 6.2 % apart:
   - the DIRECT allocator's `alloc_thrust_coeff` 4.260431e-05 (the deliberate 2026-09-12 experiment value);
   - the SAFETY map `vehicle_thrust_scaling/idle_thrust`, derived at the bench kf 4.540431e-05.

   At the switch SAFETY's first collective (0.579–0.609) sits 0.015–0.023 below DIRECT's motor mean, and the
   SAFETY UDE then re-learns −1.6 to −2.5 N over 6 s. The 6.2 % mismatch predicts −2.3 to −2.5 N. During a pick,
   an abort with the gripper near the table would lose that ~20 cm. Until the two models agree, keep ≥ 0.3 m of
   clearance under the gripper whenever a revert is possible.

   Two ways to make them agree:
   - re-derive the SAFETY pair at the allocator's kf. The flights' measured kf is 4.03–4.31e-05, so this is the
     more physical choice.
   - or restore the bench `alloc_thrust_coeff`.

   Both are the user's call: the experiment value is deliberate.
3. **Coverage gaps.** Altitude and yaw, the two controls a pick needs besides x/y, have not flown on hardware.
   Neither has a continuous stick deflection, nor base and arm moving at the same time.
4. **Tap granularity.** 0.20 m/s gives ~30 mm per short tap, which is coarse for aligning a gripper. Lowering
   v_xy in the station's rate box (e.g. 0.08–0.10 m/s → 12–15 mm per tap) is a live parameter change.
5. **Contact sensing is not covered by this mode.** χ stays free, and the raw collision reading `F̂` low-passed
   at 2 rad/s reaches p99 2.4 N / max 2.8 N in this flight's free flight (0928: 1.6 N p99). A 2 N contact threshold
   would false-trigger during teleop. At 0.25 rad/s it is 1.3 N.

## Figures (`figures/`)

- `fig1_overview.png` — pad inputs, CoM target / reference / measured in x and y, q2/q3, and the error traces.
- `fig2_dpad_taps.png` — two D-pad tap sequences: the smoother lag and the tracking.
- `fig3_arm.png` — the EE target relative to the airframe, where the walls hit, and q2/q3 target / reference /
  measured.
- `fig4_revert_dip.png` — altitude and SAFETY UDE after every DIRECT → SAFETY revert on the 4-D hardware rig.

## Tools (`tools/`)

1. `extract_bag.py <bagdir> <out.npz>` — the 0928 extractor plus the teleop topics (`rc/input`, `teleop/state`,
   `teleop/note`, `current_skeleton`). Run it with ROS sourced on `/usr/bin/python3`. Then run
   `np1_compat.py <npz>`.
2. `analyze.py` → `analysis/metrics.txt`, and `figures.py` → `figures/`. Run both with
   `PYTHONNOUSERSITE=1 /usr/bin/python3`, with `$AM_NPZ` holding `p1.npz` (this flight) and, for the revert
   table, `x_<date>_<time>.npz` of the 0918 / 0921 / 0924 whole-body bags.
3. `common.py` — the `wb_control_debug` and `teleop/state` index maps, and the pad decoding with the planner's
   default axes and deadzone.
