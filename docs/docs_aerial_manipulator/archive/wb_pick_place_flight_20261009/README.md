# 2026-10-09 first hardware pick-and-place flights — whole-body 4-D L1, T650-AM

Bags: `docs/experimental_data_ros2_bag/1009 - T650-AM whole-body Pick&Place-*/1009 - T650-AM whole-body Pick&Place/`

| run | bag | what was flown |
|---|---|---|
| PP-1 (`p1`) | `flight_wb_l1_4d_pick_and_place_20261009_172557` | all six legs to COMPLETE, **no payload on the claw**; `obj_0` not streamed |
| PP-2 (`p2`) | `flight_wb_l1_4d_pick_and_place_20261009_180228` | pick (hooked), carry, place (failed), to land start; **PX4 link froze** at 144.67 s |
| PP-3 (`p3`) | `flight_wb_l1_4d_pick_and_place_20261009_181817` | pick missed (EE Offset z 0.20), Reset, flown home |

Report: `report.html`, published as the artifact "1009 Pick-and-Place Flights"
(https://claude.ai/artifact/SKoEL1B8DjJR1fLYaLpfLS).

## Run order

```bash
cd tools
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
B="<bag root>/1009 - T650-AM whole-body Pick&Place"
/usr/bin/python3 extract_bag.py "$B/flight_wb_l1_4d_pick_and_place_20261009_172557" ../npz/p1.npz   # also p2, p3
/usr/bin/python3 np1_compat.py ../npz/p1.npz ../npz/p2.npz ../npz/p3.npz      # NOT with PYTHONNOUSERSITE=1
export PYTHONNOUSERSITE=1                                                    # everything below: numpy 1.21 + apt matplotlib
/usr/bin/python3 timeline.py p2            # mode / planner / pick-and-place status / log lines in time order
/usr/bin/python3 gaps.py p2 100 150        # per-topic last message and largest gaps (the link freeze)
/usr/bin/python3 tracking.py               # -> analysis/tracking.json, analysis/series_<run>.json
/usr/bin/python3 grasp.py                  # -> analysis/grasp.json (claw vs basket at the pick, hang, carry)
/usr/bin/python3 hover_error.py            # -> analysis/hover.json, fig_hover.json (settled hover, with / without the basket)
/usr/bin/python3 hover_pick_place.py       # -> analysis/hover_pick_place.json (hover RMSE at the pick and at the place:
                                           #    claw 20.2 mm horizontal / 3.1 mm height at the pick, no payload, 57.7 s;
                                           #    54.0 / 9.5 mm at the place with the basket, 23.6 s, PP-2 only)
/usr/bin/python3 payload_budget.py         # -> analysis/payload_budget.json, fig_load.json (what the vehicle carried)
/usr/bin/python3 payload_budget.py table p2 40 60 1   # the same quantities as a time table
/usr/bin/python3 views.py                  # -> analysis/fig_views.json (pick and place, top and side view)
/usr/bin/python3 trim_window.py            # -> analysis/trim_window.json (the planner's descent trim replayed on the long hovers)
/usr/bin/python3 trim_choice.py            # -> analysis/trim_choice.json (trim off vs on, 1 / 5 / 10 s windows, real descent times)
/usr/bin/python3 setpoint_tuning.py        # -> analysis/setpoint_tuning.json (claw vs arch height, sideways error, how the basket hangs)
/usr/bin/python3 fig_data.py               # -> analysis/fig_*.json
python3 build_report.py && python3 build_artifact_page.py <out.html>
python3 measure_layout.py <out.html> 1440  # also 1100, 500
```

`npz/` is not committed (26–60 MB each). `pp_common.py` reuses the 0928 analysis' model and loaders
(`../wb_vs_decoupled_flight_20260928/tools/common.py`); `extract_bag.py` is that campaign's extractor plus the
pick-and-place topics (`/obj_0/mocap`, `pick_place/{status,info,arrival_error,path}`, `fused_odom_status`, `pixhawk_euler`).

## Results

**Controller.** PP-1 (no payload): airframe (CoM) error 33 mm rms / 105 mm max over 159 s of DIRECT, claw vs airframe
2.2 mm rms, claw heading 0.24° rms, tilt ≤ 2.5°, 0 saturated / 0 clamped ticks. Hover 15–28 mm rms, moving legs 39 mm.
Hover wander at the pick (39 s): std 15 / 16 / 3 mm, p-p 67 / 79 / 14 mm, period 6–7 s, mean 4 mm.

**Payload (PP-2).** The basket weighs 190 g (the user's scale). The first version of the report said 0.31 kg; that
was the step in the controller's thrust command divided by g, and wrong: the command is in the allocator's newtons
(7 % above delivered at the pick, 13 % at the end of the flight) and the step also held battery sag. Gauges:
pitch moment on the vehicle 0.44–0.46 N·m → 1.7–1.9 N at the claw (agrees with the scale); battery power 466 → 512 W
→ 2.2–2.4 N; delivered thrust (motor commands + battery voltage, 0820 fit) +2.7–2.8 N; controller command +3.7 N.
Without payload the same hover's command ranged 34.3–40.3 N over the three flights (battery 24.2 → 23.0 V) while
delivered thrust stayed 34.5 ± 0.3 N and power 465 ± 4 W. Lift: 252 mm (x) / 123 mm (y) airframe excursion, 6.6°
tilt. The hooked basket turned joint 4 by 13° (carry) and 52° (after the failed place).

**Hover error of the claw, settled** (reference still, first 3 s after arrival dropped). No payload, 95 s in 12 hovers:
offset 10 mm rms, wander 20, total 22 mm rms, 95 % inside 43 mm, max 68. With the basket, 23 s in 3 hovers (PP-2):
offset 27, wander 34, total 45 mm rms, 95 % inside 84, max 92; within 20 / 50 / 70 mm for 24 / 67 / 90 % of the time.
Height ±5–6 mm in both. (The 63 mm rms / 164 max of the tracking table counts every sample between two legs, with the
arrival and lift transients.)

**Arm torque caps.** Applied j2 / j3 torque peaked at 2.18 / 1.36 N·m = 89 % / 96 % of `max_effort` (2.44 / 1.42), the
arm controller's total torque term at 97 % / 98 %, no tick clipped; above 80 % for 0.36 s / 1.59 s. Recommended (not
applied): j3 192 → 230 duty counts with PWM Limit 330 → 380; j2 unchanged (its limit is heat: 180–224 current counts
for 70 s against the ~250 of the 08-26 overload shutdown).

**Pick geometry** (claw in the basket's rest frame, mm; planner frame / mocap frame):

| | PP-2 | PP-3 |
|---|---|---|
| EE Offset | −20, 0, 160 | −20, 0, 200 |
| at the close command | −21, 1, 158 / −27, −4, 154 | −32, −4, 192 / −38, −3, 186 |
| basket lifts off at claw z | 167 / 163 | not lifted |
| Get vs the basket's rest pose | −2.3, −1.9, −0.4 mm, 0.03° | −2.7, −2.6, −0.6 mm |

Hanging: claw at (−29, 10, 170) mm in the basket's own frame, basket tilted 10.8° about the arch (hanger plane is
27.5 mm off the basket centre in the CAD), up to 19° in the carry. In world axes at yaw 0 the claw is (+0.5, +17.6,
171.5) mm from the basket centre, i.e. the centre sits 20 mm closer to the vehicle than the planner's (−20, 0, 160).

**Place (PP-2).** Place pressed 0.8 s after arrival → descent trim +55 mm (1 s window sampled the arrival overshoot);
loaded tracking offset 60–70 mm; basket hang offset 20–30 mm. Touchdown 166 mm short / 25 mm sideways of the place
point (hat radius 100 mm); gripper opened at "EE 140 mm"; basket slid off the rim, stayed on the open fingers, was
lifted again by the exit, removed at 119 s. PP-1 (no payload): pressed after 1.6 s, trim (+36, +22), opened at 65 mm.

**What a Pick / Place press does.** The planner accepts it as soon as the approach leg has ended; with the trim on its
50 mm gate compares the claw with its own 1 s average (a speed check, not a distance to the target), and that average
becomes the descent's shift. PP-2's Place was accepted 76 mm from its target with a 57 mm trim. The ground station then
gates the pick's close (20 mm, 3 mm across) but not the hook place's release. Replayed on the long hovers the 1 s trim
is worse than no trim in every case. With the real descent times (bottom 3.2 s after the press with the trim off,
8.4 s with it on), loaded: off 48 mm rms, 1 s 70, 5 s 38, 10 s about the wander (34 mm; replay 22 on 3 s of data).
The loaded hover has a room-fixed offset of about (+25, +10) mm in all three loaded hovers, cause unknown.
Decided 2026-10-10 (user): `pick_place_descent_trim_window_s: 5.0`, applied to the hardware whole-body
`_pick_and_place.yaml` through `../pick_place_top_hat_20261008/tools/make_hw_pick_and_place_yamls.py` (uncommitted).
Press Place no earlier than 8 s after WAITING: under 2.5 s the planner applies no shift, at exactly 5 s the average
still holds the arrival swing (49 mm wrong on the loaded hover, about 20 mm from 6 s on).

**Recommended (revised 2026-10-10, `tools/setpoint_tuning.py`).** EE Offset (−0.02, 0, 0.16), not 0.17 (yesterday's
claw heights shifted to 0.17 are above the arch contact for 67 % of the slide-in); Pick = Get x + 0.01; Place = Get x −
0.03 (the basket centre hung 10–22 mm behind the claw against the planner's +20; check it on the ground with the new,
wider handle), Get y, Get z − 0.02, yaw −180; Vertical / Side Margin 0.20 / 0.10 unchanged; `pick_place_descent_trim: false` on the whole-body rig; wait
≥ 5 s before Place; gate the hook release on the claw error (not implemented).

**Problems.**
1. PP-2: every `/fmu/out` topic stopped at 144.666 s (18:04:53) in a 1.0 m hover; one fresh sample at 149.90
   (EKF2 had dropped EV, STAB, disarmed by RC switch), then nothing. Vehicle: roll 11.6° / 0.62 m/s after 0.93 s,
   levelled by 146.0 (PX4's own failsafe timing), coasted at 1.2–1.5 m/s, touchdown 147.35 at (−2.41, −1.51), rest at
   (−2.70, −2.00). PP-3: the same stream stopped at 89.7 s on the ground, 7 s after disarm.
2. Both manual (STAB) landings tipped over: PP-1 45° nose down (dropped 1.2 m/s from 0.43 m), PP-3 57° nose up
   (1.75 m/s). In PP-1 the flight node still reported DIRECT, the planner's tilt guard (18.4°) flew its ABORT and the
   arm pushed into the floor at the servo caps (j2 −2.7, j3 −1.4 N·m) for ≥ 2.4 s.
3. `obj_0`: not streamed in PP-1 (11 423 identical samples republished at 60 Hz, status "normal"); orientation flips
   180° (+5° tilt) whenever the claw is near in PP-2/3 (99.8–100 % correct at rest before the approach).
4. PX4 attitude vs mocap attitude: 1.9–2.2° roll / 2.2–2.9° pitch body-fixed (PP-3: 1.2 / 1.4), 0.4–0.6° world-fixed
   → 5–10 mm of claw height between the planner frame and the mocap frame.

## Traps met here
- `np.interp` across the 5.2 s feedback gap in PP-2 draws a straight line: `pp_common.direct_span` ends the scoring
  window where the odometry stream stops.
- The basket's solved yaw flips near the claw: use its REST pose for the pick frame, de-flip by continuity for the carry.
- `px_eul` (pixhawk_euler) is NED/FRD (pitch + = nose up); the mocap Euler angles here are ENU/FLU (pitch + = nose down).
- A headless full-page Chrome screenshot of a page taller than ~8000 px wraps around; screenshot per section.
- `joint_states.effort` is Present Current in counts; divide by κτ (162.4, 154.0, 150.5, 153.4 counts per N·m).
