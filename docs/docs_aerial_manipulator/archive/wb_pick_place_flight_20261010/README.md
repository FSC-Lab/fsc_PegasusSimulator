# 2026-10-10 hardware pick-and-place flight — whole-body 4-D L1, T650-AM (PP-4)

Bag: `docs/experimental_data_ros2_bag/1010 - T650-AM whole-body Pick&Place-*/1010 - T650-AM whole-body Pick&Place/`
`flight_wb_l1_4d_pick_and_place_20261010_190719` (one flight, 90 s) and `VID_20261010_190704.mp4` (20 fps,
14.0 ± 0.05 s ahead of the bag clock).

| run | bag | what was flown |
|---|---|---|
| PP-4 (`p4`) | `flight_wb_l1_4d_pick_and_place_20261010_190719` | pick (hooked), carry, place: basket touched down on the hat and was pulled off it 0.9 s later; pilot took STAB at 74.8 s |

Report: section 3–5 of the artifact "1009 + 1010 Pick-and-Place Flights"
(https://claude.ai/artifact/SKoEL1B8DjJR1fLYaLpfLS). The page's builder stays in
`../wb_pick_place_flight_20261009/tools/build_report.py`; it reads this folder's `analysis/*.json`,
`figures/setdown_strip.jpg` and `tools/figures10.js`.

## Run order

```bash
T09=../../wb_pick_place_flight_20261009/tools
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
B="<bag root>/1010 - T650-AM whole-body Pick&Place"
/usr/bin/python3 $T09/extract_bag.py "$B/flight_wb_l1_4d_pick_and_place_20261010_190719" ../npz/p4.npz
/usr/bin/python3 $T09/np1_compat.py ../npz/p4.npz             # NOT with PYTHONNOUSERSITE=1
for r in p1 p2 p3; do ln -sf ../../wb_pick_place_flight_20261009/npz/$r.npz ../npz/$r.npz; done
export PYTHONNOUSERSITE=1                                    # everything below: numpy 1.21 + apt matplotlib/scipy
AM_NPZ=$PWD/../npz /usr/bin/python3 $T09/timeline.py p4     # mode / planner / pick-and-place status / log lines
/usr/bin/python3 tracking10.py        # -> analysis/tracking_p4.json, series_p4.json (per-leg tracking)
/usr/bin/python3 q2_guard.py p4       # -> analysis/q2_guard_p4.json (joint 2 near its 45 deg guard)
/usr/bin/python3 pp4_analysis.py      # -> analysis/pp4.json (pick, hang, loaded hover, trim, set-down, takeover, link)
/usr/bin/python3 load10.py            # -> analysis/load_p4.json (load gauges, arm commands against the caps)
/usr/bin/python3 extra10.py           # -> analysis/extra_p4.json (errors at the presses, skids vs the hat)
/usr/bin/python3 fig10_data.py        # -> analysis/fig_p4.json (3d / err / views / land in the 10-09 layouts + trim, setdown, carry)
/usr/bin/python3 hang_views.py        # -> analysis/fig_hang.json (basket under the claw from the side, PP-2 against PP-4; Fig. 7)
/usr/bin/python3 hang_follow.py       # -> analysis/hang_follow.json (basket tilt against the fingers' pitch while it hangs; Fig. 8)
/usr/bin/python3 sim_hang.py          # -> analysis/sim_hang.json (the same on the recorded Isaac missions)
/usr/bin/python3 -I video_strip.py "$B/VID_20261010_190704.mp4" ../figures/setdown_strip.jpg
cd $T09 && python3 build_report.py && python3 build_artifact_page.py <out.html> && python3 measure_layout.py <out.html> 1440
```

`pp10_common.py` sets `AM_NPZ` to this folder's `npz/`, puts the 10-09 tools on the path and registers `p4` in their
`RUNS`, so `tracking.build("p4")`, `grasp_geom.series("p4")`, `payload_budget.build("p4")` work unchanged.
`explore.py`, `place_hires.py`, `info_dump.py`, `basket_rest.py`, `landing.py` print raw series (used to find the events).
`npz/` is not committed.

## Settings flown (from `pick_place/info`)

Pick point (0.930, 0.9579, 0.9002, yaw 0.9°) = Get x + 0.01; EE Offset (−0.02, 0, 0.16); Place point
(−1.11, −1.1039, 0.87, −180°) = Get x − 0.03, Get z − 0.01 (x and z have two decimals = typed; y four = from Get), so
the place hat's centre is taken as (−1.08, −1.104) with the basket resting at 0.882 m; margins 0.20 / 0.10; trim window
5 s. Pick pressed 1.5 s after arrival (no shift: under half a window), Place 5.0 s after arrival (trim +38, +19, −12 mm).
fsc_autopilot_ros2 `71ec97a` (MicroXRCEAgent at ordinary priority) was committed 27 min before the flight.

## Results

**Pick.** The rotor wash moved the basket 21 mm and 7.5° on the pick hat as the vehicle arrived (22.2–22.6 s, vehicle
0.41 m away, 0.25 m above); on 10-09 the same arrival moved it 4–6 mm. Gripper closed 10 mm from its target (planner),
fingers 1–2 mm from the hanger plane; fingers 8 mm (mean) / 4 mm (min) under the arch in the slide-in; lift-off at claw
height 171 mm (mocap) / 170 (planner) above the basket origin (10-09: 163 / 167). EE Offset z what-if: 0.15 → 14–18 mm,
0.17 → above the contact height 70–80 % of the slide-in. **The jaws centre ~20 mm to the left of obj_0's origin**
(claw 22 mm left at the close, basket moved 1 mm; hangs 15–24 mm to the right of the claw; 10-09: 11–20 mm) →
EE Offset y = +0.02 or re-zero the body's pivot. Lift: airframe pushed 231 mm forward (10-09: 267), CoM error 217 mm,
tilt 6.4°, q2 gave way 10°.

**Hang** (`hang_views.py`, report Section 3.2 + Fig. 7). The planner has NO payload model: claw target = point +
Rz(yaw)·EE Offset at the pick AND at the place (`fsc_trajectory_planner/src/pick_place.cpp`), i.e. it takes the basket's
mocap origin to be 20 mm ahead of the claw and 160 mm below it, level, at both ends. Hanging from the arch it is not:

| stage (arm straight ahead) | PP-2 centre along / tilt | PP-4 centre along / tilt |
|---|---|---|
| hover after the lift | −1 mm / 10° | −13 mm / 14° |
| above the place | −10 mm / 16° | −19 mm / 19° |
| descent, until the touchdown | −17 mm / 18° | −27 mm / 22° |

(+ = ahead of the claw; tilt = top forward, the body's reading at rest on the pick hat taken out.) "Short" = 20 − along:
37 mm (PP-2 over the descent; 35 at the touchdown instant) and 47 mm (PP-4). Parts before the touchdown: arch on the
fingers +8 / +12 mm toward the tips, hanger plane +26 / +25, tilt −51 / −65. A free hang would tilt 9° (27.5 / 167).
**10-10 reads 10–13 mm further back and 4° more tilted at every stage (171·sin 4° = 12 mm, one change). Whether that
is the new handle (arch contact 171 vs 163 mm above the origin, different weight) or a re-defined mocap body (rest
reading 3.3° vs 0.2°, the 180° flips gone) cannot be told from the bag** — the first report text said "with the new
handle" and was corrected. For Place x it does not matter: Get captures the same origin.

**What sets the hanging angle** (`hang_follow.py`, report Fig. 8 + Section 5.3 "A handle that hangs level"; the user
asked whether a handle tilted BACK toward the vehicle would counter the offset). The closed jaws hold the stem by
friction: the basket is NOT a free pendulum. In the claw's own vertical plane (pitch of the fingers | basket tilt):

| | PP-2 | PP-4 |
|---|---|---|
| hover after the lift | 28° \| 10° | 29° \| 14° |
| carry, after joint 2's jump | 12° \| 12° | 9° \| 13° |
| before the touchdown | 23° \| 18° | 22° \| 22° |
| pitch change carry -> touchdown followed | 57 % | 73 % |
| pitch changes faster than 2 s followed (slope, corr) | 0.64, 0.65 | 0.84, 0.78 |

A jolt (the lift, a joint-2 stick-slip jump) lets the stem slip and the basket settles near its free-hang angle
(9.6° / 9.1° = atan(27.5 / h)); between jolts it turns with the claw and creeps back at 0.1-0.3°/s. Tilt at the
touchdown = free hang + 9 / 13° added by the claw. **Design consequence: a handle leaning BACK moves the arch further
behind the centre of mass -- the basket still settles with its CoM under the arch, hangs steeper (21° free hang at a
15° lean, 30-34° at the touchdown) and the pick-vs-place mismatch grows to 73-82 mm; a handle leaning FORWARD ~12°
(arch above the CoM) hangs level, leaves 9-18 mm / 9-13°, and EE Offset x becomes ~+0.01 for pick and place alike.**
The remainder goes with equal claw pitch in the carry and place poses or jaws that do not grip the stem. Estimates
assume the basket behaves as in these two flights (`handle_case` in `build_report.py`).

**The same measurement in SIMULATION** (`sim_hang.py` on `../pick_place_top_hat_20261008/runs/*.npz`, 3 whole-body + 2
decoupled missions -> `analysis/sim_hang.json`). The sim payload is still `Box_Payload.usda` from the 2026-10-07 STL
= the FIRST handle (3 x 2.5 mm stem); the re-printed handle's CAD is not in the repo. The sim claw cannot close below
~19 mm, so it never grips the stem: the basket is a free pendulum. Tilt 8-10 deg at every stage from the carry to the
touchdown while the fingers' pitch goes 10 -> 20 deg (slope 0.00-0.02), swinging +-2-3 deg; centre within 9 mm of the
point under the true claw (hardware: 17-27 mm behind). So the sim reproduces the free-hang 9 deg and NOT the grip part
(+9 / +13 deg), and never showed the 37-47 mm hang offset. To reproduce it: the new handle's CAD, or an invisible grip
shim on the stem at finger height (the push scene's `PEGASUS_PUSH_FIN_GRIP_MM` pattern). Not done.

**Carry.** Loaded airframe error 74 mm rms in hover / 81 moving, 193 max. Joint 2 stick–slip: sits at 36° or 45°
(reference 38°), 9° jumps every 2.2 s, commanded torque 0.6 ↔ 2.3 N·m; airframe ±0.15 m with it. Arm commands cut at the
arm controller's caps for 0.07 s (j2, 2.44 N·m) / 0.10 s (j3, 1.42 N·m), above 90 % for 0.5 / 0.8 s; servo current mean
197 / 115 counts, peak 378 / 218. q2 reached its 45° position guard twice (45.2° max; above 44° for 3.2 s).
Load gauges vs 10-09: pitch moment 0.60–0.64 N·m (2.4–2.5 N at the claw; 10-09 0.44–0.46 → 1.7–1.9 N), power
461 → 513 W (2.5–2.7 N), delivered thrust +3.0–3.1 N, command +4.9 N → the basket with the new handle is probably
heavier than 190 g (not weighed).

**Place.** Arrival overshoot 87 mm (75 beyond the target) 1.0 s after arrival; settled (from 3 s) 13 mm beyond.
The 5 s window covered the whole wait: logged trim (+38.1, +19.2, −11.5) = bag average (+38.3, +19.1, −11.6).
Touchdown 72.68 s: basket 106 mm short of the typed point / 37 mm to the side = trim 38 + tracking 22 + hang 47;
85 mm from the hat's centre (17 mm inside the rim in the drag direction). Claw from its typed target at the
touchdown: 67 mm as flown, 24 mm with the trim off, 38 mm with the settled-part average.

**Set-down.** Touchdown → airframe moving back +0.36 s → basket (flat on the hat, tilt 2°, centre 0.882 m) over the rim
+0.92 s (dragged 12 cm) → leg end +1.31 s → open command +1.38 s → fingers open +1.68 s → floor. Airframe moved back
227 mm at up to 0.32 m/s, pitch 4.2° nose up then 8° nose down as the weight came back; the claw stayed 21 mm above its
planned height, at the arch-contact height, gripper closed on the stem. Planned bottom 56 mm below the claw height at
the touchdown (tilted basket ~20 + target ~20 under the arch + 12 trim). PP-2 (10-09) had no push at its set-down
because the hat never took the weight (basket stayed tilted 15–18° on the rim).

**Takeover.** STAB at 74.82 s (1.16 m); climb to 1.42 m; battery current 22 A → <1 A at 76.16 s; impact 76.76 s at
3.8 m/s; rest 1.0 m away, tipped 36°; disarmed 76.94 s. Flight node reported DIRECT to the end of the bag; planner's
tilt guard (20.5°) flew its ABORT on the floor (arm torque ≤ 1.25 N·m this time).

**Link.** 400.0 Hz attitude / sensor topics, largest gap 15 ms; odometry 100.0 Hz, 21 ms; control loop 250.0 Hz, 15 ms.

**Other.** PX4 vs mocap attitude, body-fixed: roll 2.5°, pitch 0.2° unloaded (10-09: 1.9–2.2° / 2.2–2.9°). obj_0: no
orientation flip at the pick; 4 single-sample jumps in the carry's turn. Skids (T650 gear, 242.5 mm below the body
origin, ±163 mm fore-aft, inner faces 142 mm from the centre line): tips 41 mm short of the near rim and 83 mm above
the hat top at the touchdown; the basket lands 66 mm ahead of the tips.

## Recommendations written into the report (Section 5; nothing applied to any yaml or code)

1. Release at the touchdown (planner reports it from obj_0's height, GS opens on it) or jaws that do not grip the stem.
2. EE Offset (−0.02, +0.02, 0.15); Pick = Get x + 0.01; Place = Get x − 0.06, Get y, Get z; margins unchanged.
3. `pick_place_descent_trim: false` for whole-body flights (live-settable), or a planner that averages only from 3 s
   after arrival and refuses an earlier press.
4. Place platform 200 mm wide × 300 mm deep, extended away from the vehicle, aim point 18 cm from the near rim, rim / mat.
5. Weigh the basket; lighter payload scales the push (~half weight ≈ 11–13 cm); payload mass in the controller's model.
6. j3 cap 192 → 230 counts (PWM Limit 380); carry pose with q2 at 33°; SAFETY landing from 0.8 m; flight node to report
   SAFETY when PX4 leaves OFFBOARD.

## Traps met here
- The 10-09 `grasp.run()` takes the basket's rest pose from samples with tilt < 2°; on 10-10 the body reads 3.3–3.8°
  at rest and moved before the grasp, so `pp4_analysis.py` uses the pose in the hover behind the stem.
- The lift-off criterion (basket 3 mm up) puts the "arch contact height" a few mm above the true contact: the claw
  coincides with it after the touchdown, it never went clearly below.
- `ffmpeg` is not installed; `PYTHONNOUSERSITE=1 /usr/bin/python3 -I` + `cv2` reads the mp4.
- Headless Chrome's smallest window is 500 px wide: a 420 px screenshot is a crop, not a layout.
