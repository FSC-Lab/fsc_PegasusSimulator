# MAC (modular adaptive) on the pick-and-place comparison task (2026-10-26)

The user asked whether the modular adaptive controller (MAC, Yadav et al., TMECH 2025) can fly
the paper's pick-and-place task (07 scene, hat platforms, the 200 g CAD basket, hook grasp, the
10-09 timetable), so the comparison has all three controllers. If not, try a lighter payload.

On 10-08 MAC went 0 of 6: every attempt left DIRECT ~2 s after entering it, the arm torques on
the 3 N·m clamp.

> **CORRECTION, same day (2026-10-26 afternoon):** the first version of the drain below took only ONE
> message per subscription per physics step (rclpy's `wait_for_ready_callbacks` raises `TimeoutException`
> after each zero-timeout wait, and the loop stopped there). It removed the competition between
> subscriptions, but an effort backlog still never emptied, so the applied torque could stay ~36-40 ms late:
> that is the 32-40 ms measured at the onset of `mac_drop2/3`'s j1 oscillation. Fixed in 06 (wait again
> until a wait finds nothing). With the fix MAC completed the 200 g pick-and-drop 2 of 2 — see
> `../pick_and_drop_comparison_20261026/README.md`. Every flight in this folder used the half fix.

## 1. The 0 of 6 was a simulator artifact: the arm effort queue backed up

06 pumped the arm bridge node with ONE `spin_once(timeout_sec=0)` per 4 ms physics step, so it
ran at most 250 callbacks/s. The arm stack publishes efforts at 250 Hz, which by itself uses the
whole budget. In the pick-and-place scene the same node also serves other traffic: 07's
`pick_place/info` subscription and parameter client, and the mission driver's 50 Hz gripper
stream. The effort queue (depth 10) therefore fills, and the torque Isaac applies lags the
command by 10 x 4 ms.

Measured on the recorded runs (`tools/lag.py`, cross-correlation of the law's published torque
with the applied joint effort):

| run | lag j2 / j3 |
|---|---|
| 10-08 MAC attempt 1 (`cmp_mod_r1_a1`) | 40 / 40 ms |
| today, same stack (`mac_base1`) | 40 / 40 ms -- reproduced, left DIRECT 2.74 s after entry |
| today, drain on (`mac_drain1`, ...) | 0-8 ms |

MAC's arm module is a stiff joint loop: about 13 N·m/rad on j2, ~2.6 Hz, ζ ≈ 0.5, and no
gravity model. At DIRECT entry the 0.7 N·m gravity hold vanishes and the arm sags. The same
yaml in free flight sagged 3° and settled. With 40 ms of extra delay the identical sag rang up
to the clamp in 1 s and took the airframe with it. The whole-body law is softer and flew the same
delay; the 10-09 whole-body runs read 24-36 ms.

**Fix (opt-in, default OFF):** `PEGASUS_ARM_SPIN_DRAIN=1` in 06 runs every ready callback of the
arm node each physics step (at most 32) on a dedicated executor, so the newest effort command is
the one applied, as hardware ros2_control does. The base launcher validates it and bakes it into
the Isaac pane. Default off means every existing campaign's plant is unchanged. **Every run
flown before 2026-10-26 had the queue, the 10-09 whole-body and Geo-L1 pick-and-place runs
included**, so a parallel set needs all three re-flown with the drain on.

## 2. With the drain MAC flies the mission; at 200 g it cannot place

| run | payload | outcome |
|---|---|---|
| `mac_drain1` | 200 g | picked, carried; on the Ready To Place approach the basket struck the PLACE hat's rim, pitch to 12°, left DIRECT |
| `mac_drain2_200g` | 200 g | picked, carried; the basket struck the rim and stayed caught on it (centre 0.815 m, 110 mm off the axis); the rest of the mission flew with an empty claw |
| `mac_drain_m100` | 100 g | picked; the basket slid off the hook mid-carry (71.5 s) and fell to the floor |
| `mac_drain_m100_b` | 100 g | picked; slid off the hook early in the carry |
| `mac_drain_m50` | 50 g | never reached the pick: attitude-module torque oscillation in the hover, tilt guard at 13.5 s |

The basket's clearance over the hat rim, measured when its footprint first overlaps the platform
on the approach (`tools/clearance.py`):

| run | clearance |
|---|---|
| whole-body 10-09 run 1 / run 2 | +94 / +78 mm |
| Geo-L1 10-09 run 1 / run 2 | +77 / +85 mm |
| MAC 200 g (`mac_drain1` / `mac_drain2_200g`) | −99 / −115 mm |

Why MAC is ~190 mm short: it has no integral action and no arm gravity model, so the payload
shows up as standing errors. Mean over the hover before the pick vs the carry with 200 g
(`mac_drain1`):

| | unloaded | 200 g |
|---|---|---|
| base, along the nose (toward the hat) | +3.7 cm | +10 cm |
| base, height | −2 cm | −8 cm |
| pitch error | 1.5° | 5.5° |
| arm sag j2 / j3 | 3.2° / 2.3° | 5.5° / 5.9° |

The Ready To Place leg climbs diagonally onto the hat. A basket that sits 10 cm ahead and ~12 cm
low reaches the rim early and below the platform top.

## 3. Why the hook loses the basket: MAC carries the arm folded ~10° too low

`mac_drain_ps14` (200 g) raised Place Start from 1.0 to 1.4 m so the basket would cross the hat
level (`configs/modular_sim_pick_and_place_place_start_z1p4.yaml`, exploratory). It lost the
basket off the hook mid-carry instead. Over the carry leg (q2+q3 planned 70 -> 80°):

| run | slipped off | q2+q3 mean (min) | basket tilt p95 |
|---|---|---|---|
| whole-body 10-09 run 1 / run 2 | no | 73.2 / 73.1 (61) | 12° |
| whole-body 10-09 failed attempt | yes, 71.5 s | 68.9 (62) | 13.5° |
| Geo-L1 10-09 run 1 | no | 72.5 (62) | 9° |
| MAC 200 g (drain1 / drain2) | no (rim strike first) | 63.6 / 63.3 (57) | 13° |
| MAC 100 g (m100 / m100_b) | yes / yes | 63.0 / 61.3 (61) | 15-16° |
| MAC 200 g Place Start 1.4 (ps14) | yes | 58.9 (57) | 13° |

The claw pitches 90 − (q2+q3) below level: about 17° for the other two controllers and 26-31°
for MAC. The hanger's arch rests on the fingers' tops, which slope 21-26° toward the tips, so
MAC's claw slides it off. The one whole-body loss also had that controller's lowest fold. The
deficit is MAC's arm module (no gravity model, no integral). It is no smaller at 100 g than at
200 g, so a lighter payload does not help, and the hook does not hold a lighter basket any better.

## 4. Pick-and-drop: one success in three

The drop task (07 `PEGASUS_PNP_TASK=drop`: the 180 g assembly payload clamped by its fin, 0.77 m
pillars, the 35 mm tray; the release is from above the tray) avoids both set-down failure modes.
MAC needs its own pick-and-drop yaml: `make_modular_yaml.py` gained the `pick_and_drop` profile,
and the yaml is generated into `configs/` (not into fsc_autopilot_ros2). It is flown with
`WB_SIM_YAML=` (`tools/chain_drops.sh`) and the drop driver arguments
`--drop-place --grip-via-action --grip-axis-tol 0.003`.

| run | outcome |
|---|---|
| `mac_drop1` | **completed**. Claw 167 mm off the drop point at the open (MAC sits ~12 cm low, ~11 cm behind); payload in the tray, 84 mm off centre, tilted 17.8° (not flat) |
| `mac_drop2` | aborted at the Go To Start hover: a j1 (arm yaw) oscillation, ±2° -> ±30° in 0.75 s at ~2 Hz, every arm torque on the clamp |
| `mac_drop3` | aborted at the pick hover, same j1 oscillation |

For comparison, the earlier drops on the same tray (flown WITHOUT the drain):

| controller | claw error at open | payload |
|---|---|---|
| whole-body (tray5, tray6) | 6-11 mm | flat in the tray |
| Geo-L1 (geo_drop1-3) | 30-56 mm | in the tray, tilted 8-13° |

The j1 oscillation is MAC's stiff joint loop again: Mbar 0.022 x λ1 394 = 8.7 N·m/rad, ~2.7 Hz,
ζ ≈ 0.5. At its onset the applied torque lagged the command 32-40 ms, with the drain on. That
measure uses the driver's receive times of `modular_control_debug` and of Isaac's
`joint_states`, so a backlog in the driver itself would read the same way. Where that lag comes
from is NOT settled.

## Conclusions

- The 10-08 0 of 6 was a simulator artifact (the arm effort queue), not MAC. `PEGASUS_ARM_SPIN_DRAIN=1`
  removes it. Opt-in, default off.
- The pick-and-place set-down task as flown on 10-09: MAC 0 of 6 placed with the drain on. Two
  rim strikes and three hook losses, both from its standing errors under load (base 10 cm
  forward / 8 cm low, arm fold ~10° low); one hover trip. A lighter payload does not fix it:
  100 g lost the hook 2 of 2, and 50 g tripped before the pick.
- Pick-and-drop: MAC 1 of 3, payload in the tray but not flat. The two failures are a
  pick-hover j1 oscillation of its arm module.
- A parallel set needs the whole-body and Geo-L1 rigs re-flown with the drain on too.

## Files

- `tools/fly_one.sh <rig> <tag> [VAR=val ...]`: one flight between clean slates; copies the files to `runs/`.
- `tools/chain_masses.sh`, `tools/chain_drops.sh`: the sequences flown here.
- `tools/lag.py`: command -> applied arm-torque lag.
- `tools/clearance.py`: the basket's clearance over the hat rim.
- `configs/`: the exploratory Place Start 1.4 yaml and the generated MAC pick-and-drop yaml.
- `runs/`: every flight (npz, driver, runner, stack, Isaac logs).
- Code: 06 (`ARM_SPIN_DRAIN`, `_drain_arm_node`), `scripts/indoor_sim/lib/start_single_drone_x650.sh`
  (validates and bakes `PEGASUS_ARM_SPIN_DRAIN`), `modular_adaptive_20260930/tools/make_modular_yaml.py`
  (`pick_and_drop` profile).
- Trap: the MAC launcher has no pick-and-place wrapper, so `PNP_PAYLOAD_MASS` alone never reached
  the Isaac pane (07 printed 200 g). Pass `PEGASUS_PNP_*` themselves; the controller stack starts
  the tmux server with them. Check the scene banner every time.

## 5. Pick-and-place again with the CORRECTED drain (2026-10-26 pm): still 0 of 3

User: "can MAC do pick-and-place after you fix the problem?". `tools/chain_fixed_pnp.sh` flew the same
set-down task (200 g basket, hook, `--hook-place --timetable`) with the fixed `_drain_arm_node`.

| run | stable | basket | carry q2+q3 mean (min) | why not placed |
|---|---|---|---|---|
| macfix_pnp_1 | yes, peak arm torque 1.38 N·m | slid off the hook 9.2 s into the carry | 60.0 (57.3) | hook loss |
| macfix_pnp_2 | yes, 1.41 N·m | struck the place hat's rim, clearance −113 mm, fell to the floor | 63.4 (57.0) | rim strike |
| macfix_pnp_3 | yes, 1.46 N·m | slid off the hook 6.3 s into the carry | 58.8 (57.4) | hook loss |

`tools/carry_fold.py` scores the fold and the hook; `tools/clearance.py` the rim.

The fix removed the instability: every flight completed the whole mission, and no arm torque came near
its clamp. The two failure modes are unchanged from section 2-3, because they are MAC's STANDING errors,
not delay:
- an arm folded ~10-15° short in the carry (whole-body 73°), which tips the claw ~30° down so the
  arch slides off the fingers;
- a base ~8-10 cm low and forward, which brings the hung basket into the rim on the climbing approach.

**MAC cannot do the hook set-down task as specified without changing MAC** (an arm gravity model or
integral action), or **the task for everyone** (a deeper carry fold, a higher approach margin).
Pick-and-drop clamps the payload and releases from above the tray, and MAC completes it (200 g 2/2).
