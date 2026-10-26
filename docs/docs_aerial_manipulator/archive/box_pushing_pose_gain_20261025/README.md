# 2026-10-25 — push-pose sweep and attitude-gain test for the 280 g / 0.40 m box push

User decisions after the 10-24 evening flights (`../wb_box_pushing_flight_20261024_evening/`): only the
280 g box, push ~0.40 m. Asked: find the safest push pose that keeps the push stable; test the H1b
attitude gains in Isaac and adopt them if they pass; match the SAFETY thrust map; a fresh pack per push.

## Setup (every Isaac run)

Headless, raw mocap at 120 Hz, the whole-body 4-D L1 sim box yaml, 280 g box (mu 0.25 / 0.25), push
0.40 m at 21 mm/s, hardware-like drift (`sim_plant_force_noise_n` 0.32 N at 0.12 Hz, seeds 1 and 2), the
hardware guard values (planner tilt guard 10 deg, abort release wait 1.5 s), the arm station running
Ready / the close / Push / Exit (`--gs-auto`, exit 0.2 s), as on hardware. Harness: copies of the 10-24
`runner.sh` / `fly.sh` / `run_pl.sh` / `pl_mission.py` / `phase_metrics.py` in `tools/`; one simulator
at a time; ~4.5 min per run.

**Isaac does not reproduce the hardware's contact oscillation** (clamped tilt 0.4-1.0 deg vs 2.1 on
hardware; `../wb_box_pushing_flight_20261024_evening/` section 6). These runs check that a change keeps
the mission complete and where the arm sits in its range; they cannot rank changes on the hardware
oscillation itself.

## 1. The push pose

Offline screen first (`../wb_box_pushing_flight_20261024_evening/tools/pose_sweep.py`,
`pose_sweep.txt` there): every (q2, q3) inside the real range, with the arm STROKE along the push:
how far the vehicle can move with the claw clamped before a joint leaves its range.

| pose | fold | back away (q3 -> 10 guard) | toward (q3 -> 50 stop) | claw under CoM | skids above table |
|---|---|---|---|---|---|
| [0, 32, 28, 0] flown | 60 | 22 mm | 36 mm | 113 mm | 117 mm |
| **[0, 28, 32, 0]** | 60 | **28 mm** | 30 mm | 118 mm | 122 mm |
| [0, 22, 34, 0] | 56 | 32 mm | 28 mm | 137 mm | 143 mm |
| [0, 6, 34, 0] | 40 | 35 mm | 32 mm | 198 mm | 209 mm |

The stretch side is the one that failed on 10-24 (the arm straightened). Lower folds buy more stroke but
put the claw further under the CoM, the lever that turns a contact force into a pitch moment.

Isaac, 2 seeds each (`phase_metrics.py`; q3 / q2 over contact + release):

| pose | complete | q3 range | q2 max | q3 min at the release | open-jaw hold | notes |
|---|---|---|---|---|---|---|
| [32, 28] | 2/2 | 14.2-40.2 | 39.0 | 24.1 / **14.2** | 2.9 / 1.7 s | |
| **[28, 32]** | **2/2** | **26.7-44.2** | **33.4** | **29.5 / 31.4** | 1.5 / 1.5 s | |
| [22, 34] | **1/2** | 24.3-43.3 | 29.0 | 24.3 | 1.8 s | seed 1: the claw swung 20-73 mm at up to 138 mm/s for 6 s after Ready, never settled; the station's 6 s limit aborted (jaws open, safe) |

No run touched either stop. **Adopted: [0, 28, 32, 0]** (same 60 deg fold: same claw pitch, the claw
7 mm back and 6 mm down on the body, the vehicle 6 mm higher).
- Moved: `make_push_pull_yaml.py` -> the sim and hardware box yamls (each diff = the pose line).
- Both whole-body hardware stacks expect it (box-profile tuned check 60/60 OK).

**Seen in every pose: the box moves 15-30 mm during the 1.5 s clamped hold, in either direction**
(+20, +31, -17 mm), following where the jaws closed relative to the grasp point (8-14 mm along). The
room-held target stays at the grasp point, so after the close the arm pulls the clamped box by that
residual (120 N/m x ~10 mm = 1.2 N > the ~0.7 N it needs to slide). Hardware: 55 mm on 10-24. Not
fixed: it needs a planner change (start the push from where the claw actually closed).

## 2. The attitude gains (H1b, k_R / k_w 2.134 / 1.567) on [28, 32]

| run | result |
|---|---|
| seed 1 | complete, but the Ready descent ended 57 mm off and the claw swung (up to 72 mm, 160 mm/s) for ~4 s; the close came 5 s late and 21 mm off; the box moved back 14 mm in the hold and 26 mm in the exit. Slide residual 1.40 N (1.78 at 1.6 / 1.2), contact tilt 0.05-1 Hz 0.74 (0.83) but 1.2-1.8 Hz 0.27 (0.15) |
| seed 2 | **ABORTED before any contact**: a 1.6 Hz tilt oscillation grew 2 -> 9.9 deg during Go To Start; the planner's 10 deg tilt guard |

**Not adopted.** It is the attitude mode that sent pick-and-place back to 1.6 / 1.2 (10-02). That mode
has been judged sim-only (10-07), so this does not prove H1b is bad on hardware, but it fails the test
the user set.

## 3. The SAFETY thrust map (applied, no flight needed in sim)

`vehicle_thrust_scaling` / `vehicle_idle_thrust` 0.036758 / 0.241528 (bench kf) -> **0.037947 / 0.252449**,
re-derived with t650_params' tangent formula at the kf the allocator flies (4.260431e-05, m 3.746170,
g 9.81). The hover command moves from 0.579 to 0.601, against the ~0.61 DIRECT flew. In the hardware box
yaml only (`make_hw_box_pushing_yaml.py` HW_NODE). Both hardware stacks now expect that pair on
`AM_HW_PROFILE=box_pushing` and the bench pair elsewhere. Verify with one DIRECT -> SAFETY revert in
free air before a push.

## 4. The straight-climb abort (`push_pull_abort_back` 0, applied)

The 10-24 evening abort knocked the 280 g box off the table: the open claw stayed at fin height for
4.5 s (the release wait, then a 3 cm back-off leg). Now the abort opens the jaws, holds the release
wait, then climbs STRAIGHT up. Injected 8 s into the push (`pl_mission.py --inject abort@8.0`),
280 g / 0.40 m, the hardware guard values (release wait 1.5 s), hardware-like drift
(`tools/abort_score.py`):

| seed | jaws open after | box lifted | box moved | box tilt | vehicle tilt | q3 min |
|---|---|---|---|---|---|---|
| 1 | 0.35 s | 0.0 mm | 0.7 mm | 0.0 deg | 2.7 deg | 30.1 |
| 2 | 0.23 s | 0.5 mm | 50 mm | 0.3 deg | 3.1 deg | 29.1 |

Seed 2's 50 mm is the open claw PUSHING the fin during the 1.5 s hold: the drift carried the
CoM-anchored claw on along the push, the box went with it (claw-box within 10 mm for 2 s), then the
climb left it upright. Not a safety issue (the box stays on the table: the 0.40 m push leaves its
front face 0.26 m from the end), but it is the known open-claw drag. The hold exists for the real
gripper's two WiFi hops; shorter is the only lever and it is not taken.

## 5. The push axis at natural inertia (applied: `wb_my_y` 1.64, `wb_ky_y` 196.8, `wb_dy_y` 33.128)

The law's arm term is `Lambda M^-1 (D e_v + K e) - (Lambda M^-1 - I) F_hat_y`. Along the push the
natural task inertia is 1.64 kg at [0, 28, 32, 0] (1.70 at [0, 32, 28, 0]) against the designed
M_y 1.0, so the law fed 0.64 x the contact-force ESTIMATE back into the arm -- the estimate that
reads 1-1.5 N rms and stick-slips on hardware. **Shipped version:** M_y = 1.64 zeroes that term
along the push, and K_y / D_y are scaled by the same 1.64 so the spring and damper the arm APPLIES
are exactly the 10-24 ones (197 N/m, 33 N.s/m). In free flight (F_hat_y = 0) the law is therefore
identical to the old one along any heading; in contact the only difference is the dropped F_hat_y
feedback, and the box sees the natural-inertia impedance 1.64 / 33.1 / 197 (no force estimate
needed). The control law and code are unchanged -- three yaml parameters. The contact switch chi
is kept (the user's design); F_hat is still estimated, guarded and used across / vertical / heading.
In `make_push_pull_yaml.py` OVERRIDES; sim + hardware box yamls regenerated; both whole-body
hardware stacks expect the three values on `AM_HW_PROFILE=box_pushing` (62 / 60 OK).

The first version (M_y 1.64, D 25.8, K 120, results below the next table) also softened the
free-flight arm along y (spring 197 -> 120, damper 33 -> 26) and was REPLACED: its two pre-contact
aborts could not be separated from that change.

Isaac, the shipped version, same conditions and seeds as below (`nat_*`, `--stop-after-exit`):

| seed | old (s2 = p2832_s2, s3 = base_s3) | shipped (`nat_*`) |
|---|---|---|
| 2 | complete, travel 376 mm, release q3 31.4, box back in the settle 17.1 mm | complete, 386 mm, 28.1, **22.6 mm** |
| 3 | complete, 414 mm, 30.5 / 31.2 (push / release q3), exit box move 6.0 mm | complete, 403 mm, 25.1 / 31.6, 0.2 mm |
| none | -- | complete, 404 mm, q3 30.6 / 32.3, tilt 2.1 deg, exit box move 0.5 mm |

3 of 3, no clamp, contact tilt 0.51-0.81 deg (old 0.58-0.83). Pre-contact identical by construction.
**One cost:** the box sees a 197 N/m spring instead of an effective 120, so where the jaws close off
the grasp point the clamped hold pulls the box harder (seed 2: jaws 16 mm off, box back 22.6 mm vs
17.1). Moving the push's start to where the claw closed (planner change, not built) would remove it.
The impedance residual is scored against each version's own declared impedance, so its N values are
not comparable across versions.

### The first version (M_y 1.64, D 25.8, K 120) -- replaced

**M_Y is a WORLD-axis diagonal**: world y is the push axis only when the push heading is -90 deg,
i.e. the box's mocap yaw ~0, as on every flight 10-21..10-24 (within +-10 deg the push-axis inertia
is still 1.60-1.64). Across (x) and vertical (z) keep the 10-09 shaping. In free flight F_hat_y = 0,
so the change there is a 22 % slower EE response along world y.

Offline (`tools/imp_bench.py`, the exact-law clamped-box bench, flown pose): identical without
contact-force noise; with the hardware-level noise on F_hat_y it helps on the worst seed (box speed
std 41.7 -> 33.9 mm/s, CoM p-p 137 -> 96 mm) and is neutral on the others (`imp_bench.txt`). The
bench does not reproduce the hardware amplitude, so this is a direction, not a prediction.

Isaac, 280 g / 0.40 m, hardware guards, hardware-like drift, paired seeds (old = `p2832_s1/2`,
`base_s3/4`; new = `imp_s1..4`, `imp_nom` without drift). From seed 4 on, `--stop-after-exit` (below):

| seed | old | new |
|---|---|---|
| 1 | complete | **abort before contact** (GS settle limit, see below) |
| 2 | complete, slide e_imp 1.73 N, box lag 15.6 mm | complete, 1.79 N, 8.1 mm |
| 3 | complete, 1.11 N, release q3 31.2 | complete, 1.40 N, release q3 20.1, box shoved 32 mm in the exit |
| 4 | complete, 1.30 N | **abort before contact** (the GS's automatic Ready never passed) |
| none | -- | complete, 0.70 N, release q3 31.4 |

Contact phase (`isaac_contact_tilt.py`): clamped tilt 0.58-0.83 deg old vs 0.54-0.75 new, box speed
std 22-28 vs 23-28 mm/s -- no difference; Isaac does not reproduce the hardware oscillation the change
targets. No crash, no joint clamp, every abort clean (box upright, on the table).

**Both new-value aborts happened before the jaws touched the box, in the hover / Ready window, and
trace to the base's drift, not the arm**: the arm's motion relative to the base in that hover is the
same in both configs (claw-base std 1.0-1.5 mm), and the base speed just after Go To Start is not
higher with the new values (24-41 vs 34-48 mm/s mean). (1) imp_s1: the GS's automatic Ready waits
for a claw slower than 30 mm/s for 0.5 s, and a slow swing is slowest at its extremes -- Ready
started with the base 45 mm off, the open claw came down 47 mm along / 18 mm across from the grasp
point and landed on the fin top (claw 208 mm above the box centre = the fin top), the arm folded 8 deg,
then the GS's 6 s settle limit aborted. (2) imp_s4: the base never held below 30 mm/s for 0.5 s within
the GS's 10 s retry (true claw speed median 31 mm/s; the planner reads a noisier FK claw). Still, 2 of 4
new vs 0 of 4 old is the one thing four seeds cannot clear. Two gaps this exposed, both independent of
the impedance: the Ready gate checks speed only (add "within ~15 mm of the point above the handle"),
and the drift model wanders the base 15-20 mm std in hover.

Also seen (every abort run, old values too): the driver's post-abort switch to SAFETY at 1.3 m tilts
the vehicle 13-18 deg and drops it 0.35-0.40 m in Isaac -- the sim's SAFETY handover, not the law.

**`--stop-after-exit`** (this folder's `pl_mission.py`, default off): ends the run 1 s after the exit
or after a planner abort finishes; no Go To Land Start, no landing (the runner's clean slate closes
the sim). The scored windows end at exit:end + 0.5 s (abort: + 3 s). ~45 s saved per run.

## Not touched

- The coordinated-admittance and arbitrary-time box yamls still carry [32, 28]; their generators now
  report STALE. Their recorded comparison results flew [32, 28]; regenerate them only when the
  comparison is re-flown with the new pose.
- The valve yamls: unchanged.

## Files

`runs/` (npz, logs, as-flown yamls), `runner.log`, `queue/done/` (the job files), `tools/` (harness copies).
Scoring:
```bash
cd tools && PYTHONNOUSERSITE=1 /usr/bin/python3 phase_metrics.py ../runs/*.npz
cd ../../wb_box_pushing_flight_20261024_evening/tools
/usr/bin/python3 isaac_tilt_band.py ../../box_pushing_pose_gain_20261025/runs/*.npz
/usr/bin/python3 isaac_contact_tilt.py ../../box_pushing_pose_gain_20261025/runs/*.npz
```
