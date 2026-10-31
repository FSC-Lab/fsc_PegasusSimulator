# Experiment results, MATLAB data

The hardware flight results for the manuscript, converted from the ros2 bags. There is one self-contained folder
per task, like `matlab_simulation_data`, so a task folder can be copied alone to plot it. Each task folder has
its own README.md: file naming, the run index, the conditions and the struct layout.

This page summarizes the results of every task, what they show, and the wording and caveats for the
experimental section of `main_v2.tex`. It was written 2026-10-29 by hand, and no exporter overwrites it. Every
number below comes from the `.mat` files here. If a task folder is re-exported, check this page against that
folder's `*_metrics.mat` / `*_rmse.mat`.

| folder | task | flights used | written by | already in `main_v2.tex`? |
|---|---|---|---|---|
| `free_flight_tracking/` | free-flight trajectory tracking, Proposed vs Geo-L1 (the paper's experimental table) | 20 one-lap tests (two per setting), 2026-10-27 to 10-29 | `results/utils/experiment_tracking.py` | **no: the table and text there are older and must be replaced (section 1)** |
| `pick_and_drop/` | pick-and-drop at a 5 cm Drop Margin, with the session's 12 / 8 / 6 / 5 cm comparison | M5, 2026-10-28 | `results/utils/experiment_pick_and_drop.py` | **no: the manuscript reports D-2; replace it with M5 if the paper reports the newest flight (section 2)** |
| `pick_and_place/` | the earlier pick-and-drop flight, Drop Margin 20 cm | D-2, 2026-10-16 | `results/utils/experiment_pick_and_place.py` | yes, matches this data |
| `box_pushing/` | box pushing (feasibility, not the accuracy comparison) | BP-1 (reported), BP-2, 2026-10-27 | `results/utils/experiment_box_pushing.py` | yes, matches this data |

Not flown on hardware, so nothing to report: Geo-L1 at 0.20 m/s, MAC in any task, and a completed valve
turn (section 4).

## 1. Free-flight trajectory tracking (`tab:exp_free_flight_tracking`)

### How the table is made: the mean of two repeated tests

Each entry is the mean, column by column, of the RMSE of two repeated one-lap tests of that setting:
- **A test is one lap.** Each flight flies two laps back to back, and each lap is its own planner run from rest
  to rest. Its RMSE is computed over that lap alone, excluding the hover and the transition between laps.
- **Which tests:** both laps of the setting's latest flight in the final campaign, 2026-10-27 to 10-29. The
  rule looks only at the flight time, never at the error.
- **One configuration per method** throughout: the Proposed with the 2026-10-16 wrist-roll fix, Geo-L1 with
  its 2026-10-01 tune.
- **Only one setting was flown twice:** Proposed, circle, 0.13 m/s, on 10-28 at 14:46 (WB-13 26.9 mm, WB-14
  45.4 mm, mean 36.2) and 15:08 (WB-15, WB-16, the ones in the table). The earlier flight is scored and listed
  in `rmse_all` but not averaged. Either flight gives the same ordering.

Caption note, as in `free_flight_tracking/free_flight_tracking_exp.tex`: *"Each entry is the mean over two
repeated one-lap tests of that setting, flown in the final flight campaign."*

### Results

| trajectory | speed [m/s] | method | EE position RMSE [mm], mean (runs) | platform position RMSE [mm] | EE heading RMSE [deg] | runs |
|---|---|---|---|---|---|---|
| circle | 0.10 | Proposed | **29.3** (29.5, 29.1) | 27.9 | 0.22 | WB-11, WB-12 |
| circle | 0.10 | Geo-L1 | **82.5** (80.5, 84.5) | 72.4 | 3.14 | DEC-7, DEC-8 |
| circle | 0.13 | Proposed | **35.3** (31.1, 39.6) | 33.8 | 0.49 | WB-15, WB-16 |
| circle | 0.13 | Geo-L1 | **90.1** (90.0, 90.1) | 80.0 | 3.70 | DEC-9, DEC-10 |
| circle | 0.20 | Proposed | **36.5** (35.8, 37.2) | 35.2 | 0.33 | WB-7, WB-8 |
| circle | 0.20 | Geo-L1 | not flown | | | |
| figure-8 | 0.10 | Proposed | **41.6** (38.1, 45.1) | 40.2 | 1.20 | WB-19, WB-20 |
| figure-8 | 0.10 | Geo-L1 | **84.6** (83.0, 86.1) | 78.4 | 3.61 | DEC-13, DEC-14 |
| figure-8 | 0.13 | Proposed | **36.1** (38.5, 33.8) | 35.1 | 0.45 | WB-17, WB-18 |
| figure-8 | 0.13 | Geo-L1 | **85.2** (84.9, 85.4) | 77.9 | 4.73 | DEC-11, DEC-12 |
| figure-8 | 0.20 | Proposed | **43.8** (40.1, 47.5) | 43.3 | 0.86 | WB-9, WB-10 |
| figure-8 | 0.20 | Geo-L1 | not flown | | | |

The full table, with all 16 error columns including the joints, is `free_flight_tracking/free_flight_tracking_exp.tex`.

### What the results show

1. **The Proposed controller tracks better at every setting flown.**
   - End-effector position RMSE: 2.0 to 2.8 times smaller (circle 2.82 and 2.55, figure-8 2.03 and 2.36).
   - Platform position RMSE: 1.95 to 2.6 times smaller.
   - End-effector heading: 0.22–1.20° against Geo-L1's 3.14–4.73°.
   - Platform height: 2.5–5.3 mm against 10.1–13.1 mm.
2. **The joint columns favour Geo-L1:** q2/q3 0.5–1.1° against the Proposed's 1.8–3.4°. Its arm is a
   position-controlled servo tracking its own joint reference, while the Proposed arm is torque-controlled by
   the whole-body law. The manuscript's current 13-column layout omits the joints. Do not claim better joint
   tracking.
3. **On the circle, the error grows with speed for both controllers.**
   - Proposed: 29.3 → 35.3 → 36.5 mm.
     - Both 0.10 m/s tests (29.1–29.5 mm) are below every test at 0.13 and 0.20 m/s (31.1–39.6 mm).
     - The step from 0.13 to 0.20 m/s is small, 1.2 mm, and lies within the 0.13 m/s tests' spread.
     - A line through the six tests rises 6.2 ± 3.6 mm per 0.1 m/s.
   - Geo-L1: 82.5 → 90.1 mm, with no overlap between the tests.
4. **On the figure-8, the error is flat within the run-to-run spread.**
   - Proposed: means 41.6 / 36.1 / 43.8 mm, runs 33.8–47.5 mm across all three speeds. A line through all six
     runs rises 3.8 ± 5.1 mm per 0.1 m/s, which is not distinguishable from zero.
   - Geo-L1: 84.6 → 85.2 mm, runs 83.0–86.1 mm.
   - Do not claim a speed trend for the figure-8. Run selection cannot honestly produce one: one 0.13 m/s run
     (33.8 mm) is below both 0.10 m/s runs.
5. **Against the simulation** (`matlab_simulation_data/free_flight_tracking`, mean EE RMSE):

   | trajectory | speed [m/s] | Proposed sim → hardware [mm] | Geo-L1 sim → hardware [mm] |
   |---|---|---|---|
   | circle | 0.10 | 12.2 → 29.3 | 44.0 → 82.5 |
   | circle | 0.13 | 15.1 → 35.3 | 46.9 → 90.1 |
   | circle | 0.20 | 19.9 → 36.5 | 52.5 → not flown |
   | figure-8 | 0.10 | 15.4 → 41.6 | 42.9 → 84.6 |
   | figure-8 | 0.13 | 18.7 → 36.1 | 45.3 → 85.2 |
   | figure-8 | 0.20 | 25.3 → 43.8 | 60.4 → not flown |

   - The hardware error is 1.7–2.7 times the simulated one for the Proposed and about 1.9 times for Geo-L1.
   - The simulation shows the speed trend on both shapes. The ordering of the controllers is the same in both.

### Suggested wording

> In the experiments, the end-effector error of both controllers grew with speed on the circle: the
> proposed controller's RMSE was 29.3, 35.3 and 36.5 mm at 0.10, 0.13 and 0.20 m/s, and Geo-$\mathcal{L}_1$'s
> 82.5 and 90.1 mm at 0.10 and 0.13 m/s. On the figure-8, the differences between speeds were within the
> spread of the repeated tests (33.8–47.5 mm for the proposed controller and 83.0–86.1 mm for
> Geo-$\mathcal{L}_1$, over all tests), so no dependence on speed could be resolved there. At every setting
> flown, the proposed controller's end-effector error was 2.0 to 2.8 times smaller than Geo-$\mathcal{L}_1$'s,
> and its end-effector heading error at most 1.2°, against 3.1° to 4.7°.

### Sentence in `main_v2.tex` that no longer holds (line 7565)

> Geo-$\mathcal{L}_1$ was also not flown at $\bar{v} = 0.20$~m/s: its flight looked increasingly unstable as
> the mean speed rose, and on the figure-8, for instance, its platform height RMSE almost doubled from 0.10
> to 0.13~m/s.

With the final runs, Geo-L1's figure-8 platform height RMSE goes from 10.1 to 13.1 mm (+30 %). On the circle
it goes from 10.8 to 10.3 mm. Possible replacement:

> ... its flight looked increasingly unstable as the mean speed rose; from 0.10 to 0.13~m/s its
> end-effector error rose from 82.5 to 90.1~mm on the circle, and its end-effector heading error from 3.6 to
> 4.7$^\circ$ on the figure-8.

### Table rows in the manuscript's current layout

This is the 13-column layout of `main_v2.tex`: mm to one decimal, degrees to two, the lower value of each pair
in bold, and the manuscript's `\notflownmm` / `\notflowndeg` macros. The rows replace everything between
`\midrule` and `\bottomrule`. They were generated from `free_flight_tracking_rmse.mat` (`rmse_mean`), not typed
by hand.

```latex
 & \smash{\raisebox{-0.5\baselineskip}{0.10}} & Proposed & \textbf{20.3} & \textbf{18.9} & \textbf{2.5} & \textbf{0.21} & \textbf{0.61} & 2.09 & \textbf{21.4} & \textbf{19.8} & \textbf{2.8} & \textbf{0.22} \\
 &  & Geo-$\mathcal{L}_1$ & 49.3 & 51.9 & 10.8 & 3.09 & 0.82 & \textbf{2.07} & 56.3 & 59.4 & 10.0 & 3.14 \\
\addlinespace
\smash{\raisebox{-0.5\baselineskip}{Circle}} & \smash{\raisebox{-0.5\baselineskip}{0.13}} & Proposed & \textbf{25.5} & \textbf{21.8} & \textbf{4.1} & \textbf{0.22} & \textbf{0.58} & \textbf{2.08} & \textbf{26.6} & \textbf{22.7} & \textbf{4.5} & \textbf{0.49} \\
 &  & Geo-$\mathcal{L}_1$ & 59.4 & 52.6 & 10.3 & 3.90 & 0.63 & 2.29 & 66.9 & 59.8 & 7.2 & 3.70 \\
\addlinespace
 & \smash{\raisebox{-0.5\baselineskip}{0.20}} & Proposed & 25.4 & 23.9 & 3.2 & 0.26 & 0.63 & 2.10 & 26.5 & 24.6 & 3.6 & 0.33 \\
 &  & Geo-$\mathcal{L}_1$ & \notflownmm & \notflownmm & \notflownmm & \notflowndeg & \notflowndeg & \notflowndeg & \notflownmm & \notflownmm & \notflownmm & \notflowndeg \\
\addlinespace[0.8em]
 & \smash{\raisebox{-0.5\baselineskip}{0.10}} & Proposed & \textbf{27.7} & \textbf{27.9} & \textbf{5.3} & \textbf{0.27} & \textbf{0.68} & \textbf{2.11} & \textbf{28.3} & \textbf{29.2} & \textbf{5.7} & \textbf{1.20} \\
 &  & Geo-$\mathcal{L}_1$ & 55.7 & 54.3 & 10.1 & 3.63 & 0.84 & 2.20 & 60.7 & 57.8 & 11.2 & 3.61 \\
\addlinespace
\smash{\raisebox{-0.5\baselineskip}{Figure-8}} & \smash{\raisebox{-0.5\baselineskip}{0.13}} & Proposed & \textbf{20.7} & \textbf{27.9} & \textbf{4.5} & \textbf{0.38} & \textbf{0.62} & \textbf{2.18} & \textbf{21.3} & \textbf{28.8} & \textbf{4.6} & \textbf{0.45} \\
 &  & Geo-$\mathcal{L}_1$ & 56.0 & 52.5 & 13.1 & 4.71 & 0.79 & 2.30 & 61.7 & 57.4 & 12.2 & 4.73 \\
\addlinespace
 & \smash{\raisebox{-0.5\baselineskip}{0.20}} & Proposed & 29.5 & 31.3 & 3.8 & 0.54 & 0.74 & 2.22 & 29.7 & 31.8 & 4.3 & 0.86 \\
 &  & Geo-$\mathcal{L}_1$ & \notflownmm & \notflownmm & \notflownmm & \notflowndeg & \notflowndeg & \notflowndeg & \notflownmm & \notflownmm & \notflownmm & \notflowndeg \\
```

Then add the caption note above (the manuscript's table currently has no table note).

### Which run to plot in a single-run figure

- **Rule:** the first of the two tests (`run1`). Both tests are equally far from their mean, so neither is
  more representative. Say the rule in the caption.
- **The two settings the manuscript plots now** (circle at 0.13 m/s, figure-8 at 0.10 m/s):

| setting | Proposed | Geo-L1 |
|---|---|---|
| circle, 0.13 m/s | `circle/v0p13/whole_body_l1_circle_v0p13_run1.mat` (WB-15, 31.1 mm) | `circle/v0p13/geometric_l1_circle_v0p13_run1.mat` (DEC-9, 90.0 mm) |
| figure-8, 0.10 m/s | `figure8/v0p10/whole_body_l1_figure8_v0p10_run1.mat` (WB-19, 38.1 mm) | `figure8/v0p10/geometric_l1_figure8_v0p10_run1.mat` (DEC-13, 83.0 mm) |

The 3-D trajectory figure uses `run1` of each setting as well.

### Conditions (details in `free_flight_tracking/README.md`)

- **Vehicle and feedback:** T650 with the OM-X arm, 3.746 kg. OptiTrack fused in PX4 EKF2 (~100 Hz).
- **Trajectories:**
  - circle: radius 0.50 m;
  - figure-8: 1.40 × 0.70 m;
  - both: one lap with 4 s min-snap ramps, and the EE heading along the tangent;
  - arm: q2 = 25° ± 15°, four cycles per lap, fold q2 + q3 = 55°.
- **Configs as flown:** `free_flight_tracking/configs/`.

## 2. Pick-and-drop (`pick_and_drop/` M5, and `pick_and_place/` D-2)

The manuscript's experimental pick-and-drop text and table describe D-2 (2026-10-16, Drop Margin 20 cm).
The newest flight is M5 (2026-10-28, Drop Margin 5 cm). It is better on every tracking number, and the
basket landed flatter. If the paper reports M5, replace D-2's numbers with the M5 column.

| metric (whole mission) | hardware M5 (79.0 s) | hardware D-2 (84.4 s) | simulation, Proposed (mean of 3) |
|---|---|---|---|
| Drop Margin [m] | 0.05 | 0.20 | 0.20 |
| ‖ε<sub>UAV</sub>‖ max / RMS [mm] | 158.9 / 47.2 | 175.8 / 49.8 | 133 / 22 |
| ρ<sub>UAV</sub> max / RMS | 0.429 / 0.128 | 0.475 / 0.135 | — |
| EE position RMSE [mm] | 50.2 | 53.4 | 25 |
| EE heading RMSE [deg] | 0.59 | 1.50 | — |
| tilt max [deg] | 5.8 | 6.0 | — |
| claw from its target at the gripper's OPEN [mm] | 56 | 65 | 7 |
| payload at rest | in the tray, 58 mm from the drop point, tilted 2.9° | in the tray, 30 mm from the drop point, tilted 5.1° | in the tray (3 of 3 flat) |

**M5 and its session (`pick_and_drop/README.md`):**
- **Four flights on 2026-10-28** lowered the Drop Margin 12 → 8 → 6 → 5 cm. Mission, controller and payload
  were the same. All four completed the six legs.
- **The drop got better as the margin came down.** Heights are the basket centre at the gripper's OPEN
  command above its rest on the tray floor.
  - 12 cm: released 138 mm up, hit at 1.05 m/s, bounced off the tray onto the floor.
  - 8 cm: 120 mm, in the tray but tilted 13°.
  - 6 cm: 81 mm, slid 66 mm.
  - 5 cm: 75 mm, hit at 0.73 m/s, no bounce, slid 16 mm, rested flat (2.9°).
- **Tracking was also best at 5 cm:** EE RMSE 59.7 / 56.1 / 53.4 / 50.2 mm for 12 / 8 / 6 / 5 cm.
- **Where the errors peak (M5):** at the lift (pick, 159 mm) and at the release (drop, 156 mm), when the
  payload's weight comes on and goes off.
- **How the drop was flown:** the operator pressed Drop 0.5 s after the release hold was reached. The planner's
  automatic drop and its aim at the measured basket were not used.
- **The margin is not recorded in the bag:** it was typed on the arm ground station, and the bag name gives it.
  The 8 cm flight's claw sat about 2 cm higher than an 8 cm margin gives. The heights above are measured on
  the basket, so they do not depend on the names.

**D-2 (`pick_and_place/README.md`):**
- **Flights:** on 2026-10-16 two flights completed the mission with the payload; D-2 is the second.
- **D-1, not reported:** the gripper closed wrongly on the holder's fin, and the payload came to rest 112 mm
  from the drop point, tilted 25°.
- **E-1** flew the mission without the payload (control flight).
- **Where the errors peak:** at the lift (pick, 157 mm) and at the release (drop, 176 mm).

**The simulation column** is the pick-and-drop campaign in `matlab_simulation_data/pick_and_drop`. It drops
from 0.20 m for all three controllers.

## 3. Box pushing (`box_pushing/`, BP-1 reported)

This is a feasibility result. The accuracy comparison between controllers is the simulation's. The
manuscript's experimental box-pushing text already matches this data.

- **Task:** a 280 g box (friction 0.25, 0.69 N to slide), pushed 0.40 m in 19 s with the arm held still.
  The push axis has natural inertia (1.64 kg, 33.1 N·s/m, 196.8 N/m).

| run | outcome | box pushed [mm] | ‖ε<sub>UAV</sub>‖ max / RMS [mm] | EE RMSE [mm] | vehicle tilt max [deg] | box tilt max [deg] |
|---|---|---|---|---|---|---|
| BP-1 | push complete. After the release, the q3 safety guard ended the mission at the start of the exit; the abort climbed away without touching the box | 390 | 83.9 / 34.6 | 33.0 | 4.9 | 1.54 |
| BP-2 | push stopped 1.5 s before its end by the q3 safety guard; in the abort q3 dipped to −3.8° for 0.64 s | 352 | 129.2 / 34.4 | 34.3 | 3.9 | 3.06 |

"Box pushed" is `box_pushing_metrics.mat` → `metrics_runs.box_pushed_mm`.

- **Impedance residual over the slide** (BP-1): 4.86 N RMS (26.3 mm as a displacement through the stiffness).
  - It uses the controller's own contact-force estimate, because the hardware has no force sensor. It is not
    comparable with the simulation's residual, which uses the PhysX contact force.
  - The residual is mostly the stiffness term of a ~25 mm end-effector offset that follows the platform's
    position error.
- **The box stick-slips** about its plan (BP-1): up to 66 mm from it, 31 mm RMS over the slide.
  - d<sub>box</sub> − d<sub>box,d</sub> is along the push, from the grip: the box (`tracking.box_pos_m`)
    against the claw's reference (`tracking.ee_pos_ref_m`).
  - The slide window is `impedance.windows` "SLIDE".
- Both landings were normal, and the box stayed upright and on the table in both flights.

**Consistency check for the manuscript:**
- The simulated box-pushing section of `main_v2.tex` (line 7457) still describes an older campaign: 0.5 kg,
  0.50 m in 24 s, two controllers.
- `matlab_simulation_data/box_pushing` now holds the 2026-10-28/29 campaign: 280 g and 0.40 m in 19 s, the
  same box and push as the experiment, with three controllers.
- Sentences that compare the experiment with "simulation" should cite the campaign actually used.

## 4. Flown but not reportable as results (not in this folder)

- **Valve turning, 2026-10-28** (two attempts): both turns were stopped by a false tilt reading of the valve's
  motion-capture body. The aborts were clean.
- **Valve turning, 2026-10-29** (one grasp): the vehicle swayed sideways in contact, the sway grew, and the
  flight ended in a crash after a safety abort. No hardware valve turn was completed.
- **Earlier tracking flights** (2026-09-28 to 10-05) used older controller configurations. They are scored in
  `free_flight_tracking_rmse.mat` (`rmse_all`, with the reason each is not averaged) but are not in the table.
  The same holds for the earlier of the two circle 0.13 m/s Proposed flights (WB-13, WB-14).

## Files

```
README.md                                            this page: the task index and the results summary
free_flight_tracking/free_flight_tracking_rmse.mat   rmse_mean (the table), rmse_runs (its 20 tests), rmse_all (every scored run)
free_flight_tracking/free_flight_tracking_exp.tex    the table in LaTeX, 16 error columns
free_flight_tracking/<shape>/v<speed>/*_run<k>.mat   one run each, struct `run`
pick_and_drop/, pick_and_place/, box_pushing/        one run per flight + *_metrics.mat
*/configs/                                           the controller configs as flown
*/README.md                                          naming, struct layout, conditions, MATLAB snippets
```

The original ros2 bags are on the lab desktop: `results/experiment_results/flight_test_data/` (the runs used
here) and `docs/experimental_data_ros2_bag/` (every flight). `results/experiment_results/flight_test_index.md`
lists every bag.
