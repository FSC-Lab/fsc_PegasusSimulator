# Aerial-manipulator flight tests (ros2 bags)

Every aerial-manipulator flight-test bag in `docs/experimental_data_ros2_bag/`, and which runs fill the paper's
experimental free-flight trajectory-tracking table (`tab:exp_free_flight_tracking`).
Not listed: the robotic-arm calibration bench bags (09-09, 09-11) and the bare-T650 recordings (08-06, 08-07).

Regenerate (re-scores from the bags, updates this file and the table):

```bash
/usr/bin/python3 results/utils/experiment_tracking.py --paper <paper>/main.tex
```

## 1. All flight-test bags

| # | date | bag | controller | flown | scored runs |
|---|---|---|---|---|---|
| 1 | 09-12 | `flight_wb_l1_20260912_190224` | whole-body L1 (6-D observer) | first hardware flight of the whole-body law, short | -- |
| 2 | 09-12 | `flight_wb_l1_20260912_201430` | whole-body L1 (6-D observer) | hover and arm moves in DIRECT (development) | -- |
| 3 | 09-18 | `flight_wb_l1_4d_20260918_144828` | whole-body L1 4-D | first 4-D flight: planner legs (EE excursions, go-home, 0.61 m base step) | -- |
| 4 | 09-18 | `flight_wb_l1_4d_20260918_152112` | whole-body L1 4-D | metadata only, no data file | -- |
| 5 | 09-21 | `flight_wb_l1_4d_circle_20260921_112328` | whole-body L1 4-D | circle attempt (development) | -- |
| 6 | 09-21 | `flight_wb_l1_4d_circle_20260921_115207` | whole-body L1 4-D | circle attempt (development) | -- |
| 7 | 09-21 | `flight_wb_l1_4d_circle_20260921_123637` | whole-body L1 4-D | circle attempt: go-to-start only, mocap flips | -- |
| 8 | 09-24 | `flight_wb_l1_4d_circle_20260924_120228` | whole-body L1 4-D | circle, earlier tune, arm q2 30 +- 10 deg / 48 s; mocap freeze mid-run | -- |
| 9 | 09-24 | `flight_wb_l1_4d_circle_20260924_120546` | whole-body L1 4-D | circle, earlier tune, arm q2 30 +- 10 deg / 48 s | -- |
| 10 | 09-24 | `flight_wb_l1_4d_circle_20260924_165654` | whole-body L1 4-D | circle, earlier tune, arm q2 25 +- 15 deg / 12 s | -- |
| 11 | 09-24 | `flight_wb_l1_4d_circle_20260924_172101` | whole-body L1 4-D | circle, earlier tune, arm q2 25 +- 15 deg / 6 s | -- |
| 12 | 09-28 | `flight_wb_l1_4d_circle_20260928_171154` | whole-body L1 4-D | circle 0.13 m/s | WB-1 |
| 13 | 09-28 | `flight_wb_l1_4d_circle_20260928_172242` | whole-body L1 4-D | circle 0.13 m/s | WB-2 |
| 14 | 09-28 | `flight_decoupled_l1_circle_20260928_182835` | Geo-L1 (previous gains) | circle 0.13 m/s | DEC-1 |
| 15 | 09-28 | `flight_decoupled_l1_circle_20260928_183608` | Geo-L1 (previous gains) | circle 0.13 m/s | DEC-2 |
| 16 | 09-29 | `flight_wb_l1_4d_ps4_20260929_160027` | whole-body L1 4-D | PS4 teleoperation | -- |
| 17 | 10-02 | `flight_decoupled_l1_circle_20261002_134247` | Geo-L1 (2026-10-01 tune) | circle 0.13 m/s, two runs | DEC-3, DEC-4 |
| 18 | 10-02 | `flight_decoupled_l1_circle_ps420261002_134830` | Geo-L1 (2026-10-01 tune) | PS4 teleoperation | -- |
| 19 | 10-05 | `flight_wb_l1_4d_figure8_vel010_20261005_162406` | whole-body L1 4-D | figure-8 0.10 m/s, two runs | WB-3, WB-4 |
| 20 | 10-05 | `flight_decoupled_l1_figure8_vel010_20261005_163734` | Geo-L1 (2026-10-01 tune) | figure-8 0.10 m/s | DEC-5 |
| 21 | 10-05 | `flight_decoupled_l1_figure8_vel013_20261005_165431` | Geo-L1 (2026-10-01 tune) | figure-8 0.13 m/s | DEC-6 |
| 22 | 10-05 | `flight_wb_l1_4d_figure8_vel013_20261005_170129` | whole-body L1 4-D | figure-8 0.13 m/s, two runs | WB-5, WB-6 |

Bag folders: `<date> - T650-AM .../<date> - T650-AM .../<bag>/` under `docs/experimental_data_ros2_bag/`.

## 2. Scored runs of the formal free-flight comparison

Each run is scored over its whole planned trajectory (speed-up and slow-down included), with the same
definitions as the simulation table. **Selected** = the run in the paper table: the eligible run with the
lowest end-effector position RMSE of its setting.

| run | date | method | trajectory | speed [m/s] | EE position RMSE [mm] | platform position RMSE [mm] | EE heading RMSE [deg] | status |
|---|---|---|---|---|---|---|---|---|
| WB-1 | 09-28 17:11 | Proposed (whole-body L1 4-D) | circle | 0.13 | 26.25 | 25.35 | 0.48 | candidate |
| WB-2 | 09-28 17:22 | Proposed (whole-body L1 4-D) | circle | 0.13 | 24.37 | 23.43 | 0.44 | **selected** |
| DEC-1 | 09-28 18:28 | Geo-L1 | circle | 0.13 | 192.05 | 151.29 | 10.31 | not eligible: previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune |
| DEC-2 | 09-28 18:36 | Geo-L1 | circle | 0.13 | 218.44 | 176.84 | 10.31 | not eligible: previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune |
| DEC-3 | 10-02 13:42 | Geo-L1 | circle | 0.13 | 77.45 | 66.19 | 3.48 | **selected** |
| DEC-4 | 10-02 13:42 | Geo-L1 | circle | 0.13 | 72.34 | 58.34 | 3.82 | not eligible: incomplete: the operator returned to SAFETY 0.8 s before the plan ended; flown through a ground-station clock step that reset the L1 estimate |
| WB-3 | 10-05 16:24 | Proposed (whole-body L1 4-D) | figure-8 | 0.10 | 37.27 | 36.76 | 0.46 | **selected** |
| WB-4 | 10-05 16:24 | Proposed (whole-body L1 4-D) | figure-8 | 0.10 | 43.74 | 43.61 | 0.48 | candidate |
| DEC-5 | 10-05 16:37 | Geo-L1 | figure-8 | 0.10 | 72.59 | 66.56 | 3.58 | **selected** |
| DEC-6 | 10-05 16:54 | Geo-L1 | figure-8 | 0.13 | 76.86 | 68.73 | 4.69 | **selected** |
| WB-5 | 10-05 17:01 | Proposed (whole-body L1 4-D) | figure-8 | 0.13 | 42.66 | 42.11 | 0.54 | **selected** |
| WB-6 | 10-05 17:01 | Proposed (whole-body L1 4-D) | figure-8 | 0.13 | 58.19 | 58.31 | 0.48 | candidate |

## 3. The experimental table: which run fills each setting

| trajectory | speed [m/s] | Proposed | Geo-L1 |
|---|---|---|---|
| circle | 0.10 | not flown | not flown |
| circle | 0.13 | WB-2 | DEC-3 |
| circle | 0.20 | not flown | not flown |
| figure-8 | 0.10 | WB-3 | DEC-5 |
| figure-8 | 0.13 | WB-5 | DEC-6 |
| figure-8 | 0.20 | not flown | not flown |

## 4. The selected runs' data in this folder

Named like the simulation results: `<method>_<trajectory>_v<speed>_<run>`.

- `free_flight_tracking/<trajectory>/v<speed>/<name>/`: the run's ros2 bag, copied unchanged (the `.db3`
  and `metadata.yaml` inside keep their recorded names; `ros2 bag info <name>` works on the folder).
- `matlab_experiment_data/free_flight_tracking/<trajectory>/v<speed>/<name>.mat`: the run converted for MATLAB
  (`load(...).run`), the same struct as `simulation_results/matlab_simulation_data`. Only the selected run of the
  bag is exported.
- `matlab_experiment_data/free_flight_tracking_rmse.mat`: the experimental table's numbers.

| run | name | original bag | run in the bag |
|---|---|---|---|
| DEC-3 | `geometric_l1_circle_v0p13_DEC-3` | `flight_decoupled_l1_circle_20261002_134247` | first of 2 |
| WB-2 | `whole_body_l1_circle_v0p13_WB-2` | `flight_wb_l1_4d_circle_20260928_172242` | only run |
| DEC-5 | `geometric_l1_figure8_v0p10_DEC-5` | `flight_decoupled_l1_figure8_vel010_20261005_163734` | only run |
| WB-3 | `whole_body_l1_figure8_v0p10_WB-3` | `flight_wb_l1_4d_figure8_vel010_20261005_162406` | first of 2 |
| DEC-6 | `geometric_l1_figure8_v0p13_DEC-6` | `flight_decoupled_l1_figure8_vel013_20261005_165431` | only run |
| WB-5 | `whole_body_l1_figure8_v0p13_WB-5` | `flight_wb_l1_4d_figure8_vel013_20261005_170129` | first of 2 |

Every selected run: T650 aerial manipulator, total mass 3.746 kg, EKF2-fused OptiTrack feedback, the
planner's EE trajectory with the gripper heading along the tangent, q2 = 25 +- 15 deg four cycles per lap,
fold 55 deg, one lap. Circle: radius 0.50 m. Figure-8: 1.40 x 0.70 m, long axis on world x.
