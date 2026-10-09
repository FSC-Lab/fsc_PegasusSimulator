"""README text for results/simulation_results/matlab_simulation_data and results/experiment_results/matlab_experiment_data.

Both READMEs are GENERATED (build_free_flight.py and experiment_tracking.py call these functions) so that
each MATLAB data folder is self-contained on a machine that has only that folder: file naming, the
run index, the conditions, the struct layout and the mapping to the paper's notation.
"""

METHOD_NAMES = {
    "whole_body_l1": ("Proposed", "whole-body L1 impedance control (this paper)"),
    "geometric_l1": ("Geo-L1", "geometric control with L1 adaptive augmentation, Cai et al., Control Eng. Pract. 164 "
                               "(2025) 106418; arm in position mode"),
    "modular_adaptive": ("MAC", "modular adaptive control, Yadav et al., IEEE/ASME Trans. Mechatronics 30(4) (2025)"),
}
SHAPE_NAMES = {"circle": "circle", "figure8": "figure-8"}


def _md_table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


STRUCT_COMMON = """\
Every run file holds one struct, `run`:

| field | content |
|---|---|
| `meta` | {meta} |
| `rmse` | this run's numbers: `platform_pos_xyz_mm` (x y z), `platform_pos_mm` (3-D norm), `platform_roll_deg`, `platform_pitch_deg`, `platform_heading_deg`, `ee_pos_xyz_mm`, `ee_pos_mm`, `ee_heading_deg`, `joints_deg` (q1..q4), plus `com_pos_mm`, `ee_pos_max_mm`, `tilt_max_deg` |
| `tracking` | 100 Hz over the trajectory run, `t` in s since the run started. Measured, reference (`_ref_`) and error (`_err_`) for the platform position (`platform_pos_*`) and attitude (`platform_quat_xyzw`, `platform_att_err_deg`, `platform_heading_*`, `platform_tilt_deg`), the system CoM (`com_pos_*`), the end-effector position (`ee_pos_*`) and heading (`ee_heading_*`), and the joints (`q_deg`, `q_ref_deg`, `q_err_deg`) |
| `raw` | {raw} |

Units: positions in m (world ENU, z up) unless the field name says mm; angles in deg unless the name says
rad. Errors are measured − reference. `platform_att_err_deg` columns are `[roll pitch heading]`.
"""

STRUCT_SIM = STRUCT_COMMON.format(
    meta="method, shape, nominal mean speed, lap time, laps, circle radius / figure-8 half-axes, arm sweep (q2 centre, "
         "amplitude, period, fold), plant description, controller yaml, driver arguments, flight time, minimum RTF, run window",
    raw="every recorded stream, time in s on the flight driver's clock: `odom` (position, velocity, attitude), `joints` "
        "(q, model convention), `reference` (the planner's whole-body reference: CoM chain, model-frame headings, EE "
        "position and heading, joint reference, airframe reference `x_b`), `planner_ee` (the planner's own current / "
        "reference EE), `arm_reference`, `controller_debug` (the method's debug array), `events` (the driver's event log)")

STRUCT_EXP = STRUCT_COMMON.format(
    meta="run tag, method, shape, nominal mean speed, original bag, which run in the bag, flight date, run window "
         "(s since the bag's first message), vehicle, feedback, trajectory, selection rule",
    raw="the bag's streams, time in s since the bag's first message: `odom` (EKF2-fused odometry: position, velocity, "
        "attitude, body rates), `joints` (q and qdot, model convention), `reference` (the planner's whole-body "
        "reference), `controller_debug` (`wb_control_debug` for the proposed method, `l1_control_debug` for Geo-L1), "
        "`motors` (PX4 motor commands, normalised), `battery` (voltage, current), `mocap_vrpn` (raw OptiTrack pose), "
        "`events.mode` / `events.planner_status` (transition times and labels)")

NOTATION = """\
## Paper notation

How the table's symbols map to the fields here. The RMSE appears in two places: the summary file
`free_flight_tracking_rmse.mat`, and each run file's `run.rmse`. The time series behind each RMSE is in
`run.tracking`, as measured − reference.

| paper symbol | meaning | summary file field | run file: `run.rmse.` | run file: `run.tracking.` |
|---|---|---|---|---|
| r<sub>0,x</sub>, r<sub>0,y</sub>, r<sub>0,z</sub> | platform position (body origin O<sub>0</sub>) in {I} [mm] | `platform_x_mm`, `platform_y_mm`, `platform_z_mm` | `platform_pos_xyz_mm(1:3)` | `platform_pos_err_m(:,1:3)` [m] |
| ψ<sub>0</sub>, φ<sub>0</sub>, θ<sub>0</sub> | platform heading, roll, pitch: body-axis components of the rotation vector of R<sub>0,d</sub><sup>T</sup> R<sub>0</sub> [deg] | `platform_heading_deg`, `platform_roll_deg`, `platform_pitch_deg` | same names | `platform_att_err_deg(:,3)`, `(:,1)`, `(:,2)` |
| r<sub>e,x</sub>, r<sub>e,y</sub>, r<sub>e,z</sub> | end-effector position in {I} [mm] | `ee_x_mm`, `ee_y_mm`, `ee_z_mm` | `ee_pos_xyz_mm(1:3)` | `ee_pos_err_m(:,1:3)` [m] |
| ψ<sub>e</sub> | end-effector heading: azimuth of b<sub>1,e</sub> [deg] | `ee_heading_deg` | `ee_heading_deg` | `ee_heading_err_deg` |
| q<sub>1</sub> … q<sub>4</sub> | joint angles, model convention [deg] | `q1_deg` … `q4_deg` | `joints_deg(1:4)` | `q_err_deg(:,1:4)` |

- **Reference:** every error is measured against the planner's dynamically compatible whole-body reference
  trajectory, which is the same for every method.
  - Platform position: x<sub>b</sub> = x<sub>c,d</sub> − R<sub>0,d</sub> r<sub>0c</sub>(q<sub>d</sub>).
  - Platform attitude R<sub>0,d</sub>: thrust along the CoM reference acceleration + g e<sub>3</sub>, heading
    from the reference. It is not the controller's internal attitude command.
  - End effector: position r<sub>e,d</sub>, heading b<sub>1,e,d</sub>. Joints: q<sub>d</sub>.
- **Measured end effector:** the planner model's forward kinematics of the measured platform pose and joint
  angles.
- **Window:** each RMSE covers the whole planned trajectory, including the 4 s speed-up and slow-down, with
  0.1 s trimmed at each end, on a 100 Hz grid.
- **Norms:** `platform_pos_mm` and `ee_pos_mm` are the RMSE of the 3-D error norm. They are not in the paper
  table but are useful for plots.
"""

TRAJECTORY = """\
## Trajectories

The planner generates one end-effector trajectory per setting, and every method receives the same reference.
- **Circle:** radius 0.50 m.
- **Figure-8:** a Gerono lemniscate, half-length 0.70 m and half-width 0.35 m (1.40 × 0.70 m), long axis on
  world x, path length 4.268 m per lap.
- **Timing:** one lap with a 4 s minimum-snap speed-up and slow-down. Lap time = path length / mean EE speed.
- **Heading:** the gripper heading follows the path tangent.
- **Arm sweep:** q2 = 25° ± 15°, four cycles per lap, fold q2 + q3 = 55°, q1 = 0. The airframe follows the
  compatible whole-body reference.

| mean EE speed [m/s] | circle lap [s] | figure-8 lap [s] |
|---|---|---|
| 0.10 | 31.42 | 42.68 |
| 0.13 | 24.17 | 32.83 |
| 0.20 | 15.71 | 21.34 |
"""


def sim_readme(summary, runs, attempts, plant, configs):
    """summary: mean_over_runs rows; runs: per-run rows; attempts: campaign.jsonl records; plant: [(key, value)]
    from the flown yaml; configs: config file names copied next to this README."""
    by_cell = {}
    for r in runs:
        by_cell.setdefault((r["shape"], f"{r['mean_speed_mps']:.2f}", r["method"]), []).append(r)
    att = {}
    for a in attempts:
        k = (a["shape"], f"{a['speed']:.2f}", a["method"])
        att.setdefault(k, {"n": 0, "pre_run": 0, "in_run": 0, "no_data": 0, "completed": 0})
        att[k]["n"] += 1
        att[k][a["status"]] += 1
    first = min(a["time"] for a in attempts) if attempts else "?"
    last = max(a["time"] for a in attempts) if attempts else "?"
    rows = []
    for shape in ("circle", "figure8"):
        for v in ("0.10", "0.13", "0.20"):
            for m in ("whole_body_l1", "geometric_l1", "modular_adaptive"):
                rr = sorted(by_cell.get((shape, v, m), []), key=lambda x: x["run"])
                s = next((x for x in summary if x["shape"] == shape and f"{x['mean_speed_mps']:.2f}" == v
                          and x["method"] == m), None)
                a = att.get((shape, v, m), {})
                folder = f"`free_flight_tracking/{shape}/v{v.replace('.', 'p')}/`"
                files = ", ".join(f"`{x['name']}.mat`" for x in rr) or "--"
                rows.append([SHAPE_NAMES[shape], v, METHOD_NAMES[m][0], folder, files,
                             " / ".join(f"{x['ee_pos_mm']:.1f}" for x in rr) or "--",
                             f"{s['ee_pos_mm']:.2f}" if s else "--",
                             f"{a.get('n', 0)} ({a.get('pre_run', 0)} hover trip{'' if a.get('pre_run', 0) == 1 else 's'}, {a.get('in_run', 0)} failed in run)"])
    meth = [[f"`{k}`", n[0], n[1]] for k, n in METHOD_NAMES.items()]
    tot = {}
    for a in attempts:
        t = tot.setdefault(a["method"], {"n": 0, "pre_run": 0, "in_run": 0})
        t["n"] += 1
        t[a["status"]] = t.get(a["status"], 0) + 1
    L = [
        "# Simulation results, MATLAB data (free-flight trajectory tracking)", "",
        "This is the data behind the paper's simulation table (Table II, `tab:sim_free_flight_tracking`).",
        "The folder is self-contained: these files plus MATLAB are all you need.", "",
        "## Files", "",
        "```",
        "free_flight_tracking/<shape>/v<speed>/<method>_<shape>_v<speed>_run<k>.mat   one run, struct `run`",
        "free_flight_tracking_rmse.mat     rmse_mean = Table II (mean over runs), rmse_runs = every run",
        "configs/                          the exact controller + plant config each method flew",
        "```", "",
        "- **`<shape>`:** `circle` or `figure8`.",
        "- **`v<speed>`:** the mean end-effector speed, e.g. `v0p13` = 0.13 m/s.",
        "- **`run<k>`:** the repeat, 1..3.",
        "- **`<method>`:** see the table below.", "",
        _md_table(["method key", "table name", "method"], meth), "",
        "```matlab",
        "r = load('free_flight_tracking/circle/v0p13/whole_body_l1_circle_v0p13_run1.mat').run;",
        "plot(r.tracking.t, vecnorm(r.tracking.ee_pos_err_m, 2, 2) * 1e3)        % EE position error [mm]",
        "plot3(r.tracking.ee_pos_ref_m(:,1), r.tracking.ee_pos_ref_m(:,2), r.tracking.ee_pos_ref_m(:,3), '--')",
        "S = load('free_flight_tracking_rmse.mat');",
        "T = struct2table(S.rmse_mean);     % Table II: one row per (shape, speed, method)",
        "R = struct2table(S.rmse_runs);     % every run",
        "```", "",
        # the pick-and-place data share this folder (build_pick_and_place.py writes that README; same text there)
        "**Payload pick-and-place** (the paper's `tab:sim_pick_place`): `pick_and_place/`, "
        "`pick_and_place_metrics.mat` and `configs/*_sim_pick_and_place.yaml`. See `README_pick_and_place.md`.", "",
        "## Simulation index", "",
        f"Flown {first} to {last}, in one campaign with one launch condition:",
        "- **Simulator:** Isaac Sim, headless, at real time (RTF 1.000 during every trajectory).",
        "- **Stack:** the same PX4 SITL, planner and arm stack for every method; raw mocap-emulator feedback.",
        "- **Repeats:** 3 runs per setting per method.",
        "- **Gains:** each method used ONE fixed config for all six settings (`configs/`); only the control law",
        "  differs between methods. Geo-L1's arm runs a position-mode servo stack, as in its paper.", "",
        "Each table cell is the mean over its 3 runs of the per-run RMSE.", "",
        _md_table(["trajectory", "speed [m/s]", "method", "folder", "files", "EE RMSE per run [mm]",
                   "mean (table) [mm]", "attempts"], rows), "",
        "Some attempts failed while hovering in DIRECT, before the trajectory started; they were re-flown. No",
        "trajectory failed after starting. Totals: " + "; ".join(
            f"{METHOD_NAMES[m][0]} {t.get('pre_run', 0)} of {t['n']} attempts" for m, t in tot.items()) + ".",
        "- **Proposed:** a growing roll oscillation at about 1.55 Hz, a simulator-only mode near the rotor-lag pole.",
        "- **MAC:** a roll and pitch divergence at about 2 Hz shortly after entering DIRECT.", "",
        TRAJECTORY,
        "## Plant (identical for every run and every method)", "",
        "The simulated plant mirrors the uncertainties identified from the hardware flights:",
        "- **Motor delay:** first-order rotor lag, λ = 10.03 s⁻¹.",
        "- **Model mismatch:** thrust coefficient ×1.037 and yaw-torque coefficient scaled at the plant; the",
        "  airframe CoM shifted 17.85 mm.",
        "- **Battery sag:** the thrust coefficient falls 3.6 %/min from lift-off.",
        "- **Standing wrench bias:** force (0.55, −0.50, 0) N and torque (0, 0, −0.095) N·m in the body frame.",
        "- **Imperfect joint actuation:**",
        "  - gearbox friction, ×(1.0, 0.70, 0.65, 1.5) per joint;",
        "  - current-loop residual, (22, 15, 24, 23) mN·m rms;",
        "  - reported joint velocity lagged 48 ms and quantised to 0.024 rad/s;",
        "  - joint torque limit 3 N·m.",
        "- **Feedback:** emulated motion capture at 60 Hz with noise.", "",
        "All plant keys, as flown (`sim_*` in `configs/whole_body_l1_4d_mirror_sim.yaml`; the same in all three):", "",
        "```"] + [f"{k}: {v}" for k, v in plant] + ["```", "",
        "Configs here: " + ", ".join(f"`configs/{c}`" for c in configs) + ".", "",
        "## The struct `run`", "", STRUCT_SIM, NOTATION]
    return "\n".join(L)


def exp_readme(sel_rows, scored_rows, index_rows, cells, not_flown):
    """sel_rows: [(file, tag, traj, speed, method, flown, bag, run_in_bag, ee_rmse)]; scored_rows: [(tag, flown,
    method, traj, speed, ee, platform, head, status)]; index_rows: [(n, date, bag, controller, flown, runs)];
    cells: [(traj, speed, proposed, geo)]."""
    L = [
        "# Experiment results, MATLAB data (free-flight trajectory tracking)", "",
        "This is the data behind the paper's experimental table (Table III, `tab:exp_free_flight_tracking`): the",
        "selected hardware run of each filled setting, converted from its ros2 bag. The folder is self-contained:",
        "these files plus MATLAB are all you need, with no ROS or custom message definitions.", "",
        "## Files", "",
        "```",
        "free_flight_tracking/<shape>/v<speed>/<method>_<shape>_v<speed>_<run>.mat   one run, struct `run`",
        "free_flight_tracking_rmse.mat     struct `rmse` = Table III (one row per selected run)",
        "```", "",
        "- **`<shape>` / `v<speed>`:** as in `matlab_simulation_data` (`v0p13` = 0.13 m/s).",
        "- **`<method>`:** `whole_body_l1` (Proposed) or `geometric_l1` (Geo-L1, position-mode arm).",
        "- **`<run>`:** the flight tag (WB-n, DEC-n) used in the lab's comparison reports.", "",
        "```matlab",
        "r = load('free_flight_tracking/circle/v0p13/whole_body_l1_circle_v0p13_WB-2.mat').run;",
        "plot(r.tracking.t, vecnorm(r.tracking.ee_pos_err_m, 2, 2) * 1e3)        % EE position error [mm]",
        "T = struct2table(load('free_flight_tracking_rmse.mat').rmse);           % Table III",
        "```", "",
        "## Experiment index", "",
        "### The runs in this folder (one per filled table cell)", "",
        "**Selection rule:** among the repeated flights of a setting, the completed run with the lowest",
        "end-effector position RMSE, flown with the controller version the paper compares.", "",
        _md_table(["file", "run", "trajectory", "speed [m/s]", "method", "flown", "original ros2 bag",
                   "run in the bag", "EE RMSE [mm]"], sel_rows), "",
        "The bags themselves are kept in `results/experiment_results/free_flight_tracking/<shape>/v<speed>/<same name>/`",
        "on the lab machine; the `.mat` files here hold everything the table and the plots need.", "",
        "### Which run fills each table cell", "",
        _md_table(["trajectory", "speed [m/s]", "Proposed", "Geo-L1"], cells), "",
        "Not flown on hardware (\"--\" in the table): " + ", ".join(not_flown) + ".", "",
        "### Every scored run of the formal comparison", "",
        "Same metric definitions as the simulation table.", "",
        _md_table(["run", "flown", "method", "trajectory", "speed [m/s]", "EE RMSE [mm]", "platform RMSE [mm]",
                   "EE heading RMSE [deg]", "status"], scored_rows), "",
        "### Conditions of the selected runs", "",
        "- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm, total mass 3.746 kg.",
        "- **Feedback:** OptiTrack fused in PX4 EKF2, about 100 Hz; joint encoders.",
        "- **Proposed:**",
        "  - the 2026-09-27 tune: k_x / k_v 50.03 / 12.58, k_R / k_ω 2.134 / 1.567, K_y / D_y 211.9 / 26.82,",
        "    K_ψ / D_ψ 0.2484 / 0.2903;",
        "  - L1 bandwidth ω_c 2.927 / 0.7428 / 0.8479 rad/s (translation / rotation / arm);",
        "  - the arm in torque mode with friction feed-forward and the velocity observer.",
        "- **Geo-L1:**",
        "  - the 2026-10-01 tune: K_p 20.11 (x, y) / 13.5 (z), K_v 11.05 / 10.82, k_R 3.337 / 1.737,",
        "    k_ω 0.9505 / 0.4105;",
        "  - L1: A_s 4.334 / 2.791, ω_c 1.0 rad/s;",
        "  - the arm in position mode.",
        "- **Allocator thrust coefficient:** 4.260431e-05 for both.", "",
        "### All aerial-manipulator flight-test bags (for reference)", "",
        "Arm-calibration bench bags and bare-T650 recordings are not listed.", "",
        _md_table(["#", "date", "bag", "controller", "flown", "scored runs"], index_rows), "",
        TRAJECTORY.replace("## Trajectories", "## Trajectories (hardware runs: 0.10 and 0.13 m/s only)"),
        "## The struct `run`", "", STRUCT_EXP, NOTATION]
    return "\n".join(L)
