#!/usr/bin/env python3
"""Free-flight trajectory tracking, EXPERIMENT: score the formal hardware runs from their ros2 bags,
select the best run of each setting, and write the paper table and the bag index.

    /usr/bin/python3 results/utils/experiment_tracking.py [--paper <main.tex>] [--copy-bags]

Writes results/experiment_results/flight_test_index.md (every aerial-manipulator flight-test bag, the
scored runs, which run fills each table cell) and, with --paper, the table labelled
tab:exp_free_flight_tracking in main.tex, and with --copy-bags copies the bag of every selected run to
results/experiment_results/ros2bags/<shape>_v<speed>/<method>/<bag name>/ (unchanged). Extracted bags are cached in $AM_EXP_CACHE
(default /tmp/am_experiment_cache), outside the repository.

Metrics: the hardware report's definitions (docs/docs_aerial_manipulator/archive/wb_vs_decoupled_flight_
20260928/tools/metrics.py), which the simulation table uses too. Each run is scored over its whole planner
run ('EXECUTING T=...' with T >= 20 s, the n-th of the bag, from its start to the next planner status),
trimmed 0.1 s at each end, 100 Hz grid, against the planner's streamed whole-body reference; the measured
EE is the planner model's FK of the EKF2-fused odometry and the joint encoders.

Selection: per (trajectory, speed, method), the ELIGIBLE run with the lowest EE position RMSE. Eligible =
the controller configuration the paper compares (whole-body: the 2026-09-27 tune with the arm
compensation; Geo-L1: the 2026-10-01 tune), the planned task (q2 = 25 +- 15 deg, four cycles per lap,
fold 55 deg), and a run that reached the end of its plan.

Interpreters: extraction needs rclpy + px4_msgs (ROS sourced, the user-site numpy 2); scoring needs
scipy (apt, numpy 1.21) -> stage 2 re-executes this file with PYTHONNOUSERSITE=1.
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.abspath(os.path.join(HERE, ".."))
REPO = os.path.abspath(os.path.join(RESULTS, ".."))
BAGS = os.path.join(REPO, "docs", "experimental_data_ros2_bag")
ARCH = os.path.join(REPO, "docs", "docs_aerial_manipulator", "archive")
TOOLS = os.path.join(ARCH, "wb_vs_decoupled_flight_20260928", "tools")
OUT_MD = os.path.join(RESULTS, "experiment_results", "flight_test_index.md")
BAG_COPIES = os.path.join(RESULTS, "experiment_results", "ros2bags")
CACHE = os.environ.get("AM_EXP_CACHE", "/tmp/am_experiment_cache")

# The formal free-flight comparison runs. key: (tag, date time, method, shape, speed, bag, n-th planner run,
#                                               eligible, note)
RUNS = {
    "w1": ("WB-1", "09-28 17:11", "whole_body_l1", "circle", 0.13, "flight_wb_l1_4d_circle_20260928_171154", 0, True, ""),
    "w2": ("WB-2", "09-28 17:22", "whole_body_l1", "circle", 0.13, "flight_wb_l1_4d_circle_20260928_172242", 0, True, ""),
    "d1": ("DEC-1", "09-28 18:28", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_20260928_182835", 0, False,
           "previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune"),
    "d2": ("DEC-2", "09-28 18:36", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_20260928_183608", 0, False,
           "previous Geo-L1 gain set (flown since 2026-08-21), superseded by the 2026-10-01 tune"),
    "r1": ("DEC-3", "10-02 13:42", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_20261002_134247", 0, True, ""),
    "r2": ("DEC-4", "10-02 13:42", "geometric_l1", "circle", 0.13, "flight_decoupled_l1_circle_20261002_134247", 1, False,
           "incomplete: the operator returned to SAFETY 0.8 s before the plan ended; flown through a "
           "ground-station clock step that reset the L1 estimate"),
    "w3": ("WB-3", "10-05 16:24", "whole_body_l1", "figure8", 0.10, "flight_wb_l1_4d_figure8_vel010_20261005_162406", 0, True, ""),
    "w4": ("WB-4", "10-05 16:24", "whole_body_l1", "figure8", 0.10, "flight_wb_l1_4d_figure8_vel010_20261005_162406", 1, True, ""),
    "r5": ("DEC-5", "10-05 16:37", "geometric_l1", "figure8", 0.10, "flight_decoupled_l1_figure8_vel010_20261005_163734", 0, True, ""),
    "r6": ("DEC-6", "10-05 16:54", "geometric_l1", "figure8", 0.13, "flight_decoupled_l1_figure8_vel013_20261005_165431", 0, True, ""),
    "w5": ("WB-5", "10-05 17:01", "whole_body_l1", "figure8", 0.13, "flight_wb_l1_4d_figure8_vel013_20261005_170129", 0, True, ""),
    "w6": ("WB-6", "10-05 17:01", "whole_body_l1", "figure8", 0.13, "flight_wb_l1_4d_figure8_vel013_20261005_170129", 1, True, ""),
}

# Every aerial-manipulator flight-test bag (the index). (date, bag, controller, what was flown, runs scored here)
INDEX = [
    ("09-12", "flight_wb_l1_20260912_190224", "whole-body L1 (6-D observer)", "first hardware flight of the whole-body law, short", "--"),
    ("09-12", "flight_wb_l1_20260912_201430", "whole-body L1 (6-D observer)", "hover and arm moves in DIRECT (development)", "--"),
    ("09-18", "flight_wb_l1_4d_20260918_144828", "whole-body L1 4-D", "first 4-D flight: planner legs (EE excursions, go-home, 0.61 m base step)", "--"),
    ("09-18", "flight_wb_l1_4d_20260918_152112", "whole-body L1 4-D", "metadata only, no data file", "--"),
    ("09-21", "flight_wb_l1_4d_circle_20260921_112328", "whole-body L1 4-D", "circle attempt (development)", "--"),
    ("09-21", "flight_wb_l1_4d_circle_20260921_115207", "whole-body L1 4-D", "circle attempt (development)", "--"),
    ("09-21", "flight_wb_l1_4d_circle_20260921_123637", "whole-body L1 4-D", "circle attempt: go-to-start only, mocap flips", "--"),
    ("09-24", "flight_wb_l1_4d_circle_20260924_120228", "whole-body L1 4-D", "circle, earlier tune, arm q2 30 +- 10 deg / 48 s; mocap freeze mid-run", "--"),
    ("09-24", "flight_wb_l1_4d_circle_20260924_120546", "whole-body L1 4-D", "circle, earlier tune, arm q2 30 +- 10 deg / 48 s", "--"),
    ("09-24", "flight_wb_l1_4d_circle_20260924_165654", "whole-body L1 4-D", "circle, earlier tune, arm q2 25 +- 15 deg / 12 s", "--"),
    ("09-24", "flight_wb_l1_4d_circle_20260924_172101", "whole-body L1 4-D", "circle, earlier tune, arm q2 25 +- 15 deg / 6 s", "--"),
    ("09-28", "flight_wb_l1_4d_circle_20260928_171154", "whole-body L1 4-D", "circle 0.13 m/s", "WB-1"),
    ("09-28", "flight_wb_l1_4d_circle_20260928_172242", "whole-body L1 4-D", "circle 0.13 m/s", "WB-2"),
    ("09-28", "flight_decoupled_l1_circle_20260928_182835", "Geo-L1 (previous gains)", "circle 0.13 m/s", "DEC-1"),
    ("09-28", "flight_decoupled_l1_circle_20260928_183608", "Geo-L1 (previous gains)", "circle 0.13 m/s", "DEC-2"),
    ("09-29", "flight_wb_l1_4d_ps4_20260929_160027", "whole-body L1 4-D", "PS4 teleoperation", "--"),
    ("10-02", "flight_decoupled_l1_circle_20261002_134247", "Geo-L1 (2026-10-01 tune)", "circle 0.13 m/s, two runs", "DEC-3, DEC-4"),
    ("10-02", "flight_decoupled_l1_circle_ps420261002_134830", "Geo-L1 (2026-10-01 tune)", "PS4 teleoperation", "--"),
    ("10-05", "flight_wb_l1_4d_figure8_vel010_20261005_162406", "whole-body L1 4-D", "figure-8 0.10 m/s, two runs", "WB-3, WB-4"),
    ("10-05", "flight_decoupled_l1_figure8_vel010_20261005_163734", "Geo-L1 (2026-10-01 tune)", "figure-8 0.10 m/s", "DEC-5"),
    ("10-05", "flight_decoupled_l1_figure8_vel013_20261005_165431", "Geo-L1 (2026-10-01 tune)", "figure-8 0.13 m/s", "DEC-6"),
    ("10-05", "flight_wb_l1_4d_figure8_vel013_20261005_170129", "whole-body L1 4-D", "figure-8 0.13 m/s, two runs", "WB-5, WB-6"),
]

KEYMAP = {"platform_x_mm": ("base_pos", "rms_xyz", 0), "platform_y_mm": ("base_pos", "rms_xyz", 1),
          "platform_z_mm": ("base_pos", "rms_xyz", 2), "platform_roll_deg": ("att", "rms", 0),
          "platform_pitch_deg": ("att", "rms", 1), "platform_heading_deg": ("att", "rms", 2),
          "ee_x_mm": ("ee_pos", "rms_xyz", 0), "ee_y_mm": ("ee_pos", "rms_xyz", 1), "ee_z_mm": ("ee_pos", "rms_xyz", 2),
          "q1_deg": ("joint", "rms", 0), "q2_deg": ("joint", "rms", 1), "q3_deg": ("joint", "rms", 2),
          "q4_deg": ("joint", "rms", 3)}


def bag_dir(name):
    hits = glob.glob(os.path.join(BAGS, "*", "*", name))
    if len(hits) != 1:
        raise SystemExit(f"bag {name}: found {hits}")
    return hits[0]


# ---------------------------------------------------------------- stage 1 (ROS python, numpy 2)
def extract():
    os.makedirs(CACHE, exist_ok=True)
    for bag in sorted({r[5] for r in RUNS.values()}):
        out = os.path.join(CACHE, bag + ".npz")
        if os.path.isfile(out):
            continue
        cmd = (f"source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && "
               f"/usr/bin/python3 extract_bag.py '{bag_dir(bag)}' '{out}' && /usr/bin/python3 np1_compat.py '{out}'")
        print("extracting", bag, flush=True)
        subprocess.run(["bash", "-c", cmd], cwd=TOOLS, check=True, stdout=subprocess.DEVNULL)
    for key, r in RUNS.items():
        link = os.path.join(CACHE, key + ".npz")
        if os.path.lexists(link):
            os.remove(link)
        os.symlink(r[5] + ".npz", link)


# ---------------------------------------------------------------- stage 2 (numpy 1.21 + scipy)
def score():
    os.environ["AM_NPZ"] = CACHE
    sys.path.insert(0, TOOLS)
    import common as C
    import metrics as M
    C.FLIGHTS.update({k: f"{v[0]} {v[1]}" for k, v in RUNS.items()})

    def windows(nm):
        d, t0 = C.load(nm)
        mk = "wbmode" if "wbmode__recv" in d.files else "gmode"
        md = C.edges(d, t0, mk)
        st = C.edges(d, t0, "pl_status")
        runs = [(t, st[i + 1][0] if i + 1 < len(st) else float(d[f"{mk}__recv"][-1] - t0))
                for i, (t, v) in enumerate(st) if v.startswith("EXECUTING T=") and float(v.split("=")[1].rstrip("s")) >= 20.0]
        c0, c1 = runs[RUNS[nm][6]]
        a = max(t for t, v in md if v == "DIRECT" and t < c0)
        b = next((t for t, v in md if v == "SAFETY" and t > c0), float(d[f"{mk}__recv"][-1] - t0))
        return a, b, c0, c1

    C.windows = windows
    M.windows = windows
    out = {}
    for key in RUNS:
        o, _ = M.analyse(key)
        m = o["metrics"]
        row = {c: float(m[g][f][i]) for c, (g, f, i) in KEYMAP.items()}
        row.update(ee_heading_deg=float(m["ee_head"]["rms"]), ee_pos_mm=float(m["ee_pos"]["rms_norm"]),
                   platform_pos_mm=float(m["base_pos"]["rms_norm"]), window=list(o["window"]),
                   tilt_max_deg=float(m["tilt_max_deg"]))
        out[key] = row
        print(f"{RUNS[key][0]:6s} window {o['window'][1] - o['window'][0]:6.2f} s  EE {row['ee_pos_mm']:6.1f} mm  "
              f"platform {row['platform_pos_mm']:6.1f} mm", flush=True)
    json.dump(out, open(os.path.join(CACHE, "scores.json"), "w"), indent=1)


# ---------------------------------------------------------------- outputs
def bag_copy_dir(key):
    r = RUNS[key]
    return os.path.join(BAG_COPIES, f"{r[3]}_v{r[4]:.2f}".replace(".", "p"), r[2], r[5])


def copy_bags(keys):
    for key in sorted(keys):
        src, dst = bag_dir(RUNS[key][5]), bag_copy_dir(key)
        if os.path.isdir(dst) and sorted(os.listdir(dst)) == sorted(os.listdir(src)) and all(
                os.path.getsize(os.path.join(dst, f)) == os.path.getsize(os.path.join(src, f)) for f in os.listdir(src)):
            continue
        if os.path.isdir(dst):
            shutil.rmtree(dst)
        shutil.copytree(src, dst)
        print("copied", RUNS[key][0], "->", os.path.relpath(dst, RESULTS), flush=True)


def write_outputs(paper, copy=False):
    sys.path.insert(0, HERE)
    import make_latex_table as T
    sc = json.load(open(os.path.join(CACHE, "scores.json")))
    chosen = {}
    for key, r in RUNS.items():
        if r[7]:
            cell = (r[3], f"{r[4]:.2f}", r[2])
            if cell not in chosen or sc[key]["ee_pos_mm"] < sc[chosen[cell]]["ee_pos_mm"]:
                chosen[cell] = key
    cells = {cell: sc[key] for cell, key in chosen.items()}
    tex = T.exp_table(cells)
    if paper:
        T.splice(paper, tex, T.EXP_LABEL, anchor="\\subsection{Experiment}\n\\subsubsection{Free Flight: Trajectory Tracking}")
        print("updated", paper)

    name = {"whole_body_l1": "Proposed (whole-body L1 4-D)", "geometric_l1": "Geo-L1"}
    shape = {"circle": "circle", "figure8": "figure-8"}
    sel = set(chosen.values())
    if copy:
        copy_bags(sel)
    L = ["# Aerial-manipulator flight tests (ros2 bags)", "",
         "Every aerial-manipulator flight-test bag in `docs/experimental_data_ros2_bag/`, and which runs fill the paper's",
         "experimental free-flight trajectory-tracking table (`tab:exp_free_flight_tracking`).",
         "Not listed: the robotic-arm calibration bench bags (09-09, 09-11) and the bare-T650 recordings (08-06, 08-07).", "",
         "Regenerate (re-scores from the bags, updates this file and the table):", "",
         "```bash", "/usr/bin/python3 results/utils/experiment_tracking.py --paper <paper>/main.tex", "```", "",
         "## 1. All flight-test bags", "",
         "| # | date | bag | controller | flown | scored runs |", "|---|---|---|---|---|---|"]
    for i, (date, bag, ctrl, what, runs) in enumerate(INDEX, 1):
        L.append(f"| {i} | {date} | `{bag}` | {ctrl} | {what} | {runs} |")
    L += ["", "Bag folders: `<date> - T650-AM .../<date> - T650-AM .../<bag>/` under `docs/experimental_data_ros2_bag/`.", "",
          "## 2. Scored runs of the formal free-flight comparison", "",
          "Each run is scored over its whole planned trajectory (speed-up and slow-down included), with the same",
          "definitions as the simulation table. **Selected** = the run in the paper table: the eligible run with the",
          "lowest end-effector position RMSE of its setting.", "",
          "| run | date | method | trajectory | speed [m/s] | EE position RMSE [mm] | platform position RMSE [mm] | EE heading RMSE [deg] | status |",
          "|---|---|---|---|---|---|---|---|---|"]
    for key, r in RUNS.items():
        s = sc[key]
        status = "**selected**" if key in sel else ("candidate" if r[7] else f"not eligible: {r[8]}")
        L.append(f"| {r[0]} | {r[1]} | {name[r[2]]} | {shape[r[3]]} | {r[4]:.2f} | {s['ee_pos_mm']:.2f} | "
                 f"{s['platform_pos_mm']:.2f} | {s['ee_heading_deg']:.2f} | {status} |")
    L += ["", "## 3. The experimental table: which run fills each setting", "",
          "| trajectory | speed [m/s] | Proposed | Geo-L1 |", "|---|---|---|---|"]
    for sh in ("circle", "figure8"):
        for v in (0.10, 0.13, 0.20):
            row = []
            for m in ("whole_body_l1", "geometric_l1"):
                k = chosen.get((sh, f"{v:.2f}", m))
                row.append(RUNS[k][0] if k else "not flown")
            L.append(f"| {shape[sh]} | {v:.2f} | {row[0]} | {row[1]} |")
    L += ["", "## 4. Copies of the selected bags", "",
          "The bag of every selected run, copied unchanged (folder and file names as recorded) to",
          "`results/experiment_results/ros2bags/<trajectory>_v<speed>/<method>/<bag>/`. Some bags hold two runs;",
          "the table uses the one named here (the n-th planner trajectory run in the bag).", "",
          "| run | copy | run in the bag |", "|---|---|---|"]
    for key in sorted(sel, key=lambda k: (RUNS[k][3], RUNS[k][4], RUNS[k][2])):
        r = RUNS[key]
        n = sum(1 for x in RUNS.values() if x[5] == r[5])
        where = f"{['first', 'second'][r[6]]} of {n}" if n > 1 else "only run"
        mark = "" if os.path.isdir(bag_copy_dir(key)) else " (not copied yet: run with --copy-bags)"
        L.append(f"| {r[0]} | `{os.path.relpath(bag_copy_dir(key), os.path.dirname(OUT_MD))}/`{mark} | {where} |")
    L += ["", "Every selected run: T650 aerial manipulator, total mass 3.746 kg, EKF2-fused OptiTrack feedback, the",
          "planner's EE trajectory with the gripper heading along the tangent, q2 = 25 +- 15 deg four cycles per lap,",
          "fold 55 deg, one lap. Circle: radius 0.50 m. Figure-8: 1.40 x 0.70 m, long axis on world x.", ""]
    os.makedirs(os.path.dirname(OUT_MD), exist_ok=True)
    open(OUT_MD, "w").write("\n".join(L))
    print("wrote", OUT_MD)
    for cell, key in sorted(chosen.items()):
        print(cell, "->", RUNS[key][0], f"EE {sc[key]['ee_pos_mm']:.2f} mm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--paper", default=None)
    ap.add_argument("--stage2", action="store_true")
    ap.add_argument("--copy-bags", action="store_true", help="copy the selected runs' bags into experiment_results/ros2bags")
    a = ap.parse_args()
    if a.stage2:
        score()
        return
    extract()
    env = dict(os.environ, PYTHONNOUSERSITE="1", AM_EXP_CACHE=CACHE)
    subprocess.run(["/usr/bin/python3", os.path.abspath(__file__), "--stage2"], env=env, check=True)
    write_outputs(a.paper, a.copy_bags)


if __name__ == "__main__":
    main()
