#!/usr/bin/env python3
"""Free-flight trajectory tracking UNDER DISTURBANCES, EXPERIMENT: the 2026-11-05 robust-tracking flights (circle,
0.13 m/s) -> MATLAB, laid out like matlab_experiment_data/free_flight_tracking/.

    /usr/bin/python3 results/utils/experiment_robust_tracking.py [--copy-bags]

Two disturbance studies, one task folder each, under results/experiment_results/:
  free_flight_model_uncertainty/   a 50 g or 100 g mass on the end effector that the controller does not model
  free_flight_wind_disturbance/    5 m/s of wind from a fan 4 m from the circle's centre
Written per task:
  matlab_experiment_data/<task>/[<setting>/]<method>_circle_v0p13_<setting>_run<k>.mat   one LAP, struct `run`
  matlab_experiment_data/<task>/<task>_rmse.mat   rmse_mean (per method, mean over the laps), rmse_runs (each lap),
                                                  rmse_flights (each bag's two-lap run as a whole), rmse_nominal
                                                  (the undisturbed circle 0.13 m/s entry of free_flight_tracking),
                                                  bags (every bag of the study, with what the download holds)
  matlab_experiment_data/<task>/configs/, README.md
  flight_test_data/<task>/[<setting>/]<method>_circle_v0p13_<setting>_<lap tags>/   (--copy-bags) each bag, files
                                                  unchanged; a metadata.yaml missing from the download is rebuilt
                                                  in the COPY by `ros2 bag reindex` (the original is never touched)

The bags are DISCOVERED by name anywhere under docs/experimental_data_ros2_bag/<folder>/<subfolder>/:
flight_<wb_l1_4d|decoupled_l1>_circle_013_<50g|100g|wind>_<date>_<time>. A bag whose .db3 is not on disk (the
Google Drive download is split into several zip parts) is listed as missing; re-run after unpacking the other parts
into the same folder and it is scored too.

Each flight (bag) flies ONE planner run of TWO laps back to back ("circle x2", T = 52.33 s, lap 24.17 s): unlike the
nominal campaign (two separate one-lap runs per bag). The laps are split where the planner says the first lap ends
(whole_body_planner/ee_trajectory/info[15], bisected on its own time profile): lap 1 = run start -> that time
(the 2 s speed-up + one lap), lap 2 = that time -> run end (one lap + the 2 s slow-down). Each lap is scored like a
nominal run (the hardware report's definitions, 0.1 s trimmed at each end, 100 Hz) and counts as one repeated test.
A setting's entry is the mean over all its laps (two bags x two laps = four).

Interpreters as experiment_tracking.py: stage 1 needs ROS (rclpy, px4_msgs), stage 2 re-executes this file with
PYTHONNOUSERSITE=1 (scipy, numpy 1.21).
"""
import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import experiment_tracking as ET   # noqa: E402

RESULTS, EXP, BAGS, TOOLS = ET.RESULTS, ET.EXP, ET.BAGS, ET.TOOLS
FLIGHT, MAT_DIR, CACHE = ET.FLIGHT, ET.MAT_DIR, ET.CACHE
NAME = re.compile(r"^flight_(wb_l1_4d|decoupled_l1)_circle_013_(50g|100g|wind)_(\d{8})_(\d{6})$")
METHOD = {"wb_l1_4d": "whole_body_l1", "decoupled_l1": "geometric_l1"}
MSHORT = {"whole_body_l1": "WB", "geometric_l1": "DEC"}
MNAME = {"whole_body_l1": "Proposed", "geometric_l1": "Geo-L1"}
TASKS = {
    "free_flight_model_uncertainty": dict(
        title="free-flight trajectory tracking with an unmodelled end-effector mass",
        what="a mass on the end effector that the controller's model does not include"),
    "free_flight_wind_disturbance": dict(
        title="free-flight trajectory tracking in wind",
        what="wind from a fan, 5 m/s, placed 4 m from the circle's centre"),
}
SETTINGS = {   # bag-name token -> the setting
    "50g": dict(key="ee50g", task="free_flight_model_uncertainty", sub="ee_mass_50g", label="EE +50 g",
                long="a 50 g mass on the end effector, not in the controller model", ee_mass_kg=0.05),
    "100g": dict(key="ee100g", task="free_flight_model_uncertainty", sub="ee_mass_100g", label="EE +100 g",
                 long="a 100 g mass on the end effector, not in the controller model", ee_mass_kg=0.10),
    "wind": dict(key="wind", task="free_flight_wind_disturbance", sub="", label="wind 5 m/s",
                 long="5 m/s wind from a fan placed 4 m from the circle's centre", wind_speed_mps=5.0,
                 fan_distance_m=4.0),
}
SETTING_BY_KEY = {s["key"]: s for s in SETTINGS.values()}
LAPS = 2                       # laps per flight (one planner run)
EXPECTED_FLIGHTS = 2           # per method and setting: two bags = four laps (the session's design)
# The configuration flown on 2026-11-05: the final campaign's hardware yamls, byte-identical (git diff a316625 ->
# dev_robotic_arm is empty for both); the node was built from fsc_autopilot_ros2 dev_robotic_arm (it publishes the
# 150-value wb_control_debug added there 2026-11-03), whose later features the free-flight yaml leaves off.
CONFIGS = {
    "whole_body_l1": "whole-body L1 4-D, the final campaign's hardware yaml unchanged (2026-09-27 tune + the 2026-10-16 "
                     "wrist-roll fix; fsc_autopilot_ros2 05c9cdb); node built from the dev_robotic_arm branch",
    "geometric_l1": "Geo-L1, the 2026-10-01 tune unchanged (fsc_autopilot_ros2 d85be8a); node built from the "
                    "dev_robotic_arm branch",
}
CONFIG_FILES = [c for c in ET.CONFIG_FILES]            # the same four files as the nominal table


# ---------------------------------------------------------------- the bags
def discover():
    """Every robust-tracking bag under BAGS, in flight order, with its tags. A bag folder present twice (a re-unpacked
    zip) counts once, preferring the copy that holds the .db3."""
    seen = {}
    for d in glob.glob(os.path.join(BAGS, "*", "*", "flight_*_circle_013_*")):
        m = NAME.match(os.path.basename(d))
        if not m or not os.path.isdir(d):
            continue
        db3 = sorted(glob.glob(os.path.join(glob.escape(d), "*.db3")))    # the folders carry "[Success]"
        f = dict(bag=m.group(0), dir=d, method=METHOD[m.group(1)], token=m.group(2),
                 setting=SETTINGS[m.group(2)]["key"], date=m.group(3), time=m.group(4),
                 flown=f"{m.group(3)[4:6]}-{m.group(3)[6:8]} {m.group(4)[:2]}:{m.group(4)[2:4]}",
                 present=bool(db3), has_meta=os.path.isfile(os.path.join(d, "metadata.yaml")),
                 db3=[os.path.basename(x) for x in db3])
        old = seen.get(f["bag"])
        if old is None or (f["present"] and not old["present"]):
            seen[f["bag"]] = f
    out = sorted(seen.values(), key=lambda f: (f["date"], f["time"]))
    count = {}
    for f in out:
        c = (f["method"], f["setting"])
        count[c] = count.get(c, 0) + 1
        f["flight_no"] = count[c]                                    # 1, 2 ... per method and setting
        s = SETTINGS[f["token"]]
        f["lap_tags"] = [f"{MSHORT[f['method']]}-{s['key'].replace('ee', '')}-{LAPS * (count[c] - 1) + j}"
                         for j in (1, 2)]
        f["key"] = f"rt{f['date'][4:]}{f['time']}"
    return out


def lap_key(f, lap):
    return f"{f['key']}L{lap}"


def whole_key(f):
    return f"{f['key']}W"


def base_name(f):
    return f"{f['method']}_circle_v0p13_{f['setting']}"


def task_dir(root, f):
    s = SETTINGS[f["token"]]
    return os.path.join(root, s["task"], s["sub"]) if s["sub"] else os.path.join(root, s["task"])


# ---------------------------------------------------------------- stage 1 (ROS python, numpy 2)
def extract(flights):
    os.makedirs(CACHE, exist_ok=True)
    for f in flights:
        if not f["present"]:
            continue
        out = os.path.join(CACHE, f["bag"] + ".npz")
        if not os.path.isfile(out):
            cmd = (f"source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && "
                   f"/usr/bin/python3 extract_bag.py '{f['dir']}' '{out}' && /usr/bin/python3 np1_compat.py '{out}'")
            print("extracting", f["bag"], flush=True)
            subprocess.run(["bash", "-c", cmd], cwd=TOOLS, check=True, stdout=subprocess.DEVNULL)
        for key in (lap_key(f, 1), lap_key(f, 2), whole_key(f)):
            link = os.path.join(CACHE, key + ".npz")
            if os.path.lexists(link):
                os.remove(link)
            os.symlink(f["bag"] + ".npz", link)


# ---------------------------------------------------------------- stage 2 (numpy 1.21 + scipy)
def run_windows(C, f):
    """The bag's EE-trajectory run: DIRECT span (a, b), run start/end (c0, c1), and the planner's numbers."""
    import numpy as np
    d, t0 = C.load(whole_key(f))
    mk = "wbmode" if "wbmode__recv" in d.files else "gmode"
    md = C.edges(d, t0, mk)
    st = C.edges(d, t0, "pl_status")
    end = float(d[f"{mk}__recv"][-1] - t0)
    runs = [(t, st[i + 1][0] if i + 1 < len(st) else end, float(v.split("=")[1].rstrip("s")),
             st[i + 1][1] if i + 1 < len(st) else "")
            for i, (t, v) in enumerate(st) if v.startswith("EXECUTING T=") and float(v.split("=")[1].rstrip("s")) >= 10.0]
    if len(runs) != 1:
        raise SystemExit(f"{f['bag']}: expected one EE-trajectory run of >= 10 s, found {len(runs)}")
    c0, c1, T_status, after = runs[0]
    a = max(t for t, v in md if v == "DIRECT" and t < c0)
    b = next((t for t, v in md if v == "SAFETY" and t > c0), end)
    I = d["ee_info2__data"]
    ti = d["ee_info2__recv"] - t0
    rows = [k for k in range(len(ti)) if ti[k] <= c0 and np.isfinite(I[k, 15])]
    if not rows:
        raise SystemExit(f"{f['bag']}: no ee_trajectory/info before the run")
    r = I[rows[-1]]
    info = dict(T_total=float(r[2]), T_lap=float(r[3]), laps=int(round(r[4])), ramp=float(r[5]), s=float(r[0]),
                lap_end_in_run=float(r[15]))
    if info["laps"] != LAPS:
        raise SystemExit(f"{f['bag']}: the run flew {info['laps']} laps, expected {LAPS}")
    complete = after.startswith("HOLD") and abs((c1 - c0) - info["T_total"]) < 0.5
    return dict(a=a, b=b, c0=c0, c1=c1, tl=c0 + info["lap_end_in_run"], complete=complete, after=after, **info)


def score(flights):
    import numpy as np
    os.environ["AM_NPZ"] = CACHE
    sys.path.insert(0, TOOLS)
    import common as C
    import metrics as M
    present = [f for f in flights if f["present"]]
    W, RW = {}, {}
    for f in present:
        w = run_windows(C, f)
        RW[f["bag"]] = w
        W[lap_key(f, 1)] = (w["a"], w["b"], w["c0"], w["tl"])
        W[lap_key(f, 2)] = (w["a"], w["b"], w["tl"], w["c1"])
        W[whole_key(f)] = (w["a"], w["b"], w["c0"], w["c1"])
        for j, key in ((1, lap_key(f, 1)), (2, lap_key(f, 2))):
            C.FLIGHTS[key] = f"{f['lap_tags'][j - 1]} {f['flown']}"
        C.FLIGHTS[whole_key(f)] = f"{f['lap_tags'][0]}+{f['lap_tags'][1]} {f['flown']}"
    C.windows = lambda nm: W[nm]
    M.windows = C.windows

    def row(key):
        o, _ = M.analyse(key)
        m = o["metrics"]
        r = {c: float(m[g][fld][i]) for c, (g, fld, i) in ET.KEYMAP.items()}
        r.update(ee_heading_deg=float(m["ee_head"]["rms"]), ee_pos_mm=float(m["ee_pos"]["rms_norm"]),
                 platform_pos_mm=float(m["base_pos"]["rms_norm"]), window=list(o["window"]),
                 tilt_max_deg=float(m["tilt_max_deg"]))
        return r

    sc = {}
    for f in present:
        w = RW[f["bag"]]
        for j in (1, 2):
            sc[lap_key(f, j)] = row(lap_key(f, j))
        sc[whole_key(f)] = row(whole_key(f))
        print(f"{f['bag']}: run {w['c1'] - w['c0']:.2f} s (planned {w['T_total']:.2f}, then {w['after'] or 'end'}), "
              f"lap 1 {w['tl'] - w['c0']:.2f} s / lap 2 {w['c1'] - w['tl']:.2f} s; EE RMSE lap 1 "
              f"{sc[lap_key(f, 1)]['ee_pos_mm']:.1f}, lap 2 {sc[lap_key(f, 2)]['ee_pos_mm']:.1f}, both "
              f"{sc[whole_key(f)]['ee_pos_mm']:.1f} mm", flush=True)
    json.dump(dict(scores=sc, windows={k: dict(v) for k, v in RW.items()}, nominal=nominal_rows()),
              open(os.path.join(CACHE, "robust_tracking_scores.json"), "w"), indent=1)

    # the files: register each lap in experiment_tracking's tables, then reuse its writer
    cells = cell_laps(flights, RW)
    written = set()
    for (method, setting), laps in sorted(cells.items()):
        tags = [t for _, _, t in laps]
        for f, j, tag in laps:          # every lap of the setting first: the writer names the laps averaged with it
            ET.RUNS[lap_key(f, j)] = (tag, f["flown"], f["method"], "circle", 0.13, f["bag"], 0, True, "",
                                      "rt_" + f["method"])
            ET.CONFIGS["rt_" + f["method"]] = CONFIGS[f["method"]]
        for k, (f, j, tag) in enumerate(laps, 1):
            key = lap_key(f, j)
            written.add(os.path.join(task_dir(MAT_DIR, f), f"{base_name(f)}_run{k}.mat"))
            w = RW[f["bag"]]
            s = SETTING_BY_KEY[setting]
            name = f"{base_name(f)}_run{k}"
            extra = dict(
                name=name, setting=setting, setting_label=s["label"], disturbance=s["long"],
                averaged_with=", ".join(tags), runs_in_mean=len(laps), lap=j, laps_in_planner_run=LAPS,
                run_in_bag=1, planner_run_start_s=w["c0"], planner_run_end_s=w["c1"],
                planner_run_duration_s=w["T_total"], lap_time_s=w["T_lap"], ramp_time_s=w["ramp"],
                lap_start_s=W[key][2], lap_end_s=W[key][3],
                trajectory="circle, radius 0.50 m, 0.13 m/s (lap 24.17 s); q2 = 25 +- 15 deg four cycles per lap, fold "
                           "55 deg; TWO laps back to back in one planner run. This file is lap "
                           f"{j}: " + ("the 2 s speed-up and the first lap" if j == 1 else
                                       "the second lap and the 2 s slow-down"),
                clock="raw streams: s since the bag's first message; tracking.t: s since this lap started "
                      "(meta.run_start_s = meta.lap_start_s)",
                flown=f"2026-{f['flown']}",
                selection=f"one of the {len(laps)} laps averaged in this setting's entry: every lap of the setting's "
                          "2026-11-05 flights (each flight = one planner run of two laps)",
                **{k_: v for k_, v in s.items() if k_.endswith(("_kg", "_mps", "_m"))})
            path = os.path.join(task_dir(MAT_DIR, f), name + ".mat")
            ET.export_mat(C, key, sc[key], k, [lap_key(x, i) for x, i, _ in laps], path=path, meta_extra=extra)
    for task in TASKS:                  # lap files of an earlier selection (tags shift when a bag is added)
        for p in glob.glob(os.path.join(MAT_DIR, task, "**", "*_run*.mat"), recursive=True):
            if p not in written:
                os.remove(p)
                print("removed", os.path.relpath(p, RESULTS), flush=True)
    export_summaries(flights, sc, RW, cells)


def cell_laps(flights, RW):
    """(method, setting) -> [(flight, lap, tag)] in flight order: every lap of the complete runs."""
    cells = {}
    for f in flights:
        if f["present"] and RW[f["bag"]]["complete"]:
            for j in (1, 2):
                cells.setdefault((f["method"], f["setting"]), []).append((f, j, f["lap_tags"][j - 1]))
    return cells


def nominal_rows():
    """The undisturbed reference: free_flight_tracking's circle 0.13 m/s entries (its summary file)."""
    import scipy.io
    S = scipy.io.loadmat(os.path.join(ET.FF_MAT, "free_flight_tracking_rmse.mat"), squeeze_me=True)["rmse_mean"]
    out = []
    for i in range(len(S["shape"].item())):
        if str(S["shape"].item()[i]) == "circle" and abs(float(S["mean_speed_mps"].item()[i]) - 0.13) < 1e-6:
            r = dict(method=str(S["method"].item()[i]), runs=float(S["runs"].item()[i]), tags=str(S["tags"].item()[i]))
            r.update({c: float(S[c].item()[i]) for c in ET.TABLE_COLS + ["tilt_max_deg"]})
            out.append(r)
    return sorted(out, key=lambda r: list(MSHORT).index(r["method"]))


def export_summaries(flights, sc, RW, cells):
    import numpy as np
    import scipy.io

    def colstruct(rows):
        out = {}
        for c in rows[0]:
            v = [x[c] for x in rows]
            out[c] = np.array(v, dtype=object) if isinstance(v[0], str) else np.array(v, dtype=float)
        return out

    cols = ET.TABLE_COLS + ["tilt_max_deg"]
    nominal = nominal_rows()
    for task in TASKS:
        fl = [f for f in flights if SETTINGS[f["token"]]["task"] == task]
        if not fl:
            continue
        means, runs, whole, bags = [], [], [], []
        for (method, setting), laps in sorted(cells.items()):
            if SETTING_BY_KEY[setting]["task"] != task:
                continue
            for k, (f, j, tag) in enumerate(laps, 1):
                runs.append(dict(name=f"{base_name(f)}_run{k}", tag=tag, setting=setting, method=method,
                                 flown=f"2026-{f['flown']}", bag=f["bag"], lap=float(j),
                                 **{c: sc[lap_key(f, j)][c] for c in cols}))
            m = dict(setting=setting, setting_label=SETTING_BY_KEY[setting]["label"], method=method,
                     runs=float(len(laps)), tags=", ".join(t for _, _, t in laps))
            m.update({c: float(np.mean([sc[lap_key(f, j)][c] for f, j, _ in laps])) for c in cols})
            means.append(m)
        for f in fl:
            w = RW.get(f["bag"])
            bags.append(dict(bag=f["bag"], flown=f"2026-{f['flown']}", method=f["method"], setting=f["setting"],
                             laps=f"{f['lap_tags'][0]}, {f['lap_tags'][1]}",
                             status=("scored" if w and w["complete"] else "incomplete run, not averaged" if w else
                                     "db3 not in the download"),
                             metadata_yaml_in_download=float(f["has_meta"])))
            if w:
                whole.append(dict(bag=f["bag"], tags=f"{f['lap_tags'][0]}+{f['lap_tags'][1]}", setting=f["setting"],
                                  method=f["method"], flown=f"2026-{f['flown']}", complete=float(w["complete"]),
                                  run_s=w["c1"] - w["c0"], **{c: sc[whole_key(f)][c] for c in cols}))
        out = dict(bags=colstruct(bags), rmse_nominal=colstruct(nominal))
        if means:
            out.update(rmse_mean=colstruct(means), rmse_runs=colstruct(runs), rmse_flights=colstruct(whole))
        path = os.path.join(MAT_DIR, task, f"{task}_rmse.mat")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        scipy.io.savemat(path, out, oned_as="column")
        print("wrote", os.path.relpath(path, RESULTS), flush=True)


# ---------------------------------------------------------------- outputs (system python)
def copy_bags(flights):
    """flight_test_data/<task>/[<setting>/]<base>_<lap tags>/ <- the bag, files unchanged. A metadata.yaml the download
    lacks is rebuilt in the copy with `ros2 bag reindex` (marked by REINDEXED.txt) and replaced by the original once
    the original is on disk."""
    keep = {os.path.join(task_dir(FLIGHT, f), f"{base_name(f)}_{'+'.join(f['lap_tags'])}") for f in flights if f["present"]}
    for task in TASKS:                  # copies of an earlier selection (the tags shift when a bag is added)
        for d in glob.glob(os.path.join(glob.escape(os.path.join(FLIGHT, task)), "**", "*_circle_v0p13_*"),
                           recursive=True):
            if os.path.isdir(d) and d not in keep:
                shutil.rmtree(d)
                print("removed", os.path.relpath(d, RESULTS), flush=True)
    for f in flights:
        if not f["present"]:
            continue
        dst = os.path.join(task_dir(FLIGHT, f), f"{base_name(f)}_{'+'.join(f['lap_tags'])}")
        src_files = sorted(os.listdir(f["dir"]))
        same = os.path.isdir(dst) and all(
            os.path.isfile(os.path.join(dst, x)) and os.path.getsize(os.path.join(dst, x)) ==
            os.path.getsize(os.path.join(f["dir"], x)) for x in src_files)
        if same and (f["has_meta"] or os.path.isfile(os.path.join(dst, "metadata.yaml"))) and \
                not (f["has_meta"] and os.path.isfile(os.path.join(dst, "REINDEXED.txt"))):
            continue
        if os.path.isdir(dst):
            shutil.rmtree(dst)
        shutil.copytree(f["dir"], dst)
        if not f["has_meta"]:
            subprocess.run(["bash", "-c", f"source /opt/ros/humble/setup.bash && ros2 bag reindex '{dst}' -s sqlite3"],
                           check=True, stdout=subprocess.DEVNULL)
            open(os.path.join(dst, "REINDEXED.txt"), "w").write(
                "metadata.yaml was not in the downloaded zip part that held this bag's .db3: it was rebuilt here with\n"
                "`ros2 bag reindex <this folder> -s sqlite3` (the .db3 is unchanged). Re-run\n"
                "results/utils/experiment_robust_tracking.py --copy-bags once the original is on disk to replace it.\n")
        print("copied", f["bag"], "->", os.path.relpath(dst, RESULTS), flush=True)


def copy_configs(task):
    out = []
    os.makedirs(os.path.join(MAT_DIR, task, "configs"), exist_ok=True)
    for dst, repo, path in CONFIG_FILES:
        rev = subprocess.run(["git", "-C", repo, "log", "-1", "--format=%h %ad", "--date=short", "--", path],
                             check=True, capture_output=True, text=True).stdout.split()
        text = subprocess.run(["git", "-C", repo, "show", f"{rev[0]}:{path}"], check=True, capture_output=True,
                              text=True).stdout
        with open(os.path.join(MAT_DIR, task, "configs", dst), "w") as fh:
            fh.write(f"# {os.path.basename(repo)} {rev[0]} ({rev[1]}): {path}, as flown on 2026-11-05 (the final "
                     "campaign's file, unchanged)\n" + text)
        out.append((dst, f"{os.path.basename(repo)} {rev[0]} ({rev[1]})"))
    return out


def write_readmes(flights):
    from readme_text import _md_table, STRUCT_EXP
    data = json.load(open(os.path.join(CACHE, "robust_tracking_scores.json")))
    sc, RW = data["scores"], data["windows"]
    cells = cell_laps(flights, RW)
    nominal = {r["method"]: r for r in data["nominal"]}    # computed in stage 2 (scipy is not importable here)
    for task, tinfo in TASKS.items():
        fl = [f for f in flights if SETTINGS[f["token"]]["task"] == task]
        if not fl:
            continue
        configs = copy_configs(task)
        settings = [s for s in SETTINGS.values() if s["task"] == task]
        f1 = lambda x: f"{x:.1f}"           # noqa: E731
        # the entries: mean over the laps, against the nominal
        res = []
        for m in ("whole_body_l1", "geometric_l1"):
            n = nominal.get(m)
            if n:
                res.append([MNAME[m], "none (nominal, 2026-10-28)", f"{int(n['runs'])} ({n['tags']})", f1(n["ee_pos_mm"]),
                            f"{n['ee_heading_deg']:.2f}", f1(n["platform_pos_mm"]), f"{n['platform_heading_deg']:.2f}",
                            f"{n['tilt_max_deg']:.1f}"])
            for s in settings:
                laps = cells.get((m, s["key"]), [])
                if not laps:
                    have = [f for f in fl if f["method"] == m and f["setting"] == s["key"]]
                    res.append([MNAME[m], s["label"], "0 (no bag in the download)" if not have
                                else "0 (bag(s) in the download without their .db3)",
                                "--", "--", "--", "--", "--"])
                    continue
                v = lambda c: sum(sc[lap_key(f, j)][c] for f, j, _ in laps) / len(laps)   # noqa: E731
                n_s = f"{len(laps)} of {LAPS * EXPECTED_FLIGHTS}" + (" (**incomplete**)" if len(laps) < LAPS * EXPECTED_FLIGHTS else "")
                res.append([MNAME[m], s["label"], f"{n_s} ({', '.join(t for _, _, t in laps)})", f1(v("ee_pos_mm")),
                            f"{v('ee_heading_deg'):.2f}", f1(v("platform_pos_mm")), f"{v('platform_heading_deg'):.2f}",
                            f"{max(sc[lap_key(f, j)]['tilt_max_deg'] for f, j, _ in laps):.1f}"])
        laps_rows, files_rows, bag_rows = [], [], []
        for (m, s), laps in sorted(cells.items()):
            if SETTING_BY_KEY[s]["task"] != task:
                continue
            for k, (f, j, tag) in enumerate(laps, 1):
                r = sc[lap_key(f, j)]
                rel = os.path.join(SETTING_BY_KEY[s]["sub"], f"{base_name(f)}_run{k}.mat") if SETTING_BY_KEY[s]["sub"] \
                    else f"{base_name(f)}_run{k}.mat"
                laps_rows.append([f"`{rel}`", tag, MNAME[m], SETTING_BY_KEY[s]["label"], f"2026-{f['flown']}",
                                  str(j), f"{r['window'][1] - r['window'][0]:.2f}", f1(r["ee_pos_mm"]),
                                  f"{r['ee_heading_deg']:.2f}", f1(r["platform_pos_mm"])])
        for f in fl:
            w = RW.get(f["bag"])
            sub = SETTINGS[f["token"]]["sub"]
            copy = f"`{(sub + '/') if sub else ''}{base_name(f)}_{'+'.join(f['lap_tags'])}/`"
            if not f["present"]:
                status, copy = "**.db3 not in the download** (another zip part)", "--"
            elif not w["complete"]:
                status = "run incomplete: not averaged"
            else:
                status = "scored" + ("" if f["has_meta"] else "; metadata.yaml rebuilt in the copy (not in the download)")
            bag_rows.append([f"2026-{f['flown']}", f"`{f['bag']}`", MNAME[f["method"]], SETTINGS[f["token"]]["label"],
                             ", ".join(f["lap_tags"]), status, copy,
                             f1(sc[whole_key(f)]["ee_pos_mm"]) if w else "--"])
        missing = [f for f in fl if not f["present"]]
        short = [f"{MNAME[m]} {SETTING_BY_KEY[s]['label']}: {len(l)} of {LAPS * EXPECTED_FLIGHTS} laps"
                 for (m, s), l in sorted(cells.items()) if SETTING_BY_KEY[s]["task"] == task
                 and len(l) < LAPS * EXPECTED_FLIGHTS]
        example = next((r[0].strip("`") for r in laps_rows if "whole_body_l1" in r[0]), laps_rows[0][0].strip("`")
                       if laps_rows else f"{task}_rmse.mat")
        layout = ("<setting>/<method>_circle_v0p13_<setting>_run<k>.mat" if any(s["sub"] for s in settings)
                  else "<method>_circle_v0p13_<setting>_run<k>.mat")
        L = [f"# Experiment results, MATLAB data ({tinfo['title']})", "",
             f"Circle tracking at 0.13 m/s with {tinfo['what']}, flown on 2026-11-05, converted from the hardware ros2",
             "bags. Same task, configuration and metric definitions as `../free_flight_tracking/` (the undisturbed",
             "circle 0.13 m/s entry is the reference row below), so a script written for those files runs on these.",
             "This folder is self-contained: these files plus MATLAB are all you need.", ""]
        if missing or short:
            L += ["> **Incomplete download.** The Google Drive download of the 11-05 bags is split into several zip parts",
                  "> and only the first part is on disk."]
            if missing:
                L += ["> - No .db3 for: " + ", ".join(f"`{f['bag']}`" for f in missing) + "."]
            if short:
                L += ["> - Fewer laps than planned: " + "; ".join(short) + "."]
            L += ["> Unpack the other parts into `docs/experimental_data_ros2_bag/1105 - .../` and re-run (below): the",
                  "> missing bags are found by name and scored, and every file here is rewritten.", ""]
        L += ["## Files", "", "```",
              f"{layout:<58s} one LAP, struct `run`",
              f"{task + '_rmse.mat':<58s} rmse_mean, rmse_runs, rmse_flights, rmse_nominal, bags",
              f"{'configs/':<58s} the hardware configs as flown (taken from git)", "```", ""]
        if any(s["sub"] for s in settings):
            L += ["- **`<setting>`:** " + ", ".join(f"`{s['sub']}` = {s['long']}" for s in settings) + "."]
        L += ["- **`<method>`:** `whole_body_l1` (Proposed) or `geometric_l1` (Geo-L1, position-mode arm).",
              "- **`run<k>`:** the k-th lap of that setting and method, in flight order (two laps per flight).",
              "  `run.meta.tag` is the lap's tag (e.g. `WB-50g-3`), `run.meta.lap` says which lap of its flight it is.", "",
              "Summary file (`struct2table` each):", "",
              "- `rmse_mean`: one row per setting and method, the mean over its laps (the entry to report).",
              "- `rmse_runs`: every lap in it.",
              "- `rmse_flights`: each flight's two-lap run scored as a whole (for reference).",
              "- `rmse_nominal`: the undisturbed circle 0.13 m/s entry of `free_flight_tracking` (2026-10-28, WB-15/16,",
              "  DEC-9/10).",
              "- `bags`: every bag of the study and what the download holds.", "",
              "```matlab", f"r = load('{example}').run;",
              "plot(r.tracking.t, vecnorm(r.tracking.ee_pos_err_m, 2, 2) * 1e3)        % EE position error [mm]",
              f"S = load('{task}_rmse.mat');",
              "T = struct2table(S.rmse_mean);    N = struct2table(S.rmse_nominal);",
              "```", "",
              "## How the entries are made", "",
              "- **Runs = laps.** Each flight flies ONE planner run of two laps back to back (T = 52.33 s, lap 24.17 s),",
              "  not the nominal campaign's two separate one-lap runs. The laps are split where the planner says the",
              "  first lap ends (`ee_trajectory/info[15]`, 26.17 s into the run). So lap 1 holds the 2 s speed-up and",
              "  lap 2 the 2 s slow-down; a nominal run holds both ramps in its 28.17 s.",
              "- **Each lap is scored like a nominal run:** the hardware report's definitions, 0.1 s trimmed at each end,",
              "  100 Hz grid, against the planner's streamed whole-body reference.",
              f"- **Entry = the mean over all laps of the setting:** {EXPECTED_FLIGHTS} flights × {LAPS} laps = "
              f"{EXPECTED_FLIGHTS * LAPS}. The rule never looks at the error.", "",
              "## Results (mean over the laps)", "",
              _md_table(["method", "disturbance", "laps", "EE RMSE [mm]", "EE heading RMSE [deg]",
                         "platform RMSE [mm]", "platform heading RMSE [deg]", "max tilt [deg]"], res), "",
              "The nominal rows are `../free_flight_tracking/`'s circle 0.13 m/s entries (one-lap runs, flown 10-28).", "",
              "### Every lap", "",
              _md_table(["file", "lap tag", "method", "disturbance", "flown", "lap of the flight", "window [s]",
                         "EE RMSE [mm]", "EE heading RMSE [deg]", "platform RMSE [mm]"], laps_rows), "",
              "### The bags", "",
              _md_table(["flown", "original ros2 bag", "method", "disturbance", "laps", "status",
                         "bag copy (`flight_test_data/" + task + "/`)", "both laps, EE RMSE [mm]"], bag_rows), "",
              "The originals are in `docs/experimental_data_ros2_bag/1105 - T650-AM Mail-Box, Robust Tracking, "
              "Box-Pushing-*/Robust Tracking [Success]/`.", "",
              "## Conditions", "",
              "- **Disturbance:** " + "; ".join(f"{s['label']}: {s['long']}" for s in settings) + ".",
              "- **Trajectory:** the nominal circle: radius 0.50 m, 0.13 m/s (lap 24.17 s), q2 = 25 ± 15 deg four",
              "  cycles per lap, fold 55 deg, EE heading along the tangent; two laps per flight.",
              "- **Vehicle:** T650 aerial manipulator with the 4-DOF OM-X arm, total mass 3.746 kg in the controller",
              "  model (the disturbance mass is not in it).",
              "- **Feedback:** OptiTrack fused in PX4 EKF2, about 100 Hz; joint encoders.",
              "- **Controllers:** the final campaign's hardware yamls, unchanged (`git diff` between the final campaign's",
              "  commit and the branch flown on 11-05 is empty for both); the nodes were built from the `dev_robotic_arm`",
              "  branch, whose later features the free-flight yaml leaves off.",
              "  - Proposed: " + CONFIGS["whole_body_l1"] + ".",
              "  - Geo-L1: " + CONFIGS["geometric_l1"] + ".",
              "- **Configs:** " + "; ".join(f"`configs/{n}` ({src})" for n, src in configs) + ".", "",
              "## The struct `run`", "", STRUCT_EXP.rstrip(), "",
              "Extra `meta` fields here: `setting`, `setting_label`, `disturbance` (and `ee_mass_kg` or `wind_speed_mps` /",
              "`fan_distance_m`), `lap`, `laps_in_planner_run`, `lap_start_s` / `lap_end_s`, `planner_run_start_s` /",
              "`planner_run_end_s`, `planner_run_duration_s`, `lap_time_s`, `ramp_time_s` (all s since the bag's first",
              "message, except the durations). `raw` holds the whole bag, so both laps of a flight carry the same `raw`.", "",
              "Regenerate (from the bags in `docs/experimental_data_ros2_bag/`):", "",
              "```bash", "/usr/bin/python3 results/utils/experiment_robust_tracking.py --copy-bags", "```", ""]
        path = os.path.join(MAT_DIR, task, "README.md")
        open(path, "w").write("\n".join(L))
        print("wrote", os.path.relpath(path, RESULTS), flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage2", action="store_true")
    ap.add_argument("--copy-bags", action="store_true",
                    help="copy the bags into experiment_results/flight_test_data/<task>/")
    a = ap.parse_args()
    flights = discover()
    if a.stage2:
        score(flights)
        return
    for f in flights:
        print(f"{f['flown']}  {f['bag']:58s} {'db3' if f['present'] else 'NO .db3':8s} "
              f"{'' if f['has_meta'] else '(no metadata.yaml)'}", flush=True)
    extract(flights)
    env = dict(os.environ, PYTHONNOUSERSITE="1", AM_EXP_CACHE=CACHE)
    subprocess.run(["/usr/bin/python3", os.path.abspath(__file__), "--stage2"], env=env, check=True)
    if a.copy_bags:
        copy_bags(flights)
    write_readmes(flights)


if __name__ == "__main__":
    main()
