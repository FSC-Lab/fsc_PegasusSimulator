#!/usr/bin/env python3
"""Score the 10-28 (no wall) and 11-02 (wall up) whole-body circle runs, 0.13 m/s, with the paper's tracking scorer
(results/utils/experiment_tracking.py -> the 09-28 metrics), WITHOUT writing anything under results/: the cache is
../cache. Stage 1 (extract; ROS + user-site numpy):  track_score.py extract
        Stage 2 (score; PYTHONNOUSERSITE=1):          track_score.py score"""
import os, sys, json, subprocess
HERE = os.path.dirname(os.path.abspath(__file__))
os.environ["AM_EXP_CACHE"] = os.path.join(HERE, "..", "cache")
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "results", "utils"))
import experiment_tracking as E
BAG_1102 = os.path.join(REPO, "docs", "experimental_data_ros2_bag", "1102 - T650-AM Final Movies",
                        "drive-download-20261103T232136Z-1-002", "1-Free-Flight Trajectory-Tracking",
                        "1-whole_body_circle_013_movie_20261102_182844")
RUNS = {k: E.RUNS[k] for k in ("w13", "w14", "w15", "w16", "w17", "w18", "r9", "r10", "r11", "r12")}
RUNS["n1"] = ("WB-1102", "11-02 18:28", "whole_body_l1", "circle", 0.13, "whole_body_circle_013_movie_20261102_182844", 0, True, "", "wb_1016")
RUNS["n1a"] = ("1102-lap1", "11-02 18:28", "whole_body_l1", "circle", 0.13, "whole_body_circle_013_movie_20261102_182844", 0, True, "", "wb_1016")
RUNS["n1b"] = ("1102-lap2", "11-02 18:28", "whole_body_l1", "circle", 0.13, "whole_body_circle_013_movie_20261102_182844", 0, True, "", "wb_1016")
LAP = 24.17 + 2.0     # the 2-lap run = ramp + 2 x 24.17 s + ramp; split at its middle
# the three bags whose .db3 the Drive download put in the other zip part: paired by symlinks under ../bags
BAGS2 = os.path.join(HERE, "..", "bags")
NEW = {"183342": ("WB-1102f8", "whole_body_l1", "figure8", "2-whole_body_figure8_013_movie_20261102_183342"),
       "184129": ("DEC-1102c", "geometric_l1", "circle", "3-decoupled_circle_013_movie_20261102_184129"),
       "184743": ("DEC-1102f8", "geometric_l1", "figure8", "4-decoupled_figure8_013_movie_20261102_184743")}
for ts, (nm, meth, shape, d) in NEW.items():
    for k, half in (("a", "lap1"), ("b", "lap2")):     # one two-lap run per bag, split at its middle
        RUNS[f"x{ts}_{k}"] = (f"{nm}-{half}", "11-02", meth, shape, 0.13, d, 0, True, "", "wb_1016")
E.RUNS.clear(); E.RUNS.update(RUNS)
_bag_dir = E.bag_dir
E.bag_dir = lambda name: BAG_1102 if name.endswith("182844") else (os.path.join(BAGS2, name) if name[:2] in ("2-", "3-", "4-") else _bag_dir(name))
if sys.argv[1] == "extract":
    E.extract()
else:
    os.environ["AM_NPZ"] = E.CACHE
    sys.path.insert(0, E.TOOLS)
    import common as C, metrics as M
    C.FLIGHTS.update({k: f"{v[0]} {v[1]}" for k, v in RUNS.items()})
    src = open(E.__file__).read()
    # the scorer's own window rule (copied from score())
    def windows(nm):
        d, t0 = C.load(nm)
        mk = "wbmode" if "wbmode__recv" in d.files else "gmode"
        md = C.edges(d, t0, mk); st = C.edges(d, t0, "pl_status")
        runs = [(t, st[i + 1][0] if i + 1 < len(st) else float(d[f"{mk}__recv"][-1] - t0))
                for i, (t, v) in enumerate(st) if v.startswith("EXECUTING T=") and float(v.split("=")[1].rstrip("s")) >= 10.0]
        c0, c1 = runs[RUNS[nm][6]]
        if nm == "n1a" or nm.endswith("_a"): c1 = c0 + (c1 - c0) / 2
        elif nm == "n1b" or nm.endswith("_b"): c0 = c0 + (c1 - c0) / 2
        a = max(t for t, v in md if v == "DIRECT" and t < c0)
        b = next((t for t, v in md if v == "SAFETY" and t > c0), float(d[f"{mk}__recv"][-1] - t0))
        return a, b, c0, c1
    C.windows = windows; M.windows = windows
    out = {}
    for key in RUNS:
        o, _ = M.analyse(key); m = o["metrics"]
        row = {c: float(m[g][f][i]) for c, (g, f, i) in E.KEYMAP.items()}
        row.update(ee_heading_deg=float(m["ee_head"]["rms"]), ee_pos_mm=float(m["ee_pos"]["rms_norm"]),
                   platform_pos_mm=float(m["base_pos"]["rms_norm"]), window=list(o["window"]), tilt_max_deg=float(m["tilt_max_deg"]))
        out[key] = row
        print(f"{RUNS[key][0]:8s} {RUNS[key][1]}  window {o['window'][1] - o['window'][0]:6.2f} s | EE {row['ee_pos_mm']:5.1f} mm "
              f"(x {row['ee_x_mm']:5.1f} y {row['ee_y_mm']:5.1f} z {row['ee_z_mm']:5.1f}), heading {row['ee_heading_deg']:4.2f} deg | platform "
              f"{row['platform_pos_mm']:5.1f} mm (x {row['platform_x_mm']:5.1f} y {row['platform_y_mm']:5.1f} z {row['platform_z_mm']:5.1f}) "
              f"roll {row['platform_roll_deg']:4.2f} pitch {row['platform_pitch_deg']:4.2f} deg | tilt max {row['tilt_max_deg']:4.1f}")
    json.dump(out, open(os.path.join(E.CACHE, "scores.json"), "w"), indent=1)
