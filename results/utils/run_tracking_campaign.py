#!/usr/bin/env python3
"""Fly the free-flight trajectory-tracking table in Isaac: 2 shapes x 3 speeds x 3 methods x N runs.

    setsid nohup /usr/bin/python3 results/utils/run_tracking_campaign.py \
        > results/simulation_results/free_flight_tracking/campaign.out 2>&1 < /dev/null &

One launch condition for every flight (the 2026-10-01 RTF-1 comparison's): headless, real-time
pacer, Isaac pinned to the P-cores (machine config), no wall-clock rescale, raw mocap feedback,
planner time scale 1.0. Each flight is one am_compare_cycle.sh (clean slate -> stack -> Isaac ->
am_ee_compare_driver.py). Each method flies its yaml snapshot in results/simulation_results/free_flight_tracking/configs/:
the same plant (every sim_* key identical), the same planner section, the same guards.

  shape     circle: r 0.5 m, takeoff yaw 0      figure-8: A 0.70 / B 0.35 m, takeoff yaw 45 deg
  speed     mean EE speed 0.10 / 0.13 / 0.20 m/s -> lap = path / speed
  arm       q2 = 25 +- 15 deg, four cycles per lap (period = lap / 4), fold 55 deg
  planner   one lap, s = 1, EE-run bounds a 0.40 m/s^2, yaw rate 1.0 rad/s (the hardware yaml's)

Every attempt is classified from its npz and filed:
  completed      run_start and run_end marked, no abort, no guard trip
                 -> <shape>/v<speed>/<method>_<shape>_v<speed>_run<k>.npz (+ logs/<name>/)
  pre_run        never reached Start (hover trip, start gate, plan refused) -> retried
  in_run         aborted / guard-tripped after Start: a FAILURE OF THE METHOD -> kept under
                 failed/, counted; the run index is retried once, and a cell with 3 such
                 failures is not flown further
  no_data        the cycle produced no npz (stack or Isaac did not come up) -> retried
campaign.jsonl gets one line per attempt.
"""
import json
import math
import os
import shutil
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.abspath(os.path.join(HERE, ".."))                  # results/
ROOT = os.path.join(RESULTS, "simulation_results")
REPO = os.path.abspath(os.path.join(RESULTS, ".."))
FF = os.path.join(ROOT, "free_flight_tracking")
CONFIGS = os.path.join(FF, "configs")
STAGING = os.path.join(FF, "_staging")
CYCLE = os.path.join(REPO, "application", "robotic_arm", "utils", "am_compare_cycle.sh")
LOG = os.path.join(FF, "campaign.jsonl")

RUNS = int(os.environ.get("CAMPAIGN_RUNS", "3"))
MAX_ATTEMPTS = int(os.environ.get("CAMPAIGN_MAX_ATTEMPTS", "4"))
SPEEDS = [0.10, 0.13, 0.20]
# pass order: the two fastest cells first, so a refused plan shows up early
CELLS = [("circle", 0.20), ("figure8", 0.20), ("circle", 0.10), ("circle", 0.13), ("figure8", 0.10),
         ("figure8", 0.13)]
METHODS = ["whole_body_l1", "geometric_l1", "modular_adaptive"]
RIG = {"whole_body_l1": "wb", "geometric_l1": "decoupled", "modular_adaptive": "modular"}
WB_YAML = os.path.join(CONFIGS, "whole_body_l1_4d_mirror_sim.yaml")
YAML = {"whole_body_l1": WB_YAML,
        "geometric_l1": os.path.join(CONFIGS, "geometric_l1_mirror_sim.yaml"),
        "modular_adaptive": os.path.join(CONFIGS, "modular_adaptive_mirror_sim.yaml")}
FIG8_A, FIG8_B, RADIUS = 0.70, 0.35, 0.50


def vtag(v):
    return "v" + f"{v:.2f}".replace(".", "p")


def path_length(shape):
    if shape == "circle":
        return 2 * math.pi * RADIUS
    th = np.linspace(0, 2 * np.pi, 400001)
    return float((np.trapezoid if hasattr(np, 'trapezoid') else np.trapz)(np.hypot(FIG8_A * np.cos(th), 2 * FIG8_B * np.cos(2 * th)), th))


def driver_args(shape, v):
    lap = round(path_length(shape) / v, 4)
    common = ["--lap-time", f"{lap:.4f}", "--laps", "1", "--q2-period", f"{lap / 4:.6f}", "--fold-deg", "55",
              "--q2-center-deg", "25", "--q2-amp-deg", "15", "--ee-a-max", "0.40", "--ee-w-max", "1.0",
              "--time-scale", "1.0", "--start-pos-tol", "0.10", "--gate-speed", "0.10"]
    if shape == "circle":
        return ["--shape", "circle", "--radius", f"{RADIUS}"] + common, lap
    return ["--shape", "figure8", "--fig8-a", f"{FIG8_A}", "--fig8-b", f"{FIG8_B}", "--yaw-deg", "45"] + common, lap


def classify(npz):
    if not os.path.isfile(npz):
        return "no_data", "no npz written"
    d = np.load(npz, allow_pickle=True)
    marks = {}
    for m in d["marks"]:
        k, _, val = str(m).partition("=")
        try:
            marks[k] = float(val)
        except ValueError:
            pass
    if "run_start" not in marks:
        return "pre_run", f"never started (aborted={bool(d['aborted'])} {d['reason']})"
    if bool(d["aborted"]) or "run_end" not in marks or ("guard_trip" in marks and marks["guard_trip"] < marks.get("run_end", 1e9)):
        return "in_run", f"aborted after Start (reason={d['reason']}, guard_trip={marks.get('guard_trip')})"
    return "completed", f"run {marks['run_end'] - marks['run_start']:.1f} s"


def rtf_readings(pane):
    out = []
    for line in pane.splitlines():
        if "RTF " in line:
            try:
                out.append(float(line.split("RTF ")[1].split()[0]))
            except (IndexError, ValueError):
                pass
    return out


def file_attempt(method, shape, v, k, attempt, status, staging_tag):
    rig = RIG[method]
    name = f"{method}_{shape}_{vtag(v)}_run{k}"
    cell = os.path.join(FF, shape, vtag(v))
    if status == "completed":
        dst_dir, dst_name = cell, name
    else:
        dst_dir, dst_name = os.path.join(cell, "failed"), f"{name}_attempt{attempt}_{status}"
    logd = os.path.join(dst_dir, "logs", dst_name)
    os.makedirs(logd, exist_ok=True)
    src_npz = os.path.join(STAGING, f"{rig}_{staging_tag}.npz")
    if os.path.isfile(src_npz):
        shutil.move(src_npz, os.path.join(dst_dir, dst_name + ".npz"))
    for src, dst in ((f"{rig}_{staging_tag}.log", "driver.log"), (f"stack_{staging_tag}.log", "stack.log"),
                     (f"pegasus_{staging_tag}.log", "pegasus.log"), (f"cycle_{staging_tag}.log", "cycle.log"),
                     (f"isaac_pane_{staging_tag}.txt", "isaac_pane.txt")):
        p = os.path.join(STAGING, "logs", src)
        if os.path.isfile(p):
            shutil.move(p, os.path.join(logd, dst))
    return os.path.join(dst_dir, dst_name + ".npz")


def fly(method, shape, v, k, attempt):
    rig = RIG[method]
    staging_tag = f"{shape}_{vtag(v)}_run{k}_a{attempt}"
    args, lap = driver_args(shape, v)
    env = dict(os.environ)
    for key in ("WB_SIM_PROFILE", "WB_PLANNER_YAML"):
        env.pop(key, None)
    env.update(DISPLAY=env.get("DISPLAY", ":1"), PEGASUS_HEADLESS="1", PEGASUS_REALTIME="1", PEGASUS_SIM_RTF="1.0",
               AM_CMP_FEEDBACK="raw", AM_CMP_OUT=STAGING, WB_SIM_YAML=YAML[method])
    if method == "geometric_l1":
        env["WB_PLANNER_YAML"] = WB_YAML
    os.makedirs(os.path.join(STAGING, "logs"), exist_ok=True)
    cyc_log = os.path.join(STAGING, "logs", f"cycle_{staging_tag}.log")
    t0 = time.time()
    with open(cyc_log, "w") as f:
        rc = subprocess.run([CYCLE, rig, staging_tag, "shiqi_machine", "--"] + args, env=env, stdout=f,
                            stderr=subprocess.STDOUT).returncode
    pane = subprocess.run(["tmux", "capture-pane", "-J", "-p", "-t", "px4_isaac:0.1", "-S", "-5000"],
                          capture_output=True, text=True).stdout
    with open(os.path.join(STAGING, "logs", f"isaac_pane_{staging_tag}.txt"), "w") as f:
        f.write(pane)
    status, why = classify(os.path.join(STAGING, f"{rig}_{staging_tag}.npz"))
    dst = file_attempt(method, shape, v, k, attempt, status, staging_tag)
    rtf = rtf_readings(pane)
    rec = dict(time=time.strftime("%Y-%m-%d %H:%M:%S"), method=method, shape=shape, speed=v, run=k,
               attempt=attempt, status=status, why=why, cycle_rc=rc, wall_s=round(time.time() - t0, 1),
               lap_time_s=lap, rtf_min=min(rtf) if rtf else None, rtf_n=len(rtf), file=os.path.relpath(dst, ROOT),
               args=args, yaml=os.path.relpath(YAML[method], ROOT))
    with open(LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")
    print(f"[{rec['time']}] {method:17s} {shape:8s} {v:.2f} run{k} a{attempt}: {status:9s} {why}  "
          f"rtf_min={rec['rtf_min']} wall={rec['wall_s']}s", flush=True)
    return status


def done_runs():
    """(method, shape, speed) -> set of completed run indices already on disk (resume support)."""
    out = {}
    for method in METHODS:
        for shape, v in CELLS:
            for k in range(1, RUNS + 1):
                if os.path.isfile(os.path.join(FF, shape, vtag(v), f"{method}_{shape}_{vtag(v)}_run{k}.npz")):
                    out.setdefault((method, shape, v), set()).add(k)
    return out


def main():
    for p in YAML.values():
        if not os.path.isfile(p):
            sys.exit(f"missing config {p}")
    have = done_runs()
    in_run_fails = {}
    print(f"campaign start {time.strftime('%F %T')}: {RUNS} runs x {len(CELLS)} cells x {len(METHODS)} methods; "
          f"already done: {sum(len(s) for s in have.values())}", flush=True)
    for k in range(1, RUNS + 1):
        for shape, v in CELLS:
            for method in METHODS:
                if k in have.get((method, shape, v), set()):
                    continue
                if in_run_fails.get((method, shape, v), 0) >= 3:
                    print(f"skip {method} {shape} {v:.2f} run{k}: 3 in-run failures in this cell", flush=True)
                    continue
                tries_in_run = 0
                for attempt in range(1, MAX_ATTEMPTS + 1):
                    st = fly(method, shape, v, k, attempt)
                    if st == "completed":
                        break
                    if st == "in_run":
                        in_run_fails[(method, shape, v)] = in_run_fails.get((method, shape, v), 0) + 1
                        tries_in_run += 1
                        if tries_in_run >= 2:
                            break
    print(f"campaign end {time.strftime('%F %T')}", flush=True)
    subprocess.run([os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh")],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    main()
