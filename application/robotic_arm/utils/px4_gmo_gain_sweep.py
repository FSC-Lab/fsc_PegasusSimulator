#!/usr/bin/env python3
"""
px4_gmo_gain_sweep.py — full-stack gain sweep for
application/robotic_arm/03_px4_direct_aerial_manipulator_free.py.

Author: Shiqi Gao (shiqi.gao907@gmail.com)

Runs the COMPLETE direct-actuator rig once per trial — PX4 SITL (none_iris,
headless) + MicroXRCEAgent + px4_direct_actuator_rotor_bridge.py + headless
Isaac Sim — with per-trial gain overrides injected through the demo's AM_SWEEP
env JSON (k_x/k_v/k_R/k_w scalars, K_y/D_y 4-lists, M_r_d 3-list, K_o 10-list,
...). Each trial ends when the demo does (a landing mode closes itself after
touchdown; t_end is the cap), writes its npz, the stack is torn down, and
flight-quality metrics are computed from the log. This is the "gain traverse"
harness the demo's header refers to, specialised to the PX4-primary plant
(LaggedQuadraticThrustCurve lambda=10.51, bench PX4 map, 3.416 kg body
override — none of which this script touches).

Usage (system python3; numpy only, no ROS/Isaac imports in THIS process):
    python3 px4_gmo_gain_sweep.py --trials trials.json \
        [--traj poly_whole] [--t-end 120] [--out results.jsonl] \
        [--results-dir DIR] [--retry 1]

trials.json: [{"name": "baseline", "sweep": {}},
              {"name": "kx24", "sweep": {"k_x": 24.0, "k_v": 12.0}}, ...]
sweep keys = the demo's AM_SWEEP keys. headless/t_end/log_path/traj_type are
injected automatically; a trial's own "traj_type" / "t_end" / "headless" win.

SCORING follows the trajectory. A HOVER trial is scored on how still it sits
(the original metric set). A trajectory trial with a phase table in its npz
(the demo writes phase_t/phase_names) is scored per PHASE — tracking error
while the plan runs, EE error while the tool is pinned, and whether the flight
actually reached its landing — because "small rms over the whole log" on a
flight that spends half its time hovering says almost nothing about the part
that matters.

Only ONE trial stack runs at a time (single GPU; PX4 ports 4560/8888/14540 are
hardcoded across the rig). An already-running MicroXRCEAgent is reused, same
as the interactive launcher.
"""

import argparse
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np

# ── machine paths (defaults = scripts/config/shiqi_machine.conf) ─────────────
REPO = os.environ.get("FSC_PEGASUS_ROOT", "/home/shiqi/fsc_PegasusSimulator")
PX4_DIR = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
ISAAC_PY = os.environ.get("ISAAC_PY", os.path.expanduser("~/isaacsim/python_r_fsc.sh"))
ROS2_SETUP = os.environ.get("ROS2_SETUP", "/opt/ros/humble/setup.bash")
PX4_MSGS_SETUP = os.environ.get(
    "PX4_MSGS_SETUP", os.path.expanduser("~/workspaces/isaacsim/install/setup.bash"))

DEMO = os.path.join(REPO, "application/robotic_arm/03_px4_direct_aerial_manipulator_free.py")
BRIDGE = os.path.join(
    REPO, "extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/"
          "px4_direct_actuator_rotor_bridge.py")

UAV_NS = "uav_0"
PX4_TARGET = "none_iris"

# ── timing (wall-clock seconds) ──────────────────────────────────────────────
PX4_BOOT_WAIT = 60       # max wait for the pxh prompt before sending params
BRIDGE_DELAY = 17        # start the bridge this long after PX4 (params applied first)
ENGAGE_TIMEOUT = 420     # Isaac start -> "PX4 ENGAGED" (includes ~1-2 min asset load)
TRIAL_TIMEOUT = 900      # absolute per-trial wall clock
KILL_GRACE = 12

# ── scoring ──────────────────────────────────────────────────────────────────
SETTLE_T = 8.0           # [s] hover trials: score from here (takeoff + settle)
LATE_WIN = 5.0           # [s] final window for divergence detection
TAU_MAX = 3.0            # must match the demo's clamp


def _rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2))) if x.size else float("nan")


def _phases(d):
    """{name: (t0, t1)} from the npz phase table, or {} for an older log."""
    if "phase_t" not in d or "phase_names" not in d:
        return {}
    edges = np.asarray(d["phase_t"], float)
    names = [str(n) for n in np.asarray(d["phase_names"])]
    return {nm: (float(edges[i]), float(edges[i + 1]))
            for i, nm in enumerate(names) if i + 1 < len(edges)}


def score_hover(d, t, m, t_end):
    """How still does it sit? The original hover metric set."""
    w = t >= SETTLE_T                      # scoring window
    late = t >= (t_end - LATE_WIN)
    mid = w & ~late
    if not np.any(w):
        m["error"] = "no samples past settle window"
        return m, 1e6

    pos_err = np.linalg.norm(d["x_c"][:, :3] - d["x_cd"][:, :3], axis=1)
    ee_err = np.linalg.norm(d["e_y"][:, :3], axis=1)

    m["rms_pos"] = _rms(pos_err[w])
    m["max_pos"] = float(np.max(pos_err[w]))
    m["rms_pos_mid"] = _rms(pos_err[mid])
    m["rms_pos_late"] = _rms(pos_err[late])
    m["rms_att"] = _rms(np.linalg.norm(d["e_R"], axis=1)[w])
    m["rms_rate"] = _rms(np.linalg.norm(d["omega0"], axis=1)[w])
    m["rms_ee"] = _rms(ee_err[w])
    m["rms_ee_late"] = _rms(ee_err[late])
    m["rms_eeh"] = _rms(d["e_y"][w, 3])
    m["rms_qdot"] = _rms(np.linalg.norm(d["qdot"], axis=1)[w])
    m["tau_sat_frac"] = float(np.mean(np.max(np.abs(d["tau_j"][w]), axis=1) >= 0.999 * TAU_MAX))
    m["thrust_std"] = float(np.std(d["thrust"][w]))
    # divergence: the late window is meaningfully worse than the mid window
    m["diverging"] = bool(
        (m["rms_pos_late"] > 1.5 * m["rms_pos_mid"] and m["rms_pos_late"] > 0.05)
        or (m["rms_ee_late"] > 1.5 * m["rms_ee"] and m["rms_ee_late"] > 0.05))

    if m["crashed"]:
        return m, 1e6
    cost = (10.0 * m["rms_pos"] + 8.0 * m["rms_pos_late"] + 25.0 * m["rms_att"]
            + 6.0 * m["rms_rate"] + 20.0 * m["rms_ee"] + 2.0 * m["rms_eeh"]
            + 1.5 * m["rms_qdot"] + 4.0 * m["tau_sat_frac"]
            + (500.0 if m["diverging"] else 0.0))
    return m, float(cost)


def score_task(d, t, m, ph):
    """How well does it FLY THE PLAN? Scored per phase off the npz phase table.

    The task window is what the gains are for; the pinned window is what the
    showcase claims (the tool tip frozen in the air while the base moves); and
    a flight that never reached its landing did not finish, however small its
    errors were up to that point.
    """
    pos_err = np.linalg.norm(d["x_c"][:, :3] - d["x_cd"][:, :3], axis=1)
    ee_err = np.linalg.norm(d["e_y"][:, :3], axis=1)
    att_err = np.linalg.norm(d["e_R"], axis=1)
    task_names = [nm for nm in ph if nm in ("fly-in", "pinned", "fly-out", "yaw")]
    if not task_names:
        m["error"] = "no task phase in the log (never left takeoff)"
        return m, 1e6
    t0 = min(ph[nm][0] for nm in task_names)
    t1 = max(ph[nm][1] for nm in task_names)
    w = (t >= t0) & (t <= t1)

    m["phases"] = {k: [round(v[0], 2), round(v[1], 2)] for k, v in ph.items()}
    if not np.any(w):
        m["error"] = "crashed before the task phase produced samples"
        return m, 1e6
    m["rms_pos"] = _rms(pos_err[w])
    m["max_pos"] = float(np.max(pos_err[w]))
    m["rms_att"] = _rms(att_err[w])
    m["max_att"] = float(np.max(att_err[w]))
    m["rms_rate"] = _rms(np.linalg.norm(d["omega0"], axis=1)[w])
    m["rms_ee"] = _rms(ee_err[w])
    m["max_ee"] = float(np.max(ee_err[w]))
    m["rms_eeh"] = _rms(d["e_y"][w, 3])
    m["rms_qdot"] = _rms(np.linalg.norm(d["qdot"], axis=1)[w])
    m["tau_sat_frac"] = float(np.mean(np.max(np.abs(d["tau_j"][w]), axis=1) >= 0.999 * TAU_MAX))
    # A crashed run still has every PLANNED phase in its table (the table comes
    # from the plan, not from what was flown), so a window can be empty —
    # np.max would then raise and lose the whole trial's metrics.
    def _max(x):
        return float(np.max(x)) if np.size(x) else float("nan")
    if "pinned" in ph:
        pw = (t >= ph["pinned"][0]) & (t <= ph["pinned"][1])
        m["rms_ee_pin"] = _rms(ee_err[pw])
        m["max_ee_pin"] = _max(ee_err[pw])
        m["max_att_pin"] = _max(att_err[pw])
    # did the flight finish? "landing" only exists once the task-end hover gate
    # cleared, and z at the end says whether it actually came down.
    m["landed"] = "landing" in ph
    m["z_final"] = float(d["p"][-1, 2])
    # worst attitude ANYWHERE after takeoff — a rig that is going unstable
    # usually shows it as a tilt excursion before anything else
    aw = t >= (ph.get("takeoff", (0.0, 0.0))[1])
    m["max_att_all"] = _max(att_err[aw])

    if m["crashed"]:
        return m, 1e6
    cost = (10.0 * m["rms_pos"] + 5.0 * m["max_pos"] + 25.0 * m["rms_att"]
            + 10.0 * m["max_att"] + 6.0 * m["rms_rate"] + 20.0 * m["rms_ee"]
            + 10.0 * m.get("rms_ee_pin", 0.0) + 2.0 * m["rms_eeh"]
            + 1.5 * m["rms_qdot"] + 4.0 * m["tau_sat_frac"]
            + (200.0 if not m["landed"] else 0.0))
    return m, float(cost)


def score_npz(path, t_end):
    """Flight-quality metrics from a demo npz. Returns (metrics, cost).

    Which metric set is used follows the LOG, not a CLI flag: a log carrying a
    task phase gets the per-phase trajectory score, anything else the hover
    score."""
    d = np.load(path, allow_pickle=False)
    t = d["t"]
    m = {"crashed": bool(np.asarray(d["crashed"])), "t_final": float(t[-1])}
    if t.size == 0:
        m["error"] = "empty log"
        return m, 1e6
    ph = _phases(d)
    if any(nm in ph for nm in ("fly-in", "pinned", "fly-out", "yaw")):
        return score_task(d, t, m, ph)
    return score_hover(d, t, m, t_end)


# ── process helpers ──────────────────────────────────────────────────────────

def _spawn(cmd, log_path, cwd=None, env=None, stdin=None):
    log = open(log_path, "ab", buffering=0)
    return subprocess.Popen(
        cmd, cwd=cwd, env=env, stdin=stdin, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)          # own process group -> clean killpg


def _killpg(proc, name):
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        return
    for sig, wait_s in ((signal.SIGTERM, KILL_GRACE), (signal.SIGKILL, 5)):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        t0 = time.time()
        while time.time() - t0 < wait_s:
            if proc.poll() is not None:
                return
            time.sleep(0.5)
    print(f"    [cleanup] {name} did not die", flush=True)


def _pkill_strays():
    """Kill leftovers from a previous (possibly aborted) trial. The agent is
    deliberately NOT in this list — it is shared/reused across trials."""
    for pat in (r"bin/px[4]", r"px4_direct_actuator_rotor_bridge\.p[y]",
                r"03_px4_direct_aerial_manipulator_free\.p[y]"):
        subprocess.run(["pkill", "-9", "-f", pat], check=False)


def _log_contains(path, needle):
    try:
        with open(path, "rb") as f:
            return needle.encode() in f.read()
    except FileNotFoundError:
        return False


def check_no_foreign_publishers():
    """Refuse to sweep while another /uav_0 offboard controller is alive.

    Found the hard way (2026-07-30 diagnostic run): a leftover
    autopilot_direct_actuation_node from an earlier bare-X650 test session was
    still streaming ActuatorMotors on /uav_0 — PX4's output module interleaved
    the two publishers, and its iris-convention allocation on the AM's mirrored
    rotor layout rolled the vehicle over within 1 s. Two rigs on the same
    namespace can NEVER run at once (ports 8888/4560/14540 + DDS ns uav_0)."""
    pats = (r"autopilot_direct_actuation_nod[e]", r"virtual_remot[e]",
            r"x650_direct_actuator_test_nod[e]", r"x650_ros_offboar[d]")
    offenders = []
    for pat in pats:
        r = subprocess.run(["pgrep", "-af", pat], capture_output=True, text=True)
        offenders += [l for l in r.stdout.splitlines() if l.strip()]
    if offenders:
        sys.exit("[sweep] ABORT — another /uav_0 offboard controller is running "
                 "(it would interleave ActuatorMotors with the bridge and crash "
                 "every trial):\n  " + "\n  ".join(offenders)
                 + "\nStop that stack (tmux kill-session) and re-run.")


def ensure_agent(log_dir):
    if subprocess.run(["pgrep", "-x", "MicroXRCEAgent"], capture_output=True).returncode == 0:
        return None                       # reuse the existing one
    print("  starting MicroXRCEAgent (none running)", flush=True)
    return _spawn(["MicroXRCEAgent", "udp4", "-p", "8888"],
                  os.path.join(log_dir, "xrce.log"))


# ── one trial ────────────────────────────────────────────────────────────────

def run_trial(name, sweep, t_end, results_dir, traj):
    log_dir = os.path.join(results_dir, name)
    os.makedirs(log_dir, exist_ok=True)
    npz_path = os.path.join(log_dir, "log.npz")
    px4_log = os.path.join(log_dir, "px4.log")
    isaac_log = os.path.join(log_dir, "isaac.log")
    bridge_log = os.path.join(log_dir, "bridge.log")

    am_sweep = dict(sweep)
    am_sweep.update({"headless": True, "t_end": float(t_end), "log_path": npz_path})
    am_sweep.setdefault("traj_type", traj)   # a per-trial traj_type wins
    if "t_end" in sweep:                  # a per-trial t_end beats the CLI default
        am_sweep["t_end"] = float(sweep["t_end"])
        t_end = float(sweep["t_end"])
    if "headless" in sweep:               # rendered validation trials (opens a window)
        am_sweep["headless"] = bool(sweep["headless"])

    _pkill_strays()
    time.sleep(1.0)
    agent = ensure_agent(log_dir)

    # PX4 SITL (pxh console on our stdin pipe — the param writes go there)
    px4_env = dict(os.environ, PX4_UXRCE_DDS_NS=UAV_NS, HEADLESS="1")
    px4 = _spawn(["make", "px4_sitl", PX4_TARGET,
                  "mavlink_udp_remote:=127.0.0.1", "mavlink_udp_port:=14540"],
                 px4_log, cwd=PX4_DIR, env=px4_env, stdin=subprocess.PIPE)

    # Isaac (boots in parallel with PX4 — it holds until the bridge engages)
    isaac_env = dict(os.environ, AM_SWEEP=json.dumps(am_sweep), PYTHONUNBUFFERED="1")
    isaac = _spawn([ISAAC_PY, DEMO], isaac_log, env=isaac_env)
    t_isaac0 = time.time()

    bridge = None
    result = {"name": name, "sweep": sweep, "t_end": t_end, "status": "?"}
    try:
        # wait for the pxh prompt, then apply the offboard params (same three
        # scripts/apply_aerial_manipulator_px4_offboard_params.sh sends)
        t0 = time.time()
        while time.time() - t0 < PX4_BOOT_WAIT:
            if _log_contains(px4_log, "Startup script returned"):
                break
            if px4.poll() is not None:
                raise RuntimeError("PX4 exited during boot")
            time.sleep(1.0)
        for cmd in ("param set UXRCE_DDS_SYNCT 0",
                    "param set COM_DISARM_LAND 0",
                    "param set COM_DISARM_PRFLT 0"):
            px4.stdin.write((cmd + "\n").encode())
            px4.stdin.flush()
            time.sleep(0.3)

        while time.time() - t0 < BRIDGE_DELAY:
            time.sleep(0.5)
        bridge = _spawn(
            ["bash", "-c",
             f'source "{ROS2_SETUP}" && source "{PX4_MSGS_SETUP}" && '
             f'exec python3 "{BRIDGE}" --ros-args -r __ns:=/{UAV_NS}'],
            bridge_log, env=dict(os.environ, PYTHONUNBUFFERED="1"))

        # wait for the Isaac process to finish the trial
        engaged = False
        while True:
            if isaac.poll() is not None:
                break
            el = time.time() - t_isaac0
            if not engaged and _log_contains(isaac_log, "PX4 ENGAGED"):
                engaged = True
                print(f"    engaged at +{el:.0f}s", flush=True)
            if not engaged and el > ENGAGE_TIMEOUT:
                raise RuntimeError(f"no engagement within {ENGAGE_TIMEOUT}s")
            if el > TRIAL_TIMEOUT:
                raise RuntimeError(f"trial exceeded {TRIAL_TIMEOUT}s")
            time.sleep(2.0)

        if not os.path.exists(npz_path):
            raise RuntimeError("Isaac exited but wrote no npz (see isaac.log)")
        metrics, cost = score_npz(npz_path, t_end)
        result.update(status="crashed" if metrics.get("crashed") else "ok",
                      metrics=metrics, cost=cost)
    except (RuntimeError, BrokenPipeError, OSError) as exc:
        result.update(status="failed", error=str(exc), cost=1e6)
    finally:
        _killpg(isaac, "isaac")
        _killpg(bridge, "bridge")
        _killpg(px4, "px4")
        if agent is not None:
            pass                          # leave our agent running for the next trial
        _pkill_strays()
        result["wall_s"] = round(time.time() - t_isaac0, 1)
    return result


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", required=True, help="JSON list of {name, sweep}")
    ap.add_argument("--traj", default="poly_whole",
                    help="trajectory for trials that do not name their own "
                         "(default: the mode this rig is tuned for)")
    # t_end is a CAP, not the flight length: a landing mode closes itself after
    # touchdown, so this only bounds a run that never gets there.
    ap.add_argument("--t-end", type=float, default=120.0)
    ap.add_argument("--out", default=None, help="results JSONL (append)")
    ap.add_argument("--results-dir", default=None)
    ap.add_argument("--retry", type=int, default=1,
                    help="stack-failure retries per trial (crashes are NOT retried)")
    args = ap.parse_args()

    check_no_foreign_publishers()
    with open(args.trials) as f:
        trials = json.load(f)
    results_dir = args.results_dir or os.path.join(os.path.dirname(args.trials), "runs")
    os.makedirs(results_dir, exist_ok=True)
    out_path = args.out or os.path.join(results_dir, "results.jsonl")

    print(f"[sweep] {len(trials)} trial(s), t_end={args.t_end}s, results -> {out_path}",
          flush=True)
    for i, tr in enumerate(trials):
        name = tr["name"]
        print(f"[sweep] ({i + 1}/{len(trials)}) {name}: {tr['sweep']}", flush=True)
        for attempt in range(args.retry + 1):
            res = run_trial(name, tr["sweep"], args.t_end, results_dir, args.traj)
            if res["status"] != "failed" or attempt == args.retry:
                break
            print(f"    stack failure ({res.get('error')}) — retrying", flush=True)
        with open(out_path, "a") as f:
            f.write(json.dumps(res) + "\n")
        mtr = res.get("metrics", {})
        nan = float("nan")
        # trajectory trials report what a trajectory run is judged on (pinned
        # EE error, did it land); hover trials keep the divergence flag
        tail = (f"eePin={mtr.get('rms_ee_pin', nan):.4f} "
                f"landed={mtr.get('landed')}" if "rms_ee_pin" in mtr
                else f"div={mtr.get('diverging', '?')}")
        print(f"[sweep] {name}: {res['status']} cost={res.get('cost', nan):.3f} "
              f"pos={mtr.get('rms_pos', nan):.4f} "
              f"att={mtr.get('rms_att', nan):.4f} "
              f"ee={mtr.get('rms_ee', nan):.4f} "
              f"qdot={mtr.get('rms_qdot', nan):.3f} "
              f"sat={mtr.get('tau_sat_frac', nan):.2f} "
              f"{tail} wall={res.get('wall_s')}s", flush=True)
    print("[sweep] done", flush=True)


if __name__ == "__main__":
    main()
