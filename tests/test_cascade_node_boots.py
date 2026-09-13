#!/usr/bin/env python3
"""The cascade node must CONSTRUCT and STAY ALIVE on the ROS path, and answer its command topic.

    ~/envs/isaac_env/bin/python3 tests/test_cascade_node_boots.py [--node base|tank|both]

The offline gate (`--test`) uses the class defaults and never runs the constructor's
parameter/subscription code, so a bug there is invisible to every gate that ran before flight
turns_loiter_rth_a1 (2026-09-13): `self.gp('px4_ns')` cast the namespace STRING to float, the
node printed its schedule banner and died two lines later, the launcher took the banner as
READY, and the mission was flown by the geometric controller. The same gap swallowed the
`descent_budget` declaration mismatch of 2026-09-07 (108 m fall). This test runs each node
exactly as the launcher does (rosenv + isaac_env python + `--ros-args`), waits for its standby
line, publishes one guidance command, and requires the GUIDE ack in the node's own log and the
process still alive at the end. Needs a sourced ROS env; no stack, no Isaac.
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from controller_path import controller_dir              # noqa: E402

PY = os.path.expanduser("~/envs/isaac_env/bin/python3")
PKG = os.path.join(controller_dir(), "px4_vtol_direct_actuator")
NODES = {"base": "nmpc_cascade_node.py", "tank": "nmpc_cascade_tank_node.py"}
NS = "uav_0"


def run(node, wait_s=60.0):
    path = os.path.join(PKG, NODES[node])
    log = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"casc_boot_{node}.log")
    cmd = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; exec {PY} {path} --ros-args "
           f"-p px4_ns:={NS} -p auto_arm:=false -p controller_id:=cascade -p active_at_start:=false "
           f"-p vcruise:=16.0 -p ttr:=10.0 -p cruise_alt:=30.0 -p latency_s:=0.049 -p l1_on:=true")
    with open(log, "w") as lf:
        proc = subprocess.Popen(["bash", "-c", cmd], stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
        t0 = time.time(); ready = False
        while time.time() - t0 < wait_s:
            time.sleep(1.0)
            txt = open(log, errors="replace").read()
            if "Traceback" in txt or proc.poll() is not None:
                break
            if "outer NMPC built" in txt:
                ready = True; break
        txt = open(log, errors="replace").read()
        if not ready:
            os.killpg(proc.pid, 15); proc.wait(10)
            return False, f"{node}: never reached standby (exit {proc.poll()})\n" + txt[-1500:]
        # the protocol, end to end: topic -> on_cmd -> Guidance -> logged ack (state is None, so
        # the node must say so rather than throw -- that is the branch a dead subscription hides)
        # listen for the ack on the status topic FIRST (the ground station's Guidance row reads it)
        echo = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; timeout 15 ros2 topic echo --once /{NS}/cascade/status "
                f"std_msgs/msg/String")
        ep = subprocess.Popen(["bash", "-c", echo], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        time.sleep(2.0)
        pub = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; ros2 topic pub --once -w 1 /{NS}/cascade/cmd "
               f"std_msgs/msg/String \"{{data: 'wp 100 0'}}\"")
        subprocess.run(["bash", "-c", pub], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        t1 = time.time(); acked = False
        while time.time() - t1 < 10.0:
            time.sleep(0.5)
            txt = open(log, errors="replace").read()
            if "cascade/cmd 'wp 100 0' ignored: no state yet" in txt or "GUIDE 'wp 100 0'" in txt:
                acked = True; break
        alive = proc.poll() is None
        txt = open(log, errors="replace").read()      # BEFORE the kill: SIGTERM itself logs an rclpy shutdown traceback
        try:
            status_out, _ = ep.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            ep.kill(); status_out = ""
        os.killpg(proc.pid, 15); proc.wait(10)
        if "Traceback" in txt:
            return False, f"{node}: traceback in the node log\n" + txt[txt.find("Traceback"):][:1500]
        if not alive:
            return False, f"{node}: process died after standby (exit {proc.returncode})"
        if not acked:
            return False, f"{node}: no reaction to /{NS}/cascade/cmd within 10 s (subscription dead?)"
        # the ack must ALSO reach /<ns>/cascade/status -- that is what the ground station's
        # Guidance row displays (before engage the answer is "ignored: no state yet")
        if "ack" not in status_out or "wp 100 0" not in status_out.replace("''", "'"):
            return False, f"{node}: no ack on /{NS}/cascade/status (got {status_out.strip()[:120]!r})"
        return True, f"{node}: booted, acked the command on the status topic, alive"


def main():
    which = sys.argv[sys.argv.index("--node") + 1] if "--node" in sys.argv else "both"
    nodes = list(NODES) if which == "both" else [which]
    fails = 0
    for n in nodes:
        ok, msg = run(n)
        print(("PASS " if ok else "FAIL ") + msg)
        fails += 0 if ok else 1
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
