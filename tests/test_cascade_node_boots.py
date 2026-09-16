#!/usr/bin/env python3
"""The cascade node must CONSTRUCT and STAY ALIVE on the ROS path, and answer its command topic.

    ~/envs/isaac_env/bin/python3 tests/test_cascade_node_boots.py [--node base|tank|adaptive|imu|px4src|aspd|both]

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
# "adaptive" = the STESO + slosh-predictor variant booted AS ITS OWN CONTROLLER (2026-09-14):
# controller_id cascade_adaptive -> node /nmpc_cascade_control_adaptive, guidance on
# /uav_0/cascade_adaptive/{cmd,status}. It is what the launchers' 6i-adaptive window and the
# ground station's "Adaptive NMPC + Geo" run; the check is the same protocol on the OWN topics.
# "imu" = the IMU-force-twin copy (2026-09-15) booted the same way; it must ALSO subscribe to
# PX4's sensor_combined and log the first sample converted FRD -> FLU (the wiring the gate cannot see).
# "px4src" = the base node with `state_source: px4` (2026-09-15): it must subscribe to EKF2's
# vehicle_odometry instead of the mocap, say so, and still answer the command topic.
NODES = {"base": ("nmpc_cascade_node.py", "cascade"), "tank": ("nmpc_cascade_tank_node.py", "cascade"),
         "adaptive": ("nmpc_cascade_steso_node.py", "cascade_adaptive"),
         "imu": ("nmpc_cascade_imu_node.py", "cascade_adaptive"),
         "px4src": ("nmpc_cascade_node.py", "cascade", "-p state_source:=px4"),
         # "aspd" = the base node with `airspeed_source: px4` (piece 4, 2026-09-15): it must subscribe to
         # PX4's airspeed_validated and log the first TAS it receives
         "aspd": ("nmpc_cascade_node.py", "cascade", "-p airspeed_source:=px4")}
NS = "uav_0"


def run(node, wait_s=60.0):
    fname, cid = NODES[node][:2]; extra = NODES[node][2] if len(NODES[node]) > 2 else ""
    path = os.path.join(PKG, fname)
    log = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"casc_boot_{node}.log")
    cmd = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; exec {PY} {path} --ros-args "
           f"-p px4_ns:={NS} -p auto_arm:=false -p controller_id:={cid} -p active_at_start:=false "
           f"-p vcruise:=16.0 -p ttr:=10.0 -p cruise_alt:=30.0 -p latency_s:=0.049 -p l1_on:=true {extra}")
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
            # a JIT rebuild (a new cache key: mass mode, airframe file edit) takes ~2-4 min; the
            # 60 s budget is for a node that loads its cache, not for the compiler
            if "building 6-state outer NMPC" in txt and time.time() - t0 < 360.0:
                wait_s = max(wait_s, 360.0)
        txt = open(log, errors="replace").read()
        if not ready:
            os.killpg(proc.pid, 15); proc.wait(10)
            return False, f"{node}: never reached standby (exit {proc.poll()})\n" + txt[-1500:]
        # the protocol, end to end: topic -> on_cmd -> Guidance -> logged ack (state is None, so
        # the node must say so rather than throw -- that is the branch a dead subscription hides)
        # listen for the ack on the status topic FIRST (the ground station's Guidance row reads it)
        echo = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; timeout 15 ros2 topic echo --once /{NS}/{cid}/status "
                f"std_msgs/msg/String")
        ep = subprocess.Popen(["bash", "-c", echo], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        time.sleep(2.0)
        pub = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; ros2 topic pub --once -w 1 /{NS}/{cid}/cmd "
               f"std_msgs/msg/String \"{{data: 'wp 100 0'}}\"")
        subprocess.run(["bash", "-c", pub], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
        t1 = time.time(); acked = False
        while time.time() - t1 < 10.0:
            time.sleep(0.5)
            txt = open(log, errors="replace").read()
            if f"{cid}/cmd 'wp 100 0' ignored: no state yet" in txt or "GUIDE 'wp 100 0'" in txt:
                acked = True; break
        imu_ok = True
        if node == "imu":
            # one accelerometer message (FRD: 0.5 forward, 0.2 right, -9.81 down) must be logged as its
            # FLU conversion (+0.50, -0.20, +9.81) -- the subscription, the QoS and the frame in one line
            pubi = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; ros2 topic pub --once -w 1 /{NS}/fmu/out/sensor_combined "
                    f"px4_msgs/msg/SensorCombined \"{{accelerometer_m_s2: [0.5, 0.2, -9.81]}}\"")
            subprocess.run(["bash", "-c", pubi], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
            t2 = time.time(); imu_ok = False
            while time.time() - t2 < 10.0:
                time.sleep(0.5)
                txt = open(log, errors="replace").read()
                if "first accelerometer sample: FLU (+0.50, -0.20, +9.81)" in txt:
                    imu_ok = True; break
        aspd_ok = True
        if node == "aspd":
            puba = (f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; ros2 topic pub --once -w 1 /{NS}/fmu/out/airspeed_validated "
                    f"px4_msgs/msg/AirspeedValidated \"{{true_airspeed_m_s: 12.5, indicated_airspeed_m_s: 12.4, calibrated_airspeed_m_s: 12.4}}\"")
            subprocess.run(["bash", "-c", puba], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
            t2 = time.time(); aspd_ok = False
            while time.time() - t2 < 10.0:
                time.sleep(0.5)
                txt = open(log, errors="replace").read()
                if "first airspeed_validated: TAS 12.50 m/s" in txt:
                    aspd_ok = True; break
        alive = proc.poll() is None
        nl = subprocess.run(["bash", "-c", f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; timeout 15 ros2 node list"],
                            capture_output=True, text=True).stdout.split()
        want_node = "/nmpc_cascade_control" + ("" if cid == "cascade" else "_" + cid.replace("cascade_", "", 1))
        info = ""
        if node in ("px4src", "aspd"):      # BEFORE the kill: the graph forgets a dead node
            info = subprocess.run(["bash", "-c", f"source {ROOT}/scripts/rosenv.sh 2>/dev/null; timeout 20 ros2 node info {want_node}"],
                                  capture_output=True, text=True).stdout
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
        if want_node not in nl:
            return False, f"{node}: node {want_node} not in `ros2 node list` (got {nl})"
        if not acked:
            return False, f"{node}: no reaction to /{NS}/{cid}/cmd within 10 s (subscription dead?)"
        # the ack must ALSO reach /<ns>/cascade/status -- that is what the ground station's
        # Guidance row displays (before engage the answer is "ignored: no state yet")
        if "ack" not in status_out or "wp 100 0" not in status_out.replace("''", "'"):
            return False, f"{node}: no ack on /{NS}/{cid}/status (got {status_out.strip()[:120]!r})"
        if node == "px4src":
            if "state source: px4" not in txt:
                return False, f"{node}: the node did not report 'state source: px4'"
            if f"/{NS}/fmu/out/vehicle_odometry: px4_msgs/msg/VehicleOdometry" not in info:
                return False, f"{node}: {want_node} does not subscribe to /{NS}/fmu/out/vehicle_odometry"
            if f"/{NS}/mocap:" in info:
                return False, f"{node}: state_source px4 must NOT also subscribe to the mocap (two state sources)"
        if node == "aspd":
            if "airspeed source: px4" not in txt:
                return False, f"{node}: the node did not report 'airspeed source: px4'"
            if f"/{NS}/fmu/out/airspeed_validated: px4_msgs/msg/AirspeedValidated" not in info:
                return False, f"{node}: {want_node} does not subscribe to /{NS}/fmu/out/airspeed_validated"
            if not aspd_ok:
                return False, f"{node}: an AirspeedValidated message (TAS 12.5) was not logged as 'first airspeed_validated: TAS 12.50 m/s' within 10 s"
        if node == "imu":
            if "[cascade+imu] accelerometer: /" + NS + "/fmu/out/sensor_combined" not in txt:
                return False, f"{node}: the sensor_combined subscription was never attached (no '[cascade+imu] accelerometer:' line)"
            if not imu_ok:
                return False, f"{node}: a SensorCombined message on /{NS}/fmu/out/sensor_combined was not logged as FLU (+0.50, -0.20, +9.81) within 10 s"
        return True, f"{node}: booted as {want_node}, acked the command on /{NS}/{cid}/status, alive" + (", accelerometer wired FRD->FLU" if node == "imu" else "") + (", pitot wired (TAS logged)" if node == "aspd" else "")


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
