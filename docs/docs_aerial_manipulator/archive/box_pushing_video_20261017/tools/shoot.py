#!/usr/bin/env python3
"""Record one presentation-video mission: each window to its own file while the
mission driver flies, then compose.py cuts them into the side-by-side video.

    shoot.py TAKE -- DRIVER ARGS...        (the driver prints one JSON event per line)

  Isaac      the whole flight
  drone GS   from the start until SWITCH_AFTER_DIRECT + 2 s past the DIRECT event
  arm GS     from the DIRECT event to the end
Separate captures, not one pipeline: each window read-back stalls Isaac's frame a
little, and three at once held the simulation below real time. Each capture's
wall-clock start is logged, so compose.py can line the files up.
Writes ../takes/TAKE/{isaac,drone,arm}.mp4, timeline.json, driver.log, rec_*.log.

REPLAYED videos (2026-10-18, setup.sh with REPLAY_TAKE): SHOOT_WINDOWS=drone,arm
captures only the ground stations (the scene runs headless), and SHOOT_TOPICS=1 also
runs topic_recorder.py -> TAKE/topics.jsonl for the overlays; replay_scene.py then
renders isaac.mp4 from the flight's log and adds it to timeline.json.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
FPS = 25
SWITCH_AFTER_DIRECT = 3.0
WINDOWS = {"isaac": "Isaac Sim", "drone": "Drone Ground Station", "arm": "Indoor Robotic Arm"}
CAPTURE = [w for w in os.environ.get("SHOOT_WINDOWS", "isaac,drone,arm").split(",") if w]

take = sys.argv[1]
assert sys.argv[2] == "--", "shoot.py TAKE -- DRIVER ARGS..."
driver_cmd = sys.argv[3:]
OUT = os.path.join(HERE, "..", "takes", take)
os.makedirs(OUT, exist_ok=True)
timeline = {"fps": FPS, "switch_after_direct": SWITCH_AFTER_DIRECT, "rec": {}, "events": []}


def xid(title):
    return subprocess.run([sys.executable, os.path.join(HERE, "xwin.py"), "find", title],
                          capture_output=True, text=True, check=True).stdout.strip()


class Rec:
    def __init__(self, name):
        self.name = name
        pipe = (f"ximagesrc xid={xid(WINDOWS[name])} use-damage=false show-pointer=false "
                f"! video/x-raw,framerate={FPS}/1 ! videorate ! video/x-raw,framerate={FPS}/1 "
                f"! videoconvert ! video/x-raw,format=I420 "
                f"! queue max-size-buffers=100 max-size-time=0 max-size-bytes=0 "
                f"! x264enc speed-preset=ultrafast tune=zerolatency pass=quant quantizer=12 threads=4 "
                f"! h264parse ! mp4mux ! filesink location={os.path.join(OUT, name + '.mp4')}")
        self.log = open(os.path.join(OUT, f"rec_{name}.log"), "w")
        self.t_launch = time.time()
        self.p = subprocess.Popen(["taskset", "-c", "16-31", "gst-launch-1.0", "-e"] + pipe.split(),
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                  start_new_session=True)
        self.t0 = None
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            if self.t0 is None and "Setting pipeline to PLAYING" in line:
                self.t0 = time.time()
                timeline["rec"][self.name] = {"t0": self.t0, "t_launch": self.t_launch}
            self.log.write(f"{time.time():.3f} {line}")
            self.log.flush()

    def stop(self):
        if "t_stop" in timeline["rec"].get(self.name, {}):
            return                          # already stopped (the drone GS's timer)
        if self.p.poll() is None:
            os.killpg(self.p.pid, signal.SIGINT)
            try:
                self.p.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGKILL)
        timeline["rec"].setdefault(self.name, {})["t_stop"] = time.time()
        print(f"[{time.strftime('%T')}] {self.name} capture stopped (rc {self.p.returncode})", flush=True)


recs = {n: Rec(n) for n in ("isaac", "drone") if n in CAPTURE}
time.sleep(2.0)
for r in recs.values():
    if r.p.poll() is not None:
        sys.exit(f"capture {r.name} died at start, see {OUT}/rec_{r.name}.log")
print(f"[{time.strftime('%T')}] recording -> {OUT}", flush=True)

env = dict(os.environ, FASTRTPS_DEFAULT_PROFILES_FILE=os.path.expanduser(
    "~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"))
ros = ("source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/rosdeps/local_setup.bash 2>/dev/null; "
       "source $HOME/ros2_ws/install/setup.bash; ")
topics = None
if os.environ.get("SHOOT_TOPICS") == "1":
    topics = subprocess.Popen(["bash", "-c", ros + "exec /usr/bin/python3 " + os.path.join(HERE, "topic_recorder.py")
                               + " " + os.path.join(OUT, "topics.jsonl")],
                              stdout=open(os.path.join(OUT, "topic_recorder.log"), "w"), stderr=subprocess.STDOUT,
                              env=env, start_new_session=True)
    time.sleep(2.0)
src = ros + "exec /usr/bin/python3 " + " ".join(driver_cmd)
drv = subprocess.Popen(["bash", "-c", src], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
dlog = open(os.path.join(OUT, "driver.log"), "w")
drone_stop_at = None
for line in drv.stdout:
    dlog.write(line)
    dlog.flush()
    try:
        e = json.loads(line)
    except ValueError:
        continue
    timeline["events"].append(e)
    print(f"[{time.strftime('%T')}] {e['ev']} {', '.join(f'{k}={v}' for k, v in e.items() if k not in ('t', 'ev'))}",
          flush=True)
    if e["ev"] == "DIRECT" and "arm" not in recs and "arm" in CAPTURE:
        recs["arm"] = Rec("arm")
        drone_stop_at = e["t"] + SWITCH_AFTER_DIRECT + 2.0
        if "drone" in recs:
            threading.Thread(target=lambda: (time.sleep(max(0.0, drone_stop_at - time.time())), recs["drone"].stop()),
                             daemon=True).start()
    if e["ev"] in ("END", "ABORT", "EXIT"):
        break
time.sleep(0.5)
for r in recs.values():
    r.stop()
if topics is not None and topics.poll() is None:
    os.killpg(topics.pid, signal.SIGINT)
    try:
        topics.wait(timeout=10)
    except subprocess.TimeoutExpired:
        os.killpg(topics.pid, signal.SIGKILL)
with open(os.path.join(OUT, "timeline.json"), "w") as f:
    json.dump(timeline, f, indent=1)
rest = drv.communicate(timeout=60)[0] if drv.poll() is None else drv.stdout.read()
dlog.write(rest or "")
print(f"[{time.strftime('%T')}] done (driver rc {drv.returncode}) -> {OUT}", flush=True)
