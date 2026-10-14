#!/usr/bin/env python3
"""Record the Isaac window (left half) and the arm ground station (right half) side by
side while the autonomous pick-and-drop driver flies the mission.

    shoot.py <isaac_xid> <gs_xid> <out.mp4> [half_w half_h]
"""
import os
import signal
import subprocess
import sys
import time

isaac, gs, out = sys.argv[1], sys.argv[2], os.path.expanduser(sys.argv[3])
W = int(sys.argv[4]) if len(sys.argv) > 4 else 1280
H = int(sys.argv[5]) if len(sys.argv) > 5 else 800
V = os.path.dirname(os.path.abspath(__file__))
DRIVER = os.path.expanduser("~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/"
                            "pick_place_controllers_20261003/tools/pnp_mission_v2.py")

# the Isaac window is cropped to its viewport (the Stage / Property / Content panels around
# it are dropped): ISAAC_CROP = left right top bottom in pixels
CROP = [int(v) for v in os.environ.get("ISAAC_CROP", "53 440 64 316").split()]
src = ("ximagesrc xid={x} use-damage=false show-pointer=false ! video/x-raw,framerate=30/1 "
       "{crop}! videoconvert ! queue ! mix.sink_{i}")
isaac_crop = "! videocrop left={} right={} top={} bottom={} ".format(*CROP)
pipe = (f"compositor name=mix background=black "
        f"sink_0::xpos=0 sink_0::ypos=0 sink_0::width={W} sink_0::height={H} sink_0::sizing-policy=keep-aspect-ratio "
        f"sink_1::xpos={W} sink_1::ypos=0 sink_1::width={W} sink_1::height={H} sink_1::sizing-policy=keep-aspect-ratio "
        f"! videorate ! video/x-raw,width={2 * W},height={H},framerate=30/1 ! videoconvert ! video/x-raw,format=I420 "
        f"! x264enc speed-preset=veryfast tune=zerolatency bitrate=6000 ! h264parse ! mp4mux faststart=true "
        f"! filesink location={out} "
        + src.format(x=isaac, i=0, crop=isaac_crop) + " " + src.format(x=gs, i=1, crop=""))
env = dict(os.environ, DISPLAY=":1")
rec_log = open(os.path.join(V, "recorder.log"), "w")
rec = subprocess.Popen(["taskset", "-c", "16-31", "gst-launch-1.0", "-e"] + pipe.split(),
                       stdout=rec_log, stderr=subprocess.STDOUT, env=env, preexec_fn=os.setsid)
time.sleep(3)
if rec.poll() is not None:
    sys.exit(f"recorder died at start, see {V}/recorder.log")
print(f"[{time.strftime('%T')}] recording -> {out}", flush=True)

drv_env = dict(env, FASTRTPS_DEFAULT_PROFILES_FILE=os.path.expanduser(
    "~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"))
with open(os.path.join(V, "driver.log"), "w") as dl:
    # the same sourcing as run_pnp.sh (an older workspace's messages may be on the path)
    cmd = ("source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/rosdeps/local_setup.bash 2>/dev/null; "
           "source $HOME/ros2_ws/install/setup.bash; "
           f"exec /usr/bin/python3 {DRIVER} --rig wb --drop-place --out {os.path.join(V, 'video_run.npz')}")
    rc = subprocess.call(["bash", "-c", cmd], stdout=dl, stderr=subprocess.STDOUT, env=drv_env)
print(f"[{time.strftime('%T')}] driver exited rc={rc}", flush=True)
# propellers off after the landing (PX4 refuses the plain disarm on this rig): the forced
# disarm, then a few seconds of the stopped rotors on the recording
SRC = ("source /opt/ros/humble/setup.bash; source $HOME/ros2_ws/rosdeps/local_setup.bash 2>/dev/null; "
       "source $HOME/ros2_ws/install/setup.bash; ")
out_d = subprocess.run(["bash", "-c", SRC + f"exec /usr/bin/python3 {os.path.join(V, 'force_disarm.py')}"],
                       capture_output=True, text=True, env=drv_env)
print(f"[{time.strftime('%T')}] force disarm rc={out_d.returncode}: {out_d.stdout.strip()} {out_d.stderr.strip()[-200:]}",
      flush=True)
time.sleep(6)
os.killpg(rec.pid, signal.SIGINT)
try:
    rec.wait(timeout=60)
except subprocess.TimeoutExpired:
    os.killpg(rec.pid, signal.SIGKILL)
print(f"[{time.strftime('%T')}] recorder stopped rc={rec.returncode}", flush=True)
print("done", flush=True)
