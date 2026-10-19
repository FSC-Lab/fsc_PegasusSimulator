#!/usr/bin/env python3
"""The RENDERING half of a replayed presentation video (2026-10-18, valve turning).

record_scene.py flew the mission headless (the plant at real time, exactly as in the
results campaign) and logged every rigid body's pose plus the exported stage;
shoot.py captured the two ground stations live and topic_recorder.py logged the
planner topics the overlays are drawn from. This scene re-renders the Isaac view:

  * the recorded stage opened in a window sized like video_scene.py's (the viewport
    fills an undecorated 1244 x 1400 window), the timeline NEVER played -- no physics,
    no stack, no PX4, nothing published;
  * every logged prim set to its recorded world pose, interpolated at the wall-clock
    time of the replay mapped onto the flight's (positions linear, rotations
    normalised-linear), as fast as Kit draws;
  * the overlays redrawn by the scene's OWN methods (06's blue / red planned curves
    and heading arrows, the 07 scene's waypoint balls and labels), fed the logged
    messages at their logged times;
  * the window captured to takes/TAKE/isaac.mp4 at 25 fps; its start, mapped onto
    the flight's wall clock, is added to takes/TAKE/timeline.json as rec.isaac, so
    compose.py cuts it against the ground-station captures unchanged.

    replay.sh TAKE [--start EVENT[+-s]] [--end EVENT[+-s]]
    (Isaac's python, DISPLAY :1, after a clean slate -- nothing else may run)

Camera and window from takes/TAKE/view.json (the view file the recording used).
"""
import argparse
import bisect
import importlib.util
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.expanduser("~/fsc_PegasusSimulator/application/robotic_arm")
SCRIPTS = {"free_flight": "06_px4_t650_aerial_manipulator_free_flight.py",
           "box_pushing": "07_px4_t650_aerial_manipulator_box_pushing.py",
           "valve_turning": "07_px4_t650_aerial_manipulator_valve_turning.py",
           "pick_and_place": "07_px4_t650_aerial_manipulator_pick_and_place.py",   # 2026-10-18
           "pick_and_drop": "07_px4_t650_aerial_manipulator_pick_and_place.py",
           "pick_and_place_assembly": "07_px4_t650_aerial_manipulator_pick_and_place.py"}
CLASSES = {"box_pushing": "AmT650BoxPushing", "valve_turning": "AmT650ValveTurning",
           "pick_and_place": "AmT650PickAndPlace", "pick_and_drop": "AmT650PickAndPlace",
           "pick_and_place_assembly": "AmT650PickAndPlace"}
PICK_SCENES = ("pick_and_place", "pick_and_drop", "pick_and_place_assembly")    # waypoints from pick_place/info + the planner parameters
FPS = 25

ap = argparse.ArgumentParser()
ap.add_argument("take")
ap.add_argument("--start", default="CLIMB-3.0", help="first replayed instant (compose.py starts at CLIMB-0.5)")
ap.add_argument("--end", default="END+1.0", help="last replayed instant (compose.py ends at END)")
ap.add_argument("--preroll", type=float, default=6.0, help="seconds of drawing before the capture (RTX warm-up)")
ap.add_argument("--view", default="", help="camera/window json (default: the take's view.json, the one recorded with)")
a = ap.parse_args()
D = os.path.abspath(os.path.join(HERE, "..", "takes", a.take))
tl = json.load(open(os.path.join(D, "timeline.json")))
meta = json.load(open(os.path.join(D, "poses.json")))
view = json.load(open(a.view or os.path.join(D, "view.json")))
SCENE = meta["scene"]
W, H = view.get("window", [1244, 1400])


def event_time(spec):
    m = re.fullmatch(r"([A-Za-z_:]+)([+-][0-9.]+)?", spec)
    name, off = m.group(1), float(m.group(2) or 0.0)
    # an event (CLIMB, DIRECT, END, ...) or a step mark by its name (push:start, exit:end, ...)
    ts = [e["t"] for e in tl["events"] if e["ev"] == name or e.get("name") == name]
    if not ts:
        sys.exit(f"no event {name} in the take")
    return ts[0] + off


T_BEGIN, T_END = event_time(a.start), event_time(a.end)

# Kit window settings as unknown argv (video_scene.py's), and nothing of ours left in argv
sys.argv = [sys.argv[0],
            "--/app/window/hideUi=1", "--/app/window/noDecorations=1",
            f"--/app/window/width={int(W)}", f"--/app/window/height={int(H)}",
            "--/app/window/x=70", "--/app/window/y=27",
            "--/app/renderer/resolution/width=0", "--/app/renderer/resolution/height=0"]
os.environ["PEGASUS_HEADLESS"] = "0"
_spec = importlib.util.spec_from_file_location(f"replay_{SCENE}", os.path.join(APP, SCRIPTS[SCENE]))
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)            # starts the SimulationApp (06); defines the scene class -- NOT instantiated
am06 = mod.am06 if SCENE in CLASSES else mod
SceneCls = getattr(mod, CLASSES[SCENE]) if SCENE in CLASSES else mod.AmT650WholeBodyArmSim
sim_app = am06.simulation_app

import numpy as np  # noqa: E402  (after the SimulationApp)
import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

kit = omni.kit.app.get_app()


def pump(n=1):
    for _ in range(n):
        kit.update()


# ── the recorded stage ───────────────────────────────────────────────────────
ctx = omni.usd.get_context()
t_open = time.time()
# the export carries the headless run's own viewport render product under /Render: drop it, so the
# replay's viewport makes its own at the window's size
from pxr import Sdf  # noqa: E402
_lay = Sdf.Layer.OpenAsAnonymous(os.path.join(D, "stage.usda"))
if _lay.GetPrimAtPath("/Render"):
    _edit = Sdf.BatchNamespaceEdit()
    _edit.Add(Sdf.NamespaceEdit.Remove("/Render"))
    _lay.Apply(_edit)
_lay.Export(os.path.join(D, "stage_replay.usda"))
# composed into the app's own stage as a SUBLAYER (the live scene was authored into this same default
# stage), not opened as a new one: replacing the stage under the live viewport crashed Kit in
# omni.graph.image.core on the next update (2026-10-18). Pose edits go to the root layer, which is stronger.
stage = ctx.get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
stage.GetRootLayer().subLayerPaths.append(os.path.join(D, "stage_replay.usda"))
pump(30)
while True:
    msg, done, total = ctx.get_stage_loading_status()
    if total == 0 or done >= total:
        break
    pump(5)
pump(60)
stage = ctx.get_stage()
print(f"[REPLAY] stage opened in {time.time() - t_open:.1f} s: {stage.GetRootLayer().identifier}", flush=True)

# ── the log ──────────────────────────────────────────────────────────────────
paths = meta["paths"]
N = len(paths)
raw = np.fromfile(os.path.join(D, "poses.bin"), dtype="<f8")
rec_len = 2 + 7 * N
raw = raw[: (raw.size // rec_len) * rec_len].reshape(-1, rec_len)
WALL = raw[:, 0]
POSE = raw[:, 2:].reshape(-1, N, 7)
print(f"[REPLAY] {len(WALL)} pose records ({N} prims) over {WALL[-1] - WALL[0]:.1f} s; replaying "
      f"{T_END - T_BEGIN:.1f} s ({a.start} .. {a.end})", flush=True)
msgs = []
tp = os.path.join(D, "topics.jsonl")
if os.path.exists(tp):
    with open(tp) as f:
        for line in f:
            try:
                msgs.append(json.loads(line))
            except ValueError:
                pass
msgs.sort(key=lambda m: m["t"])
print(f"[REPLAY] {len(msgs)} overlay messages", flush=True)

# ── the prims: one transform op each, the parent's world transform fixed or logged ──
logged = {p: i for i, p in enumerate(paths)}
xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
items = []
for i, p in enumerate(paths):
    prim = stage.GetPrimAtPath(p)
    if not prim:
        print(f"[REPLAY] WARNING: {p} not in the recorded stage -- skipped", flush=True)
        continue
    xf = UsdGeom.Xformable(prim)
    scale = Gf.Transform(xf.GetLocalTransformation()).GetScale()
    parent = prim.GetParent()
    pidx = logged.get(str(parent.GetPath()))
    pworld = None if pidx is not None else xcache.GetLocalToWorldTransform(parent)
    xf.ClearXformOpOrder()
    op = xf.AddTransformOp()
    items.append((i, op, Gf.Matrix4d().SetScale(Gf.Vec3d(scale)), pidx, pworld))
# parents before children
items.sort(key=lambda it: str(stage.GetPrimAtPath(paths[it[0]]).GetPath()).count("/"))


def world_matrices(t):
    k = int(np.clip(np.searchsorted(WALL, t), 1, len(WALL) - 1))
    w0, w1 = WALL[k - 1], WALL[k]
    s = float(np.clip((t - w0) / max(w1 - w0, 1e-9), 0.0, 1.0))
    P0, P1 = POSE[k - 1], POSE[k]
    pos = (1.0 - s) * P0[:, :3] + s * P1[:, :3]
    q0, q1 = P0[:, 3:], P1[:, 3:]
    sign = np.where(np.sum(q0 * q1, axis=1, keepdims=True) < 0.0, -1.0, 1.0)
    q = (1.0 - s) * q0 + s * sign * q1
    q /= np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)
    return pos, q


def apply_poses(t):
    pos, q = world_matrices(t)
    world = {}
    for i, op, S, pidx, pworld in items:
        if not np.all(np.isfinite(pos[i])) or not np.all(np.isfinite(q[i])):
            continue
        M = Gf.Matrix4d(Gf.Rotation(Gf.Quatd(float(q[i, 0]), Gf.Vec3d(*map(float, q[i, 1:])))),
                        Gf.Vec3d(*map(float, pos[i])))
        world[i] = M
        parent_w = world.get(pidx) if pidx is not None else pworld
        if parent_w is None:
            continue
        op.Set(S * M * parent_w.GetInverse())


# ── the overlays: the scene's own drawing methods on a bare object ──────────
class Overlay:
    pass


ov = Overlay()
ov.stage = stage
for name in ("_wp_setup_view", "_wp_entries", "_wp_make", "_wp_label", "_wp_refresh"):
    if hasattr(SceneCls, name):
        setattr(Overlay, name, getattr(SceneCls, name))
for name in ("_acquire_debug_draw", "_draw_trajectory", "_on_viz_path", "_on_viz_pose"):
    setattr(Overlay, name, getattr(am06.AmT650WholeBodyArmSim, name))
Overlay._arrow_lines = staticmethod(am06.AmT650WholeBodyArmSim._arrow_lines)
ov._viz_path = ov._viz_pose = None
ov._draw = None
ov._draw_ok = False
ov._wp_info = None
ov._wp_ready = False
ov._wp_params = {}                       # pick-and-place: the planner parameters (logged)
ov._payload_pose = lambda: None          # pick-and-place before Get only (Get comes before the video)
ov._acquire_debug_draw()
if hasattr(ov, "_wp_setup_view") and getattr(mod, "WAYPOINTS_VIZ", True):
    try:
        ov._wp_setup_view()
    except Exception as exc:
        print(f"[REPLAY] waypoint view off: {exc}", flush=True)
mi = 0


def apply_overlays(t):
    global mi
    path_new = info_new = pose_new = False
    info_key = "ppinfo" if SCENE in PICK_SCENES else "info"
    while mi < len(msgs) and msgs[mi]["t"] <= t:
        m = msgs[mi]
        mi += 1
        d = types.SimpleNamespace(data=m["data"])
        if m["topic"] == "path":
            ov._on_viz_path(d)
            path_new = True
        elif m["topic"] == "pose":
            ov._on_viz_pose(d)
            pose_new = True
        elif m["topic"] == info_key:
            ov._wp_info = list(m["data"])
            info_new = True
        elif m["topic"] == "params" and SCENE in PICK_SCENES:
            ov._wp_params.update(m["data"])
            info_new = True
    if (path_new or pose_new) and ov._draw is not None:
        ov._draw_trajectory()
    if info_new and ov._wp_ready:
        try:
            ov._wp_refresh()
        except Exception as exc:
            ov._wp_ready = False
            print(f"[REPLAY] waypoint view stopped: {exc}", flush=True)


# ── the camera (video_scene.py's), once: nothing moves it here ──────────────
cam = view.get("camera", {})
try:
    try:
        from isaacsim.core.utils.viewports import set_camera_view
    except ImportError:
        from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(eye=list(cam["pos"]), target=list(cam["target"]), camera_prim_path="/OmniverseKit_Persp")
    if cam.get("focal"):
        prim = stage.GetPrimAtPath("/OmniverseKit_Persp")
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            prim.GetAttribute("focalLength").Set(float(cam["focal"]))
    print(f"[REPLAY] camera pos {cam['pos']} target {cam['target']} focal {cam.get('focal')}", flush=True)
except Exception as exc:
    print(f"[REPLAY] camera failed: {exc}", flush=True)


# ── the capture (shoot.py's pipeline) ────────────────────────────────────────
def xwin(*args):
    return subprocess.run(["/usr/bin/python3", os.path.join(HERE, "xwin.py"), *args],
                          capture_output=True, text=True).stdout.strip()


class Rec:
    def __init__(self, title, out):
        xid = xwin("find", title)
        pipe = (f"ximagesrc xid={xid} use-damage=false show-pointer=false "
                f"! video/x-raw,framerate={FPS}/1 ! videorate ! video/x-raw,framerate={FPS}/1 "
                f"! videoconvert ! video/x-raw,format=I420 "
                f"! queue max-size-buffers=100 max-size-time=0 max-size-bytes=0 "
                f"! x264enc speed-preset=ultrafast tune=zerolatency pass=quant quantizer=12 threads=4 "
                f"! h264parse ! mp4mux ! filesink location={out}")
        self.log = open(out + ".log", "w")
        self.p = subprocess.Popen(["taskset", "-c", "16-31", "gst-launch-1.0", "-e"] + pipe.split(),
                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                  start_new_session=True)
        self.t0 = None
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.p.stdout:
            if self.t0 is None and "Setting pipeline to PLAYING" in line:
                self.t0 = time.time()
            self.log.write(f"{time.time():.3f} {line}")
            self.log.flush()

    def stop(self):
        if self.p.poll() is None:
            os.killpg(self.p.pid, signal.SIGINT)
            try:
                self.p.wait(timeout=60)
            except subprocess.TimeoutExpired:
                os.killpg(self.p.pid, signal.SIGKILL)


xwin("place", "Isaac Sim", "70", "27", str(W), str(H), "--undecorate")
apply_poses(T_BEGIN)
apply_overlays(T_BEGIN)
t_pre = time.time()
while time.time() - t_pre < a.preroll:
    pump()
rec = Rec("Isaac Sim", os.path.join(D, "isaac.mp4"))
while rec.t0 is None and rec.p.poll() is None:
    pump()
if rec.t0 is None:
    sys.exit(f"the Isaac capture died at start, see {D}/isaac.mp4.log")
t_cap = time.time()
while time.time() - t_cap < 1.0:
    pump()
w0 = time.time()
frames = 0
next_report = w0 + 10.0
while True:
    now = time.time()
    t = T_BEGIN + (now - w0)
    if t > T_END:
        break
    apply_poses(t)
    apply_overlays(t)
    pump()
    frames += 1
    if now >= next_report:
        next_report += 10.0
        print(f"[REPLAY] {t - T_BEGIN:6.1f} / {T_END - T_BEGIN:.1f} s, {frames / (now - w0):.1f} drawn frames/s",
              flush=True)
w_end = time.time()
pump(5)
rec.stop()
offset = w0 - T_BEGIN              # replay wall clock = flight wall clock + offset
tl["rec"]["isaac"] = {"t0": rec.t0 - offset, "replay": {"t_cap": rec.t0, "w0": w0, "t_begin": T_BEGIN,
                                                        "t_end": T_END, "frames_drawn": frames,
                                                        "draw_fps": frames / (w_end - w0)}}
with open(os.path.join(D, "timeline.json"), "w") as f:
    json.dump(tl, f, indent=1)
print(f"[REPLAY] done: {frames} frames drawn in {w_end - w0:.1f} s ({frames / (w_end - w0):.1f} fps) -> "
      f"{D}/isaac.mp4; timeline.json rec.isaac added", flush=True)
sim_app.close()
