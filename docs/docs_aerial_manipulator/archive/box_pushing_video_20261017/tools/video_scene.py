#!/usr/bin/env python3
"""An Isaac scene for the presentation videos: 06 (free flight: the EE circle and
figure-8) or 07 (box pushing), unchanged except for the WINDOW -- the viewport
fills an undecorated window sized for the left half of a 2560x1440 frame,
rendered at that size (not the default 1280x720 texture) -- and the viewport
camera, read from video_view.json and RE-APPLIED every 0.5 s (edit the json while
the scene runs to frame the shot; the re-apply also stops the view drifting during
take-off, the 10-12 recording's trap).

Started by the 4-D launcher as AM_ISAAC_SCENE_SCRIPT (setup.sh). 06 / 07 are loaded
as modules, exactly as 07 loads 06, so neither file is modified. The launcher's
environment does not reach the Isaac pane (tmux server), so the settings live in
the json:

    {"scene": "free_flight" | "box_pushing",
     "window": [W, H],                      app window = viewport size, px
     "render_every": N,                     draw a frame every N 4 ms physics steps
     "camera": {"pos": [x, y, z], "target": [x, y, z], "focal": mm}}
"""
import importlib.util
import json
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "video_view.json")
APP = os.path.expanduser("~/fsc_PegasusSimulator/application/robotic_arm")
SCRIPTS = {"free_flight": "06_px4_t650_aerial_manipulator_free_flight.py",
           "box_pushing": "07_px4_t650_aerial_manipulator_box_pushing.py",
           "valve_turning": "07_px4_t650_aerial_manipulator_valve_turning.py",   # 2026-10-17
           "pick_and_place": "07_px4_t650_aerial_manipulator_pick_and_place.py",  # 2026-10-18
           "pick_and_drop": "07_px4_t650_aerial_manipulator_pick_and_place.py",   # (PEGASUS_PNP_TASK=drop, setup.sh)
           "pick_and_place_assembly": "07_px4_t650_aerial_manipulator_pick_and_place.py"}   # (the knobs: setup.sh)
TASK_SCENES = ("box_pushing", "valve_turning", "pick_and_place", "pick_and_drop", "pick_and_place_assembly")   # 07 scenes: 06 loaded as a module, own CAMERA_*


def _cfg():
    try:
        with open(CFG) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


cfg = _cfg()
SCENE = cfg.get("scene", "box_pushing")
W, H = cfg.get("window", [1244, 1400])
# Kit settings travel as unknown command-line args: SimulationApp appends them
# AFTER its own, so these win (renderer resolution 0 x 0 = render at the
# viewport's UI size; hideUi = the viewport fills the main window)
sys.argv += [
    "--/app/window/hideUi=1",
    "--/app/window/noDecorations=1",
    f"--/app/window/width={int(W)}",
    f"--/app/window/height={int(H)}",
    "--/app/window/x=70",
    "--/app/window/y=27",
    "--/app/renderer/resolution/width=0",
    "--/app/renderer/resolution/height=0",
]

_spec = importlib.util.spec_from_file_location(f"video_{SCENE}", os.path.join(APP, SCRIPTS[SCENE]))
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)           # starts the SimulationApp (06), builds the scene module
am06 = mod.am06 if SCENE in TASK_SCENES else mod
Base = {"box_pushing": getattr(mod, "AmT650BoxPushing", None), "valve_turning": getattr(mod, "AmT650ValveTurning", None),
        "pick_and_place": getattr(mod, "AmT650PickAndPlace", None),
        "pick_and_drop": getattr(mod, "AmT650PickAndPlace", None),
        "pick_and_place_assembly": getattr(mod, "AmT650PickAndPlace", None)}.get(SCENE) \
    or mod.AmT650WholeBodyArmSim
if cfg.get("render_every"):
    am06.RENDER_EVERY = int(cfg["render_every"])      # read by 06's run loop at run time

cam0 = cfg.get("camera", {})
if SCENE in TASK_SCENES:
    if "pos" in cam0:
        mod.CAMERA_POS = list(cam0["pos"])
    if "target" in cam0:
        mod.CAMERA_TARGET = list(cam0["target"])

import omni.kit.app  # noqa: E402  (after the SimulationApp)
import omni.usd  # noqa: E402
from pxr import Usd  # noqa: E402


class VideoScene(Base):
    def __init__(self):
        super().__init__()
        self._vid_last = 0.0
        self._vid_cam = None
        self._vid_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
            self._vid_tick, name="presentation_video_camera")
        print(f"\033[1;35m[VIDEO] {SCENE}: window {W}x{H}, a frame every {am06.RENDER_EVERY} physics steps, "
              f"viewport camera from {CFG} (re-applied every 0.5 s)\033[0m", flush=True)

    def _vid_tick(self, _e):
        now = time.time()
        if now - self._vid_last < 0.5:
            return
        self._vid_last = now
        cam = _cfg().get("camera", {})
        if "pos" not in cam or "target" not in cam:
            return
        pos, tgt, focal = cam["pos"], cam["target"], cam.get("focal")
        try:
            self.pg.set_viewport_camera(list(pos), list(tgt))
            if focal:
                stage = omni.usd.get_context().get_stage()
                prim = stage.GetPrimAtPath("/OmniverseKit_Persp")
                if prim.IsValid():
                    with Usd.EditContext(stage, stage.GetSessionLayer()):
                        prim.GetAttribute("focalLength").Set(float(focal))
            key = (tuple(pos), tuple(tgt), focal)
            if key != self._vid_cam:
                self._vid_cam = key
                print(f"[VIDEO] camera pos {list(pos)} target {list(tgt)} focal {focal}", flush=True)
        except Exception as e:   # never let the camera take the run down
            print(f"[VIDEO] camera update failed: {e}", flush=True)


if __name__ == "__main__":
    VideoScene().run()
