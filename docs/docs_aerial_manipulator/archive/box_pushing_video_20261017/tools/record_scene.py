#!/usr/bin/env python3
"""The RECORDING half of a replayed presentation video (2026-10-18, valve turning).

The valve's contact phase already overruns the 4 ms physics step, so a windowed
scene cannot draw frames without slowing the plant the wall-clock controller flies
(box_pushing_video_20261017/README.md, "Valve turning"). This scene flies the
mission HEADLESS -- the plant exactly as in the results campaign -- and only
records what the window would have shown:

  <out>/stage.usda   the stage's root layer, exported once the scene is built (the
                     environment, table, valve and vehicle are references in it)
  <out>/poses.json   the logged prims (every rigid body of the vehicle + the valve
                     rotor or the pick-and-place payload) and the record layout
  <out>/poses.bin    one record every EVERY_STEPS physics steps (default 5 = 50 Hz):
                     float64 [wall time, sim time, then per prim px py pz qw qx qy qz]

replay_scene.py then re-renders the flight from these in a window, with no physics
and no stack, at a steady 25 fps. The planner topics the viewport overlays are drawn
from are logged by topic_recorder.py (a separate process, started by shoot.py).

Started by the 4-D launcher as AM_ISAAC_SCENE_SCRIPT with PEGASUS_HEADLESS=1
(setup.sh with REPLAY_TAKE=<take>). The launcher's environment does not reach the
Isaac pane, so the settings live in record_cfg.json next to this file:
    {"scene": "valve_turning", "out_dir": "/abs/path", "every_steps": 5}
The scene class is the task's own, unchanged; the logging runs after its control
step and can never stop the plant.
"""
import importlib.util
import json
import math
import os
import struct
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, "record_cfg.json")
APP = os.path.expanduser("~/fsc_PegasusSimulator/application/robotic_arm")
SCRIPTS = {"free_flight": "06_px4_t650_aerial_manipulator_free_flight.py",
           "box_pushing": "07_px4_t650_aerial_manipulator_box_pushing.py",
           "valve_turning": "07_px4_t650_aerial_manipulator_valve_turning.py",
           "pick_and_place": "07_px4_t650_aerial_manipulator_pick_and_place.py",   # 2026-10-18
           "pick_and_drop": "07_px4_t650_aerial_manipulator_pick_and_place.py",
           "pick_and_place_assembly": "07_px4_t650_aerial_manipulator_pick_and_place.py"}    # (PEGASUS_PNP_TASK=drop)
CLASSES = {"box_pushing": "AmT650BoxPushing", "valve_turning": "AmT650ValveTurning",
           "pick_and_place": "AmT650PickAndPlace", "pick_and_drop": "AmT650PickAndPlace",
           "pick_and_place_assembly": "AmT650PickAndPlace"}

with open(CFG) as f:
    cfg = json.load(f)
SCENE = cfg.get("scene", "valve_turning")
OUT = cfg["out_dir"]
EVERY = int(cfg.get("every_steps", 5))

_spec = importlib.util.spec_from_file_location(f"record_{SCENE}", os.path.join(APP, SCRIPTS[SCENE]))
mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mod)           # starts the SimulationApp (06), builds the scene module
Base = getattr(mod, CLASSES[SCENE]) if SCENE in CLASSES else mod.AmT650WholeBodyArmSim

import omni.usd  # noqa: E402  (after the SimulationApp)
from pxr import Usd, UsdPhysics  # noqa: E402


class RecordScene(Base):
    def __init__(self):
        super().__init__()
        stage = omni.usd.get_context().get_stage()
        paths = [str(p.GetPath()) for p in Usd.PrimRange(stage.GetPrimAtPath(self.drone_path))
                 if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        # the task's one moving body outside the vehicle: the valve rotor, or the
        # pick-and-place payload (2026-10-18)
        for name in ("ROTOR_PRIM", "PAYLOAD_PRIM"):
            extra = getattr(mod, name, None)
            if extra and stage.GetPrimAtPath(extra) and extra not in paths:
                paths.append(extra)
        self._rec_paths = paths
        self._rec_h = None
        self._rec_n = 0
        self._rec_err = None
        os.makedirs(OUT, exist_ok=True)
        t = time.time()
        stage.GetRootLayer().Export(os.path.join(OUT, "stage.usda"))
        with open(os.path.join(OUT, "poses.json"), "w") as f:
            json.dump({"scene": SCENE, "paths": paths, "drone_path": self.drone_path,
                       "every_steps": EVERY, "record": ["wall_s", "sim_s"] + ["px", "py", "pz", "qw", "qx", "qy", "qz"],
                       "n_prims": len(paths)}, f, indent=1)
        self._rec_f = open(os.path.join(OUT, "poses.bin"), "wb")
        self._rec_flush = time.time()
        print(f"\033[1;35m[RECORD] {SCENE}: stage exported ({time.time() - t:.1f} s), logging {len(paths)} rigid "
              f"bodies every {EVERY} physics steps -> {OUT}\033[0m", flush=True)

    def _control_step_inner(self, dt):
        super()._control_step_inner(dt)
        self._rec_n += 1
        if self._rec_n % EVERY:
            return
        try:
            self._rec_log()
        except Exception as exc:             # a recording must never stop the plant
            if self._rec_err is None:
                self._rec_err = exc
                print(f"[RECORD] logging failed once ({exc}); continuing", flush=True)

    def _rec_log(self):
        dc = self._dc
        if self._rec_h is None or not all(self._rec_h):
            self._rec_h = [dc.get_rigid_body(p) for p in self._rec_paths]
        vals = [time.time(), float(self.world.current_time)]
        for h in self._rec_h:
            if not h:
                vals += [math.nan] * 7
                continue
            P = dc.get_rigid_body_pose(h)
            vals += [P.p.x, P.p.y, P.p.z, P.r.w, P.r.x, P.r.y, P.r.z]
        self._rec_f.write(struct.pack(f"<{len(vals)}d", *vals))
        now = time.time()
        if now - self._rec_flush > 1.0:
            self._rec_f.flush()
            self._rec_flush = now


if __name__ == "__main__":
    RecordScene().run()
