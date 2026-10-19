#!/usr/bin/env python3
"""The PICK-AND-PLACE mission for the presentation video (2026-10-18):
pick_place_controllers_20261003/tools/pnp_mission_v2.py unchanged (run it with its
own arguments, e.g. --rig wb --hook-place), plus the JSON event lines shoot.py cuts
on: CLIMB = the arm command (the climb reference follows 2 s later), DIRECT = the
switch, MARK = every step mark, END = the planner's Land step done and the
post-hold over (the vehicle hovering at the land point; the driver then lands in
SAFETY, after the recording has stopped). The pl_video_driver.py pattern.

    pnp_video_driver.py --rig wb --hook-place --out TAKE/mission.npz ...
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                "pick_place_controllers_20261003", "tools"))
import pnp_mission_v2 as M  # noqa: E402


def jev(event, **kw):
    print(json.dumps({"t": time.time(), "ev": event, **kw}), flush=True)


class VideoDriver(M.Driver):
    def trig(self, name, must=True):
        if name == "arm":
            jev("CLIMB")
        return super().trig(name, must)

    def mark(self, name):
        super().mark(name)
        jev("DIRECT" if name == "direct" else "MARK", name=name)

    def mission(self):
        super().mission()
        jev("END")


M.Driver = VideoDriver
sys.exit(M.main())
