#!/usr/bin/env python3
"""The BOX-PUSHING mission for the presentation video: push_pull_20261003/tools/
pl_mission.py unchanged (run it with its own arguments), plus the JSON event
lines shoot.py cuts on:  CLIMB = the arm command (the climb reference follows
2 s later), DIRECT = the switch, MARK = every step mark, END = the planner's Land
step done and the post-hold over (the vehicle hovering at the land point; the
driver then lands in SAFETY, after the recording has stopped).

    pl_video_driver.py --release-at-open --direct-settle 6 ... (pl_mission.py's options)
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                "push_pull_20261003", "tools"))
import pl_mission as M  # noqa: E402


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
