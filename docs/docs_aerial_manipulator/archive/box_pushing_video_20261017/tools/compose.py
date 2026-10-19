#!/usr/bin/env python3
"""Cut a take (shoot.py) into the presentation video: 2560 x 1440, 25 fps, Isaac on
the left, the drone GS on the right until SWITCH_AFTER_DIRECT s past the DIRECT
switch, the arm GS after it. Run with PYTHONNOUSERSITE=1 /usr/bin/python3.

    compose.py TAKE OUT.mp4 [--start EVENT[+-s]] [--end EVENT[+-s]] [--quality Q]

  --start  default CLIMB-0.5   (CLIMB = the arm command; the climb follows 1 s later)
  --end    default END         (the driver's last event: the vehicle hovering after the task)
  --skip A:B  (repeatable, 2026-10-18) cut the span from event A to event B out of the video (e.g. the
           hover after the DIRECT switch: --skip DIRECT+5:go_to_start:start-1); events as for --start,
           including step marks by name
"""
import argparse
import json
import os
import re
import subprocess
import sys

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame import FH, FW, compose  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def event_time(tl, spec):
    m = re.fullmatch(r"([A-Za-z_:]+)([+-][0-9.]+)?", spec)
    name, off = m.group(1), float(m.group(2) or 0.0)
    # an event (CLIMB, DIRECT, END, ...) or a step mark by its name (push:start, exit:end, ...)
    ts = [e["t"] for e in tl["events"] if e["ev"] == name or e.get("name") == name]
    if not ts:
        sys.exit(f"no event {name} in the take")
    return ts[0] + off


class Stream:
    """Frames of one capture by WALL time (its start logged by shoot.py)."""
    def __init__(self, path, t0, fps):
        self.cap = cv2.VideoCapture(path)
        self.t0, self.fps = t0, fps
        self.k, self.img = -1, None
        self.n = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

    def at(self, t):
        want = max(0, int(round((t - self.t0) * self.fps)))
        while self.k < want:
            ok, img = self.cap.read()
            if not ok:
                break
            self.k += 1
            self.img = img
        return self.img


ap = argparse.ArgumentParser()
ap.add_argument("take")
ap.add_argument("out")
ap.add_argument("--start", default="CLIMB-0.5")
ap.add_argument("--end", default="END")
ap.add_argument("--quality", type=int, default=17, help="x264 constant quality (lower = better)")
ap.add_argument("--skip", action="append", default=[], help="A:B -- cut the span between two events")
a = ap.parse_args()
D = os.path.join(HERE, "..", "takes", a.take)
tl = json.load(open(os.path.join(D, "timeline.json")))
fps = tl["fps"]
t_start, t_end = event_time(tl, a.start), event_time(tl, a.end)
t_switch = event_time(tl, "DIRECT") + tl["switch_after_direct"]
S = {n: Stream(os.path.join(D, n + ".mp4"), tl["rec"][n]["t0"], fps) for n in ("isaac", "drone", "arm")}
print(f"cut {t_end - t_start:.1f} s: start {a.start}, switch to the arm GS at DIRECT+{tl['switch_after_direct']} s "
      f"({t_switch - t_start:.1f} s in), end {a.end}", flush=True)

enc = subprocess.Popen(
    ["gst-launch-1.0", "-q", "-e", "fdsrc", "fd=0", "!",
     "rawvideoparse", f"width={FW}", f"height={FH}", "format=bgr", f"framerate={fps}/1", "!",
     "videoconvert", "!", "video/x-raw,format=I420", "!",
     "x264enc", "speed-preset=slow", "pass=qual", f"quantizer={a.quality}", "bframes=0", "!",
     "video/x-h264,profile=high", "!", "h264parse", "!", "mp4mux", "faststart=true", "!",
     "filesink", f"location={a.out}"], stdin=subprocess.PIPE)
def split_skip(spec):
    # A:B where an event name may itself contain a colon (push:start): split at the colon that
    # leaves two parseable events
    parts = spec.split(":")
    for k in range(1, len(parts)):
        a_, b_ = ":".join(parts[:k]), ":".join(parts[k:])
        try:
            return event_time(tl, a_), event_time(tl, b_)
        except (SystemExit, AttributeError):
            continue
    sys.exit(f"cannot parse --skip {spec}")


skips = sorted(split_skip(sp) for sp in a.skip)
cut = sum(max(0.0, min(b_, t_end) - max(a_, t_start)) for a_, b_ in skips)
for a_, b_ in skips:
    print(f"  skip {b_ - a_:.1f} s at {a_ - t_start:.1f} s", flush=True)
n = int(round((t_end - t_start - cut) * fps))
for i in range(n):
    t = t_start + i / fps
    for a_, b_ in skips:
        if t >= a_:
            t += max(0.0, b_ - max(a_, t_start)) if a_ >= t_start else 0.0
    right = S["drone"].at(t) if t < t_switch else S["arm"].at(t)
    enc.stdin.write(compose(S["isaac"].at(t), right).tobytes())
    if i % (10 * fps) == 0:
        print(f"  {i / fps:5.1f} / {n / fps:.1f} s", flush=True)
enc.stdin.close()
enc.wait()
print(f"wrote {a.out} ({n} frames, {n / fps:.1f} s)", flush=True)
