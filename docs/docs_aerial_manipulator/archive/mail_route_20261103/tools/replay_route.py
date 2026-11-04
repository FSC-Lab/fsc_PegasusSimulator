#!/usr/bin/env python3
"""Replay a mail-route run as a video (2026-11-03): the recorded flight drawn in a TOP
and a SIDE view -- the drone (its four 15" prop discs, from the recorded pose), the arm
to the claw, the three envelopes and their trails, the hats, the console table and the
two mailboxes, the planner's status line -- from runs/<tag>.npz and runs/<tag>.log. It
replays what the headless run did; nothing is re-simulated.

    replay_route.py runs/route21 [--hat-line 0.70,0.0,0.6] [--speed 4] [--fps 15] [--out runs/route21.mp4]

--hat-line is the scene's PEGASUS_ROUTE_HAT_LINE (nearest x, y, spacing) the run used.
"""
import argparse
import re

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation, patches

ap = argparse.ArgumentParser()
ap.add_argument("run", help="runs/<tag> (without .npz)")
ap.add_argument("--hat-line", default="0.70,0.0,0.6")
ap.add_argument("--speed", type=float, default=4.0)
ap.add_argument("--fps", type=int, default=15)
ap.add_argument("--out", default=None)
a = ap.parse_args()
out = a.out or a.run + ".mp4"

# the scene (08_px4_t650_aerial_manipulator_mail_route.py defaults)
HX, HY, HS = (float(v) for v in a.hat_line.split(","))
HATS = [(HX - HS * k, HY) for k in range(3)]
HAT_R, HAT_TOP = 0.1003, 0.80
TABLE_X0, TABLE_DX, TABLE_DY, TABLE_TOP = 1.70, 0.15, 1.0, 0.818
BOX_W, BOX_D, BOX_H = 0.235, 0.125, 0.2725
BOXES = {"RED": (0.25, "#c81010"), "BLUE": (-0.25, "#1438b8")}
MOUTH_Z0, MOUTH_H = TABLE_TOP + 0.2025, 0.090
ROTOR, PROP_R = 0.2299, 0.19
ENV_L = 0.150
LETTER_RGB = ["#e05a00", "#7a2bd6", "#0a9a5a"]

d = np.load(a.run + ".npz", allow_pickle=True)
o = d["odom"]
env = [d[f"env{k}"] for k in range(3)]
claw = d["claw"]
names = [str(n) for n in d["marks_name"]]
mt = d["marks_t"]
log = open(a.run + ".log").read()
status = [(float(m.group(1)), m.group(2)) for m in re.finditer(r"^\[\s*([0-9.]+)s\] pick_place: (.*)$", log, re.M)]
done = re.search(r"\[\s*([0-9.]+)s\] pick_place: COMPLETE", log)
t_direct = mt[names.index("direct")] if "direct" in names else o[0, 0]
t0 = t_direct - 8.0
t1 = (float(done.group(1)) + 6.0) if done else o[-1, 0]


def at(arr, t):
    i = min(np.searchsorted(arr[:, 0], t), len(arr) - 1)
    return arr[i]


def rot(qw, qx, qy, qz):
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])


fig, (ax_top, ax_side) = plt.subplots(1, 2, figsize=(14, 6.6), gridspec_kw={"width_ratios": [1.15, 1]})
fig.subplots_adjust(left=0.05, right=0.98, top=0.86, bottom=0.08, wspace=0.18)
ys = [h[1] for h in HATS] + [-1.0, 0.25, -0.25]
ax_top.set_xlim(min(h[0] for h in HATS) - 0.8, 2.05)
ax_top.set_ylim(min(ys) - 0.5, max(ys) + 0.5)
ax_top.set_aspect("equal")
ax_top.set_title("top view (x forward to the mailboxes, y left)")
ax_top.set_xlabel("x [m]")
ax_top.set_ylabel("y [m]")
ax_side.set_xlim(min(h[0] for h in HATS) - 0.8, 2.05)
ax_side.set_ylim(0.0, 1.6)
ax_side.set_aspect("equal")
ax_side.set_title("side view (looking along +y)")
ax_side.set_xlabel("x [m]")
ax_side.set_ylabel("z [m]")
for ax in (ax_top, ax_side):
    ax.grid(alpha=0.25)
# static scene
for k, (hx, hy) in enumerate(HATS):
    ax_top.add_patch(patches.Circle((hx, hy), HAT_R, fc="#d8d8d8", ec="#777"))
    ax_top.text(hx, hy - HAT_R - 0.07, f"hat {k + 1}", ha="center", fontsize=8, color="#555")
    ax_side.add_patch(patches.Rectangle((hx - HAT_R, 0.0), 2 * HAT_R, HAT_TOP, fc="#e4e4e4", ec="#999", lw=0.6))
ax_top.add_patch(patches.Rectangle((TABLE_X0, -TABLE_DY / 2), TABLE_DX, TABLE_DY, fc="#efe3cf", ec="#a08060"))
ax_side.add_patch(patches.Rectangle((TABLE_X0, 0.0), TABLE_DX, TABLE_TOP, fc="#efe3cf", ec="#a08060"))
for name, (by, col) in BOXES.items():
    ax_top.add_patch(patches.Rectangle((TABLE_X0, by - BOX_W / 2), BOX_D, BOX_W, fc=col, alpha=0.55, ec=col))
    ax_top.text(TABLE_X0 + BOX_D + 0.03, by, name, va="center", fontsize=9, color=col, weight="bold")
    ax_side.add_patch(patches.Rectangle((TABLE_X0, TABLE_TOP), BOX_D, BOX_H, fc=col, alpha=0.25, ec=col))
ax_side.add_patch(patches.Rectangle((TABLE_X0 - 0.005, MOUTH_Z0), 0.012, MOUTH_H, fc="k"))
ax_side.text(TABLE_X0 + 0.02, TABLE_TOP + BOX_H + 0.04, "RED / BLUE\n(same x)", fontsize=8, color="#444")
# moving parts
props_top = [patches.Circle((0, 0), PROP_R, fc="#55555522", ec="#333", lw=1.0) for _ in range(4)]
for p in props_top:
    ax_top.add_patch(p)
body_top, = ax_top.plot([], [], "k-", lw=2.5)
arm_top, = ax_top.plot([], [], "-", color="#444", lw=1.5)
claw_top, = ax_top.plot([], [], "o", color="#222", ms=5)
body_side, = ax_side.plot([], [], "k-", lw=3)
props_side, = ax_side.plot([], [], "-", color="#666", lw=1.2)
arm_side, = ax_side.plot([], [], "-", color="#444", lw=1.5)
claw_side, = ax_side.plot([], [], "o", color="#222", ms=5)
env_top = [ax_top.plot([], [], "-", color=c, lw=4, solid_capstyle="butt")[0] for c in LETTER_RGB]
env_side = [ax_side.plot([], [], "-", color=c, lw=4, solid_capstyle="butt")[0] for c in LETTER_RGB]
trail_top = [ax_top.plot([], [], ":", color=c, lw=1)[0] for c in LETTER_RGB]
path_top, = ax_top.plot([], [], "-", color="#88a", lw=0.8, alpha=0.6)
path_side, = ax_side.plot([], [], "-", color="#88a", lw=0.8, alpha=0.6)
title = fig.suptitle("", fontsize=11, x=0.02, ha="left")
legend = [patches.Patch(color=c, label=f"letter {k + 1}") for k, c in enumerate(LETTER_RGB)]
ax_top.legend(handles=legend, loc="lower left", fontsize=8)

frames = np.arange(t0, t1, a.speed / a.fps)
hist = {"x": [], "y": [], "z": []}
env_hist = [([], []) for _ in range(3)]


def draw(t):
    s = at(o, t)
    p = s[1:4]
    R = rot(*s[7:11])
    motors = [p + R @ np.array([sx * ROTOR, sy * ROTOR, 0.06]) for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    for c, m in zip(props_top, motors):
        c.center = (m[0], m[1])
    body_top.set_data([motors[0][0], motors[2][0], np.nan, motors[1][0], motors[3][0]],
                      [motors[0][1], motors[2][1], np.nan, motors[1][1], motors[3][1]])
    cl = at(claw, t)[1:4]
    arm_top.set_data([p[0], cl[0]], [p[1], cl[1]])
    claw_top.set_data([cl[0]], [cl[1]])
    xs = [m[0] for m in motors]
    zs = [m[2] for m in motors]
    i0, i1 = int(np.argmin(xs)), int(np.argmax(xs))
    body_side.set_data([xs[i0], xs[i1]], [zs[i0], zs[i1]])
    props_side.set_data([xs[i0] - PROP_R, xs[i0] + PROP_R, np.nan, xs[i1] - PROP_R, xs[i1] + PROP_R],
                        [zs[i0] + 0.01] * 2 + [np.nan] + [zs[i1] + 0.01] * 2)
    arm_side.set_data([p[0], cl[0]], [p[2], cl[2]])
    claw_side.set_data([cl[0]], [cl[2]])
    for k in range(3):
        e = at(env[k], t)
        c = e[1:4]
        u = rot(*e[4:8])[:, 0] * ENV_L / 2
        env_top[k].set_data([c[0] - u[0], c[0] + u[0]], [c[1] - u[1], c[1] + u[1]])
        env_side[k].set_data([c[0] - u[0], c[0] + u[0]], [c[2] - u[2], c[2] + u[2]])
        env_hist[k][0].append(c[0])
        env_hist[k][1].append(c[1])
        trail_top[k].set_data(env_hist[k][0][-300:], env_hist[k][1][-300:])
    hist["x"].append(p[0]); hist["y"].append(p[1]); hist["z"].append(p[2])
    path_top.set_data(hist["x"][-400:], hist["y"][-400:])
    path_side.set_data(hist["x"][-400:], hist["z"][-400:])
    st = [x for tt, x in status if tt <= t]
    line = st[-1] if st else ""
    line = line if len(line) < 150 else line[:147] + "..."
    title.set_text(f"{a.run.split('/')[-1]}   t = {t - t_direct:6.1f} s after DIRECT   (x{a.speed:.0f} speed)\n{line}")
    return []


writer = animation.FFMpegWriter(fps=a.fps, bitrate=2400)
with writer.saving(fig, out, dpi=100):
    for k, t in enumerate(frames):
        draw(t)
        writer.grab_frame()
print(f"wrote {out}: {len(frames)} frames, {len(frames) / a.fps:.0f} s of video for {t1 - t0:.0f} s of flight")
