#!/usr/bin/env python3
"""Print `ok` if the vehicle is fit to start a mission, else why not.

Fit means UPRIGHT (R[2][2] > 0.7), LOW (z < 2 m), STOPPED (|v| < 1 m/s) and near the origin
(|x|,|y| < 200 m) -- i.e. sitting on the pad, not inverted or parked downrange from the last
flight.  Run with the SYSTEM python after sourcing rosenv.sh.
"""
import math, re, subprocess, sys

try:
    out = subprocess.run(["ros2", "topic", "echo", "/uav_0/mocap", "--once"],
                         capture_output=True, timeout=8).stdout.decode()
except Exception as exc:
    print(f"nostate ({exc})"); sys.exit(1)

q = re.findall(r"orientation:\s*\n\s*x:\s*([-0-9.e]+)\s*\n\s*y:\s*([-0-9.e]+)\s*\n"
               r"\s*z:\s*([-0-9.e]+)\s*\n\s*w:\s*([-0-9.e]+)", out)
p = re.findall(r"position:\s*\n\s*x:\s*([-0-9.e]+)\s*\n\s*y:\s*([-0-9.e]+)\s*\n"
               r"\s*z:\s*([-0-9.e]+)", out)
v = re.findall(r"linear:\s*\n\s*x:\s*([-0-9.e]+)\s*\n\s*y:\s*([-0-9.e]+)\s*\n"
               r"\s*z:\s*([-0-9.e]+)", out)
if not (q and p):
    print("nostate"); sys.exit(1)

x, y, z, w = (float(t) for t in q[0])
px, py, pz = (float(t) for t in p[0])
speed = math.sqrt(sum(float(t) ** 2 for t in v[0])) if v else 0.0
up = 1.0 - 2.0 * (x * x + y * y)          # R[2][2]

bad = []
if up <= 0.7:            bad.append(f"attitude(R22={up:+.2f})")
if pz >= 2.0:            bad.append(f"airborne(z={pz:.1f})")
if speed >= 1.0:         bad.append(f"moving(v={speed:.1f})")
if abs(px) > 200 or abs(py) > 200:
    bad.append(f"downrange({px:.0f},{py:.0f})")
print("ok" if not bad else "bad:" + ",".join(bad))
sys.exit(0 if not bad else 1)
