#!/usr/bin/env python3
"""One TSV verdict line per trial, computed from the ROSBAG.

Deliberately reads the BAG rather than the flight driver's own summary. This project has been
flattered by its own harness four separate times -- a dive, a post-dropout glide, a frozen
simulator and the zero-crossings of a phugoid all produced headline numbers that were true of
the samples and false of the flight. The bag carries who was actually flying.

Usage: night_metrics.py <bagdir|-> <trial> <params>
"""
import math
import os
import re
import sys
import glob
import sqlite3

TRIAL = sys.argv[2] if len(sys.argv) > 2 else "?"
PARAMS = sys.argv[3] if len(sys.argv) > 3 else ""
LAND_ERR = sys.argv[4] if len(sys.argv) > 4 else ""
globals().setdefault('_le', None)   # metres from the commanded point


def out(verdict, **kw):
    cols = [TRIAL, PARAMS, verdict, kw.get("wb_s", ""),
            kw.get("peakV", ""), kw.get("maxtilt", ""), kw.get("maxroll", ""),
            kw.get("maxyaw", ""), kw.get("zmin", ""), kw.get("zmax", ""),
            kw.get("fails", ""), LAND_ERR, kw.get("bag", "-")]
    print("\t".join(str(c) for c in cols))
    sys.exit(0)


bagdir = sys.argv[1] if len(sys.argv) > 1 else "-"
if bagdir == "-":
    out("NO_BAG")

try:
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message
    db = sorted(glob.glob(bagdir.rstrip("/") + "/*.db3"))[0]
    con = sqlite3.connect(db)
    tp = {r[0]: (r[1], r[2]) for r in con.execute("select id,name,type from topics")}
    ids = [k for k, v in tp.items() if v[0] == "/rosout"]
    if not ids:
        out("NO_ROSOUT", bag=bagdir)
    T = get_message(tp[ids[0]][1])
    msgs = [deserialize_message(d, T) for (d,) in
            con.execute(f"select data from messages where topic_id={ids[0]} order by timestamp")]
except Exception as e:
    out("BAG_UNREADABLE:%s" % type(e).__name__, bag=bagdir)

# ★ SCORE ONLY THE ENGAGED FLIGHT.
# The bag starts before takeoff, so the whole-bag minimum altitude is the vehicle sitting on the
# ground at z=0.1 -- which scored a 74 s TRUE FIXED-WING cruise as CRASHED. Everything before
# "FULL-NMPC ENGAGED" is the safe controller climbing, and everything the NMPC did afterwards is
# what the trial is actually testing.
eng = next((i for i, m in enumerate(msgs) if "FULL-NMPC ENGAGED" in m.msg), None)
full = [m.msg for m in msgs[eng:] if m.msg.startswith("FULL ")] if eng is not None else []

# ★ SCORE FROM MOCAP, NOT FROM LOG STRINGS.
# /rosout is a lossy way to measure a flight: trial bt_aft25 recorded only 70 rosout messages
# (others had 292-1202) and scored NEVER_ENGAGED, despite flying a complete mission with a 68.5 s
# wing-borne cruise and landing on its target. Mocap is the simulator's own truth and is always
# present -- 2171 samples in that same bag. Tilt comes from the servo command, which is what the
# aircraft actually flew.

def classify(Z, VV, TILT_M, RPY, peakV, maxtilt, maxroll, maxyaw, zmin, zmax):
    """Verdict, distinguishing a departure the aircraft SURVIVED from one it did not.

    ★ THE SIMULATOR RESURRECTS THE VEHICLE, so anything after a ground impact is fiction.
    corridor_repeat rolled to -104 deg in the back-transition and struck the ground at
    z=0.6 m doing 14.83 m/s at roll -53. Isaac does not destroy it, so the safe controller then
    picked the wreck up, flew it back to 19.8 m and landed it on the target -- and the flight
    driver duly reported "TRUE FIXED-WING (sustained, level, controlled)" and
    "LANDING ERROR 0.00 m". A verdict that looks only at the END of a flight will call a crash a
    success on this rig, every time.
    So: find the FIRST loss of control, then ask what happened next. Reaching the ground while
    still out of control is a crash no matter how well the aircraft flies afterwards.
    """
    # ★ STOP ANALYSING AT THE FIRST TOUCHDOWN.
    # Once the aircraft is down and stationary, the controller keeps commanding its target and
    # the vehicle gets DRAGGED along the ground -- bt_flare20 landed cleanly at t=367.8
    # (z=0.10, V=0.00, roll 0) and then read 11.43 m/s at z=0.10 sixty seconds later. Judging
    # "did it arrive fast" over the whole record condemns a perfect landing on an artefact that
    # happened long after the flight ended.
    end = len(Z)
    for i in range(len(Z)):
        if Z[i] < 0.5 and VV[i] < 1.0:
            end = i + 1
            break
    Z, VV, RPY = Z[:end], VV[:end], RPY[:end]
    zmin = min(Z)

    # Arriving at the ground FAST is a crash whatever the attitude: bt_flare20's sibling runs
    # struck at 11-15 m/s with modest roll, and a rule that inspected only the MINIMUM-altitude
    # sample saw the settled wreck (slow, level) and called it a landing.
    if zmax > 10.0 and any(Z[i] < 2.0 and VV[i] > 5.0 for i in range(len(Z))):
        return "CRASHED"
    departed_at = [i for i, (r, _, _) in enumerate(RPY) if abs(r) > 60]
    if departed_at:
        i0 = departed_at[0]
        # did it get back to controlled flight after the excursion?
        recovered = any(abs(RPY[j][0]) < 20 and Z[j] > 5.0 for j in range(i0, len(Z)))
        # ...and was it still out of control when it reached the ground?
        low_while_bad = any(Z[j] < 2.0 and abs(RPY[j][0]) > 45 for j in range(i0, len(Z)))
        if low_while_bad or not recovered:
            return "CRASHED"
        return "DEPARTED_OK"          # survived a departure -- NOT a pass
    if maxyaw > 90:
        return "DEPARTED_OK" if zmin >= 2.0 else "CRASHED"
    if zmax > 10.0 and zmin < 2.0:
        i_low = min(range(len(Z)), key=lambda i: Z[i])
        return "LANDED" if VV[i_low] < 5.0 else "CRASHED"
    if maxtilt >= 88 and peakV >= 14.13:
        return "WINGBORNE"
    if maxtilt >= 60:
        return "PARTIAL_TILT"
    return "NO_TRANSITION"

def g(s, key):
    """Pull a numeric field out of a status line; tilt/pitch are printed 'applied/reference'."""
    i = s.find(key + "=")
    if i < 0:
        return None
    tok = s[i + len(key) + 1:].split()[0].split("/")[0].rstrip("|")
    try:
        return float(tok)
    except ValueError:
        return None


def read(name):
    """Return (timestamps, messages). Topics record at different rates -- mocap ~30 Hz, servos
    ~200 Hz -- so anything that pairs them by INDEX is wrong: doing that collapsed a 70 s
    wing-borne cruise to 0.0 s."""
    ids = [k for k, v in tp.items() if v[0] == name]
    if not ids:
        return [], []
    TT = get_message(tp[ids[0]][1])
    rows = list(con.execute(f"select timestamp,data from messages where topic_id={ids[0]} order by timestamp"))
    return [r[0] for r in rows], [deserialize_message(r[1], TT) for r in rows]

t_moc, moc = read("/uav_0/mocap")

# ★ FALL BACK TO THE FLIGHT LOG WHEN THE BAG IS TRUNCATED.
# Recording is the least reliable part of this rig: two recorders writing /tmp at high rate makes
# sqlite report "database is locked" and the recorder dies mid-flight. bt_aft25 captured 18 s of a
# 300 s mission and bt_lead_loose 29 s -- both scored as failures on flights that actually reached
# 90 deg tilt and landed on target. The driver samples the SAME mocap topic and prints every
# sample, so its log is a complete, independent record. Prefer the bag; use the log when the bag
# is clearly short.
FLIGHT_LOG = os.path.join(os.path.dirname(bagdir.rstrip("/")), "..",
                          os.path.basename(bagdir.rstrip("/")).rsplit("_", 1)[0] + ".flight")
def from_flight_log(path):
    rows = []
    rx = re.compile(r"t=\s*([\d.]+) p=\(\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)"
                    r"\s*V=\s*([-\d.]+) tilt=\s*([-\d.]+).*?rpy=\(\s*([-+\d]+),\s*([-+\d]+),\s*([-+\d]+)\)")
    try:
        for line in open(path):
            m = rx.search(line)
            if m:
                t, x, y, zz, vv, tl, r_, p_, yw = (float(g_) for g_ in m.groups())
                rows.append((t, x, y, zz, vv, tl, r_, p_, yw))
    except OSError:
        return None
    return rows or None

_span = (t_moc[-1] - t_moc[0]) / 1e9 if len(t_moc) > 1 else 0.0
_log = from_flight_log(FLIGHT_LOG) if _span < 120.0 else None
if _log:
    T_ = [r[0] for r in _log]
    Z = [r[3] for r in _log]; VV = [r[4] for r in _log]
    TILT_M = [r[5] for r in _log]
    RPY = [(r[6], r[7], r[8]) for r in _log]
    XY = [(r[1], r[2]) for r in _log]
    tk = next((i for i, zz in enumerate(Z) if zz > 5.0), None)
    if tk is None:
        out("NEVER_FLEW", bag=bagdir + " (log)")
    Z, VV, TILT_M, RPY, XY, T_ = (Z[tk:], VV[tk:], TILT_M[tk:], RPY[tk:], XY[tk:], T_[tk:])
    peakV = max(VV); maxtilt = max(TILT_M)
    maxroll = max(abs(r) for r, _, _ in RPY)
    yaw0 = RPY[0][2]
    maxyaw = max(abs((yw - yaw0 + 180) % 360 - 180) for _, _, yw in RPY)
    zmin, zmax = min(Z), max(Z)
    dt = (T_[-1] - T_[0]) / max(1, len(T_) - 1)
    run = best = 0
    for vv, tl in zip(VV, TILT_M):
        if vv >= 14.13 and tl >= 88:
            run += 1; best = max(best, run)
        else:
            run = 0
    wb_s = round(best * dt, 1)
    v = classify(Z, VV, TILT_M, RPY, peakV, maxtilt, maxroll, maxyaw, zmin, zmax)
    if v in ("CRASHED", "DEPARTED_OK"):
        LAND_ERR = ""
    out(v, wb_s=wb_s, peakV=round(peakV, 1), maxtilt=round(maxtilt),
        maxroll=round(maxroll), maxyaw=round(maxyaw), zmin=round(zmin, 1),
        zmax=round(zmax, 1), fails="log", bag=bagdir + " (log)")
t_srv, srv = read("/uav_0/fmu/in/actuator_servos")
if not moc:
    out("NO_MOCAP", bag=bagdir)

def rpy(q):
    w, x, y, z_ = q.w, q.x, q.y, q.z
    r = math.degrees(math.atan2(2*(w*x + y*z_), 1 - 2*(x*x + y*y)))
    p_ = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z_*x)))))
    yw = math.degrees(math.atan2(2*(w*z_ + x*y), 1 - 2*(y*y + z_*z_)))
    return r, p_, yw

Z = [m.pose.position.z for m in moc]
XY = [(m.pose.position.x, m.pose.position.y) for m in moc]
VV = [math.sqrt(m.twist.linear.x**2 + m.twist.linear.y**2 + m.twist.linear.z**2) for m in moc]
RPY = [rpy(m.pose.orientation) for m in moc]
# tilt: servo channel 0 is tilt_left, commanded as a fraction of tilt_max (90 deg)
import bisect
_tilt_src = [abs(sv.control[0]) * 90.0 for sv in srv] if srv else []
def tilt_at_time(t):
    if not _tilt_src:
        return 0.0
    j = min(bisect.bisect_left(t_srv, t), len(_tilt_src) - 1)
    return _tilt_src[j]
TILT_M = [tilt_at_time(t) for t in t_moc]          # tilt sampled on the MOCAP timeline
TILT = _tilt_src or [0.0]

# analyse from TAKEOFF onward: before that the aircraft is on the ground and every altitude
# statistic is about the ground, which is what scored a 74 s cruise as CRASHED earlier tonight.
tk = next((i for i, zz in enumerate(Z) if zz > 5.0), None)
if tk is None:
    out("NEVER_FLEW", bag=bagdir)
Z, VV, RPY, XY, TILT_M = Z[tk:], VV[tk:], RPY[tk:], XY[tk:], TILT_M[tk:]

peakV = max(VV)
maxtilt = max(TILT_M) if TILT_M else 0.0
maxroll = max(abs(r) for r, _, _ in RPY)
yaw0 = RPY[0][2]
maxyaw = max(abs((yw - yaw0 + 180) % 360 - 180) for _, _, yw in RPY)
zmin, zmax = min(Z), max(Z)
# solve failures: parse the ok= field. Testing endswith(" 0") silently counted ZERO for
# every trial -- the status line ends with the surface commands, not with ok.
fails = sum(1 for s in full if g(s, "ok") == 0)

# wing-borne seconds: mocap is ~30 Hz, so count samples above stall with the rotors tilted over
# mocap sample rate, measured from the bag rather than assumed
span = (t_moc[-1] - t_moc[0]) / 1e9 if len(t_moc) > 1 else 1.0
hz = max(1.0, (len(t_moc) - 1) / max(span, 1e-6))
run = best = 0
for vv, tl in zip(VV, TILT_M):
    if vv >= 14.13 and tl >= 88:
        run += 1; best = max(best, run)
    else:
        run = 0
wb_s = round(best / hz, 1)

# ORDER MATTERS. A departure is unambiguous, so decide it BEFORE any altitude reasoning.
v = classify(Z, VV, TILT_M, RPY, peakV, maxtilt, maxroll, maxyaw, zmin, zmax)

# ★ A LANDING ERROR MEANS NOTHING NEXT TO A CRASH.
# bt_flare25 and bt_lead_tight both departed and hit the ground, yet both carried "land_err 0.00"
# -- the aircraft came to rest near the target and the landing phase then commanded a point it
# was already lying on. Reading that column beside a CRASHED verdict is exactly how tonight's
# first "0.00 m landing" nearly got reported as success. Blank it unless the flight survived.
if v in ("CRASHED", "DEPARTED_OK"):
    LAND_ERR = ""

out(v, wb_s=wb_s, peakV=round(peakV, 1), maxtilt=round(maxtilt), maxroll=round(maxroll),
    maxyaw=round(maxyaw), zmin=round(zmin, 1), zmax=round(zmax, 1),
    fails=f"{fails}/{len(full)}", bag=bagdir)
