#!/usr/bin/env python3
"""The STESO cascade's slosh-force estimate against the simulator's own bob reaction.

    source scripts/rosenv.sh
    ~/envs/isaac_env/bin/python3 scripts/ana_slosh_est.py ~/flight_bags/slosh/<bag_dir> ~/flight_bags/slosh/pane_2-isaac_<HHMMSS>.log

The estimate is what the node published on /uav_0/cascade/status ("... | slosh F=(x,y,z) N
M=(x,y,z) Mres=(x,y,z)", 5 Hz, body frame). The truth is the plant's pendulum reaction printed
by VTOLSlosh with PEGASUS_SLOSH_DEBUG=N ("[SLOSH] ... F=(x,y) N", ~10 Hz at N=24). The Isaac
log carries a wall-clock second and a millisecond-since-start counter; the two are aligned by
fitting second = a + ms/1000 over the whole log (sub-second, good to the log's ~30 ms jitter).
Prints per-phase rms / correlation of estimate vs truth for the lateral (x, y) force, plus the
attributed tank moment and the residual (non-slosh) moment. 2026-09-14.
"""
import glob, os, re, sqlite3, sys, datetime as dt
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from fsc_autopilot_ros2_msgs.msg import Mocap

bagdir, isaac_log = sys.argv[1], sys.argv[2]
bag = glob.glob(os.path.join(bagdir, "*.db3"))[0]
cur = sqlite3.connect(bag).cursor()
topics = {tid: name for tid, name, typ in cur.execute("select id, name, type from topics")}
t0ns = cur.execute("select min(timestamp) from messages").fetchone()[0]
wall0 = t0ns / 1e9


def rows(sub, cls):
    tid = [k for k, v in topics.items() if v.endswith(sub)]
    return [((ts - t0ns) / 1e9, deserialize_message(d, cls)) for ts, d in
            cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[0],))] if tid else []


# --- the estimate, from the status feed -----------------------------------------------------
est = []
pat = re.compile(r"slosh F=\(([+-][\d.]+),([+-][\d.]+),([+-][\d.]+)\) N M=\(([+-][\d.]+),([+-][\d.]+),([+-][\d.]+)\) Mres=\(([+-][\d.]+),([+-][\d.]+),([+-][\d.]+)\)")
for t, m in rows("cascade/status", String):
    g = pat.search(m.data)
    if g:
        est.append((t, *[float(v) for v in g.groups()]))
est = np.array(est)
if len(est) == 0:
    sys.exit("no slosh estimate on the status feed (was the STESO cascade flying, STESO_LF > 0?)")
te = est[:, 0]
print(f"estimate: {len(est)} status lines from t={te[0]:.0f} to {te[-1]:.0f} s")

# --- the truth, from the Isaac pane log -----------------------------------------------------
tru = []
lpat = re.compile(r"(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)Z \[([\d,]+)ms\].*\[SLOSH\] f_b=\(([+-][\d.]+),([+-][\d.]+),([+-][\d.]+)\) bob=\(([+-][\d.]+),([+-][\d.]+)\) mm F=\(([+-][\d.]+),([+-][\d.]+)\)")
for line in open(isaac_log, errors="ignore"):
    g = lpat.search(line)
    if g:
        w = dt.datetime.strptime(g.group(1), "%Y-%m-%dT%H:%M:%S").replace(tzinfo=dt.timezone.utc).timestamp()
        ms = int(g.group(2).replace(",", ""))
        tru.append((w, ms / 1000.0, *[float(v) for v in g.groups()[2:]]))
tru = np.array(tru)
if len(tru) == 0:
    sys.exit("no [SLOSH] lines in the Isaac log (PEGASUS_SLOSH_DEBUG not forwarded?)")
# align: wall = a + ms; a = median(wall_second - ms) (+0.5 s: a second stamp floors the time)
a = np.median(tru[:, 0] - tru[:, 1]) + 0.5
tt = a + tru[:, 1] - wall0
Fx_t, Fy_t = tru[:, 7], tru[:, 8]
print(f"truth:    {len(tru)} [SLOSH] lines, aligned to the bag clock (median offset), bob |x| max {np.abs(tru[:, 5]).max():.0f} mm, |F| max {np.hypot(Fx_t, Fy_t).max():.2f} N")

# --- phases from the mocap (V) and the commands ----------------------------------------------
mc = rows("/mocap", Mocap)
T = np.array([t for t, _ in mc]); V = np.array([np.hypot(m.twist.linear.x, m.twist.linear.y) for _, m in mc])
cmds = [(t, m.data) for t, m in rows("cascade/cmd", String)]
t_wp = next((t for t, c in cmds if c.startswith("wp")), te[0])
t_rth = next((t for t, c in cmds if c.strip() == "rth"), te[-1])
st = [(t, m.data) for t, m in rows("cascade/status", String)]
t_dec = next((t for t, s in st if "decel=1" in s), t_rth + 40)
i_stop = np.where((T > t_dec) & (V < 1.0))[0]
t_stop = T[i_stop[0]] if len(i_stop) else t_dec + 20

# interpolate the estimate onto the truth's times (the estimate is the slower stream)
sel = (tt > te[0]) & (tt < te[-1])
Fx_e = np.interp(tt[sel], te, est[:, 1]); Fy_e = np.interp(tt[sel], te, est[:, 2]); tt_s = tt[sel]
Fx_t, Fy_t = Fx_t[sel], Fy_t[sel]


def stats(w, tag):
    if w.sum() < 10:
        return
    out = [f"  {tag:26s}"]
    for lab, e, t in (("x", Fx_e, Fx_t), ("y", Fy_e, Fy_t)):
        err = e[w] - t[w]
        c = np.corrcoef(e[w], t[w])[0, 1] if t[w].std() > 1e-3 else float("nan")
        out.append(f"{lab}: true rms {np.sqrt((t[w] ** 2).mean()):4.2f} max {np.abs(t[w]).max():4.2f} | est rms {np.sqrt((e[w] ** 2).mean()):4.2f} err {np.sqrt((err ** 2).mean()):4.2f} corr {c:+.2f}")
    print(" | ".join(out))


print("phase                       estimate vs the plant's pendulum reaction [N, body]")
stats((tt_s > t_wp + 5) & (tt_s < t_wp + 70), "legs + fly-by turns")
stats((tt_s > t_wp + 80) & (tt_s < t_rth), "orbit")
stats((tt_s > t_dec) & (tt_s < t_stop), "back-transition")
stats((tt_s > t_stop) & (tt_s < t_stop + 15), "hover after the stop")
# the attributed moment and the residual, per phase
Mz = est[:, 6]; Mry = est[:, 8]; Mrz = est[:, 9]
for a_, b_, tag in ((t_wp + 5, t_wp + 70, "legs + fly-by turns"), (t_wp + 80, t_rth, "orbit"), (t_dec, t_stop, "back-transition")):
    w = (te > a_) & (te < b_)
    if w.sum():
        print(f"  {tag:26s} M_slosh_z rms {np.sqrt((Mz[w] ** 2).mean()):4.2f} max {np.abs(Mz[w]).max():4.2f} N.m | residual M_y mean {Mry[w].mean():+.2f} sd {Mry[w].std():.2f}  M_z mean {Mrz[w].mean():+.2f} sd {Mrz[w].std():.2f}")
