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

★ THE REAL-TIME FACTOR IS IN THE ESTIMATE. The observer integrates the model's SIM-time forces
against a velocity it samples on the WALL clock (mocap stamps are wall time; Isaac runs at
0.86-0.93x), so in HIL it reports F_bob - (1 - RTF) * m * a in addition to the bob: -1.33 N of
steady "lateral force" in the R 100 orbit of flight 8, +0.9 N fore-aft through the decel -- the
two "model gaps" of 2026-09-14 morning, neither of which exists (scripts/ana_force_model.py: the
model closes to 0.05 N rms on the plant's own clock). RTF is measured from the PX4 stamps in the
bag and the plant's acceleration comes from /uav_0/state/accel, so the scorer judges the estimate
against BOTH truths: the bob alone (what it means) and bob + RTF term (what it must read in HIL).
Hardware has RTF 1 and only the first column.
"""
import glob, os, re, sqlite3, sys, datetime as dt
import numpy as np
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from fsc_autopilot_ros2_msgs.msg import Mocap
from geometry_msgs.msg import AccelStamped
from px4_msgs.msg import VehicleAttitude
sys.path.insert(0, os.path.expanduser("~/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator"))

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
# the predictor's own reading (2026-09-14 pm, "| pred bob=(..)mm f1=.. F=(x,y) M=(..) bF=(x,y) bM=(..)")
ppat = re.compile(r"pred bob=\(([+-]?[\d.]+),([+-]?[\d.]+)\)mm.*? F=\(([+-][\d.]+),([+-][\d.]+)\) M=.*? bF=\(([+-][\d.]+),([+-][\d.]+)\)")
pred = []
for t, m in rows("cascade/status", String):
    g = pat.search(m.data)
    if g:
        est.append((t, *[float(v) for v in g.groups()]))
        gp = ppat.search(m.data)
        pred.append((float(gp.group(3)), float(gp.group(4)), float(gp.group(5)), float(gp.group(6)), float(gp.group(1)), float(gp.group(2))) if gp else (np.nan,) * 6)
est = np.array(est); pred = np.array(pred)
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
Q = np.array([[m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z] for _, m in mc])

# --- the real-time-factor term the observer must also read in HIL ---------------------------
att = rows("/vehicle_attitude", VehicleAttitude)
acc = rows("/state/accel", AccelStamped)
RTF = float("nan"); F_rtf = None
if att and acc:
    import nmpc_so3_transition_node as so3
    A_ = np.array([(t, m.timestamp / 1e6) for t, m in att]); RTF = float(np.polyfit(A_[:, 0], A_[:, 1], 1)[0])
    # the factor drifts by phase (0.861 in flight 8's orbit, 0.870 in its decel): use a local slope,
    # sim seconds per wall second over a +-5 s window
    _tw = np.arange(A_[0, 0], A_[-1, 0], 1.0); _ts = np.interp(_tw, A_[:, 0], A_[:, 1])
    _rl = np.array([np.polyfit(_tw[max(0, i - 5):i + 6], _ts[max(0, i - 5):i + 6], 1)[0] if i + 6 <= len(_tw) and i >= 5 else RTF for i in range(len(_tw))])
    Ta = np.array([t for t, _ in acc]); Aa = np.array([[m.accel.linear.x, m.accel.linear.y, m.accel.linear.z] for _, m in acc])

    def _R(q):
        w, x, y, z = q
        return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

    def F_rtf(t):
        """-(1 - RTF) m a_sim in the body frame at bag time t: what a wall-clock observer of a
        sim-time plant reads as force (see the docstring)."""
        i = min(np.searchsorted(T, t), len(T) - 1)
        a = np.array([np.interp(t, Ta, Aa[:, j]) for j in range(3)])
        return -(1.0 - float(np.interp(t, _tw, _rl))) * so3.M * (_R(Q[i]).T @ a)
    print(f"real-time factor {RTF:.4f} whole bag, {_rl.min():.3f}-{_rl.max():.3f} locally (PX4 stamps vs bag clock): the observer also reads -(1-RTF) m a = {(1 - RTF) * so3.M:.2f} N per m/s^2 of plant acceleration")
else:
    print("no /vehicle_attitude or /state/accel in the bag: RTF term not available, truth = the bob alone")
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
if F_rtf is not None:
    Frt = np.array([F_rtf(t) for t in tt_s]); Fx_r, Fy_r = Fx_t + Frt[:, 0], Fy_t + Frt[:, 1]
else:
    Fx_r, Fy_r = Fx_t, Fy_t
HAVE_PRED = pred.size and np.isfinite(pred[:, 0]).any()
if HAVE_PRED:
    Px_e = np.interp(tt_s, te, pred[:, 0]); Py_e = np.interp(tt_s, te, pred[:, 1])


def stats(w, tag):
    if w.sum() < 10:
        return
    out = [f"  {tag:22s}"]
    for lab, e, t, tr in (("x", Fx_e, Fx_t, Fx_r), ("y", Fy_e, Fy_t, Fy_r)):
        err = e[w] - t[w]; err_r = e[w] - tr[w]
        c = np.corrcoef(e[w], t[w])[0, 1] if t[w].std() > 1e-3 else float("nan")
        cr = np.corrcoef(e[w], tr[w])[0, 1] if tr[w].std() > 1e-3 else float("nan")
        out.append(f"{lab}: bob rms {np.sqrt((t[w] ** 2).mean()):4.2f} | est rms {np.sqrt((e[w] ** 2).mean()):4.2f} err {np.sqrt((err ** 2).mean()):4.2f} corr {c:+.2f}"
                   f" | vs bob+RTF: err {np.sqrt((err_r ** 2).mean()):4.2f} corr {cr:+.2f}")
        if HAVE_PRED:
            pe = (Px_e if lab == "x" else Py_e)
            cp = np.corrcoef(pe[w], t[w])[0, 1] if t[w].std() > 1e-3 else float("nan")
            out.append(f"PREDICTOR vs bob: err {np.sqrt(((pe[w] - t[w]) ** 2).mean()):4.2f} corr {cp:+.2f}")
    print(" | ".join(out))


print("phase                   estimate vs the plant's pendulum reaction, and vs pendulum + the RTF term [N, body]")
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
