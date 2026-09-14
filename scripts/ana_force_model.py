#!/usr/bin/env python3
"""The controller's FORCE model against the plant, from a bag: m*a - m*g - F_rotors - F_aero - F_bob.

    source scripts/rosenv.sh
    VTOL_TANK_KG=0.502 ~/envs/isaac_env/bin/python3 scripts/ana_force_model.py ~/flight_bags/slosh/<bag_dir> \
        [~/flight_bags/slosh/pane_2-isaac_<HHMMSS>.log] [--wall] [--transport 0.03]

What it does. The bag's commands (actuator_motors / actuator_servos) are pushed through the
plant's own actuator dynamics -- ~30 ms of transport, the 0.12 s rotor spin lag, the 0.03 s /
400 deg/s surface servo, the 80 deg/s tilt slew -- and the controller's own models turn them
into a body-frame force: the four thrusts through the momentum propeller (`so3` constants) on
their tilted axes, and `so3.aero_body` on the mocap state. That is subtracted from the plant's
inertial force m*a - m*g, with the pendulum's reaction (the Isaac pane log's [SLOSH] lines,
when given) taken out too. What is left is what the plant does that the model does not, per
axis and per phase, and a regression of the lateral residual on the turn variables.

★ USE THE PLANT'S OWN ACCELERATION (/uav_0/state/accel, sim time, map frame -- the default), NOT
a derivative of the mocap velocity. The bag's clock is WALL time and Isaac runs at 0.86-0.93x, so
dv/dt on the bag clock is RTF x the plant's acceleration, and the "residual" is then
-(1 - RTF) * m * a: -1.33 N of steady lateral force in the R 100 m orbit of flight 8 (2026-09-14),
+0.9 N fore-aft through the decel. That is exactly what the STESO force observer reports in HIL
(it integrates sim-time model forces against a wall-clock velocity), and it was read as two plant
model gaps for a day. `--wall` reproduces the artefact for the A/B; `RTF` from the PX4 stamps
is printed either way. With the sim's acceleration the model closes to 0.05 N rms in every axis
through the legs, turns, orbit and RTH capture of flight 8 -- there is no force gap to model.

Servo channel order is [tilt_L, tilt_R, aileron, elevator, rudder] (SERVO_ORDER); unpacking
the aero surfaces as (de, da, dr) instead of (da, de, dr) makes a 4 N "lift gap". Ask me how I know.
"""
import glob, os, re, sys, sqlite3, math, datetime as dt
import numpy as np
sys.path.insert(0, os.path.expanduser("~/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator"))
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
from geometry_msgs.msg import AccelStamped
from fsc_autopilot_ros2_msgs.msg import Mocap
from px4_msgs.msg import ActuatorServos, ActuatorMotors, VehicleAttitude
import nmpc_so3_transition_node as so3

AERO = so3._aero_numeric()
args = [a for a in sys.argv[1:] if not a.startswith("--")]
bagdir = args[0]; isaac_log = args[1] if len(args) > 1 else None
WALL = "--wall" in sys.argv
TRANSPORT = float(sys.argv[sys.argv.index("--transport") + 1]) if "--transport" in sys.argv else 0.03
bag = glob.glob(os.path.join(bagdir.rstrip("/"), "*.db3"))[0]
cur = sqlite3.connect(bag).cursor()
topics = {tid: name for tid, name, typ in cur.execute("select id, name, type from topics")}
t0ns = cur.execute("select min(timestamp) from messages").fetchone()[0]; wall0 = t0ns / 1e9


def rows(sub, cls):
    tid = [k for k, v in topics.items() if v.endswith(sub)]
    return [((ts - t0ns) / 1e9, deserialize_message(d, cls)) for ts, d in
            cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[0],))] if tid else []


mc = rows("/mocap", Mocap)
T = np.array([t for t, _ in mc])
Vw = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for _, m in mc])
Q = np.array([[m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z] for _, m in mc])
W = np.array([[m.twist.angular.x, m.twist.angular.y, m.twist.angular.z] for _, m in mc])
mot = rows("/actuator_motors", ActuatorMotors); Tm = np.array([t for t, _ in mot]); TH = np.array([m.control[:4] for _, m in mot])
srv = rows("/actuator_servos", ActuatorServos); Ts = np.array([t for t, _ in srv]); SV = np.array([m.control[:5] for _, m in srv])
st = [(t, m.data) for t, m in rows("cascade/status", String)]
cmds = [(t, m.data) for t, m in rows("cascade/cmd", String)]
t_wp = next((t for t, c in cmds if c.startswith("wp") or c.startswith("loiter")), None)
t_rth = next((t for t, c in cmds if c.strip() == "rth"), None)
t_dec = next((t for t, s in st if "decel=1" in s), None)
V2 = np.linalg.norm(Vw[:, :2], axis=1)
t_stop = None
if t_dec is not None:
    i_stop = np.where((T > t_dec) & (V2 < 1.0))[0]; t_stop = T[i_stop[0]] if len(i_stop) else None

# real-time factor: PX4's stamps are sim time, the bag's are wall time
att = rows("/vehicle_attitude", VehicleAttitude)
RTF = float("nan")
if att:
    A_ = np.array([(t, m.timestamp / 1e6) for t, m in att]); RTF = np.polyfit(A_[:, 0], A_[:, 1], 1)[0]
print(f"bag {os.path.basename(bagdir.rstrip('/'))}: M {so3.M:.4f} kg; wp {t_wp} rth {t_rth} decel {t_dec} stop {t_stop}; "
      f"RTF {RTF:.4f} sim-s/wall-s; transport {TRANSPORT * 1e3:.0f} ms; acceleration = {'mocap dv/dt on the WALL clock' if WALL else 'the plant (/state/accel)'}")


def Rm(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def hold(tsrc, X, tq):
    i = np.clip(np.searchsorted(tsrc, tq, side="right") - 1, 0, len(tsrc) - 1)
    return X[i]


# ---- the plant's actuators (vtol.py): transport, spin lag, servo lag + slew, tilt slew ----
DT = 1.0 / 240.0
tg = np.arange(T[0], T[-1], DT)
th_cmd = hold(Tm, TH, tg - TRANSPORT); sv_cmd = hold(Ts, SV, tg - TRANSPORT)
om = np.zeros((len(tg), 4)); om[0] = th_cmd[0]
surf = np.zeros((len(tg), 3)); tilt = np.zeros(len(tg))
a_sp = DT / 0.12; a_sv = DT / 0.03; slew = math.radians(400.0) * DT / math.radians(25.0); tslew = math.radians(80.0) * DT
for k in range(1, len(tg)):
    om[k] = om[k - 1] + a_sp * (th_cmd[k] - om[k - 1])
    s = surf[k - 1] + a_sv * (sv_cmd[k, 2:5] - surf[k - 1])
    surf[k] = surf[k - 1] + np.clip(s - surf[k - 1], -slew, slew)
    tc = float(np.clip(sv_cmd[k, 0], -25.0 / 90.0, 1.0)) * math.pi / 2
    tilt[k] = tilt[k - 1] + np.clip(tc - tilt[k - 1], -tslew, tslew)

n_max = 749.0 / (2 * math.pi); D = so3.PROP_D; J0 = so3.PROP_J0
ct0 = so3.MAX_THRUST / (so3.RHO * n_max ** 2 * D ** 4)


def thrust(omf, v_ax):
    n = omf * n_max
    return max(0.0, so3.RHO * D * D * (ct0 * (n * D) ** 2 - (ct0 / J0) * (n * D) * v_ax))


# ---- the plant's acceleration ----
if WALL:
    Aw = np.gradient(Vw, T, axis=0)
    ker = np.ones(9) / 9.0
    Aw = np.column_stack([np.convolve(Aw[:, j], ker, mode="same") for j in range(3)])
else:
    acc = rows("/state/accel", AccelStamped)
    if not acc:
        sys.exit("no /uav_0/state/accel in the bag; use --wall (and read the docstring)")
    Ta = np.array([t for t, _ in acc]); Aa = np.array([[m.accel.linear.x, m.accel.linear.y, m.accel.linear.z] for _, m in acc])
    Aw = np.column_stack([np.interp(T, Ta, Aa[:, j]) for j in range(3)])
g = np.array([0.0, 0.0, -so3.G])

# ---- the bob reaction from the Isaac log (body frame, at the tank) ----
tt = None
if isaac_log:
    tru = []
    lpat = re.compile(r"(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)Z \[([\d,]+)ms\].*\[SLOSH\] f_b=\(([+-][\d.]+),([+-][\d.]+),([+-][\d.]+)\) bob=\(([+-][\d.]+),([+-][\d.]+)\) mm F=\(([+-][\d.]+),([+-][\d.]+)\)")
    for line in open(isaac_log, errors="ignore"):
        gg = lpat.search(line)
        if gg:
            w = dt.datetime.strptime(gg.group(1), "%Y-%m-%dT%H:%M:%S").replace(tzinfo=dt.timezone.utc).timestamp()
            tru.append((w, int(gg.group(2).replace(",", "")) / 1000.0, float(gg.group(8)), float(gg.group(9))))
    tru = np.array(tru)
    if len(tru):
        a_off = np.median(tru[:, 0] - tru[:, 1]) + 0.5; tt = a_off + tru[:, 1] - wall0
        print(f"bob reaction: {len(tru)} [SLOSH] lines, |F| max {np.hypot(tru[:, 2], tru[:, 3]).max():.2f} N")

# ---- evaluate on a 40 Hz grid ----
out = []
for i in range(0, len(T), 3):
    t = T[i]; R = Rm(Q[i]); vb = R.T @ Vw[i]
    k = min(int((t - tg[0]) / DT), len(tg) - 1); b = tilt[k]
    vax_f = vb[0] * math.sin(b) + vb[2] * math.cos(b); vax_r = vb[2]
    Tf = thrust(om[k, 0], vax_f) + thrust(om[k, 1], vax_f); Tr = thrust(om[k, 2], vax_r) + thrust(om[k, 3], vax_r)
    F_rot = Tf * np.array([math.sin(b), 0, math.cos(b)]) + np.array([0, 0, Tr])
    da, de, dr = surf[k] * math.radians(25.0)          # servo order: aileron, elevator, rudder
    Fa = np.array(AERO(vb, W[i], de, da, dr)[0]).ravel()
    F_true_b = R.T @ (so3.M * Aw[i] - so3.M * g)
    Fs = np.array([np.interp(t, tt, tru[:, 2]), np.interp(t, tt, tru[:, 3])]) if tt is not None else np.zeros(2)
    spd = np.linalg.norm(vb); beta = math.asin(max(-1.0, min(1.0, vb[1] / max(spd, 1e-3))))
    roll = math.atan2(R[2, 1], R[2, 2])
    res = F_true_b - F_rot - Fa
    out.append((t, res[0] - Fs[0], res[1] - Fs[1], res[2], Fs[0], Fs[1], roll, beta, W[i, 0], W[i, 2], da, dr, spd, math.degrees(b)))
O = np.array(out)
t_, gx, gy, gz, sx, sy, roll, beta, p, r, da, dr, spd, tiltd = O.T


def rms(x):
    return math.sqrt(float((x ** 2).mean())) if len(x) else float("nan")


phases = []
if t_wp is not None and t_rth is not None:
    phases += [("legs + turns", (t_ > t_wp + 5) & (t_ < t_wp + 70)), ("orbit", (t_ > t_wp + 80) & (t_ < t_rth))]
    if t_dec is not None:
        phases.append(("rth capture", (t_ > t_rth) & (t_ < t_dec)))
elif t_dec is not None:
    phases.append(("cruise", (t_ < t_dec) & (spd > 14.5)))
if t_dec is not None and t_stop is not None:
    phases += [("back-transition", (t_ > t_dec) & (t_ < t_stop)), ("hover after stop", (t_ > t_stop) & (t_ < t_stop + 15))]
print("\nresidual = m a - m g - F_rotors - F_aero - F_bob  [N, body FLU]:")
for tag, w in phases:
    w = w & (spd > 0.5)
    if w.sum() < 10:
        continue
    print(f"  {tag:17s} x {gx[w].mean():+5.2f} ({rms(gx[w]):4.2f} rms) | y {gy[w].mean():+5.2f} ({rms(gy[w]):4.2f}) | z {gz[w].mean():+5.2f} ({rms(gz[w]):4.2f})"
          f"   bob y rms {rms(sy[w]):.2f}, tilt {tiltd[w].mean():+.1f} deg")
# ---- the MOMENT balance, same recipe: I w_dot + w x I w - M_rotors - M_aero - r_tank x F_bob ----
# w_dot from the plant's own rates (/state/twist, every physics step, clean to 0.03 rad/s^2),
# smoothed over 37 ms; the controller's inertia, rotor arms and reaction torques (so3, which folds
# VTOL_TANK_DX/DZ into the CG and inertia since 2026-09-14), the controller's aero moment. The
# residual is what the plant does in torque that the controller does not model. Before 2026-09-14
# 15:00 every bag carried a steady -0.45 N.m nose-up in cruise: the sim's aero centre defaulted to
# the BODY LINK's CoM, 18.6 mm ahead of the assembled CG (`--aero_arm 0.0186` reproduces that
# prediction, r_ac x F_aero, for those bags; vtol.py now applies the aero at the assembled CG and
# flight 9 reads +0.11). Before the same afternoon an offset tank (VTOL_TANK_DX) also moved the
# plant's CG 38 mm without telling the controller: +1.5 N.m in the hover (flight 8), 0.04 after.
from geometry_msgs.msg import TwistStamped
tw = rows("/state/twist", TwistStamped)
Mres = None
if tw and not WALL:
    Tw = np.array([t for t, _ in tw]); Wb = np.array([[m.twist.angular.x, m.twist.angular.y, m.twist.angular.z] for _, m in tw])
    Wd = np.gradient(Wb, Tw, axis=0) / (RTF if RTF == RTF else 1.0)     # bag clock -> plant seconds
    ker = np.ones(9) / 9.0
    Wd = np.column_stack([np.convolve(Wd[:, j], ker, mode="same") for j in range(3)])
    Imat = np.diag([so3.IXX, so3.IYY, so3.IZZ])
    # the sim's aero centre was the body link's CoM, (+18.6, 0, -7.7) mm from the assembled CG, until
    # 2026-09-14 15:00 (vtol.py now defaults to the assembled CG): pass --aero_arm 0.0186 for older bags
    _arm = float(sys.argv[sys.argv.index("--aero_arm") + 1]) if "--aero_arm" in sys.argv else 0.0
    r_ac = np.array([_arm, 0.0, -0.0077 if _arm else 0.0])
    tank_r = np.array([float(os.environ.get("VTOL_TANK_DX", "0") or 0.0), 0.0, float(os.environ.get("VTOL_TANK_DZ", "0") or 0.0)])
    rowsM = []
    for i in range(0, len(T), 3):
        t = T[i]; R = Rm(Q[i]); vb = R.T @ Vw[i]
        k = min(int((t - tg[0]) / DT), len(tg) - 1); b = tilt[k]
        vax_f = vb[0] * math.sin(b) + vb[2] * math.cos(b); vax_r = vb[2]
        Ti = [thrust(om[k, 0], vax_f), thrust(om[k, 1], vax_f), thrust(om[k, 2], vax_r), thrust(om[k, 3], vax_r)]
        M_rot = np.zeros(3)
        for j in range(4):
            bj = b * so3.TILTABLE[j]; d = np.array([math.sin(bj), 0.0, math.cos(bj)])
            M_rot += np.cross(so3.RPOS[j], Ti[j] * d) - so3.SPIN[j] * so3.CQ * Ti[j] * d
        da_, de_, dr_ = surf[k] * math.radians(25.0)
        Fa, Ma = AERO(vb, W[i], de_, da_, dr_); Fa = np.array(Fa).ravel(); Ma = np.array(Ma).ravel()
        wi = W[i]; wd = np.array([np.interp(t, Tw, Wd[:, j]) for j in range(3)])
        M_true = Imat @ wd + np.cross(wi, Imat @ wi)
        Fs = np.array([np.interp(t, tt, tru[:, 2]), np.interp(t, tt, tru[:, 3]), 0.0]) if tt is not None else np.zeros(3)
        M_bob = np.cross(tank_r, Fs)
        res = M_true - M_rot - Ma - M_bob
        rowsM.append((t, *res, *np.cross(r_ac, Fa), np.linalg.norm(vb)))
    Mres = np.array(rowsM)
    print("\nmoment residual = I w_dot + w x I w - M_rotors - M_aero - r_tank x F_bob  [N.m, body FLU]  |  r_ac x F_aero (the aero-centre arm)")
    for tag, w in phases:
        w = w & (spd > 0.5)
        if w.sum() < 10:
            continue
        r_ = Mres[w]
        print(f"  {tag:17s} x {r_[:, 1].mean():+5.2f} ({rms(r_[:, 1]):4.2f}) | y {r_[:, 2].mean():+5.2f} ({rms(r_[:, 2]):4.2f}) | z {r_[:, 3].mean():+5.2f} ({rms(r_[:, 3]):4.2f})"
              f"   | arm predicts y {r_[:, 5].mean():+5.2f}")

w = (spd > 14.5) & (np.abs(roll) > math.radians(5))
if w.sum() > 50:
    print(f"\nlateral residual in wing-borne banked flight ({w.sum()} samples, rms {rms(gy[w]):.2f} N):")
    for nm, xk in (("sin(roll)", np.sin(roll)), ("sideslip [rad]", beta), ("r [rad/s]", r), ("rudder [rad]", dr)):
        A = np.c_[xk[w], np.ones(w.sum())]; c, *_ = np.linalg.lstsq(A, gy[w], rcond=None); pr = A @ c
        r2 = 1 - ((gy[w] - pr) ** 2).sum() / max(((gy[w] - gy[w].mean()) ** 2).sum(), 1e-9)
        print(f"   gap_y ~ {nm:15s} slope {c[0]:+7.2f} N/unit  R^2 {r2:.2f}")
    print(f"   (the real-time-factor artefact would be gap_y = +(1-RTF) m g sin(roll) = {(1 - RTF) * so3.M * so3.G:+.2f} sin(roll); it vanishes on the plant clock)")
