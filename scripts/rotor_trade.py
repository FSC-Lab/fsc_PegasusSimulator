#!/usr/bin/env python3
"""Motor + propeller trade study for the FSC quadplane: predict each combination's operating points
from what was MEASURED, and score it against the requirements the flights and gates established.

    ~/envs/isaac_env/bin/python3 scripts/rotor_trade.py [--db ~/data/uiuc-propDB/UIUC-propDB]
            [--volts 22.2] [--json out.json] [--quiet]

THE MODEL, piece by piece, and where each number comes from.

MOTOR (a voltage-mode ESC driving a DC motor). At duty d:  d*W = omega + s*(Q + Kt*I0), with
  W = Kv*V (no-load speed at full duty) and s = R_sys/Kt^2 (the torque-speed droop, rad/s per N.m).
  For the two motors on the bench (MN4010 KV370, MN4014 KV400; identification/2026-09-17_...), s is
  FITTED from the settled step points (omega + s*Q = alpha*throttle + beta: the stand logged throttle,
  speed and shaft torque; its power sensor was not wired, so V and I are absent). The fit gives
  the no-load speed at full throttle too, which identifies the Kv variants on a ~6S source
  (416 and 356 rpm/V at 22.2 V -> KV400 and KV370). The datasheet Kv variants of the same frames
  reuse the frame's measured "rest of the system" resistance (ESC + wiring + source =
  s*Kt^2 - R_motor): 0.127 ohm (4014 bench) and 0.086 ohm (4010 bench).
  I0 (the no-load current, iron + bearing loss) is the datasheet's, taken as constant.

PROPELLER. C_T(J, rpm) and C_P(J, rpm) from the UIUC Propeller Data Site wind-tunnel sweeps
  (Brandt & Selig; static + advancing), interpolated in J and across rpm, J = V/(nD) along the
  rotor axis. The T-Motor-class carbon multirotor props (15x5 on the bench, and 13x4.4 / 14x4.8 /
  16x5.4: the same family, p/D 0.33-0.34) have no wind-tunnel data: they take the BENCH 15x5's
  static C_T0 / C_P0 (0.0708 / 0.0222, from thrust = 4.627e-5 w^2 and torque/thrust 0.0190 m),
  J0 from the database line J0 = 0.922 p/D + 0.216 (fitted over 134 props >= 10 in, 2026-09-17), and
  the SHAPE of C_T/C_T0 and C_P/C_P0 against J/J0 averaged over the database's low-pitch analogues
  (Aero-naut CAM 13x5, 14x6, 15x6, 16x6; p/D 0.38-0.43).

INERTIA -> ROTOR LAG. Linearised about hover: tau = J_rot / (c_e/s + 2 Q_h/omega_h). J_rot = J_motor
  + k m D^2 (k = 0.065, a tapered blade; m from published weights: T-Motor carbon 12x4 14.5 g,
  16x5.4 28 g -> 28 (D/16)^2.3; APC Thin Electric 14x7 35 g, 15x8 46 g, 16x10 57 g -> 46 (D/15)^3.5).
  c_e is calibrated PER FRAME so the bench 15x5's measured tau is reproduced exactly (0.067 s on the
  4014, 0.077 s on the 4010); the prediction for another prop is therefore a RATIO off a measurement.

THE AIRCRAFT'S DEMAND (the controller's own model, `nmpc_so3_transition_node`): hover M g / 4 per
  rotor; the corridor's front rotor ~ its trim at 8 m/s / 45 deg tilt (Schedule) into 5 m/s of axial
  inflow; the steady level cruise at 16 and 18 m/s solved on build_ode with the least rotor use
  (front ~2.3-2.5 N per rotor), re-solved at each candidate's own mass (induced drag and stall).

THE REQUIREMENTS, each from a measurement in this project (CLAUDE.md):
  * rotor lag tau <= 0.16 s at hover (the 2026-09-16 actuator sweep: 0.18 marginal, 0.20 unflyable).
  * static thrust >= 2x hover per rotor (the full hover yaw differential; the 0.74 N.m authority).
  * corridor: full-throttle thrust at 5 m/s axial inflow >= 1.46x the front trim (what the flown
    12x8 had -- it reached 0.93 throttle there at 0.7 kg).
  * cruise: full-throttle thrust at 16 m/s >= 2.3x the front trim (the 12x8's, which flew with 22%
    of throttle spare; at 18 m/s it had 1.9x and saturated in turbulence).
  * diameter <= 16 in: the CAD's rear props (15.9 in) already reach the fuselage side at the rear
    hubs (y 0.284 - R vs fuselage half-width 0.080 m); front/rear share a 0.571 m boom.
  * motor current at the corridor demand <= the datasheet's 180 s continuous rating.
AND THE OBJECTIVES: mission battery energy (60 s of hover-class flight + 300 s of 16 m/s cruise),
  stall speed (mass: the tank aircraft's stall margin is 0.88 m/s -- the thinnest number it has),
  the hover crosswind a heading can be held in (yaw authority = 4 T_h cQ against the fin's
  0.052 N.m/(m/s)^2 weathercock, 0.7 headroom -- the 2026-09-21 finding), and the rotor lag.
"""
import argparse
import contextlib
import glob
import io
import json
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
sys.path.insert(0, ROOT)
from controller_path import controller_dir                       # noqa: E402

RHO, G = 1.225, 9.81
IN = 0.0254
BENCH = os.path.join(ROOT, "identification", "2026-09-17_rotor_MN4010_MN4014")
M0 = 3.9516                     # the flown tank aircraft (3.4496 + 0.502 water) WITH MN4014 + carbon 15x5
REF_MOTOR_G, REF_PROP = 150.0, "TM 15x5 CF"
V_STALL0 = 15.12                # at M0
WEATHERCOCK = 0.052             # N.m per (m/s)^2 of pure crosswind in hover (1.87 N.m at 6 m/s)
YAW_HEADROOM = 0.7
K_BLADE = 0.065                 # J_prop = k m D^2 for a tapered blade
T_HOVER_S, T_CRUISE_S = 60.0, 300.0


# ------------------------------------------------------------------------------------------------ props
def _rows(path):
    out = []
    for line in open(path):
        p = line.split()
        try:
            out.append([float(x) for x in p])
        except ValueError:
            continue
    return np.array(out)


def parse_name(fn):
    m = re.match(r"^([a-z0-9]+?)_(\d+(?:\.\d+)?)x(\d+(?:\.\d+)?)_", os.path.basename(fn))
    if not m:
        return None
    fam, d, p = m.group(1), float(m.group(2)), m.group(3)
    pv = float(p)
    if pv / d > 1.0 and "." not in p and len(p) > 1:          # '65' is 6.5
        pv = float(p[:-1] + "." + p[-1])
    return fam, d, pv


class UiucProp:
    """Wind-tunnel C_T, C_P against J at several rpm, plus the static sweep."""
    def __init__(self, fam, D_in, P_in, files):
        self.fam, self.D_in, self.P_in = fam, D_in, P_in
        self.D = D_in * IN
        st = [f for f in files if "_static_" in f]
        if not st:
            raise ValueError("no static file")
        s = _rows(st[0]); s = s[np.argsort(s[:, 0])]
        self.s_rpm, self.s_ct, self.s_cp = s[:, 0], s[:, 1], s[:, 2]
        # the tunnel records each rpm as TWO runs a few rpm apart (a low-J and a high-J segment,
        # e.g. 4018 and 4043): merge runs within 3% of rpm into one curve before anything else,
        # or an interpolation "across rpm" mixes two J ranges and extrapolates nonsense
        runs = []
        for f in files:
            if "_static_" in f or "_geom" in f:
                continue
            m = re.search(r"_(\d+)\.txt$", f)
            a = _rows(f)
            if not m or len(a) < 3 or a.shape[1] < 3:
                continue
            runs.append((float(m.group(1)), a[:, :3]))
        runs.sort(key=lambda q: q[0])
        groups = []
        for rpm, a in runs:
            if groups and abs(rpm - groups[-1][0][-1]) <= 0.03 * rpm:
                groups[-1][0].append(rpm); groups[-1][1].append(a)
            else:
                groups.append(([rpm], [a]))
        self.sweeps = []
        for rpms, arrs in groups:
            a = np.vstack(arrs); a = a[np.argsort(a[:, 0])]
            J, inv = np.unique(np.round(a[:, 0], 4), return_inverse=True)       # average repeated J
            ct = np.bincount(inv, a[:, 1]) / np.bincount(inv); cp = np.bincount(inv, a[:, 2]) / np.bincount(inv)
            if len(J) >= 5:
                self.sweeps.append((float(np.mean(rpms)), J, ct, cp))
        if not self.sweeps:
            raise ValueError("no advancing sweeps")

    def _static(self, rpm):
        return (float(np.interp(rpm, self.s_rpm, self.s_ct)), float(np.interp(rpm, self.s_rpm, self.s_cp)))

    def _sweep(self, sw, J):
        rpm, Js, ct, cp = sw
        c0, p0 = self._static(rpm)
        if J <= Js[0]:                                  # blend from the static point to the first sample
            w = J / Js[0] if Js[0] > 0 else 1.0
            return c0 + w * (ct[0] - c0), p0 + w * (cp[0] - p0)
        if J >= Js[-1]:                                 # linear extrapolation past the last sample
            k = len(Js) - 1
            dj = Js[k] - Js[k - 1]
            return (ct[k] + (J - Js[k]) * (ct[k] - ct[k - 1]) / dj, cp[k] + (J - Js[k]) * (cp[k] - cp[k - 1]) / dj)
        return float(np.interp(J, Js, ct)), float(np.interp(J, Js, cp))

    def coef(self, J, rpm):
        if J <= 1e-6:
            return self._static(rpm)
        rp = [q[0] for q in self.sweeps]
        if rpm <= rp[0]:
            return self._sweep(self.sweeps[0], J)
        if rpm >= rp[-1]:
            return self._sweep(self.sweeps[-1], J)
        i = int(np.searchsorted(rp, rpm))
        a, b = self._sweep(self.sweeps[i - 1], J), self._sweep(self.sweeps[i], J)
        w = (rpm - rp[i - 1]) / (rp[i] - rp[i - 1])
        return a[0] + w * (b[0] - a[0]), a[1] + w * (b[1] - a[1])

    def j0(self, rpm=5000.0):
        """The zero-thrust advance ratio of the merged sweep nearest `rpm`, by a linear C_T(J) fit
        over its measured points (the 2026-09-17 method; the momentum model the sim flies is linear)."""
        sw = min(self.sweeps, key=lambda q: abs(q[0] - rpm))
        b, c = np.polyfit(sw[1], sw[2], 1)
        return float(-c / b) if b < 0 else float("nan")


class ShapeProp:
    """A propeller with a measured static point and a borrowed non-dimensional J-shape."""
    def __init__(self, name, D_in, P_in, ct0, cp0, j0, shape):
        self.fam, self.D_in, self.P_in, self.D = name, D_in, P_in, D_in * IN
        self.ct0, self.cp0, self.J0, self.shape = ct0, cp0, j0, shape

    def coef(self, J, rpm):
        jh = J / self.J0
        fT = float(np.interp(jh, self.shape[0], self.shape[1], right=np.nan))
        fP = float(np.interp(jh, self.shape[0], self.shape[2], right=np.nan))
        if not np.isfinite(fT):                         # past the grid: extend linearly
            g = self.shape
            fT = g[1][-1] + (jh - g[0][-1]) * (g[1][-1] - g[1][-2]) / (g[0][-1] - g[0][-2])
            fP = g[2][-1] + (jh - g[0][-1]) * (g[2][-1] - g[2][-2]) / (g[0][-1] - g[0][-2])
        return self.ct0 * fT, self.cp0 * fP

    def j0(self, rpm=5000.0):
        return self.J0


def analogue_shape(props, rpm=4500.0):
    """C_T/C_T0 and C_P/C_P0 against J/J0, averaged over low-pitch database props."""
    jh = np.linspace(0.0, 1.15, 116)
    fT, fP = [], []
    for p in props:
        j0 = p.j0(rpm)
        c0, p0 = p.coef(0.0, rpm)
        fT.append([p.coef(x * j0, rpm)[0] / c0 for x in jh])
        fP.append([p.coef(x * j0, rpm)[1] / p0 for x in jh])
    return jh, np.mean(fT, axis=0), np.mean(fP, axis=0)


def load_uiuc(db):
    groups = {}
    for f in glob.glob(os.path.join(db, "volume-*", "data", "*.txt")):
        n = parse_name(f)
        if n:
            groups.setdefault(n, []).append(f)
    out = {}
    for (fam, d, p), files in groups.items():
        try:
            out[(fam, d, p)] = UiucProp(fam, d, p, files)
        except ValueError:
            pass
    return out


def prop_mass_g(kind, D_in):
    if kind == "cf":
        return 28.0 * (D_in / 16.0) ** 2.3
    return 46.0 * (D_in / 15.0) ** 3.5                  # APC Thin Electric


# ------------------------------------------------------------------------------------------------ motors
def bench_line(motor_dir):
    """-> (alpha, beta, s, kT, kQ, tau_hover, omega_hover) from the settled step points."""
    import ana_spin_lag as A
    pairs = [(o.replace("_outputs", "_inputs"), o)
             for o in sorted(glob.glob(os.path.join(motor_dir, "**", "steps_*_outputs.csv"), recursive=True))]
    with contextlib.redirect_stdout(io.StringIO()):
        r = A.analyse(pairs, os.path.basename(motor_dir), quiet=True)
    x, om, T, Q = np.array(r["static"]).T
    k = (x > 5) & np.isfinite(Q) & np.isfinite(T) & (om > 50)
    x, om, T, Q = x[k], om[k], T[k], Q[k]
    (al, be, s), *_ = np.linalg.lstsq(np.vstack([x, np.ones_like(x), -Q]).T, om, rcond=None)
    kT = float(np.sum(T * om ** 2) / np.sum(om ** 4)); kQ = float(np.sum(Q * om ** 2) / np.sum(om ** 4))
    om_h = math.sqrt(9.69 / kT)
    near = sorted(r["rows"], key=lambda q: abs(q["om"] - om_h))[:6]
    tau_h = float(np.median([q["tau"] for q in near]))
    return dict(alpha=al, beta=be, s=s, kT=kT, kQ=kQ, tau_h=tau_h, om_h=om_h, rms=float(np.std(om - (al * x + be - s * Q))))


class Motor:
    def __init__(self, name, frame, kv, R, I0, mass_g, imax, src, s_bench=None, r_rest=None):
        self.name, self.frame, self.kv, self.R, self.I0 = name, frame, kv, R, I0
        self.mass_g, self.imax, self.src = mass_g, imax, src
        self.kv_si = kv * 2 * math.pi / 60.0
        self.Kt = 1.0 / self.kv_si
        self.s = s_bench if s_bench is not None else (R + r_rest) * self.kv_si ** 2
        self.R_sys = self.s * self.Kt ** 2
        self.V = 22.2
        self.c_e = 1.0
        self.J_mot = 0.0

    @property
    def W(self):
        return self.kv_si * self.V


# ------------------------------------------------------------------------------------------------ physics
def forces(prop, omega, V_ax):
    n = omega / (2 * math.pi)
    J = V_ax / (n * prop.D) if n > 1e-3 else 0.0
    ct, cp = prop.coef(J, n * 60.0)
    T = ct * RHO * n * n * prop.D ** 4
    Q = cp / (2 * math.pi) * RHO * n * n * prop.D ** 5
    return T, Q, J


def duty(m, omega, Q):
    return (omega + m.s * (Q + m.Kt * m.I0)) / m.W


def omega_full(m, prop, V_ax):
    lo, hi = 1.0, m.W
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        _, Q, _ = forces(prop, mid, V_ax)
        if duty(m, mid, max(Q, 0.0)) > 1.0:
            hi = mid
        else:
            lo = mid
    return lo


def omega_for(prop, T_req, V_ax, om_hi):
    lo, hi = 1.0, om_hi
    if forces(prop, hi, V_ax)[0] < T_req:
        return None
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if forces(prop, mid, V_ax)[0] < T_req:
            lo = mid
        else:
            hi = mid
    return hi


def point(m, prop, T_req, V_ax):
    om = omega_for(prop, T_req, V_ax, 3.0 * m.W)
    if om is None:
        return None
    T, Q, J = forces(prop, om, V_ax)
    d = duty(m, om, Q)
    I = Q / m.Kt + m.I0
    P = (om / m.kv_si + m.R_sys * I) * I          # battery power = d V I for an ideal ESC
    return dict(omega=om, T=T, Q=Q, J=J, duty=d, I=I, P=P, feasible=d <= 1.0)


def tau_hover(m, prop, pt, m_prop_g):
    J = m.J_mot + K_BLADE * m_prop_g * 1e-3 * prop.D ** 2
    return J / (m.c_e / m.s + 2.0 * pt["Q"] / pt["omega"])


# ------------------------------------------------------------------------------------------------ demand
def demand_table():
    """Cruise trim at 16/18 m/s for several masses, the corridor's front trim, on the controller's model."""
    sys.path.insert(0, os.path.join(controller_dir(), "px4_vtol_direct_actuator"))
    with contextlib.redirect_stdout(io.StringIO()):
        from px4_vtol_direct_actuator import nmpc_so3_transition_node as so3
        from px4_vtol_direct_actuator import nmpc6_cruise_node as n6
    f = so3.build_ode()

    def xdot(theta, V, Tf, Tr, de):
        c, s_ = math.cos(theta), math.sin(theta)
        R = np.array([[c, 0, -s_], [0, 1, 0], [s_, 0, c]])       # nose UP for +theta (FLU)
        x = np.zeros(19); x[3:12] = R.reshape(-1); x[12] = math.pi / 2; x[13] = V
        return np.array(f(x, np.array([Tf / 2, Tf / 2, Tr / 2, Tr / 2, math.pi / 2, de, 0, 0]))).ravel()

    def cruise(V, M):
        # trim_split balances the vertical force against M g and zeroes the pitch moment for THIS
        # mass; the forward force is then Fx = so3.M * xdot[13] (build_ode divides by its own mass,
        # gravity has no x part), so its SIGN -- all the zero crossing needs -- is read directly
        best = None
        ths = np.radians(np.linspace(-2.0, 14.0, 321))
        for de in np.linspace(-0.40, 0.30, 36):
            res = []
            for th in ths:
                Tf, Tr = so3.trim_split(math.pi / 2, th, V, de, Fz_req=M * so3.G)
                ax = xdot(th, V, Tf, Tr, de)[13]
                res.append((ax, Tf, Tr))
            res = np.array(res)
            idx = np.where(np.diff(np.sign(res[:, 0])) != 0)[0]
            for i in idx:
                w = res[i, 0] / (res[i, 0] - res[i + 1, 0])
                Tf = res[i, 1] + w * (res[i + 1, 1] - res[i, 1]); Tr = res[i, 2] + w * (res[i + 1, 2] - res[i, 2])
                if Tf >= 0 and Tr >= -0.05 and (best is None or Tf + Tr < best[0]):
                    best = (Tf + Tr, Tf / 2, max(Tr, 0.0) / 2)
        return best[1], best[2]

    S = n6.Schedule(V_cruise=16.0)
    corr = S.at(8.0)
    out = dict(M=[], Tf16=[], Tf18=[], so3M=so3.M, corridor_Tf=corr["Tf"] / 2, corridor_tilt=math.degrees(corr["beta"]),
               corridor_theta=math.degrees(corr["theta"]))
    for M in (3.70, 3.9516, 4.20):
        out["M"].append(M)
        out["Tf16"].append(cruise(16.0, M)[0]); out["Tf18"].append(cruise(18.0, M)[0])
    return out


# ------------------------------------------------------------------------------------------------ study
def build_candidates(db):
    uiuc = load_uiuc(db)
    analogues = [uiuc[k] for k in (("ancf", 13, 5.0), ("ancf", 14, 6.0), ("ancf", 15, 6.0), ("ancf", 16, 6.0)) if k in uiuc]
    shape = analogue_shape(analogues)
    ct0, cp0 = 4.627e-5 * (2 * math.pi) ** 2 / (RHO * (15 * IN) ** 4), None
    cp0 = 2 * math.pi * 0.0190 / (15 * IN) * ct0
    props = []
    for d, p in ((13, 4.4), (14, 4.8), (15, 5.0), (16, 5.4)):
        j0 = 0.922 * p / d + 0.216
        pr = ShapeProp(f"TM {d}x{p:g} CF", d, p, ct0, cp0, j0, shape)
        pr.kind, pr.mass_g = "cf", prop_mass_g("cf", d)
        pr.src = "bench 15x5 (static) + UIUC low-pitch J-shape" + (" -- MEASURED on the bench" if d == 15 else " -- same family, scaled")
        props.append(pr)
    # carbon mid-pitch (fixed-wing) props: T-Motor FS 15x8 17 g, 16x8 19.8 g (single blade-pair, the
    # makers' spec); aero = the UIUC Aero-naut CAM CARBON blades of the same size and pitch (measured)
    import copy
    for (d, p, mg) in ((15, 8.0, 17.0), (16, 8.0, 19.8)):
        if ("ancf", d, p) in uiuc:
            pr = copy.copy(uiuc[("ancf", d, p)])
            pr.kind, pr.mass_g, pr.fam = "cf-fw", mg, f"CF {d}x{p:g} FS"
            pr.src = f"UIUC Aero-naut CAM carbon {d}x{p:g} (aero, measured) + T-Motor FS {d}x{p:g} mass {mg:g} g"
            props.append(pr)
    for (fam, d, p), pr in sorted(uiuc.items()):
        if fam == "apce" and 13 <= d <= 16:
            pr.kind, pr.mass_g = "apc", prop_mass_g("apc", d)
            pr.src = "UIUC wind tunnel (static + advancing)"
            pr.fam = f"APC {d:g}x{p:g}E"
            props.append(pr)
    return props, shape, analogues


def motors():
    b14 = bench_line(os.path.join(BENCH, "MN_4014"))
    b10 = bench_line(os.path.join(BENCH, "MN_4010"))
    rr14 = b14["s"] / (400 * 2 * math.pi / 60) ** 2 - 0.067
    rr10 = b10["s"] / (370 * 2 * math.pi / 60) ** 2 - 0.098
    ms = [
        Motor("MN4014 KV400", "4014", 400, 0.067, 1.3, 150.0, 30.0, "BENCH (line + lag) + datasheet mass/I0", s_bench=b14["s"]),
        Motor("MN4014 KV330", "4014", 330, 0.085, 1.1, 150.0, 25.0, "datasheet Kv/R/I0/mass + the 4014 bench's system resistance", r_rest=rr14),
        Motor("MN4010 KV370", "4010", 370, 0.098, 0.8, 112.0, 20.0, "BENCH (line + lag) + datasheet mass/I0", s_bench=b10["s"]),
        Motor("MN4010 KV475", "4010", 475, 0.098, 0.8, 112.0, 30.0, "datasheet Kv/R/I0/mass + the 4010 bench's system resistance", r_rest=rr10),
        Motor("MN4010 KV580", "4010", 580, 0.042, 1.3, 112.0, 31.0, "datasheet Kv/R/I0/mass + the 4010 bench's system resistance", r_rest=rr10),
    ]
    return ms, dict(b14=b14, b10=b10, rr14=rr14, rr10=rr10)


def calibrate_lag(ms, props, bench):
    """Per frame: J_motor from the bell size, c_e so the bench 15x5 reproduces the measured tau."""
    ref = next(p for p in props if p.fam == REF_PROP)
    Jm = {"4014": 2.4e-5, "4010": 1.9e-5}
    for fr, b, name in (("4014", bench["b14"], "MN4014 KV400"), ("4010", bench["b10"], "MN4010 KV370")):
        m = next(q for q in ms if q.name == name)
        pt = point(m, ref, 9.69, 0.0)
        Jr = Jm[fr] + K_BLADE * ref.mass_g * 1e-3 * ref.D ** 2
        b_need = Jr / b["tau_h"]
        c_e = (b_need - 2 * pt["Q"] / pt["omega"]) * m.s
        for q in ms:
            if q.frame == fr:
                q.J_mot, q.c_e = Jm[fr], c_e
    return


def evaluate(m, pf, dem, pr=None):
    """One motor on all four rotors, `pf` on the front (tilting) pair and `pr` (default: the same) on the
    rear pair. Only the front pair meets the airflow -- the corridor and the cruise are its job; the
    rear pair only hovers (and idles in the least-rotor cruise trim)."""
    pr = pr or pf
    dm = 4 * (m.mass_g - REF_MOTOR_G) / 1e3 + 2 * (pf.mass_g + pr.mass_g - 2 * prop_mass_g("cf", 15)) / 1e3
    M = M0 + dm
    Th = M * G / 4.0
    Tf16 = float(np.interp(M, dem["M"], dem["Tf16"])); Tf18 = float(np.interp(M, dem["M"], dem["Tf18"]))
    Tcorr = dem["corridor_Tf"] * M / dem["so3M"]
    name = pf.fam if pr is pf else f"{pf.fam} / {pr.fam}"
    r = dict(motor=m.name, prop=name, front=pf.fam, rear=pr.fam, mixed=pr is not pf, D_in=max(pf.D_in, pr.D_in),
             P_in=pf.P_in, kind=pf.kind, dm_g=dm * 1e3, M=M, Vstall=V_STALL0 * math.sqrt(M / M0), J0=pf.j0())
    hf, hr = point(m, pf, Th, 0.0), point(m, pr, Th, 0.0)
    r["hover"], r["hover_rear"] = hf, hr
    T0f = forces(pf, omega_full(m, pf, 0.0), 0.0)[0]; T0r = forces(pr, omega_full(m, pr, 0.0), 0.0)[0]
    om0 = omega_full(m, pf, 0.0); _, Q0, _ = forces(pf, om0, 0.0)
    r["Tmax0"], r["Tmax0_rear"], r["I_max0"] = T0f, T0r, Q0 / m.Kt + m.I0
    Tc = forces(pf, omega_full(m, pf, 5.0), 5.0)[0]
    r["corr_ratio"] = Tc / Tcorr
    pc = point(m, pf, Tcorr, 5.0)
    r["I_corr"] = pc["I"] if pc else float("nan")
    for V, Tt, key in ((16.0, Tf16, "c16"), (18.0, Tf18, "c18")):
        Tv = forces(pf, omega_full(m, pf, V), V)[0]
        r[key + "_ratio"] = Tv / Tt
        r[key] = point(m, pf, Tt, V)
        r[key + "_trim"] = Tt
        if r[key]:
            r[key]["eta_prop"] = Tt * V / (r[key]["Q"] * r[key]["omega"])
    if hf is None or hr is None:
        return r
    r["cq"] = 0.5 * (hf["Q"] / hf["T"] + hr["Q"] / hr["T"])
    r["yaw_max"] = 2.0 * min(Th, max(T0f - Th, 0.0)) * hf["Q"] / hf["T"] + 2.0 * min(Th, max(T0r - Th, 0.0)) * hr["Q"] / hr["T"]
    r["crosswind"] = math.sqrt(YAW_HEADROOM * r["yaw_max"] / WEATHERCOCK)
    r["tau"] = max(tau_hover(m, pf, hf, pf.mass_g), tau_hover(m, pr, hr, pr.mass_g))
    r["P_hover"] = 2 * hf["P"] + 2 * hr["P"]
    if r["c16"]:
        # battery energy for the reference mission: all four rotors in hover-class flight, the front pair
        # in cruise (the rear pair carries ~0 N in the least-rotor cruise trim; its idle is left out for all)
        r["P_cruise"] = 2 * r["c16"]["P"]
        r["E_Wh"] = (r["P_hover"] * T_HOVER_S + r["P_cruise"] * T_CRUISE_S) / 3600.0
    checks = dict(
        D=max(pf.D_in, pr.D_in) <= 16.0,
        tau=r["tau"] <= 0.16,
        thrust=min(T0f, T0r) >= 2.0 * Th,
        corridor=r["corr_ratio"] >= 1.46,
        cruise16=r["c16_ratio"] >= 2.3,
        current=(r["I_corr"] <= m.imax) if np.isfinite(r["I_corr"]) else False,
        hover_duty=max(hf["duty"], hr["duty"]) <= 0.75,
    )
    r["checks"] = checks
    r["feasible"] = all(checks.values())
    return r


def write_cfg(r, m, prop, out_path):
    """An airframe_id.cfg for one candidate: the flown file with the propulsion lines and the mass /
    inertia moved -- plant and controller both read it through FSC_AIRFRAME_ID (a matched model, as a
    real build would be after its bench session). Every other line is the flown file's, verbatim."""
    base = os.path.join(controller_dir(), "px4_vtol_direct_actuator", "airframe_id.cfg")
    lines = open(base).read().split("\n")
    om0 = omega_full(m, prop, 0.0)
    T0 = forces(prop, om0, 0.0)[0]
    hubs, vals = {}, {}
    for ln in lines:
        mm = re.match(r"^(hub_\w+|cg_x|airframe_kg|ixx|iyy|izz)\s*=\s*([^#]+)", ln)
        if mm:
            vals[mm.group(1)] = [float(x) for x in mm.group(2).split(",")]
    cg = np.array([vals["cg_x"][0], 0.0, 0.0])
    dm_st = (m.mass_g - REF_MOTOR_G + prop.mass_g - prop_mass_g("cf", 15)) / 1e3      # per rotor station
    dI = np.zeros(3)
    for k in ("hub_fl", "hub_fr", "hub_rl", "hub_rr"):
        x, y, z = np.array(vals[k]) - cg
        dI += dm_st * np.array([y * y + z * z, x * x + z * z, x * x + y * y])
    new = dict(airframe_kg=vals["airframe_kg"][0] + 4 * dm_st, ixx=vals["ixx"][0] + dI[0], iyy=vals["iyy"][0] + dI[1],
               izz=vals["izz"][0] + dI[2], max_thrust=T0, omega_max=om0, cq=r["hover"]["Q"] / r["hover"]["T"],
               prop_d=prop.D, prop_j0=prop.j0(4000.0), tau_spin=r["tau"])
    fmt = dict(airframe_kg="{:.6f}", ixx="{:.4f}", iyy="{:.4f}", izz="{:.4f}", max_thrust="{:.2f}", omega_max="{:.1f}",
               cq="{:.4f}", prop_d="{:.4f}", prop_j0="{:.3f}", tau_spin="{:.3f}")
    outl, done = [], set()
    for ln in lines:
        mm = re.match(r"^(\w+)(\s*=\s*)([^#]*?)(\s*#.*)?$", ln)
        if mm and mm.group(1) in new and mm.group(1) not in done:
            done.add(mm.group(1))
            ln = f"{mm.group(1)}{mm.group(2)}{fmt[mm.group(1)].format(new[mm.group(1)])}   # rotor_trade: {m.name} + {prop.fam}"
        outl.append(ln)
    assert done == set(new), set(new) - done
    hdr = (f"# GENERATED by scripts/rotor_trade.py -- {m.name} + {prop.fam} ({prop.src}); a PREDICTION for the gate,\n"
           f"# not a measured airframe. Base: {base}\n")
    open(out_path, "w").write(hdr + "\n".join(outl))
    return new


def main():
    global T_HOVER_S, T_CRUISE_S, K_BLADE
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", default=os.path.expanduser("~/data/uiuc-propDB/UIUC-propDB"))
    ap.add_argument("--volts", type=float, default=22.2)
    ap.add_argument("--json", default=None)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--t_hover", type=float, default=T_HOVER_S, help="mission seconds of hover-class flight")
    ap.add_argument("--t_cruise", type=float, default=T_CRUISE_S, help="mission seconds of 16 m/s cruise")
    ap.add_argument("--k_blade", type=float, default=K_BLADE, help="J_prop = k m D^2")
    ap.add_argument("--cf_fm", type=float, default=0.0,
                    help="give the carbon multirotor family this static figure of merit instead of the bench's "
                         "(0.60): tests whether our stand under-reads the 15x5 (T-Motor's own table implies ~0.67)")
    ap.add_argument("--fw_aero", type=float, default=1.0,
                    help="scale the carbon mid-pitch props' borrowed C_P (their aero is the Aero-naut CAM blade's)")
    ap.add_argument("--rank", type=int, default=0, help="print only the N best feasible uniform builds (sensitivity runs)")
    ap.add_argument("--cfg", default=None, help="write an airframe_id.cfg per --pick candidate into this directory")
    ap.add_argument("--pick", default="", help="'tag=motor|prop;tag=motor|prop' for --cfg")
    a = ap.parse_args()

    T_HOVER_S, T_CRUISE_S, K_BLADE = a.t_hover, a.t_cruise, a.k_blade
    props, shape, analogues = build_candidates(a.db)
    for p in props:
        if p.kind == "cf" and a.cf_fm > 0:
            p.cp0 = p.ct0 ** 1.5 / (math.sqrt(2.0) * a.cf_fm)          # same thrust, less power
        if p.kind == "cf-fw" and a.fw_aero != 1.0:
            p.s_cp = p.s_cp * a.fw_aero
            p.sweeps = [(q[0], q[1], q[2], q[3] * a.fw_aero) for q in p.sweeps]
    ms, bench = motors()
    for m in ms:
        m.V = a.volts
    calibrate_lag(ms, props, bench)
    dem = demand_table()

    print(f"BENCH MOTOR LINES (voltage-mode DC model, settled points): "
          f"MN4014 s {bench['b14']['s']:.0f} rad/s/Nm (rms {bench['b14']['rms']:.1f}), no-load at full throttle "
          f"{bench['b14']['alpha'] * 100 + bench['b14']['beta']:.0f} rad/s; MN4010 s {bench['b10']['s']:.0f} "
          f"(rms {bench['b10']['rms']:.1f}), {bench['b10']['alpha'] * 100 + bench['b10']['beta']:.0f} rad/s; "
          f"rest-of-system resistance 4014 {1e3 * bench['rr14']:.0f} / 4010 {1e3 * bench['rr10']:.0f} mOhm")
    print(f"LAG CALIBRATION: " + ", ".join(f"{m.name} J_mot {m.J_mot:.1e} c_e {m.c_e:.2f}" for m in ms if m.name in ("MN4014 KV400", "MN4010 KV370")))
    print(f"DEMAND at M0 {M0}: hover {M0 * G / 4:.2f} N/rotor; corridor front trim {dem['corridor_Tf']:.2f} N/rotor "
          f"(V 8, tilt {dem['corridor_tilt']:.0f}, pitch {dem['corridor_theta']:.0f}); cruise front "
          + ", ".join(f"M {M:.2f}: 16 m/s {a_:.2f} N, 18 m/s {b_:.2f} N" for M, a_, b_ in zip(dem["M"], dem["Tf16"], dem["Tf18"])))
    print(f"CF J-SHAPE from {', '.join(f'{p.fam}_{p.D_in:g}x{p.P_in:g} (J0 {p.j0(4500):.2f})' for p in analogues)}: "
          f"C_T/C_T0 at J/J0 0.5 = {np.interp(0.5, shape[0], shape[1]):.2f}, C_P/C_P0 = {np.interp(0.5, shape[0], shape[2]):.2f}")

    rows = [evaluate(m, p, dem) for m in ms for p in props]
    # mixed builds: a cruise prop on the front pair, a hover prop on the rear pair (needs per-pair prop
    # parameters in the controller -- the airframe file carries ONE prop today; the plant has one per rotor)
    mixed = [evaluate(m, pf, dem, pr) for m in ms for pf in props for pr in props
             if pf is not pr and pr.kind == "cf" and pf.kind != "cf"]
    # the bench cross-check: the model's own hover point for the bench pair against the log
    ref = next(r for r in rows if r["motor"] == "MN4014 KV400" and r["prop"] == REF_PROP)
    print(f"REFERENCE (MN4014 KV400 + {REF_PROP}, the flown build): hover {ref['hover']['omega']:.0f} rad/s "
          f"(bench 458), duty {ref['hover']['duty']:.2f}, tau {ref['tau']:.3f} s (bench 0.067), "
          f"static max {ref['Tmax0']:.1f} N (airframe_id.cfg carries 36.3 -- a linear extrapolation)")
    hdr = (f"{'motor':13s} {'prop':15s} {'dm g':>6s} {'Vstall':>6s} {'J0':>5s} {'hov W':>6s} {'duty':>5s} {'cQ mm':>6s} "
           f"{'xwind':>5s} {'tau':>5s} {'Tmax':>5s} {'corr':>5s} {'c16':>5s} {'c18':>5s} {'Icor':>5s} {'cru W':>6s} {'E Wh':>5s}  ok / failed")
    good = sorted([r for r in rows if r.get("E_Wh")], key=lambda r: r["E_Wh"])
    good_mixed = sorted([r for r in mixed if r.get("E_Wh") and r.get("feasible")], key=lambda r: r["E_Wh"])[:12]
    if a.rank:
        ok = [r for r in good if r["feasible"]][:a.rank]
        ref_E = ref["E_Wh"]
        print(f"RANK (volts {a.volts}, hover {T_HOVER_S:.0f} s + cruise {T_CRUISE_S:.0f} s, k_blade {K_BLADE}, cf_fm {a.cf_fm or 'bench'}, "
              f"fw_aero x{a.fw_aero}); reference {ref_E:.1f} Wh:")
        for i, r in enumerate(ok):
            print(f"  {i + 1}. {r['motor']:13s} {r['prop']:12s} E {r['E_Wh']:5.1f} Wh ({100 * (r['E_Wh'] / ref_E - 1):+4.0f}%)  "
                  f"hover {r['P_hover']:4.0f} W  cruise {r['P_cruise']:4.0f} W  tau {r['tau']:.3f}  Vstall {r['Vstall']:.2f}  "
                  f"corr {r['corr_ratio']:.2f}  c16 {r['c16_ratio']:.1f}")
        cf15 = [r for r in good if r["prop"] == "TM 15x5 CF" and r["motor"] in ("MN4010 KV370", "MN4014 KV400")]
        for r in cf15:
            print(f"     vs {r['motor']:13s} {r['prop']:12s} E {r['E_Wh']:5.1f} Wh  hover {r['P_hover']:4.0f} W  cruise {r['P_cruise']:4.0f} W  "
                  f"{'feasible' if r['feasible'] else 'FAILS ' + ','.join(k for k, v in r['checks'].items() if not v)}")
        return 0
    if not a.quiet:
        print("\n" + hdr)
        for r in good:
            fails = [k for k, v in r["checks"].items() if not v]
            print(f"{r['motor']:13s} {r['prop']:15s} {r['dm_g']:+6.0f} {r['Vstall']:6.2f} {r['J0']:5.2f} {r['P_hover']:6.0f} "
                  f"{r['hover']['duty']:5.2f} {1e3 * r['cq']:6.1f} {r['crosswind']:5.2f} {r['tau']:5.3f} {r['Tmax0']:5.1f} "
                  f"{r['corr_ratio']:5.2f} {r['c16_ratio']:5.2f} {r['c18_ratio']:5.2f} {r['I_corr']:5.1f} "
                  f"{2 * r['c16']['P']:6.0f} {r['E_Wh']:5.1f}  {'OK' if r['feasible'] else ' - '} {','.join(fails)}")
        print("\nBEST MIXED (front cruise prop / rear hover prop), feasible only:\n" + hdr)
        for r in good_mixed:
            print(f"{r['motor']:13s} {r['prop'][:15]:15s} {r['dm_g']:+6.0f} {r['Vstall']:6.2f} {r['J0']:5.2f} {r['P_hover']:6.0f} "
                  f"{r['hover']['duty']:5.2f} {1e3 * r['cq']:6.1f} {r['crosswind']:5.2f} {r['tau']:5.3f} {r['Tmax0']:5.1f} "
                  f"{r['corr_ratio']:5.2f} {r['c16_ratio']:5.2f} {r['c18_ratio']:5.2f} {r['I_corr']:5.1f} "
                  f"{r['P_cruise']:6.0f} {r['E_Wh']:5.1f}  {r['prop']}")
    if a.cfg:
        os.makedirs(a.cfg, exist_ok=True)
        for item in [q for q in a.pick.split(";") if q.strip()]:
            tag, spec = item.split("=", 1); mn, pn = spec.split("|")
            m = next(q for q in ms if q.name == mn.strip()); p = next(q for q in props if q.fam == pn.strip())
            r = next(q for q in rows if q["motor"] == m.name and q["prop"] == p.fam)
            v = write_cfg(r, m, p, os.path.join(a.cfg, f"{tag.strip()}.cfg"))
            print(f"cfg {tag.strip()}: {m.name} + {p.fam}: " + " ".join(f"{k} {x:.4g}" for k, x in v.items()))
    if a.json:
        def clean(o):
            if isinstance(o, dict):
                return {k: clean(v) for k, v in o.items()}
            if isinstance(o, (list, tuple)):
                return [clean(v) for v in o]
            if isinstance(o, (np.floating, float)):
                return None if not np.isfinite(o) else float(o)
            if isinstance(o, (np.bool_,)):
                return bool(o)
            return o
        json.dump(clean(dict(rows=rows, mixed=good_mixed, demand=dem, bench=bench, volts=a.volts,
                             motors=[dict(name=m.name, kv=m.kv, R=m.R, R_sys=m.R_sys, s=m.s, I0=m.I0, mass_g=m.mass_g,
                                          imax=m.imax, src=m.src, W=m.W) for m in ms],
                             props=[dict(name=p.fam, D=p.D_in, P=p.P_in, kind=p.kind, mass_g=p.mass_g, J0=p.j0(), src=p.src) for p in props])),
                  open(a.json, "w"), indent=1)
        print(f"\n-> {a.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
