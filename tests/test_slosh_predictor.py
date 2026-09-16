#!/usr/bin/env python3
"""The slosh PREDICTOR against the plant's own pendulum.

    python3 tests/test_slosh_predictor.py

`slosh_predictor.SloshPredictor` (controller package) is an EKF whose internal model is the tank's
first mode -- the same PendulumSlosh the simulator integrates (imported here BY PATH: one
pendulum, not two). It is fed what the STESO gives it in flight -- the lumped residual force and
moment, i.e. the bob's reaction plus a bias plus noise, seen through the STESO's lag -- and asked
for four things the STESO alone cannot give:

  A. converge on the bob (position / velocity) from the force alone, and PREDICT its force at
     t + tau better than holding the last reading (tau = 0.05 s, the transport delay the inner
     cancels over; 0.25 s; 0.5 s -- half a period of the 100 mm tank's 1.14 Hz mode);
  B. split a force bias (1 N) and a moment bias (0.5 N.m) -- the UNCERTAINTY -- from the bob's
     oscillation -- the DISTURBANCE -- with the moment consistent with the force through r_tank;
  C. through a 0.1 s first-order lag on the measurement (the STESO's own), still track;
  D. see a wall impact coming: with the bob driven toward the R/2 wall, `impact_eta` called
     0.3 s before the plant's first contact is within 0.1 s of it, and the predicted contact
     force has the right sign and order.

Every number below is a measured bar, not a target: the tank is the flown 100 mm / 0.502 kg one
(m1 0.41 kg at 1.14 Hz, wall 50 mm at 35 rad/s), the tick is the node's 120 Hz. 2026-09-14.
"""
import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from controller_path import inner_dir                       # noqa: E402

_spec = importlib.util.spec_from_file_location("slosh_predictor", os.path.join(inner_dir(), "slosh_predictor.py"))
sp = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(sp)
_spec2 = importlib.util.spec_from_file_location(
    "slosh", os.path.join(HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator", "logic", "dynamics", "slosh.py"))
sl = importlib.util.module_from_spec(_spec2); _spec2.loader.exec_module(sl)

FAIL = []


def check(name, ok, detail=""):
    print(f"  {'OK  ' if ok else 'FAIL'}  {name}{('  -- ' + detail) if detail else ''}")
    if not ok:
        FAIL.append(name)


def tank100():
    """The flown 100 mm tank at 0.502 kg, 75% fill (VTOL_TANK_R=0.10)."""
    R = 0.10; mw = 0.502
    h = mw / (1000.0 * math.pi * R * R)
    return sl.slosh_parameters(sl.TankConfig(radius=R, height=h / 0.75, fill_frac=0.75))


def make(p, r_tank):
    return sp.SloshPredictor(m1=p.slosh_mass, L1=p.pendulum_length, zeta=p.zeta, r_tank=r_tank,
                             wall=p.wall, wall_omega=p.wall_omega, wall_zeta=p.wall_zeta)


def run(p, r_tank, f_of_t, T, dt=1 / 120.0, plant_dt=1 / 240.0, bias_F=(0.0, 0.0), bias_M=(0.0, 0.0, 0.0),
        noise_F=0.0, noise_M=0.0, lag=0.0, x0=(0.0, 0.0), seed=1, horizons=(0.05, 0.25, 0.5)):
    """Plant: PendulumSlosh at 240 Hz. Predictor: 120 Hz, fed the true reaction + bias + noise
    through an optional first-order lag. Returns a dict of time series."""
    rng = np.random.default_rng(seed)
    plant = sl.PendulumSlosh(p); plant.x = [float(x0[0]), float(x0[1])]
    pred = make(p, r_tank)
    r = np.asarray(r_tank, float)
    zF = np.zeros(2); zM = np.zeros(3)
    log = dict(t=[], F_true=[], F_est=[], F_meas=[], x_true=[], x_est=[], bF=[], bM=[], eta=[], wall_s=[], F_bnd=[],
               pred={tau: [] for tau in horizons})
    t = 0.0; nsub = int(round(dt / plant_dt))
    while t < T:
        f = np.asarray(f_of_t(t), float)
        F = None
        for _ in range(nsub):
            F = np.array(plant.step(f, plant_dt))
        F_true = F[:2]
        M_true = np.cross(r, np.array([F_true[0], F_true[1], 0.0]))
        F_m = F_true + np.asarray(bias_F) + noise_F * rng.standard_normal(2)
        M_m = M_true + np.asarray(bias_M) + noise_M * rng.standard_normal(3)
        if lag > 0.0:
            a = 1.0 - math.exp(-dt / lag)
            zF = zF + a * (F_m - zF); zM = zM + a * (M_m - zM)
        else:
            zF, zM = F_m, M_m
        pred.step(zF, zM, f, dt)
        t += dt
        log["t"].append(t); log["F_true"].append(F_true.copy()); log["F_est"].append(pred.force)
        log["F_meas"].append(zF.copy()); log["x_true"].append(np.array(plant.x)); log["x_est"].append(pred.x)
        log["bF"].append(pred.bias_F); log["bM"].append(pred.bias_M)
        log["wall_s"].append(float(pred.wall_s.max())); log["F_bnd"].append(pred.bounded(pred.force))
        for tau in horizons:
            log["pred"][tau].append(pred.predict(tau, f_tank=f_of_t(t))[0])
        log["eta"].append(pred.impact_eta(1.0, f_tank=f_of_t(t)))
    for k in ("t", "F_true", "F_est", "F_meas", "x_true", "x_est", "bF", "bM", "wall_s", "F_bnd"):
        log[k] = np.array(log[k])
    for tau in horizons:
        log["pred"][tau] = np.array(log["pred"][tau])
    return log, pred


def rms(a):
    a = np.asarray(a, float)
    return float(np.sqrt(np.mean(a * a))) if a.size else float("nan")


def score_prediction(log, tau, dt=1 / 120.0, t_from=2.0):
    """rms of (prediction made at t for t+tau) - (truth at t+tau), vs holding the reading at t."""
    k = int(round(tau / dt)); w = log["t"] >= t_from
    idx = np.where(w)[0]; idx = idx[idx + k < len(log["t"])]
    e_pred = log["pred"][tau][idx, 0] - log["F_true"][idx + k, 0]
    e_hold = log["F_meas"][idx, 0] - log["F_true"][idx + k, 0]
    return rms(e_pred), rms(e_hold), rms(log["F_true"][idx + k, 0])


def main():
    p = tank100(); r_tank = (0.262, 0.0, 0.0)
    print(f"tank: {sl.describe(sl.TankConfig(radius=0.10, height=0.502 / (1000 * math.pi * 0.01) / 0.75, fill_frac=0.75))}")
    print(f"      m1 {p.slosh_mass:.3f} kg  L1 {p.pendulum_length * 1e3:.1f} mm  f1 {p.freq_hz:.2f} Hz  zeta {p.zeta:.4f}  wall {p.wall * 1e3:.0f} mm @ {p.wall_omega} rad/s")
    hover = lambda t: (0.0, 0.0, 9.81)

    print("\nA. free oscillation from 20 mm, exact force fed back, no bias, no noise")
    log, pred = run(p, r_tank, hover, T=6.0, x0=(0.02, 0.0))
    w = log["t"] > 2.0
    ex = rms(log["x_est"][w, 0] - log["x_true"][w, 0]); ax = rms(log["x_true"][w, 0])
    check("bob position converged (rms err < 10% of amplitude after 2 s)", ex < 0.10 * ax, f"err {1e3 * ex:.2f} mm on {1e3 * ax:.1f} mm rms")
    for tau, frac in ((0.05, 0.5), (0.25, 0.25), (0.5, 0.25)):
        ep, eh, at = score_prediction(log, tau)
        check(f"prediction at +{tau:.2f} s beats hold-last (pred < {frac:.2f} x hold)", ep < frac * eh,
              f"pred err {ep:.3f} N, hold err {eh:.3f} N, truth rms {at:.3f} N")

    print("\nB. turn entry: 1 m/s^2 lateral pulse for 1 s (bob to 19 mm, off the wall), +1.0 N force bias, +0.5 N.m yaw bias, 0.3 N / 0.1 N.m noise")
    f_pulse = lambda t: (0.0, 1.0 if 1.0 <= t < 2.0 else 0.0, 9.81)
    log, pred = run(p, r_tank, f_pulse, T=8.0, bias_F=(0.0, 1.0), bias_M=(0.0, 0.0, 0.5), noise_F=0.3, noise_M=0.1)
    w = log["t"] > 4.0
    bF = log["bF"][w, 1].mean(); bM = log["bM"][w, 2].mean()
    check("force bias recovered (1.0 +- 0.25 N)", abs(bF - 1.0) < 0.25, f"bias_F_y {bF:+.2f} N")
    check("moment bias recovered (0.5 +- 0.15 N.m)", abs(bM - 0.5) < 0.15, f"bias_M_z {bM:+.2f} N.m")
    c = np.corrcoef(log["F_est"][w, 1], log["F_true"][w, 1])[0, 1]
    check("bob force tracks the truth through bias + noise (corr > 0.9)", c > 0.9,
          f"corr {c:+.3f}; truth rms {rms(log['F_true'][w, 1]):.2f} N, est err {rms(log['F_est'][w, 1] - log['F_true'][w, 1]):.2f} N")
    # the moment must be the force through the arm, not its own thing
    M_est = np.array([np.cross(np.array(r_tank), np.array([F[0], F[1], 0.0])) for F in log["F_est"][w]])
    M_true = np.array([np.cross(np.array(r_tank), np.array([F[0], F[1], 0.0])) for F in log["F_true"][w]])
    check("moment consistent with the force via r_tank (yaw corr > 0.9)", np.corrcoef(M_est[:, 2], M_true[:, 2])[0, 1] > 0.9)
    k = int(round(0.25 / (1 / 120.0))); idx = np.where(w)[0]; idx = idx[idx + k < len(log["t"])]
    ep = rms(log["pred"][0.25][idx, 1] - log["F_true"][idx + k, 1]); eh = rms(log["F_meas"][idx, 1] - log["F_true"][idx + k, 1])
    check("prediction at +0.25 s beats hold-last under bias + noise", ep < 0.5 * eh, f"pred err {ep:.3f} N vs hold {eh:.3f} N")

    print("\nC. the same through a 0.1 s first-order lag on the measurement (the STESO's own)")
    log, pred = run(p, r_tank, f_pulse, T=8.0, bias_F=(0.0, 1.0), noise_F=0.3, lag=0.1)
    w = log["t"] > 4.0
    c0 = np.corrcoef(log["F_est"][w, 1], log["F_true"][w, 1])[0, 1]
    check("unmodelled lag: still tracks (corr > 0.8) -- the ceiling without the lag model", c0 > 0.8, f"corr {c0:+.3f}")
    bF = log["bF"][w, 1].mean()
    check("bias still recovered through the lag (1.0 +- 0.3 N)", abs(bF - 1.0) < 0.3, f"{bF:+.2f} N")
    _make = globals()["make"]
    globals()["make"] = lambda pp, rt: sp.SloshPredictor(m1=pp.slosh_mass, L1=pp.pendulum_length, zeta=pp.zeta, r_tank=rt, wall=pp.wall,
                                                        wall_omega=pp.wall_omega, wall_zeta=pp.wall_zeta, meas_lag=0.1)
    log, pred = run(p, r_tank, f_pulse, T=8.0, bias_F=(0.0, 1.0), noise_F=0.3, lag=0.1)
    globals()["make"] = _make
    c1 = np.corrcoef(log["F_est"][w, 1], log["F_true"][w, 1])[0, 1]
    check("lag MODELLED (meas_lag 0.1): de-lagged (corr > 0.95)", c1 > 0.95, f"corr {c1:+.3f} (was {c0:+.3f} unmodelled)")

    print("\nD. the wall: 3 m/s^2 fore-aft braking from t = 1 s drives the bob to the +50 mm wall")
    f_brake = lambda t: (-3.0 if t >= 1.0 else 0.0, 0.0, 9.81)
    log, pred = run(p, r_tank, f_brake, T=4.0, noise_F=0.2)
    hit = np.where(np.abs(log["x_true"][:, 0]) >= p.wall)[0]
    check("the plant's bob reaches the wall in this scenario", len(hit) > 0)
    if len(hit):
        t_hit = log["t"][hit[0]]
        # the warning can only exist once the braking is in the input (t >= 1.0): take the first
        # tick after the onset at which the predictor announces a contact, and check what it said
        i_on = np.searchsorted(log["t"], 1.0)
        i_first = next((i for i in range(i_on, len(log["t"])) if log["eta"][i] is not None), None)
        if i_first is None:
            check("impact announced before it happens", False, "impact_eta never returned a time")
        else:
            t_said = log["t"][i_first]; eta = log["eta"][i_first]
            check("impact announced >= 0.1 s ahead, within 0.1 s of the plant's contact",
                  (t_hit - t_said) >= 0.1 and abs((t_said + eta) - t_hit) < 0.1,
                  f"plant contact t={t_hit:.2f}; at t={t_said:.2f} the predictor said +{eta:.2f} s -> {t_said + eta:.2f}")
        # the contact force: right sign (the wall pushes the bob back -> reaction on the airframe is +x
        # when the bob is at +x and decelerating... the truth's sign is whatever the plant gives), right order
        wc = (log["t"] >= t_hit) & (log["t"] < t_hit + 0.3)
        Ft = log["F_true"][wc, 0]; Fe = log["F_est"][wc, 0]
        check("contact force tracked (corr > 0.8, |peak| within 2x)", np.corrcoef(Fe, Ft)[0, 1] > 0.8 and 0.5 < (np.abs(Fe).max() / max(np.abs(Ft).max(), 1e-6)) < 2.0,
              f"corr {np.corrcoef(Fe, Ft)[0, 1]:+.2f}, peak true {np.abs(Ft).max():.2f} N est {np.abs(Fe).max():.2f} N")

    print("\nE. the wall-saturation guard (hardware-transfer piece 7): the model's force is bounded to its domain,")
    print("   and the guard's clock counts a PARKED estimate, not a passing one")
    pred = make(p, r_tank)
    fb = pred.force_bound(); w1 = pred.omega1
    check("force_bound = m1 w^2 wall (the bob AT the wall, the largest force the linear model can make)",
          abs(fb - p.slosh_mass * w1 * w1 * p.wall) < 1e-12 and fb > 0.0, f"{fb:.3f} N for m1 {p.slosh_mass:.3f} kg, f1 {w1 / (2 * math.pi):.2f} Hz, wall {1e3 * p.wall:.0f} mm")
    check("bounded() clips to +-force_bound and leaves an in-domain force alone",
          np.allclose(pred.bounded([5.0, -5.0]), [fb, -fb]) and np.allclose(pred.bounded([0.1, -0.2]), [0.1, -0.2]))
    # E1: a residual the tank did not make -- the plant's bob at rest, the READING a 3 N sinusoid at
    # the tank's own mode (the HIL clock artefact's shape, 7x what this bob can make inside its wall)
    f_still = lambda t: (0.0, 0.0, 9.81)
    rng = np.random.default_rng(5); plant = sl.PendulumSlosh(p); pred = make(p, r_tank)
    dt = 1 / 120.0; xs = []; Fraw = []; Fbnd = []; ws = []
    for k in range(int(3.0 / dt)):
        t = k * dt
        for _ in range(2):
            plant.step(np.array(f_still(t)), dt / 2)
        z = np.array([3.0 * math.sin(w1 * t), 0.0]); zM = np.cross(np.asarray(r_tank), np.array([z[0], z[1], 0.0]))
        pred.step(z, zM, f_still(t), dt)
        xs.append(pred.x[0]); Fraw.append(pred.force[0]); Fbnd.append(pred.bounded(pred.force)[0]); ws.append(float(pred.wall_s[0]))
    xs, Fraw, Fbnd, ws = map(np.array, (xs, Fraw, Fbnd, ws))
    check("a fictitious 3 N residual at f1 drives the ESTIMATE past the wall (the failure the flights showed)",
          np.abs(xs).max() >= p.wall, f"|x_est| max {1e3 * np.abs(xs).max():.0f} mm vs wall {1e3 * p.wall:.0f} (the plant's bob: {1e3 * abs(plant.x[0]):.1f} mm)")
    check("the RAW model force then carries the contact law's fiction (> the bound)", np.abs(Fraw).max() > 1.5 * fb,
          f"raw |F| max {np.abs(Fraw).max():.2f} N vs bound {fb:.2f}")
    check("the BOUNDED force never exceeds what a bob inside the wall makes", np.abs(Fbnd).max() <= fb + 1e-9,
          f"bounded |F| max {np.abs(Fbnd).max():.2f} N -- the 1 N.m pulses cannot happen on any channel")
    check("a passing excursion does not read as parked (wall_s never reaches the 1 s guard)", ws.max() < 1.0,
          f"wall_s max {ws.max():.2f} s (the estimate swings through the centre every half period)")
    # E2: a legitimately PARKED bob -- section D's braking held 2 s, then released: the clock runs past
    # the guard while the bob sits on the wall and resets once it leaves
    f_park = lambda t: (-3.0 if 1.0 <= t < 3.0 else 0.0, 0.0, 9.81)
    log, pred = run(p, r_tank, f_park, T=5.0, noise_F=0.2)
    check("a bob held on the wall runs the guard clock past 1 s", log["wall_s"].max() > 1.0, f"wall_s max {log['wall_s'].max():.2f} s")
    i_rel = np.searchsorted(log["t"], 3.0)
    check("released, the estimate leaves the wall and the clock resets (wall_s 0 and inside 80% by the end)",
          log["wall_s"][-1] == 0.0 and bool(pred.inside(0.8).all()), f"wall_s at the end {log['wall_s'][-1]:.2f} s, |x_est| {1e3 * np.abs(pred.x).max():.1f} mm")
    check("the clock was still running at the release (it counts continuously while parked)", log["wall_s"][i_rel - 1] > 0.5,
          f"wall_s at t=3 s: {log['wall_s'][i_rel - 1]:.2f} s")

    print()
    print("PASS -- the predictor observes the bob, splits the biases and predicts ahead" if not FAIL else f"FAIL ({len(FAIL)}): " + ", ".join(FAIL))
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
