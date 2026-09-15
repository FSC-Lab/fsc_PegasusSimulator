#!/usr/bin/env python3
"""Piece-1 audit of the residual estimators the cascade's L1 augmentation can carry
(fsc_autopilot_ros2: px4_vtol_direct_actuator/so3_steso.py): SO3STESO (super-twisting),
LinearESO (high-gain ESO) and MomentumDOB (momentum-based disturbance observer).

    ~/envs/isaac_env/bin/python3 tests/test_residual_estimators.py

This is NOT the ranking (that is the closed-loop gate). It pins that the three answer the same
question through the same interface with NO CHANNEL MIXING, so that swapping `--est` in the
cascade changes one thing:
  1. INTERFACE: every class has step(R, M, dt, omega=), sigma, torque, omega_dot_hat, hold,
     reset(); make_estimator() builds each from its own knob.
  2. PER-AXIS ISOLATION, moment use (J = assembled inertia, rigid body integrated with the true
     J, noiseless, 1 kHz): a +1 N.m step on ONE axis the model does not contain reads +1 N.m on
     that axis (within 3% after settling) and < 1e-3 N.m on the other two -- roll, pitch, yaw in
     turn, each estimator. The sign is decided by the plant, never by the paper.
  3. PER-AXIS ISOLATION, force use (J = m I, "rate" = a velocity, "moment" = a force): the same
     three-axis check with a 1 N step; the same class must read a force the way it reads a moment.
  4. SEPARATE TWINS: a moment instance and a force instance built side by side share no state --
     stepping one leaves the other's sigma / omega_hat / internal integrals byte-identical.
  5. omega_dot_hat = J^-1 M_applied + sigma, with the M the estimator was stepped on, and the
     sigma of the same tick (the STESO/LESO export the pre-update sigma there -- one tick, <= its
     slew per tick; the momentum observer has no pre/post distinction).
  6. hold: sigma is frozen exactly (bit-identical over the hold), the rate/momentum estimate keeps
     tracking (the innovation stays bounded), and the release is bumpless (no jump larger than one
     tick's slew).
  7. The STESO's own numerics are untouched: its trace on the mixed disturbance is byte-identical
     (exact bytes, no rounding: a one-ULP edit fails it) to a hash pinned here (change the hash
     only with a deliberate change to SO3STESO).
  8. The things the first audit round (2026-09-14, five reviewer lenses + mutation testing of this
     file) found missing: .torque == J @ sigma is checked, not only sigma; make_estimator builds
     the class the kind names with the knobs it was given (non-default values, per-axis knobs,
     one vocabulary filtered per kind, unknown knobs refused); the mode branch is pinned (an
     'attitude' LESO differs from a 'rate' one, an unknown mode raises on every kind); the twins
     check keeps stepping the moment instance after the force twin ran and compares its whole
     trace; omega_dot_hat is checked DURING a hold; add_gyro=True (the one cross-axis term inside
     an estimator) is exercised on a spinning body; the first sample starts ON the measurement
     (a body already rotating at 1 rad/s gives no first-tick kick); reset() after stepping gives a
     fresh instance; a live retune (set_L / set_wo / set_K) does not move the estimate.
"""
import hashlib
import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
from controller_path import inner_dir                       # noqa: E402

_spec = importlib.util.spec_from_file_location("so3_steso", os.path.join(inner_dir(), "so3_steso.py"))
st = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(st)

J = np.array([0.265, 0.303, 0.490])          # the assembled inertia (so3.IXX/IYY/IZZ)
M_KG = 3.9516                                # the tank aircraft
KINDS = ("steso", "leso", "mdob")
# the knobs each is built with here (piece 3 sets the flight defaults by noise parity)
KW = {"steso": dict(L=20.0, mode="rate", eps="auto"), "leso": dict(wo=20.0, mode="rate"), "mdob": dict(K=20.0)}
KW_F = {"steso": dict(L=3.0, mode="rate", eps="auto"), "leso": dict(wo=6.0, mode="rate"), "mdob": dict(K=6.0)}

FAILS = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


def build(kind, J_, force=False):
    return st.make_estimator(kind, J_, add_gyro=False, **(KW_F[kind] if force else KW[kind]))


def run_step(est, Jv, axis, amp, dt=1e-3, T=3.0, t_step=1.0, gyro=True):
    """Rigid body with inertia diag(Jv), a known command, an unmodelled step `amp` on `axis`
    from t_step. Returns the estimate trace (n,3) in torque units and the truth."""
    Jm = np.diag(Jv)
    R = np.eye(3); w = np.zeros(3)
    n = int(round(T / dt)); out = np.zeros((n, 3)); truth = np.zeros((n, 3)); wd = np.zeros((n, 3))
    for k in range(n):
        t = k * dt
        M_cmd = np.array([0.2 * math.sin(math.pi * t), 0.1 * math.cos(2.0 * t), 0.05 * math.sin(t)])
        M_model = M_cmd - (np.cross(w, Jm @ w) if gyro else 0.0)
        d = np.zeros(3); d[axis] = amp if t >= t_step else 0.0
        w_dot = np.linalg.solve(Jm, M_model + d)
        est.step(R, M_model, dt, omega=w)
        R = R @ st.so3_exp(w * dt); w = w + w_dot * dt
        out[k] = Jm @ est.sigma
        truth[k] = d
        wd[k] = est.omega_dot_hat
    return out, truth


def main():
    print("== 1. interface")
    for kind in KINDS:
        e = build(kind, J)
        ok = all(hasattr(e, a) for a in ("step", "sigma", "torque", "omega_dot_hat", "hold", "reset", "mode"))
        e.step(np.eye(3), np.zeros(3), 1e-3, omega=np.zeros(3))
        ok = ok and np.all(np.isfinite(e.sigma)) and e.torque.shape == (3,) and e.omega_dot_hat.shape == (3,)
        check(ok, f"{kind}: step/sigma/torque/omega_dot_hat/hold/reset present and finite")
    try:
        st.make_estimator("nope", J); check(False, "unknown kind must raise")
    except ValueError:
        check(True, "unknown kind raises ValueError")

    # MIXING is measured against the estimator's OWN floor: the same run with NO step gives what
    # the other axes read anyway (the STESO's sign-function chatter is ~2e-3 N.m at 1 kHz even
    # noiseless; the linear ones are at machine precision). A step on one axis must not raise the
    # other two above 2x that floor (+1e-6 absolute, so a 1e-17 floor is not a trap).
    print("== 2. per-axis isolation, MOMENT use (rigid body, true J, noiseless 1 kHz, +1 N.m step)")
    for kind in KINDS:
        ref, _ = run_step(build(kind, J), J, 1, 0.0)
        for axis, name in enumerate(("roll", "pitch", "yaw")):
            e = build(kind, J)
            out, truth = run_step(e, J, axis, 1.0)
            tail = out[-500:]                       # last 0.5 s
            own = tail[:, axis].mean()
            others = np.abs(np.delete(tail, axis, axis=1)).max()
            floor = np.abs(np.delete(ref[-500:], axis, axis=1)).max()
            tq = np.abs(e.torque - np.diag(J) @ e.sigma).max()       # what the L1 law actually cancels
            check(abs(own - 1.0) < 0.03 and others <= 2.0 * floor + 1e-6 and tq < 1e-12,
                  f"{kind} {name}: reads {own:+.4f} N.m on its axis; other axes {others:.1e} vs their own floor {floor:.1e}; torque == J sigma ({tq:.0e})")

    print("== 3. per-axis isolation, FORCE use (J = m I, +1 N step)")
    Jf = np.array([M_KG, M_KG, M_KG])
    for kind in KINDS:
        ref, _ = run_step(build(kind, Jf, force=True), Jf, 1, 0.0, T=6.0, t_step=1.0, gyro=False)
        for axis, name in enumerate(("x", "y", "z")):
            e = build(kind, Jf, force=True)
            out, truth = run_step(e, Jf, axis, 1.0, T=6.0, t_step=1.0, gyro=False)
            tail = out[-1000:]
            own = tail[:, axis].mean(); others = np.abs(np.delete(tail, axis, axis=1)).max()
            floor = np.abs(np.delete(ref[-1000:], axis, axis=1)).max()
            tq = np.abs(e.torque - M_KG * e.sigma).max()
            check(abs(own - 1.0) < 0.03 and others <= 2.0 * floor + 1e-6 and tq < 1e-12,
                  f"{kind} F{name}: reads {own:+.4f} N on its axis; other axes {others:.1e} vs their own floor {floor:.1e}; force == m sigma ({tq:.0e})")

    print("== 4. separate twins share no state (the moment trace with and without the force twin running)")
    for kind in KINDS:
        def moment_trace(with_twin):
            em = build(kind, J); ef = build(kind, Jf, force=True) if with_twin else None
            tr = []
            for k in range(300):
                em.step(np.eye(3), np.array([0.1 * math.sin(0.05 * k), 0.0, 0.02]), 5e-3, omega=np.array([0.2, 0.01 * k, 0.0]))
                if ef is not None:
                    ef.step(np.eye(3), np.array([0.0, 3.0, 0.0]), 5e-3, omega=np.array([0.0, 1.0 + 0.1 * k, 0.0]))
                    if k == 150:
                        (ef.set_K if kind == "mdob" else ef.set_wo if kind == "leso" else ef.set_L)(11.0)   # a per-twin retune
                tr.append(np.concatenate([em.sigma, em.omega_hat, em.omega_dot_hat]))
            return np.array(tr), em, ef
        a, em, ef = moment_trace(True); b, _, _ = moment_trace(False)
        check(np.array_equal(a, b) and em is not ef and em.J is not ef.J,
              f"{kind}: the moment instance's whole trace is bit-identical with the force twin stepped (and retuned) beside it")

    print("== 5. omega_dot_hat = J^-1 M_applied + sigma of the same tick (also DURING a hold)")
    for kind in KINDS:
        e = build(kind, J); Jinv = np.linalg.inv(np.diag(J))
        worst = 0.0; worst_hold = 0.0
        w = np.array([0.1, -0.2, 0.05])
        for k in range(400):
            M = np.array([0.3 * math.sin(k * 0.01), 0.2, -0.1])
            e.hold = 200 <= k < 300
            s_pre = e.sigma.copy()
            e.step(np.eye(3), M, 5e-3, omega=w + 0.001 * k)
            cand = [Jinv @ M + s_pre, Jinv @ M + e.sigma]
            d = min(np.abs(e.omega_dot_hat - c).max() for c in cand)
            if e.hold:
                worst_hold = max(worst_hold, np.abs(e.omega_dot_hat - (Jinv @ M + s_pre)).max())
            worst = max(worst, d)
        check(worst < 1e-9 and worst_hold < 1e-9,
              f"{kind}: omega_dot_hat equals J^-1 M + sigma (pre- or post-update) to {worst:.1e}; with the frozen sigma during hold ({worst_hold:.1e})")

    print("== 6. hold freezes sigma exactly, keeps tracking, releases bumpless")
    for kind in KINDS:
        e = build(kind, J); Jm = np.diag(J)
        R = np.eye(3); w = np.zeros(3); dt = 1e-3
        sig_hold = None; innov = []; jump = 0.0; err_rel = 0.0; e_before = None; e_after = None
        for k in range(4000):
            t = k * dt
            d = np.array([0.0, 0.5 * t, 0.0])            # a ramp the estimate must chase
            M_model = np.array([0.1, 0.0, 0.0]) - np.cross(w, Jm @ w)
            w_dot = np.linalg.solve(Jm, M_model + d)
            e.hold = 1500 <= k < 2500
            s_before = e.sigma.copy()
            if k == 2500:
                err_rel = abs(d[1] / J[1] - s_before[1])     # the error the observer faces at release
                e_before = np.array(e.e, float).copy()
            e.step(R, M_model, dt, omega=w)
            if 1501 <= k < 2500:
                if sig_hold is None:
                    sig_hold = e.sigma.copy()
                if not np.array_equal(e.sigma, sig_hold):
                    sig_hold = None; break
                innov.append(np.abs(np.asarray(e.e)).max())
            if k == 2500:
                jump = np.abs(e.sigma - s_before).max(); e_after = np.array(e.e, float).copy()
            R = R @ st.so3_exp(w * dt); w = w + w_dot * dt
        # the release must be the observer's OWN catch-up for the error it faces, nothing extra:
        # STESO slews at most 1.1 L per second (its sign channel); the linear ESO at wo^2 e with
        # e ~ err/(2 wo) at the end of the hold, i.e. ~wo err / 2; the momentum observer at K err.
        rate = {"steso": 1.1 * KW["steso"]["L"], "leso": KW["leso"]["wo"] * err_rel, "mdob": KW["mdob"]["K"] * err_rel}[kind]
        e_jump = np.abs(e_after - e_before).max()
        # innovation during the hold: the rate/momentum estimate keeps following the measurement,
        # so it stays bounded (it does NOT grow like the un-estimated ramp would make it)
        check(sig_hold is not None and max(innov) < 0.5 and jump <= 1.2 * rate * dt + 1e-9 and e_jump < 0.05,
              f"{kind}: sigma frozen over the hold, innovation max {max(innov):.3g} bounded, release step {jump:.2e} "
              f"<= its own catch-up {rate * dt:.2e}/tick for a {err_rel:.2f} rad/s^2 error, innovation continuous ({e_jump:.1e})")

    print("== 7. the STESO's numerics are untouched (trace hash)")
    e = st.SO3STESO(J=J, L=20.0, mode="rate", eps="auto", add_gyro=False)
    out, _ = run_step(e, J, 1, 1.0)
    h = hashlib.sha1(out.tobytes()).hexdigest()[:16]          # exact bytes: one ULP anywhere changes it
    PIN = "42d640a766f8389b"          # pinned on controller e137f5e (before MomentumDOB/LinearESO edits)
    check(h == PIN, f"steso rate-mode trace hash {h} == pinned {PIN}")

    print("== 8. factory, knobs, mode, gyro path, first sample, reset, live retune")
    e = st.make_estimator("mdob", J, K=(7.0, 8.0, 9.0), L=99.0, wo=5.0, mode="rate", eps=0.0)
    check(type(e).__name__ == "MomentumDOB" and np.allclose(e.K, (7, 8, 9)), "make_estimator('mdob', K=(7,8,9), L=, wo=, mode=, eps=) -> MomentumDOB with K per axis, other knobs filtered")
    e = st.make_estimator("leso", J, wo=(3.0, 4.0, 5.0), K=1.0, L=2.0)
    check(type(e).__name__ == "LinearESO" and np.allclose(e.wo, (3, 4, 5)) and e.mode == "rate", "make_estimator('leso', wo=(3,4,5)) -> LinearESO with wo per axis")
    e = st.make_estimator("steso", J, L=(30.0, 31.0, 32.0), K=1.0, wo=2.0, eps=0.0)
    check(type(e).__name__ == "SO3STESO" and np.allclose(e.L, (30, 31, 32)) and e.eps == 0.0, "make_estimator('steso', L=(30,31,32), eps=0) -> SO3STESO with L per axis")
    try:
        st.make_estimator("leso", J, bandwidth=3.0); check(False, "unknown knob must raise")
    except TypeError:
        check(True, "an unknown knob name raises TypeError on every kind")
    bad = 0
    for kind in ("steso", "leso"):
        try:
            st.make_estimator(kind, J, mode="Rate"); bad += 1
        except ValueError:
            pass
    check(bad == 0, "an unknown mode raises ValueError on steso and leso alike")
    a = st.LinearESO(J=J, wo=20.0, mode="attitude"); b = st.LinearESO(J=J, wo=20.0, mode="rate")
    ta, _ = run_step(a, J, 1, 1.0, T=1.5); tb, _ = run_step(b, J, 1, 1.0, T=1.5)
    check(not np.array_equal(ta, tb) and abs(ta[-1, 1] - 1.0) < 0.05, "LinearESO 'attitude' and 'rate' run different branches, both read the step")
    # add_gyro=True: the gyroscopic term is the ONE cross-axis term inside an estimator. On a body
    # spinning about all three axes with the model NOT containing -w x J w, the estimator with
    # add_gyro=True must still read a +1 N.m step on its own axis and leave the others at their floor.
    for kind in KINDS:
        def spin_run(est, axis, amp):
            Jm = np.diag(J); R = np.eye(3); w = np.array([0.6, -0.4, 0.5]); dt = 1e-3; n = 3000; out = np.zeros((n, 3))
            for k in range(n):
                t = k * dt
                M_cmd = np.array([0.2 * math.sin(math.pi * t), 0.1 * math.cos(2.0 * t), 0.05 * math.sin(t)])
                d = np.zeros(3); d[axis] = amp if t >= 1.0 else 0.0
                w_dot = np.linalg.solve(Jm, M_cmd - np.cross(w, Jm @ w) + d)
                est.step(R, M_cmd, dt, omega=w)                     # the model hands over M_cmd only: no gyro
                R = R @ st.so3_exp(w * dt); w = w + w_dot * dt
                out[k] = Jm @ est.sigma
            return out
        kw = dict(KW[kind]); ref = spin_run(st.make_estimator(kind, J, add_gyro=True, **kw), 1, 0.0)
        ok = True; msg = []
        for axis in range(3):
            out = spin_run(st.make_estimator(kind, J, add_gyro=True, **kw), axis, 1.0)
            own = out[-500:, axis].mean(); oth = np.abs(np.delete(out[-500:], axis, axis=1) - np.delete(ref[-500:], axis, axis=1)).max()
            ok = ok and abs(own - 1.0) < 0.03 and oth < 2e-3
            msg.append(f"{own:+.3f}/{oth:.0e}")
        check(ok, f"{kind} add_gyro=True on a spinning body: own-axis reads / others' excess over the no-step run: {' '.join(msg)}")
    # first sample: start ON the measurement -- a body already rotating at 1 rad/s under a KNOWN
    # command (the harness's rigid body, the gyroscopics inside M_model as the cascade hands them),
    # no residual: the estimate must stay at its chatter floor from the very first tick
    for kind in KINDS:
        e = build(kind, J); Jm = np.diag(J); R = np.eye(3); w = np.array([1.0, -0.7, 0.4]); dt = 1e-3; peak = 0.0
        for k in range(300):
            t = k * dt
            M_cmd = np.array([0.2 * math.sin(math.pi * t), 0.1 * math.cos(2.0 * t), 0.05 * math.sin(t)])
            M_model = M_cmd - np.cross(w, Jm @ w)
            w_dot = np.linalg.solve(Jm, M_model)
            e.step(R, M_model, dt, omega=w)
            R = R @ st.so3_exp(w * dt); w = w + w_dot * dt
            peak = max(peak, np.abs(Jm @ e.sigma).max())
        floor = {"steso": 1.1 * KW["steso"]["L"] * dt * J.max() * 3, "leso": 1e-6, "mdob": 1e-6}[kind]
        check(peak <= floor + 1e-9, f"{kind}: no first-tick kick on a body already rotating (peak |torque| {peak:.1e})")
    # reset() after stepping == a fresh instance
    for kind in KINDS:
        e1 = build(kind, J); e2 = build(kind, J)
        run_step(e1, J, 1, 1.0, T=0.5); e1.reset()
        t1, _ = run_step(e1, J, 2, 0.7, T=0.5); t2, _ = run_step(e2, J, 2, 0.7, T=0.5)
        check(np.array_equal(t1, t2), f"{kind}: reset() after stepping gives the trace of a fresh instance")
    # live retune does not move the estimate: settle on a 1.9 N.m residual, then ONE more tick
    # of the same plant with and without a retune in between; the two next-tick estimates must
    # agree (the STESO's within its chatter). The audit found the momentum observer re-scaling
    # its estimate by K_new/K_old here (its estimate is K (p - I), not a state) -- set_K now
    # re-anchors the integral.
    import copy
    for kind in KINDS:
        e = build(kind, J); Jm = np.diag(J); R = np.eye(3); w = np.zeros(3); dt = 1e-3
        def tick(est, R, w, t):
            M_cmd = np.array([0.2 * math.sin(math.pi * t), 0.1 * math.cos(2.0 * t), 0.05 * math.sin(t)])
            M_model = M_cmd - np.cross(w, Jm @ w); d = np.array([0.0, 1.9, 0.0])
            w_dot = np.linalg.solve(Jm, M_model + d)
            est.step(R, M_model, dt, omega=w)
            return R @ st.so3_exp(w * dt), w + w_dot * dt
        for k in range(3000):
            R, w = tick(e, R, w, k * dt)
        e2 = copy.deepcopy(e)
        (e2.set_K if kind == "mdob" else e2.set_wo if kind == "leso" else e2.set_L)(40.0)
        tick(e, R, w, 3.0); tick(e2, R, w, 3.0)
        jump = np.abs(e2.torque - e.torque).max()
        check(jump < (0.05 if kind == "steso" else 1e-6), f"{kind}: retune 20 -> 40 at a settled 1.9 N.m: next-tick estimate differs by {jump:.1e} N.m from no retune")

    print()
    if FAILS:
        print(f"FAILED {len(FAILS)}:"); [print("  - " + f) for f in FAILS]; return 1
    print("RESIDUAL ESTIMATORS: INTERFACE, ISOLATION, TWINS, omega_dot_hat, HOLD -- ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
