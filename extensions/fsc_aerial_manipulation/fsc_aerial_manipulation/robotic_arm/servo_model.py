#!/usr/bin/env python3
"""OM-X (Dynamixel XM430-W350) servo model for the aerial-manipulator arm.

The arm flies in Dynamixel **Operating Mode 16 (PWM)**, so the number the arm
controller writes to the servo is a *duty*, i.e. a **voltage**, not a current.
The winding therefore sees ``V - Ke*qd`` across ``R`` rather than ``V``, and the
torque the motor actually makes falls by a fixed amount per unit of joint speed
no matter what torque was asked for::

    tau_applied,i = clip(tau_cmd,i, +-tau_cap,i)  -  b_i * (qd_i - s_emf * qd_ref,i)
    b_i = Kt_i^2 / R_i                                       [N.m per rad/s]

Isaac applies the commanded joint effort *exactly*, so without this term the
simulated arm is an ideal torque source and the whole-body law's arm-reaction
pre-compensation is exact by construction — which is precisely why Isaac has
never reproduced the hardware behaviour (see the 2026-09-03 report,
``docs/experimental_data_ros2_bag/0903 - T650-AM whole-body/analysis/``).

IDENTIFICATION (2026-09-03, from the 2026-09-02 12:46 and 2026-09-03 10:16
whole-body DIRECT bags; the report carries the plots and the residuals):

* the command chain ``duty = clip_+-885[ clip_+-max_effort(tau*N*rho)
  + s_emf*(885*Ke/Vs)*qd_ref ]`` with ``rho = 885*R*I_LSB/Vs`` reproduces the
  logged Goal PWM to <= 0.23 duty counts (1.8 mN.m) over 32 700 samples, so the
  torque the servo was TOLD to make is known exactly;
* at rest (|qd| < 0.005 rad/s) joints 2/3 deliver 99.4 / 94.0 % (3 Sep) and
  100.1 / 97.6 % (2 Sep) of that -- the arm is an honest torque source standing
  still, nothing needs a gain;
* in motion the drop is LINEAR and SYMMETRIC in qd, with slope (fitted above
  0.15 rad/s, where the velocity observer's ~0.02 rad/s noise no longer dilutes
  the regression) 0.950 on joint 2 and 1.409 / 1.476 on joint 3 -- against
  ``Kt^2/R`` = 0.934 / 1.493 with NO fitted parameter.

``Kt = Ke`` in SI units for any DC machine, so the single calibrated constant
``nm_to_effort_joints`` (counts per N.m, from the arm repo's 2026-08-17
campaign) fixes the droop as well as the torque. NOTE the arm controller's own
``back_emf_ke[2] = 2.86`` contradicts joint 2's calibrated ``Kt = 2.139`` by
34 %; that feedforward did nothing in these flights (it is driven by the
REFERENCE velocity, which whole-body DIRECT holds near zero) but it is wrong.

WHAT THIS MODEL DOES NOT CONTAIN:

* **joints 1 and 4 are not identified.** Their commanded torque (0.03-0.04 N.m
  rms) never left the current sensor's noise floor in either flight, and what
  little is measurable disagrees with theory (joint 1 fits 0.457 against 1.102,
  R^2 0.50). ``BACKEMF_JOINTS`` therefore defaults to joints 2 and 3 only.
* **gearbox friction.** ``tau_applied`` here is Kt x Present Current, the
  ELECTROMAGNETIC torque; the 353.5:1 gearbox's Coulomb and viscous losses sit
  downstream of it and the flight data cannot separate them (there is no torque
  sensor). A breakaway torque has to come from a bench measurement.
* **current-control mode.** In Mode 0 the servo closes its own current loop, R
  and Ke drop out and ``b`` goes to zero. If the arm ever moves to Mode 0 this
  model should be DELETED, not retuned.

Pure numpy -- no Isaac imports -- so it is testable standalone::

    /usr/bin/python3 servo_model.py            # self-test, replays the flights
"""

from __future__ import annotations

import numpy as np

# ── servo constants ─────────────────────────────────────────────────────────
# Calibrated counts per N.m (arm repo utils_calibration, 2026-08-17). The
# datasheet's 205-208.5 is 20-42 % off per joint -- do not substitute it.
NM_TO_COUNTS = np.array([160.0, 173.8, 146.7, 160.0])
# The arm controller's configured motor_resistance_ohm.
RESISTANCE_OHM = np.array([4.90, 4.90, 4.30, 4.90])
SUPPLY_V = 12.0            # supply_voltage_v
PWM_FULL_SCALE = 885.0     # pwm_full_scale
CURRENT_LSB_A = 0.00269    # current_lsb_a, the XM430 Present Current unit

# Torque constant implied by the calibration, N.m/A: [2.3234, 2.1389, 2.5341, 2.3234]
KT = 1.0 / (NM_TO_COUNTS * CURRENT_LSB_A)
# Back-EMF droop, N.m per rad/s: [1.1017, 0.9337, 1.4934, 1.1017]
B_BACKEMF = KT ** 2 / RESISTANCE_OHM

# Duty counts per N.m and per rad/s.
RHO = PWM_FULL_SCALE * RESISTANCE_OHM * CURRENT_LSB_A / SUPPLY_V
NM_TO_DUTY = NM_TO_COUNTS * RHO             # [155.54, 168.95, 125.14, 155.54]
KE_TO_DUTY = PWM_FULL_SCALE * KT / SUPPLY_V  # duty counts per rad/s of qd_ref

# ``max_effort`` in duty counts, and the same ceiling in N.m.
DUTY_CAP_AS_FLOWN = np.array([57.6117, 364.8743, 192.0391, 57.6117])
TAU_CAP_AS_FLOWN = DUTY_CAP_AS_FLOWN / NM_TO_DUTY   # [0.370, 2.160, 1.535, 0.370]
# The caps were raised to a uniform 3.0 N.m on 2026-09-04, with the servos' own
# PWM Limit re-sized to match, so this is the arm as it is TODAY.
TAU_CAP_CURRENT = np.full(4, 3.0)

# 0-based indices of the joints whose droop is identified from flight data.
BACKEMF_JOINTS = (1, 2)


class DynamixelPwmServo:
    """A PWM-mode OM-X joint set: a voltage source, not a torque source.

    Parameters
    ----------
    tau_cap : array-like or None
        Per-joint torque ceiling in N.m -- the ``max_effort`` duty cap expressed
        as a torque. ``TAU_CAP_AS_FLOWN`` reproduces the 2/3 Sep flights;
        ``TAU_CAP_CURRENT`` (the default) is the arm since 2026-09-04.
    backemf_joints : iterable of int
        Which joints carry the droop. Defaults to the two the flights identify.
        Pass ``range(4)`` to apply ``Kt^2/R`` everywhere (physically the same
        machine, but UNVERIFIED on joints 1 and 4).
    b : array-like or None
        Override the droop coefficients outright, N.m per rad/s. Only for
        sensitivity studies -- the default is first-principles.
    back_emf_ff_scale : float
        The arm controller's ``back_emf_ff_scale``. The feedforward cancels the
        droop at the REFERENCE velocity; pass ``qd_ref`` to ``applied()`` to
        reproduce it. Both flights ran at 1.0 with qd_ref ~ 0.
    """

    def __init__(self, tau_cap=None, backemf_joints=BACKEMF_JOINTS,
                 b=None, back_emf_ff_scale=1.0):
        self.tau_cap = (np.array(TAU_CAP_CURRENT, float) if tau_cap is None
                        else np.asarray(tau_cap, float).copy())
        if self.tau_cap.shape != (4,):
            raise ValueError(f"tau_cap must have 4 entries, got {self.tau_cap.shape}")
        if np.any(self.tau_cap <= 0.0):
            raise ValueError(f"tau_cap must be positive, got {self.tau_cap}")
        self.b = np.zeros(4) if b is None else np.asarray(b, float).copy()
        if b is None:
            for j in backemf_joints:
                if not 0 <= int(j) < 4:
                    raise ValueError(f"backemf_joints out of range: {backemf_joints}")
                self.b[int(j)] = B_BACKEMF[int(j)]
        elif self.b.shape != (4,):
            raise ValueError(f"b must have 4 entries, got {self.b.shape}")
        if np.any(self.b < 0.0):
            raise ValueError(f"b must be >= 0, got {self.b}")
        self.back_emf_ff_scale = float(back_emf_ff_scale)

    # ── the model ───────────────────────────────────────────────────────────

    def applied(self, tau_cmd, qdot, qdot_ref=None):
        """Torque the motor actually produces, N.m.

        ``tau_cmd`` is what the whole-body law asked for (the arm controller's
        own clamp is applied here), ``qdot`` the measured joint speed and
        ``qdot_ref`` the controller's reference velocity, whose droop the
        back-EMF feedforward cancels. All shape (4,).
        """
        tau = np.clip(np.asarray(tau_cmd, float), -self.tau_cap, self.tau_cap)
        v = np.asarray(qdot, float)
        if qdot_ref is not None:
            v = v - self.back_emf_ff_scale * np.asarray(qdot_ref, float)
        return tau - self.b * v

    def duty(self, tau_cmd, qdot_ref=None):
        """The Goal PWM count the arm controller would write. Diagnostics only."""
        d = np.clip(np.asarray(tau_cmd, float) * NM_TO_DUTY,
                    -self.tau_cap * NM_TO_DUTY, self.tau_cap * NM_TO_DUTY)
        if qdot_ref is not None:
            d = d + self.back_emf_ff_scale * KE_TO_DUTY * np.asarray(qdot_ref, float)
        return np.clip(d, -PWM_FULL_SCALE, PWM_FULL_SCALE)

    # ── numerics ────────────────────────────────────────────────────────────

    def explicit_stability_ratio(self, inertia, dt):
        """``b*dt/I`` per joint.

        The droop is applied as an explicit torque, so it is a damper integrated
        explicitly: stable while this stays below 2, and well-behaved below ~0.5.
        With the arm's reflected rotor inertia alone (armature 353.5^2 * 1.6e-7
        = 0.0200 kg.m^2) and the 250 Hz physics step this is 0.19-0.30, so the
        explicit form is safe here. If the physics step ever grows, move the
        droop into the PhysX drive damping instead, where it is implicit.
        """
        return self.b * float(dt) / np.asarray(inertia, float)

    def __repr__(self):
        return (f"DynamixelPwmServo(b={np.round(self.b, 4).tolist()} N.m/(rad/s), "
                f"tau_cap={np.round(self.tau_cap, 3).tolist()} N.m, "
                f"back_emf_ff_scale={self.back_emf_ff_scale})")


# ── self-test ───────────────────────────────────────────────────────────────

_BAGS = [
    ("3 Sep 10:16", "0903 - T650-AM whole-body/analysis/whole_body_test_20260903_101609.npz",
     6.65, 40.65, {1: 0.134, 2: 0.067}),
    ("2 Sep 12:46", "0902 - T650-AM whole-body/analysis/wb_tracking_20260902_124652.npz",
     6.44, 103.94, {1: 0.025, 2: 0.039}),
]


def _replay(npz_path, t0, t1, servo):
    """Model error against the measured Present Current, per joint, on a flight."""
    d = np.load(npz_path, allow_pickle=True)
    law, lt = d["law"], d["law_t"]
    t = np.arange(t0 + 0.30, t1 - 0.05, 0.004)
    on = lambda ts, x: np.stack([np.interp(t, ts, x[:, i]) for i in range(x.shape[1])], 1)
    duty = on(lt, law[:, 13:17])
    tau_goal = duty / NM_TO_DUTY                       # what the servo was told
    tau_app = on(d["js_t"], d["js_eff"][:, 2:6]) / NM_TO_COUNTS   # what it made
    qdot = on(d["vo_t"], d["vo_vel"])
    # One control cycle of transport lag on the measured side, as in the report.
    tau_goal, qdot, tau_app = tau_goal[:-1], qdot[:-1], tau_app[1:]
    sat = np.abs(duty[:-1]) >= DUTY_CAP_AS_FLOWN * 0.999
    out = {}
    for j in range(4):
        ok = ~sat[:, j]
        model = tau_goal[ok, j] - servo.b[j] * qdot[ok, j]
        out[j] = (float(np.std(tau_app[ok, j] - model)),
                  float(np.std(tau_app[ok, j] - tau_goal[ok, j])))
    return out


def _self_test():
    import os
    ok = True
    print("Kt        =", np.round(KT, 4), "N.m/A")
    print("b = Kt^2/R=", np.round(B_BACKEMF, 4), "N.m per rad/s")
    print("N*rho     =", np.round(NM_TO_DUTY, 3), "duty per N.m")
    print("tau cap   =", np.round(TAU_CAP_AS_FLOWN, 4), "N.m as flown,",
          np.round(TAU_CAP_CURRENT, 1), "N.m today")

    s = DynamixelPwmServo(tau_cap=TAU_CAP_AS_FLOWN)
    print("\n", s, sep="")
    assert np.allclose(s.b, [0.0, B_BACKEMF[1], B_BACKEMF[2], 0.0]), "default joint mask"
    # At rest the model is the identity, ceiling aside.
    tau = np.array([0.1, 0.5, -0.4, 0.05])
    assert np.allclose(s.applied(tau, np.zeros(4)), tau), "zero speed must be exact"
    # The feedforward cancels the droop exactly at the reference velocity.
    v = np.array([0.2, -0.5, 0.4, 0.1])
    assert np.allclose(s.applied(tau, v, qdot_ref=v), tau), "qd == qd_ref must cancel"
    # Sign reversal where the report says it is.
    q_rev = s.tau_cap[2] / s.b[2]
    assert s.applied(np.full(4, 10.0), np.array([0, 0, q_rev * 1.01, 0]))[2] < 0.0
    # The duty ceiling is the max_effort count, recovered.
    assert np.allclose(s.duty(np.full(4, 99.0)), DUTY_CAP_AS_FLOWN), "duty ceiling"
    # Explicit-integration margin at the armature inertia and 250 Hz.
    arm = 353.5 ** 2 * 1.6e-7
    r = s.explicit_stability_ratio(np.full(4, arm), 1.0 / 250.0)
    print(f"b*dt/I at the armature alone ({arm:.4f} kg.m^2, 250 Hz) = {np.round(r, 3)}")
    assert r.max() < 0.5, "explicit damping margin"

    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", "..", "..", "..",
                                        "docs", "experimental_data_ros2_bag"))
    for tag, rel, t0, t1, expect in _BAGS:
        p = os.path.join(root, rel)
        if not os.path.exists(p):
            print(f"\n{tag}: bag npz not present, replay skipped ({p})")
            continue
        res = _replay(p, t0, t1, s)
        print(f"\n{tag}: model error vs measured Present Current [N.m rms]")
        for j in range(4):
            mdl, ideal = res[j]
            mark = ""
            if j in expect:
                mark = "  OK" if abs(mdl - expect[j]) < 0.004 else f"  MISMATCH (want {expect[j]})"
                ok &= abs(mdl - expect[j]) < 0.004
            print(f"   joint {j+1}: model {mdl:.4f}   ideal-joint {ideal:.4f}{mark}")
    print("\nself-test", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_self_test())
