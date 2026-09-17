#!/usr/bin/env python3
"""OM-X (Dynamixel XM430-W350) servo model for the aerial-manipulator arm.

The arm commands Dynamixel **Operating Mode 16 (PWM)**, and since 2026-09-09 the
arm controller closes a **software current loop** around it: an integral trim on
the current error, added to the duty after the ``max_effort`` clamp, deliberately
slow at ``wc`` = 1.5 Hz so that the machine's own damping survives above the band
the trajectory lives in. Design, bring-up and bench validation:
``fsc_open_manipulator/doc/Current Loop Design.md``.

**With the loop closed the arm is a torque source again, to within a residual
scatter, and that residual is the whole of this model.** Bench-measured (that
doc, §9.6, ``doc/current_error_all.png``, and the 2026-09-11 sine session
recorded in ``external_torque_controller_hardware_aerial_pwm.yaml``):

| joint | trim   | current error, loop OFF | **as shipped** | torque delivered |
|-------|--------|-------------------------|----------------|------------------|
| j1    | OFF    | 11 mA rms *             | **11 mA** *    | (carries ~none)  |
| j2    | 1.5 Hz | 13-15 mA rms            | **4.2-4.6 mA** | 93.3 % -> 99.4 % |
| j3    | 1.5 Hz | 14-16 mA rms            | **6.7-6.9 mA** | 87.5 % -> 91.8 % |
| j4    | OFF    | 11 mA rms *             | **11 mA** *    | (carries ~none)  |

**The trim is per joint, ``[0.0, 1.5, 1.5, 0.0]`` Hz**: on j1/j4 the commanded
current is mostly noise, so the integral chases it and makes them WORSE
(11 -> 14 and 11 -> 17 mA), and the shipped config leaves them open-loop.

``*`` **j1/j4's own numbers are NOT used by this model, and must not be.** They
sit below their own current-sensor noise floor -- at stalled samples j4's
measured current correlates 0.58 with its applied duty against 0.97 on j2 -- so
they measure the SENSOR, not the motor. They take j3's figure instead; see
``CURRENT_NOISE_A``.

The **shape** is the result, not the magnitude: with the loop off both joints sat
on a persistent one-sided offset of about -20 mA and never crossed zero -- the
back-EMF droop, a bias, which is what corrupts a torque-controlled flight. With
the loop closed the error is centred and symmetric. So the plant model here is

    tau_applied,i = clip(tau_cmd,i, +-tau_cap,i)  +  Kt_i * i_err,i(t)

with ``i_err`` a zero-mean, band-limited random current error and **no velocity
term at all**. The pre-2026-09-09 back-EMF droop ``- Kt^2/R * qd`` is GONE from
this file: the loop removes it, and simulating a droop the flight arm no longer
has would be modelling last week's actuator.

WHAT THE NOISE IS, EXACTLY. ``current_error_all.png`` plots the **5 Hz content**
of ``I_measured - I_commanded``, so the rms values quoted above are of a signal
already low-passed at 5 Hz. It is reproduced here as an exact-discretization
AR(1) -- white noise through a first-order low pass at ``current_noise_bw_hz``,
scaled so the stationary rms is ``current_noise_a`` at ANY step size:

    a        = exp(-2*pi*fc*dt)
    i_err   <- a*i_err + sqrt(1 - a^2) * sigma * randn(4)

At the shipped ``fc`` = 5 Hz that puts exactly 1/2 the variance (0.71 of the
rms) below 5 Hz, so the measured in-band figures are converted to the total
``sigma`` this model wants by multiplying by sqrt(2): the two joints whose
residual is REAL, ``[4.4, 6.8] mA`` on j2/j3, become ``[6.2, 9.6] mA``, and
j1/j4 take j3's. The four channels are drawn independently -- four motors, four
sensors.

Until 2026-09-11 this was a single 7 mA on all four joints, because j1 and j4
had never been characterised. They have been now, and the answer was that the
measurement is below its own noise floor there -- so the treatment is unchanged
in substance and only j2 moves. **The choice that remains a judgement is the
SPEED**: these are the 10 deg/s numbers, which is the band the arm flies in.

THE DIGITAL COMMAND PATH is unchanged and still orthogonal to the above. The
number that reaches the servo is an **int16 Goal PWM register**, so between the
law and the winding sit:

1. **quantization**, which TRUNCATES TOWARD ZERO rather than rounding
   (``dynamixel_info.hpp``'s ``ConvertUnitToValue`` ends in
   ``static_cast<T>(adjusted_value / unit)``). The loss is systematic, up to a
   full count (5.9 / 6.7 / 7.4 / 6.7 mN.m per joint), never a gain, and any
   command under one count delivers exactly zero -- a real dead zone;
2. **the +-885 rail**, which the torque term alone cannot reach at a 3.0 N.m cap
   (508 / 449 / 406 / 446 counts);
3. **calibration error**, the arm's analogue of the allocator's ``kf`` mismatch.
   The chain converts N.m to counts with what it BELIEVES
   (``nm_to_counts_nominal``); the winding then makes torque per count according
   to what is TRUE (``nm_to_counts_true``). Delivered torque is scaled by
   ``nominal/true``.

Both default OFF, so a plain ``DynamixelPwmServo()`` is the clamp plus the
current-loop residual and nothing else.

A CONSEQUENCE WORTH KNOWING: a pure calibration error is **invisible in
``joint_states.effort``**. The servo reports Present Current, which the driver
scales back to N.m with the same NOMINAL constant, so the readback shows very
nearly the COMMANDED torque while the joint is really making ``nominal/true``
times it (``sense()`` reproduces this). The residual current error, by contrast,
IS in Present Current and so IS visible there -- it is measured from exactly that
signal.

GEARBOX FRICTION (2026-09-14, user request: the calibration report's friction
and gravity compensation are the dominant terms, so the plant must carry the
friction the arm controller compensates, with a controllable mismatch).
``tau_applied`` above is Kt x Present Current, the ELECTROMAGNETIC torque; the
353.5:1 gearbox's losses sit downstream of it. The calibration campaign
identified them (``fsc_open_manipulator/doc/Calibration Result for PWM Torque
Control.pdf`` S1.5, eq. 5, the antisymmetric part of the inverse-dynamics
residual over 3 poses x 5 speeds per joint):

    tau_f = [fc + mu * |tau_load|] * sgn(qd) + fv * qd

with, in the controller's duty counts, ``fc`` = [2.9, 4.705, 7.778, 7.778],
``mu`` = [0, 0.246, 0.161, 0] and ``fv`` shipped 0 (the report's own choice:
j3's measured viscous rise is reproduced by the tanh shape). The plant applies
the SAME shape the controller pays, ``tanh(qd / w)`` at the same width, on the
MEASURED velocity and the TRANSMITTED (clipped) torque, scaled by
``friction_scale``:

    tau_applied -= friction_scale * ([fc + mu * |tau|] * tanh(qd / w) + fv * qd)

so ``friction_scale`` = 1.05 is a plant with 5 % MORE friction than the
controller's feed-forward pays -- the "imperfect compensation" under test --
and 0 (the default) is the frictionless plant every earlier campaign flew.
Two things are deliberately NOT the report's: (a) the plant's friction is a
function of the actual ``qd`` where the compensation is driven by the reference
``qd_d``, so in a transient the two differ by more than the scale (that is
real, and it is what hardware does); (b) STICTION is not modelled -- ``tanh``
is exactly zero at rest, so there is no breakaway and nothing for a dither to
break, which is why the sim compensation runs with ``dither_amplitude`` 0.

NUMERICS. Friction is applied EXPLICITLY as a torque on a joint whose smallest
possible effective inertia is the armature, 0.0200 kg m^2, at 250 Hz. A
Coulomb term steeper than ``I/dt`` near zero velocity would flip the joint's
velocity sign inside one step and inject energy (the ``b*dt/I < 2`` rule the
back-EMF droop recorded), and ``mu*|tau|`` at the 3 N.m clamp is 8x too steep
for that. So the applied friction is MOMENTUM-CLAMPED: it may never remove
more than the joint's own momentum in one step, ``|tau_f| <= I_min*|qd|/dt``.
Near rest that is a linear ramp reaching the Coulomb level at
``(fc + mu|tau|)*dt/I_min`` (0.04 rad/s on a loaded j2, 0.01 on j4), which
is also the physically sensible "stick": the joint stops, it does not
oscillate. ``_self_test`` spins an isolated inertia down under it and asserts
the velocity never reverses.

WHAT THIS MODEL DOES NOT CONTAIN:

* **stiction / breakaway.** See above: ``tanh`` is zero at rest. A breakaway
  torque has to come from a bench measurement, and the report's own escape
  mechanism for it (the gated dither) therefore has nothing to do in sim.
* **the current loop's own dynamics.** Only its steady outcome is modelled. The
  loop is a 1.5 Hz integrator, so a torque command stepping faster than that is
  briefly delivered at the open-loop gain (0.85 incremental) before the trim
  catches up. Whole-body DIRECT commands change slowly compared with 1.5 Hz.
* **anything above 5 Hz.** The measurement the noise is scaled from was filtered
  there; content above it exists on the real arm and is not characterised.
* **current-control mode (Dynamixel Mode 0).** There the servo closes its own
  current loop in firmware at FOC rates and the residual would be smaller again.

Pure numpy -- no Isaac imports -- so it is testable standalone::

    /usr/bin/python3 servo_model.py            # self-test
"""

from __future__ import annotations

import numpy as np

# ── servo constants ─────────────────────────────────────────────────────────
# Counts per N.m, the arm repo's ``nm_to_effort_joints``. CALIBRATED 2026-09-11
# by the lever/RLS campaign (doc/"Motor Constant Calibration.md"), superseding
# the 2026-08-17 set [160.0, 173.8, 146.7, 160.0] in which j2 was fragile
# (+-15 %) and j1/j4 had never been measured at all. The j1 and j4 UNITS were
# calibrated by swapping them into the j2/j3 brackets, where gravity gives them
# a lever. The datasheet's 205-208.5 is 20-42 % off per joint -- do not
# substitute it. Believed to +-2 %: the campaign found kappa CONDITIONING-
# limited, not measurement-limited, and its own formal +-0.5 % is not achieved.
NM_TO_COUNTS = np.array([162.4, 154.0, 150.5, 153.4])
# The arm controller's configured motor_resistance_ohm, MEASURED per unit
# 2026-09-11 (was [4.90, 4.90, 4.30, 4.90]). Still here because the duty scale
# RHO carries it; nothing else uses it now that the droop is gone. R rises
# ~0.4 %/degC and does not repeat between sessions -- a tuning knob, not a
# constant.
RESISTANCE_OHM = np.array([5.26, 4.90, 4.53, 4.88])
SUPPLY_V = 12.0            # supply_voltage_v
PWM_FULL_SCALE = 885.0     # pwm_full_scale
CURRENT_LSB_A = 0.00269    # current_lsb_a, the XM430 Present Current unit

# Torque constant implied by the calibration, N.m/A: [2.2891, 2.4139, 2.4701,
# 2.4234]. CROSS-CHECK, and the reason to trust this set: against the same
# session's MEASURED back-EMF constants Ke = [2.427, 2.559, 2.623, 2.570] V per
# rad/s these give an efficiency eta = Kt/Ke of [0.943, 0.943, 0.942, 0.943] --
# four independent motors agreeing to 0.1 % on a sensible gearbox loss. The
# superseded set implied eta > 1 on j2, which is physically impossible and is
# what opened the campaign. ``_selftest`` asserts this.
KT = 1.0 / (NM_TO_COUNTS * CURRENT_LSB_A)
KE_MEASURED = np.array([2.427, 2.559, 2.623, 2.570])   # V/(rad/s), 2026-09-11

# Duty counts per N.m. THE CONTROLLER LOGS THIS EXACT VECTOR at configure, and
# on hardware 2026-09-11 it logged 169.5 / 149.7 / 135.3 / 148.5 -- which is
# what ties this file's constants to the arm actually flying.
RHO = PWM_FULL_SCALE * RESISTANCE_OHM * CURRENT_LSB_A / SUPPLY_V
NM_TO_DUTY = NM_TO_COUNTS * RHO             # [169.47, 149.70, 135.25, 148.51]

# ── the current loop's residual, PER JOINT ─────────────────────────────────
# THE LOOP IS NOT ON EVERY JOINT. ``current_loop_bandwidth_hz_joints`` ships as
# [0.0, 1.5, 1.5, 0.0]: the trim helps the two loaded joints and HURTS j1/j4,
# whose commanded current is mostly noise, so the integral chases it (bench
# 2026-09-11: j1 rms 11 -> 14 mA, j4 11 -> 17). The plant therefore carries the
# LOOP-CLOSED residual on j2/j3 and the LOOP-OPEN one on j1/j4.
#
#   joint    trim      measured rms of I_meas - I_cmd, 5 Hz band, at 10 deg/s
#   j1       off       11 mA        -- BELOW ITS OWN SENSOR NOISE FLOOR, see below
#   j2       1.5 Hz    4.2-4.6 mA   (doc/"Current Loop Design.md" 9.6)
#   j3       1.5 Hz    6.7-6.9 mA   (same; the 09-11 sine session read 5)
#   j4       off       11 mA        -- BELOW ITS OWN SENSOR NOISE FLOOR
#
# 10 deg/s is the column that matters -- the arm flies below it, and the
# campaign found its own friction model inadequate there, which is one more
# reason not to model this band from theory. Where two sessions disagree the
# WORSE number is taken.
#
# These are rms of an already-5-Hz-low-passed signal, while the constant below
# is the noise's TOTAL rms. A first-order low pass puts exactly half its
# variance below its own corner, so total = sqrt(2) x in-band.
#
# ** j1 AND j4 DO NOT TAKE THEIR OWN MEASUREMENT, AND THAT IS THE WHOLE POINT **
# (arm repo 612775a, 2026-09-11). Their 11 mA is NOT a torque: at stalled
# samples, where I = V/R must hold exactly, j4's measured current correlates
# only **0.58** with its applied duty at any filter time constant, against
# **0.97 on j2** -- the residual is 7.6 counts against a signal of similar
# size, because j4 runs at a few current counts where j2 runs at 100+. The arm
# repo's conclusion is "stop quoting a metric below its own noise floor", and
# it is what explains three failed fixes there (the trim made j4 WORSE, the
# dither did nothing, only ~20 % of the reversal excess was ever friction).
# Injecting 11 mA of APPLIED TORQUE on those two joints would be modelling the
# CURRENT SENSOR, not the motor.
#
# So j1/j4 take j3's figure -- the worse of the two joints whose residual IS
# real -- exactly as this model did before 2026-09-11, when the reason was
# "uncharacterised" rather than "characterised and found to be below the noise
# floor". It is deliberately CONSERVATIVE: a duty/R error scales with duty, and
# j1/j4 command ~500x less of it than j2 (0.2 vs 108 LSB at the steady hold),
# so their real torque error is probably far SMALLER than this.
#   2026-09-11 morning, WRONG and shipped for one campaign: [15.6, 6.2, 9.6,
#   15.6], which took j1/j4's own 11 mA at face value and made the two
#   untrimmed joints the noisiest in the model.
CURRENT_NOISE_A = np.array([0.0096, 0.0062, 0.0096, 0.0096])
CURRENT_NOISE_BW_HZ = 5.0      # [Hz] first-order corner; the figure's own band
# Torque noise this implies, N.m rms: [0.0220, 0.0150, 0.0237, 0.0233]
TORQUE_NOISE_NM = KT * CURRENT_NOISE_A

# ``max_effort`` in duty counts, and the same ceiling in N.m, as flown 2/3 Sep.
# The COUNTS are unchanged; their N.m equivalent moved with the calibration
# (it was [0.370, 2.160, 1.535, 0.370] under the superseded constants).
DUTY_CAP_AS_FLOWN = np.array([57.6117, 364.8743, 192.0391, 57.6117])
TAU_CAP_AS_FLOWN = DUTY_CAP_AS_FLOWN / NM_TO_DUTY   # [0.340, 2.437, 1.420, 0.388]
# The caps were raised to a uniform 3.0 N.m on 2026-09-04, with the servos' own
# PWM Limit re-sized to match, so this is the arm as it is TODAY.
TAU_CAP_CURRENT = np.full(4, 3.0)

# ── gearbox friction, the calibration report's eq. (5) ─────────────────────
# In the controller's DUTY COUNTS, exactly as
# external_torque_controller_hardware_aerial_pwm.yaml carries them
# (friction_ff / friction_load_coeff / viscous_ff / friction_ff_width), so the
# plant and the compensation are one set of numbers. N.m through NM_TO_DUTY.
FRICTION_FC_DUTY = np.array([2.9, 4.705, 7.7776, 7.7776])   # Coulomb, duty
FRICTION_FC_NM = FRICTION_FC_DUTY / NM_TO_DUTY   # [0.01711, 0.03143, 0.05751, 0.05237]
FRICTION_MU = np.array([0.0, 0.246, 0.161, 0.0])        # x |transmitted torque|
FRICTION_FV_NM_S = np.zeros(4)                           # viscous, N.m per rad/s
FRICTION_WIDTH_RAD_S = 0.015                             # tanh half-width, = the compensation's
# Smallest effective inertia a joint can present: the armature 06 authors,
# 353.5^2 * 1.6e-7 = 0.0200 kg m^2 (the link adds to it). The momentum clamp
# uses this, so it is conservative for every pose.
ARMATURE_KG_M2 = 353.5 ** 2 * 1.6e-7


class DynamixelPwmServo:
    """An OM-X joint set with the software current loop closed.

    Parameters
    ----------
    tau_cap : array-like or None
        Per-joint torque ceiling in N.m -- the ``max_effort`` duty cap expressed
        as a torque. ``TAU_CAP_CURRENT`` (the default) is the arm since
        2026-09-04; ``TAU_CAP_AS_FLOWN`` is the 2/3 Sep configuration.
    current_noise_a : float or array-like
        Stationary rms of the residual current error, amps. Scalar broadcasts to
        all four joints. ``0.0`` makes the servo an exact torque source (the
        ``ideal`` A/B).
    current_noise_bw_hz : float or array-like
        First-order corner of that noise, Hz. ``0.0`` freezes it, which with a
        zero initial state is another way of switching it off.
    seed : int or None
        Seed for the noise generator. An int makes a run reproducible; ``None``
        draws from the OS entropy pool.
    quantize : bool
        Route the command through the int16 Goal PWM register: truncate toward
        zero (as the driver's ``static_cast`` does) and rail at
        ``pwm_full_scale``. Default False.
    nm_to_counts_nominal, nm_to_counts_true : array-like or None
        Counts per N.m -- the same quantity the arm repo calls
        ``nm_to_effort_joints`` -- as BELIEVED by the command chain and as
        actually TRUE of the winding. Both default to the calibrated
        ``NM_TO_COUNTS``; making them differ injects a calibration error and
        scales delivered torque by ``nominal/true``.
    pwm_full_scale : float
        The Goal PWM rail, counts.
    """

    def __init__(self, tau_cap=None, current_noise_a=CURRENT_NOISE_A,
                 current_noise_bw_hz=CURRENT_NOISE_BW_HZ, seed=0,
                 quantize=False, nm_to_counts_nominal=None,
                 nm_to_counts_true=None, pwm_full_scale=PWM_FULL_SCALE,
                 friction_scale=0.0, friction_width=FRICTION_WIDTH_RAD_S,
                 friction_fc_nm=None, friction_mu=None, friction_fv=None,
                 inertia_min=ARMATURE_KG_M2):
        self.tau_cap = (np.array(TAU_CAP_CURRENT, float) if tau_cap is None
                        else np.asarray(tau_cap, float).copy())
        if self.tau_cap.shape != (4,):
            raise ValueError(f"tau_cap must have 4 entries, got {self.tau_cap.shape}")
        if np.any(self.tau_cap <= 0.0):
            raise ValueError(f"tau_cap must be positive, got {self.tau_cap}")

        # ── the current-loop residual ───────────────────────────────────────
        def _four(v, what, lo_ok=True):
            a = np.broadcast_to(np.asarray(v, float), (4,)).astype(float).copy()
            if not np.all(np.isfinite(a)) or (not lo_ok and np.any(a < 0.0)):
                raise ValueError(f"{what} must be finite and >= 0, got {v}")
            return a

        self.current_noise_a = _four(current_noise_a, "current_noise_a", False)
        self.current_noise_bw_hz = _four(current_noise_bw_hz,
                                         "current_noise_bw_hz", False)
        if np.any(self.current_noise_a < 0.0):
            raise ValueError(f"current_noise_a must be >= 0, got {current_noise_a}")
        if np.any(self.current_noise_bw_hz < 0.0):
            raise ValueError(f"current_noise_bw_hz must be >= 0, got "
                             f"{current_noise_bw_hz}")
        self.seed = seed
        self._rng = np.random.default_rng(seed)
        self._i_err = np.zeros(4)

        # ── the digital command path ────────────────────────────────────────
        def _counts(v, what):
            c = (np.array(NM_TO_COUNTS, float) if v is None
                 else np.asarray(v, float).copy())
            if c.shape != (4,):
                raise ValueError(f"{what} must have 4 entries, got {c.shape}")
            if not np.all(np.isfinite(c)) or np.any(c <= 0.0):
                raise ValueError(f"{what} must be finite and > 0, got {c}")
            return c

        self.quantize = bool(quantize)
        self.nm_to_counts_nominal = _counts(nm_to_counts_nominal,
                                            "nm_to_counts_nominal")
        self.nm_to_counts_true = _counts(nm_to_counts_true, "nm_to_counts_true")
        self.pwm_full_scale = float(pwm_full_scale)
        if not np.isfinite(self.pwm_full_scale) or self.pwm_full_scale <= 0.0:
            raise ValueError(f"pwm_full_scale must be > 0, got {pwm_full_scale}")
        # Duty counts per N.m on each side of the register.
        self.nm_to_duty_nominal = self.nm_to_counts_nominal * RHO
        self.nm_to_duty_true = self.nm_to_counts_true * RHO
        # Delivered torque per commanded torque, quantization aside.
        self.gain_error = self.nm_to_counts_nominal / self.nm_to_counts_true
        # The winding's own torque constant: a current error becomes torque
        # through what is TRUE, not through what the chain believes. Reduces to
        # KT exactly when the two calibrations match.
        self.kt_true = 1.0 / (self.nm_to_counts_true * CURRENT_LSB_A)
        # True if anything makes the digital path differ from a plain clamp;
        # when False `applied()` takes the closed form.
        self.digital = self.quantize or not np.allclose(
            self.gain_error, 1.0, rtol=0.0, atol=0.0)

        # ── gearbox friction (module docstring) ─────────────────────────────
        self.friction_scale = float(friction_scale)
        self.friction_width = float(friction_width)
        if (not np.isfinite(self.friction_scale) or self.friction_scale < 0.0
                or not np.isfinite(self.friction_width) or self.friction_width <= 0.0):
            raise ValueError(f"friction_scale must be >= 0 and friction_width > 0, "
                             f"got {friction_scale}, {friction_width}")
        self.friction_fc = (np.array(FRICTION_FC_NM, float) if friction_fc_nm is None
                            else _four(friction_fc_nm, "friction_fc_nm", False))
        self.friction_mu = (np.array(FRICTION_MU, float) if friction_mu is None
                            else _four(friction_mu, "friction_mu", False))
        self.friction_fv = (np.array(FRICTION_FV_NM_S, float) if friction_fv is None
                            else _four(friction_fv, "friction_fv", False))
        self.inertia_min = float(inertia_min)
        if not np.isfinite(self.inertia_min) or self.inertia_min <= 0.0:
            raise ValueError(f"inertia_min must be > 0, got {inertia_min}")

    # ── the model ───────────────────────────────────────────────────────────

    def applied(self, tau_cmd, dt, qdot=None):
        """Torque that reaches the joint over the next ``dt``, N.m.

        ``tau_cmd`` is what the whole-body law asked for (the arm controller's
        own clamp is applied here), shape (4,). ``dt`` is the step, seconds --
        it advances the noise state, so call this **once per physics step** and
        no more. ``qdot`` is the MEASURED joint velocity, rad/s; it is needed
        only when ``friction_scale`` > 0 (the gearbox loss is a function of
        the joint's actual motion), and omitting it then is an error rather
        than a silent frictionless step.
        """
        tau = np.clip(np.asarray(tau_cmd, float), -self.tau_cap, self.tau_cap)
        if self.digital:
            # The register is the plant boundary: the chain converts to counts
            # with the NOMINAL calibration, the winding makes torque per count
            # with the TRUE one.
            tau_e = self.duty(tau) / self.nm_to_duty_true
        else:
            tau_e = tau
        out = tau_e + self.kt_true * self.step_noise(dt)
        if self.friction_scale > 0.0:
            if qdot is None:
                raise ValueError("applied(): qdot is required when friction_scale > 0")
            out = out - self.friction_torque(tau, qdot, dt)
        return out

    def friction_torque(self, tau_transmitted, qdot, dt):
        """The gearbox loss for this step, N.m, SIGNED with ``qdot`` (subtract it).

        ``friction_scale * ([fc + mu*|tau|] * tanh(qd/w) + fv*qd)``, then
        momentum-clamped to ``inertia_min*|qd|/dt`` so an explicit application
        can never reverse the joint (module docstring, NUMERICS).
        """
        qd = np.asarray(qdot, float)
        load = np.abs(np.asarray(tau_transmitted, float))
        f = self.friction_scale * ((self.friction_fc + self.friction_mu * load)
                                   * np.tanh(qd / self.friction_width)
                                   + self.friction_fv * qd)
        cap = self.inertia_min * np.abs(qd) / float(dt)
        return np.clip(f, -cap, cap)

    def friction_coulomb_nm(self, tau_transmitted=0.0):
        """Full-speed friction level per joint at a given load, N.m (diagnostic)."""
        load = np.abs(np.broadcast_to(np.asarray(tau_transmitted, float), (4,)))
        return self.friction_scale * (self.friction_fc + self.friction_mu * load)

    def step_noise(self, dt):
        """Advance the residual current error by ``dt`` and return it, amps.

        Exact discretization of a first-order low pass driven by white noise, so
        the stationary rms is ``current_noise_a`` at any step size -- unlike an
        Euler form, whose variance would scale with ``dt`` and silently retune
        the plant if the physics step ever changed. (The same lesson
        ``lagged_thrust_curve.py`` records for the rotor lag.)
        """
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"dt must be finite and > 0, got {dt}")
        a = np.exp(-2.0 * np.pi * self.current_noise_bw_hz * dt)
        self._i_err = (a * self._i_err
                       + np.sqrt(np.maximum(1.0 - a * a, 0.0))
                       * self.current_noise_a * self._rng.standard_normal(4))
        return self._i_err

    def reset(self, seed=None):
        """Zero the noise state; re-seed if a seed is given."""
        self._i_err = np.zeros(4)
        if seed is not None:
            self.seed = seed
            self._rng = np.random.default_rng(seed)

    @property
    def i_err(self):
        """The current residual now, amps -- diagnostics only."""
        return self._i_err.copy()

    def duty(self, tau_cmd):
        """The Goal PWM count the arm controller writes, counts.

        ``clip_+-885[ clip_+-max_effort(tau * N * rho) ]`` and then, if
        ``quantize``, the driver's truncation toward zero into the int16
        register. The current loop's own trim is added to the duty on the real
        controller and is NOT reproduced here -- its outcome is, as the noise.
        """
        cap = self.tau_cap * self.nm_to_duty_nominal
        d = np.clip(np.asarray(tau_cmd, float) * self.nm_to_duty_nominal,
                    -cap, cap)
        d = np.clip(d, -self.pwm_full_scale, self.pwm_full_scale)
        return np.trunc(d) if self.quantize else d

    def sense(self, tau_applied):
        """What ``joint_states.effort`` reports for an applied torque, N.m.

        The servo measures Present Current in ``CURRENT_LSB_A`` counts and the
        driver scales it back with the NOMINAL calibration, so a CALIBRATION
        error very nearly cancels here: the readback shows the commanded torque
        while the joint makes ``gain_error`` times it. The current-error NOISE
        does not cancel -- it lives in Present Current, which is the signal it
        was measured from. Rounding, not truncation -- an ADC picks the nearest
        code. Diagnostics: nothing in the control loop consumes effort.
        """
        counts = np.round(np.asarray(tau_applied, float) * self.nm_to_counts_true)
        return counts / self.nm_to_counts_nominal

    def torque_noise_nm(self):
        """Stationary rms of the torque noise, N.m per joint."""
        return self.kt_true * self.current_noise_a

    def quantum_nm(self):
        """One Goal PWM count, in N.m of commanded torque. [6.43 5.92 7.99 6.43]"""
        return 1.0 / self.nm_to_duty_nominal

    def __repr__(self):
        return (f"DynamixelPwmServo(current_noise="
                f"{np.round(self.current_noise_a * 1e3, 2).tolist()} mA rms @ "
                f"{np.round(self.current_noise_bw_hz, 2).tolist()} Hz -> "
                f"{np.round(self.torque_noise_nm() * 1e3, 1).tolist()} mN.m, "
                f"tau_cap={np.round(self.tau_cap, 3).tolist()} N.m, "
                f"quantize={self.quantize}, "
                f"gain_error={np.round(self.gain_error, 4).tolist()}, "
                f"friction x{self.friction_scale:g}"
                + (f" [fc {np.round(self.friction_fc * 1e3, 1).tolist()} mN.m, "
                   f"mu {self.friction_mu.tolist()}, w {self.friction_width:g} rad/s]"
                   if self.friction_scale > 0.0 else "") + ")")


# ── self-test ───────────────────────────────────────────────────────────────

def _self_test():
    ok = True
    dt = 1.0 / 250.0
    print("Kt          =", np.round(KT, 4), "N.m/A")
    print("N*rho       =", np.round(NM_TO_DUTY, 3), "duty per N.m")
    print("noise       =", CURRENT_NOISE_A * 1e3, "mA rms @",
          CURRENT_NOISE_BW_HZ, "Hz ->",
          np.round(TORQUE_NOISE_NM * 1e3, 2), "mN.m rms")
    print("tau cap     =", np.round(TAU_CAP_AS_FLOWN, 4), "N.m as flown,",
          np.round(TAU_CAP_CURRENT, 1), "N.m today")

    # (0) THE CALIBRATION'S OWN CONSISTENCY CHECK, and the cheapest guard this
    # file has against a mistyped constant: eta = Kt/Ke is a gearbox
    # efficiency, so it must be <= 1, and the 2026-09-11 campaign measured the
    # four motors agreeing on it to 0.1 %. The superseded constants gave j2
    # eta = 1.13, which is what opened the campaign -- this assert would have
    # caught it.
    eta = KT / KE_MEASURED
    print("eta = Kt/Ke =", np.round(eta, 4), "(gearbox efficiency, must be <= 1)")
    assert np.all(eta <= 1.0), f"eta > 1 is physically impossible: {eta}"
    assert np.ptp(eta) < 0.01, f"eta must agree across units, got {eta}"
    # The controller logs NM_TO_DUTY at configure; hardware read this on
    # 2026-09-11. If this fires, this file and the flying arm disagree.
    assert np.allclose(NM_TO_DUTY, [169.5, 149.7, 135.3, 148.5], atol=0.05), \
        f"NM_TO_DUTY {NM_TO_DUTY} != the arm controller's logged constants"

    s = DynamixelPwmServo()
    print("\n", s, sep="")

    # (a) the noise is ZERO-MEAN and its rms is the configured one, at the
    # configured step and at a 4x different one -- the exact-discretization
    # property that an Euler form would not have.
    print("\n--- current-loop residual ---")
    for step in (dt, 4.0 * dt):
        t = DynamixelPwmServo(seed=1)
        n = 400_000
        x = np.empty((n, 4))
        for k in range(n):
            x[k] = t.step_noise(step)
        rms, mean = x.std(axis=0), x.mean(axis=0)
        print(f"dt = {step*1e3:5.1f} ms: rms = {np.round(rms*1e3, 3).tolist()} mA, "
              f"mean = {np.round(mean*1e3, 4).tolist()} mA")
        bad = np.max(np.abs(rms / CURRENT_NOISE_A - 1.0))
        ok &= bad < 0.02
        assert bad < 0.02, f"rms must be dt-invariant, off by {bad:.3f}"
        assert np.max(np.abs(mean)) < 0.2e-3, "noise must be zero-mean"

    # (b) the corner is where it was asked for: the lag-1 autocorrelation of an
    # AR(1) is exp(-2*pi*fc*dt) by construction, so recover fc from the data.
    t = DynamixelPwmServo(seed=2)
    n = 400_000
    x = np.empty((n, 4))
    for k in range(n):
        x[k] = t.step_noise(dt)
    r1 = np.array([np.corrcoef(x[:-1, j], x[1:, j])[0, 1] for j in range(4)])
    fc = -np.log(r1) / (2.0 * np.pi * dt)
    print(f"recovered corner = {np.round(fc, 2).tolist()} Hz "
          f"(configured {CURRENT_NOISE_BW_HZ})")
    ok &= np.max(np.abs(fc / CURRENT_NOISE_BW_HZ - 1.0)) < 0.03
    # In-band content: what the 5 Hz-filtered bench figure would have shown.
    print(f"torque noise = {np.round(x.std(axis=0)*KT*1e3, 2).tolist()} mN.m rms, "
          f"of which below {CURRENT_NOISE_BW_HZ:g} Hz "
          f"~{np.round(0.7071*x.std(axis=0)*1e3, 2).tolist()} mA "
          f"(bench, in-band, j2/j3 only: 4.2-4.6 / 6.7-6.9 mA)")

    # (c) sigma = 0 is an EXACT torque source -- the `ideal` A/B must be exact,
    # not merely quiet.
    z = DynamixelPwmServo(current_noise_a=0.0)
    tau = np.array([0.1, 0.5, -0.4, 0.05])
    for _ in range(100):
        assert np.array_equal(z.applied(tau, dt), tau), "sigma=0 must be exact"
    print("sigma = 0 reproduces the commanded torque exactly")

    # (d) NO velocity dependence anywhere: the droop is gone, and the only way
    # two calls differ is the noise draw.
    a = DynamixelPwmServo(seed=7).applied(tau, dt)
    b = DynamixelPwmServo(seed=7).applied(tau, dt)
    assert np.array_equal(a, b), "same seed must reproduce"
    assert not np.array_equal(a, DynamixelPwmServo(seed=8).applied(tau, dt)), \
        "different seeds must differ"
    print("seeding reproduces a run exactly")

    # (e) the clamp still binds, and it binds on the COMMAND.
    c = DynamixelPwmServo(tau_cap=TAU_CAP_AS_FLOWN, current_noise_a=0.0)
    assert np.allclose(c.applied(np.full(4, 99.0), dt), TAU_CAP_AS_FLOWN)
    assert np.allclose(c.duty(np.full(4, 99.0)), DUTY_CAP_AS_FLOWN), "duty ceiling"
    print("clamp and duty ceiling recovered")

    # ── the digital command path (unchanged, still orthogonal) ──────────────
    print("\n--- digital command path ---")
    rng = np.random.default_rng(0)
    # (f) EXACT reduction: the duty path with no quantization and a matched
    # calibration must reproduce the plain clamp to 0.0, not merely close.
    ref = DynamixelPwmServo(tau_cap=TAU_CAP_AS_FLOWN, current_noise_a=0.0)
    dig = DynamixelPwmServo(tau_cap=TAU_CAP_AS_FLOWN, current_noise_a=0.0,
                            quantize=False, nm_to_counts_true=NM_TO_COUNTS)
    assert dig.digital is False, "matched + no quantize must take the closed form"
    dig.digital = True                      # force the duty path for the proof
    worst = 0.0
    for _ in range(2000):
        tc = rng.uniform(-4, 4, 4)
        worst = max(worst, float(np.max(np.abs(
            dig.applied(tc, dt) - ref.applied(tc, dt)))))
    print(f"duty path vs clamp, matched + unquantized: max |diff| = {worst:.3e} N.m")
    ok &= worst < 1e-12
    assert worst < 1e-12, f"digital path must reduce to round-off, got {worst}"

    # (g) quantization truncates toward zero: never a gain, up to one count lost,
    # and a dead zone under one count.
    q = DynamixelPwmServo(tau_cap=TAU_CAP_CURRENT, current_noise_a=0.0,
                          quantize=True)
    quantum = q.quantum_nm()
    print("one Goal PWM count =", np.round(quantum * 1e3, 2), "mN.m of command")
    tc = rng.uniform(-2.5, 2.5, (4000, 4))
    err = np.array([q.applied(t, dt) for t in tc]) - tc
    assert np.all(err * np.sign(tc) <= 1e-12), "truncation can never over-deliver"
    assert np.all(np.abs(err) < quantum + 1e-12), "loss must be under one count"
    print(f"quantization loss: mean {np.abs(err).mean()*1e3:.2f} mN.m, "
          f"max {np.abs(err).max()*1e3:.2f} mN.m")
    assert np.allclose(q.applied(0.9 * quantum, dt), 0.0), "sub-count dead zone"

    # (h) calibration error scales delivered torque by nominal/true.
    err_pc = 1.10
    g = DynamixelPwmServo(tau_cap=TAU_CAP_CURRENT, current_noise_a=0.0,
                          nm_to_counts_true=NM_TO_COUNTS * err_pc)
    assert g.digital, "a calibration split must engage the duty path"
    assert np.allclose(g.applied(tau, dt), tau / err_pc), "gain error"
    print(f"true = {err_pc:g}x nominal -> delivers {100.0/err_pc:.1f}% of every "
          f"commanded N.m; a current error there is worth "
          f"{np.round(g.torque_noise_nm()*1e3, 2).tolist()} mN.m")

    # (i) that error is INVISIBLE in the effort readback.
    seen = g.sense(g.applied(tau, dt))
    print(f"commanded {tau.tolist()} -> applied "
          f"{np.round(g.applied(tau, dt), 4).tolist()} -> reported "
          f"{np.round(seen, 4).tolist()} N.m")
    assert np.max(np.abs(seen - tau)) < 1.1 / NM_TO_COUNTS.min(), \
        "readback must show ~the commanded torque despite the gain error"

    # (j) the noise, by contrast, IS visible there -- it is what was measured.
    nz = DynamixelPwmServo()
    d = np.array([nz.sense(nz.applied(tau, dt)) - tau for _ in range(20000)])
    print(f"effort readback scatter = {np.round(d.std(axis=0)*1e3, 2).tolist()} "
          f"mN.m rms (torque noise "
          f"{np.round(nz.torque_noise_nm()*1e3, 2).tolist()})")
    ok &= np.all(d.std(axis=0) > 0.5 * nz.torque_noise_nm())

    # ── gearbox friction ────────────────────────────────────────────────────
    print("\n--- gearbox friction (report eq. 5, momentum-clamped) ---")
    print("fc          =", np.round(FRICTION_FC_NM * 1e3, 2), "mN.m  (duty",
          FRICTION_FC_DUTY.tolist(), ")")
    print("mu          =", FRICTION_MU.tolist(), " fv =", FRICTION_FV_NM_S.tolist(),
          " w =", FRICTION_WIDTH_RAD_S, "rad/s")
    fr = DynamixelPwmServo(current_noise_a=0.0, friction_scale=1.05)
    print(fr)
    rng = np.random.default_rng(3)
    # (k) dissipative, zero at rest, odd in qd, and never a gain in sign
    for _ in range(2000):
        v = rng.uniform(-1.0, 1.0, 4) * rng.choice([1e-4, 1e-2, 1.0])
        t = rng.uniform(-3.0, 3.0, 4)
        f = fr.friction_torque(t, v, dt)
        assert np.all(f * v >= 0.0), "friction must oppose motion (f*qd >= 0)"
        assert np.allclose(fr.friction_torque(t, -v, dt), -f), "odd in qd"
        assert np.all(np.abs(f) * dt <= fr.inertia_min * np.abs(v) + 1e-15), \
            "momentum clamp: |f| dt <= I_min |qd|"
    assert np.all(fr.friction_torque(np.ones(4), np.zeros(4), dt) == 0.0), "zero at rest"
    print("dissipative, odd, zero at rest, momentum-clamped: OK")
    # (l) at speed the level is scale*(fc + mu|tau|), i.e. the 5 % surplus
    lvl = fr.friction_torque(np.full(4, 0.7), np.full(4, 0.5), dt)
    exp = 1.05 * (FRICTION_FC_NM + FRICTION_MU * 0.7)
    assert np.allclose(lvl, exp, rtol=1e-6), f"level {lvl} vs {exp}"
    print(f"at 0.5 rad/s under 0.7 N.m: {np.round(lvl * 1e3, 2).tolist()} mN.m "
          f"= 1.05 x (fc + mu*0.7)   (compensation pays {np.round(exp / 1.05 * 1e3, 2).tolist()})")
    # (m) an isolated armature spinning down under friction alone: explicit
    # integration must never reverse the sign, and must come to rest.
    for I in (ARMATURE_KG_M2, 3.0 * ARMATURE_KG_M2):
        v = np.full(4, 0.3)
        reversed_ = False
        for _ in range(2000):
            f = fr.friction_torque(np.full(4, 3.0), v, dt)   # worst load: the clamp
            v_new = v - dt / I * f
            reversed_ |= bool(np.any(v_new * v < 0.0))
            v = v_new
        assert not reversed_, f"velocity reversed under friction at I={I}"
        assert np.all(np.abs(v) < 1e-9), f"did not come to rest: {v}"
    print("spin-down at the 3 N.m clamp: stops, never reverses, I = armature and 3x")
    # (n) scale 0 is the frictionless plant, bit for bit
    z0 = DynamixelPwmServo(current_noise_a=0.0)
    assert z0.friction_scale == 0.0
    assert np.array_equal(z0.applied(tau, dt, qdot=np.ones(4)), tau), "scale 0 = no friction"
    try:
        fr.applied(tau, dt)
        raise AssertionError("friction without qdot must refuse")
    except ValueError:
        pass
    print("scale 0 is exact; friction without a velocity is refused")

    print("\nself-test", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(_self_test())
