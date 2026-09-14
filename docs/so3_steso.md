# SO(3) super-twisting extended state observer / uncertainty-and-disturbance estimator

**Code:** `fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator/so3_steso.py`
(numpy only; no ROS, CasADi or Isaac).
**Gate:** `~/envs/isaac_env/bin/python3 tests/test_so3_steso.py` (~20 s; `--quick` ~10 s). PASS 2026-09-13.
**Status:** built, gated offline on a rigid body and on the plant double. **Not wired into any node,
not flown.** Nothing that flies today was touched.

## What it is

Xie et al. (arXiv 2111.02046) estimate, per Euler channel, the attitude, the body rate and a lumped
disturbance with a super-twisting extended state observer (Levant's second-order robust exact
differentiator with the known dynamics fed forward), then cancel the disturbance in an adaptive
recursive sliding-mode attitude law. Their observer lives under a small-angle assumption (Euler
rates = body rates). This module is **their observer on SO(3)**, and only the observer -- the
ARSMC controller half of the paper is not implemented; the estimate is meant for the inner loop
this project already has, the way `L1Aug`'s `u_ad` is.

Plant as the estimator sees it (body FLU, world ENU, `R = R_WB` from the mocap quaternion, `w`
the body rate -- exactly the cascade's inner-loop inputs):

    R_dot   = R [w]_x
    J w_dot = M_model + J sigma

`M_model` is the caller's own account of `J w_dot` (for the cascade: `model_moment()` -- rotors
through `B_rot` at the actual tilt, aero, gyroscopics). `sigma` [rad/s²] is the lumped residual:
CG error, unmodelled aero, actuator mismatch, slosh, wind. **The residual is estimated, not the
plant** (same division of labour as L1Aug: an estimator that learns the whole plant also cancels
the aero stiffness the outer NMPC plans with).

### Mode `"attitude"` -- the paper's structure, order 3, only `R` measured

    e             = Log(R_hat^T R)                          rotation vector R_hat -> R [rad]
    R_hat_dot     = R_hat [ R_e (w_hat + k1 sig^{2/3}(e)) ]_x
    w_hat_dot     = J^{-1} M_model + sigma_hat + k2 sig^{1/3}(e)
    sigma_hat_dot =                              k3 sign(e)
    k1 = 2.0 L^{1/3}    k2 = 2.12 L^{2/3}    k3 = 1.1 L      (Levant's n = 2 gains, per axis)

The innovation is the rotation vector, valid at any attitude (a robust Log handles 0 and near π).
The rotation vector's axis is the same vector in the true and the estimated body frame, so `e`
needs no transport; the rate fed to the kinematics does (`R_e = R_hat^T R`), and with it the error
rotation obeys exactly `R_e_dot = R_e [w - w_hat - k1 sig^{2/3}(e)]_x` -- Levant's differentiator
with a vanishing perturbation, instead of the `O(|e||w|)` term an Euler-channel observer carries
through a turn. `R_hat` is advanced with the exponential map and stays on the group.

### Mode `"rate"` -- order 2, `R` and `w` measured (this rig; any IMU)

    e_w           = w - w_hat
    w_hat_dot     = J^{-1} M_model + sigma_hat + 1.5 L^{1/2} sig^{1/2}(e_w)
    sigma_hat_dot =                              1.1 L       sign(e_w)

This is the one to use where a gyro exists: estimating an acceleration from an angle costs a cube
root of the noise (`L^{2/3} ε^{1/3}`), from a rate a square root.

**Sign, checked empirically** (the paper's eq. 12 has the correction sign flipped relative to its
own error dynamics eq. 13; eq. 13 is what converges): a +1 N·m pitch torque the model does not
contain reads as `sigma_y = +1/Iyy` and `torque_y = +0.9999 N·m` in both modes, other axes ~1e-5.

## Knobs

| knob | meaning | guidance |
|---|---|---|
| `L` (scalar or per axis) | bound on `|sigma_dot|` [rad/s³] -- how fast the disturbance angular acceleration can change | **the only real knob.** A CG moment is constant (small L). A 100 mm tank's 4.2 N·m at ~1 Hz on pitch needs ≥ 90. Below the true slope the estimator **cannot follow at all** (L 5 vs a 16.6 rad/s³ slope: 2.25 rad/s² of lag); above it, noise grows as `L^{2/3}` / `L^{1/2}`. |
| `eps` | boundary layer on `sign()` (rad / rad/s); `"auto"` = 1.5 k3 dt³ / 1.0 k2 dt² | the discretisation chatter is ±1.1 L dt per tick (0.18 rad/s² at L 20 / 120 Hz); `"auto"` cuts the floor 2-4× at 120-500 Hz while still following a 16.6 rad/s³ ramp. Larger multiples trade ramp tracking for quiet. |
| `tau_out` | low-pass on `sigma_filt` / `torque_filt` | for what is **fed back**, not for what is measured: 0.05 s is 18° of lag at 1 Hz. L1Aug's `w_c = 2` rad/s is `tau_out ≈ 0.5`. |
| `tau_bias` | splits `sigma` into `sigma_bias` (quasi-static: CG, trim -- the "uncertainty") and `sigma_dyn` (the "disturbance") | diagnostic split; the estimator itself is lumped |
| `delay` | transport delay between `M_model` being issued and the plant applying it | 0.03 on this rig. Buffers like `L1Aug.propagate`; the actuator **lags** (rotor spin 0.12 s, servo 0.03 s) are the caller's -- hand it lagged thrusts/deflections (`L1Aug.applied_moment`) or `sigma` learns the lag and, fed back, hunts (the L1 finding of 2026-09-13). |
| `hold` | keep the estimate, learn nothing | through the back-transition, as L1 does |
| `sigma_max` | clamp | safety net: the estimate can never exceed what the actuators could produce |
| `add_gyro` | add `-w × J w` inside | `False` for the cascade (`model_moment` includes it) |

Outputs: `sigma` [rad/s²], `torque = J sigma` [N·m], the filtered pair, `R_hat`, `omega_hat`,
`e` (innovation), `converged(tol)`.

## What was measured (tests/test_so3_steso.py)

**Rigid body, assembled inertia, noiseless, 1 kHz, L 30, a disturbance ramping at 16.6 rad/s³
with a 1 N·m step:** attitude mode from a 20° initial error -- innovation 1.8e-7 rad by 1.0 s,
estimate exact to the chatter (rms 0.03-0.05 vs 1.1 L dt = 0.033), the step re-acquired in
0.24 s; rate mode the same in 0.19 s with `|w_hat - w|` < 1e-4. **The same estimate from a level
body at rest and from an inverted body tumbling at (0.5, 0, 1.0) rad/s** (difference < chatter);
a 120° initial estimate error on the tumbling body converges in 2.2 s.

**Noise floor at the rig's 120 Hz** (`eps="auto"`, pitch, rms rad/s², gentle disturbance):

| attitude mode, mocap jitter | L 5 | L 20 | L 80 |
|---|---|---|---|
| 0 | 0.05 | 0.16 | 0.39 |
| 0.0004° (this HIL rig) | 0.08 | 0.15 | 0.40 |
| 0.02° | 0.21 | 0.29 | 0.89 |
| 0.1° (real OptiTrack, worst) | 0.34 | 0.42 | 1.15 |

| rate mode, gyro noise | L 5 | L 20 | L 80 |
|---|---|---|---|
| 0 | 0.04 | 0.04 | 0.12 |
| 0.1°/s | 0.08 | 0.14 | 0.29 |
| 1°/s | 0.23 | 0.27 | 0.68 |

At zero noise the floor is the discretisation chatter; HIL's 0.0004° is invisible under it.
For scale: the CG-error moment L1 was built for is 1.9 N·m = 6.4 rad/s² on pitch.

**Against a linear ESO tuned to the same noise floor (120 Hz, 0.05° / 0.5°/s) -- PARITY:**

| | STESO L 20 | linear ESO, matched wo | ratio |
|---|---|---|---|
| rate: 1 Hz sine 0.3 / 0.8 N·m | 0.28 / 0.48 | 0.28 / 0.49 (wo 54) | 0.97 / 1.02× |
| rate: 1 N·m step re-acquired | 0.12 s | 0.06 s | |
| attitude: 1 Hz sine 0.3 / 0.8 N·m | 0.44 / 0.98 | 0.49 / 1.07 (wo 33) | 1.11 / 1.09× |
| attitude: 1 N·m step | 0.15 s | 0.12 s | |

The super-twisting third channel slews at most 1.1 L per second (a 3.3 rad/s² step takes 0.15 s at
L 20 just to ramp); the linear correction grows with the error. **The finite-time exactness is a
noiseless, high-rate property. At 120 Hz with noise the two designs are the same filter with
different knobs.** What remains in the STESO's favour: `L` means something physical and the
accuracy needs no re-tuning against the disturbance's size within it; the attitude-only mode.
Against it: `L` below the slope is a hard failure where the linear one merely lags. The
Taylor-consistent attitude increment (Hanan/Jbara/Levant low-chattering scheme) was tried and
changes nothing -- the chatter is the sign term.

**Plant double, hover, CG 5 cm forward of what the controller believes** (SE(3) hold, kR 9 / kW
1.8, nominal geometry, 200 Hz). Open-loop, symmetric hover thrust makes +1.938 N·m on y (T·dx):

| loop | pitch steady | torque_y estimate | position error |
|---|---|---|---|
| P-D, attitude + altitude only | −5.8° (drifts; the residual then includes the wing) | +0.90 | -- |
| P-D with position hold | 0.00° (the position loop tilts R_d to fight the drift) | **+1.938 ± 0.005** | **1.08 m** |
| + feed-forward, rate mode | 0.00° | +1.938 ± 0.005 | 0.00 m |
| + feed-forward, attitude mode | 0.00° | +1.939 ± 0.03 | 0.00 m |
| + ff, rate, 30 ms delay, 0.05° / 0.5°/s noise, tau_out 0.1 | 0.00° | +1.936 ± 0.06 | 0.00 m |
| + ff, attitude, same | 0.01° | +1.949 ± 0.06 | 0.01 m |

(The plant double has no rotor spin lag, so the L1 hunt that the lag caused cannot be reproduced
here; the cascade's faithful gate is where that lives.)

## In the cascade: `nmpc_cascade_steso_node.py` (2026-09-14, gated, not flown)

A variant, not a copy (the tank node's argument): it imports the base node and the tank schedule,
subclasses `L1Aug` as `L1Steso` (same C(s), clamps, freeze, hold, lag-aware applied moment and
delay buffer; only the predictor + PWC law are replaced by the STESO), and re-points the base's
two module-level seams `L1Aug` and `inner_step`. `--est pwc` is bit-identical to the tank node
(verified on the trace). Closed-loop page with the diagram, laws and the gate's figures:
https://claude.ai/code/artifact/bb0006e0-75aa-4248-8454-332766f86bdf

| faithful gate (lat 30 / comp 49 / outer_lag 30 ms, momentum + spin lag, pendulum tank) | base L1 law | STESO |
|---|---|---|
| hover, 5 cm CG error: pitch / σ̂_y read (true 1.938) | −0.59° / 1.881 (97 %) | −0.27° / 1.938 (100 %) |
| same, raw σ̂ sd under 0.1° / 1°/s noise | 1.53 N·m | 0.06 N·m |
| full mission, 2.6 cm CG error (flown value) | stops, 0/1701 | stops, 0/1701, same trajectory |
| full mission, 5 cm, cold start into the corridor (ttr 10 or 20) | crash t≈10–14 s | crash t≈10–14 s (base envelope, not the estimator) |
| turning mission, 100 mm tank 0.3 m ahead (3.6–4.3 N·m at the wall) | 0/4001, RTH 5.7 m | 0/4001, RTH 5.6 m; σ̂ shows the 1.14 Hz tank yaw at ±0.5 N·m |

Same u_ad and same flight every time; the STESO's raw estimate is 25× quieter and exact where the
PWC law is 3 % low. That headroom (a faster w_c) is untested. The launchers still start the tank
node; flying this is a one-line launcher change.

### The recipe as it was written (kept for reference)

Next to `L1Aug` in `nmpc_cascade_node.py`:

```python
from .so3_steso import SO3STESO
self.steso = SO3STESO(J=(so3.IXX, so3.IYY, so3.IZZ), L=20.0, mode="rate", eps="auto",
                      delay=latency_s, tau_out=0.5, sigma_max=(10, 10, 4))
# each inner tick, where l1.adapt()/propagate() run:
M_model = l1.applied_moment(beta, T_cmd, vb, w, de, da, dr, dt)   # LAG-AWARE, the L1 lesson
self.steso.step(R, M_model, dt, omega=w)
M_des -= np.clip(self.steso.torque_filt, -umax, umax)              # or only J*sigma_bias
self.steso.hold = decel_active                                      # as L1's hold
```

`tau_out 0.5` reproduces L1's separation (fast estimate, ~2 rad/s cancellation). Gate it the same
way L1 was: `--plant_cg_dx 0.05 --hover_only` on the faithful gate, then the 30 s engage hover in
HIL. Expect the same −0.8° result L1 gives -- the parity measurement says it will not be better,
only differently tuned. Where it could differ is a **fast** unmodelled moment inside L (an offset
tank in a turn entry) that L1's 2 rad/s filter blurs; measure that on the gate before claiming it.

## What it cannot claim

- Nothing here is flown. The offline gate and the plant double have no rotor spin lag in the
  estimator's path unless the caller supplies it.
- The near-π innovation is a free sign choice (a topological fact for any continuous observer on
  SO(3)); the observer proceeds either way, but a 180° initial estimate error is a measure-zero case
  the test does not cover.
- The ARSMC controller of the paper is not implemented; the estimate feeds the existing inner loop.
