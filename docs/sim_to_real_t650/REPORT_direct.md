# T650 DIRECT actuation: IsaacSim vs the 2026-08-07 indoor flights

Baseline for the direct-actuation mode, run **with the shipped parameters untouched** — no
tuning was applied and none is recommended until the items in §6 are settled.

| flight | bag | DIRECT window | steps compared |
|---|---|---|---|
| **D1** | `debug_recording_direct_actuation_20260807_145416` | 30.3–191.8 s (161.5 s) | 13 (9 position, 3 yaw) |
| **D2** | `debug_recording_direct_actuation_20260807_150135` | 39.8–224.8 s (185.0 s) | 20 (11 position, 9 yaw) |

Both bags begin under baseline/SAFETY control and switch to DIRECT partway through. D2 runs
three yaw steps in **baseline** mode first (t = 18–28 s); those are excluded — only the
DIRECT segment is compared. Segmentation is taken from the
`direct_actuation/mode` topic, not assumed. **t = 0 is DIRECT engagement** on both sides.

## 1. Two independent comparisons

The campaign runs the same flights through two different tests, because they answer
different questions and only the second isolates the plant.

**(a) Closed loop** — `run_direct_case.sh` + `stack_driver_direct.py`. The recorded
reference waypoints are replayed into the live stack; the controller regenerates its own
attitude setpoints, body-rate commands and motor commands. Same controller build, same
config file (`params_single_drone_direct_actuation_t650.yaml` is shared verbatim with
hardware), same waypoints. Answers *"does the whole system behave the same?"*

**(b) Same-command plant test** — `plant_replay.py`. The motor commands the real vehicle
**actually flew** are driven through the simulator's own plant model
(`LaggedQuadraticThrustCurve` maths, same k, c, λ, mass, inertia) and the predicted
rigid-body response is compared against what the real vehicle actually did. The input is
identical by construction, so every difference that remains is plant. Answers *"is the
plant right?"*

Test (b) exists because (a) cannot separate a plant error from the controller's
compensation for it — the baseline campaign already demonstrated that trap (position shape
RMSE 5.5 cm while the acceleration producing it was off by +22.6%).

> **Why not replay commands through Isaac open-loop?** A multirotor is open-loop unstable in
> attitude; 160 s of recorded commands with no feedback diverges in a second or two. The
> well-posed form of "same command, compare state" is the instantaneous response — angular
> acceleration and specific thrust — which is exactly what the plant parameters set.

## 2. Closed-loop result: position good, yaw and rates not

| metric | D1 real | D1 sim | D2 real | D2 sim |
|---|---|---|---|---|
| position overshoot [%] | 23.2 | 19.7 | 30.9 | 22.4 |
| position rise 10–90 [s] | 2.27 | **1.75** | 2.60 | **1.92** |
| position shape RMSE [m] | — | **0.240** | — | **0.269** |
| yaw overshoot [%] | 15.6 | **0.33** | −8.0 | **−0.07** |
| yaw rise 10–90 [s] | 1.45 | **0.85** | 1.47 | **1.01** |
| yaw shape RMSE [deg] | — | **7.58** | — | **9.07** |
| body-rate p RMS [deg/s] | 5.02 | **1.62** | 28.79 | **1.40** |
| body-rate q RMS [deg/s] | 7.69 | **1.87** | 16.43 | **1.80** |
| roll bias [deg] | — | +0.85 | — | +0.97 |

Trajectory correlation is high (position 0.94–0.998, yaw 0.975–0.980) and the run overlays
look convincing — but that is the trap above. Underneath:

- **Simulation is systematically faster.** Position rise is 23–26 % short on both flights,
  and yaw rise 30–41 % short.
- **Simulated yaw has essentially no overshoot** (0.1–0.3 % against a real 8–16 %). Note
  this is the *opposite sign* of the baseline-mode finding, where simulated yaw overshot
  44.9 % against a real 9.2 %. Different mode, opposite error — see §4.
- **Body rates barely correlate** (p: 0.14 and 0.02; q: 0.10 and 0.04). Simulated rate RMS
  is 3–20× smaller. Position shape RMSE is **0.24–0.27 m against the baseline campaign's
  0.055 m** — DIRECT tracks the trajectory far less well than baseline did.

## 3. Same-command plant test: the plant is wrong in three specific ways

Dynamic response about trim, band 0.2–12 Hz, both traces filtered identically
(`figures/direct/D*_plant.png`):

| axis | D1 gain | D2 gain | reading |
|---|---|---|---|
| roll | 0.431 | 0.375 | model over-predicts angular accel **2.3–2.7×** |
| pitch | 0.504 | 0.395 | model over-predicts **2.0–2.5×** |
| yaw | 0.202 | 0.163 | model over-predicts **5–6×** |
| thrust | 0.936 | 0.869 | model over-predicts thrust **7–15 %** |

*gain = measured / model-predicted; < 1 means the simulator produces more response than
reality for the identical command.*

### 3a. Thrust: ~5 % high, and battery sag is visible within each flight

Over-prediction in 20 s windows through the DIRECT segment:

| | 0 s | 20 s | 40 s | 60 s | 80 s | 100 s | 120 s | 140 s |
|---|---|---|---|---|---|---|---|---|
| **D1** | +4.7% | +5.6% | +7.0% | +7.3% | +7.4% | +7.3% | +7.1% | — |
| **D2** | +12.4% | +13.5% | +14.1% | +14.2% | +15.0% | +16.0% | +17.6% | +16.6% |

Two separate effects:

- **Within a flight** it grows +2.7 pts (D1) and +5.2 pts (D2) — battery sag, the same
  effect the baseline campaign found, now measured on the thrust path directly.
- **Between flights** D2 starts 7.7 pts higher than D1. D2 was flown ~7 minutes after D1 on
  the same pack.

So the only uncontaminated number is **D1's first window, +4.7 %**. Taken there:

| attribute the 4.7 % to | implied value | vs shipped |
|---|---|---|
| mass | **3.18 kg** | shipped 3.034 kg |
| thrust constant k | **4.47e-05** | shipped 4.68e-05; **bench 4.54e-05** |

Either reading points the same way: **the bench `k` was right and `THRUST_FIT_FACTOR` is
not.** The implied k lands within 1.5 % of the bench value, while the shipped (tuned) k is
3.1 % above it in the wrong direction. This is an independent measurement, from a different
flight and a different method, of the exact question raised when `THRUST_FIT_FACTOR` was
introduced.

### 3b. Roll/pitch: angular acceleration over-predicted ~2.4×

Consistent across both flights (0.375–0.504). Since the **thrust** gain is ~0.95, `k` is
approximately right, so the error is not in the force — it is in **torque/inertia**. Taken
as inertia: implied Ixx, Iyy ≈ **0.13–0.17 kg·m²** against the shipped 0.0596/0.0661, i.e.
**2.0–2.6× larger**.

This is consistent with the standing suspicion recorded in `t650_params.py`: the inertia
diagonal is **copied from the X650's CAD for a 1.467 kg bare frame**, on the assumption that
the additional ~1.5 kg is centrally concentrated. These numbers say that assumption is
wrong.

**Caveat, and it is a real one.** This is closed-loop data, so inertia and aerodynamic
damping cannot be cleanly separated — regressing `alpha_meas = a·alpha_pred + b·omega`
returns a **positive** `b` on roll/pitch, which is non-physical for damping and indicates
the regressors are correlated through the controller. R² is 0.21–0.51. Treat the 2.4× as a
strong indication of the direction and rough size, **not** as a calibrated value. A proper
determination needs either a bifilar-pendulum measurement or real T650 CAD.

### 3c. Yaw: this is the ×3 `YAW_TORQUE_FIT_FACTOR`, and in DIRECT it is harmful

Re-running the same test with the **bench** `c` moves the yaw gain from **0.202 → 0.607**,
i.e. almost exactly the factor of 3. The `YAW_TORQUE_FIT_FACTOR = 3.0` — fitted in
**baseline** mode against flight C's two ±20° yaw steps through PX4's yaw loop — is
over-driving the simulated yaw axis in DIRECT by very close to that same 3×.

This is exactly the non-portability that was predicted when the factor was introduced: it
compensates a missing term (`I_rotor·ω̇`) at one operating point and one gain set, and
does not transfer. The deployed direct-actuation config already flags it — its own comment
puts simulated yaw authority at 2.45× the value its rate gains were tuned for.

The residual 0.607 with bench `c` is in the same direction as roll/pitch (over-prediction),
consistent with a shared inertia error rather than a yaw-specific one.

### 3d. Measured actuation dead time: 25–40 ms, not modelled at all

Sweeping a time shift between the command stream and the measured response, correlation
peaks at:

| | roll | pitch | yaw |
|---|---|---|---|
| D1 | 40 ms | 35 ms | 0 ms |
| D2 | 30 ms | 25 ms | 0 ms |

This is delay **beyond** the modelled rotor lag (λ = 10.0265, τ ≈ 100 ms, already in the
prediction). The simulator has no such term. At rate-loop frequencies 30 ms is tens of
degrees of phase, at unity gain — the cheapest possible way to lose stability margin, and
the one error that makes simulation look *more* stable than hardware.

Correcting for it changes the gains only marginally (roll 0.431 → 0.463), so it does **not**
explain §3b.

### 3e. A standing trim the simulator does not have

The real airframe holds attitude with markedly asymmetric motor commands
(`[0.485, 0.525, 0.516, 0.559]` on D1). Through the perfectly symmetric model those same
commands imply a constant torque the real vehicle plainly is not feeling (its mean gyro is
~0):

| | D1 | D2 |
|---|---|---|
| implied pitch torque | −0.490 N·m | −0.514 N·m |
| equivalent CoM offset in x | **−15.5 mm** | **−15.0 mm** |
| equivalent CoM offset in y | +0.8 mm | +1.4 mm |

The two flights agree to 0.5 mm — this is a repeatable airframe property, not noise. Left in
the simulator, those commands would pitch the simulated vehicle at −7.4 rad/s². It is
almost certainly the same effect behind the baseline campaign's unexplained "both vehicles
hover tilted, in opposite directions, ~2.5° apart" finding, now quantified for the first
time. `ratectl_trim_x/y/z` are all 0.0 in the shipped config, with a comment saying the
simulated plant is symmetric and has nothing to trim — correct for the simulator, wrong for
the vehicle.

## 4. Why simulated yaw overshoots in baseline and not in DIRECT

Same plant error, opposite symptom, because the loop around it differs. In baseline PX4's
yaw loop met a 3×-over-powered plant and rang. In DIRECT the fsc_autopilot rate loop meets
the same over-powered plant but with its own gains and a much stiffer inner loop, so it
drives the axis to target fast and hard with no overshoot — while the real vehicle, with
the true (weaker) authority, overshoots. The plant error is the constant; the sign of the
symptom is not. This is why yaw gains tuned in simulation must not be carried to hardware —
now confirmed a fourth time, and in a fourth way.

## 5. Not modelled, cannot be tuned

- **Vibration.** Real body-rate RMS reaches 28.8 deg/s on D2 against a simulated 1.40. Much
  of the real content is above the rigid-body band. This matters far more in DIRECT than in
  baseline, because `ratectl_kd` and the 30 Hz D-term filter now consume that gyro directly.
  **Do not tune D-terms in simulation.**
- **Battery sag** — quantified in §3a, still unmodelled.
- **The CoM/trim offset** of §3e.

## 6. What to change, in order

Nothing here is a sweep; each item is a measurement or a model term.

1. **Weigh the assembled vehicle.** §3a says the plant is ~5 % light *or* `k` is ~5 % high,
   and that the bench `k` is the better of the two available values. One measurement settles
   `BODY_MASS` and very likely sends `THRUST_FIT_FACTOR` to 1.0.
2. **Add the `I_rotor·ω̇` term** to `LaggedQuadraticThrustCurve` and revert
   `YAW_TORQUE_FIT_FACTOR` to 1.0. §3c shows the ×3 is actively wrong in DIRECT, and the
   term is nearly free — the class already tracks lagged ω, so ω̇ is available.
3. **Measure the roll/pitch inertia properly** (bifilar pendulum, or T650 CAD). §3b says it
   is ~2.4× low but cannot pin it from closed-loop data. This is the largest single error in
   the plant and the copied-from-X650-CAD value has no support for a 3 kg vehicle.
4. **Model the 25–40 ms actuation dead time**, or record it as a known bias and stop
   trusting marginal-stability results from simulation (§3d).
5. **Only then** revisit λ. It was untouched yesterday and is bench-derived; it should not
   move until 1–4 are settled, since all of them alias into apparent lag.

Do **not** tune any of the 20 control gains — they are shared verbatim with hardware and are
the thing being predicted.

## 7. Reproducing

```bash
# extract + segment (needs ROS 2 + px4_msgs sourced; system python3.10)
tools/extract_direct_bag.py <bagdir> data/real_D1.npz
tools/make_direct_refs.py   data/real_D1.npz data/ref_D1.npz

# closed-loop case (starts the whole stack; SIMULATION ONLY -- it auto-arms)
tools/run_direct_case.sh data/ref_D1.npz data/sim_direct_D1.npz D1

# figures + metrics
tools/plot_direct.py    D1 data/real_D1.npz data/ref_D1.npz data/sim_direct_D1.npz figures/direct
PYTHONPATH=tools tools/metrics_direct.py D1 data/real_D1.npz data/ref_D1.npz data/sim_direct_D1.npz

# same-command plant test (no Isaac needed)
tools/plant_replay.py data/real_D1.npz figures/direct/D1
tools/plant_replay.py data/real_D1.npz /tmp/benchc --c 8.247173e-07   # isolate the x3
```

Analysis runs on the **system** `python3` (numpy 2.x + matplotlib). The installed scipy 1.8
is compiled against numpy 1.x and fails to import — `plant_replay.py` therefore carries its
own zero-phase FIR rather than using `scipy.signal`.
