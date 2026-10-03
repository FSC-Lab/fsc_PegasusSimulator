# Flight State Machine — `02_aerial_manipulator_free.py`

Three phases, one-way: **takeoff → task → landing**. Every switch is gated on the
measured **hover error**, never on the clock. Takeoff and landing are position
**setpoints**; only the middle phase tracks a planned trajectory — the same
workflow a real flight uses.

Applies to the landing modes `MODE = "poly_whole"` / `"poly_drone"`. Non-landing
modes (`hover_*`, `circle_*`, `figure8_*`) run phases 1–2 only.

---

## Init (before physics)

Spawn → arm teleported to folded home `q = [0, 40°, 46°, 0]` → body rigidly
re-seated → **ground-seated** so body z = `GROUND_BODY_Z` (0.305 m, on its legs).
One silent physics step anchors the trajectory, prints the flight-volume report
and draws the waypoint markers; then the timeline **pauses** for inspection.

Press **PLAY** to fly. Never press **STOP** — stop→play resets PhysX to its
snapshot (arm at q = 0, midair spawn) and silently undoes the init.

Headless/sweep runs skip the pause.

---

## Phase 1 — TAKEOFF

| | |
|---|---|
| Reference | one constant setpoint = the task trajectory's **start point** (≈ spawn xy at 1.5 m CoM). No ramp. |
| Arm | PD hold + gravity comp at home. `ctrl.hold = True` → `u3 = 0`, GMO frozen |
| Body | whole-body law (always, in every phase) |

**Exit gate** — all of, held **continuously** for `HOVER_DWELL`; any violation
resets the timer:

- 3-D CoM error ≤ `HOVER_POS_TOL`
- body speed ≤ `HOVER_V_TOL`
- arm within `HOVER_Q_TOL` of home

On exit, in one step: re-anchor the trajectory at the **actual** hover pose
(starts the task at `e_x = e_y = 0`), hand the arm to the whole-body law, enable
the GMO, start the task clock.

## Phase 2 — TASK

| | |
|---|---|
| Reference | the 18 s compatible plan: fly-in 6 s → pinned 6 s → fly-out 6 s. **Ends hovering** at `land_wp` — no descent in the plan |
| Arm | whole-body law, GMO active |

**Exit gate:** plan clock ≥ T **and** the hover gate (position + speed) held for
`HOVER_DWELL` at the task **end point**.

On exit the arm returns to the PD hold **while the reference stays parked at the
end point**.

## Phase 3 — LANDING

One control step later — with the arm confirmed on the PD — the reference steps
down to CoM z = `LAND_SP_COM_Z`.

| | |
|---|---|
| Reference | one setpoint: task end xy, z = `LAND_SP_COM_Z` |
| Arm | PD hold at home throughout, GMO frozen |

Settled at that setpoint (same gate) → rotors ramp to zero over
`LAND_DISARM_TIME`, gravity takes the last ~4 cm → after `CLOSE_DELAY` the npz
is saved and the app **closes itself**.

---

## Sequencing rule (why the phases are staged this way)

**Change one thing at a time.** The arm's torque-source switch always happens at
a *constant* reference, and a reference **step** is only commanded once the arm
is already on its new source.

This exists because of a real failure: dropping the landing reference 1.17 m in
the same step that handed the arm back made the body cut thrust, so
`F_trans = u1·R0·e3 − m·g·e3 ≈ −32 N` flooded `u3` through the coupling
feedforward and the demanded arm torque hit **90 N·m** — 22× `TAU_MAX`,
saturating two joints. Visible as a violent arm swing.

Two consequences worth keeping in mind:

- **No torque blend is needed.** Measured across a properly sequenced switch the
  two arm torques differ by 0.005 N·m at takeoff and 0.000 at landing (0.1% of
  `TAU_MAX`): at a settled hover `e_y = 0` and `F_trans = 0`, so `u3 ≈ 0` and the
  law's arm torque *is* the gravity term the PD already applies.
- **Only steps that CREATE error need isolating.** The takeoff re-anchor moves
  the reference *onto* the vehicle (shrinking error), and the plan is
  rest-to-rest, so switching the arm and starting the plan together is safe.

General rule for the whole-body law: it is feedback-linearizing, so it converts
an infeasible reference into an infeasible *torque demand* rather than into a
bounded tracking error. **It must only ever see continuous, dynamically feasible
references.** Anything step-like goes through a smoother, or to a controller that
does not care — which is what the PD hold is.

---

## Knobs

| Constant | Value | Meaning |
|---|---|---|
| `START_PAUSED` | `True` | pause after init (rendered runs only) |
| `GROUND_BODY_Z` | 0.305 m | measured resting body height |
| `TAKEOFF_ARM_HOLD` | `True` | PD-hold the arm through takeoff |
| `HOVER_POS_TOL` | 0.15 m | must exceed the ~0.1 m steady lateral offset (no integrator) or the gate never clears |
| `HOVER_V_TOL` | 0.20 m/s | |
| `HOVER_Q_TOL` | 0.05 rad | takeoff gate only |
| `HOVER_DWELL` | 2.0 s | continuous, resets on violation |
| `LAND_SP_COM_Z` | 0.33 m | ~4 cm above the resting CoM (0.290 m) |
| `LAND_DISARM_TIME` | 2.0 s | rotor ramp-to-zero |
| `CLOSE_DELAY` | 2.0 s | settle before save-and-exit |

## Terminal output

One two-panel frame per second (and immediately on every phase event) — plain
lines, no cursor control, so it survives the launcher's `tee`:

```
┌────────────────────────────┬──────────────────────────────────────────────────────────────┐
│ FLIGHT                     │ STATE   t =   12.0 s   arm: whole-body   GMO: on             │
├────────────────────────────┼──────────────────────────────────────────────────────────────┤
│   0.0 seated z=0.305       │ DRONE  pos      [m]       +0.004   +2.113   +1.436           │
│   0.0 PLAY                 │        att rpy  [deg]      +0.21    -0.09    +1.20           │
│   3.5 TAKEOFF ok 0.018 m   │ ARM    joint    [deg]     -18.40   +33.10   +41.70    +0.00  │
│   3.5 TASK arm->law        │ ERR    CoM      [m]       -0.001   +0.003   -0.041           │
│                            │        att geom [-]       +0.000   -0.001   +0.000           │
│                            │        EE pos   [m]       +0.002   +0.001   -0.040           │
│                            │        EE yaw   [-]       +0.000                             │
│                            │ CTRL   thrust   [N]        32.75                             │
│                            │        moment   [N.m]     +0.680   -0.004   -0.003           │
│                            │        arm      [N.m]      -0.00    +0.66    +0.18    -0.00  │
│                            │ DIST   force    [N]        +0.02    -0.01    +0.34           │
│                            │        moment   [N.m]      +0.00    -0.00    +0.00           │
│                            │        joint    [N.m]      +0.00    +0.03    -0.01    +0.00  │
└────────────────────────────┴──────────────────────────────────────────────────────────────┘
  gate: takeoff  pos 0.412/0.15 m |v| 0.883/0.20 m/s dq 0.001/0.05 rad dwell 0.0/2.0 s
```

- **Left** — the phase log, colour-tagged: grey init · cyan takeoff · green task
  · yellow landing · magenta done · red warning. Labels are ASCII (`->`), not
  arrow glyphs, which rendered badly.
- **Right** — four groups. Every quantity is **named in the label column** with
  an explicit **[unit]**; the value area is pure numbers, no inline symbols.
  Channels keep their components (one row each) instead of collapsing to a norm,
  so a fault shows which axis it is on:
  - `DRONE` — position, attitude (roll/pitch/yaw)
  - `ARM` — joint angles
  - `ERR` — body channel first (CoM tracking, attitude), then the EE task
    channel (EE position, EE yaw). **`att geom` is the GEOMETRIC attitude
    error** `e_R = ½·vee(R_dᵀR − RᵀR_d)`, *not* roll/pitch/yaw: a body-frame
    dimensionless 3-vector equal to `sin(φ)` for a single-axis error — so ≈
    radians while small, and only meaningful below 90°.
  - `CTRL` — one row per actuator channel: thrust, body moment, arm torque
  - `DIST` — GMO estimates: force, moment, joint
- **`gate:` line** appears only while a phase gate is pending, showing live
  value/tolerance per condition — so a tolerance that can never clear is visible
  rather than silent.

Box is 95 columns. `PANEL_ENABLE`, `PANEL_PERIOD`, `PANEL_W_*` and
`AM_NO_COLOR=1` control it; colour is **not** gated on `isatty()` because the
launcher pipes stdout through `tee`. Rows that would overflow raise a red
`PANEL row ... needs N cols` event rather than truncating silently.

Nominal duration ≈ 29 s (1.2 s climb + 2 dwell + 18 task + 2 dwell + ~1.5
descent + 2 ramp + 2 close).

---

## Open items

- **Not yet run in Isaac.** All of the above is verified offline (planner solve,
  z-step profiles, a mirrored 250 Hz state-machine sim). A rendered run is the
  real gate — see the 2026-07-31b lesson: headless-clean gains crashed a
  rendered run.
- The descent is a **single 1.17 m step**, so the vehicle free-falls briefly
  (peaks ~1.7 m/s, down to 0.16 m/s by 5 cm above the setpoint). Safe, but split
  it into two setpoints if the ride looks bad.

## Arm servo saturation

Where each signal is clamped:

```
                   ┌────────────────────── controller.py ───────────────────────┐
                   │                                                             │
  state, ref ─────►│  hold = True    ──►  u3 = 0                                 │
                   │  (takeoff/land)      tau_joint = 0     ← nothing to clamp   │
                   │                                                             │
                   │  hold = False   ──►  law u3                                 │
                   │  (task)              τ = Tᵀ[ u1·R0e3 ; u2 ; u3 ]            │
                   │                             │                               │
                   │                             ▼                               │
                   │        ╔═════════════ CLAMP ① ═════════════════╗            │
                   │        ║  tau_joint = clip(τ[6:], ±TAU_MAX)    ║            │
                   │        ║  u3        = tau_joint − u1·Aᵀe3      ║  realized  │
                   │        ║  τ         = Tᵀ[ u1·R0e3 ; u2 ; u3 ]  ║  REBUILD   │
                   │        ╚═══════════════════════════════════════╝            │
                   │                    │                    │                   │
                   └────────────────────┼────────────────────┼───────────────────┘
                        tau_joint       │                    │  tau_body = τ[3:6]
                                        │                    │  ← consistent with ①
  ┌─────────────────────────────────────┼────────────────────┼────────────────────┐
  │ 02_aerial_manipulator_free.py       │                    ▼                    │
  │                                     │            mixer ──► 4 rotors           │
  │   PD hold                           │            (OMEGA_MAX — not TAU_MAX)    │
  │   −KP·Δq − KD·q̇ + g_arm ────────────┤                                         │
  │                                     ▼                                         │
  │                  ╔═════════════ CLAMP ② ═══════════════╗                      │
  │                  ║        clip(tau_j, ±TAU_MAX)        ║                      │
  │                  ╚═════════════════════════════════════╝                      │
  │                     hold path → the ONLY clamp (pure actuator limit)          │
  │                     law  path → idempotent repeat (final guard)               │
  │                                     ▼                                         │
  │                  set_joint_efforts ──► 4 arm servos                           │
  └───────────────────────────────────────────────────────────────────────────────┘

  both clamps read the SAME number:  demo sets  C.TAU_MAX = TAU_MAX  at startup
```

| Signal | Clamped by `TAU_MAX`? | Where |
|---|---|---|
| Law arm torque | **yes** | ① controller (with coupling rebuild), then ② demo (no-op) |
| PD-hold arm torque | **yes** | ② demo only — the controller never sees it |
| `tau_body` (body moment) | no limit of its own | rebuilt at ① to match the *clamped* arm torque |
| Rotor speeds | no — `OMEGA_MAX` instead | demo, after the mixer |

**Why ① cannot simply move to the actuator.** The rotors are commanded *before*
the arm torque is clamped:

```python
res   = self.ctrl(...)                          # tau_body computed here
omega = mixer.mix(res["thrust"], res["tau_body"])
backend.input_ref[i] = omega[i]                 # rotors already committed
...
tau_j = np.clip(tau_j, ±TAU_MAX)                # arm clamped only now
```

So a clamp at the actuator can never inform `tau_body` — by then the mixer has
already turned an unclamped `N1ᵀ·u3` into rotor speeds. ① is not an actuator
limit at all; it is a **model correction**, and it has to live where `T`, `A`
and `N1` do.

**What gets clamped, and on what.** The servo limit applies to the TOTAL joint
torque `tau_joint = u1·Aᵀe3 + u3`, so the clamp goes on `tau_joint` and the
realized `u3` is *derived* from it. Clamping `u3` directly would not bound what
the servo sees: at hover the thrust-recruitment term `u1·Aᵀe3` is about
`[0, 0.66, 0.18, 0]` N·m, so a `u3` capped at 4.1 could still demand ~4.76 N·m
from joint 2.

**The PD hold needs no rebuild.** During a hold `u3 = 0`, so `tau_body` carries
no arm reaction and nothing but the servos reads the hold torque. There is no
model to keep consistent — a pure actuator limit, correctly placed at the
actuator.

`res["n_sat"]` counts the law's clamped joints at ①; the demo adds the hold's
clamped count at ②. The two are disjoint, so `self._n_sat` is the total and
**both paths are reported identically**.

`TAU_MAX = 4.1 N·m` (XM430-W350 stall @ 12 V) is enforced **inside the
controller**, and the base moment is rebuilt from the *realized* `u3`:

```
tau_joint = clip(tau_joint, ±TAU_MAX)
u3        = tau_joint - u1·Aᵀe3      # back out what the servos actually deliver
tau       = Tᵀ·[u1·R0e3 ; u2 ; u3]   # base moment now matches the arm
```

This matters because `tau_body` carries the arm's reaction `N1ᵀ·u3`. Clamping
only at the caller — which is what this file used to do — saturates the arm but
leaves the rotors pre-compensating a reaction the servos never produced, the
documented flip path on this rig. Ordering also matters: the clamp runs *before*
the GMO integrates `u`, so the observer sees the realized wrench and cannot
mistake saturation for an external disturbance.

The PD-hold path is clamped separately in the demo (the controller never sees
it), and both sides read the same `TAU_MAX` — the demo pushes its value onto
`tau_max` in the scenario's YAML (robotic_arm/config/<scenario>.yaml).

`res["n_sat"]` counts saturated joints per step: the panel header shows
`SAT n/4` live, and the first occurrence raises a red `ARM SAT` event. Treat it
as a fault signal — the vehicle stays consistent, but the EE is tracking worse
than commanded.
