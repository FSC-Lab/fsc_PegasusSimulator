# Key Items

Decisions and gotchas on the aerial manipulator that are **easy to get wrong and
expensive to rediscover**. Everything here is measured, not assumed.

See also: [Flight State Machine.md](<Flight State Machine.md>) (phases and gates),
[Command.md](Command.md) (the showcase flight), [Hover Gain Tuning.md](<Hover Gain Tuning.md>).

---

# 1. Arm home pose — folded, singularity-certified, spawned there

The pose the arm holds whenever it is **not** doing task work: on the ground,
through takeoff, and through the whole landing descent. It leaves it only for
fly-in → pinned → fly-out, and folds back before descending.

**HOME = q [0, 40°, 46°, 0]** — β = q₂+q₃ = 86°, split 0.465.

| Pose | q (deg) | Lowest arm point | Which point | Tool reach | σ_nd |
|---|---|---|---|---|---|
| hanging (old spawn) | [0, 0, 0, 0] | −0.282 m | gripper pad | 0.279 m | 0.381 |
| **HOME** | **[0, 40, 46, 0]** | **−0.165 m** | **elbow (O₂)** | **0.139 m** | **0.529** |
| β = 88° | [0, 40, 48, 0] | −0.165 m | elbow | 0.134 m | 0.530 |
| β = 96° | [0, 48, 48, 0] | −0.150 m | elbow | 0.123 m | 0.530 |
| sketched fold (link 2 horizontal) | [0, 79, 44, 0] | −0.083 m | elbow | 0.088 m | 0.398 |

σ_nd = non-dimensional σ_min of the arm task Jacobian J_3y⁰ (translational rows
scaled by 1/L_char) — the metric `refs_matlab/utils/utils_singularity/singularity_sweep.m`
certifies against, **safe ≥ 0.10**. Home is 5.3× that margin. At the measured
0.305 m resting body height, ground clearance is **0.140 m folded vs 0.023 m
hanging**.

### The sign insight (the thing that inverts the MATLAB figure)

The MATLAB model's joints 2/3 rotate about **+y** (positive q pitches *down*);
`AM_realign`'s rotate about **+x** (positive q swings *up*). So the certified
singular branches — at *positive* q₃ ≈ +42° in MATLAB — appear at **negative
q₂/q₃ on this asset** (deepest valley q₃ ≈ −52°; the measured cond ≈ 880 point at
q₂ = q₃ = −43° is that elbow branch). The whole positive-fold side is clean.

Consequence: **compact, ground-safe, and best-conditioned are the same pose** —
folding up toward the body both shortens the downward reach and raises σ_nd.
That alignment is not automatic; do not assume it survives an asset change.

### Why not fold tighter — three walls, in binding order

1. **Joint stop.** q₂ and q₃ share an authored **+50°** limit. The sketched
   link-2-horizontal pose needs q₂ ≈ 79° — unreachable, *and* worse conditioned
   (σ_nd 0.398 vs 0.529).
2. **Past home the binding point is the ELBOW, not the tool** (tool is tucked at
   −0.038 m), and **elbow height depends on q₂ alone** — q₃ does not move it.
   β = 86° and β = 88° have identical −0.165 m lowest points. Buying 1.5 cm more
   costs q₂ 40° → 48°.
3. **That q₂ travel is the handover margin.** The hold → law handover overshoots
   **q₂ by ~7–8° but q₃ by ~1°** (measured). q₂ = 48° at rest would leave nothing.

Meanwhile σ_nd has **saturated** (0.529 / 0.530 / 0.530 across β = 86–96°), so
there is no conditioning left to gain. Both objectives are at their ceiling.

The asymmetric **split 0.465** exists for wall 3: equal split wastes q₃'s unused
headroom. It buys reach 0.152 → 0.139 m at the *same* q₂ = 40°, σ_nd 0.521 →
0.529. Measured flight peaks: **q₂ 47.2° (2.8° margin), q₃ 46.4° (3.6°)**.
β = 84° same split gives ≈ 2.9°/4.0° margins at 0.142 m if more margin is wanted.

**No Z-fold.** Link 3 is a **+x** offset while links 2/4 run **−z**, so
counter-rotating (q₃ < 0) *straightens* the chain: q₂ = +50°/q₃ = −30° gives
0.273 m reach and σ_nd 0.23 — worse on both — heading into the elbow branch. By
q₃ = −60°, σ_nd = 0.057, **below the 0.10 keep-out**.

### Spawning in the home pose

`02_aerial_manipulator_free.py` spawns the arm directly at the pose the takeoff
hold will command, instead of hanging at q = 0 and slewing up during the climb
(which left the gripper pad 2.3 cm off the floor and swung the arm 64° while low).

- **`Q_SPAWN`** ← `_takeoff_arm_pose()`: the plan's own `q0` for
  `compatible*` (via the cached pure-numpy `get_plan`, so it resolves before
  Isaac starts and `build_traj` reuses the same solve), `q_hold` for
  `circle_bent`, zeros otherwise. **Single source with the hold target** — they
  cannot drift apart.
- Written **after `Articulation.initialize()`** via `set_joint_positions`, then
  a rigid re-seat (below).
- `ARM_HOLD_RATE` (hold-target slew) is now a no-op on a normal takeoff; kept for
  the hold's re-engagement paths.

### ⚠ Two PhysX gotchas — read before ANY joint teleport on this asset

**1. Authoring `PhysicsJointStateAPI` attributes before `world.reset()` does not
take effect.** The pre-reset `world.step()` calls in `_wait_for_prim` let PhysX
capture its reset snapshot at the parsed q = 0, and `world.reset()` restores that
over the authored values. Measured **q(t₀) = 0** — a *silent* failure, no error.
Write through the articulation API after `initialize()` instead.

**2. PhysX roots this asset's tree at an ARM link, not the body.** So
`set_joint_positions` swings the **body around the arm**. Measured: rolled
**−86.0° about x** (= −(q₂+q₃), the giveaway) and **0.207 m displaced**; the
reference then chased a sideways thrust axis and it tumbled at t = 1.0 s.
Fix = **rigid re-seat** right after the teleport: read the body pose before/after,
form `R_corr = R_a·R_bᵀ`, `p_corr = p_a − R_corr·p_b`, left-apply to the
articulation root via `set_world_pose`. Exact no-op on a body-rooted tree.

*(Related: the same physics-vs-render split means a teleport is invisible in the
viewport until a physics step runs — see `START_PAUSED` in Flight State Machine.)*

### Validation (34 s headless showcase, vs the old hanging start)

| Metric | hanging | **spawn at home** |
|---|---|---|
| max \|q − home\| during climb | 64.1° / 53.6° | **2.0° / 0.6°** |
| lowest arm point, world z, first 6 s | 0.188 m | **0.432 m** |
| EE error during takeoff (mean / max) | 406 / 1031 mm | **57 / 144 mm** |
| q₂ / q₃ peak, whole flight | 47.9° / 48.1° | **47.2° / 46.4°** |

Landing unaffected: pinned EE 0.43 mm mean / 1.17 mm max, touchdown at exactly
0.305 m, zero drift, arm back at home.

### Scope

- **Only `02_aerial_manipulator_free.py` spawns at home.** `01_..._track.py`,
  `02_..._pick.py`, `..._push.py` and `px4_direct_actuator_...py` still spawn
  hanging. Port `Q_SPAWN` **and** the rigid re-seat together, or not at all.
- **Headless-validated only** — per the 2026-07-31b lesson a rendered run is the
  true gate (rendered stepping is bursty and wakes modes headless timing misses).
- If the USDA is re-authored (link 2 length, q₂ limit, body frame), **re-check the
  elbow, not the tool tip** — it is the binding point now.

### Reproducing the geometry numbers

Pure numpy, no Isaac. The `cd` is required (`fsc_aerial_manipulation` is only
importable with that directory as cwd):

```bash
cd extensions/fsc_aerial_manipulation && python3 - <<'EOF'
import numpy as np
from fsc_aerial_manipulation.robotic_arm.utils_planner import compatible_trajectory as CT
from fsc_aerial_manipulation.robotic_arm.utils_controller.controller import make_params, joint_rotation

P = make_params(); Lchar = 0.5*sum(np.linalg.norm(l) for l in P["l_i"])
S = np.diag([1/Lchar]*3 + [1.0]); PAD = np.array([0., 0., -0.0494])   # pad midpoint, EE frame

def probe(q_deg):
    q = np.radians(q_deg); R = [np.eye(3)]
    for i in range(4): R.append(R[i] @ joint_rotation(P["h_i_im1"][i], q[i]))
    O = [np.zeros(3)]
    for i in range(1, 5): O.append(O[i-1] + R[i] @ P["l_i"][i-1])
    pts = O + [O[-1] + R[-1] @ PAD]
    return min(p[2] for p in pts), np.linalg.norm(O[-1]), \
           np.linalg.svd(S @ CT._J3y(q, P), compute_uv=False)[-1]

for name, qd in [("q=0", [0,0,0,0]), ("home b=86", [0,40,46,0]),
                 ("b=96", [0,48,48,0]), ("sketch", [0,79,44,0])]:
    z, r, s = probe(qd)
    print(f"{name:12s} lowest {z:+.3f} m  reach {r:.3f} m  sigma_nd {s:.3f}")
EOF
```

The planner also certifies every plan it generates — `min sigma_nd` is in its
startup summary and it warns below 0.10.

---

# 2. Arm torque saturation — clamped in two places, for two different reasons

`TAU_MAX = 4.1 N·m` — XM430-W350 **stall** torque at 12 V. Note that is *stall*,
not usable continuous torque; consider lowering it for the real rig.

The two arm-torque sources are clamped in different places, and this is **not**
redundancy:

| Source | Computed in | Clamped in | Coupling rebuild? |
|---|---|---|---|
| Whole-body law | controller | **controller** (+ demo, idempotent) | **yes** |
| PD hold | demo | **demo only** | no — `u3 = 0` during hold |

Both read the same number: the demo sets `C.TAU_MAX = TAU_MAX` at startup.

### Why the law's clamp cannot move to the actuator

`tau_body` carries the arm's reaction `N1ᵀ·u3`, and the rotors are commanded
**before** the arm torque is ever clamped:

```python
res   = self.ctrl(...)                          # tau_body computed here
omega = mixer.mix(res["thrust"], res["tau_body"])
backend.input_ref[i] = omega[i]                 # rotors already committed
...
tau_j = np.clip(tau_j, ±TAU_MAX)                # arm clamped only now
```

A clamp at the actuator can never inform `tau_body` — by then the mixer has
already turned an unclamped `N1ᵀ·u3` into rotor speeds, leaving the rotors
compensating a torque the servos never produced. That is the documented flip
mechanism. **The law's clamp is a model correction, not an actuator limit**, so
it must live where `T`, `A`, `N1` do.

The PD hold has no such dependency: during a hold `u3 = 0`, so `tau_body` carries
no arm reaction and nothing but the servos reads the hold torque. A pure actuator
limit, correctly placed at the actuator.

### Clamp `tau_joint`, not `u3`

```
tau_joint = τ[6:] = u1·Aᵀe3 + u3
                    └────────┘  thrust recruitment
```

The servo limit applies to the **total**, so the clamp goes on `tau_joint` and
the realized `u3` is *derived* from it (`u3 = tau_joint − u1·Aᵀe3`), then `τ` is
rebuilt. Clamping `u3` directly would not bound what the servo sees: at hover
`u1·Aᵀe3 ≈ [0, 0.66, 0.18, 0]` N·m, so a `u3` capped at 4.1 could still demand
~4.76 N·m from joint 2.

### Ordering: clamp before the GMO integrates `u`

The observer advances `p_hat` from the applied wrench. If it saw the *demanded*
`u`, saturation would be misread as an external disturbance and the GMO would
fight it. The clamp runs first — verified.

### Reporting

`res["n_sat"]` counts the law's clamped joints; the demo adds the hold's. The two
are disjoint, so they sum. The panel header shows `SAT n/4` live and the first
occurrence raises a red `ARM SAT` event naming the source (`law` / `hold`).
**Treat it as a fault signal** — the vehicle stays consistent, but the EE is
tracking worse than commanded.

### Scope

`controller_track.py` still has the
original gap — they clamp downstream of `tau_body` with no rebuild.
`controller_hover.py` and `controller_free.py` are correct.

### Measured

| Case | Result |
|---|---|
| Settled hover | `n_sat = 0`, max \|τ\| 0.664 — behaviour unchanged |
| 1.17 m landing step (the bug that motivated this) | demanded **90.8 N·m**, 2 of 4 joints clamp; `tau_body` matches the realized `u3` to 0.0e+00 |
