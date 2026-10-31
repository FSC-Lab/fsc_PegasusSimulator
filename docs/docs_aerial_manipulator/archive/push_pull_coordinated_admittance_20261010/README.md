# Coordinated admittance framework (Liu et al., CEP 2024) vs whole-body L1 — box pushing, 2026-10-10

Paper: Q. Liu, S. Lyu, K. Guo, J. Wang, X. Yu, L. Guo, "A coordinated framework of aerial
manipulator for safe and compliant physical interaction", Control Engineering Practice 146 (2024)
105898 (`~/Paper/.../references/[passivity] A coordinated framework of aerial.pdf`). Simulation only,
no hardware twin. The third external comparison controller after the geometric+L1 (Cai et al.) and
the modular adaptive (Yadav et al.) rigs, built the same way: everything is NEW FILES, the whole-body
code is untouched, the plant and the mission are the whole-body push campaign's.

## What exists

| piece | where |
|---|---|
| Python reference of the whole method (law, estimator, admittance, weights, kinematic MPC, ADMM QP; self-test, 11 checks) | `extensions/.../robotic_arm/utils_controller/coordinated_admittance.py` |
| C++ flight node (eqs 11-14 + eq 25), a parallel fork of the geometric+L1 client | fsc_autopilot_ros2 `fsc_autopilot_ros2_node/single_aerial_manipulator_coordinated_admittance_direct_actuation/` (`autopilot_coordinated_admittance_direct_actuation_node`, ns `fsc_autopilot_ros2/coordinated_admittance_direct_actuation`) |
| parity test (180 law cases + 900 estimator steps, 1e-9) + fixture generator | fsc_autopilot_ros2 `tests/src/test_coordinated_parity.cpp`; `application/robotic_arm/utils/generate_coordinated_truth.py` |
| the paper's onboard planner (admittance + weights + MPC), **C++ with the QP on OSQP** (v0.6.3, fetched at build time; no system package) | fsc_autopilot_ros2 `.../client_lib/{include,src}/coordinated_planner.{hpp,cpp}` + `coordinated_admittance_planner_node.cpp` (executable `coordinated_admittance_planner_node`) |
| its Python reference (same maths, OSQP when importable, else the built-in ADMM; the offline tools import it) | fsc_autopilot_ros2 `scripts/tools/coordinated_admittance_planner.py` (no longer started by the stack) |
| planner parity test: arm model / admittance / weights to 1e-9, the assembled QP (P, q, A, l, u) to 1e-8, the OSQP solution against a 1e-9 reference solve | fsc_autopilot_ros2 `tests/src/test_coordinated_planner_parity.cpp` (fixture block `planner`) |
| offline tools: MPC replay of a recorded run (`tools/replay_mpc.py runs/<tag>`), closed-loop sway/entry bench (`tools/sway_bench.py`) -- run with `PYTHONNOUSERSITE=1 PYTHONPATH=~/sources/pyosqp` for the OSQP backend | this directory |
| sim yamls (GENERATED from the 4-D whole-body ones: plant, SAFETY, allocator, planner sections verbatim) | `config/params_single_aerial_manipulator_coordinated_admittance_direct_actuation_t650_sim{,_push_pull}.yaml`; `tools/make_coordinated_yaml.py` |
| stack / Pegasus launcher / push-scene wrapper | fsc_autopilot_ros2 `scripts/isaacsim/start_coordinated_admittance_direct_actuation_t650_aerial_manipulator_stack.sh`; `scripts/indoor_sim/start_t650_aerial_manipulator_coordinated_admittance_direct_actuation_sitl.sh`, `..._coordinated_admittance_push_and_pull_sitl.sh` |
| arm config (position mode, arm_planner on this node's mode topic) | fsc_open_manipulator `open_manipulator_x_isaac_bridge/config/position_controller_isaac_aerial_cadm.yaml` |
| campaign runner / summary | `tools/run_pl_cadm.sh <tag>`, `tools/summarize_cadm.py runs/*.npz [--metrics]` |
| the mission driver (shared) gained `--da / --dbg-topic / --dbg-len` | `push_pull_20261003/tools/pl_mission.py` |
| the impedance scorer (shared) reads the admittance gains and the planner-commanded base | `push_pull_20261003/tools/pl_metrics.py` |

## The method as implemented (paper sections 3-4)

- **UAV controller** (flight node, 250 Hz): position PD with acceleration feed-forward (eq 11),
  flatness attitude from F_des and the yaw reference (eqs 12-13), PD attitude law in roll-pitch-yaw
  Euler coordinates mapped to a body torque through W(η) (eq 14). No arm model (Remark 3), no
  adaptation. Allocation through the geometric+L1 fork's wrench allocator (compiled from its source).
- **Momentum estimator** (eq 25 / Algorithm 1): second order, K1 = 20, K2 = 5 (the paper's), on
  χ = [m v; M_η η̇] with the ACHIEVED wrench, M_η = WᵀIW, C_ηᵀη̇ = ∂T/∂η in closed form. Published on
  `external_wrench_estimate` (force world, torque Euler-generalised). Sign of eq 25's inner bracket
  taken as Ruggiero et al. 2015 (the printed form does not reduce the lumped force).
- **Admittance filter** (eq 27) on the EE reference P_o = the trajectory planner's `r_ed` stream (the
  same mission the whole-body rig flies; only the EE task of that stream is read).
- **Adaptive weights** (eqs 31-32) as the planner's input weight W_u.
- **Kinematic MPC** (eqs 28-39): h = [P_b; q], double integrator with B = [0; Δt I], Δt 0.1 s,
  N_p 10, N_c 5, E_ter = K_p E_e + Ė_e at every step, the CoM term, the input term; boxes on x, u, Δu,
  the EE in the body frame, |P_c,xy| ≤ l_m/√2. EE and CoM maps linearised about the current q each
  solve. One QP per tick, solved by an ADMM written for it (no QP package on the flight computer).

## Where the paper is silent and what this rig chose (all printed at startup)

(a) **EE heading.** The paper plans the EE position only; the fin needs the claw's heading, so the
planner carries a quadratic term on q1 (ψ_e = ψ_b + q1 with q4 = 0). (b) **Norm constraints as
boxes** (inner approximation); the terminal cone is the cost. (c) **η̇_d = η̈_d = 0.** (d)
**Joint posture toward the streamed q_d** (w 1.0 on q2..q4): without it a 1 m EE transit parked q2
on its −18° stop with the base 16 cm past the goal (offline). (e) **Integral action on the UAV
position error** (K_Pi 1.5, clamp 1.0 m·s; 0 = the paper): the paper's PD parks ~8 cm low under the
mirror plant's ~1.4 N thrust-model error and the planner cannot absorb it (the arm's upward reach at
the push pose is ~3 cm). (f) **Reference-state MPC**: the planner integrates its own command and is
re-synced from the measurement only beyond 0.15 m / 0.2 rad (cadm_03: planned from the measured
position, the next-step command is a carrot 5 mm ahead of the vehicle). (g) **Admittance K_d 150
(xy) / 300 (z) N/m** instead of the paper's push-pull 1.0: a 1.2 N box reaction would move the paper's
reference 1.2 m, i.e. no push; the vertical channel carries the thrust-model error. (h) **Weights read
the horizontal force** (the vertical estimate is the thrust-model error, ~1.5 N, which flipped w_uav
at every hover). (i) **Joint box pre-processed** (Remark 2): q2 the real arm's [-20, 45]°, q3 ≥ 5°.
(j) **Integral action on the Euler attitude error** (K_ηi 1/1/0.5 N·m/(rad·s), clamp 0.5 rad·s;
0 = the paper): the paper's UAV controller has no arm model (Remark 3) and no integral, so the
arm's standing moment (~0.2 N·m pitch at the folded home) and the plant's −0.1 N·m yaw bias park
its PD at a steady Euler error τ/K_ηp — measured 3° pitch / 4° yaw in every hover of cadm_06..08,
i.e. the claw heading 4° off the fin, and (bench) a 10 cm standing position error the clamped
position integral cannot remove. Bench with the integral: standing error 11 mm, sway damping
unchanged (ζ 0.25–0.33), stable at 40 ms feedback delay.
(k) **Soft state/EE/CoM boxes** (one slack per group, quadratic penalty 1e6): hard boxes made the QP
infeasible whenever the commanded state sat outside them (cadm_08). A linear (exact) penalty term
is a kink OSQP crawls on — 2750 vs 50 iterations per hover tick for the same plan — so it is 0.
(l) **x0 leash instead of a jump re-sync** ("full" mode): the command is pulled toward the
measurement by exactly its excess over 0.15 m / 0.2 rad and keeps its own velocity; a QP the
solver does not certify holds the last command. The wrist input weight w_u,q4 is 0.2 (eq 32 gives
q4 no adaptive weight and it is coaxial with the grasp point: at 0.02 its acceleration was a flat
direction of the cost, solver-dependent by ±2 rad/s²).
(m) **The arm compensates the base's tracking error** ("full" mode): the published joint command is
the MPC's plus the damped least-squares joint step Δq = J_e⁺ Rᵀ(p_cmd − p_meas) (error low-passed
0.1 s, result clipped to the joint box), so the MEASURED base + arm lands on P_r while the PD still
pulls the base to its command. This is the paper's measured-state MPC coordination (eq 28 from the
measured h) kept with the reference-state base command cadm_03 needed; without it the claw rode every
base excursion (cadm_11). It must stay OUTSIDE the MPC: offsetting the MPC's EE reference by the error
let the MPC move the base command by it as well — positive feedback, divergent in 2 s (cadm_12).
With a 0.1 s low-pass the arm chased the 0.33 Hz sway to its joint limits and its reaction sustained
it (cadm_13, ±180 mm limit cycle), and at 1.0 s (45 % gain, 90° lag at the sway frequency) the same
cycle persisted (cadm_14) — the term is OFF in the shipped yaml: on this plant the arm's reaction
onto the softly damped base is a loop the paper's decoupled design has no term for.
(n) **The paper's own torque estimate fed back**: τ_η −= gain · LPF(τ̂) (eq 25's estimate, gain 1,
0.1 s). The paper uses the estimate only for the admittance; on this plant a ~1 N claw force 0.3 m
below the CoM is a 0.3 N·m torque the integral-free PD answers with the 5° tilt seen at every contact
(cadm_11/13). Bench, sustained 0.3 N·m step: tilt 7.6 → 4.1°, excursion 14 → 9 mm — but in flight
(cadm_15) it formed a growing ~1 Hz attitude oscillation and a flip: the bench's rigid body has no
arm, the estimate carries the law's inertia mismatch with it, and unity feedback against the rotor-lag
pole is unstable. OFF in the shipped yaml (gain 0 = the paper).
(o) **The arm is held in the contact phase** (the mission's contact flag, below):
the posture term holds all four joints at their pose at contact onset (weight 10) and the EE-heading
term (addition (a)) is dropped. With the claw clamped to the box the chain base–arm–box–table is
closed, so a joint motion moves the BASE: the heading term's q1 counter-rotation yawed the base
exponentially (cadm_19/20). The paper's eq 32 already penalises the ARM INPUTS at large force; this
is the state-side counterpart.
(p) **A contact phase from the mission, a tared estimate, a claw-frame contact admittance.** The
trajectory planner's contact flag (the same SetBool the whole-body node receives; here from the jaws'
stall to the Exit press) reaches the planner node on `coordinated_planner/set_contact`. (i) eq 25's
estimate is the LUMPED external force — on this plant a standing ~0.7 N horizontal / ~2 N vertical
model error — so it is tared: its 2 s low-pass in the body-yaw frame while the flag is off, frozen
while it is on; the admittance and the weights read F̂ − tare. (ii) The admittance gains are in the
claw frame [along, lateral, vertical]: 150/150/300 N/m free, **150/20/20 in contact** — a box dragged
18 mm sideways at the grasp is then 0.36 N on the claw instead of its full 1.1 N friction, whose
0.29 N·m yaw moment (0.26 m claw lever) exceeded the plant's yaw authority (cadm_19/20), and the
altitude loop no longer hauls a gripped box up (the 4–6 cm lunge at every grasp). This is the paper's
admittance doing its job; the stiff isotropic 150 of (g) was the price of an untared estimate.

## Flight log (Isaac, headless, RTF 1, raw mocap, the mirror plant, config A)

| run | change | outcome |
|---|---|---|
| cadm_01 | paper-literal planner (measured state, u_max 1.5) | DIRECT entry: ±10° attitude swing within 1.6 s, 15° watchdog. The first-step acceleration chattered ±u_max at 20 Hz on the mocap velocity. |
| cadm_02 | commanded velocities in x0, u_max 0.6, xy-force weights, z admittance 300, integral 1.5/0.3 | hover held (tilt ≤ 1.4°, F̂ = the plant's +0.55/−0.52 N bias); claw 55–90 mm short of the start point for 45 s (integral on its clamp, admittance 12 mm), Ready refused |
| cadm_03 | integral clamp 1.0, K_d xy 150 | reached Ready (go-to-start accepted), descent knocked the box 38 mm: claw 50 mm behind the reference for 20 s, then overshoot |
| cadm_04 | reference-state MPC (f) | start point reached at once; the 3 s descent lagged 33 mm then overshot 34 mm along the approach, box knocked 83 mm |
| cadm_05 | K_p 2 → 4, planned-acceleration feed-forward on | descent clean, close step reached; the claw arrived 17 mm short / 15 mm high (= the base lagging its own command), the near jaw tip landed ON the fin, the fin's friction shoved the base 9 cm back, the PD (20 N/m) overshot 14 cm into the box, tilt watchdog |
| cadm_06 | planner descent trim on (3 s window), integral clamp 0.5, K_Pd 3.5 (the 6 s dwell did not reach the driver) | trim +13 mm z applied, descent clean, arrival gate passed, jaws closed on NOTHING: the claw wobbled ±10 mm at ~1 Hz around the grasp point (the PD cascade's lateral sway) and lagged 9 mm high at the end of the descent |
| (cadm_06 measured) | — | over the 4 s hold at the fin the BASE wobbles 9 / 11 / 2.5 mm std (x/y/z) at 0.50 Hz while the planner's base command holds to 2 mm and the joints to 0.3°: the PD-plus-attitude cascade's lateral sway mode (the decoupled geometric rig's known ζ≈0.1 mode at 0.5–0.6 Hz with the 20/11 N/m pair, fixed there by 15/10) |
| cadm_07 | horizontal PD 4.0 / 2.67 (the decoupled rig's pick-and-place pair), vertical 8.0 / 5.0, 6 s dwell | descent clean, gate passed, jaws closed on nothing: the base still swung ±40 mm at 0.5 Hz through the descent, the claw pressed the fin at arrival and walked the box 9 cm before the close (the gate measures the claw against the CAPTURED box pose) |
| (bench) | `tools/sway_bench.py` | reproduces the flights: the 5.37/2.95 pair ζ≈0 at 0.5 Hz, 4.0/2.67 ζ 0.2; stiffer attitude pairs (K_ηp 5–6.5) damp it but diverge at 40 ms feedback delay; **2.5/2.5 per kg with K_ηd 1.2** keeps ζ 0.45–0.55 and a 6–9 mm hold at 0/24/40 ms |
| cadm_08 | PD 2.5 / 2.5 (xy), 8 / 5 (z), K_ηd 1.2, K_Pi 1.0 | vehicle wandered 3.6 m during the dwell, aborted. **Root cause found in the record, not the gains**: at DIRECT entry the vehicle surges to 0.3 m/s (the integral-free attitude PD under the arm's moment), crosses the 0.15 m re-sync at 1.2 s, the re-sync JUMPS x0 162 mm and hands the MPC the measured 0.28 m/s velocity, the solver hits its 400-iteration cap and from then on 91 % of ticks are uncertified: u0 chatters ±u_max, q commands slam between the joint stops, 346 re-syncs. The offline replay of the same tick stream with the recorded references is clean (|P_r−P_e| 8 mm rms, 60 iterations) — the planner was never asked a sane question. |
| (offline) | `tools/replay_mpc.py`, `tools/sway_bench.py` | the Python ADMM could not solve the soft-box QP (400-iteration cap, 14 ms); on the user's request the planner was ported to C++ with OSQP (hover tick: 50 iterations, 0.3 ms) — additions (j)–(l) above |
| cadm_09 | C++/OSQP planner node, K_ηi 1/1/0.5 (clamp 0.5), leash, hold-on-unconverged, soft boxes, w_u,q4 0.2 | stable all the way to the fin, no watchdog; planner 53 iterations / 1.1 ms per tick, 560/560 certified, 0 leash events in the hold; e_η settles to 0.1° (the integral); claw on its reference to 2 mm — **but 27 mm BESIDE the fin**: the 3 s descent trim sampled the base's slow sway (±34 mm std) and shifted the goal 27.6 mm, the gate (3 mm across) never passed. Also the claw 9 mm low: the z integral on its 0.5 m·s clamp against the ~2 N thrust-model error. |
| cadm_10 | descent trim OFF (the whole-body planner block verbatim), integral clamp 1.0 | standing hold now clean, but the go-to-start (whole-body bounds 0.30 m/s / 0.15 m/s²) arrives with the base 70/120 mm behind its command and swinging (±35 mm, ~0.4 Hz, still 17 mm after the 6 s dwell); the 3 s descent then runs the lagging claw (+35..80 mm in y) onto the fin from the side, box disturbed 16 mm, abort. The base's tracking lag is the decoupled PD's structure: stiffer gains undamp the cascade's sway (bench). |
| cadm_11 | planner bounds for this rig 0.20 m/s / 0.08 m/s² / 0.20 rad/s (gentler transits and descent), hover dwell 15 s | descent clean (box untouched), gate passed with the claw centred on the fin to 3 mm — then, with the jaws still open, the base swung 150 mm along the push axis (a 5° pitch swing of the ~0.4–0.6 Hz sway mode, excited by the contact) and dragged the box 154 mm; the jaws closed on nothing. In "full" mode the arm never sees the base's tracking error, so the claw rides every base excursion. |
| cadm_12 | addition (m) as an EE-reference offset INSIDE the MPC | diverged 2 s after DIRECT entry: the MPC moved the BASE command by the error as well (cheaper than the arm at w_uav ≈ 0.95), the PD saw twice the error, 4 m off in 6 s, SAFETY revert. |
| cadm_13 | addition (m) as an arm-only joint correction OUTSIDE the MPC (0.1 s low-pass) | ±180 mm, 0.33 Hz limit cycle from DIRECT entry: the arm chased the sway to its joint box every half-cycle and its reaction sustained it (e_η ±12°); SAFETY revert in the dwell. |
| cadm_14 | compensation low-pass 1.0 s; addition (n) τ̂ feedback, gain 1 / 0.1 s | the same ±180 mm limit cycle (45 % gain with 90° lag at the sway frequency still pumps it); SAFETY revert at 25 s. The τ̂ feedback never acted: its gain was declared in only one of the two parameter-schema `members()` overloads and loaded as 0 — found by running the node standalone on the yaml and reading its summary. |
| cadm_15 | arm compensation OFF (addition (m) retired on this rig); addition (n) actually loaded, gain 1 | growing ~1 Hz attitude oscillation from entry (e_η ±5 → ±12°, τ̂ ±1 N·m in antiphase with the commanded torque), flipped at 8 s. The estimate carries the law's inertia mismatch with the arm; fed back at unity against the rotor-lag pole the loop is unstable. Addition (n) retired (gain 0 = the paper). |
| cadm_16 | cadm_11's configuration again (compensation and torque feedback both off, attitude integral on) — a repeat, and the driver's close dwell confirmed at its 0.3 s default | start-point hold now quiet (base std 7/4 mm), descent clean, gate passed at 10.6 mm / 2.9 mm across — then the moment a finger touched the fin the vehicle drove INTO the box (−y at up to 140 mm/s, box pushed 132 mm, recoil 290 mm/s) and the jaws closed on nothing. Mechanism: the ~1 N contact 0.3 m below the CoM is a 0.3 N·m pitch moment; the attitude PD at K_ηp 3.337 answers with 5° of nose-down tilt toward the box, and the thrust follows. |
| cadm_17 | K_ηp 5.0 / K_ηd 1.6 (bench-safe; contact tilt 5 → 3.4°), K_ηi 2.0, arrival tolerance 12 mm (fingers straddle the fin rather than press it) | **attitude now holds**: e_η std 0.3/0.4°, no drive into the box, base std 3/17/2 mm at the fin (the 17 mm is a slow sway ALONG the fin), claw on its reference to 1 mm mean. The gate (12 mm / 3 mm across vs the planner's captured grasp point) timed out at 15 / 4.7 mm: the finger pads brushing the fin faces dragged the box 38 mm along the fin with the sway, so the captured point walked away while the claw was on the fin. |
| cadm_18 | the driver gates the close on the measured claw vs the LIVE box (`--grip-live-box`: 4 mm across the fin, 25 mm along its 60 mm width) — what an operator watching the box does; no controller change | **first grasp and push**: gate passed in 0.3 s (6.5 mm along / 2.4 mm across), jaws stalled on the fin, the slide started — then the base yawed 0 → −9° over 7 s (lateral sway × the 0.25 m arm lever ≈ 0.3 N·m of yaw against K_ηz 1.74), the claw swung sideways, the box rotated and the vehicle tilted to the 15° watchdog 10 s into the push. Box moved 56 mm during the 1.5 s settle (the vehicle lunged at the jaw stall). |
| cadm_19 | yaw K_ηp 1.737 → 4.0, K_ηd 0.41 → 0.8, K_ηi 0.5 → 2.0 (inside the 0.4 N·m yaw clamp) | grasp again in 0.3 s; yaw now within ±5° through the push — but the base's ±30 mm lateral motion rotated the held box (−8° in 10 s) and dragged it sideways 63 mm, the box's reaction swung the claw 250 mm and the lateral loop let go at 11 s (tilt 21°). Also the vehicle runs 30–58 mm ahead of its command through the push. |
| cadm_20 | position pair 2.5/2.5 → 4.0/3.5 per kg (bench: ζ 0.21–0.24 under the 5.0/1.6 attitude loop) | grasp in 0.4 s; the yaw error then grows EXPONENTIALLY through the push (−3 → −19° in 3 s) while the MPC counter-rotates q1 by +17° to hold the claw heading: with the claw clamped to the box the chain base–arm–box–table is closed, so the joint motion yaws the BASE, which the heading term answers with more q1. The box rotates and is thrown sideways; tilt 15° at 9 s. |
| (cadm_20, fine trace) | — | from the grasp on, the yaw torque command sits ON its 0.40 N·m clamp (motors 0.39/0.39/0.81/0.80; the mirror delivers 0.70 of it) against an estimated −0.41 N·m yaw disturbance: the box, dragged 18 mm sideways at the grasp, puts its full 1.1 N friction on the claw laterally × the 0.26 m claw lever. Yaw runs away at −7°/s; the q1 heading loop only follows. Also: the claw sits 10–18 mm low at the fin (thrust sag vs K_Pi,z 1), and the altitude loop pulling up on the gripped box pitches the nose down — the 4–6 cm lunge at every grasp. |
| cadm_21 | additions (o) + (p): mission contact flag (`coordinated_planner/set_contact`, from the jaws' stall to the Exit press) → force tare frozen, admittance in the claw frame K [150, 20, 20] / D [17.3, 20, 20] (compliant across the push and vertically), arm held, heading term off. F_max 1.5 on the tared force, yaw clamp 0.55, K_Pi,z 3 | **COMPLETE — the first full mission on this rig.** Grasp in 0.3 s, contact flag on through the push, arm held (joint span ≤ 3°), box pushed **521 mm** (8 mm sideways, yaw −8.5°), released, exited, landed. Slide: tared force +0.5…+1.0 N along the push, |e_R| ≤ 2°, ρ_UAV max 0.145 / rms 0.07, EE 26 mm rms (the claw lags the task reference by 20–40 mm — the base's tracking lag), 13 mm low. Impedance residual 3.0 N rms in the slide, 2.95 of it ALONG the push: the rig does not render its 150 N/m there, the base lag masquerades as compliance. Planner: 66 iterations / 1.2 ms per tick, 0 uncertified. |

## Results (campaign: `results/utils/run_box_pushing_campaign.sh` with `METHOD=coordinated_admittance`)

Report page (private artifact, 2026-10-10): https://claude.ai/artifact/FhKMmqvsmBZJsJBECb85J1 — "Box Pushing, Two
Controllers": the results table, the four figures, the mechanism and the caveats. Campaign: 3/3 at the first
attempt, box 525 / 521 / 518 mm.

The shipped configuration is cadm_21's: the generated `..._sim_push_pull.yaml` (additions (a)–(e), (g)–(l),
(o), (p); (m), (n) retired), planner bounds 0.20 / 0.08 / 0.20, hover dwell 15 s, the driver's live-box
grasp gate (4 mm across, 25 mm along the fin). The campaign flies three runs on the driver's timetable
(`--slot-ready 30`, the rest the whole-body slots), copies them to
`results/simulation_results/Isaac_simulation_data/box_pushing/coordinated_admittance_push_run<k>.npz` and
`build_box_pushing.py` scores both methods with one definition (see `results/simulation_results/README.md`
and `matlab_simulation_data/box_pushing/`).

What the comparison measures, on the same plant, scene, mission and timetable:
- the whole-body 4-D L1 law renders its impedance in the slide to 0.66 N rms (K_y 120 N/m, claw
  deflection 10 mm = F/K_y, box 492–498 mm, ρ_UAV 0.047 rms);
- the coordinated framework pushes the box as far (≥ 0.5 m) but its rendered impedance along the push
  is off by ~3 N rms: the paper's admittance (eq 27) presumes the UAV tracks the planned base motion,
  and this rig's decoupled PD base lags its command by 20–40 mm under a 1 N load — a lag the kinematic
  MPC cannot see (it plans from its own command, addition (f)) and the admittance renders as if it were
  compliance. Lateral and vertical, where the contact gains are 20 N/m, the residual is 0.3–0.5 N.
- ρ_UAV (the airframe's departure from its planned motion, L = 0.37 m) is 2–3× the whole-body rig's.

What it took to get here (the flight log above, 21 flights): the paper's framework as printed could not
fly this plant — no attitude integral under the arm's standing moment (j), an untared lumped force
estimate that kept the admittance stiff (p), and a free-flight MPC that keeps moving the arm in contact
(o) were each a failed mission; the additions are the paper's own ingredients applied where the paper
is silent, every one documented and switchable (0 = the paper).
