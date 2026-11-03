# Task: make the simulated closed loop independent of simulator speed, then real-time if needed

## Background

In software-in-the-loop setups, the physics simulator advances in **simulated time**. The rest of the loop
often runs on the operating system's **wall clock**: flight-controller firmware in SITL, middleware bridges,
estimators, controllers, trajectory planners, actuator and arm controllers, sensor or motion-capture
emulators, and mission or test drivers.

When the simulator runs slower than real time, every wall-clock component sees a plant moving in slow
motion:
- references are flown faster in plant time than intended
- filters, observers and integrators run at the wrong bandwidth relative to the plant
- transport delays shrink
- dwells, watchdogs and timeouts expire early

The symptom: identical code and gains succeed on hardware but fail or degrade in simulation, and the
simulated errors grow as the simulator gets slower. "Compensations" that rescale a few plant parameters by
an assumed real-time factor (RTF) only patch part of this.

Your job: find out whether this system has the problem. If it does, fix it so that **the simulator's clock
is the only clock in the loop**. Then, if throughput matters, make the simulator run in real time.

## Rules

- Investigate first. Report findings and a proposed design, and **wait for approval before editing**.
- **Hardware behaviour must not change.** Every change sits behind one opt-in simulation switch whose
  default reproduces today's behaviour exactly. Don't touch hardware launch paths or hardware configs.
- Don't change control laws, gains or plant parameters. This is timing plumbing, and changing anything else
  breaks the comparison with real data.
- Read each repository's own documentation (README, CLAUDE.md, design notes) before editing. Keep the
  existing unit, parity and loopback tests green with the switch off and on.
- Commit or push only when the user says so. Report honestly: if a component can't be put on simulated
  time, say which one and what it costs.

## Phase 1: check (read-only)

1. **Measure the RTF** over a full representative run, not just the first seconds:
   - simulated time elapsed / wall time elapsed, from the simulator's own statistics or from its clock or
     message stamps against the wall clock
   - mean and minimum RTF
   - the distribution of wall time per physics step against the physics step size (mean, p99, worst)
2. **Inventory the closed loop.** List every process that sits between "sensor data leaves the simulator"
   and "actuator commands come back": simulator, firmware SITL, bridges and agents, estimators, controllers,
   planners, actuator and arm control frameworks, sensor emulators, test drivers. Recorders matter for
   analysis.
3. **Find each component's time source.** Search the code. Typical patterns:
   - ROS 2 C++:
     - `create_wall_timer` = wall
     - `rclcpp::create_timer(node, node->get_clock(), ...)` = node clock
     - `std::chrono::steady_clock` and `system_clock` = wall
     - `rclcpp::WallRate` = wall; `rclcpp::Rate` = check
     - `now()`, `get_clock()->now()` = node clock (sim time only with `use_sim_time`)
   - ROS 2 Python:
     - `node.create_timer` = node clock
     - `time.time()`, `time.monotonic()`, `time.sleep()` = wall
     - rates and sleeps in loops = check
   - Other languages and frameworks: the equivalent of each.
   - **Where `dt` comes from** in filters, observers and integrators: a fixed period, a wall-clock delta or
     message stamps.
   - **Every watchdog, freshness gate, dwell and timeout** that changes behaviour.
   - The **firmware SITL time mode**: lockstep, or free-running on wall time. If lockstep is disabled,
     find out why from comments, docs or history.
   - Whether the simulator **publishes a clock**, and which clock **stamps** its messages.
   - Any **RTF compensation or time-scaling** hacks, and what they assume.
4. **Decide**, and include the evidence in your report:

   | finding | decision |
   |---|---|
   | RTF holds ~1.0 sustained over a full run (p99 step time under the physics step) | fix optional; note the risk. Go to Phase 4 only if the user wants robustness to slower machines |
   | RTF below ~0.98 or unstable, **and** any closed-loop component on wall time | **fix required: Phase 2** |
   | every closed-loop component already on simulated time | no clock fix; RTF only affects throughput, so go to Phase 4 if speed matters |

## Phase 2: one master clock (only if Phase 1 says required, after approval)

1. The **simulator publishes its clock** every physics step, and stamps every message it publishes with
   simulated time.
2. **Every closed-loop node** runs with simulated time in simulation mode (`use_sim_time` or equivalent):
   - timers on the node's ROS clock, not wall timers
   - direct steady or system clock reads replaced by **one clock abstraction per codebase**: steady on
     hardware, the ROS clock in simulation. No scattered conditionals.
   - `dt`, watchdogs, freshness gates, dwells and timeouts all on that same clock
3. **Firmware SITL** runs locked to simulated time (lockstep or equivalent):
   - Reconcile the firmware-to-middleware timestamp synchronisation.
   - Verify heartbeat and offboard timeouts and estimator behaviour.
   - If lockstep was disabled for a stated reason, show that the reason no longer holds once every peer
     runs on simulated time, or document why it still does and what that costs.
4. **Actuator and arm control frameworks** (for example ros2_control): check what clock their update loop
   uses under simulated time. Every timeout that decides behaviour must use simulated time.
5. **Disable the RTF compensation hacks** in simulation-time mode. They become wrong once every component
   shares the plant's clock.
6. **Tooling:** recorders use simulated time; test drivers' dwells and timeouts use simulated time.
7. All of it behind **one opt-in switch**, propagated to every process. Where a terminal multiplexer or a
   launcher starts processes, make sure the switch actually reaches them: an exported variable often
   doesn't reach an already-running session.

## Phase 3: validate (required before anything uses the simulation for decisions)

1. **Rates:** every loop runs at its nominal rate measured against the simulated clock.
2. **RTF invariance (the decisive test).** Run the same scenario with the simulator deliberately slowed (for
   example to RTF ~0.5 with a per-step sleep) and unthrottled, at least 3 repeats each. Task outcome and
   tracking metrics must agree within run-to-run scatter. If they don't, something is still on wall time;
   find it.
3. **Reproduce real data.** Pick a recorded hardware run and simulate it with the flown code, gains and a
   matched scene. Score both with **identical analysis code**:
   - task outcome
   - tracking RMS per mission phase, peak errors
   - actuator usage, judged by **sustained** saturation (time continuously at the limit) and percentiles,
     not by the single largest sample of a noisy command

   At least 3 repeats. One run proves nothing near a stability boundary.
4. **Tests:** the existing suites pass with the switch off (hardware-identical) and on.

## Phase 4: real-time performance (if needed for throughput, or Phase 1 found the fix optional)

1. **Profile the simulator step** with its built-in profiler or a sampling or deterministic profiler, plus
   physics-engine statistics. The budget per step is the physics step size. Break it down: physics solve,
   per-step scripting and callbacks, middleware publish and executor spin, firmware bridge, sensor models,
   allocations.
2. **Candidates.** Apply one at a time, each opt-in, and re-profile after each:
   - Publish topics at the rate their consumers need, not the physics rate.
   - Spin executors fewer times per step.
   - Remove per-step allocations in hot paths.
   - Drop simulated sensors nothing fuses.
   - Tune physics: solver iteration counts, continuous collision detection only where needed, sleep
     thresholds, worker threads.
   - Replace high-polygon collision meshes with simple primitives, then check that contacts (landing,
     grasping) still behave.
   - Pin the simulator to dedicated cores and keep other loop processes off them.
3. **Acceptance:** sustained RTF 1.0 with p99 step time under the physics step through a full
   representative run, and Phase 3's metrics unchanged.

## Common traps

- Environment variables don't reach processes inside an already-running terminal multiplexer server.
  Pass switches on the command line of each pane or process.
- `pkill -f <pattern>` also matches your own shell when your command line contains the pattern. Use a
  self-excluding pattern (for example `"[p]attern"`).
- After switching branches of a message or interface package, clean-rebuild it. Stale generated language
  bindings fail silently at runtime. Also check that every package depending on it declares the dependency,
  or parallel builds race.
- Middleware language bindings are built for one specific interpreter version. Use that one.
- Simulator-bundled interpreters may not expose their libraries until the simulator app has started.
- A wall-clock component hiding in a test driver produces misleading verdicts ("the controller times out")
  that are really timing artifacts.

## Deliverables

1. **Phase 1 report:** RTF numbers, the per-component time-source table, the decision with evidence.
2. **Design note:** the switch, the clock abstraction per codebase, the firmware time-mode decision.
   Wait for approval.
3. **Implementation:** opt-in, tests green both ways.
4. **Phase 3 results:** the RTF-invariance pair and the real-data reproduction.
5. **Phase 4 results,** if run.
