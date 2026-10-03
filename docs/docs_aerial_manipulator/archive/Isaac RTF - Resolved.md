# Isaac RTF — Resolved

2026-10-01, shiqi-desktop (i9-14900KF). This note explains how Isaac Sim was brought from
**RTF 0.48 to 1.00**, and how to carry the fix to another desktop.

## The problem

RTF (real-time factor) is simulated seconds per wall-clock second. Isaac ran at 0.48: every
4 ms physics step took 8.3 ms of wall time. The ROS 2 controllers, planners and timers run on
the **wall clock**, so they saw a plant at half speed:

- reference velocities arrived ×RTF, and accelerations ×RTF²;
- every plant lag looked 1/RTF longer.

Rescaling the plant lags by `SIM_RTF` only approximated this. Simulation results were biased,
by up to 4× on a tracking metric.

## Causes (measured)

1. **Per-step Python work.** The Isaac app's physics callback evaluated a full dynamics model
   on every 4 ms step, about 3 ms of the 8.3. Only one rarely used branch needed the result.
2. **Hybrid CPU.** On the i9-14900KF, CPUs 0–15 are P-cores (5.7–6.0 GHz) and CPUs 16–31 are
   E-cores (4.4 GHz). Isaac's main thread runs the physics fetch and every Python callback.
   The scheduler sometimes put that thread on an E-core: RTF 0.76 even after fix 1.
3. **Rendering.** `world.step(render=True)` draws a frame and advances 4 physics steps in one
   burst. Headless runs skip the draw.

## Fixes

| fix | where | effect |
|---|---|---|
| Evaluate the model only when needed | the Isaac app (`application/robotic_arm/06_px4_t650_aerial_manipulator_free_flight.py`) | callback 3.98 → 0.79 ms; RTF 0.48 → 0.87 windowed, 1.01 headless |
| Pin Isaac to the P-cores | `ISAAC_CPUS` in the machine config → `taskset -c` in the base launcher | stops the 0.76 dips; ~19 % idle per frame headless |
| Real-time pacer | `SIM_REALTIME=1` (machine config) or `PEGASUS_REALTIME=1` | Isaac sleeps whenever sim time gets ahead of the wall clock, so RTF stays ≤ 1. Prints `RTF x.xxx over the last 10.0 s` every 10 s |
| Windowed runs step physics singly | `PEGASUS_RENDER_EVERY` (default 8 under the pacer) | one 4 ms `world.step(render=False)` per loop, plus `world.render()` (redraw, no physics) every 8 steps |

Measured during real flights:

| | before | after |
|---|---|---|
| headless | 0.48 | **1.000** in every 10 s window (after spawn) |
| windowed | 0.48 | 0.95–1.00, with occasional 50–155 ms catch-ups |

## Transplanting to another desktop

1. **Pull the code.** The fix lives in the Isaac app `06_px4_t650_aerial_manipulator_free_flight.py`
   (lazy evaluation, pacer, profiler, single-step rendering). The shared base launcher
   `start_single_drone_x650.sh` carries the pinning, `SIM_REALTIME`, and the baked
   `PEGASUS_*` knobs. Every `start_t650_*` launcher execs that base launcher.
2. **Find the fast cores.**
   ```bash
   lscpu --extended=CPU,CORE,MAXMHZ
   ```
   On a hybrid CPU, the P-cores are the CPUs with the higher MAXMHZ; list them including
   their hyperthreads (e.g. `0-15`). On a non-hybrid CPU, leave `ISAAC_CPUS` unset.
3. **Measure before committing to settings.** Launch once with the knobs on the command line
   and leave the machine config alone:
   ```bash
   PEGASUS_ISAAC_CPUS=0-15 PEGASUS_REALTIME=1 PEGASUS_HEADLESS=1 \
     scripts/indoor_sim/<launcher>.sh <machine_config>
   ```
   Read the pacer's `RTF … over the last 10.0 s` lines in the Isaac pane. Ignore the first
   window, which covers the spawn.
4. **Write the machine config** (`scripts/config/<machine>.conf`):
   - If the pacer holds **1.000**:
     ```bash
     SIM_RTF=1.0
     SIM_REALTIME=1
     ISAAC_CPUS=0-15     # this box's P-cores; omit on a non-hybrid CPU
     ```
   - If it reads **r < 1**: set `SIM_RTF=r` (what the pacer reads) and keep `SIM_REALTIME=1`.
     The pacer never sleeps then, but it still prints the RTF. The plant lags are rescaled by r.
5. **Optional profile.** For a box that stays below 1:
   ```bash
   PEGASUS_PROFILE_START=120 PEGASUS_PROFILE_STEPS=4000 PEGASUS_PROFILE_OUT=/tmp/prof …
   ```
   This writes `/tmp/prof.txt`: RTF, ms per physics step, the app callback's share, and a
   cProfile top-45. It also writes `/tmp/prof.prof`; open it with `python3 -m pstats`.

## Rules and traps

- **`SIM_RTF` and the pacer go together.** `SIM_RTF=1.0` with the pacer off is wrong:
  headless Isaac then runs ~1.2× faster than the wall clock.
- **Use headless for any quantitative run.** Headless leaves ~19 % idle per frame; windowed
  leaves 0–5 %. Ground-station GUIs, bag recording and other heavy processes eat into that margin.
- **Only the one app has the pacer, profiler and single-step rendering**
  (`06_px4_t650_aerial_manipulator_free_flight.py`). Other Isaac apps launched
  through the same base launcher get the pinning but not the pacer. To add the pacer, port
  `_RealTimePacer`: about 40 lines, called after each `world.step()`.
- **Pass every knob through the launcher.** The base launcher bakes each knob into the Isaac
  tmux pane's command line, because the tmux server keeps its own environment. An `export`
  does not reach an already-running server.
- `PEGASUS_ISAAC_CPUS=` (empty) does **not** unpin; give the full CPU list (e.g. `0-31`).
- **Real time removes margin the slow clock gave.** At RTF < 1 every lag was faster in plant
  time, so a loop tuned at RTF 0.48 can sit closer to its limit at 1.00. Re-check marginal tunes.
- **Results flown at RTF 0.48 (everything before 2026-10-01 on shiqi-desktop) do not compare
  with RTF-1 results.**
- `fsc_lab_machine.conf` still says `SIM_RTF=0.34`, a value measured before fix 1. It is
  flagged stale; re-measure with step 3.

Data, profiles and the measurement runs: `docs/docs_aerial_manipulator/rtf_profile_20261001/`.
Full log: Command.md §7.23.
