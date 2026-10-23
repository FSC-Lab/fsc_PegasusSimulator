#!/usr/bin/env python
"""
| File: 05_px4_single_drone_t650.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2026, Longhao Qian. All rights reserved.
| Description: Single Tarot 650 (T650) quadrotor SITL simulation environment - bare
|   airframe, no payload. Structurally the same scenario as
|   03_px4_single_drone_x650.py, but the T650 now has its OWN parameter and spawn
|   modules rather than borrowing the X650's:
|
|       rotorcraft/t650_params.py            physical + calibration constants
|       rotorcraft/t650_bare_frame_utils.py  spawn_t650_with_mavlink()
|
|   Neither imports the X650 modules, so a change to one vehicle cannot silently
|   move the other.
|
|   What actually differs from the X650:
|
|     1. MOTORS: MN4010 + 15x5" instead of MN4014 + 15x5"
|        (docs/propeller_testing/MN_4010_15x5_report.pdf).
|     2. MASS: body mass 2.95 kg (total 3.0339 kg with the asset's four authored
|        rotor bodies) instead of the X650's 3.416 kg body / 3.5 kg total.
|
|   The USD asset is SHARED - there is no separate T650 model; both vehicles use
|   x650_new.usd, and the T650's body inertia is copied from the X650's CAD values.
|
|   Two consequences worth knowing before flying it:
|
|   * It hovers HIGHER on the stick: 0.5117 against the X650's 0.4800, because the
|     MN4010's top rotor speed is lower (730.05 vs 817.59 rad/s) at a nearly
|     identical thrust constant, and because the body mass is now 2.95 kg rather
|     than 2.95 kg TOTAL as before (which gave 0.5032). Expected, not a
|     calibration error - but see t650_params.BODY_MASS, which records that
|     0.5032 is the value validated against hardware.
|   * The rotor is SLOWER: lambda 10.0265 vs 10.51 1/s, i.e. tau 99.7 vs 95.1 ms.
|     Less phase margin, so the softened X650 PX4 gains (MC_*RATE_K=0.3,
|     MC_ROLL_P/MC_PITCH_P=3.25, MC_YAW_P=1.4) are if anything more necessary
|     here. The launch scripts apply them before EKF2 starts.
"""

# Imports to start Isaac Sim from this script
import os

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
HEADLESS = os.environ.get("PEGASUS_HEADLESS", "0") == "1"
STEP_LIMIT = int(os.environ.get("PEGASUS_STEPS", "0"))
# Real-time knobs, ported 2026-10-02 from 06_px4_t650_aerial_manipulator_free_flight.py (same
# environment variables, baked into the Isaac pane by lib/start_single_drone_x650.sh):
#   PEGASUS_REALTIME=1       pace the loop so simulated time never runs AHEAD of the wall clock
#                            (RTF capped at 1); prints the achieved RTF every 10 s. It cannot
#                            make a slow machine faster -- below 1 it only reports.
#   PEGASUS_RENDER_EVERY=N   windowed runs: one 4 ms world.step(render=False) per loop and a
#                            redraw every N steps, instead of world.step(render=True) (4 physics
#                            steps per frame in one burst). Default 8 under the pacer, else 0.
#   PEGASUS_PROFILE_START=S  S wall seconds after play, time PEGASUS_PROFILE_STEPS world.step()
#                            calls plainly, then the same under cProfile; writes
#                            PEGASUS_PROFILE_OUT(.txt/.prof).
REALTIME = os.environ.get("PEGASUS_REALTIME", "0") == "1"
RENDER_EVERY = int(os.environ.get("PEGASUS_RENDER_EVERY", "") or (8 if REALTIME else 0))
PROFILE_START = float(os.environ.get("PEGASUS_PROFILE_START", "0") or 0.0)
PROFILE_STEPS = int(os.environ.get("PEGASUS_PROFILE_STEPS", "1500") or 1500)
PROFILE_OUT = (os.environ.get("PEGASUS_PROFILE_OUT", "") or
               os.path.expanduser("~/.cache/pegasus_profile/05_profile"))
PX4_LOCKSTEP = os.environ.get("PEGASUS_PX4_LOCKSTEP", "1") == "1"

# Optional PAYLOAD, in kg, added to the airframe body mass at spawn time (added
# 2026-08-24 for the 769 g L1-saturation campaign, docs report "L1 Against Battery
# Fade"). Default 0.0 leaves the bare T650 plant bit-identical, so every existing
# T650 rig that sources this script is unchanged.
#
# The payload is modelled as EXTRA MASS AT THE BODY CoM ONLY: the identified
# INERTIA_TENSOR is left alone, and no CoM offset is introduced. That matches this
# repo's standing T650 assumption that added mass is centrally concentrated. It does
# NOT reproduce the hardware payload's measured ~4 mm forward CoM shift (0.5 -> 0.8
# N.m of standing pitch moment in the flight report), so a sim run will show a much
# smaller L1 torque estimate than the corresponding hardware flight.
_payload_raw = os.environ.get("PEGASUS_PAYLOAD_MASS", "").strip()
PAYLOAD_MASS = float(_payload_raw) if _payload_raw else 0.0
if PAYLOAD_MASS < 0.0:
    raise ValueError(f"PEGASUS_PAYLOAD_MASS must be >= 0, got {PAYLOAD_MASS}")

# Optional AERODYNAMIC DRAG injection (added 2026-09-30 for the CCM disturbance-estimation tests).
#   PEGASUS_LINEAR_DRAG="cx,cy,cz"   linear body drag [N/(m/s)], body FLU, F = -diag(c) v_body
#                                    (unset: the Pegasus default 0.50,0.30,0.0)
#   PEGASUS_ANGULAR_DRAG="kx,ky,kz"  rotational damping [N m/(rad/s)], tau = -diag(k) omega_body
#                                    (unset: none)
#   PEGASUS_REPORT_MASS_PROPS=1      print the PhysX mass properties once the sim runs
#   PEGASUS_MASS_REPORT_FILE=<path>  and write them as JSON
def _vec3_env(name):
    raw = os.environ.get(name, "").strip()
    if not raw:
        return None
    vals = [float(v) for v in raw.replace(" ", "").split(",")]
    if len(vals) != 3 or any(v < 0.0 for v in vals):
        raise ValueError(f"{name} must be three non-negative numbers 'x,y,z', got '{raw}'")
    return vals


LINEAR_DRAG = _vec3_env("PEGASUS_LINEAR_DRAG")
ANGULAR_DRAG = _vec3_env("PEGASUS_ANGULAR_DRAG")
REPORT_MASS_PROPS = os.environ.get("PEGASUS_REPORT_MASS_PROPS", "0") == "1"
MASS_REPORT_FILE = os.environ.get("PEGASUS_MASS_REPORT_FILE", "").strip() or None
simulation_app = SimulationApp({"headless": HEADLESS})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import time

import numpy as np
import omni.timeline
from omni.isaac.core.world import World
import omni.usd

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

# The T650's own spawn helper and constants -- independent of the X650 modules.
from fsc_aerial_manipulation.rotorcraft.t650_bare_frame_utils import spawn_t650_with_mavlink
from fsc_aerial_manipulation.rotorcraft import t650_params
from fsc_aerial_manipulation.utils import add_dome_lighting


class FscDroneSim:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the FscDroneSim and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

         # ---------- Add dome lighting ----------
        stage = omni.usd.get_context().get_stage()

        add_dome_lighting(
            stage=stage,
            dome_path = "/World/DomeLight",
            intensity=2500.0,
            exposure=0.0,
            color=(1.0, 1.0, 1.0)
        )

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # Create the vehicle. Inertia and the MN4010 thrust curve come from t650_params
        # via the helper's own defaults, so there is exactly one place to change a T650
        # number. Body mass is passed explicitly ONLY so an optional payload can be
        # folded in; with PEGASUS_PAYLOAD_MASS unset this is exactly BODY_MASS.
        body_mass = t650_params.BODY_MASS + PAYLOAD_MASS
        self.drone_path = spawn_t650_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=(0.0, 0.0, 0.07),
            spawn_euler=(0.0, 0.0, 0.0),
            enable_lockstep=PX4_LOCKSTEP,
            body_mass=body_mass,
            linear_drag=LINEAR_DRAG,
            angular_drag=ANGULAR_DRAG,
        )
        print(
            f"[T650] DRAG: linear "
            f"{LINEAR_DRAG if LINEAR_DRAG is not None else [0.5, 0.3, 0.0]} N/(m/s) body FLU "
            f"({'PEGASUS_LINEAR_DRAG' if LINEAR_DRAG is not None else 'Pegasus default'}), "
            f"angular {ANGULAR_DRAG if ANGULAR_DRAG is not None else [0.0, 0.0, 0.0]} N.m/(rad/s)",
            flush=True,
        )
        # Recomputed at the ACTUAL flying mass rather than read from t650_params, whose
        # module-level HOVER_COMMAND / THRUST_TO_WEIGHT are the bare-airframe values.
        total_mass = body_mass + t650_params.ROTOR_MASS_TOTAL
        t_hover = total_mass * 9.81 / t650_params.NUM_ROTORS
        omega_span = t650_params.MAX_ROTOR_VEL - t650_params.ZERO_POSITION_ARMED
        omega_hover = float(np.sqrt(t_hover / t650_params.ROTOR_CONSTANT))
        hover_command = (omega_hover - t650_params.ZERO_POSITION_ARMED) / omega_span
        thrust_to_weight = (
            t650_params.NUM_ROTORS * t650_params.ROTOR_CONSTANT
            * t650_params.MAX_ROTOR_VEL ** 2
        ) / (total_mass * 9.81)
        if PAYLOAD_MASS > 0.0:
            print(
                f"[T650] PAYLOAD {PAYLOAD_MASS:.6f} kg added to the body mass "
                f"(inertia UNCHANGED -- payload assumed at the body CoM)",
                flush=True,
            )
        print(
            f"[T650] total mass {total_mass:.6f} kg "
            f"(body {body_mass:.6f} kg = airframe {t650_params.BODY_MASS:.6f} "
            f"+ payload {PAYLOAD_MASS:.6f} | + 4 authored rotors "
            f"{t650_params.ROTOR_MASS_TOTAL:.6f} kg) | "
            f"inertia {tuple(t650_params.INERTIA_DIAG)} kg.m^2 | "
            f"expected hover command {hover_command:.4f} | "
            f"thrust/weight {thrust_to_weight:.2f}",
            flush=True,
        )
        print(
            f"[T650] the paired controller yaml's vehicle_mass MUST equal "
            f"{total_mass:.6f} -- this printout is the truth",
            flush=True,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()
        steps = 0
        prof = _RtfProfiler(self) if PROFILE_START > 0 else None
        pacer = _RealTimePacer(self) if REALTIME else None

        # The "infinite" loop
        while (
            simulation_app.is_running()
            and not self.stop_sim
            and (not STEP_LIMIT or steps < STEP_LIMIT)
        ):

            if prof is not None:
                prof.before_step()
            if HEADLESS or RENDER_EVERY > 0:
                # one physics step; with a window, redraw every RENDER_EVERY steps
                # (render() draws without advancing physics)
                self.world.step(render=False)
                if not HEADLESS and steps % RENDER_EVERY == 0:
                    self.world.render()
            else:
                # Update the UI of the app and perform the physics step
                self.world.step(render=True)
            steps += 1
            if prof is not None:
                prof.after_step()
            if pacer is not None:
                pacer.after_step()
            if REPORT_MASS_PROPS and steps == 10:
                try:
                    from fsc_aerial_manipulation.rotorcraft.t650_mass_report import report_mass_properties
                    report_mass_properties(self.drone_path, MASS_REPORT_FILE)
                except Exception as exc:  # a diagnostic must never stop the sim
                    print(f"[T650 MASS] report failed: {exc!r}", flush=True)

        # Cleanup and stop
        carb.log_warn("FscDroneSim Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


class _RealTimePacer:
    """PEGASUS_REALTIME: keep simulated time from running ahead of the wall clock, so the
    wall-clock control stack sees a real-time plant (port of 06's pacer). Anchored on absolute
    time (sim - sim0 <= wall - wall0); a stall of more than MAX_LAG_S behind moves the anchor
    instead of letting the sim sprint to catch up. Prints the achieved RTF every REPORT_S."""

    MAX_LAG_S = 0.05
    REPORT_S = 10.0

    def __init__(self, sim):
        self.sim = sim
        self.w0 = time.perf_counter()
        self.s0 = float(sim.world.current_time)
        self.rep_w, self.rep_s = self.w0, self.s0
        self.worst_lag = 0.0
        self.reanchors = 0
        self.slept = 0.0
        mode = "headless" if HEADLESS else (f"window, 4 ms physics steps, a frame every {RENDER_EVERY}"
                                             if RENDER_EVERY > 0 else "window, 4 physics steps per frame")
        print(f"[T650] REAL-TIME PACER on ({mode}): simulated time is held to the wall clock (RTF <= 1).",
              flush=True)

    def after_step(self):
        now = time.perf_counter()
        st = float(self.sim.world.current_time)
        ahead = (st - self.s0) - (now - self.w0)
        if ahead > 0.0:
            time.sleep(ahead)
            self.slept += ahead
            now = time.perf_counter()
        elif -ahead > self.MAX_LAG_S:
            self.worst_lag = max(self.worst_lag, -ahead)
            self.reanchors += 1
            self.w0, self.s0 = now, st
        else:
            self.worst_lag = max(self.worst_lag, -ahead)
        if now - self.rep_w >= self.REPORT_S:
            rtf = (st - self.rep_s) / (now - self.rep_w)
            print(f"[T650] RTF {rtf:.3f} over the last {now - self.rep_w:.1f} s "
                  f"(worst lag {1e3 * self.worst_lag:.1f} ms, {self.reanchors} re-anchor(s), "
                  f"slept {self.slept:.2f} s)", flush=True)
            self.rep_w, self.rep_s = now, st
            self.worst_lag, self.reanchors, self.slept = 0.0, 0, 0.0


class _RtfProfiler:
    """PEGASUS_PROFILE_START: where a physics step's wall time goes (port of 06's profiler).
    Window A (plain): RTF and ms of wall per 4 ms physics step over PROFILE_STEPS world.step()
    calls. Window B (cProfile, same length): the main thread's Python call tree -- Pegasus
    sensors, the MAVLink and ROS 2 backends, PhysX as the time inside the step call. cProfile
    slows Python-heavy code, so read B as a breakdown and A as the speed."""

    def __init__(self, sim):
        import cProfile
        self.sim = sim
        self.cprofile = cProfile.Profile()
        self.t_play = time.perf_counter()
        self.phase = "wait"
        self.n = 0
        os.makedirs(os.path.dirname(PROFILE_OUT) or ".", exist_ok=True)

    def _sim_t(self):
        return float(self.sim.world.current_time)

    def before_step(self):
        now = time.perf_counter()
        if self.phase == "wait" and now - self.t_play >= PROFILE_START:
            self.phase, self.n = "A", 0
            self.a0 = (now, self._sim_t())
        elif self.phase == "A" and self.n >= PROFILE_STEPS:
            self.a1 = (now, self._sim_t())
            self.phase, self.n = "B", 0
            self.b0 = (now, self._sim_t())
            self.cprofile.enable()
        elif self.phase == "B" and self.n >= PROFILE_STEPS:
            self.cprofile.disable()
            self.b1 = (now, self._sim_t())
            self.phase = "done"
            self._report()

    def after_step(self):
        if self.phase in ("A", "B"):
            self.n += 1

    def _report(self):
        import io
        import pstats
        dt = float(self.sim.world.get_physics_dt())
        wa, sa = self.a1[0] - self.a0[0], self.a1[1] - self.a0[1]
        wb, sb = self.b1[0] - self.b0[0], self.b1[1] - self.b0[1]
        n_phys = sa / dt
        lines = [f"[T650] PROFILE ({'headless' if HEADLESS else 'rendering'}, {PROFILE_STEPS} world.step() calls per window)",
                 f"  window A (plain):    wall {wa:.2f} s, sim {sa:.2f} s -> RTF {sa / wa:.3f}; "
                 f"{n_phys:.0f} physics steps, {1e3 * wa / max(n_phys, 1):.2f} ms wall per {1e3 * dt:.0f} ms step",
                 f"  window B (cProfile): wall {wb:.2f} s, sim {sb:.2f} s -> RTF {sb / wb:.3f}"]
        buf = io.StringIO()
        st = pstats.Stats(self.cprofile, stream=buf)
        st.sort_stats("cumulative").print_stats(45)
        st.sort_stats("tottime").print_stats(35)
        self.cprofile.dump_stats(PROFILE_OUT + ".prof")
        with open(PROFILE_OUT + ".txt", "w") as f:
            f.write("\n".join(lines) + "\n\n" + buf.getvalue())
        print("\n".join(lines) + f"\n  -> {PROFILE_OUT}.txt / .prof", flush=True)


def main():
    # Instantiate the template app
    fsc_drone_sim = FscDroneSim()

    # Run the application loop
    fsc_drone_sim.run()


if __name__ == "__main__":
    main()
