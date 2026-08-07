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
PX4_LOCKSTEP = os.environ.get("PEGASUS_PX4_LOCKSTEP", "1") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

# -----------------------------------
# The actual script should start here
# -----------------------------------
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

        # Create the vehicle. Mass, inertia and the MN4010 thrust curve all come from
        # t650_params via the helper's own defaults -- nothing is passed in here, so
        # there is exactly one place to change a T650 number.
        self.drone_path = spawn_t650_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=(0.0, 0.0, 0.07),
            spawn_euler=(0.0, 0.0, 0.0),
            enable_lockstep=PX4_LOCKSTEP,
        )
        print(
            f"[T650] total mass {t650_params.MASS:.6f} kg "
            f"(body {t650_params.BODY_MASS:.6f} kg + 4 authored rotors "
            f"{t650_params.ROTOR_MASS_TOTAL:.6f} kg) | "
            f"inertia {tuple(t650_params.INERTIA_DIAG)} kg.m^2 | "
            f"expected hover command {t650_params.HOVER_COMMAND:.4f} | "
            f"thrust/weight {t650_params.THRUST_TO_WEIGHT:.2f}",
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

        # The "infinite" loop
        while (
            simulation_app.is_running()
            and not self.stop_sim
            and (not STEP_LIMIT or steps < STEP_LIMIT)
        ):

            # Update the UI of the app and perform the physics step
            self.world.step(render=not HEADLESS)
            steps += 1

        # Cleanup and stop
        carb.log_warn("FscDroneSim Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    # Instantiate the template app
    fsc_drone_sim = FscDroneSim()

    # Run the application loop
    fsc_drone_sim.run()


if __name__ == "__main__":
    main()
