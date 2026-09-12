#!/usr/bin/env python
"""
CAD VTOL under EXTERNAL ROS 2 hover control (no PX4).

Isaac side of the standalone control loop driven by scripts/start_vtol_hover_control.sh:

    Isaac VTOL --[ROS2Backend]------> /uav_0/state/*
               --[mocap emulator]---> /uav_0/mocap
               --[fsc_vtol_controller]-> /uav_0/vtol/actuator_cmd
               --[VTOLCommandROS2Backend]-> VTOL command (this scene)

Forces are ON (thrust(RPM^2) + aero + gravity); the ROS 2 controller provides the
motor commands and holds the tilt fixed. Run from a CLEAN shell (Isaac's bundled
rclpy) — the launcher does this. Standalone (for debugging without the controller
the vehicle just sits, since no command arrives).
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.vtol import VTOL
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from fsc_aerial_manipulation.rotorcraft import fsc_cad_vtol_config, VTOLCommandROS2Backend


NS = "uav_"


class HoverControlApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        simulation_app.update()

        config = fsc_cad_vtol_config()
        config.enable_forces = True
        config.disable_gravity = False
        config.rotor_dynamics = True
        config.backends = [
            # backend[0] = command IN (from the fsc_vtol_controller node)
            VTOLCommandROS2Backend(vehicle_id=0, namespace=NS),
            # state OUT (-> /uav_0/state/* -> mocap emulator -> controller)
            ROS2Backend(vehicle_id=0, num_rotors=0, config={
                "namespace": NS, "pub_state": True, "pub_sensors": False,
                "pub_graphical_sensors": False, "sub_control": False,
            }),
        ]

        self.vehicle = VTOL(vehicle_id=0, init_pos=(0.0, 0.0, 1.5), config=config)

        self.world.reset()
        self.pg.set_viewport_camera([5.0, -5.0, 3.0], [0.0, 0.0, 1.4])

    def run(self):
        self.timeline.play()
        carb.log_warn("Hover-control scene: waiting for /uav_0/vtol/actuator_cmd from the controller.")
        while simulation_app.is_running():
            self.world.step(render=True)
        self.timeline.stop()
        simulation_app.close()


def main():
    HoverControlApp().run()


if __name__ == "__main__":
    main()
