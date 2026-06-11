#!/usr/bin/env python
"""
| File: 03_px4_single_drone_mt2216.py
| Description: Single-drone SITL simulation using the MT2216 / 10×4.5 thrust
|              curve with first-order rotor lag (MT2216ThrustCurve).
|              Structurally identical to 01_px4_single_drone.py — the only
|              change is replacing the default QuadraticThrustCurve with the
|              physics-based MT2216ThrustCurve.
|
| How to run:
|   scripts/start_single_drone_sitl_mt2216.sh <config_name>
|   e.g.  scripts/start_single_drone_sitl_mt2216.sh pravin_machine
|
|   Then in QGroundControl: arm → takeoff → move the slider to hover.
"""

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ------------------------------------------------------------------
import omni.timeline
import omni.usd
from omni.isaac.core.world import World
from scipy.spatial.transform import Rotation

from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.thrusters import MT2216ThrustCurve

from fsc_aerial_manipulation.utils import add_dome_lighting


class FscDroneSimMT2216:
    """
    Single-drone SITL environment with the MT2216 rotor-lag thrust model.
    """

    # Spawn location (ENU, metres)
    SPAWN_POS    = (0.0, 0.0, 0.07)
    SPAWN_EULER  = (0.0, 0.0, 0.0)   # degrees, XYZ
    VEHICLE_ID   = 0
    TCP_PORT     = 4560               # must match PX4 SITL connection

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        # Lighting
        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(
            stage=stage,
            dome_path="/World/DomeLight",
            intensity=2500.0,
            exposure=0.0,
            color=(1.0, 1.0, 1.0),
        )

        # Pegasus / World setup  (identical to 01_px4_single_drone.py)
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])

        # ------------------------------------------------------------------
        # Vehicle spawning — same as spawn_rotorcraft_with_mavlink() but
        # with MT2216ThrustCurve replacing the default QuadraticThrustCurve.
        # ------------------------------------------------------------------
        quat_xyzw = Rotation.from_euler(
            "XYZ", self.SPAWN_EULER, degrees=True
        ).as_quat()

        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": self.VEHICLE_ID,
            "connection_type": "tcpin",
            "connection_ip": "127.0.0.1",
            "connection_baseport": self.TCP_PORT,
            "px4_autolaunch": False,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe,
        })

        ros2_backend = ROS2Backend(
            vehicle_id=self.VEHICLE_ID,
            config={
                "namespace": "uav_",
                "pub_sensors": False,
                "pub_graphical_sensors": False,
                "pub_state": True,
                "pub_twist": True,
                "pub_accel": True,
                "pub_twist_inertial": True,
                "pub_tf": True,
                "sub_control": False,
            },
        )

        config = MultirotorConfig()

        # ← MT2216 rotor-lag model (replaces default QuadraticThrustCurve)
        config.thrust_curve = MT2216ThrustCurve()

        config.backends = [PX4MavlinkBackend(mavlink_config), ros2_backend]

        drone_prim_path = f"/World/quadrotor_{self.VEHICLE_ID}"

        Multirotor(
            drone_prim_path,
            ROBOTS["Iris"],
            self.VEHICLE_ID,
            self.SPAWN_POS,
            quat_xyzw,
            config=config,
        )

        self.world.reset()
        self.stop_sim = False

    def run(self):
        self.timeline.play()

        while simulation_app.is_running() and not self.stop_sim:
            self.world.step(render=True)

        carb.log_warn("FscDroneSimMT2216: simulation closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    pg_app = FscDroneSimMT2216()
    pg_app.run()


if __name__ == "__main__":
    main()
