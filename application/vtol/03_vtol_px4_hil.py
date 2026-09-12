#!/usr/bin/env python
"""
VTOL under PX4 SITL HIL  (Step 4 of the VTOL bring-up).

This is the VTOL analogue of application/px4_base/01_px4_single_drone.py.  It
spawns the generic articulated VTOL and wires it to PX4 over the MAVLink HIL
link via PX4VTOLMavlinkBackend, which routes PX4's actuator outputs into the
VTOL's channels (motors + tilt servo + elevons).

PX4 SITL is launched SEPARATELY (a tmux script), running a VTOL airframe.  With
`direct_actuator` offboard (a ROS2 node streaming ActuatorMotors / ActuatorServos
— see docs/direct_actuator_offboard.md), PX4 passes commands straight through to
its outputs and the simulator applies them.

Bring-up tips:
  - Launch this with  PEGASUS_HIL_DEBUG=1  to log the actuator path:
      [VTOL_HIL vid=0] armed=True controls[:8]=[...] -> {'throttle_front_L': ..., 'tilt': ...}
    Use that to VERIFY / fix the channel_map (the actuator index order is
    airframe-dependent).
  - The connection waits for PX4 SITL on TCP 4560+vehicle_id.
"""

import os
import time

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World
import omni.usd
from isaacsim.core.utils.prims import is_prim_path_valid

from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface

from fsc_aerial_manipulation.rotorcraft import (
    spawn_vtol_with_mavlink,
    fsc_cad_vtol_config,
    fsc_cad_vtol_slosh_config,
)
from fsc_aerial_manipulation.utils import add_dome_lighting

# PX4 SITL airframe for the CAD quad (2 front tilting + 2 rear static rotors).
# px4_vehicle_model is informational here (px4_autolaunch=False — the tmux script
# starts SITL with this airframe); the channel_map is auto-selected to match.
PX4_VTOL_AIRFRAME = "fsc_quad_tiltrotor"


class VTOLPx4HilApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(
            stage=stage, dome_path="/World/DomeLight",
            intensity=2500.0, exposure=0.0, color=(1.0, 1.0, 1.0),
        )

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Large open flat terrain (no walls) + scaled up, so the VTOL can fly a long
        # transition (accelerate to cruise, tilt to 90 deg, fly far) and land far out.
        # load_environment() is ASYNC -> pump updates until the layout prim exists, scale it.
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Flat Plane"])
        try:
            from pxr import UsdGeom, Gf
            _stage = omni.usd.get_context().get_stage()
            _scaled = False
            for _ in range(400):
                simulation_app.update()
                _layout = _stage.GetPrimAtPath("/World/layout")
                if _layout and _layout.IsValid():
                    UsdGeom.XformCommonAPI(_layout).SetScale(Gf.Vec3f(8.0, 8.0, 1.0))
                    carb.log_warn("Expanded ground (/World/layout x8) for long-range flight")
                    _scaled = True
                    break
            if not _scaled:
                carb.log_warn("layout prim never appeared; ground left at default size")
        except Exception as _e:
            carb.log_warn(f"could not scale ground: {_e}")
        simulation_app.update()

        # Spawn the dual INDEPENDENT-tiltrotor wired to PX4 HIL.
        #   - config       : the dual tiltrotor airframe (2 tilt rotors + a/e/r)
        #   - channel_map  : how PX4's actuator outputs map to this airframe's
        #                    channels (motor_left/right, tilt_left/right,
        #                    aileron/elevator/rudder) — VERIFY w/ PEGASUS_HIL_DEBUG=1
        # VTOL_ROS2_STATE=1 also publishes /uav_0/state/{pose,twist,twist_inertial}
        # (feeds the mocap emulator + controller). Off by default -> plain HIL.
        publish_state = os.environ.get("VTOL_ROS2_STATE", "0") not in ("0", "", "false", "False")
        if publish_state:
            carb.log_warn("VTOL_ROS2_STATE set -> publishing /uav_0/state/* for mocap")

        # VTOL_SLOSH=1 flies the SAME airframe on the free-surface-tank model
        # (VTOLSlosh) instead of the baseline VTOL. In phase 1 of the tank plan
        # the tank is None, so the two are identical step for step -- which is
        # the point: fly the mission on the copy and compare it with the
        # `pre-slosh` baseline before any tank physics exists to blame.
        use_slosh = os.environ.get("VTOL_SLOSH", "0") not in ("0", "", "false", "False")
        vtol_config = fsc_cad_vtol_slosh_config() if use_slosh else fsc_cad_vtol_config()
        if use_slosh:
            carb.log_warn("VTOL_SLOSH set -> VTOLSlosh model, tank=None "
                          "(phase 1: behaviour identical to the baseline VTOL)")

        self.vtol_path = spawn_vtol_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_vehicle_model=PX4_VTOL_AIRFRAME,
            vehicle_id=0,
            spawn_pos=(0.0, 0.0, 0.3),
            spawn_euler=(0.0, 0.0, 0.0),
            config=vtol_config,                  # CAD quad; channel_map auto-selected
            publish_state_ros2=publish_state,
        )
        simulation_app.update()

        if is_prim_path_valid(self.vtol_path):
            carb.log_warn(f"Spawned VTOL at {self.vtol_path}")
        else:
            carb.log_error(f"VTOL prim was not created at {self.vtol_path}")

        self.world.reset()
        self.pg.set_viewport_camera([5.0, -5.0, 3.0], [0.0, 0.0, 0.5])
        self.stop_sim = False

    def run(self):
        self.timeline.play()
        carb.log_warn(
            "VTOLPx4HilApp render loop "
            f"(DISPLAY={os.environ.get('DISPLAY')}, running={simulation_app.is_running()})"
        )
        # ---- CHASE CAMERA: follow the VTOL body (keeps it centred as it flies far). --------
        # Toggle with PEGASUS_CHASE_CAM=0 (static camera). Offset (world ENU, m) behind/left/above
        # the vehicle via PEGASUS_CHASE_OFFSET="dx,dy,dz". Read the live pose from the vehicle state.
        chase = os.environ.get("PEGASUS_CHASE_CAM", "1") not in ("0", "", "false", "False")
        try:
            off = [float(x) for x in os.environ.get("PEGASUS_CHASE_OFFSET", "-8,-8,4").split(",")]
        except Exception:
            off = [-8.0, -8.0, 4.0]
        vehicle = None
        if chase:
            try:
                vehicle = self.pg.get_vehicle(self.vtol_path)
            except Exception:
                try:
                    vehicle = self.pg.get_all_vehicles()[0]
                except Exception:
                    vehicle = None
            carb.log_warn(f"chase cam {'ON' if vehicle else 'OFF (no vehicle handle)'} offset={off}")
        cam = [0.0, 0.0, 0.0]; nstep = 0
        while not self.stop_sim:
            if not simulation_app.is_running():
                time.sleep(1.0)
                continue
            self.world.step(render=True)
            if vehicle is not None:
                nstep += 1
                if nstep % 2 == 0:                     # every other frame -> smooth + cheap
                    try:
                        p = vehicle._state.position
                        eye = [float(p[0]) + off[0], float(p[1]) + off[1], float(p[2]) + off[2]]
                        # low-pass the camera eye so it glides rather than snapping (esp. at speed)
                        a = 0.15
                        cam = [cam[i] + a * (eye[i] - cam[i]) for i in range(3)] if nstep > 2 else eye
                        self.pg.set_viewport_camera(cam, [float(p[0]), float(p[1]), float(p[2])])
                    except Exception:
                        pass

        carb.log_warn("VTOLPx4HilApp closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    app = VTOLPx4HilApp()
    app.run()


if __name__ == "__main__":
    main()
