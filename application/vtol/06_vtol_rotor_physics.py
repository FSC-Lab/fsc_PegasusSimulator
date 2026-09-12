#!/usr/bin/env python
"""
Rotor + servo PHYSICS-MODEL demo (config.rotor_dynamics = True).

Exercises the model added to vtol.py:
  - rotor spin-up: omega chases throttle*omega_max with time constant rotor_tau_spin
  - thrust:        T = max_thrust * (omega/omega_max)^2  (lags throttle, ~RPM^2)
  - yaw torque:    Q = rotor_moment_ratio * T * (-spin_dir) about the prop axis
  - tilt servo:    first-order (tilt_tau) response, capped at the slew rate

It uses the CAD model SPLIT into parts (~/vtol_cad_usd/parts_usd/{body,front_left,
front_right,rear_left,rear_right}.usd), so the REAL front rotors tilt and all props
spin. FORCES OFF (enable_forces=False) and gravity off, so the model sits in place
while the tilt servos ease and the rotors spin. The logged omega/thrust are what the
model COMPUTES; with forces off they are not applied -- flip enable_forces=True to
actually apply them. (The whole rotor assembly spins as one body for now; splitting
motor from prop would spin only the blade.)

It applies STEP inputs and logs the response each 0.25 s: throttle steps up at
t=2 s (omega ramps in over ~5*tau), tilt servo steps at t=6 s, then both return.

Run like the other scenes (clean shell + Isaac python). Tune via the config fields
set below (rotor_omega_max / rotor_tau_spin / tilt_tau).
"""

import math
import os

import carb
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World

from pegasus.simulator.logic.backends.backend import Backend
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.vtol import VTOL
from pegasus.simulator.params import SIMULATION_ENVIRONMENTS
from fsc_aerial_manipulation.rotorcraft import fsc_cad_vtol_config


class StepInputBackend(Backend):
    """Open-loop step schedule so the rotor/servo dynamics are easy to see."""

    def __init__(self):
        super().__init__(config=None)
        self.t = 0.0
        self._cmd = {}

    def input_reference(self):
        return self._cmd

    def update(self, dt: float):
        self.t += dt
        throttle = 0.6 if 2.0 <= self.t < 12.0 else 0.0     # step up @2s, down @12s
        tilt = 1.0 if 6.0 <= self.t < 10.0 else 0.0         # tilt to cruise @6s, back @10s
        # gentle surface wiggle so the aileron / elevator / rudder are visible
        surf = math.sin(2.0 * math.pi * self.t / 3.0)
        self._cmd = {
            "motor_front_L": throttle, "motor_front_R": throttle,
            "motor_rear_L": throttle, "motor_rear_R": throttle,
            "tilt_left": tilt, "tilt_right": tilt,
            "aileron": surf, "elevator": surf, "rudder": surf,
        }

    def update_sensor(self, sensor_type, data): pass
    def update_graphical_sensor(self, sensor_type, data): pass
    def update_state(self, state): pass
    def start(self): pass
    def stop(self): pass
    def reset(self): self.t = 0.0


class RotorPhysicsApp:
    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()

        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Curved Gridroom"])
        simulation_app.update()

        # The FSC CAD airframe (split parts + tuned hinge), FORCES OFF so it stays
        # in place while the tilt/spin joints animate.
        config = fsc_cad_vtol_config()
        config.rotor_dynamics = True              # real props tilt + spin (visual only now)
        config.enable_forces = False              # <-- FORCE = 0 (no thrust, no aero)
        config.disable_gravity = True             # no forces + no gravity -> stays in place
        # Model parameters (still shape the spin/tilt animation):
        config.rotor_omega_max = 1000.0           # [rad/s] at throttle 1
        config.rotor_tau_spin = 0.25              # [s] spin-up time constant
        config.tilt_tau = 0.30                    # [s] tilt-servo time constant

        self.backend = StepInputBackend()
        config.backends = [self.backend]

        self.init_pos = (0.0, 0.0, 1.5)
        self.vehicle = VTOL(vehicle_id=0, init_pos=self.init_pos, config=config)
        self.omega_max = config.rotor_omega_max
        self.rotor = self.vehicle._rotors[0]      # front_L, for the readout

        self.world.reset()
        self._add_pivot_markers()                 # show each tilt joint
        self.pg.set_viewport_camera([4.0, -4.0, 2.5], [0.0, 0.0, 1.4])
        self._last_log = -1.0

    def _add_pivot_markers(self):
        """Draw the tilt-joint (centre of rotation) for each tiltable rotor: a
        bright red sphere at the joint + an orange rod along the tilt axis. These
        are pure visual gprims (no physics), so you can see exactly where the hinge
        is and read off any correction. The joint is fixed in the body frame at
        rotor.position, so the marker is placed at init_pos + rotor.position."""
        from pxr import UsdGeom, Gf, Sdf
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        ip = self.init_pos
        for r in self.vehicle._rotors:
            if not r.tiltable:
                continue
            w = (ip[0] + r.position[0], ip[1] + r.position[1], ip[2] + r.position[2])
            base = f"/World/pivot_markers/{r.name}"
            sph = UsdGeom.Sphere.Define(stage, Sdf.Path(base + "/point"))
            sph.GetRadiusAttr().Set(0.015)
            UsdGeom.Xformable(sph).AddTranslateOp().Set(Gf.Vec3d(*w))
            sph.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.05, 0.05)])
            axis = UsdGeom.Cylinder.Define(stage, Sdf.Path(base + "/axis"))   # tilt axis (Y)
            axis.GetRadiusAttr().Set(0.004)
            axis.GetHeightAttr().Set(0.40)
            axis.GetAxisAttr().Set(r.tilt_axis)
            UsdGeom.Xformable(axis).AddTranslateOp().Set(Gf.Vec3d(*w))
            axis.CreateDisplayColorAttr().Set([Gf.Vec3f(1.0, 0.45, 0.0)])
            carb.log_warn(
                f"[pivot] {r.name} tilt joint  world=({w[0]:.3f}, {w[1]:.3f}, {w[2]:.3f})  "
                f"body-frame position={tuple(round(v, 3) for v in r.position)}"
            )

    def _log(self):
        t = self.backend.t
        if t - self._last_log < 0.25:
            return
        self._last_log = t
        r = self.rotor
        omega = self.vehicle._omega[r.name]
        thrust = r.max_thrust * (omega / self.omega_max) ** 2
        throttle_cmd = self.backend._cmd.get(r.throttle_channel, 0.0)
        tilt_cmd = self.backend._cmd.get("tilt_left", 0.0)
        import math
        tilt_actual = math.degrees(self.vehicle._tilt_state.get("front_L", 0.0))
        carb.log_warn(
            f"[rotor-phys] t={t:5.2f}s  thr_cmd={throttle_cmd:.2f}  "
            f"omega={omega:6.1f} rad/s  T={thrust:5.2f} N  "
            f"tilt_cmd={tilt_cmd:.2f}  tilt={tilt_actual:5.1f} deg"
        )

    def run(self):
        self.timeline.play()
        carb.log_warn("Rotor physics demo: watch omega/thrust ramp at t=2s, tilt ease at t=6s.")
        while simulation_app.is_running():
            self.world.step(render=True)
            self._log()
        self.timeline.stop()
        simulation_app.close()


def main():
    RotorPhysicsApp().run()


if __name__ == "__main__":
    main()
