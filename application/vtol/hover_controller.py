"""
Reusable minimal hover stabiliser for the config-driven VTOL — a throwaway
stand-in for PX4 so the *plant* (thrust vectoring + aero + rotor dynamics) can be
validated open of the full HIL stack. Used by 02_vtol_hover.py and
07_vtol_cad_physics.py.

    altitude (world z) ── PID ─► total vertical thrust Fz
    attitude (level)   ── PD  ─► roll/pitch moments tau_x, tau_y   (yaw uncontrolled)
    [Fz, tau_x, tau_y] ─(pinv of allocation built from rotor positions)─► per-rotor thrust

The allocation is built from `vehicle._rotors`, so it flies any hover-capable VTOL
config (tri-tiltrotor, dual, the CAD quad, ...).
"""

import numpy as np
from scipy.spatial.transform import Rotation

import carb

from pegasus.simulator.logic.backends.backend import Backend

GRAVITY = 9.81


class HoverControllerBackend(Backend):
    """Altitude + attitude hold that allocates a hover wrench onto the rotors."""

    # ---- gains (tune in sim) ----
    KP_Z, KI_Z, KD_Z = 12.0, 3.0, 6.0        # altitude PID
    KP_ATT, KD_ATT = 4.0, 1.6                # attitude PD
    Z_INT_CLAMP = 5.0                        # altitude integrator anti-windup [N]

    def __init__(self, target_altitude: float = 1.5, forward_pulse: bool = False):
        super().__init__(config=None)
        self.z_des = target_altitude
        self.forward_pulse = forward_pulse
        self._z_int = 0.0
        self._t = 0.0
        self._cmd: dict = {}

    def initialize(self, vehicle):
        super().initialize(vehicle)
        self._rotors = vehicle._rotors
        self._surfaces = vehicle._surfaces
        self._mass = vehicle.total_mass

        # Each rotor (tilt=0, thrust +Z) at body (x,y,z) -> wrench column
        #   [Fz=1, tau_x=y, tau_y=-x, tau_z=0] * F  (matches VTOL._apply_propulsion).
        cols, self._max_thrust = [], []
        for r in self._rotors:
            x, y, _ = r.position
            cols.append([1.0, y, -x, 0.0])
            self._max_thrust.append(r.max_thrust)
        self._A = np.array(cols).T
        self._A_pinv = np.linalg.pinv(self._A)

        # With rotor_dynamics, thrust = max_thrust*(omega/omega_max)^2 ~ max_thrust*
        # throttle^2, so invert with sqrt; otherwise thrust = throttle*max_thrust.
        self._rotor_dynamics = bool(getattr(vehicle._cfg, "rotor_dynamics", False))

        # ALL tilt channels (both front servos on the CAD quad).
        self._tilt_channels = list(dict.fromkeys(
            r.tilt_channel for r in self._rotors if r.tiltable and r.tilt_channel
        ))
        carb.log_warn(
            f"[HoverController] mass={self._mass:.2f} kg, rotors={len(self._rotors)}, "
            f"hover thrust≈{self._mass * GRAVITY:.1f} N"
        )

    def input_reference(self):
        return self._cmd

    def update(self, dt: float):
        if self._vehicle is None or dt <= 0.0:
            return
        state = self._vehicle.state

        # 1) altitude hold (world frame)
        z = float(state.position[2]); vz = float(state.linear_velocity[2])
        ez = self.z_des - z
        self._z_int = float(np.clip(self._z_int + ez * dt, -self.Z_INT_CLAMP, self.Z_INT_CLAMP))
        Fz = self._mass * GRAVITY + self.KP_Z * ez + self.KI_Z * self._z_int - self.KD_Z * vz
        Fz = max(Fz, 0.0)

        # 2) attitude levelling (body frame)
        R = Rotation.from_quat(state.attitude)
        body_up_world = R.apply([0.0, 0.0, 1.0])
        err_body = R.inv().apply(np.cross(body_up_world, np.array([0.0, 0.0, 1.0])))
        omega = np.asarray(state.angular_velocity)
        tau = self.KP_ATT * err_body - self.KD_ATT * omega
        tau[2] = 0.0

        # 3) allocate wrench -> per-rotor throttle
        thrusts = self._A_pinv @ np.array([Fz, tau[0], tau[1], 0.0])
        cmd: dict = {}
        for rotor, thrust, t_max in zip(self._rotors, thrusts, self._max_thrust):
            frac = float(np.clip(thrust / t_max, 0.0, 1.0))
            cmd[rotor.throttle_channel] = np.sqrt(frac) if self._rotor_dynamics else frac

        # 4) tilt + surfaces
        tilt = self._tilt_command()
        for ch in self._tilt_channels:
            cmd[ch] = tilt
        for surface in self._surfaces:
            cmd[surface.deflect_channel] = 0.0

        self._cmd = cmd
        self._t += dt

    def _tilt_command(self) -> float:
        """Hover tilt (0), or a brief forward pulse to show thrust vectoring."""
        if not self.forward_pulse:
            return 0.0
        t = self._t
        if t < 4.0:  return 0.0
        if t < 6.0:  return 0.25 * (t - 4.0) / 2.0
        if t < 9.0:  return 0.25
        if t < 11.0: return 0.25 * (1.0 - (t - 9.0) / 2.0)
        return 0.0

    # --- unused Backend interface ---
    def update_sensor(self, sensor_type, data): pass
    def update_graphical_sensor(self, sensor_type, data): pass
    def update_state(self, state): pass
    def start(self): pass
    def stop(self): pass

    def reset(self):
        self._z_int = 0.0; self._t = 0.0; self._cmd = {}
