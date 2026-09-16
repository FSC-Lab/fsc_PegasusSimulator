"""
| File: vtol_mavlink_backend.py
| Description: PX4 MAVLink HIL backend specialised for the generic articulated VTOL.
|
| ============================================================================
|  WHAT THIS DOES  (Step 4 of the VTOL bring-up — the PX4 HIL mapping)
| ============================================================================
|
|  The stock PX4MavlinkBackend receives PX4's HIL_ACTUATOR_CONTROLS message and
|  maps EVERY channel to a rotor angular velocity — correct for a multirotor,
|  wrong for a VTOL whose outputs are a MIX of motors + tilt servos + control
|  surfaces.
|
|  This subclass reuses ALL of the base machinery (the MAVLink connection, HIL
|  sensor/GPS/heartbeat sending, lockstep handling, PX4 autolaunch ...) and only
|  changes the *last mile*: how the received `controls[16]` array is turned into
|  the command the VTOL vehicle consumes.
|
|  Data flow (direct_actuator offboard — see docs/direct_actuator_offboard.md):
|
|      ROS2 node ──DDS──► PX4 ──(passthrough, no controllers)──► actuator outputs
|                                                                      │
|                          HIL_ACTUATOR_CONTROLS.controls[16] (MAVLink)│
|                                                                      ▼
|      PX4VTOLMavlinkBackend.handle_control()  ── channel_map ──►  {channel: value}
|                                                                      │
|                          VTOL._read_command() / _drive_actuators() ◄─┘
|
|  >>> THE ONE THING YOU MUST VERIFY <<<
|  The index of each actuator inside `controls[]` is decided by PX4's OUTPUT
|  function assignment for your airframe — it is NOT guaranteed to match the
|  defaults below.  Capture the real layout by launching with
|  `PEGASUS_HIL_DEBUG=1` (the base class logs `controls[:8]`), arm in
|  direct_actuator, and read off which index moves when you command each motor /
|  servo.  Then adjust `channel_map`.
| ============================================================================
"""

__all__ = [
    "PX4VTOLMavlinkBackend",
    "PX4VTOLMavlinkBackendConfig",
    "default_tiltrotor_channel_map",
    "default_dual_tiltrotor_channel_map",
    "default_quad_tiltrotor_channel_map",
]

import math
import numpy as np
import time

import carb
from pymavlink import mavutil

from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)


# ----------------------------------------------------------------------------
#  CHANNEL MAP
#
#  Maps a VTOL command channel name -> (hil_index, scale, offset):
#
#      vtol_value = controls[hil_index] * scale + offset
#
#  The VTOL vehicle clamps each channel to its own natural range afterwards
#  (throttle/tilt -> [0,1], surfaces -> [-1,1]), so `scale`/`offset` only need
#  to convert PX4's normalisation to the VTOL's.
#
#  PX4 HIL normalisation conventions (typical SITL):
#      - motors  : controls[i] in [0, 1]   (cannot reverse)
#      - servos  : controls[i] in [-1, 1]
#
#  The default below assumes the PX4 "tiltrotor"-style output order:
#      0,1,2 = the three motors, 3 = tilt servo, 4,5 = elevons.
#  VERIFY against your airframe (see header note).
# ----------------------------------------------------------------------------

def default_tiltrotor_channel_map() -> dict:
    return {
        # --- motors: PX4 [0,1] -> throttle [0,1] (identity) ---
        "throttle_front_L": (0, 1.0, 0.0),
        "throttle_front_R": (1, 1.0, 0.0),
        "throttle_tail":    (2, 1.0, 0.0),

        # --- tilt servo: PX4 servo [-1,1] -> VTOL tilt [0,1] (0=hover, 1=cruise) ---
        #   (-1 -> 0, +1 -> 1).  If your servo already outputs [0,1], use
        #   (3, 1.0, 0.0); if the sense is reversed, use (3, -0.5, 0.5).
        "tilt":             (3, 0.5, 0.5),

        # --- elevons: PX4 servo [-1,1] -> VTOL elevon [-1,1] (identity) ---
        #   Flip a sign (scale -1.0) if a surface deflects the wrong way.
        "elevon_L":         (4, 1.0, 0.0),
        "elevon_R":         (5, 1.0, 0.0),
    }


def default_dual_tiltrotor_channel_map() -> dict:
    """Channel map for the dual INDEPENDENT-tiltrotor airframe
    (vtol.dual_tiltrotor_config).

    ASSUMED PX4 actuator-output order (VERIFY with PEGASUS_HIL_DEBUG=1) — must
    match the ActuatorMotors/ActuatorServos ordering streamed by the ROS2
    direct_actuator node:

        controls[0] motor_left     controls[1] motor_right      ([0,1] motors)
        controls[2] tilt_left      controls[3] tilt_right        ([-1,1] servos)
        controls[4] aileron        controls[5] elevator     controls[6] rudder
    """
    return {
        # motors: PX4 [0,1] -> throttle [0,1]
        "motor_left":  (0, 1.0, 0.0),
        "motor_right": (1, 1.0, 0.0),
        # tilt servos: PX4 [-1,1] -> VTOL tilt [0,1] (0=hover, 1=cruise).
        # If a servo's sense is reversed, use (idx, -0.5, 0.5).
        "tilt_left":   (2, 0.5, 0.5),
        "tilt_right":  (3, 0.5, 0.5),
        # surfaces: PX4 [-1,1] -> VTOL surface [-1,1] (flip scale sign if needed)
        "aileron":     (4, 1.0, 0.0),
        "elevator":    (5, 1.0, 0.0),
        "rudder":      (6, 1.0, 0.0),
    }


def default_quad_tiltrotor_channel_map() -> dict:
    """Channel map for the FSC CAD airframe (vtol.quad_tiltrotor_config):
    2 FRONT tilting rotors + 2 REAR static rotors, no control surfaces.

    ASSUMED PX4 actuator-output order (VERIFY with PEGASUS_HIL_DEBUG=1) — must match
    the ROS2 direct_actuator node's MOTOR/SERVO ordering for this airframe:

        controls[0] motor_front_L   controls[1] motor_front_R    ([0,1] motors)
        controls[2] motor_rear_L    controls[3] motor_rear_R
        controls[4] tilt_left       controls[5] tilt_right        ([-1,1] servos)
        controls[6] aileron    controls[7] elevator   controls[8] rudder
    """
    return {
        "motor_front_L": (0, 1.0, 0.0),
        "motor_front_R": (1, 1.0, 0.0),
        "motor_rear_L":  (2, 1.0, 0.0),
        "motor_rear_R":  (3, 1.0, 0.0),
        # tilt servos: OFFSET 0 (not 0.5!). PX4 outputs ~0 on the tilt channels in
        # MC hover, so offset 0 keeps the rotors vertical (tilt=0 = non-tilting
        # base); the small differential PX4 adds for yaw then passes through as a
        # few degrees of differential tilt -> yaw-by-tilt authority. scale caps the
        # max tilt so it can't swing to full cruise. (The old (4,0.5,0.5) mapped
        # hover's 0 -> 0.5 = 45deg, which is why it tilted on arm.)
        "tilt_left":     (4, 1.0, 0.0),
        "tilt_right":    (5, 1.0, 0.0),
        # control surfaces: PX4 [-1,1] -> VTOL surface [-1,1] (flip scale if reversed)
        "aileron":       (6, 1.0, 0.0),
        "elevator":      (7, 1.0, 0.0),
        "rudder":        (8, 1.0, 0.0),
    }


class PX4VTOLMavlinkBackendConfig(PX4MavlinkBackendConfig):
    """Same as PX4MavlinkBackendConfig, plus a `channel_map`.

    Extra config keys (on top of the base ones documented in
    PX4MavlinkBackendConfig):
        channel_map (dict): {channel_name: (hil_index, scale, offset)}.
    """

    def __init__(self, config: dict | None = None):
        config = {} if config is None else config
        super().__init__(config)
        self.channel_map: dict = config.get("channel_map", default_tiltrotor_channel_map())
        # Seconds of HIL silence after which the vehicle falls back to the
        # disarmed/safe command instead of holding the last one forever.
        self.control_timeout: float = float(config.get("control_timeout", 0.25))


class PX4VTOLMavlinkBackend(PX4MavlinkBackend):
    """PX4 HIL backend that emits a VTOL channel dict instead of rotor speeds."""

    def __init__(self, config: PX4VTOLMavlinkBackendConfig | None = None):
        # NOTE: default is None, not a config instance -- a default argument is
        # evaluated ONCE at import time and would be shared by every backend
        # (see the checklist at the top of CLAUDE.md).
        if config is None:
            config = PX4VTOLMavlinkBackendConfig()
        super().__init__(config)

        # The mapping table and the latest command we hand to the vehicle.
        self._channel_map: dict = getattr(config, "channel_map", default_tiltrotor_channel_map())
        self._command: dict = {}        # {} == disarmed/safe: motors off, tilt -> hover
        self._last_armed: bool = False

        # HIL watchdog. Without it, a dropped link (PX4 crash, DDS stall, a
        # paused debugger) leaves input_reference() returning the LAST command
        # forever -- motors and tilt frozen mid-flight, which is the worst
        # possible failure mode and looks identical to a controller hang.
        self._control_timeout: float = float(getattr(config, "control_timeout", 0.25))
        self._last_control_time: float = 0.0
        self._timeout_warned: bool = False

    # --- the ONLY behavioural override: how PX4 outputs become a VTOL command ---
    def handle_control(self, time_usec, controls, mode, flags):
        """Called by the base receive loop for each HIL_ACTUATOR_CONTROLS msg.

        We deliberately DO NOT call super().handle_control(): the base maps the
        controls onto multirotor rotor velocities, which a VTOL does not use."""

        # PX4 sets MAV_MODE_FLAG_SAFETY_ARMED (128) when armed (mode == 129 typ.).
        armed = bool(mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        self._last_armed = armed

        self._command = self._map_controls(controls) if armed else {}
        self._last_control_time = time.monotonic()
        self._timeout_warned = False

        # Optional bring-up logging (export PEGASUS_HIL_DEBUG=1). Shows both the
        # raw PX4 outputs and the channel dict we produced — exactly what you
        # need to validate / fix the channel_map.
        if self._hil_debug:
            self._hil_dbg_actuator_msgs += 1
            import time as _time
            now = _time.time()
            if (now - self._hil_dbg_last_log) > 0.5:
                self._hil_dbg_last_log = now
                raw = [round(float(c), 3) for c in controls[:8]]
                mapped = {k: round(v, 3) for k, v in self._command.items()}
                carb.log_warn(
                    f"[VTOL_HIL vid={self._vehicle_id}] armed={armed} mode={mode} "
                    f"controls[:8]={raw} -> {mapped}"
                )

    def _map_controls(self, controls) -> dict:
        """Apply the channel_map: controls[idx]*scale + offset, NaN -> 0."""
        command: dict = {}
        n = len(controls)
        for channel, (idx, scale, offset) in self._channel_map.items():
            if 0 <= idx < n:
                raw = float(controls[idx])
                if math.isnan(raw):
                    raw = 0.0
                command[channel] = raw * scale + offset
        return command

    # --- what the VTOL vehicle pulls each physics step ---
    def input_reference(self):
        """Return the latest VTOL channel dict (see VTOL._read_command), or the
        disarmed/safe empty dict if the HIL link has gone quiet."""
        if self._command and self._control_timeout > 0.0:
            silent = time.monotonic() - self._last_control_time
            if silent > self._control_timeout:
                if not self._timeout_warned:
                    self._timeout_warned = True
                    carb.log_warn(
                        f"[VTOL_HIL vid={self._vehicle_id}] no HIL_ACTUATOR_CONTROLS for "
                        f"{silent:.2f}s (> {self._control_timeout:.2f}s) — falling back to the "
                        f"safe command (motors off, tilt to hover). Check PX4 / the DDS agent."
                    )
                self._command = {}
        return self._command

    # --- synthesize an AIRSPEED sensor from the (sim/mocap) velocity ---------
    # A VTOL airframe fails PX4 preflight with "Airspeed invalid" unless it has a
    # valid airspeed. The simulator already knows the vehicle velocity (the same
    # thing the mocap measures), so we feed a real pitot reading instead of
    # disabling the check: forward (body-x) speed -> dynamic pressure
    # q = 0.5*rho*V*|V|, sent as HIL_SENSOR.diff_pressure (units: hPa; PX4
    # converts *100 -> Pa). Hover V~0 -> q~0 (a valid zero reading); forward
    # flight tracks groundspeed so PX4's airspeed<->groundspeed check is
    # satisfied. To make PX4 USE it, set SYS_HAS_NUM_ASPD=1 (airframe 4101);
    # SYS_HAS_NUM_ASPD=0 just ignores it (safe fallback).
    AIR_DENSITY: float = 1.225  # kg/m^3 (ISA sea level)

    def update_state(self, state):
        super().update_state(state)
        try:
            v_fwd = float(state.get_linear_body_velocity_ned_frd()[0])  # body-x forward [m/s]
            # ★ AIR-relative (piece 4, 2026-09-15): the vehicle writes its world wind onto the state
            # (VTOL_WIND / VTOL_TURB); a pitot reads the air, not the ground. Still air: unchanged.
            w = getattr(state, "wind_world", None)
            if w is not None and np.any(np.asarray(w, dtype=float) != 0.0):
                from scipy.spatial.transform import Rotation
                w_body_flu = Rotation.from_quat(state.attitude).inv().apply(np.asarray(w, dtype=float))
                v_fwd -= float(w_body_flu[0])          # FLU x == FRD x
        except Exception:
            return
        q_pa = 0.5 * self.AIR_DENSITY * v_fwd * abs(v_fwd)  # signed dynamic pressure [Pa]
        self._sensor_data.diff_pressure = q_pa / 100.0       # HIL_SENSOR diff_pressure [hPa]
        self._sensor_data.new_press_data = True
