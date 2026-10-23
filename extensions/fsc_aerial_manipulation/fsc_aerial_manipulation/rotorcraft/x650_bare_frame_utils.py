#!/usr/bin/env python
"""
| File: x650_bare_frame_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2026, Longhao Qian. All rights reserved.
| Description: Spawn helper for the bare X650 airframe (x650_new.usd - corrected model rotation
|   and PX4 motor order, with stock rotor/body naming,
|   no robotic arm) on the STOCK Pegasus Multirotor/MultirotorConfig, calibrated against the
|   MN4014 + 15x5" propeller bench test. Distinct from x650_rotorcraft_utils.py, which spawns
|   the arm-equipped X650 (AM_realign.usda) on the FSC MultirotorMod/VehicleMod subclasses.
"""

import os

import numpy as np

from scipy.spatial.transform import Rotation
from pxr import Gf, UsdPhysics

from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.dynamics.linear_drag import LinearDrag
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from fsc_aerial_manipulation.rotorcraft.lagged_thrust_curve import LaggedQuadraticThrustCurve
from fsc_aerial_manipulation.rotorcraft import x650_params

# The corrected model is the sole bare-X650 asset. Do not add an environment
# override or fallback to the legacy wrong-frame model: every bare-X650
# scenario must share this geometry, frame convention, and motor ordering.
X650_USD = os.path.abspath(os.path.join(os.path.dirname(__file__), "assets", "x650_new.usd"))

# x650_new.usd corrects geometry orientation and motor/propeller ordering, but
# its body mesh currently authors a 1.467 kg mass. Preserve the validated
# established mass properties: body + four authored rotor masses = 3.5 kg total.
X650_BODY_MASS = 3.416079044342041  # kg
X650_BODY_INERTIA = (0.05777498, 0.06408172, 0.065004565)  # kg*m^2
# The four rotor bodies the asset authors (3.5 kg total - body): total = body_mass + this.
X650_ROTOR_MASS_TOTAL = 3.5 - X650_BODY_MASS  # kg

# ── MN4014 + 15x5" prop bench-test calibration ──────────────────────────────
# Source: docs/propeller_testing/MN_4014_15x5_report.pdf, Step 1 polynomial fits
# (throttle x in 0-100%): rpm(x) = 70.2593x + 781.49, thrust(x) [N] =
# 0.0020455x^2 + 0.0960918x - 0.361797, torque(x) [N.m] = -2.94144e-07x^3 +
# 7.56245e-05x^2 + 0.00046387x + 0.0104334. The Step 3 lambda (spin-up-lag) model
# IS now applied via LaggedQuadraticThrustCurve (rotorcraft/lagged_thrust_curve.py,
# X650_ROTOR_LAMBDA below) - it significantly affects the dynamics, per user report
# 2026-07-13 - not just the Step 1 throttle->RPM/thrust/torque curve as before.
#
# rpm(x) is linear in x, and PX4MavlinkBackend's own controls(0-1)->omega[rad/s]
# map (omega = (controls + input_offset) * input_scaling + zero_position_armed)
# is also linear, so the two can be matched exactly (not just at the endpoints)
# by solving for the scaling/offset at controls=0 and controls=1:
#   omega(controls=0) = rpm(0)   * 2*pi/60 = 81.8374 rad/s
#   omega(controls=1) = rpm(100) * 2*pi/60 = 817.5911 rad/s
#
# X650_ZERO_POSITION_ARMED is the idle-at-0%-throttle speed *once armed* - matches
# real ESC behavior (motors don't fully stop at zero stick while armed) and is only
# ever applied by PX4MavlinkBackend.update_input_reference(), which handle_control()
# only calls when PX4 reports armed; when disarmed it calls zero_input_reference()
# instead (input_reference forced to exactly 0), bypassing this value entirely.
#
# X650_MIN_ROTOR_VEL, by contrast, is QuadraticThrustCurve's own floor on rotor
# speed and is NOT armed-gated - QuadraticThrustCurve.update() does
# `velocity = np.maximum(min_rotor_velocity, ...)` unconditionally. Setting this to
# the same nonzero idle value (as an earlier version of this file did) forced the
# rotors to keep spinning even while disarmed - confirmed live (2026-07-13): props
# were visibly spinning on the ground before arming. Stock Iris's own default is
# min_rotor_velocity=[0,0,0,0] for exactly this reason. Kept at 0 here too - the
# idle-while-armed behavior is fully and correctly handled by
# X650_ZERO_POSITION_ARMED above, so this needs no floor of its own.
X650_ZERO_POSITION_ARMED = 81.8374  # rad/s, == rpm(throttle=0%) in rad/s - armed-idle only
X650_MIN_ROTOR_VEL = 0.0            # rad/s - unconditional floor; must allow true zero (disarmed)
X650_MAX_ROTOR_VEL = 817.5911       # rad/s, == rpm(throttle=100%) in rad/s

# thrust/torque vs. throttle are fit against RPM (via the linear rpm(x) above), then
# projected onto the sim's fixed force = k*omega^2 / torque = c*omega^2 form via an
# unweighted zero-intercept least-squares fit sampled across throttle=0-100% (no
# raw CSV data available to fit force/torque vs. omega^2 directly - see CLAUDE.md).
# Fit quality: thrust RMSE 0.31 N (range 0-30 N), torque RMSE 0.0136 N.m (range
# 0.01-0.52 N.m) - the fixed-through-origin sim model can't capture the fitted
# curve's small nonzero intercept at throttle=0, but tracks it closely elsewhere.
X650_ROTOR_CONSTANT = 4.536223e-05              # N / (rad/s)^2
X650_ROLLING_MOMENT_COEFFICIENT = 8.366000e-07  # N.m / (rad/s)^2

# Step 3 lambda (rotor spin-up bandwidth, 1/s): bench-measured value is the mean of the report's
# lag_all/lag_filtered variants (10.519/10.501) - the two system-ID estimates derived directly
# from the measured step-response timing (lambda=1/tau), not direct_all/direct_filtered (noisier,
# up to ~24% unphysical negative values - see lagged_thrust_curve.py's header comment for the
# full rationale). tau = 1/lambda ~= 95ms, within the report's own physically-expected 70-200ms
# range.
#
# SETTLED (2026-07-14): the true bench-measured value (10.51) now flies cleanly - confirmed
# working against (a) the X650's true CAD-derived body inertia values (NOT
# mass-ratio-scaled - see CLAUDE.md's "X650 CAD inertia correction" note) and (b) X650-specific
# PX4 rate/attitude gain tuning, applied automatically at launch by
# scripts/apply_x650_px4_gains.sh (all 3 X650 launch scripts call it). See CLAUDE.md's "X650 PX4
# gain tuning" section for the full diagnostic history (the untuned generic none_iris defaults
# oscillated/crashed across a range of test lambda values from 10.51 up to 14.5, before the real
# fix - retuning PX4, not raising lambda - was identified).
X650_ROTOR_LAMBDA = 10.51  # 1/s
# X650_ROTOR_LAMBDA = 200  # test

# ── Selectable motor/propeller calibration sets (added 2026-08-05) ──────────
# Everything else about this vehicle -- asset, mass, inertia, rotor geometry,
# rotation directions, backends, sensors -- is shared across motor variants.
# Only the numbers in this table change, which is exactly the axis along which
# these scenarios are meant to differ, so add an entry here rather than cloning
# this module.
#
# "MN4014" is the historical default and reproduces the pre-2026-08-05 behaviour
# exactly; every existing caller gets it without passing anything. MN4010's
# constants and their derivation live in x650_params.py.
MOTOR_CALIBRATIONS = {
    "MN4014": {
        "report": "docs/propeller_testing/MN_4014_15x5_report.pdf",
        "zero_position_armed": X650_ZERO_POSITION_ARMED,
        "min_rotor_velocity": X650_MIN_ROTOR_VEL,
        "max_rotor_velocity": X650_MAX_ROTOR_VEL,
        "rotor_constant": X650_ROTOR_CONSTANT,
        "rolling_moment_coefficient": X650_ROLLING_MOMENT_COEFFICIENT,
        "rotor_lambda": X650_ROTOR_LAMBDA,
    },
    "MN4010": {
        "report": "docs/propeller_testing/MN_4010_15x5_report.pdf",
        "zero_position_armed": x650_params.MN4010_ZERO_POSITION_ARMED,
        "min_rotor_velocity": x650_params.MN4010_MIN_ROTOR_VEL,
        "max_rotor_velocity": x650_params.MN4010_MAX_ROTOR_VEL,
        "rotor_constant": x650_params.MN4010_ROTOR_CONSTANT,
        "rolling_moment_coefficient": x650_params.MN4010_ROLLING_MOMENT_COEFFICIENT,
        "rotor_lambda": x650_params.MN4010_ROTOR_LAMBDA,
    },
}
DEFAULT_MOTOR = "MN4014"


def apply_x650_mass_properties(vehicle, body_mass=None, body_inertia=None) -> None:
    """Apply the validated 3.5 kg X650 mass model to the corrected USD.

    ``body_mass``/``body_inertia`` override it for airframes that reuse this
    geometry with a different mass budget (e.g. the 2.95 kg T650). Both default
    to the X650's own validated values, so existing callers are unaffected.
    """
    body_mesh_path = f"{vehicle._stage_prefix}/body/body"
    body_mesh = vehicle._world.stage.GetPrimAtPath(body_mesh_path)
    if not body_mesh or not body_mesh.IsValid():
        raise RuntimeError(f"X650 body mass prim did not resolve: {body_mesh_path}")

    mass_api = UsdPhysics.MassAPI(body_mesh)
    if not mass_api:
        mass_api = UsdPhysics.MassAPI.Apply(body_mesh)
    mass_api.GetMassAttr().Set(X650_BODY_MASS if body_mass is None else body_mass)
    mass_api.GetDiagonalInertiaAttr().Set(
        Gf.Vec3f(*(X650_BODY_INERTIA if body_inertia is None else body_inertia))
    )


# Real-time I/O budget (opt-in, realtime_io=True; port of t650_bare_frame_utils' 2026-10-02 change):
# at 250 Hz a physics step has 4 ms of wall time. Two per-step costs buy nothing indoors:
#  * the ground-truth ROS 2 backend's rclpy.spin_once on EVERY step (~0.3 ms) although it has no
#    subscriptions (sub_control False); publishing does not need the spin. It then spins every
#    ROS2_SPIN_EVERY steps (10 Hz), which still serves the node's parameter services.
#  * the magnetometer and GPS models at the 250 Hz default (~0.2 ms): they then run at MAG_RATE_HZ /
#    GPS_RATE_HZ (typical real-sensor rates). IMU and barometer stay at 250 Hz.
ROS2_SPIN_EVERY = 25
MAG_RATE_HZ = 50.0
GPS_RATE_HZ = 10.0


class _ThrottledROS2Backend(ROS2Backend):
    """ROS2Backend whose executor spins every ROS2_SPIN_EVERY physics steps instead of every step."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._spin_count = 0

    def update(self, dt: float):
        self._spin_count += 1
        if self._spin_count >= ROS2_SPIN_EVERY:
            self._spin_count = 0
            super().update(dt)


class X650Multirotor(Multirotor):
    """Multirotor plus an optional rotational (aerodynamic) damping torque on /body:
    tau_drag = -diag(angular_drag) * omega_body  [N m], omega in body FLU [rad/s].
    With angular_drag all zero it is the stock Multirotor."""

    def __init__(self, *args, angular_drag=(0.0, 0.0, 0.0), **kwargs):
        self._angular_drag = np.asarray(angular_drag, dtype=float)
        self.last_angular_drag_torque = np.zeros(3)
        super().__init__(*args, **kwargs)

    def update(self, dt: float):
        super().update(dt)
        if np.any(self._angular_drag):
            tau = -self._angular_drag * np.asarray(self._state.angular_velocity, dtype=float)
            self.last_angular_drag_torque = tau
            self.apply_torque(tau.tolist(), "/body")


def spawn_x650_with_mavlink(
    px4_path,
    px4_default_airframe,
    vehicle_id: int = 0,
    spawn_pos=(0.0, 0.0, 0.07),
    spawn_euler=(0.0, 0.0, 0.0),
    connection_ip: str = "127.0.0.1",
    connection_baseport: int = 4560,
    enable_lockstep: bool = True,
    motor: str = DEFAULT_MOTOR,
    body_mass=None,
    body_inertia=None,
    linear_drag=None,
    angular_drag=None,
    realtime_io: bool = False,
    rotor_lambda=None,
):
    """Spawn the bare X650 airframe with a stock Pegasus ``Multirotor`` + PX4 MAVLink
    and ROS2 backends, calibrated against the MN4014+15x5" bench test. No PX4 SITL is
    launched here (it runs in a separate terminal).

    The X650 asset (x650_new.usd) follows the stock naming convention, so the stock
    Multirotor works unmodified — its hardcoded "/body", "/rotor0".."/rotor3" and
    "joint0".."joint3" line up with the asset. We spawn at stage_prefix
    "/World/quadrotor_<id>" so the referenced /x650 children land directly under
    it (/World/quadrotor_<id>/body, /rotor0, ...).

    PX4 is backend[0] (primary): PX4/QGroundControl drives the motors; the ROS2
    backend only publishes vehicle state.

    Args:
        motor: key into ``MOTOR_CALIBRATIONS`` -- "MN4014" (default, the X650's
            own bench calibration) or "MN4010" (the T650's). Selects the rotor
            speed range, thrust/torque constants and spin-up lambda together;
            these must never be mixed across motors.
        body_mass, body_inertia: override the X650's 3.416 kg / CAD inertia for
            airframes reusing this geometry at a different mass (e.g. T650).
        linear_drag: body-FLU linear drag [N/(m/s)]; None keeps the Pegasus default
            LinearDrag([0.50, 0.30, 0.0]).
        angular_drag: body-FLU rotational damping [N m/(rad/s)]; None/zero adds none.
        rotor_lambda: override the motor's spin-up lag lambda [1/s] (None: the
            calibration's), e.g. for rotor-lag robustness tests.
        realtime_io: throttle the ground-truth ROS 2 spin and run mag/GPS at
            MAG_RATE_HZ/GPS_RATE_HZ (see above). False keeps the stock per-step I/O.

    Returns:
        str: the vehicle stage prefix (drone root path).
    """
    if not os.path.isfile(X650_USD):
        raise FileNotFoundError(f"Corrected X650 USD not found: {X650_USD}")
    if motor not in MOTOR_CALIBRATIONS:
        raise ValueError(
            f"Unknown motor calibration {motor!r}; "
            f"available: {sorted(MOTOR_CALIBRATIONS)}"
        )
    cal = dict(MOTOR_CALIBRATIONS[motor])
    if rotor_lambda is not None:
        print(f"[X650] ROTOR LAG OVERRIDE: lambda {cal['rotor_lambda']} -> {float(rotor_lambda)} 1/s "
              f"(tau {1.0 / float(rotor_lambda):.4f} s)", flush=True)
        cal["rotor_lambda"] = float(rotor_lambda)
    print(f"[X650] Using corrected asset: {X650_USD}", flush=True)
    print(
        f"[X650] Motor calibration: {motor} ({cal['report']}) | "
        f"omega {cal['zero_position_armed']:.2f}..{cal['max_rotor_velocity']:.2f} rad/s | "
        f"k={cal['rotor_constant']:.6e} | lambda={cal['rotor_lambda']} 1/s",
        flush=True,
    )

    quat_xyzw = Rotation.from_euler("XYZ", spawn_euler, degrees=True).as_quat()

    # ----- PX4 MAVLink backend (primary motor command source = backend[0]) -----
    # input_offset/input_scaling/zero_position_armed calibrated against the
    # selected motor's bench test, so PX4's normalized 0-1 motor commands map onto
    # the bench-measured rotor speed range, replacing the stock Iris-tuned
    # defaults (input_scaling=1000, zero_position_armed=100).
    mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": vehicle_id,
        "connection_type": "tcpin",
        "connection_ip": connection_ip,
        "connection_baseport": connection_baseport,   # Pegasus listens on 4560+i
        "enable_lockstep": enable_lockstep,
        "px4_autolaunch": False,
        "px4_dir": px4_path,
        "px4_vehicle_model": px4_default_airframe,
        "input_offset": [0.0, 0.0, 0.0, 0.0],
        "input_scaling": [cal["max_rotor_velocity"] - cal["zero_position_armed"]] * 4,
        "zero_position_armed": [cal["zero_position_armed"]] * 4,
    })

    # ----- ROS2 backend (state publishing only; PX4 has motor authority) -----
    ros2_backend = (_ThrottledROS2Backend if realtime_io else ROS2Backend)(
        vehicle_id=vehicle_id,
        config={
            "namespace": "uav_",
            "pub_sensors": False,
            "pub_graphical_sensors": False,
            "pub_state": True,
            "pub_twist": True,
            "pub_accel": True,
            "pub_twist_inertial": True,
            "pub_tf": True,
            "sub_control": False,   # PX4 is primary → don't accept ROS2 rotor commands
        },
    )

    # Stock MultirotorConfig: default sensors (Baro/IMU/Mag/GPS). Thrust curve is
    # overridden below with the selected motor's bench-test calibration. The
    # corrected asset uses PX4 order: rotor0 front-right and rotor1 rear-left
    # are counter-clockwise; rotor2 front-left and rotor3 rear-right are
    # clockwise. Pegasus represents that convention as [-1, -1, 1, 1].
    # backend[0] = PX4 (primary).
    config = MultirotorConfig()
    config.backends = [PX4MavlinkBackend(mavlink_config), ros2_backend]
    if realtime_io:
        config.sensors = [Barometer(), IMU(), Magnetometer({"update_rate": MAG_RATE_HZ}),
                          GPS({"update_rate": GPS_RATE_HZ})]
        print(f"[X650] sensors: IMU + baro 250 Hz, mag {MAG_RATE_HZ:.0f} Hz, GPS {GPS_RATE_HZ:.0f} Hz; "
              f"ground-truth ROS 2 executor spun every {ROS2_SPIN_EVERY} steps", flush=True)
    if linear_drag is not None:
        config.drag = LinearDrag([float(c) for c in linear_drag])
    angular_drag = tuple(float(c) for c in (angular_drag or (0.0, 0.0, 0.0)))
    config.thrust_curve = LaggedQuadraticThrustCurve(config={
        "rotor_constant": [cal["rotor_constant"]] * 4,
        "rolling_moment_coefficient": [cal["rolling_moment_coefficient"]] * 4,
        "min_rotor_velocity": [cal["min_rotor_velocity"]] * 4,
        "max_rotor_velocity": [cal["max_rotor_velocity"]] * 4,
        "rotor_lambda": [cal["rotor_lambda"]] * 4,
        "rot_dir": [-1, -1, 1, 1],
    })

    drone_prim_path = f"/World/quadrotor_{vehicle_id}"

    vehicle_kwargs = dict(
        stage_prefix=drone_prim_path,
        usd_file=X650_USD,
        vehicle_id=vehicle_id,
        init_pos=spawn_pos,
        init_orientation=quat_xyzw,
        config=config,
    )
    vehicle = (X650Multirotor(angular_drag=angular_drag, **vehicle_kwargs) if any(angular_drag)
               else Multirotor(**vehicle_kwargs))
    apply_x650_mass_properties(vehicle, body_mass=body_mass, body_inertia=body_inertia)

    # Return the resolved stage prefix (get_stage_next_free_path may append a
    # suffix if the path was taken; here it stays "/World/quadrotor_<id>").
    return vehicle._stage_prefix
