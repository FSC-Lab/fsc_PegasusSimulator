#!/usr/bin/env python
"""
| File: x650_bare_frame_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2026, Longhao Qian. All rights reserved.
| Description: Spawn helper for the bare X650 airframe (x650.usd - stock rotor/body naming,
|   no robotic arm) on the STOCK Pegasus Multirotor/MultirotorConfig, calibrated against the
|   MN4014 + 15x5" propeller bench test. Distinct from x650_rotorcraft_utils.py, which spawns
|   the arm-equipped X650 (AM_realign.usda) on the FSC MultirotorMod/VehicleMod subclasses.
"""

import os

from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from fsc_aerial_manipulation.rotorcraft.lagged_thrust_curve import LaggedQuadraticThrustCurve

# Resolve the X650 asset (x650.usd) from the installed package so the path holds
# regardless of where the repo lives.
X650_USD = os.path.join(os.path.dirname(__file__), "assets", "x650.usd")

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
# working against (a) x650.usd's body inertia set to the true CAD-derived values (NOT
# mass-ratio-scaled - see CLAUDE.md's "X650 CAD inertia correction" note) and (b) X650-specific
# PX4 rate/attitude gain tuning, applied automatically at launch by
# scripts/apply_x650_px4_gains.sh (all 3 X650 launch scripts call it). See CLAUDE.md's "X650 PX4
# gain tuning" section for the full diagnostic history (the untuned generic none_iris defaults
# oscillated/crashed across a range of test lambda values from 10.51 up to 14.5, before the real
# fix - retuning PX4, not raising lambda - was identified).
X650_ROTOR_LAMBDA = 10.51  # 1/s
# X650_ROTOR_LAMBDA = 200  # test


def spawn_x650_with_mavlink(
    px4_path,
    px4_default_airframe,
    vehicle_id: int = 0,
    spawn_pos=(0.0, 0.0, 0.07),
    spawn_euler=(0.0, 0.0, 0.0),
    connection_ip: str = "127.0.0.1",
    connection_baseport: int = 4560,
    enable_lockstep: bool = True,
):
    """Spawn the bare X650 airframe with a stock Pegasus ``Multirotor`` + PX4 MAVLink
    and ROS2 backends, calibrated against the MN4014+15x5" bench test. No PX4 SITL is
    launched here (it runs in a separate terminal).

    The X650 asset (x650.usd) follows the stock naming convention, so the stock
    Multirotor works unmodified — its hardcoded "/body", "/rotor0".."/rotor3" and
    "joint0".."joint3" line up with the asset. We spawn at stage_prefix
    "/World/quadrotor_<id>" so the referenced /x650 children land directly under
    it (/World/quadrotor_<id>/body, /rotor0, ...).

    PX4 is backend[0] (primary): PX4/QGroundControl drives the motors; the ROS2
    backend only publishes vehicle state.

    Returns:
        str: the vehicle stage prefix (drone root path).
    """
    quat_xyzw = Rotation.from_euler("XYZ", spawn_euler, degrees=True).as_quat()

    # ----- PX4 MAVLink backend (primary motor command source = backend[0]) -----
    # input_offset/input_scaling/zero_position_armed calibrated against the MN4014+
    # 15x5" bench test above, so PX4's normalized 0-1 motor commands map onto the
    # bench-measured rotor speed range, replacing the stock Iris-tuned defaults
    # (input_scaling=1000, zero_position_armed=100).
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
        "input_scaling": [X650_MAX_ROTOR_VEL - X650_ZERO_POSITION_ARMED] * 4,
        "zero_position_armed": [X650_ZERO_POSITION_ARMED] * 4,
    })

    # ----- ROS2 backend (state publishing only; PX4 has motor authority) -----
    ros2_backend = ROS2Backend(
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
    # overridden below with the MN4014+15x5" bench-test calibration (rot_dir stays
    # the default [-1,-1,1,1], the iris/none_iris spin convention the x650 rotors
    # follow - see x650.usd's rotor_clock/rotor_counter_clock mesh pairing).
    # backend[0] = PX4 (primary).
    config = MultirotorConfig()
    config.backends = [PX4MavlinkBackend(mavlink_config), ros2_backend]
    config.thrust_curve = LaggedQuadraticThrustCurve(config={
        "rotor_constant": [X650_ROTOR_CONSTANT] * 4,
        "rolling_moment_coefficient": [X650_ROLLING_MOMENT_COEFFICIENT] * 4,
        "min_rotor_velocity": [X650_MIN_ROTOR_VEL] * 4,
        "max_rotor_velocity": [X650_MAX_ROTOR_VEL] * 4,
        "rotor_lambda": [X650_ROTOR_LAMBDA] * 4,
    })

    drone_prim_path = f"/World/quadrotor_{vehicle_id}"

    vehicle = Multirotor(
        stage_prefix=drone_prim_path,
        usd_file=X650_USD,
        vehicle_id=vehicle_id,
        init_pos=spawn_pos,
        init_orientation=quat_xyzw,
        config=config,
    )

    # Return the resolved stage prefix (get_stage_next_free_path may append a
    # suffix if the path was taken; here it stays "/World/quadrotor_<id>").
    return vehicle._stage_prefix
