#!/usr/bin/env python
"""
| File: t650_bare_frame_utils.py
| License: BSD-3-Clause
| Description: Spawn helper for the bare T650 (Tarot 650) airframe on the STOCK Pegasus
|   Multirotor/MultirotorConfig, calibrated against the MN4010 + 15x5" propeller bench test
|   (docs/propeller_testing/MN_4010_15x5_report.pdf).
|
|   Parallel to x650_bare_frame_utils.py, deliberately standalone: the X650 and T650 are
|   independent airframes and neither module imports the other, so a change to one cannot
|   silently move the other. All T650 constants come from t650_params.py.
|
|   THE USD ASSET IS SHARED. The T650 reuses x650_new.usd because no separate T650 asset
|   exists -- the two builds have the same frame geometry, rotor ordering and arm length,
|   and differ only in motors and mass. The asset's authored body mass is overridden at
|   spawn time (see apply_t650_mass_properties); its four authored rotor bodies are left
|   alone and PhysX adds them on top.
"""

import os

from scipy.spatial.transform import Rotation
from pxr import Gf, UsdPhysics

from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
from fsc_aerial_manipulation.rotorcraft.lagged_thrust_curve import LaggedQuadraticThrustCurve
from fsc_aerial_manipulation.rotorcraft import t650_params
from fsc_aerial_manipulation.utils.inertia_utils import author_inertia_tensor

# Shared with the X650 on purpose -- see the module docstring. Do not add an environment
# override or a fallback to the legacy wrong-frame model: every bare-airframe scenario must
# share this geometry, frame convention and motor ordering.
T650_USD = os.path.abspath(os.path.join(os.path.dirname(__file__), "assets", "x650_new.usd"))


def apply_t650_mass_properties(vehicle, body_mass=None, body_inertia=None) -> None:
    """Write the T650 mass model onto the shared USD's body mesh.

    x650_new.usd authors ~1.467 kg on its body mesh, which is the CAD frame part's own mass
    and not the flight mass of either vehicle, so it is always overridden here.
    """
    body_mesh_path = f"{vehicle._stage_prefix}/body/body"
    body_mesh = vehicle._world.stage.GetPrimAtPath(body_mesh_path)
    if not body_mesh or not body_mesh.IsValid():
        raise RuntimeError(f"T650 body mass prim did not resolve: {body_mesh_path}")

    mass_api = UsdPhysics.MassAPI(body_mesh)
    if not mass_api:
        mass_api = UsdPhysics.MassAPI.Apply(body_mesh)
    mass_api.GetMassAttr().Set(
        t650_params.BODY_MASS if body_mass is None else body_mass
    )
    # Authors diagonalInertia + principalAxes: USD cannot store products of inertia, so a
    # straight diagonalInertia write would silently drop INERTIA_TENSOR's Ixy term.
    author_inertia_tensor(
        mass_api, t650_params.INERTIA_TENSOR if body_inertia is None else body_inertia
    )


def spawn_t650_with_mavlink(
    px4_path,
    px4_default_airframe,
    vehicle_id: int = 0,
    spawn_pos=(0.0, 0.0, 0.07),
    spawn_euler=(0.0, 0.0, 0.0),
    connection_ip: str = "127.0.0.1",
    connection_baseport: int = 4560,
    enable_lockstep: bool = True,
    body_mass=None,
    body_inertia=None,
):
    """Spawn the bare T650 airframe with a stock Pegasus ``Multirotor`` plus PX4 MAVLink and
    ROS 2 backends, calibrated against the MN4010 + 15x5" bench test. No PX4 SITL is
    launched here (it runs in a separate terminal).

    The shared asset follows the stock naming convention, so the stock ``Multirotor`` works
    unmodified -- its hardcoded "/body", "/rotor0".."/rotor3" and "joint0".."joint3" line up
    with it. The vehicle is spawned at stage prefix "/World/quadrotor_<id>" so the
    referenced children land directly underneath.

    PX4 is backend[0] (primary): PX4/QGroundControl drives the motors; the ROS 2 backend
    only publishes vehicle state.

    Args:
        body_mass, body_inertia: override t650_params.BODY_MASS / INERTIA_TENSOR (a 3x3
            tensor or a length-3 diagonal; both accepted). Both default
            to the T650's own values; pass them only for a deliberate variant.

    Returns:
        str: the vehicle stage prefix (drone root path).
    """
    if not os.path.isfile(T650_USD):
        raise FileNotFoundError(f"Shared bare-airframe USD not found: {T650_USD}")

    print(f"[T650] Using shared asset: {T650_USD}", flush=True)
    print(
        f"[T650] MN4010 + 15x5\" (docs/propeller_testing/MN_4010_15x5_report.pdf) | "
        f"omega {t650_params.ZERO_POSITION_ARMED:.2f}..{t650_params.MAX_ROTOR_VEL:.2f} rad/s | "
        f"k={t650_params.ROTOR_CONSTANT:.6e} | c={t650_params.ROLLING_MOMENT_COEFFICIENT:.6e} | "
        f"lambda={t650_params.ROTOR_LAMBDA} 1/s",
        flush=True,
    )

    quat_xyzw = Rotation.from_euler("XYZ", spawn_euler, degrees=True).as_quat()

    # ----- PX4 MAVLink backend (primary motor command source = backend[0]) -----
    # input_offset / input_scaling / zero_position_armed map PX4's normalized 0-1 motor
    # commands onto the bench-measured rotor speed range exactly, replacing the stock
    # Iris-tuned defaults (input_scaling=1000, zero_position_armed=100).
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
        "input_scaling": [t650_params.MAX_ROTOR_VEL - t650_params.ZERO_POSITION_ARMED] * 4,
        "zero_position_armed": [t650_params.ZERO_POSITION_ARMED] * 4,
    })

    # ----- ROS 2 backend (state publishing only; PX4 has motor authority) -----
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
            "sub_control": False,   # PX4 is primary -> don't accept ROS 2 rotor commands
        },
    )

    # Stock MultirotorConfig: default sensors (Baro/IMU/Mag/GPS). The thrust curve is
    # overridden with the MN4010 calibration plus its bench-measured spin-up lag. Rotor
    # order is PX4 Quad-X: rotor0 front-right and rotor1 rear-left are counter-clockwise,
    # rotor2 front-left and rotor3 rear-right are clockwise; Pegasus writes that as
    # [-1, -1, 1, 1]. backend[0] = PX4 (primary).
    config = MultirotorConfig()
    config.backends = [PX4MavlinkBackend(mavlink_config), ros2_backend]
    config.thrust_curve = LaggedQuadraticThrustCurve(config={
        "rotor_constant": [t650_params.ROTOR_CONSTANT] * 4,
        "rolling_moment_coefficient": [t650_params.ROLLING_MOMENT_COEFFICIENT] * 4,
        "min_rotor_velocity": [t650_params.MIN_ROTOR_VEL] * 4,
        "max_rotor_velocity": [t650_params.MAX_ROTOR_VEL] * 4,
        "rotor_lambda": [t650_params.ROTOR_LAMBDA] * 4,
        "rot_dir": [int(d) for d in t650_params.ROT_DIR],
    })

    drone_prim_path = f"/World/quadrotor_{vehicle_id}"

    vehicle = Multirotor(
        stage_prefix=drone_prim_path,
        usd_file=T650_USD,
        vehicle_id=vehicle_id,
        init_pos=spawn_pos,
        init_orientation=quat_xyzw,
        config=config,
    )
    apply_t650_mass_properties(vehicle, body_mass=body_mass, body_inertia=body_inertia)

    return vehicle._stage_prefix
