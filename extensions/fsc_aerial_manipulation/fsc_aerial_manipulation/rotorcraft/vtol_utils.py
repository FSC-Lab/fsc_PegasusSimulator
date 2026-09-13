"""
Helpers for spawning VTOL vehicles.

  * spawn_vtol              -> standalone (scripted / custom backend supplied by caller)
  * spawn_vtol_with_mavlink -> PX4 HIL: wires a PX4VTOLMavlinkBackend so PX4's
                               actuator outputs drive the VTOL (Step 4).
"""

import os
from dataclasses import fields

from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.vehicles.vtol import VTOL, VTOLConfig
from pegasus.simulator.logic.vehicles.vtol_slosh import VTOLSlosh, VTOLSloshConfig
from pegasus.simulator.logic.dynamics.slosh import TankConfig


# Default location of the converted CAD part USDs (see application/vtol/
# split_cad_stl.py + convert_meshes_to_usd.py). Override with VTOL_CAD_USD_DIR.
CAD_USD_DIR = os.environ.get("VTOL_CAD_USD_DIR", os.path.expanduser("~/vtol_cad_usd/parts2_usd"))


def fsc_cad_vtol_config(usd_dir: str | None = None):
    """The FSC dual-tiltrotor-quadplane airframe built from the imported CAD parts
    (2 front tilting rotors + 2 rear static rotors), with the tuned hinge/geometry.

    One definition used by both the HIL scene and the rotor-physics demo. `usd_dir`
    (or $VTOL_CAD_USD_DIR) points at the converted part USDs; a missing part just
    falls back to a procedural box for that link.
    """
    from pegasus.simulator.logic.vehicles.vtol import quad_tiltrotor_config
    d = usd_dir or CAD_USD_DIR
    j = lambda n: os.path.join(d, n)                                        # noqa: E731
    return quad_tiltrotor_config(
        body_mesh=j("body.usd"),
        front_left_mesh=j("front_left_prop.usd"),
        front_right_mesh=j("front_right_prop.usd"),
        rear_left_mesh=j("rear_left_prop.usd"),
        rear_right_mesh=j("rear_right_prop.usd"),
        front_left_arm_mesh=j("front_left_arm.usd"),
        front_right_arm_mesh=j("front_right_arm.usd"),
        mesh_scale=(0.001, 0.001, 0.001),          # STL is in mm
        mesh_rpy_deg=(90.0, 0.0, 90.0),            # CAD +Z (nose) -> world +X
    )


def fsc_cad_vtol_slosh_config(usd_dir: str | None = None, tank: TankConfig | None = None,
                              tank_dx: float = 0.0):
    """The SAME FSC airframe, as a `VTOLSloshConfig` so it can carry a tank.

    Built by copying `fsc_cad_vtol_config()` FIELD BY FIELD rather than by
    repeating its numbers: the two airframes are one airframe, and a geometry
    change made in one place must not need a second edit here.  `fields()`
    carries any field added to `VTOLConfig` in future along with it.

    `tank=None` (the default) gives a config describing exactly the baseline
    aircraft -- useful for proving the copy changed nothing.  Phase 1 of the
    tank plan REFUSES to construct a vehicle with a tank set, because the slosh
    physics is not implemented yet.
    """
    base = fsc_cad_vtol_config(usd_dir)
    if tank is not None and tank_dx != 0.0:
        # ★ THE L1 TEST BED: mount the tank `tank_dx` metres FORWARD of the assembled CG
        # -- a payload the controller is NOT told about (its CG_X stays nominal). The
        # plant's CG moves by m_water/M * dx and the trim the controller flies is wrong
        # by that lever arm times the vertical thrust: 38.8 N x 3 cm = 1.2 N.m of
        # standing pitch moment in hover. That is what the adaptive augmentation exists
        # to absorb, and this is how it is proven rather than argued.
        from dataclasses import replace as _replace
        from pegasus.simulator.logic.vehicles.vtol_slosh import assembled_mass_properties
        _, cg, _ = assembled_mass_properties(base)
        tank = _replace(tank, position=(float(cg[0]) + float(tank_dx), float(cg[1]), float(cg[2])))
    return VTOLSloshConfig(
        **{f.name: getattr(base, f.name) for f in fields(base)},
        tank=tank,
    )


def _vehicle_class_for(config) -> type:
    """Which vehicle class a config asks for.

    ONE selector: the config type decides, so a caller cannot pick a slosh
    config and a baseline class (or the reverse) and get a silently wrong
    aircraft.  `vehicle_cls=` on the spawners overrides it explicitly."""
    return VTOLSlosh if isinstance(config, VTOLSloshConfig) else VTOL


def _required_channels(config: VTOLConfig) -> set:
    """The set of command channel names an airframe config actually consumes."""
    channels = set()
    for rotor in config.rotors:
        if rotor.throttle_channel:
            channels.add(rotor.throttle_channel)
        if rotor.tiltable and rotor.tilt_channel:
            channels.add(rotor.tilt_channel)
    for surface in config.surfaces:
        if surface.deflect_channel:
            channels.add(surface.deflect_channel)
    return channels


def spawn_vtol(
    vehicle_id=0,
    spawn_pos=(0.0, 0.0, 0.08),
    spawn_quat=(0.0, 0.0, 0.0, 1.0),
    config=None,
    vehicle_cls=None,
):
    """Spawn a VTOL vehicle and return its stage path.

    The caller is responsible for putting any control backend onto `config`
    (e.g. the scripted / hover backends used in 01_ and 02_)."""
    if config is None:
        config = VTOLConfig()

    vehicle = (vehicle_cls or _vehicle_class_for(config))(
        vehicle_id=vehicle_id,
        init_pos=spawn_pos,
        init_orientation=spawn_quat,
        config=config,
    )

    return vehicle.prim_path


def spawn_vtol_with_mavlink(
    px4_path,
    px4_vehicle_model,
    vehicle_id=0,
    spawn_pos=(0.0, 0.0, 0.2),
    spawn_euler=(0.0, 0.0, 0.0),
    connection_ip="127.0.0.1",
    connection_baseport=4560,
    channel_map=None,
    config=None,
    publish_state_ros2=False,
    ros2_namespace="uav_",
    vehicle_cls=None,
):
    """Spawn a VTOL wired to PX4 over the MAVLink HIL link (no PX4 autolaunch).

    PX4 SITL is launched separately (see scripts/) and must run a VTOL airframe.
    PX4's computed actuator outputs arrive as HIL_ACTUATOR_CONTROLS and are
    routed to the VTOL's channels by PX4VTOLMavlinkBackend via `channel_map`
    (defaults to the tri-tiltrotor map — VERIFY with PEGASUS_HIL_DEBUG=1).

    Args:
        px4_path: path to the PX4-Autopilot SITL tree (e.g. ~/PX4-Autopilot).
        px4_vehicle_model: PX4 SITL airframe/model name (the VTOL airframe).
        vehicle_id: drone id; added to connection_baseport (Pegasus listens on 4560+id).
        spawn_pos: initial world position (ENU).
        spawn_euler: initial orientation as XYZ Euler degrees.
        channel_map: optional override for the actuator index mapping. When None,
            a built-in map whose channels match `config` is auto-selected (dual
            tiltrotor or tri tiltrotor); a config with no matching map raises so
            the mismatch is never silent.
        config: optional VTOLConfig (forces+gravity are forced ON here).
        publish_state_ros2: if True, ALSO attach a state-only ROS2Backend that
            publishes /<ns><id>/state/{pose,twist,twist_inertial}. This is what
            feeds the mocap emulator (isaacsim_optitrack_ros2_emulator) and, in
            turn, the controller. Off by default so the plain HIL path is
            untouched. The PX4 backend stays backend[0] (VTOL reads commands from
            it); the ROS2Backend is a pure publisher.
        ros2_namespace: topic namespace prefix for the state backend. Default
            "uav_" -> /uav_0/... to match the emulator + PX4 UXRCE_DDS_NS.
    """
    # Lazy import so merely importing this module doesn't pull MAVLink/PX4 deps.
    from pegasus.simulator.logic.backends.vtol_mavlink_backend import (
        PX4VTOLMavlinkBackend,
        PX4VTOLMavlinkBackendConfig,
        default_tiltrotor_channel_map,
        default_dual_tiltrotor_channel_map,
        default_quad_tiltrotor_channel_map,
    )

    if config is None:
        config = VTOLConfig()

    # Pick a channel_map that actually matches this airframe's channels, so a
    # config + omitted map can't silently fall back to a wrong map (which would
    # zero every command). Explicit channel_map always wins.
    if channel_map is None:
        required = _required_channels(config)
        for candidate in (default_quad_tiltrotor_channel_map(),
                          default_dual_tiltrotor_channel_map(),
                          default_tiltrotor_channel_map()):
            if required.issubset(candidate.keys()):
                channel_map = candidate
                break
        if channel_map is None:
            raise ValueError(
                f"No built-in VTOL channel_map covers this airframe's channels "
                f"{sorted(required)}. Pass channel_map= explicitly."
            )

    quat_xyzw = Rotation.from_euler("XYZ", spawn_euler, degrees=True).as_quat()

    mavlink_config = PX4VTOLMavlinkBackendConfig({
        "vehicle_id": vehicle_id,
        "connection_type": "tcpin",
        "connection_ip": connection_ip,
        "connection_baseport": connection_baseport,
        "px4_autolaunch": False,                 # PX4 SITL is started by the tmux script
        "px4_dir": px4_path,
        "px4_vehicle_model": px4_vehicle_model,
        "enable_lockstep": False,                # see docs/direct_actuator_offboard.md
        "channel_map": channel_map,
    })

    # PX4 HIL drives a real flight: forces + gravity ON.
    config.enable_forces = True
    config.disable_gravity = False
    # backend[0] = PX4 (the VTOL reads its command from here); a ROS2 state
    # publisher, if requested, is appended as a pure-publisher secondary backend.
    backends = [PX4VTOLMavlinkBackend(mavlink_config)]

    if publish_state_ros2:
        # Lazy import: only pull rclpy/ROS2Backend when state publishing is on.
        from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
        backends.append(ROS2Backend(
            vehicle_id=vehicle_id,
            num_rotors=0,                       # VTOL command comes from PX4, not ROS
            config={
                "namespace": ros2_namespace,    # "uav_" + id -> /uav_0/...
                "pub_state": True,              # /state/{pose,twist,twist_inertial}
                "pub_sensors": False,           # emulator only needs pose + twists
                "pub_graphical_sensors": False,
                "sub_control": False,           # do not subscribe to rotor refs
            },
        ))

    config.backends = backends

    # Baseline config -> VTOL, slosh config -> VTOLSlosh (see _vehicle_class_for).
    vehicle = (vehicle_cls or _vehicle_class_for(config))(
        vehicle_id=vehicle_id,
        init_pos=spawn_pos,
        init_orientation=quat_xyzw,
        config=config,
    )

    return vehicle.prim_path
