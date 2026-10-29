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


# ============================================================================
#  AN AIRFRAME DESCRIBED BY ITS FILE (2026-10-27, the build plan's 1:7 aircraft)
# ============================================================================
#  The CAD airframe's plant is the CAD (its links' masses) and its airframe_id.cfg was DERIVED from
#  that plant. An airframe that exists only on paper -- the build plan's -- has no CAD, so the
#  direction is reversed: the airframe file is the description and the plant is BUILT from it. The
#  body link is solved so that the ASSEMBLY (body + the small nacelle / prop / surface links) has
#  exactly the file's mass, CG and inertia; the rotors, their per-pair propellers, the tilt and
#  surface servos and the aero coefficients are the file's. One file, one aircraft, read by the
#  plant and by every controller. A file carries this only if it has a [layout] section.

def load_airframe(path=None):
    """The controller's airframe file through the controller's own loader (numpy only), by the
    sim repo's controller_path.py. `path` None = FSC_AIRFRAME_ID, else the package's default."""
    import importlib.util
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
    spec = importlib.util.spec_from_file_location("controller_path", os.path.join(root, "controller_path.py"))
    cp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cp)
    return cp.load_airframe(path)


def aero_config_from_airframe(af):
    """VTOLAerodynamicsConfig from the file's [aero] -- the one mapping, in vtol_aerodynamics.py."""
    from pegasus.simulator.logic.dynamics.vtol_aerodynamics import aero_config_from_airframe as _a
    return _a(af)


def airframe_vtol_config(path=None, cad_meshes=None):
    """A VTOLConfig built from an airframe file with a [layout] section.

    Frame: body FLU, the body-link origin is the file's reference (the hubs' x are measured from it,
    the CG is cg_x forward of it); the hubs' z are from the CG, so the CG sits at z = 0 here. Channels
    are the FSC quad's (default_quad_tiltrotor_channel_map), so PX4 airframe 4101 drives it unchanged.

    `cad_meshes` (None = when the parts are on disk): dress it in the FSC CAD quad's meshes
    (fsc_cad_vtol_config: body, front arms, the four props) instead of boxes -- the user's ask,
    2026-10-27. VISUAL ONLY: the mesh links are the same massless-collider links, and every mass,
    hub, propeller and coefficient is still the file's. Each rotor's CAD arm + prop is placed on the
    FILE's hub with the CAD's own shaft offset; the CAD body is shifted so ITS assembled CG lands on
    this airframe's CG. Where the two layouts differ (the plan's rotors sit 0.43-0.48 m out, the CAD's
    booms 0.284 m) the props render where the thrust really acts, i.e. off the CAD booms.
    Returns (config, airframe)."""
    from dataclasses import replace
    import numpy as np
    from pegasus.simulator.logic.vehicles.vtol import (RotorConfig, SurfaceConfig, _link_positions,
                                                       _parallel_axis, assembled_mass_properties)
    af = load_airframe(path)
    if not getattr(af, "has_layout", False):
        raise ValueError(f"{af.path} has no [layout] section: it describes the controllers' model of an "
                         f"airframe whose plant is elsewhere (the CAD), not a buildable airframe")
    r, t, L = af.rotors, af.tilt, af.layout
    names = ("front_L", "front_R", "rear_L", "rear_R")
    thr = ("motor_front_L", "motor_front_R", "motor_rear_L", "motor_rear_R")
    tch = ("tilt_left", "tilt_right", None, None)
    per = {k: r.per_rotor(k) for k in ("max_thrust", "cq", "prop_d", "prop_j0", "omega_max", "tau_spin")}
    rotors = []
    for i in range(4):
        tilts = bool(r.tiltable[i])
        rotors.append(RotorConfig(
            name=names[i], position=tuple(float(v) for v in r.hubs[i]), spin_dir=int(r.spin[i]),
            spin_offset=tuple(float(v) for v in L.spin_offset) if tilts else (0.0, 0.0, 0.0),
            max_thrust=float(per["max_thrust"][i]), prop=None,
            prop_diameter_m=float(per["prop_d"][i]), prop_zero_thrust_J=float(per["prop_j0"][i]),
            omega_max=float(per["omega_max"][i]), tau_spin=float(per["tau_spin"][i]),
            moment_ratio=float(per["cq"][i]),
            tiltable=tilts, tilt_axis="Y", tilt_min_deg=float(t.min_deg) if tilts else 0.0,
            tilt_max_deg=float(t.max_deg) if tilts else 90.0, tilt_rate_deg_s=float(t.rate_deg_s),
            throttle_channel=thr[i], tilt_channel=tch[i]))
    dmax = float(af.surfaces.max_deflection_deg)
    surfaces = [
        SurfaceConfig(name="aileron_L", position=tuple(L.aileron_l), hinge_axis="Y", max_deflection_deg=dmax,
                      deflect_channel="aileron", deflect_scale=1.0, size=tuple(L.surface_aileron_size)),
        SurfaceConfig(name="aileron_R", position=tuple(L.aileron_r), hinge_axis="Y", max_deflection_deg=dmax,
                      deflect_channel="aileron", deflect_scale=-1.0, size=tuple(L.surface_aileron_size)),
        SurfaceConfig(name="elevator", position=tuple(L.elevator), hinge_axis="Y", max_deflection_deg=dmax,
                      deflect_channel="elevator", size=tuple(L.surface_elevator_size)),
        SurfaceConfig(name="rudder", position=tuple(L.rudder), hinge_axis="Z", max_deflection_deg=dmax,
                      deflect_channel="rudder", size=tuple(L.surface_rudder_size)),
    ]
    body_kw = dict(body_size=tuple(L.body_size))
    cad = None
    if cad_meshes is not False:
        try:
            cad = fsc_cad_vtol_config()
            if not (cad.body_mesh and os.path.isfile(cad.body_mesh)):
                cad = None
        except Exception:                              # noqa: BLE001 - no CAD on this machine: boxes
            cad = None
        if cad_meshes and cad is None:
            raise FileNotFoundError(f"cad_meshes=True but the CAD parts are not at {CAD_USD_DIR}")
    if cad is not None:
        _, cg_cad, _ = assembled_mass_properties(cad)
        by_name = {rc.name: rc for rc in cad.rotors}
        rotors = [replace(rc, mesh=by_name[rc.name].mesh, nacelle_mesh=by_name[rc.name].nacelle_mesh,
                          mesh_scale=by_name[rc.name].mesh_scale, mesh_offset=by_name[rc.name].mesh_offset,
                          mesh_rpy_deg=by_name[rc.name].mesh_rpy_deg,
                          spin_offset=by_name[rc.name].spin_offset if rc.tiltable else (0.0, 0.0, 0.0))
                  for rc in rotors]
        body_kw = dict(body_size=tuple(L.body_size), body_mesh=cad.body_mesh, body_mesh_scale=cad.body_mesh_scale,
                       body_mesh_rpy_deg=cad.body_mesh_rpy_deg,
                       body_mesh_offset=tuple(float(-v) for v in cg_cad),          # the CAD's CG onto ours
                       body_collision_size=tuple(L.collision_size))                # a mesh body's floor
    cfg = VTOLConfig(
        stage_prefix=str(L.name),
        thrust_model="momentum",
        rotor_omega_max=float(r.omega_max), rotor_tau_spin=float(r.tau_spin), rotor_moment_ratio=float(r.cq),
        tilt_tau=float(t.tau), surface_tau=float(af.surfaces.tau), surface_rate_deg_s=float(af.surfaces.rate_deg_s),
        rotors=rotors, surfaces=surfaces, aerodynamics=aero_config_from_airframe(af),
        link_mass=float(L.link_mass), **body_kw,
        mass=1.0, inertia=(1.0, 1.0, 1.0), body_com_offset=(0.0, 0.0, 0.0),
    )
    # ★ THE BODY LINK IS SOLVED, so that the assembly IS the file: total mass, CG, diagonal inertia.
    M = float(af.mass.airframe_kg)
    cg = np.array([float(af.mass.cg_x), 0.0, 0.0])
    I_tgt = np.array([float(af.mass.ixx), float(af.mass.iyy), float(af.mass.izz)])
    pts = _link_positions(cfg)
    m_l = float(cfg.link_mass); i_l = np.asarray(cfg.link_inertia, dtype=float)
    m_b = M - m_l * len(pts)
    c_b = (M * cg - m_l * sum(pts)) / m_b
    i_b = I_tgt - _parallel_axis(m_b, c_b - cg) - sum(i_l + _parallel_axis(m_l, p - cg) for p in pts)
    if m_b <= 0.0 or np.any(i_b <= 0.0):
        raise ValueError(f"{af.path}: the small links alone exceed the airframe's mass or inertia "
                         f"(body m {m_b:.3f} kg, I {i_b}) -- lower [layout] link_mass")
    cfg = replace(cfg, mass=float(m_b), body_com_offset=tuple(float(v) for v in c_b),
                  inertia=tuple(float(v) for v in i_b))
    m_a, cg_a, i_a = assembled_mass_properties(cfg)
    assert abs(m_a - M) < 1e-9 and np.allclose(cg_a, cg, atol=1e-12) and np.allclose(i_a, I_tgt, atol=1e-12), \
        (m_a, cg_a, i_a)
    return cfg, af


def airframe_vtol_slosh_config(path=None, tank: TankConfig | None = None, tank_dx: float = 0.0,
                               tank_dz: float = 0.0):
    """The file's airframe as a VTOLSloshConfig carrying `tank`, mounted (tank_dx, 0, tank_dz) from the
    assembled CG -- the same VTOL_TANK_DX / VTOL_TANK_DZ the controllers fold (so3.CG_SHIFT)."""
    from dataclasses import replace as _replace
    from pegasus.simulator.logic.vehicles.vtol import assembled_mass_properties
    base, af = airframe_vtol_config(path)
    if tank is not None and (tank_dx != 0.0 or tank_dz != 0.0):
        _, cg, _ = assembled_mass_properties(base)
        tank = _replace(tank, position=(float(cg[0]) + float(tank_dx), float(cg[1]), float(cg[2]) + float(tank_dz)))
    return VTOLSloshConfig(**{f.name: getattr(base, f.name) for f in fields(base)}, tank=tank), af


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


def px4ca_tilt_map(channel_map, config, spec=None):
    """★ PX4's OWN tilt convention (2026-10-28, the C++ autopilot path). When PX4's control allocation drives
    the tilt servos (airframe 4101 under a PX4 flight mode, NOT direct_actuator), its output is -1 = its
    CA_SV_TLn_MINA (upright: its allocation assumes exactly that when it rebuilds) .. +1 = MAXA, while this
    plant flies a tilt command as cmd x tilt_max_deg (the direct-actuator convention: the identity map).
    Since the 2026-09-07 re-anchor that put PX4's upright (-1) at -tilt_max, clipped to the aft stop.
    `VTOL_TILT_PX4CA="MINA,MAXA"` (or `spec`) re-maps each tilt channel onto the plant's convention -- the
    sim's servo calibration, as a real servo's PWM endpoints are: angle = MINA + (c + 1)/2 (MAXA - MINA),
    so cmd = c (MAXA - MINA)/(2 tilt_max) + (MAXA + MINA)/(2 tilt_max). Unset = the map unchanged (every
    direct-actuator flight). scripts/fsc_autopilot_config.py writes the value with the PX4 parameters."""
    spec = (os.environ.get("VTOL_TILT_PX4CA", "") if spec is None else spec).strip()
    if not spec:
        return channel_map
    mina, maxa = (float(v) for v in spec.split(","))
    out = dict(channel_map)
    done = []
    for rc in config.rotors:
        ch = getattr(rc, "tilt_channel", None)
        if not getattr(rc, "tiltable", False) or ch is None or ch not in out or ch in done:
            continue
        tmax = float(rc.tilt_max_deg)
        idx = out[ch][0]
        # 4th field: an UNWRITTEN (disarmed) output -> PX4's hover tilt, MINA (vtol_mavlink_backend._map_controls)
        out[ch] = (idx, (maxa - mina) / (2.0 * tmax), (maxa + mina) / (2.0 * tmax), mina / tmax)
        done.append(ch)
        print(f"[VTOL] tilt map: PX4 CA convention on {ch} (controls[{idx}]): -1 -> {mina:g} deg, +1 -> {maxa:g} deg, "
              f"disarmed -> {mina:g} deg (tilt_max {tmax:g}: scale {out[ch][1]:.4f} offset {out[ch][2]:.4f})", flush=True)
    if not done:
        raise ValueError(f"VTOL_TILT_PX4CA={spec}: this airframe has no tilt channel in the channel map")
    return out


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
    channel_map = px4ca_tilt_map(channel_map, config)

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
