#!/usr/bin/env python
"""
| File: x650_rotorcraft_utils.py
| Author: Ben Natra (send2ben123@gmail.com)
| Description: Utility functions for defining rotorcraft in Isaac Sim
"""

from scipy.spatial.transform import Rotation
from pxr import Usd, UsdPhysics

from pegasus.simulator.params import ROBOTS
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend
# Use the FSC aerial-manipulator vehicle classes (MultirotorMod → VehicleMod), which
# live in fsc_aerial_manipulation.robotic_arm and *subclass* the stock Multirotor/Vehicle.
# They add the body_path / rotor_paths / rotor_joint_names / usd_prim_path constructor
# kwargs that spawn_rotorcraft_with_mavlink passes below. The stock multirotor.py/vehicle.py
# base classes are left untouched (they lack these kwargs and would raise TypeError).
from fsc_aerial_manipulation.robotic_arm.x650_multirotor import MultirotorMod as Multirotor, MultirotorConfig
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve


PX4_ROTOR_INPUT_SCALING = 1000.0
PX4_ROTOR_ARMED_IDLE = 100.0


def spawn_rotorcraft_with_mavlink(
    px4_path,
    px4_default_airframe,
    vehicle_id=0,
    spawn_pos=(0.0, 0.0, 0.0),
    spawn_euler=(0.0, 0.0, 0.0),
    connection_ip="127.0.0.1",
    connection_baseport=4560,
    enable_lockstep=True,
    vehicle_type="Iris",
    usd_file=None,
    usd_prim_path=None,
    articulation_path="",
    body_path="/body",
    rotor_paths=None,
    rotor_joint_names=None,
    rotor_directions=None,
    thrust_config=None,
    enable_propeller_visual=True,
    px4_primary=False,
    include_px4=True,
    return_backend=False,
):
    """
    Spawn rotorcraft with MAVLink and ROS2 backends.

    px4_primary=True  → PX4/QGC drives motors (backend[0]=PX4); ROS2 publishes state only.
                         Use for X1_whole_body with QGroundControl.
    px4_primary=False → External geometric controller drives motors (backend[0]=ROS2 with
                         sub_control=True); PX4 backend receives state for logging only.
                         Use for bird_safe_film_sim with the WSL2 geometric controller.

    Args:
        px4_path: path to PX4-Autopilot directory
        px4_default_airframe: PX4 SITL airframe model (e.g. "gazebo-iris")
        vehicle_id: numeric vehicle ID
        spawn_pos: initial world position in ENU [m]
        px4_primary: if True PX4 is backend[0] (QGC control); if False ROS2 is backend[0]
    """
    quat_xyzw = Rotation.from_euler("XYZ", spawn_euler, degrees=True).as_quat()

    mavlink_config = PX4MavlinkBackendConfig({
        "vehicle_id": vehicle_id,
        "connection_type": "tcpin",
        "connection_ip": connection_ip,
        "connection_baseport": connection_baseport,
        "enable_lockstep": enable_lockstep,
        "px4_autolaunch": False,
        "px4_dir": px4_path,
        "px4_vehicle_model": px4_default_airframe,
        # Keep this explicit: the Offboard direct-actuator controller in
        # robotic_arm/controller.py inverts this exact normalized-control map.
        "input_offset": [0.0] * 4,
        "input_scaling": [PX4_ROTOR_INPUT_SCALING] * 4,
        "zero_position_armed": [PX4_ROTOR_ARMED_IDLE] * 4,
    })

    ros2_backend = ROS2Backend(
        vehicle_id=vehicle_id,
        config={
            "namespace": f"uav_",
            "pub_sensors": False,
            "pub_graphical_sensors": False,
            "pub_state": True,
            "pub_twist": True,
            "pub_accel": True,
            "pub_twist_inertial": True,
            "pub_tf": True,
            # sub_control=True lets the external geometric controller publish rotor
            # velocity commands.  Disabled when PX4 is primary (QGC mode) so that
            # stale or absent ROS2 commands cannot interfere with PX4 authority.
            "sub_control": not px4_primary,
        },
    )

    config_multirotor = MultirotorConfig()
    # backend[0].input_reference() is the sole motor-command source in multirotor.py.
    if not include_px4:
        # In-process / standalone: no PX4 SITL, so don't spawn the MAVLink
        # backend (it would block forever waiting for a heartbeat). ROS2 backend
        # is backend[0]; its input_ref can be driven directly in-process.
        config_multirotor.backends = [ros2_backend]
    elif px4_primary:
        config_multirotor.backends = [PX4MavlinkBackend(mavlink_config), ros2_backend]
    else:
        config_multirotor.backends = [ros2_backend, PX4MavlinkBackend(mavlink_config)]

    # Build thrust curve config, merging any caller-supplied overrides with rotor directions.
    tc = dict(thrust_config) if thrust_config else {}
    if rotor_directions is not None:
        tc["rot_dir"] = rotor_directions
    if tc:
        config_multirotor.thrust_curve = QuadraticThrustCurve(config=tc)

    drone_prim_path = f"/World/quadrotor_{vehicle_id}{articulation_path}"
    if usd_file is None:
        usd_file = ROBOTS[vehicle_type]

    Multirotor(
        stage_prefix=drone_prim_path,
        usd_file=usd_file,
        vehicle_id=vehicle_id,
        init_pos=spawn_pos,
        init_orientation=quat_xyzw,
        body_path=body_path,
        rotor_paths=rotor_paths,
        rotor_joint_names=rotor_joint_names,
        config=config_multirotor,
        usd_prim_path=usd_prim_path,
    )

    # return_backend=True hands back the ROS2 backend so a caller can drive its
    # input_ref directly (e.g. write rotor speeds from a subscriber in the demo
    # script, keeping the Pegasus ros2_backend stock — no extension edit).
    if return_backend:
        return drone_prim_path, ros2_backend
    return drone_prim_path


def lock_manipulator_joints(stage, root_path: str, stiffness: float = 1e6, damping: float = 1e4):
    """Apply a stiff position drive to every non-propeller joint under root_path.

    Revolute joints get an 'angular' DriveAPI; prismatic joints get a 'linear' DriveAPI.
    Propeller joints (named prop_joint*) are left untouched.

    Args:
        stage: The live USD stage.
        root_path: Absolute path to the vehicle root prim (e.g. '/World/quadrotor_0/contact_drone').
        stiffness: Position-control stiffness in N·m/rad (revolute) or N/m (prismatic).
        damping: Velocity damping in N·m·s/rad or N·s/m.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"[JOINTS] Root prim not found: {root_path}", flush=True)
        return

    locked = 0
    for prim in Usd.PrimRange(root):
        prim_name = prim.GetName()
        # Only lock arm joints: manip_joint1/2/3 (revolute) and prism1 (prismatic).
        # Propeller joints in contact_drone.usda are named joint0..3 — no manip_/prism_ prefix.
        if not (prim_name.startswith("manip_joint") or prim_name.startswith("prism")):
            continue

        type_name = prim.GetTypeName()
        if type_name == "PhysicsRevoluteJoint":
            token = "angular"
        elif type_name == "PhysicsPrismaticJoint":
            token = "linear"
        else:
            continue

        drive_api = UsdPhysics.DriveAPI.Apply(prim, token)
        drive_api.CreateTypeAttr().Set("force")
        drive_api.CreateStiffnessAttr().Set(stiffness)
        drive_api.CreateDampingAttr().Set(damping)
        drive_api.CreateTargetPositionAttr().Set(0.0)
        drive_api.CreateMaxForceAttr().Set(1e9)

        locked += 1
        print(
            f"[JOINTS] Locked {prim.GetPath()} ({token} drive, "
            f"stiffness={stiffness:.0e}, damping={damping:.0e})",
            flush=True,
        )

    print(f"[JOINTS] {locked} manipulator joint(s) locked under {root_path}", flush=True)


def print_mass_inertia_properties(stage, drone_path: str):
    """Query and print mass/inertia for every rigid body under drone_path.

    Call this AFTER world.reset() so PhysX has finalised computed properties.
    The output gives you the exact values to paste into X650Params in the
    geometric controller node.

    Total mass  → X650Params.mass
    Body diagonal inertia → X650Params.J  (first approximation; refine by
                            summing arm-link contributions if needed)
    """
    root = stage.GetPrimAtPath(drone_path)
    if not root or not root.IsValid():
        print(f"[MASS] Root prim not found: {drone_path}", flush=True)
        return

    total_mass = 0.0
    body_inertia = None

    print(f"\n[MASS] === Rigid-body mass properties under {drone_path} ===", flush=True)

    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue

        mass_api = UsdPhysics.MassAPI(prim)
        mass_attr = mass_api.GetMassAttr()
        inertia_attr = mass_api.GetDiagonalInertiaAttr()

        mass = mass_attr.Get() if (mass_attr and mass_attr.HasValue()) else None
        inertia = inertia_attr.Get() if (inertia_attr and inertia_attr.HasValue()) else None

        if mass is not None:
            total_mass += float(mass)

        is_body = prim.GetName() == "body"
        if is_body and inertia is not None:
            body_inertia = inertia

        tag = "  ← BODY" if is_body else ""
        print(
            f"  {prim.GetPath()}{tag}\n"
            f"      mass={mass} kg    diagonalInertia={inertia} kg·m²",
            flush=True,
        )

    print(f"\n[MASS] Total mass: {total_mass:.4f} kg", flush=True)
    print(f"[MASS] -----------------------------------------------", flush=True)
    print(f"[MASS] Paste into geometric_controller_node.py X650Params:", flush=True)
    print(f"[MASS]   mass: float = {total_mass:.4f}", flush=True)
    if body_inertia is not None:
        ixx, iyy, izz = float(body_inertia[0]), float(body_inertia[1]), float(body_inertia[2])
        print(f"[MASS]   J: np.ndarray = np.diag([{ixx:.6f}, {iyy:.6f}, {izz:.6f}])", flush=True)
    else:
        print(f"[MASS]   J: could not read body inertia — check USD MassAPI on /body prim", flush=True)
    print(f"[MASS] -----------------------------------------------\n", flush=True)


def print_rotor_positions(stage, drone_path: str, rotor_relative_paths: list):
    """Read each rotor prim's local translation from the USD stage and print
    the exact rotor_positions array to paste into X650Params.

    Call this AFTER world.reset().  The positions are in the drone-root frame,
    which equals the body frame when the body prim sits at the root origin.

    Also prints the spin-direction convention assumed by Pegasus
    QuadraticThrustCurve (rot_dir default) so you can cross-check the
    controller's rot_dir array.
    """
    from pxr import UsdGeom  # type: ignore[import]

    print(f"\n[ROTORS] === Rotor local positions under {drone_path} ===", flush=True)

    positions = []
    for rel_path in rotor_relative_paths:
        full_path = drone_path + rel_path
        prim = stage.GetPrimAtPath(full_path)
        if not prim or not prim.IsValid():
            print(f"[ROTORS]   {full_path}: NOT FOUND", flush=True)
            positions.append((0.0, 0.0, 0.0))
            continue

        xform = UsdGeom.Xformable(prim)
        t = xform.GetLocalTransformation().ExtractTranslation()
        x, y, z = float(t[0]), float(t[1]), float(t[2])
        positions.append((x, y, z))
        print(f"[ROTORS]   {rel_path:12s}  x={x:+.4f}  y={y:+.4f}  z={z:+.4f}", flush=True)

    print(f"\n[ROTORS] Paste into geometric_controller_node.py X650Params:", flush=True)
    print(f"[ROTORS]   rotor_positions: np.ndarray = np.array([", flush=True)
    for i, (rel, (x, y, z)) in enumerate(zip(rotor_relative_paths, positions)):
        print(f"[ROTORS]       [{x:+.6f}, {y:+.6f}, {z:+.6f}],   # channel {i} → {rel}", flush=True)
    print(f"[ROTORS]   ])", flush=True)

    print(f"\n[ROTORS] Spin-direction check:", flush=True)
    print(f"[ROTORS]   For each rotor, positive joint velocity = CCW from above (+1).", flush=True)
    print(f"[ROTORS]   Verify rot_dir in X650Params matches the USD joint drive directions.", flush=True)
    print(f"[ROTORS]   Diagonal pairs must share the same sign to balance yaw.", flush=True)
    print(f"[ROTORS] -----------------------------------------------\n", flush=True)


def print_arm_coupling_params(stage, drone_path: str):
    """Read arm link masses and joint positions from the USD stage and print
    exact values to paste into ArmCouplingParams in geometric_controller_node.py.

    Call this AFTER world.reset() so PhysX has finalised computed properties.

    The function follows physics:body1 from each manip_joint*/prism* prim to find
    the child link prim, then reads its MassAPI and local transform.  Output
    includes: per-link mass, local origin (to estimate mount_pos and link lengths),
    and a ready-to-paste ArmCouplingParams block.
    """
    from pxr import Usd, UsdPhysics, UsdGeom  # type: ignore[import]

    root = stage.GetPrimAtPath(drone_path)
    if not root or not root.IsValid():
        print(f"[ARM] Root prim not found: {drone_path}", flush=True)
        return

    print(f"\n[ARM] === Arm coupling parameters under {drone_path} ===", flush=True)

    # Collect (joint_name, link_name, mass_kg, local_xyz) for every manip/prism joint
    records = []
    for prim in Usd.PrimRange(root):
        name = prim.GetName()
        if not (name.startswith("manip_joint") or name.startswith("prism")):
            continue
        type_name = prim.GetTypeName()
        if type_name not in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
            continue

        # Resolve child link via physics:body1 relationship
        body1_rel = prim.GetRelationship("physics:body1")
        child_prim = None
        if body1_rel:
            targets = body1_rel.GetTargets()
            if targets:
                child_prim = stage.GetPrimAtPath(targets[0])

        if child_prim is None or not child_prim.IsValid():
            print(f"[ARM]   {name}: body1 link not found — skipping", flush=True)
            continue

        # Mass from MassAPI (set by user in Isaac Sim)
        mass_api = UsdPhysics.MassAPI(child_prim)
        mass = None
        if mass_api:
            m_attr = mass_api.GetMassAttr()
            if m_attr and m_attr.HasValue():
                mass = float(m_attr.Get())

        # Local translation of the joint prim (approximates where the joint axis sits
        # in the parent frame — useful for mount_pos and link-length estimation)
        xform = UsdGeom.Xformable(prim)
        t = xform.GetLocalTransformation().ExtractTranslation()
        jx, jy, jz = float(t[0]), float(t[1]), float(t[2])

        print(
            f"[ARM]   {name:20s}  child={child_prim.GetName():<20s}"
            f"  mass={mass} kg   joint_pos=({jx:+.4f}, {jy:+.4f}, {jz:+.4f})",
            flush=True,
        )
        records.append((name, child_prim.GetName(), mass, (jx, jy, jz)))

    if not records:
        print("[ARM]   No arm joints found — verify prim names start with manip_joint/prism", flush=True)
        return

    # Print paste-ready ArmCouplingParams block
    print(f"\n[ARM] --- Paste into geometric_controller_node.py ArmCouplingParams ---", flush=True)
    rev_records = [(n, l, m, p) for (n, l, m, p) in records if "manip_joint" in n]
    pri_records = [(n, l, m, p) for (n, l, m, p) in records if "prism" in n]

    total_arm_mass = sum(m for (_, _, m, _) in records if m is not None)
    print(f"[ARM]   # Total arm mass: {total_arm_mass:.4f} kg", flush=True)

    if len(rev_records) >= 2:
        # Link lengths: distance between consecutive joint origins
        positions = [p for (_, _, _, p) in rev_records]
        for i in range(len(positions) - 1):
            p0, p1 = positions[i], positions[i + 1]
            dist = float(sum((p1[k] - p0[k])**2 for k in range(3))**0.5)
            print(f"[ARM]   # L{i+1} (joint{i+1}→joint{i+2} dist): {dist:.4f} m", flush=True)

    # mount_pos: position of the first manip_joint in the drone body frame
    if rev_records:
        mp = rev_records[0][3]
        print(f"[ARM]   mount_pos: np.ndarray = np.array([{mp[0]:+.4f}, {mp[1]:+.4f}, {mp[2]:+.4f}])", flush=True)

    # Split arm mass roughly equally between link groups (user should refine)
    if len(rev_records) >= 2 and total_arm_mass > 0:
        masses = [m for (_, _, m, _) in rev_records if m is not None]
        if len(masses) >= 2:
            print(f"[ARM]   m1: float = {masses[0]:.4f}   # {rev_records[0][0]} link", flush=True)
            print(f"[ARM]   m2: float = {sum(masses[1:]):.4f}   # remaining links + gripper", flush=True)
        else:
            print(f"[ARM]   # Could not split masses — set m1/m2 manually from output above", flush=True)

    print(f"[ARM] -----------------------------------------------\n", flush=True)
