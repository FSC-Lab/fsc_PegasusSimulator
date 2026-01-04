#!/usr/bin/env python
"""
| File: slung_load_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Utility functions for defining slung load using Isaac Sim
"""

from pxr import UsdPhysics, UsdGeom, Gf
import math

def cylinder_inertia_uniform_body_diag(mass: float, length: float, radius: float, axis: str = "Z"):
    """
    Uniform solid cylinder inertia (kg*m^2) returned as (Ixx, Iyy, Izz) in the BODY frame,
    assuming the cylinder symmetry axis is aligned with BODY axis `axis` (X/Y/Z).

    Use this when body-axis == visual axis (always aligned).
    Then you can set principalAxes = identity.
    """
    m = float(mass)
    L = float(length)
    r = float(radius)
    ax = axis.upper()

    I_axial = 0.5 * m * r * r
    I_trans = (1.0/12.0) * m * (3.0*r*r + L*L)

    if ax == "X":
        return (I_axial, I_trans, I_trans)  # Ixx axial
    elif ax == "Y":
        return (I_trans, I_axial, I_trans)  # Iyy axial
    elif ax == "Z":
        return (I_trans, I_trans, I_axial)  # Izz axial
    else:
        raise ValueError(f"axis must be one of 'X','Y','Z', got {axis}")

def box_inertia_uniform_body_diag(mass: float, lx: float, ly: float, lz: float):
    """
    Uniform solid box inertia about its center, aligned with body axes.
    Dimensions are full lengths along X/Y/Z (meters).
    Returns (Ixx, Iyy, Izz) in kg*m^2.
    """
    m  = float(mass)
    x  = float(lx)
    y  = float(ly)
    z  = float(lz)
    Ixx = (1.0/12.0) * m * (y*y + z*z)
    Iyy = (1.0/12.0) * m * (x*x + z*z)
    Izz = (1.0/12.0) * m * (x*x + y*y)
    return Ixx, Iyy, Izz

def get_mid_point(p0_world, p1_world):
    mid = [0.5*(p0_world[0] + p1_world[0]),
           0.5*(p0_world[1] + p1_world[1]),
           0.5*(p0_world[2] + p1_world[2])]
    return mid

def create_cylinder_with_xform_root(
    stage,
    root_path: str,
    length: float,
    radius: float,
    mass: float,
    world_pos,
    world_quat_wxyz,
    enable_collision: bool=False,
    axis: str = "Z",   # cylinder axis in Xfrom prim frame
):
    """
    Create a rigid-body cylinder with:
    - Xform root (rigid-body + joint frame)
    - Cylinder geometry child
    """

    # ------------------------------------------------
    # 1) Root Xform = rigid-body frame
    # ------------------------------------------------
    root_prim = stage.DefinePrim(root_path, "Xform")
    xf = UsdGeom.Xformable(root_prim)
    xf.ClearXformOpOrder()

    # operations are peformed in the local frame
    # translate -> rotate gives the correct world position and orientation
    xf.AddTranslateOp().Set(Gf.Vec3f(*map(float, world_pos)))
    w, x, y, z = map(float, world_quat_wxyz)
    xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))

    # Rigid body lives on the Xform
    UsdPhysics.RigidBodyAPI.Apply(root_prim)

    # Mass
    try:
        mass_api = UsdPhysics.MassAPI.Apply(root_prim)
        mass_api.CreateMassAttr().Set(float(mass))
        # Principal moments (Ixx, Iyy, Izz) in kg*m^2
        Ixx, Iyy, Izz = cylinder_inertia_uniform_body_diag(mass=mass,
                                              length=length,
                                              radius=radius,
                                              axis=axis)
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(Ixx, Iyy, Izz))
        mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    except Exception as e:
        print("[Cable] MassAPI failed:", e)

    # ------------------------------------------------
    # 2) Geometry child (visual + collider)
    # ------------------------------------------------
    geom_path = root_path + "/geom"
    geom = stage.DefinePrim(geom_path, "Cylinder")
    cyl = UsdGeom.Cylinder(geom)

    cyl.CreateHeightAttr(float(length))
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateAxisAttr(axis.upper())

    # ------------------------------------------------
    # 3) Collision API — explicitly DISABLED
    # ------------------------------------------------
    col_api = UsdPhysics.CollisionAPI.Apply(geom)
    col_api.CreateCollisionEnabledAttr().Set(bool(enable_collision))

    # NOTE:
    # Joint anchors must be expressed in the Xform frame:
    #
    #   Z-aligned cable:
    #     (0, 0, ±length/2)
    #
    #   Y-aligned cable:
    #     (0, ±length/2, 0)
    #
    # Geometry axis does NOT affect joints.

    return root_prim

# ----------------------------
# Create payload
# ----------------------------
def create_brick_with_xform_root(
    stage,
    root_path: str,
    length_x: float,
    width_y: float,
    height_z: float,
    mass: float,
    world_pos,
    world_quat_wxyz,
    enable_collision: bool = True,
):
    """
    Create a rigid-body brick (box) with:
    - Xform root (rigid-body + joint frame)
    - Cube geometry child scaled to (length_x, width_y, height_z)
    - Mass + uniform box inertia on the root
    """

    # -----------------------------
    # 1) Root Xform (rigid body)
    # -----------------------------
    root = stage.DefinePrim(root_path, "Xform")
    xf = UsdGeom.Xformable(root)
    xf.ClearXformOpOrder()

    xf.AddTranslateOp().Set(Gf.Vec3f(*map(float, world_pos)))
    w, x, y, z = map(float, world_quat_wxyz)
    xf.AddOrientOp().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))

    UsdPhysics.RigidBodyAPI.Apply(root)

    try:
        mass_api = UsdPhysics.MassAPI.Apply(root)
        mass_api.CreateMassAttr().Set(float(mass))
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        Ixx, Iyy, Izz = box_inertia_uniform_body_diag(mass, length_x, width_y, height_z)
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(Ixx, Iyy, Izz))

        # Body axes already principal axes for an axis-aligned box
        mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    except Exception as e:
        print("[Brick] MassAPI failed:", e)

    # -----------------------------
    # 2) Geometry child (visual/collider)
    # -----------------------------
    geom_path = root_path + "/geom"
    geom = stage.DefinePrim(geom_path, "Cube")
    cube = UsdGeom.Cube(geom)

    # Make a unit cube and scale it into a brick
    cube.CreateSizeAttr(1.0)

    gxf = UsdGeom.Xformable(geom)
    gxf.ClearXformOpOrder()
    gxf.AddScaleOp().Set(Gf.Vec3f(float(length_x), float(width_y), float(height_z)))

    # -----------------------------
    # 3) Collision
    # -----------------------------
    col_api = UsdPhysics.CollisionAPI.Apply(geom)
    col_api.CreateCollisionEnabledAttr().Set(bool(enable_collision))

    return root


def setup_single_drone_payload(L, drone_pose):
    """
    Calculate payload position, cable position and orientation using drone position and cable length
    - L: cable length
    - drone_pose: position of the drone center
    """
    cx, cy, cz = map(float, drone_pose)
    L = float(L)
    # offset payload in world x direction
    payload_pos = [cx + L, cy, cz]
    cable_pose = get_mid_point(
        drone_pose,
        payload_pos
    )
    cable_quat_wxyz = (1.0, 0.0, 0.0, 0.0)
    return payload_pos, cable_pose, cable_quat_wxyz