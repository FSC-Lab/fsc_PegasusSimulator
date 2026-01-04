#!/usr/bin/env python
"""
| File: rotorcraft_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Utility functions for definiting articulation joints between Xforms
"""

from pxr import UsdPhysics, Gf, Sdf, Usd

# helper function to determine the path to rigid-body prim
def find_rigidbody_prim(stage, root_path: str):
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim or not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if UsdPhysics.RigidBodyAPI.Get(stage, prim.GetPath()):
            return prim
    return None

# ----------------------------
# Create spherical joint
# ----------------------------
def create_spherical_joint(stage, joint_path, body0_path, body1_path, local_pos0, local_pos1):
    # body_path is the path where rigid-body propertiy is defined.
    if (body0_path is None) or (body1_path is None):
        raise RuntimeError(f"[Joint] body path is None for {joint_path}: body0={body0_path}, body1={body1_path}")

    p0 = stage.GetPrimAtPath(body0_path)
    p1 = stage.GetPrimAtPath(body1_path)
    if (not p0) or (not p0.IsValid()) or (not p1) or (not p1.IsValid()):
            raise RuntimeError(f"[Joint] Invalid prim path for {joint_path}: body0={body0_path}, body1={body1_path}")

    joint = UsdPhysics.SphericalJoint.Define(stage, Sdf.Path(joint_path))
    joint.CreateBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, local_pos0)))
    joint.CreateLocalPos1Attr(Gf.Vec3f(*map(float, local_pos1)))
    return joint_path