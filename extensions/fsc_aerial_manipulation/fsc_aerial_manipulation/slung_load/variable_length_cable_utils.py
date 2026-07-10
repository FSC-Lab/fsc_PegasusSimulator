#!/usr/bin/env python
"""
| File: variable_length_cable_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Utility functions for defining a variable-length winch cable (2 rigid rods
|   connected by a prismatic joint) for slung-load simulation in Isaac Sim.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from .slung_load_utils import create_cylinder_with_xform_root, get_mid_point

_AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}


def create_prismatic_rod_pair(
    stage,
    root_path: str,
    rod_a_length: float,
    rod_b_length: float,
    radius: float,
    mass_a: float,
    mass_b: float,
    world_pos,
    world_quat_wxyz,
    axis: str = "X",
    enable_collision: bool = False,
):
    """
    Build the two coaxial rigid-body cylinders that make up a variable-length winch cable:
    - rod_a: proximal rod (attaches to the drone side, stands in for the motor/spool housing)
    - rod_b: distal rod (attaches to the payload side, the telescoping part of the winch)

    Both rods are flush end-to-end at simulation start, spanning a combined length of
    `rod_a_length + rod_b_length` along `axis`, centered on `world_pos`/`world_quat_wxyz` —
    the same drone/payload-midpoint convention used by `create_cylinder_with_xform_root` for
    the existing fixed-length cable (e.g. `setup_variable_length_cable_geometry`'s returned
    `winch_root_pos`). rod_a occupies the negative-axis half (drone side), rod_b the
    positive-axis half (payload side), matching the sign convention of the fixed cable's
    `cable_end_to_uav`/`cable_end_to_payload` anchors. A `UsdPhysics.PrismaticJoint` connecting
    their facing ends (see `constraints.create_prismatic_joint`) is what lets rod_b slide
    relative to rod_a to represent cable extension/retraction; each rod's own far end (for the
    drone/payload spherical joints) is at local offset `-0.5*rod_a_length`/`+0.5*rod_b_length`
    respectively, expressed in that rod's own Xform frame.

    Returns:
        (rod_a_path, rod_b_path)
    """
    axis = axis.upper()
    ax_index = _AXIS_INDEX[axis]

    rod_a_path = root_path + "_rod_a"
    rod_b_path = root_path + "_rod_b"

    half_total = 0.5 * (rod_a_length + rod_b_length)

    # Rod centers, expressed as offsets from world_pos (assembly midpoint) along the local axis.
    offset_a = [0.0, 0.0, 0.0]
    offset_a[ax_index] = -half_total + 0.5 * rod_a_length
    offset_b = [0.0, 0.0, 0.0]
    offset_b[ax_index] = half_total - 0.5 * rod_b_length

    # Rotate the local-axis offsets into the world frame using world_quat_wxyz.
    w, x, y, z = map(float, world_quat_wxyz)
    rot = Rotation.from_quat([x, y, z, w])
    base_pos = np.array(world_pos, dtype=float)
    pos_a = base_pos + rot.apply(offset_a)
    pos_b = base_pos + rot.apply(offset_b)

    create_cylinder_with_xform_root(
        stage=stage,
        root_path=rod_a_path,
        length=rod_a_length,
        radius=radius,
        mass=mass_a,
        world_pos=pos_a,
        world_quat_wxyz=world_quat_wxyz,
        enable_collision=enable_collision,
        axis=axis,
    )

    create_cylinder_with_xform_root(
        stage=stage,
        root_path=rod_b_path,
        length=rod_b_length,
        radius=radius,
        mass=mass_b,
        world_pos=pos_b,
        world_quat_wxyz=world_quat_wxyz,
        enable_collision=enable_collision,
        axis=axis,
    )

    return rod_a_path, rod_b_path


def setup_variable_length_cable_geometry(L0, drone_pose):
    """
    Calculate payload position and winch root pose using the drone position and the initial
    total cable extension L0. Analogous to `slung_load_utils.setup_single_drone_payload`.

    - L0: initial cable extension (rod_a_length + rod_b_length)
    - drone_pose: position of the drone center
    """
    cx, cy, cz = map(float, drone_pose)
    L0 = float(L0)
    # offset payload in world x direction, matching setup_single_drone_payload's convention
    payload_pos = [cx + L0, cy, cz]
    winch_root_pos = get_mid_point(drone_pose, payload_pos)
    winch_quat_wxyz = (1.0, 0.0, 0.0, 0.0)
    return payload_pos, winch_root_pos, winch_quat_wxyz
