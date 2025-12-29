#!/usr/bin/env python
"""
| File: rotation_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Utility functions for computing common rotation transformations.
"""

import math

def quat_from_x_deg(deg: float):
    """
    Create a quaternion (w, x, y, z) representing
    a rotation about the X axis by `deg` degrees.
    """
    rad = math.radians(deg)
    half = rad * 0.5
    return (math.cos(half), math.sin(half), 0.0, 0.0)

def quat_from_y_deg(deg: float):
    """
    Create a quaternion (w, x, y, z) representing
    a rotation about the Y axis by `deg` degrees.
    """
    rad = math.radians(deg)
    half = rad * 0.5
    return (math.cos(half), 0.0, math.sin(half), 0.0)

def quat_from_z_deg(deg: float):
    """
    Create a quaternion (w, x, y, z) representing
    a rotation about the Z axis by `deg` degrees.
    """
    rad = math.radians(deg)
    half = rad * 0.5
    return (math.cos(half), 0.0, 0.0, math.sin(half))