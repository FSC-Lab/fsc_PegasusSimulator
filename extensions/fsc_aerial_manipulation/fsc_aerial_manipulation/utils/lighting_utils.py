#!/usr/bin/env python
"""
| File: lighting_utils.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: Utility functions for defining lighting in Isaac Sim
"""

from pxr import UsdLux, Sdf, Gf

def add_dome_lighting(
    stage=None,
    dome_path="/World/DomeLight",
    intensity=2500.0,
    exposure=0.0,
    color=(1.0, 1.0, 1.0)
):
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path(dome_path))

    # Basic lighting parameters
    dome.CreateIntensityAttr(float(intensity))               # brightness
    dome.CreateExposureAttr(float(exposure))                   # extra EV adjustment
    dome.CreateColorAttr(Gf.Vec3f(*color))  # white light
    return dome