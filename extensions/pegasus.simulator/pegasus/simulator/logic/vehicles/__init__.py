"""
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
"""

from .vehicle import Vehicle
from .multirotor import Multirotor, MultirotorConfig
from .vtol import (
    VTOL,
    VTOLConfig,
    RotorConfig,
    SurfaceConfig,
    dual_tiltrotor_config,
    quad_tiltrotor_config,
)

# The free-surface tank variant (tank plan).  A separate model, selected
# explicitly -- importing it does not change the baseline VTOL in any way.
from .vtol_slosh import VTOLSlosh, VTOLSloshConfig
