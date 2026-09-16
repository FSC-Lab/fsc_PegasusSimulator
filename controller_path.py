"""Locate the flight controller, which now lives in its own repository.

The controllers moved out of this repo into fsc_autopilot_ros2 so they can be
released and flown without the simulator. What remains here are the tests that
genuinely span both -- the aero match and the plant double -- and they need to
find the controller wherever it is installed.

Resolution order, first hit wins:
    1. $FSC_CONTROLLER_DIR            explicit override
    2. ~/ros2_ws/src/px4_vtol_direct_actuator   the workspace's view (a symlink)
    3. ~/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator   sibling checkout

Going through the workspace symlink means a future move of the controller
repository needs no change here at all.
"""
import os
import sys

_CANDIDATES = (
    os.environ.get("FSC_CONTROLLER_DIR"),
    os.path.expanduser("~/ros2_ws/src/px4_vtol_direct_actuator"),
    os.path.expanduser("~/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator"),
)


def controller_dir():
    """Directory holding the controller's setup.py (its package root)."""
    for cand in _CANDIDATES:
        if cand and os.path.isdir(os.path.join(cand, "px4_vtol_direct_actuator")):
            return os.path.realpath(cand)
    raise RuntimeError(
        "flight controller not found. It lives in fsc_autopilot_ros2 now; set "
        "FSC_CONTROLLER_DIR if it is somewhere else. Tried: "
        + ", ".join(str(c) for c in _CANDIDATES))


def inner_dir():
    """The importable package directory itself."""
    return os.path.join(controller_dir(), "px4_vtol_direct_actuator")


def load_airframe(path=None):
    """The controller's identified airframe (airframe_id.cfg) through its own loader, by file
    path (numpy only -- importing the package would pull casadi). `path` overrides the file, as
    FSC_AIRFRAME_ID does for the nodes. Piece 3 of the hardware-transfer plan (2026-09-15)."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("airframe_id", os.path.join(inner_dir(), "airframe_id.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.load(path)


def add_to_path():
    """Put both the package root and the package dir on sys.path.

    The root supports `from px4_vtol_direct_actuator import x` (how the code
    imports its own siblings); the package dir supports the bare `import x`
    that these tests have always used.
    """
    for p in (controller_dir(), inner_dir()):
        if p not in sys.path:
            sys.path.insert(0, p)
    return inner_dir()
