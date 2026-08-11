"""
utils_planner — every desired trajectory for the aerial manipulator, in one
package.

The rule this package exists to enforce: a demo NEVER builds a reference dict
itself. It picks a trajectory by name, anchors it once, and evaluates it each
step through this API. If a motion is not expressible here, it gets added here.
Before this package the takeoff ramp was inlined in each demo's physics
callback while everything else lived in the controller, so "what is the vehicle
being told to do?" had two answers in two files, duplicated per demo.

  trajectory_library.py    the catalogue + anchoring + online evaluation.
                           Names follow `<shape>_<who>`: hover/circle/figure8/
                           poly x drone (arm locked) / whole (whole body moves)
  arm_sweep.py             the prescribed arm-sweep joint profiles + FK behind
                           the hover/circle/figure8 `…_whole` entries
  compatible_trajectory.py the offline dynamically-compatible EE+CoM solve
                           (Picard fixed point) behind the poly_whole entry

Typical use:

    from fsc_aerial_manipulation.robotic_arm import utils_planner as P

    P.set_traj_type("poly_whole")                    # once, at startup
    q0 = P.initial_arm_pose()                        # pre-Isaac spawn pose
    tr = P.build_traj(x_c0, d0, b1_0, params=params) # anchor at handover
    ref = P.setpoint_reference(p_takeoff, tr)        # takeoff / landing phases
    ref = P.generate_reference(t_since_handover, tr) # task phase
    # legacy (px4 demo): the takeoff climb RAMP instead of a setpoint
    ref, climb_z = P.takeoff_reference(climb_z, tr, dt)

Scope: EVERY desired trajectory, free-flight and contact alike. The mission
plans (pickplace*, push_home) moved here on 2026-08-09 when the three scenario
controllers merged into one law — they are trajectories, so this is where they
belong; what used to make a "pick controller" was this catalogue, not the law.

Pure numpy: importable and testable without Isaac or ROS2.
"""

from .trajectory_library import (
    TRAJ_CONFIG,
    TAKEOFF_ALTITUDE,
    TAKEOFF_TIME,
    CLIMB_RATE,
    set_traj_type,
    get_traj_type,
    traj_config,
    initial_arm_pose,
    build_traj,
    generate_reference,
    setpoint_reference,
    takeoff_reference,
    fixed_ee,
    minjerk,
    minsnap,
    arm_fk,
)
from .compatible_trajectory import plan_compatible_trajectory, get_plan

__all__ = [
    "TRAJ_CONFIG", "TAKEOFF_ALTITUDE", "TAKEOFF_TIME", "CLIMB_RATE",
    "set_traj_type", "get_traj_type", "traj_config", "initial_arm_pose",
    "build_traj", "generate_reference", "setpoint_reference",
    "takeoff_reference", "fixed_ee", "minjerk", "arm_fk", "arm_fk",
    "initial_arm_pose", "minsnap",
    "plan_compatible_trajectory", "get_plan",
]
