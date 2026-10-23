"""
utils_controller — the aerial manipulator's control laws.

    controller.py         THE controller: the coupled feedback-linearizing law
                          + generalized-momentum disturbance observer. ONE law
                          for every scenario — free flight, pick, push, and the
                          PX4-gated rig. What differs between them is only
                          NUMBERS, and the numbers live in
                          robotic_arm/config/<scenario>.yaml (see
                          control_params.py). Merged 2026-08-09 from
                          controller_free/_pick/_push, which were three copies
                          of this same law that had begun to drift apart.

    control_params.py     the YAML loader: ControlParams + strict validation.
                          No gain defaults exist in code, so a run's parameters
                          can always be traced to one file.

    controller_track.py   a DIFFERENT law, kept on purpose: joint-space posture
                          anchor, no observer. The A/B baseline the GMO design
                          is measured against (01_aerial_manipulator_track.py).

    controller_hover.py   the ROS2 two-process hover demo's controller
                          (01_aerial_manipulator_hover.py), run standalone.

Desired TRAJECTORIES do not live here — they are all in utils_planner/,
including the pickplace*/push_home mission plans, which moved there with the
merge. FLIGHT PHASES (takeoff, handover, landing, the arm hold, the rotor
ramp-off) do not live here either: they belong to the demo scripts, which know
about a particular vehicle and a particular ground. This package answers only
"given the state and the reference right now, what are thrust, base moment and
joint torques?".

Usage:

    from fsc_aerial_manipulation.robotic_arm.utils_controller import (
        controller as C, control_params as CP)
    cfg  = CP.load("pick")                  # or "free" / "push" / "px4_direct"
    ctrl = C.MatlabController(C.make_params(), cfg)
"""
