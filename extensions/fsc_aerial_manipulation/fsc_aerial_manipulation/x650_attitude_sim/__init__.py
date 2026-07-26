"""
x650_attitude_sim: standalone (no Isaac Sim required) deterministic Python
validation of a higher-fidelity X650 model -- real bench-calibrated rotor
thrust/lag (extensions/.../rotorcraft/x650_params.py, same numbers the real
Isaac Sim path uses), the new propeller parasitic-drag and rotor-
gyroscopic-torque models (extensions/.../aerodynamics/propeller_drag.py),
and a geometric SO(3) attitude tracker (ported from the companion
Automatica2026Simulation project's already-validated AGAS controller) --
starting with ATTITUDE-ONLY tracking (no position/altitude loop), per user
instruction.

Deliberately has NO import-time dependency on `pegasus.simulator`/`omni` (or
even on `fsc_aerial_manipulation.rotorcraft`'s own __init__.py, which does
have such a dependency -- see x650_params_loader.py) so it can be run in a
plain Python environment. This follows a documented precedent already in
this repo's own CLAUDE.md: Isaac Sim is not the right tool for verifying a
controller's tracking/stability behavior; that's better done in a fast
deterministic integrator first, with Isaac Sim reserved for later
integration testing of the real physics/ROS2 topology. See CLAUDE.md for
what was found running this (gain retuning needed vs. the source project's
placeholder-inertia gains; the propeller-parasitic-drag term's near-zero
contribution for pure attitude motion, verified rather than assumed).

Not yet wired into Isaac Sim / a Backend (see CLAUDE.md's scope note) --
that is an explicit next step, not part of this pass.
"""
