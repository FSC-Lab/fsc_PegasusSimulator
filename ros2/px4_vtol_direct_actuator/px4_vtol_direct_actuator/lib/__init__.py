"""Shared library code for the FSC VTOL controllers.

Modules here are pure libraries -- no ROS node, no main(), nothing that runs on
its own. They are imported by the controllers in the parent package:

    smc_outer           sliding-mode outer loop        (3 importers)
    fsc_transition_ref  hover -> cruise trim schedule  (5 importers)
    fsc_mission_ref     full-mission reference         (1 importer)
    smc_mission         mission sequencing helpers

NOT here, deliberately: nmpc_so3_transition_node and nmpc6_cruise_node. Each is
genuinely dual-role -- a runnable controller AND the shared model/schedule hub
that six other controllers import. Separating those means splitting the plant
model out of an 81 KB node, which moves the JIT cache key and changes flown
behaviour, so it is its own job and not a file move.
"""
