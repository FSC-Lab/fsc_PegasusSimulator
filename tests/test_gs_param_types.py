#!/usr/bin/env python3
"""Every parameter the auto mission sends must have a type the node will ACCEPT.

ROS 2 parameters are strongly typed, and the ground station types every mission parameter as
PARAMETER_DOUBLE unless the value is literally a bool (or the string "true"/"false"). So a node
that declares one of those names as a bool REJECTS the set -- and the mission aborts on ANY
rejected parameter, with a message naming only the parameter ("NMPC node did not accept inflow").

That is a launch-blocking failure that no offline gate and no HIL flight can catch, because it
only happens on the ground station's own path. This compares the two sides statically.
"""
import ast
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(HERE, "..", "ros2", "px4_vtol_direct_actuator",
                    "px4_vtol_direct_actuator", "nmpc_cascade_node.py")
GS = os.path.expanduser("~/ros2_ws/src/ros2_ground_station_gui/src/ROS_Node/trajectory_player.py")


def node_param_types():
    """name -> python type of the DECLARED default, from the P("name", default) calls."""
    out = {}
    for m in re.finditer(r'P\("([A-Za-z_0-9]+)"\s*,\s*([^)]+)\)', open(NODE).read()):
        name, raw = m.group(1), m.group(2).strip()
        try:
            out[name] = type(ast.literal_eval(raw))
        except Exception:
            pass
    return out


def main():
    if not os.path.exists(GS):
        print(f"  ground station not found at {GS} -- skipping")
        return 0
    gs = open(GS).read()
    types = node_param_types()

    m = re.search(r"CASC_KEYS = \{(.*?)\}", gs, re.S)
    keys = set(re.findall(r'"([A-Za-z_0-9]+)"', m.group(1))) if m else set()
    m2 = re.search(r"CASC_OVERRIDES = \{(.*?)\n    \}", gs, re.S)
    ov = dict(re.findall(r'"([A-Za-z_0-9]+)":\s*([^,\n]+)', m2.group(1))) if m2 else {}

    bad = []
    print(f"  ground station sends {len(keys)} parameters to the cascade node")
    for k in sorted(keys):
        t = types.get(k)
        if t is None:
            bad.append(f"'{k}' is sent by the mission but not declared by the node")
            continue
        if t is bool:
            bad.append(f"'{k}' is declared bool on the node, but the mission sends it as a "
                       f"DOUBLE -> the node rejects it and the mission aborts. Declare it as a "
                       f"float flag (like decel_now) instead.")
        else:
            print(f"    {k:22s} node default {t.__name__:5s} OK")

    # the override VALUES must be numeric for the same reason
    for k, raw in ov.items():
        try:
            v = ast.literal_eval(raw.strip())
        except Exception:
            continue
        if isinstance(v, bool):
            bad.append(f"CASC_OVERRIDES['{k}'] is a python bool; the node expects a float flag")

    if bad:
        print("\nPARAMETER TYPE MISMATCH:")
        for b in bad:
            print("   -", b)
        return 1
    print("\nMISSION PARAMETER TYPES MATCH THE NODE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
