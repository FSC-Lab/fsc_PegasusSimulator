#!/usr/bin/env python3
"""Does the ground station's auto-mission parameter set actually EXIST on the cascade node?

WHY: `_set_node_params` counts an unknown parameter as a failure, and the mission build ABORTS on
any failure. The two wing-borne controllers do not share a parameter surface -- nmpc_full has ten
transition-shaping knobs (tilt_hold_deg, alt_auth_trans, pitch_gate_v, trans_sink, ...) that the
cascade does not declare and does not need. So the GS filters WB_PARAMS through CASC_KEYS before
sending. If that filter drifts out of step with the node -- in either direction -- every cascade
mission fails to launch, and it fails at the ground station with a message about the trim schedule.

This compares the two SOURCES directly, with no ROS and no running node:
    GS   trajectory_player.CASC_KEYS
    node the P("...") declarations in nmpc_cascade_node.py
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# The flight controller lives in fsc_autopilot_ros2 now, not in this repo.
# controller_path resolves it (workspace symlink, or $FSC_CONTROLLER_DIR), so a
# future move of that repository needs no change here.
sys.path.insert(0, os.path.join(HERE, ".."))
from controller_path import controller_dir, inner_dir   # noqa: E402
_CTRL = controller_dir()
_CTRL_INNER = inner_dir()

NODE = os.path.join(_CTRL,
                    "px4_vtol_direct_actuator", "nmpc_cascade_node.py")
GS = os.path.expanduser("~/ros2_ws/src/ros2_ground_station_gui/src/ROS_Node/trajectory_player.py")


def main():
    if not os.path.exists(GS):
        print(f"  ground station not found at {GS} -- skipping")
        return 0
    node_params = set(re.findall(r'P\("([A-Za-z_0-9]+)"', open(NODE).read()))
    gs = open(GS).read()

    m = re.search(r"CASC_KEYS = \{(.*?)\}", gs, re.S)
    if not m:
        print("  CASC_KEYS not found in the ground station")
        return 1
    casc_keys = set(re.findall(r'"([A-Za-z_0-9]+)"', m.group(1)))

    m2 = re.search(r"WB_PARAMS = \{(.*?)\n    \}", gs, re.S)
    wb_keys = set(re.findall(r'^\s*"([A-Za-z_0-9]+)":', m2.group(1), re.M)) if m2 else set()
    # the mission also injects these two derived values
    wb_keys |= {"vcruise", "cruise_alt"}

    bad = []
    m3 = re.search(r"CASC_OVERRIDES = \{(.*?)\}", gs, re.S)
    missing = sorted(casc_keys - node_params)
    if missing:
        bad.append(f"GS would send {missing} but the cascade node does not declare them "
                   f"-> every cascade mission aborts at build")
    # ★ A NAME MATCH IS NOT A MEANING MATCH. These names exist on BOTH nodes with different
    # semantics, so sending nmpc_full's value to the cascade is silently wrong even though every
    # name resolves. Keep them out of CASC_KEYS.
    COLLISIONS = {
        "tilt_slew_deg_s": ("nmpc_full: external limiter on the commanded tilt (15 deg/s); "
                            "cascade: model of the SIM SERVO rate limit (80 deg/s, a plant "
                            "property). Sending 15 stretches 0->90 deg from 1.1 s to 6 s."),
    }
    for nm, why in COLLISIONS.items():
        if nm in casc_keys:
            bad.append(f"CASC_KEYS includes '{nm}', which means something different on each "
                       f"node -- {why}")
        else:
            print(f"  OK   '{nm}' correctly NOT sent to the cascade")

    # A key is actually SENT if it comes from WB_PARAMS (filtered) or from CASC_OVERRIDES
    # (merged on top). The propeller model lives in the overrides, so WB_PARAMS alone is no
    # longer the whole source -- checking against it would call the prop keys dead.
    ov_keys = set(re.findall(r'"([A-Za-z_0-9]+)":', m3.group(1))) if m3 else set()
    dead = sorted(casc_keys - wb_keys - ov_keys)
    if dead:
        bad.append(f"CASC_KEYS lists {dead}, which appear in neither WB_PARAMS nor "
                   f"CASC_OVERRIDES -> filtering them is a no-op and they are never sent")

    # ★ THE CASCADE OVERRIDE MUST NOT REACH nmpc_full. WB_PARAMS is shared, and `ttr`=40 there is
    # flight-validated for nmpc_full (18 s is documented to ring its phugoid). The cascade's faster
    # ramp therefore lives in CASC_OVERRIDES, applied only on the cascade path.
    if m3:
        ov = dict(re.findall(r'"([A-Za-z_0-9]+)":\s*([0-9.]+)', m3.group(1)))
        wb_ttr = re.search(r'"ttr":\s*([0-9.]+)', m2.group(1)) if m2 else None
        if wb_ttr and abs(float(wb_ttr.group(1)) - 40.0) > 1e-6:
            bad.append(f"WB_PARAMS['ttr'] is {wb_ttr.group(1)}, not 40 -- that changes nmpc_full, "
                       f"whose 40 s ramp is flight-validated")
        else:
            print("  OK   WB_PARAMS['ttr'] still 40 for nmpc_full")
        for k in ov:
            if k not in casc_keys:
                bad.append(f"CASC_OVERRIDES sets '{k}' but it is not in CASC_KEYS, so it is "
                           f"filtered out before it is ever sent")
        if "ttr" in ov:
            print(f"  OK   cascade override ttr={ov['ttr']} (applied only on the cascade path)")
    else:
        bad.append("CASC_OVERRIDES not found in the ground station")

    print(f"  node declares      {len(node_params)} parameters")
    print(f"  WB_PARAMS has      {len(wb_keys)}")
    print(f"  GS sends cascade   {len(casc_keys)}: {' '.join(sorted(casc_keys))}")
    # informational: what the cascade deliberately does NOT take
    skipped = sorted(wb_keys - casc_keys)
    print(f"  filtered out       {len(skipped)}: {' '.join(skipped)}")

    if bad:
        print("\nMISMATCH:")
        for b in bad:
            print("   -", b)
        return 1
    print("\nGS CASCADE PARAMETER SET MATCHES THE NODE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
