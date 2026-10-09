#!/usr/bin/env python3
"""Generate the DECOUPLED geometric + L1 rig's pick-and-place controller yaml
(2026-10-03, user request: both controllers fly the same pick-and-place task,
each from its own parameter file).

    /usr/bin/python3 make_geo_pick_place_yaml.py [--check]

Source: the 2026-10-01 tuned geometric + L1 MIRROR yaml
(archive/geometric_l1_tune_20261001/variants/geometric_l1_mirror_sim_tuned.yaml),
whose plant section (every sim_* key) is the whole-body mirror's -- verified
identical to the whole-body ..._sim_pick_and_place.yaml before writing. OVERRIDES
below are the pick-and-place changes to the controller (empty = the tuned
yaml as it flew the 10-01 comparison). The planner block is NOT here: the
decoupled stack reads it from the whole-body pick-and-place yaml
(WB_PLANNER_YAML), so both rigs plan the identical task.
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.abspath(os.path.join(HERE, "..", "..", "geometric_l1_tune_20261001",
                                   "variants", "geometric_l1_mirror_sim_tuned.yaml"))
CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
WB = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_place.yaml")
OUT = os.path.join(CFG, "params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim_pick_and_place.yaml")

# key -> (value, why). Applied to the controller node's section.
OVERRIDES = {
    "vehicle_name": ('"AM-T650-GEO-L1-PNP-SIM"', "this rig's identity in the logs"),
    # geo_1 (2026-10-03): the pick's EXIT failed -- the 200 g payload's weight
    # landed on the soft z loop (13.5 N/m, the L1 at omega_c 1 rad/s), the base
    # fell 0.1 m behind the 0.2 m climb, the clamped payload stayed on the cap
    # for >1 s (a closed chain with the pillar), the box was dragged sideways and
    # a ~1 Hz roll oscillation grew to the 15 deg safety guard. z acts on the
    # collective directly (not through the attitude loop): stiffen it so the
    # payload lifts at once. wn 3.2 rad/s, zeta 0.71 at 3.95 kg -- 3x under
    # the 10 rad/s rotor-lag pole.
    "l1geo_kp_z": ("40.0", "lift the payload off at once (geo_1: 13.5 lagged 0.1 m)"),
    "l1geo_kv_z": ("18.0", "zeta ~0.7 with kp_z 40"),
    # POSITION PAIR 20.11 / 11.05 -> 15 / 10 (2026-10-03, second pass). Carrying
    # the payload, this law's ~0.6 Hz roll/pitch sway (the zeta ~0.1 mode the
    # 10-02 hardware flights found in the 10-01 tune) grew to the 15 deg guard:
    # geo_5 / geo_9 (100 g), geo_10 (50 g). The linear one-axis model of the node
    # (tools/geo_sway_screen.py, with the payload as extra true inertia and
    # Isaac's mocap / attitude delays) puts its worst-case damping at +0.03 for
    # the 10-01 pair and -0.14 for the kp 30 / kv 13.5 tried first (geo_7..10 --
    # stiffer made it WORSE: kv is what erodes this mode once the payload adds
    # inertia). Kp 15 / Kv 10: zeta 0.20-0.22 for 0, 100 and 200 g; the 10-01
    # tune's 44 ms delay margin kept on its nonlinear bench (geo_kp_margin.py
    # --sway); the cost is a larger standing offset F / Kp (circle bench EE
    # 55 -> 68 mm), which the planner's descent trim cancels (cap 0.15 m).
    "l1geo_kp_x": ("15.0", "damps the payload-carrying 0.6 Hz sway (geo_sway_screen.py: zeta 0.20-0.22)"),
    "l1geo_kp_y": ("15.0", "damps the payload-carrying 0.6 Hz sway (geo_sway_screen.py: zeta 0.20-0.22)"),
    "l1geo_kv_x": ("10.0", "kv erodes the sway with the payload's inertia; 10 is the screen's optimum at kp 15"),
    "l1geo_kv_y": ("10.0", "kv erodes the sway with the payload's inertia; 10 is the screen's optimum at kp 15"),
}

HEADER = """# ============================================================================
# !! GENERATED -- DO NOT EDIT. DECOUPLED GEOMETRIC + L1 PICK-AND-PLACE (SIM). !!
#   fsc_PegasusSimulator docs/docs_aerial_manipulator/archive/pick_place_controllers_20261003/
#   tools/make_geo_pick_place_yaml.py -- edit there and regenerate.
# = archive/geometric_l1_tune_20261001/variants/geometric_l1_mirror_sim_tuned.yaml
#   (the geometric + L1 law on the whole-body MIRROR plant, 10-01 bench tune)
#   with the pick-and-place overrides listed in the generator. The planner
#   block comes from the whole-body ..._sim_pick_and_place.yaml (WB_PLANNER_YAML).
#   SIMULATION ONLY.
# ============================================================================
"""


def plant(path):
    return sorted(re.sub(r"#.*", "", l).rstrip() for l in open(path) if re.match(r"^\s+sim_", l))


def build():
    if plant(SRC) != plant(WB):
        sys.exit("ERROR: the source's sim_* plant keys differ from the whole-body pick-and-place yaml's")
    out, seen = [], set()
    for line in open(SRC):
        m = re.match(r"^(\s+)([A-Za-z0-9_]+):\s*(.*)$", line)
        if m and m.group(2) in OVERRIDES:
            v, why = OVERRIDES[m.group(2)]
            line = f"{m.group(1)}{m.group(2)}: {v}  # PICK-AND-PLACE OVERRIDE -- {why}\n"
            seen.add(m.group(2))
        out.append(line)
    missing = set(OVERRIDES) - seen
    if missing:
        sys.exit(f"ERROR: override keys not in the source: {sorted(missing)}")
    return HEADER + "".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = build()
    if a.check:
        ok = os.path.exists(OUT) and open(OUT).read() == text
        print("on-disk file is current" if ok else "on-disk file is STALE -- regenerate")
        sys.exit(0 if ok else 1)
    open(OUT, "w").write(text)
    print(f"plant identical to the whole-body pick-and-place yaml; {len(OVERRIDES)} override(s)\nwrote {OUT}")


if __name__ == "__main__":
    main()
