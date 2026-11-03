#!/usr/bin/env python3
"""Write a candidate controller yaml for run_mail_3cm.sh: the shipped mail-delivery SIM yaml with
named whole-body law keys replaced IN PLACE (same lines, comments kept), so a candidate differs
from the shipped file only where intended.

    /usr/bin/python3 make_gain_yaml.py <out.yaml> wb_k_x=60 wb_k_v=14 [--base <yaml>]
    /usr/bin/python3 make_gain_yaml.py <out.yaml> pick_place_place_approach_back=0.25 \
        --section whole_body_trajectory_planner

Only keys of ONE section are touched (/**/fsc_autopilot_ros2 unless --section names another); an
unknown key is an error, and so is a key the section carries twice. The diff against the base is
printed.
"""
import argparse
import difflib
import os
import re
import sys

BASE = os.path.expanduser(
    "~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/config/"
    "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_mail_delivery.yaml")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("sets", nargs="+", help="key=value")
    ap.add_argument("--base", default=BASE)
    ap.add_argument("--section", default="fsc_autopilot_ros2")
    a = ap.parse_args()
    lines = open(a.base).read().splitlines(keepends=True)
    start = next(i for i, ln in enumerate(lines) if ln.startswith(f"/**/{a.section}:"))
    end = next((i for i in range(start + 1, len(lines)) if re.match(r"^\S", lines[i])), len(lines))
    out = list(lines)
    for kv in a.sets:
        key, _, val = kv.partition("=")
        if not val:
            sys.exit(f"not key=value: {kv}")
        pat = re.compile(rf"^(\s+){re.escape(key)}:(\s*)([^#\n]*?)(\s*#.*)?(\n?)$")
        hits = [i for i in range(start, end) if pat.match(out[i])]
        if len(hits) != 1:
            sys.exit(f"{key}: {len(hits)} lines in the {a.section} section (need exactly 1)")
        m = pat.match(out[hits[0]])
        out[hits[0]] = f"{m.group(1)}{key}:{m.group(2) or ' '}{val}{m.group(4) or ''}{m.group(5)}"
    open(a.out, "w").writelines(out)
    sys.stdout.writelines(difflib.unified_diff(lines, out, os.path.basename(a.base), a.out, n=0))


if __name__ == "__main__":
    main()
