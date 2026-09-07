#!/usr/bin/env python3
"""Rewrite wb_l1_* values in the whole-body L1 yaml, in place.

    /usr/bin/python3 wb_l1_set_gains.py omega_c_t=6.0 omega_c_r=2.0 decompose=false
    /usr/bin/python3 wb_l1_set_gains.py --show

The node has NO on-set-parameters callback, so `ros2 param set` changes what
`param get` reports while the controller keeps flying the launch-time value.
Every gain change therefore needs a yaml edit plus a full relaunch -- this is
that edit, done in a way that cannot mistype a key: an unknown name is an
error, and the surrounding comments (which carry the measured numbers behind
each default) are preserved untouched.

Keys are given WITHOUT the wb_l1_ prefix. --file points at a different yaml;
the default is the shipped simulation one.
"""

import argparse
import os
import re
import sys

DEFAULT = os.path.join(
    os.environ.get("FSC_AUTOPILOT_WS", os.path.expanduser("~/ros2_ws")),
    "src", "fsc_autopilot_ros2", "config",
    "params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml")

KNOWN = ("observer_type a_t a_r a_q adapt_period_s omega_c_t omega_c_r "
         "omega_c_q omega_i omega_x lc_var_f lc_var_m lc_var_q decompose "
         "max_force_n max_torque_nm max_joint_nm max_wrench_force_n "
         "max_wrench_torque_nm").split()

# LAW gains, given with their full name so there is no chance of confusing
# e.g. wb_k_r (attitude) with wb_l1_a_r (predictor pole). Any wb_* key already
# present in the file is accepted; the check is existence in the file, which
# is stricter than a hard-coded list and cannot go stale.
LAW_PREFIX = "wb_"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("assignments", nargs="*", metavar="key=value")
    ap.add_argument("--file", default=DEFAULT)
    ap.add_argument("--show", action="store_true")
    a = ap.parse_args()

    s = open(a.file).read()
    if a.show or not a.assignments:
        for k in KNOWN:
            m = re.search(rf"^(\s*)wb_l1_{k}:\s*(\S+)", s, re.M)
            print(f"  wb_l1_{k:<22s} = {m.group(2) if m else '(absent)'}")
        return 0

    for asg in a.assignments:
        if "=" not in asg:
            print(f"ERROR: '{asg}' is not key=value", file=sys.stderr)
            return 2
        k, v = asg.split("=", 1)
        k = k.strip()
        if k.startswith(LAW_PREFIX) and not k.startswith("wb_l1_"):
            full = k                      # a law gain, given in full
        else:
            short = k.removeprefix("wb_l1_")
            if short not in KNOWN:
                print(f"ERROR: unknown key '{k}'. Known observer keys: "
                      f"{', '.join(KNOWN)}; law gains must be given with "
                      f"their full wb_ name.", file=sys.stderr)
                return 2
            full = f"wb_l1_{short}"
        pat = re.compile(rf"^(\s*{full}:\s*)(\S+)(.*)$", re.M)
        if not pat.search(s):
            print(f"ERROR: {full} not found in {a.file}", file=sys.stderr)
            return 2
        # Keep the quoting style the file already uses for strings.
        cur = pat.search(s).group(2)
        if cur.startswith('"') and not v.startswith('"'):
            v = f'"{v}"'
        s = pat.sub(lambda m: m.group(1) + v + m.group(3), s, count=1)
        print(f"  {full} : {cur} -> {v}")

    open(a.file, "w").write(s)
    print(f"wrote {a.file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
