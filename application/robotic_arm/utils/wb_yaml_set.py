#!/usr/bin/env python3
"""Rewrite a scalar in a controller yaml, in place, refusing an unknown key.

    /usr/bin/python3 wb_yaml_set.py --file <yaml> system_wd_max_drift_m=0.12

The companion of wb_l1_set_gains.py for keys OUTSIDE the wb_/wb_l1_ blocks
(watchdog limits, plant knobs, allocator constants). Same contract: the key
must already exist in the file -- a typo is an error, never a silent no-op --
and the surrounding comments are preserved untouched.
"""
import argparse
import re
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("assignments", nargs="+", metavar="key=value")
    ap.add_argument("--file", required=True)
    a = ap.parse_args()
    s = open(a.file).read()
    for kv in a.assignments:
        k, v = kv.split("=", 1)
        pat = re.compile(rf"^(\s*{re.escape(k.strip())}:\s*)(\S+)(\s*(#.*)?)$", re.M)
        m = pat.search(s)
        if not m:
            sys.exit(f"key '{k}' is not in {a.file}")
        s = pat.sub(lambda mm: mm.group(1) + v.strip() + mm.group(3), s, count=1)
        print(f"  {k.strip()}: {m.group(2)} -> {v.strip()}")
    open(a.file, "w").write(s)


if __name__ == "__main__":
    main()
