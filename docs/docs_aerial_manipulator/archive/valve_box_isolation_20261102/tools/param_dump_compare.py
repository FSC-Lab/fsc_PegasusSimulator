#!/usr/bin/env python3
"""Compare two `ros2 param dump` files of the same node: every parameter both declare must hold the
same value; parameters only one declares are listed with their value (a key new since the baseline
must sit at the value that reproduces the baseline behaviour -- judged by the reader, listed here).
  param_dump_compare.py <baseline.yaml> <new.yaml>"""
import sys
import yaml


def params(path):
    d = yaml.safe_load(open(path))
    (node,) = d.values()
    return node["ros__parameters"]


a, b = params(sys.argv[1]), params(sys.argv[2])
skip = {"use_sim_time", "qos_overrides"}
common = sorted((set(a) & set(b)) - skip)
diff = [k for k in common if a[k] != b[k]]
only_a = sorted(set(a) - set(b) - skip)
only_b = sorted(set(b) - set(a) - skip)
print(f"{len(common)} common parameters, {len(diff)} differ; {len(only_a)} only in the baseline, "
      f"{len(only_b)} only in the new build")
for k in diff:
    print(f"  DIFFERS {k}: {a[k]!r} -> {b[k]!r}")
for k in only_a:
    print(f"  ONLY BASELINE {k}: {a[k]!r}")
for k in only_b:
    print(f"  NEW {k}: {b[k]!r}")
sys.exit(1 if diff or only_a else 0)
