#!/usr/bin/env python3
"""SAFETY hover before DIRECT entry: the SAFETY UDE's disturbance estimate (z) and the collective,
against the DIRECT motor mean right after entry -- a SAFETY thrust map matched to the allocator
leaves the UDE near 0 N.  Usage: safety_map.py <tag> <t0> <t1> (a SAFETY hover window)"""
import sys
import numpy as np
from bp_common import Bag

tag, a, c = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
b = Bag(tag)
t = b.t("ude"); m = (t >= a) & (t <= c)
out = []
for k in sorted(b.z.files):
    if k.startswith("ude__") and k.endswith(".z"):
        v = np.asarray(b.z[k], float)
        out.append(f"{k[5:]} {np.mean(v[m]):+.2f}")
ta = b.t("attsp")
th = [k for k in b.z.files if k.startswith("attsp__thrust")]
coll = [f"{k[7:]} {np.mean(np.asarray(b.z[k], float)[(ta >= a) & (ta <= c)]):.4f}" for k in th]
tw, W = b.t("wb"), b.f("wb", "data")
md = (tw > c + 3) & (tw < c + 8) & (W[:, 0] > 0.5)
print(f"{tag} SAFETY {a}..{c}: " + ", ".join(out) + " | " + ", ".join(coll) +
      f" | DIRECT motor mean {np.nanmean(W[md, 41:45]):.4f}")
