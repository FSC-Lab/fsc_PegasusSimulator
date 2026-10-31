#!/usr/bin/env python3
"""F_hat_y as the law consumed it (debug [58..61]) in contact, sim vs hardware, and the raw 4-D
reading [97..99]: is the contact-only rendering path fed by much less in Isaac?"""
import sys, os
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import step_times
def stats(F):
    return " / ".join(f"{s:.2f}" for s in F.std(0))
print("hardware:")
for tag in BAGS:
    b = Bag(tag); st = step_times(b); tw = b.t("wb"); W = b.f("wb", "data")
    m = (tw > st["push"] + 0.3) & (tw < st["abort"]) & np.isfinite(W[:, 100])
    print(f"  {tag}: consumed sd x/y/z {stats(W[m, 58:61])} N   raw sd {stats(W[m, 97:100])} N")
d = sys.argv[1]
print("Isaac (drift model):")
for tag in sys.argv[2:]:
    z = np.load(os.path.join(d, tag + ".npz"), allow_pickle=True); M = dict(zip([str(n) for n in z["marks_name"]], z["marks_t"]))
    D = z["dbg"]; m = (D[:, 0] > M["push:start"] + 0.3) & (D[:, 0] < M.get("push:end", D[-1, 0])) & np.isfinite(D[:, 102])
    print(f"  {tag}: consumed sd x/y/z {stats(D[m, 60:63])} N   raw sd {stats(D[m, 99:102])} N")
