"""Rewrite an extractor npz so numpy 1.x can read it: object arrays (strings, string lists) -> fixed-width unicode.
The extractor runs on numpy 2 (rclpy); the plotting python (apt matplotlib) is numpy 1.21 and cannot unpickle numpy-2 object arrays."""
import sys, numpy as np
for p in sys.argv[1:]:
    d = np.load(p, allow_pickle=True); out = {}
    for k in d.files:
        a = d[k]
        if a.dtype == object:
            if a.ndim == 2 or (a.ndim == 1 and len(a) and isinstance(a[0], (list, np.ndarray))):
                a = np.array([",".join(map(str, x)) for x in a], dtype="U")
            else:
                a = np.array([str(x) for x in a], dtype="U")
        out[k] = a
    np.savez_compressed(p, **out); print("np1-compatible:", p)
