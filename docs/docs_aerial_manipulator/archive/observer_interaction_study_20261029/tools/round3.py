import os, sys
from concurrent.futures import ProcessPoolExecutor
import valve_bench as VB
J = []
for b in (0.02, 0.04, 0.06, 0.08):
    for clamp in (False, True):
        J.append((f"b {b:.2f} {'CLAMPED' if clamp else 'free   '}", VB.VALVE, dict(clamp=clamp, neg_damp=(b, 0, 0, b), fb=VB.WH.FB_RAW), 1))
with ProcessPoolExecutor(max_workers=14) as ex:
    for n, s, o in ex.map(VB.job, J):
        print(VB.fmt(n, o), flush=True)
