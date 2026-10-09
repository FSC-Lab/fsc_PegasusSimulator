import sys, numpy as np
sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))
from pose_sweep import row, TP, P, D
m = np.load("/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/pick_place_controllers_20261003/runs/arm_singularity_map.npz")
q2g, q3g, E = m["q2"], m["q3"], m["sig"]
sing = np.argwhere(E < 0.01); keep = np.argwhere(E < 0.10)
sp = np.column_stack([q2g[sing[:, 0]], q3g[sing[:, 1]]]); kp = np.column_stack([q2g[keep[:, 0]], q3g[keep[:, 1]]])
DQ2, DQ3 = 11.0, 10.0      # worst-case excursion box: q2 +-11 (WB worst 10.7), q3 +-10
print(f"excursion box q2 +-{DQ2:.0f}, q3 +-{DQ3:.0f} deg;   keep-out sigma_nd < 0.10")
print(f"{'pose':18s} beta  q2 worst  sig_nom  sig_min(box)  deg->singular  deg->keepout  reach  |tau2| |tau3| (200 g)")
for q2 in (-10, -5, 0, 5):
    for b in (10, 15, 20):
        q3 = b - q2
        if q3 > 50: continue
        r = row([0, q2, q3, 0])
        box = (np.abs(q2g[:, None] - q2) <= DQ2) & (np.abs(q3g[None, :] - q3) <= DQ3)
        smin = E[box].min()
        d0 = np.min(np.hypot(sp[:, 0] - q2, sp[:, 1] - q3)); d1 = np.min(np.hypot(kp[:, 0] - q2, kp[:, 1] - q3))
        print(f"[0,{q2:3d},{q3:3d},0]     {b:3d}   {q2-DQ2:6.0f}    {r['sk']:.3f}    {smin:.3f}         {d0:5.1f}         {d1:5.1f}     {r['reach']*1e3:4.0f}   {abs(r['tau1'][1]):.2f}  {abs(r['tau1'][2]):.2f}")
