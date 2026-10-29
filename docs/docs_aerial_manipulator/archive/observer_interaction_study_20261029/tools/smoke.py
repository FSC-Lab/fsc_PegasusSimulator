import time, numpy as np, valve_bench as VB
S = VB.hold_stream(t_total=20.0)
t0 = time.time(); v, H = VB.simulate(VB.VALVE, S, seed=1); el = time.time() - t0
print("verdict", v, "elapsed", round(el, 1), "s for", round(H["t"][-1], 1), "s sim")
o = VB.score(S, v, H); print(VB.fmt("smoke V0", o))
k = np.searchsorted(H["t"], 12.5)
print("F_rad/F_tan/F_z at 12.5 s", H["F_rad"][k], H["F_tan"][k], H["F_z"][k])
