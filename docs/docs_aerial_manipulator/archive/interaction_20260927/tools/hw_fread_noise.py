import glob, os, numpy as np
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
dt = 1 / 250
print("hardware |LP_w(F_raw[0:3])| over DIRECT (entry + last 2 s excluded), N: p50 / p95 / p99 / p99.9 / max")
allF = {}
for f in sorted(glob.glob(os.path.join(D, "hw_*.npz"))):
    d = np.load(f); A = d["A"]; t = d["t"]
    direct = A[:, 0] == 1
    idx = np.where(direct)[0]
    Fr = A[:, 97:100]
    nm = os.path.basename(f)[3:-4]
    line = f"{nm[-15:]:16}"
    # the flown reading (w_e, [72..74], EE frame, same norm)
    we = np.linalg.norm(A[:, 72:75], axis=1)
    for w in (0.207, 0.5, 1.0, 2.0, 5.0):
        a = np.exp(-w * dt); y = np.zeros(3); out = np.full(len(t), np.nan)
        for i in idx:
            y = a * y + (1 - a) * np.nan_to_num(Fr[i]); out[i] = np.linalg.norm(y)
        keep = direct & (t > t[idx[0]] + 10) & (t < t[idx[-1]] - 2)
        v = out[keep]
        allF.setdefault(w, []).append(v)
        line += f" | w={w:<5} {np.percentile(v,50):4.2f}/{np.percentile(v,95):4.2f}/{np.percentile(v,99):4.2f}/{np.percentile(v,99.9):4.2f}/{v.max():4.2f}"
    keep = direct & (t > t[idx[0]] + 10) & (t < t[idx[-1]] - 2)
    line += f" | flown w_e p99 {np.percentile(we[keep],99):.2f} max {we[keep].max():.2f} | chi_free min {np.nanmin(A[keep,105]):.0f} | F_raw std {np.nanstd(Fr[keep],axis=0).round(2)}"
    print(line)
print("POOLED over 5 flights:")
for w, L in allF.items():
    v = np.concatenate(L)
    print(f"  w={w:<5}: p50 {np.percentile(v,50):.2f}  p95 {np.percentile(v,95):.2f}  p99 {np.percentile(v,99):.2f}  p99.9 {np.percentile(v,99.9):.2f}  max {v.max():.2f}  N")
