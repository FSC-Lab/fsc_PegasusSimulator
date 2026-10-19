"""pick_place/info of PP-4 decoded: adjust, goals per leg, capture, place point, EE offset; arrival_error around the presses."""
from pp10_common import *
np.set_printoptions(precision=4, suppress=True, linewidth=220)
for nm in sys.argv[1:] or ["p4"]:
    d, t0 = load(nm)
    I = d["pp_info__data"]; ti = d["pp_info__recv"] - t0
    print("=====", nm, "info rows", I.shape)
    k = np.argmax(I[:, I_PLANNED] > 0.5); info = I[k]
    print("t of first planned info", ti[k])
    print("raw info:", np.round(info, 4).tolist())
    for j in range(len(I)):
        chg = "" if j == 0 else str(np.where(np.abs(np.nan_to_num(I[j]) - np.nan_to_num(I[j - 1])) > 1e-9)[0].tolist())
        print(f"  t={ti[j]:7.2f} planned {I[j, I_PLANNED]:.0f} done {I[j, I_DONE]:.0f} flying {I[j, I_FLYING]:.0f} tol {I[j, I_TOL]:.3f} changed idx {chg}")
    A = d["pp_arr__data"]; ta = d["pp_arr__recv"] - t0
    print("arrival_error shape", A.shape, "first", ta[0], "last", ta[-1])
    for j in range(0, len(A), max(1, len(A) // 60)):
        print(f"  t={ta[j]:7.2f}", np.round(A[j], 4).tolist())
