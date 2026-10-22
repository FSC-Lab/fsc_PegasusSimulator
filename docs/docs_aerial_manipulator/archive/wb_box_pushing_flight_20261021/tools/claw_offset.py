#!/usr/bin/env python3
"""Why the claw sat 8-10 mm LOW under the world hold (170123, report item 9).

Reconstructs, from the logged raw L1 residual d_sigma_c (wb_control_debug
[62..71]), the arm's slow internal feed-forward the law applied in FREE
flight: d_int_f = C_x(d_c), C_x = 1/(s/0.1 + 1) per row (wb_l1_omega_x_t/r/q
0.1 in the box yaml; in free flight w_int = w_sigma, so T^-T w_int = d_c).
Checks it by rebuilding u3_est (= -dls(J_1y d_t + J_2y d_r + J_3y d_q),
logged at [89..92]). Then predicts the claw's static task error from what
that feed-forward leaves uncancelled,

    Lam_y M_y^-1 K_y e_y = J_1y (d_t - d_int_t) + J_2y (d_r - d_int_r)
                           + J_3y (d_q - d_int_q),

with the "present" disturbance d = d_c averaged over the trailing 1 s, and
compares it with the logged e_y [24..27]. Valid only while chi = free
(before 39.55 s): in contact w_int's joint rows become the frozen trim.

    /usr/bin/python3 claw_offset.py > ../claw_offset.txt
"""
import os
import sys
import numpy as np

from bp_common import Bag

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, *[".."] * 5))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

# the hardware box-pushing yaml as flown on 2026-10-21
OMEGA_X = 0.1                                   # wb_l1_omega_x_t/r/q
K_Y = np.array([120.0, 120.0, 120.0, 0.2484])   # wb_ky_x/y/z/psi
M_Y = np.diag([1.0, 1.0, 1.0, 0.05])            # wb_my_*, shaped
DLS = 0.3                                       # wb_dls_lambda
CONTACT_ON = 39.55                              # chi -> CONTACT (rosout)
WORLD_HOLD = 36.50                              # Ready done, claw world-held

PARAMS = TP.make_params_t650(base_com=[0.0, -0.017854, 0.0],
                             armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
RZ_M90 = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def quat_R(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def lowpass(x, t, w):
    y = np.zeros_like(x)
    for i in range(1, len(x)):
        a = np.exp(-w * (t[i] - t[i - 1]))
        y[i] = a * y[i - 1] + (1 - a) * x[i]
    return y


def trailing_mean(x, t, span):
    c = np.concatenate([np.zeros((1, x.shape[1])), np.cumsum(x, axis=0)])
    j = np.searchsorted(t, t - span)
    n = np.arange(len(t)) + 1 - j
    return (c[1:] - c[j]) / n[:, None]


def main():
    b = Bag("170123")
    t = b.t("wb")
    A = np.vstack(b.f("wb", "data")).astype(float)
    on = A[:, 2] > 0.5                          # gmo_active: the observer runs
    i0 = int(np.argmax(on))
    t, A = t[i0:], A[i0:]
    dc = np.nan_to_num(A[:, 62:72])
    d_int = lowpass(dc, t, OMEGA_X)             # the law's slow feed-forward
    d_now = trailing_mean(dc, t, 1.0)           # the disturbance at that moment

    tm = b.t("mocap")
    qm = np.column_stack([b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"])
    rows = []
    for k in range(0, len(t), 25):              # 10 Hz
        if t[k] > CONTACT_ON:
            break
        j = min(np.searchsorted(tm, t[k]), len(tm) - 1)
        R0 = quat_R(*qm[j]) @ RZ_M90            # frame_adapter: model = actual Rz(-90)
        X = np.zeros(26)
        X[3:12] = R0.reshape(-1, order="F")
        X[12:16] = A[k, 5:9]
        dyn = C.dynamics(X, PARAMS)
        J1, J2, J3, Lam = dyn["J_1y"], dyn["J_2y"], dyn["J_3y"], dyn["Lambda_y"]

        def task(d):
            return J1 @ d[0:3] + J2 @ d[3:6] + J3 @ d[6:10]

        JJt = J3 @ J3.T + DLS ** 2 * np.eye(4)
        u3_pred = -J3.T @ np.linalg.solve(JJt, task(d_int[k]))
        miss = d_now[k] - d_int[k]
        stiff = Lam @ np.linalg.inv(M_Y) @ np.diag(K_Y)
        parts = [np.linalg.solve(stiff, J1 @ miss[0:3]),
                 np.linalg.solve(stiff, J2 @ miss[3:6]),
                 np.linalg.solve(stiff, J3 @ miss[6:10])]
        rows.append((t[k], A[k, 24:28], sum(parts), parts, u3_pred, A[k, 89:93],
                     A[k, 5:9], d_now[k][6:10], d_int[k][6:10]))

    # u3_est check: reconstruction vs log, over the free-flight window
    err = np.array([r[4] - r[5] for r in rows])
    print(f"u3_est rebuilt vs logged [89..92], {rows[0][0]:.1f}-{rows[-1][0]:.1f} s: "
          f"rms {np.sqrt((err ** 2).mean(0)).round(4)} N.m, "
          f"logged rms {np.sqrt((np.array([r[5] for r in rows]) ** 2).mean(0)).round(4)}")
    print()
    print("  t     e_y z | pred z  (t: base trans  r: base rot  q: arm rows) [mm]   "
          "q2 q3 [deg]   d_q now (j2 j3) | ff (j2 j3) [N.m]")
    for r in rows:
        if r[0] < 26.0 or int(round(r[0] * 10)) % 5:
            continue
        tt, ey, pr, pa = r[0], r[1], r[2], r[3]
        print(f"{tt:5.1f}  {1e3 * ey[2]:6.1f} | {1e3 * pr[2]:6.1f}  "
              f"({1e3 * pa[0][2]:5.1f} {1e3 * pa[1][2]:5.1f} {1e3 * pa[2][2]:5.1f})   "
              f"{np.degrees(r[6][1]):5.1f} {np.degrees(r[6][2]):5.1f}   "
              f"{r[7][1]:6.3f} {r[7][2]:6.3f} | {r[8][1]:6.3f} {r[8][2]:6.3f}")

    def mean_in(t0, t1, f):
        sel = [f(r) for r in rows if t0 <= r[0] < t1]
        return np.mean(sel, axis=0)

    print()
    for name, (t0, t1) in {"stuck hover (CoM anchor)": (31.0, WORLD_HOLD),
                           "world hold, jaws open   ": (37.5, CONTACT_ON)}.items():
        ey = mean_in(t0, t1, lambda r: r[1][2])
        pr = mean_in(t0, t1, lambda r: r[2][2])
        pq = mean_in(t0, t1, lambda r: r[3][2][2])
        print(f"{name} {t0:4.1f}-{t1:4.1f} s: e_y z {1e3 * ey:6.1f} mm, predicted "
              f"{1e3 * pr:6.1f} mm (arm rows alone {1e3 * pq:6.1f})")


if __name__ == "__main__":
    main()
