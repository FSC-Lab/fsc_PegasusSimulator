"""(2026-10-18, Opus) Finite-difference check of the free-axis anchor's derivative chain, the same
arithmetic as wb_controller.cpp (WbGains::ee_anchor_free_weight): on a body turning at a constant
yaw rate the velocity AND acceleration chains are exact; with yaw acceleration / tilt oscillation
the acceleration misses the omega_0_dot terms (dropped on purpose -- ~2 mN at a valve turn).
    PYTHONNOUSERSITE=1 /usr/bin/python3 free_axis_fd_check.py"""
import numpy as np


def Rz(a): return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
def Ry(a): return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


AX = np.array([0.0, 1.0, 0.0]); Z = np.array([0.0, 0.0, 1.0])
for tilt_amp, psidd in [(0.0, 0.0), (0.03, 0.1)]:
    def state(t):
        R = Rz(0.43 * t + psidd * np.sin(1.3 * t)) @ Ry(tilt_amp * np.sin(2.0 * t))
        ex = np.array([0.03 * np.cos(0.7 * t), 0.01 * np.sin(1.1 * t), 0.004 * np.sin(0.5 * t)])
        return R, ex

    def omega_body(t, h=1e-6):
        R0, _ = state(t); R1, _ = state(t + h); Rm, _ = state(t - h)
        W = R0.T @ ((R1 - Rm) / (2 * h)); return np.array([W[2, 1], W[0, 2], W[1, 0]])

    def offset(t):
        R, ex = state(t); h = 1e-5
        _, exp = state(t + h); _, exm = state(t - h)
        evx = (exp - exm) / (2 * h); eax = (exp - 2 * ex + exm) / h ** 2
        aw = R @ AX; hh = np.array([aw[0], aw[1], 0.0]); n = np.linalg.norm(hh); a = hh / n
        awd = R @ np.cross(omega_body(t), AX); hd = np.array([awd[0], awd[1], 0.0])
        ad = (hd - a * a.dot(hd)) / n
        phid = np.cross(Z, a).dot(ad)
        s = a.dot(ex); sd = ad.dot(ex) + a.dot(evx); sdd = a.dot(eax) + 2 * ad.dot(evx) - phid ** 2 * s
        return s * a, sd * a + s * ad, sdd * a + 2 * sd * ad - s * phid ** 2 * a

    wv = wa = 0.0
    for t in np.linspace(0.2, 9, 12):
        o, od, odd = offset(t); h = 1e-3
        op, _, _ = offset(t + h); om, _, _ = offset(t - h)
        wv = max(wv, np.linalg.norm(od - (op - om) / (2 * h)) / max(np.linalg.norm(od), 1e-9))
        wa = max(wa, np.linalg.norm(odd - (op - 2 * o + om) / h ** 2) / max(np.linalg.norm(odd), 1e-9))
    print(f"tilt {tilt_amp}, yaw accel {psidd}: worst relative error velocity {wv:.1e}, acceleration {wa:.3f}")
