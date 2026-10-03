"""Linear model of one lateral axis of the decoupled rig (x / pitch), 250 Hz, to explain the ~0.6 Hz airframe sway
of the 2026-10-02 flights. Exact discrete update of what the geometric+L1 node runs on that axis:

  plant      m x'' = m g theta;  J_t omega' = tau_act;  tau_act = first-order rotor lag (MN4010, lambda 10.03 1/s)
  feedback   x, v delayed d_p (EKF2-fused odometry), theta, omega delayed d_a (PX4 attitude / gyro)
  position   F_x = -Kp x - Kv v  [N];  theta_d = F_x / (m g)          (paper eq. 17, small angle)
  attitude   tau = -K_R (theta - theta_d) - K_w omega                 (omega_d = 0: the constant-yaw specialisation)
  L1 torque  gamma = -J_m a e^{aT}/(e^{aT}-1) (omega_hat - omega);  u_L1 <- alpha u_L1 - (1-alpha) gamma
             omega_hat: exact ZOH predictor driven by the COMMANDED torque u + gamma (not the rotor-lagged one)
Prints the lightly damped closed-loop modes (0.2-3 Hz) for the 09-28 and 2026-10-01 gain sets.
Usage: PYTHONNOUSERSITE=1 /usr/bin/python3 lateral_mode_model.py   -> ../analysis/lateral_mode_model.json
"""
import json, math
import numpy as np

M_KG, G, DT, LAM = 3.746170, 9.80665, 0.004, 10.0265
J_M = 0.119                                   # the law's l1geo_inertia_yy
SETS = {"0928 (flown 09-28)": dict(kp=4.0, kv=6.0, kr=1.0, kw=0.55, a=-2.0, wc=6.0),
        "1001 tune (flown 10-02)": dict(kp=20.11, kv=11.05, kr=3.337, kw=0.9505, a=-2.791, wc=1.0)}


def build(g, J_t=J_M, dp=6, da=3, l1=True):
    # state: x v th w tau u_l1 w_hat | x,v delay line (dp each) | th,w delay line (da each)
    n = 7 + 2 * dp + 2 * da
    ead = math.exp(g["a"] * DT); kad = g["a"] * ead / (ead - 1.0); alpha = math.exp(-g["wc"] * DT); bl = 1 - math.exp(-LAM * DT)

    def step(s):
        x, v, th, w, tau, ul1, wh = s[:7]; i = 7
        xd = s[i:i + dp]; vd = s[i + dp:i + 2 * dp]; i += 2 * dp
        thd = s[i:i + da]; wd = s[i + da:i + 2 * da]
        xm = xd[-1] if dp else x; vm = vd[-1] if dp else v; thm = thd[-1] if da else th; wm = wd[-1] if da else w
        th_d = (-g["kp"] * xm - g["kv"] * vm) / (M_KG * G)
        tcmd = -g["kr"] * (thm - th_d) - g["kw"] * wm
        if l1:
            gam = -J_M * kad * (wh - wm)
            ul1n = alpha * ul1 - (1 - alpha) * gam
            u = tcmd + ul1n
            b = (u + gam) / J_M - g["a"] * wm
            whn = ead * wh + (ead - 1) / g["a"] * b
        else:
            ul1n = 0.0; whn = 0.0; u = tcmd
        out = np.zeros(n)
        out[:7] = [x + DT * v, v + DT * G * th, th + DT * w, w + DT * tau / J_t, tau + bl * (u - tau), ul1n, whn]
        i = 7
        if dp:
            out[i:i + dp] = np.r_[x, xd[:-1]]; out[i + dp:i + 2 * dp] = np.r_[v, vd[:-1]]
        i += 2 * dp
        if da:
            out[i:i + da] = np.r_[th, thd[:-1]]; out[i + da:i + 2 * da] = np.r_[w, wd[:-1]]
        return out
    return np.column_stack([step(e) for e in np.eye(n)])


def modes(A):
    z = np.linalg.eigvals(A); s = np.log(z.astype(complex)) / DT
    r = [(abs(si.imag) / 2 / math.pi, -si.real / abs(si)) for si in s if si.imag > 1e-6 and 0.2 < abs(si.imag) / 2 / math.pi < 3.0]
    return sorted(r, key=lambda p: p[1])


if __name__ == "__main__":
    out = {}
    for name, g in SETS.items():
        for J_t in (0.119, 0.14):
            for dp, da in ((0, 0), (6, 3), (10, 5)):
                for l1 in (True, False):
                    m = modes(build(g, J_t, dp, da, l1)); lo = m[0] if m else (float("nan"), float("nan"))
                    key = f"{name} | J_true {J_t} | delay pos {dp*4} ms att {da*4} ms | L1 {'on' if l1 else 'off'}"
                    out[key] = dict(f_hz=lo[0], zeta=lo[1], all=m)
                    print(f"{key:85s} least-damped mode {lo[0]:5.2f} Hz  zeta {lo[1]:+.3f}")
    json.dump(out, open("../analysis/lateral_mode_model.json", "w"), indent=1)
