# PX4 yaw cascade on the AM-T650 mirror plant, step yaw torque at lift-off, integrator from 0.
import numpy as np
c_plant = 2.474152e-06            # t650_params ROLLING_MOMENT_COEFFICIENT (3 x bench)
kf = 4.041283e-05 * 1.037         # mirror kf
m, g = 3.746, 9.81
w = np.sqrt(m*g/4/kf)             # hover rotor speed
span = 730.0507 - 64.0603         # PX4 output -> omega
Kpsi, P, I_, K = 1.4, 0.2, 0.1, 0.3   # MC_YAW_P, MC_YAWRATE_P/I, MC_YAWRATE_K
lam = 10.0265
Izz = 0.12
def run(km_scale, tau_b, T=12.0, dt=1e-3):
    gain = 8 * c_plant*km_scale * w * span   # N.m per unit normalized yaw torque (PX4 yaw column normalised to 1)
    psi = r = integ = tau = 0.0; out = []
    for k in range(int(T/dt)):
        rsp = Kpsi * (0.0 - psi)
        e = rsp - r
        integ = np.clip(integ + K*I_*e*dt, -0.3, 0.3)   # MC_YR_INT_LIM
        u = K*P*e + integ
        tau += dt*lam*(gain*u - tau)                    # rotor lag
        r += dt*(tau + tau_b)/Izz; psi += dt*r
        out.append(np.degrees(psi))
    a = np.array(out); t = np.arange(len(a))*dt
    k = np.argmax(np.abs(a)); zc = np.where((t>t[k]) & (np.sign(a)!=np.sign(a[k])))[0]
    return gain, a[k], t[k], (t[zc[0]] if len(zc) else np.nan), a[-1]
print(f"hover omega {w:.0f} rad/s")
for lab, km, tb in [("mirror (km 0.2333, bias -0.095)", 0.2333, -0.095),
                    ("bias -0.104 (incl. force-bias moment)", 0.2333, -0.104),
                    ("km 1.0, bias -0.095", 1.0, -0.095),
                    ("km 0.2333, bias 0", 0.2333, 0.0)]:
    gain, pk, tpk, tz, fin = run(km, tb)
    print(f"{lab:40s} gain {gain:.3f} N.m/unit  peak {pk:+6.1f} deg at {tpk:4.2f}s  zero-cross {tz:5.2f}s  at 12 s {fin:+5.1f}")
