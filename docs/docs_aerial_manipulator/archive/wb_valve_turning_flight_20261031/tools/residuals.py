#!/usr/bin/env python3
"""What did the base receive that the law did not command? residuals.py <tag> <t0> <t1> [dt]
From the mocap pose (120 Hz, low-passed): F_res (world, N) = m a - (R e3 u1 - m g e3); tau_res (body FLU, N.m) =
I omega_dot + omega x I omega - tau_cmd (the law's body torque [21..23], which already carries the arm-reaction
pre-compensation). Beside it, the ARM's un-modelled joint torques (applied e.m. torque Kt*I - the streamed law torque)
projected on the base: j1 -> yaw, j2+j3 -> pitch, j4 -> (roll sin(beta), yaw cos(beta)) with beta = q2+q3 the forearm's
angle from the vertical; the arm's own error-damping duty law[28]/[31] shown too."""
import sys, numpy as np
from scipy.signal import butter, filtfilt
from vt_common import Bag, yaw_deg
tag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]); dt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1
M, G = 3.746, 9.81; I = np.diag([0.130, 0.112, 0.116])   # coupled inertia at home (10-15 note)
LSB = 0.00269; NM2EFF = np.array([162.4, 154.0, 150.5, 153.4]); KT = 1.0/(NM2EFF*LSB); KPWM = np.array([169.47, 149.70, 135.25, 148.51])
b = Bag(tag); fs = 100.0; tu = np.arange(t0, t1, 1/fs)
def lp(x, fc=3.0):
    bb, a = butter(2, fc/(fs/2)); return filtfilt(bb, a, x, axis=0)
tm = b.t("mocap"); g = lambda k: np.interp(tu, tm, b.f("mocap", k))
p = np.c_[g("pose.position.x"), g("pose.position.y"), g("pose.position.z")]
qw, qx, qy, qz = (g(f"pose.orientation.{c}") for c in "wxyz")
n = np.sqrt(qw**2+qx**2+qy**2+qz**2); qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
# rotation matrices
R = np.empty((len(tu), 3, 3))
R[:,0,0] = 1-2*(qy**2+qz**2); R[:,0,1] = 2*(qx*qy-qz*qw); R[:,0,2] = 2*(qx*qz+qy*qw)
R[:,1,0] = 2*(qx*qy+qz*qw); R[:,1,1] = 1-2*(qx**2+qz**2); R[:,1,2] = 2*(qy*qz-qx*qw)
R[:,2,0] = 2*(qx*qz-qy*qw); R[:,2,1] = 2*(qy*qz+qx*qw); R[:,2,2] = 1-2*(qx**2+qy**2)
pf = lp(p); v = np.gradient(pf, 1/fs, axis=0); a = np.gradient(v, 1/fs, axis=0)
# body rates from R: omega_hat = R^T Rdot
Rf = lp(R.reshape(len(tu), 9)).reshape(len(tu), 3, 3); Rd = np.gradient(Rf, 1/fs, axis=0)
om = np.array([[ (Rf[k].T @ Rd[k])[2,1], (Rf[k].T @ Rd[k])[0,2], (Rf[k].T @ Rd[k])[1,0]] for k in range(len(tu))])
omd = np.gradient(lp(om, 3.0), 1/fs, axis=0)
tw = b.t("wb"); W = b.f("wb","data"); ok = np.array([len(r) >= 120 for r in W]); Wm = np.array([r[:120] for r in W[ok]]); tw = tw[ok]
wi = lambda k: np.interp(tu, tw, Wm[:, k])
u1 = wi(17); tau_cmd = np.c_[wi(21), wi(22), wi(23)]
thrust_w = np.einsum('kij,j->ki', R, np.array([0,0,1.0])) * u1[:,None]
F_res = M*a - thrust_w + np.array([0,0,M*G])
tau_res = (I @ omd.T).T + np.cross(om, (I @ om.T).T) - tau_cmd
# arm: applied e.m. torque vs the streamed law torque (arm convention), un-modelled = applied - law
tj = b.t("js"); names = [x.strip(" '[]") for x in str(b.f("js","name")[0]).split(",")]
ii = [names.index(f"joint{k}") for k in (1,2,3,4)]
E = b.f("js","effort")[:, ii]; P = b.f("js","position")[:, ii]
em = np.c_[[np.interp(tu, tj, E[:, j]) for j in range(4)]].T * LSB * KT
q = np.c_[[np.interp(tu, tj, P[:, j]) for j in range(4)]].T
tl = b.t("law"); LD = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in b.f("law","data")])
law = np.c_[[np.interp(tu, tl, LD[:, 9+j]) for j in range(4)]].T
ed = np.c_[[np.interp(tu, tl, LD[:, 28+j]) for j in range(4)]].T / KPWM
un = lp(em - law, 3.0)
beta = q[:,1] + q[:,2]   # forearm from the vertical (arm convention; both +)
# base-axis projection (arm convention signs: j1/j4 flipped vs the model; magnitudes are what matter here)
yaw_arm = un[:,0] + un[:,3]*np.cos(beta); roll_arm = un[:,3]*np.sin(beta); pitch_arm = un[:,1] + un[:,2]
print(f"== {tag}: residual force (world) and torque (FLU) the law did not command; arm un-modelled torque (applied - law) projected")
print("  t   | Fres_x Fres_y Fres_z [N] | tres_roll tres_pitch tres_yaw [N.m] | tau_cmd r p y | arm un-mod j1 j2 j3 j4 [N.m] -> roll pitch yaw | errdamp j1 j4 | beta")
for t in np.arange(t0+0.3, t1-0.3, dt):
    k = int(round((t-t0)*fs))
    print(f"{t:6.2f} | {F_res[k,0]:+6.2f} {F_res[k,1]:+6.2f} {F_res[k,2]:+6.2f} | {tau_res[k,0]:+7.3f} {tau_res[k,1]:+7.3f} {tau_res[k,2]:+7.3f} | {tau_cmd[k,0]:+5.2f} {tau_cmd[k,1]:+5.2f} {tau_cmd[k,2]:+5.2f} | "
          f"{un[k,0]:+6.3f} {un[k,1]:+6.3f} {un[k,2]:+6.3f} {un[k,3]:+6.3f} -> {roll_arm[k]:+6.3f} {pitch_arm[k]:+6.3f} {yaw_arm[k]:+6.3f} | {ed[k,0]:+6.3f} {ed[k,3]:+6.3f} | {np.degrees(beta[k]):4.0f}")
