"""Which factor of Kq = omega_y^2 * M_rho limits the arm's stiffness, and how much of M_rho's soft direction comes
from the way the model represents the servos' reflected rotor inertia (J_arm = 353.5^2 * 1.6e-7 kg m^2).

Model as flown: J_arm * h h^T is added to each CHILD LINK's body inertia (controller.make_params / wb_model.cpp), so the
rotor of joint i is treated as turning with the absolute link rate (for the parallel j2/j3 axes: q2dot + q3dot).
Physical reflected inertia: the rotor spins at N*qdot_i relative to its housing, kinetic energy 1/2 * N^2 I_r qdot_i^2,
i.e. a DIAGONAL J_arm on each joint coordinate (cross terms ~ I_r*N = 5.7e-5, negligible)."""
import sys, copy
import numpy as np
sys.path.insert(0, "/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_controller.controller import dynamics
np.set_printoptions(precision=4, suppress=True, linewidth=150)
P = TP.make_params_t650(); n = P["n"]; JA = 353.5**2 * 1.6e-7
PL = copy.deepcopy(P)
for i in range(n):
    h = np.asarray(P["h_i_im1"][i], float); PL["I_i_i"][i + 1] = np.asarray(P["I_i_i"][i + 1]) - JA * np.outer(h, h)
BRK = np.array([0.215, 0.106])            # ground-calibrated breakaway at the flown loads, j2 / j3 [N.m]


def X_at(qdeg):
    X = np.zeros(18 + 2 * n); X[3:12] = np.eye(3).reshape(-1, order="F"); X[12:12 + n] = np.radians(qdeg); return X


for qdeg in ([0, 25, 30, 0], [0, 30, 30, 0], [0, 40, 40, 0]):
    X = X_at(qdeg); dA = dynamics(X, P); dL = dynamics(X, PL)
    Mq = {"as modelled (J_arm on the child link body)": dA["M_tilde"][6:, 6:],
          "links only (J_arm = 0)": dL["M_tilde"][6:, 6:],
          "links + physical reflected inertia (J_arm on each joint, diagonal)": dL["M_tilde"][6:, 6:] + JA * np.eye(n)}
    print(f"\n==== q = {qdeg} deg")
    for k, M in Mq.items():
        B = M[1:3, 1:3]; w, V = np.linalg.eigh(B); v = V[:, 0] * np.sign(V[0, 0])
        K = 20.0 * B
        worst = max(np.abs(np.degrees(np.linalg.solve(K, BRK * np.array([1, s])))).max() for s in (1, -1))
        print(f"{k:66s} j2/j3 block [[{B[0,0]:.4f},{B[0,1]:.4f}],[{B[1,0]:.4f},{B[1,1]:.4f}]]  eig {w[0]:.4f}/{w[1]:.4f}"
              f"  soft dir ({v[0]:+.2f},{v[1]:+.2f})  -> Kq at omega^2=20: soft {20*w[0]:.2f}, stiff {20*w[1]:.2f} N.m/rad, worst friction error {worst:.1f} deg")
    # side effect on the base: locked-arm rotational inertia (generalised M, base-rotation block)
    MA = dA["M"][3:6, 3:6]; ML = dL["M"][3:6, 3:6]
    print("locked-arm base rotational inertia diag, as modelled vs links only [kg m^2]:", np.diag(MA), np.diag(ML), " difference", np.diag(MA - ML))
