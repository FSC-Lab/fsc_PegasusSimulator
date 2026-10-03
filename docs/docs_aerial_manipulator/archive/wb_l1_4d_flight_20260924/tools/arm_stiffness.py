"""Joint-space stiffness that the whole-body law's END-EFFECTOR impedance actually produces, and the task gains a
joint-space stiffness would need. Pure model, no bag: the T650 whole-body model the flight law runs
(transition_planner.make_params_t650 + controller.dynamics, parity-locked to the C++ law).

The law (wb_controller.cpp / controller.py):
    u3 = - J3b^T (J3b J3b^T + lambda^2 I)^-1 [ ... + Lambda_y M_y^-1 (D_y e_vy + K_y e_y) ... ]
with J3b = the ARM block of the dynamically-consistent Jacobian (J_y^#)^T = Lambda_y J_y Mtilde^-1, i.e.
J3b = Lambda_y J_rho M_rho^-1. Linearising about a static hold (e_y = J_rho dq, J_rho = d y / d q at fixed CoM and
base attitude = dyn["J_y"][:, 6:]):
    dtau = - Kq dq,   Kq = J3b^T (J3b J3b^T + lambda^2 I)^-1 Lambda_y M_y^-1 K_y J_rho
and with lambda = 0 this is exactly  Kq = M_rho J_rho^-1 (M_y^-1 K_y) J_rho : an ACCELERATION-level impedance, whose
joint stiffness is the model arm inertia times the task bandwidth squared -- NOT J^T K_y J.
"""
import sys
import numpy as np
sys.path.insert(0, "/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_controller.controller import dynamics

np.set_printoptions(precision=4, suppress=True, linewidth=160)
P = TP.make_params_t650(); n = P["n"]
KY = np.diag([20.0, 20.0, 20.0, 0.3]); DY = np.diag([12.0, 12.0, 12.0, 0.3]); MY = np.diag([1.0, 1.0, 1.0, 0.05]); LAM = 0.3
# joint-space references: the deleted posture PID, and the ground PD+ sets flown on this arm (N.m/rad)
TARGETS = {"deleted posture PID": [2.0, 2.0, 2.0, 2.0],
           "ground PD+ current mode (observer kd)": [4.0, 11.25, 15.75, 7.5],
           "ground PD+ PWM mode (motor back-EMF damping kept)": [14.67, 101.5, 116.5, 31.4]}
BREAKAWAY = np.array([0.0, 0.031 + 0.246 * 0.75, 0.0575 + 0.161 * 0.30, 0.0])   # ground fc + mu|tau| at the flown loads


def dyn_at(qdeg):
    X = np.zeros(18 + 2 * n); X[3:12] = np.eye(3).reshape(-1, order="F"); X[12:12 + n] = np.radians(qdeg)
    return dynamics(X, P)


def law_gain(d, K, M=MY, lam=LAM):
    J3b = d["J_3y"]; G = J3b.T @ np.linalg.solve(J3b @ J3b.T + lam**2 * np.eye(4), d["Lambda_y"] @ np.linalg.inv(M))
    return G @ K @ d["J_y"][:, 6:], G


for lab, qdeg in [("F1/F2 circle centre, fold 60", [0, 30, 30, 0]), ("F3/F4 circle centre, fold 55", [0, 25, 30, 0]),
                  ("home [0,40,40,0]", [0, 40, 40, 0])]:
    d = dyn_at(qdeg); J = d["J_y"][:, 6:]; Mr = d["M_tilde"][6:, 6:]
    Kq, G = law_gain(d, KY); Kq0, _ = law_gain(d, KY, lam=0.0); Dq, _ = law_gain(d, DY)
    naive = J.T @ KY @ J
    print(f"\n==== {lab}  q = {qdeg} deg")
    print("model arm inertia M_rho diag [kg m^2]         ", np.diag(Mr))
    print("lever |dr_e/dq_j| at fixed CoM [m/rad]        ", np.linalg.norm(J[:3], axis=0))
    print("law Kq diag, K_y 20 / M_y 1, lambda 0.3        ", np.diag(Kq), " N.m/rad")
    print("law Kq diag, lambda 0 (= M_rho J^-1 M_y^-1 K_y J)", np.diag(Kq0))
    print("law Dq diag (D_y 12)                          ", np.diag(Dq), " N.m.s/rad")
    print("naive J^T K_y J diag (what the report used)    ", np.diag(naive))
    print("breakaway error = tau_break / Kq  [deg]        ", np.round(np.degrees(BREAKAWAY / np.maximum(np.diag(Kq), 1e-9)), 1))
    print("full law Kq [N.m/rad]\n", Kq)
    # per unit of the three position gains (heading row fixed): Kq is linear in K_y
    Kpos, _ = law_gain(d, np.diag([1.0, 1.0, 1.0, 0.0])); Kpsi, _ = law_gain(d, np.diag([0, 0, 0, 0.3]))
    for name, tq in TARGETS.items():
        tq = np.array(tq)
        k_iso = max((tq[j] - Kpsi[j, j]) / Kpos[j, j] for j in (1, 2))      # isotropic position gain to reach j2 AND j3
        Kfull = np.linalg.solve(G, np.diag(tq)) @ np.linalg.inv(J)          # exact task-space matrix giving Kq = diag(tq)
        print(f"  target {name} {tq[1]:.2f}/{tq[2]:.2f} (j2/j3): isotropic K_y = {k_iso:7.1f} N/m at M_y 1"
              f" -> task bandwidth sqrt(K_y/M_y) = {np.sqrt(k_iso):5.1f} rad/s ({np.sqrt(k_iso)/2/np.pi:.2f} Hz);"
              f" exact matrix diag {np.round(np.diag(Kfull),1)}, max |offdiag| {np.abs(Kfull - np.diag(np.diag(Kfull))).max():.1f}")
    # how the task channel sees the BASE: EE-minus-CoM motion per radian of base rotation vs per radian of joint rotation
    Jr = d["J_y"][:3, 3:6]
    print("EE motion per rad of base roll/pitch/yaw [m]   ", np.linalg.norm(Jr, axis=0), " vs per rad of j2/j3", np.linalg.norm(J[:3, 1:3], axis=0))
    print("task force per 1 deg base tilt at K_y 20 [N]   ", np.round(20 * np.linalg.norm(Jr[:, :2], axis=0) * np.pi / 180, 3))


# ---- the j2/j3 block: principal stiffnesses and what friction does along the soft direction ----
print("\n==== j2/j3 block of the law's stiffness (F3/F4 centre), per unit task bandwidth squared")
d = dyn_at([0, 25, 30, 0]); J = d["J_y"][:, 6:]
Kq, _ = law_gain(d, KY); K23 = 0.5 * (Kq[1:3, 1:3] + Kq[1:3, 1:3].T)
w, V = np.linalg.eigh(K23)
for k in range(2):
    v = V[:, k] * np.sign(V[0, k]); ee = np.linalg.norm(J[:3, 1:3] @ v)
    print(f"principal stiffness {w[k]:.3f} N.m/rad along (dq2, dq3) = ({v[0]:+.2f}, {v[1]:+.2f}); EE moves {1000*ee:.0f} mm per rad along it")
for s3 in (+1, -1):
    tau = np.array([BREAKAWAY[1], s3 * BREAKAWAY[2]]); e = np.linalg.solve(K23, tau)
    print(f"friction held at breakaway, tau = ({tau[0]:+.3f}, {tau[1]:+.3f}) N.m -> static joint error (q2, q3) = ({np.degrees(e[0]):+.1f}, {np.degrees(e[1]):+.1f}) deg")
for kname, Ky in [("flown 20/1 (4.5 rad/s)", 20), ("sim-flown 50/1 (7.1 rad/s)", 50), ("rotor-pole 100/1 (10 rad/s)", 100), ("in-process-only 200/1 (14.1 rad/s)", 200)]:
    Kx, _ = law_gain(d, np.diag([Ky, Ky, Ky, 0.3])); B = 0.5 * (Kx[1:3, 1:3] + Kx[1:3, 1:3].T); ww = np.linalg.eigvalsh(B)
    worst = max(np.abs(np.degrees(np.linalg.solve(B, np.array([BREAKAWAY[1], s * BREAKAWAY[2]])))).max() for s in (1, -1))
    print(f"K_y {kname:34s}: Kq diag j2/j3 {Kx[1,1]:.2f}/{Kx[2,2]:.2f}, principal {ww[0]:.2f}/{ww[1]:.2f} N.m/rad, worst static friction error {worst:.1f} deg")
# isotropic K_y that makes the SOFT direction as stiff as a joint term of k_j
soft_per_unit = w[0] / 20.0
for name, tq in TARGETS.items():
    print(f"to make the SOFTEST j2/j3 direction match {name} ({min(tq[1], tq[2]):.2f} N.m/rad): K_y = {min(tq[1], tq[2]) / soft_per_unit:.0f} N/m at M_y 1"
          f" (bandwidth {np.sqrt(min(tq[1], tq[2]) / soft_per_unit):.1f} rad/s)")
