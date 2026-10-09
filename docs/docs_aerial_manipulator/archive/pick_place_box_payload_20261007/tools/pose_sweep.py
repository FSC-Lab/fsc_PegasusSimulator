import os, sys, numpy as np
REPO = "/home/shiqi/fsc_PegasusSimulator"
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_planner import compatible_trajectory as CT
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C
D = np.radians
P = TP.make_params_t650(armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
N = P["n"]; LCHAR = 0.5 * sum(np.linalg.norm(l) for l in P["l_i"])
g = 9.81; MPAY = 0.200; OFF = 0.22   # grasp point -> box CG, straight down at the grasp

def chain(q):
    R = [np.eye(3)]
    for i in range(N): R.append(R[i] @ CT._rot(P["h_i_im1"][i], q[i]))
    O = [np.zeros(3)]
    for i in range(1, N + 1): O.append(O[i-1] + R[i] @ P["l_i"][i-1])
    return R, O

def sig_law(q):
    X = np.zeros(18 + 2 * N); X[3:12] = np.eye(3).flatten(order="F"); X[12:12+N] = q
    J = C.dynamics(X, P)["J_3y"].copy(); J[0:3] /= LCHAR
    return float(np.linalg.svd(J, compute_uv=False)[-1])

def g_arm(q):
    X = np.zeros(18 + 2 * N); X[3:12] = np.eye(3).flatten(order="F"); X[12:12+N] = q
    return np.asarray(C.dynamics(X, P)["g"]).ravel()[6:6+N]

def payload_tau(q, q_grasp):
    Rg, Og = chain(q_grasp)
    c_e = Rg[N].T @ np.array([0, 0, -OFF])          # box CG in the claw frame (rigid since the grasp)
    def V(qq):
        R, O = chain(qq); return MPAY * g * (O[N] + R[N] @ c_e)[2]
    e = 1e-6
    return np.array([(V(q + e*np.eye(N)[j]) - V(q - e*np.eye(N)[j])) / (2*e) for j in range(N)])

def row(qd, q_grasp_d=None):
    q = D(qd); qg = D(q_grasp_d if q_grasp_d is not None else qd)
    _, r0e = CT._arm_kin(q, P)
    R, O = chain(q)
    claw_ax = R[N] @ np.array([0, 0, 1.0])
    reach = np.hypot(r0e[0], r0e[1])
    tg = g_arm(q); tp = payload_tau(q, qg)
    return dict(q=qd, beta=qd[1]+qd[2], reach=reach, r0e=r0e, sk=TP._sigma_nd(q, P), sl=sig_law(q),
                tau0=tg, tau1=tg + tp)

if __name__ == "__main__":
    print("q2 limit -20..45 (user). servo caps hw: j2 2.44, j3 1.42 N.m. tau sign = model g (hold torque magnitude)")
    print(f"{'pose':22s} beta  reach_mm  r0e_z_mm  sig_kin sig_law  |tau2| |tau3| no-pay  |tau2| |tau3| 200g")
    poses = [[0,-20,30,0],[0,-30,40,0],[0,40,40,0]]
    for q2 in (-15,-10,-5,0,5,10):
        for b in (0,5,10,15,20):
            poses.append([0,q2,b-q2,0])
    for p in poses:
        r = row(p)
        print(f"{str(p):22s} {r['beta']:4.0f}  {r['reach']*1e3:7.1f}  {r['r0e'][2]*1e3:8.1f}  {r['sk']:.3f}   {r['sl']:.3f}    "
              f"{abs(r['tau0'][1]):.2f}  {abs(r['tau0'][2]):.2f}         {abs(r['tau1'][1]):.2f}  {abs(r['tau1'][2]):.2f}")
