"""Parametrisation of the modular law used by the tuner + a few hand starts."""
import copy, numpy as np
from fsc_aerial_manipulation.robotic_arm.utils_controller import modular_adaptive as MA

MBAR_ARM = np.array([0.022, 0.033, 0.016, 0.010])   # nominal joint inertia at home (Remark 3's choice)
MBAR_ATT = 0.095                                    # nominal body inertia, all axes
M_NOM = 3.746170

def make(p):
    """p: flat dict -> ModularConfig. Lambda folded into K1 = kp, K2 = kd (choice 1 with Lambda = I)."""
    one3 = np.ones(3); one4 = np.ones(4)
    pos = MA.ModuleGains(mbar=M_NOM * one3,
                         lam1=np.array([p["p_kp_xy"], p["p_kp_xy"], p["p_kp_z"]]),
                         lam2=np.array([p["p_kd_xy"], p["p_kd_xy"], p["p_kd_z"]]), Lam=one3,
                         q_e=p["p_q"], q_v=p["p_q"] * p.get("p_qv", 1.0), nu=np.array([p.get("p_nu0", p["p_nu"])] + [p.get("p_nu123", p["p_nu"])] * 3), eps=1e-4, varpi=p["p_varpi"],
                         K_init=p.get("p_k0", 0.01), zeta_init=0.1)
    att = MA.ModuleGains(mbar=MBAR_ATT * p.get("q_mbar_s", 1.0) * one3,
                         lam1=np.array([p["q_kp_rp"], p["q_kp_rp"], p["q_kp_y"]]),
                         lam2=np.array([p["q_kd_rp"], p["q_kd_rp"], p["q_kd_y"]]), Lam=one3,
                         q_e=p["q_q"], q_v=p["q_q"] * p.get("q_qv", 1.0), nu=np.array([p.get("q_nu0", p["q_nu"])] + [p.get("q_nu123", p["q_nu"])] * 3), eps=1e-4, varpi=p["q_varpi"],
                         K_init=p.get("q_k0", 0.001), zeta_init=0.01)
    arm = MA.ModuleGains(mbar=MBAR_ARM * p.get("a_mbar_s", 1.0), lam1=p["a_kp"] * one4, lam2=p["a_kd"] * one4,
                         Lam=one4, q_e=p["a_q"], q_v=p["a_q"] * p.get("a_qv", 1.0), nu=np.array([p.get("a_nu0", p["a_nu"])] + [p.get("a_nu123", p["a_nu"])] * 3), eps=1e-4,
                         varpi=p["a_varpi"], K_init=p.get("a_k0", 1e-4), zeta_init=0.01)
    return MA.ModularConfig(pos=pos, att=att, arm=arm, m_g=M_NOM, acc_filter_hz=p.get("acc_hz", 10.0))

# Table II's POLES (Lambda*lambda1, Lambda*lambda2) and adaptive constants, with Mbar = the
# nominal inertia instead of the S-500's numbers.
TABLE2_MAPPED = dict(p_kp_xy=3.0, p_kd_xy=1.5, p_kp_z=8.0, p_kd_z=4.0, p_q=1.0, p_nu=10.0, p_varpi=0.1,
                     q_kp_rp=14.0, q_kd_rp=7.0, q_kp_y=10.0, q_kd_y=5.0, q_q=1.0, q_nu=20.0, q_varpi=1.0,
                     a_kp=3.0, a_kd=1.5, a_q=1.0, a_nu=1.0, a_varpi=0.1)
