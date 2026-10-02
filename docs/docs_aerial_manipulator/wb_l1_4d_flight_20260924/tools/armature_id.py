"""Identify the arm's reflected rotor inertia (armature) from the four 0924 flights.

    PYTHONNOUSERSITE=1 /usr/bin/python3 armature_id.py <dir with a1..a4.npz>

The model (controller.make_params, wb_model.cpp) adds J_arm = 353.5^2 * 1.6e-7 = 0.020 kg m^2
as J_arm*h h^T to each CHILD LINK's body inertia. For the parallel j2/j3 axes that makes the
armature torque  tau2 = J(2 qdd2 + qdd3), tau3 = J(qdd2 + qdd3); a joint-diagonal armature
(the rotor spins at N*qdot relative to its housing) gives tau2 = J qdd2, tau3 = J qdd3.

For every joint-state sample (250 Hz, header-stamped, one read = position + Present Current):
    y = tau_applied - tau_links
    tau_applied = Present Current / kappa_tau                 (the arm's calibrated counts/N.m)
    tau_links   = M_qq qdd + M_qw wdot + S * M_qv f_b + C V   (links only, armature REMOVED)
with f_b the accelerometer's specific force in the model base frame (it carries gravity AND the
base's linear acceleration) and S the arm calibration's gravity scale. What is left is
armature + gearbox friction, fitted on the MOVING samples as
    y_j = sum_k A_jk qdd_k + fc_j sgn(qdot_j) + mu_j sgn(qdot_j)|tau_g,j| + b_j qdot_j + c_j
(A = 2x2 armature matrix for j2/j3). Every signal goes through the same zero-phase low-pass, so
the regression is not biased by filtering; q, qdot, qdd come from the ENCODER (4096 counts/rev),
not the velocity observer, so there is no observer delay in the regressor.
"""
import copy, os, sys
import numpy as np
from scipy.signal import butter, filtfilt

sys.path.insert(0, "/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT

DATA = sys.argv[1]
P = TP.make_params_t650(); n = P["n"]
JA = 353.5 ** 2 * 1.6e-7
PL = copy.deepcopy(P)                                   # links only: armature removed
for i in range(n):
    h = np.asarray(P["h_i_im1"][i], float)
    PL["I_i_i"][i + 1] = np.asarray(P["I_i_i"][i + 1]) - JA * np.outer(h, h)
KT = np.array([162.4, 154.0, 150.5, 153.4])            # Present Current counts per N.m (arm calibration)
S_G = np.array([1.0, 0.897, 0.969, 1.0])                # gravity scale from the arm calibration
JS = [2, 3, 1, 4]; IDX = [JS.index(j) for j in (1, 2, 3, 4)]   # broadcaster order -> model order
R_FRD_FLU = np.diag([1.0, -1.0, -1.0])
RZm = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])    # Rz(-90): model = actual * Rz(-90)
B_ACT2MODEL = RZm.T @ R_FRD_FLU                          # FRD body vector -> model base frame
FLIGHTS = [("a1", "F1 fold 60, 48 s"), ("a2", "F2 fold 60, 48 s"),
           ("a3", "F3 fold 55, 12 s"), ("a4", "F4 fold 55, 6 s")]


def direct_window(d):
    t0 = d["wb__recv"][0]; tmo = d["wbmode__recv"]; mo = d["wbmode__data"]
    ed, last = [], None
    for ti, vi in zip(tmo, mo):
        if vi != last:
            ed.append((ti, vi)); last = vi
    st = [b for _, b in ed]; i = st.index("DIRECT")
    return ed[i][0], (ed[i + 1][0] if i + 1 < len(ed) else tmo[-1])


def lp(x, fc, fs=250.0):
    b, a = butter(4, fc / (fs / 2))
    return filtfilt(b, a, x, axis=0)


def prepare(nm, fc):
    d = np.load(os.path.join(DATA, nm + ".npz"), allow_pickle=True)
    a, b = direct_window(d)
    th = d["js__hdr"]; tr = d["js__recv"]; off = np.median(tr - th)   # header clock -> recorder clock
    m = (tr > a + 1.0) & (tr < b - 0.5)
    tg = np.arange(th[m][0], th[m][-1], 0.004)                        # uniform 250 Hz on the header clock
    q = np.column_stack([np.interp(tg, th, d["js__position"][:, IDX[k]]) for k in range(n)])
    cur = np.column_stack([np.interp(tg, th, d["js__effort"][:, IDX[k]]) for k in range(n)])
    tau = cur / KT
    trec = tg + off
    ts = d["sc__recv"]
    acc = B_ACT2MODEL @ np.asarray(d["sc__accelerometer_m_s2"], float).T
    gyr = B_ACT2MODEL @ np.asarray(d["sc__gyro_rad"], float).T
    f_b = np.column_stack([np.interp(trec, ts, acc[k]) for k in range(3)])
    w_b = np.column_stack([np.interp(trec, ts, gyr[k]) for k in range(3)])
    qf = lp(q, fc); qd = np.gradient(qf, 0.004, axis=0); qdd = np.gradient(qd, 0.004, axis=0)
    tauf = lp(tau, fc); f_bf = lp(f_b, fc); w_bf = lp(w_b, fc); wd = np.gradient(w_bf, 0.004, axis=0)
    # links-only torque on the arm rows, every sample
    tl = np.zeros_like(q); tg_ = np.zeros_like(q)
    X = np.zeros(18 + 2 * n); X[3:12] = np.eye(3).reshape(9, order="F")
    for i in range(len(tg)):
        X[12:12 + n] = qf[i]; X[15 + n:18 + n] = w_bf[i]; X[18 + n:18 + 2 * n] = qd[i]
        dy = CT.dynamics(X, PL); M = dy["M"]; Cm = dy["C"]
        V = np.concatenate([np.zeros(3), w_bf[i], qd[i]])
        grav = M[6:, 0:3] @ f_bf[i]                  # specific force: gravity + base linear accel
        tg_[i] = S_G * grav
        tl[i] = M[6:, 6:] @ qdd[i] + M[6:, 3:6] @ wd[i] + tg_[i] + (Cm @ V)[6:]
    return dict(t=tg - tg[0], q=qf, qd=qd, qdd=qdd, tau=tauf, tl=tl, tg=tg_, y=tauf - tl,
                qdeg=np.degrees(q))


def fit(D, j, cols_q=(1, 2), vmin_deg=3.0, delay=0):
    """y_j = sum_k A_k qdd_k + fc sgn + mu sgn|tau_g| + b qdot + c on the moving samples."""
    y = np.roll(D["y"][:, j], -delay) if delay else D["y"][:, j]
    qd = D["qd"][:, j]; mv = (np.abs(qd) > np.radians(vmin_deg)) & (D["qdeg"][:, j] < 49.0)
    mv[:20] = mv[-20:] = False
    sg = np.sign(qd)
    cols = [D["qdd"][:, k] for k in cols_q] + [sg, sg * np.abs(D["tg"][:, j]), qd, np.ones_like(qd)]
    A = np.column_stack(cols)[mv]; yy = y[mv]
    c, *_ = np.linalg.lstsq(A, yy, rcond=None)
    r = yy - A @ c
    # standard errors (white-residual approximation, inflated by the low-pass: N_eff ~ N * fc/125)
    N = mv.sum(); neff = max(N * FC_NOW / 125.0, 10)
    s2 = r @ r / max(neff - A.shape[1], 1)
    se = np.sqrt(np.diag(s2 * np.linalg.pinv(A.T @ A)) * N / neff)
    r2 = 1 - r.var() / yy.var()
    return c, se, r2, N, r.std()


if __name__ == "__main__":
    res = {}
    for FC_NOW in (8.0, 12.0, 20.0):
        print(f"\n================ zero-phase low-pass {FC_NOW:.0f} Hz ================")
        for nm, lab in FLIGHTS:
            D = prepare(nm, FC_NOW)
            print(f"\n--- {lab} ({nm}): {len(D['t'])} samples, DIRECT, "
                  f"|qdd| p95 j2/j3 {np.percentile(np.abs(D['qdd'][:, 1]), 95):.2f}/"
                  f"{np.percentile(np.abs(D['qdd'][:, 2]), 95):.2f} rad/s^2")
            for j in (1, 2):
                cd, sd, r2d, N, sdev = fit(D, j, cols_q=(j,))
                cf, sf, r2f, _, _ = fit(D, j, cols_q=(1, 2))
                # residual std with the armature FIXED at each hypothesis (friction refitted)
                hyp = {}
                for name, Arow in (("none", (0, 0)), ("diag 0.020", (JA if j == 1 else 0, JA if j == 2 else 0)),
                                   ("model hh^T 0.020", (2 * JA, JA) if j == 1 else (JA, JA))):
                    DD = dict(D); DD["y"] = D["y"].copy()
                    DD["y"][:, j] = D["y"][:, j] - Arow[0] * D["qdd"][:, 1] - Arow[1] * D["qdd"][:, 2]
                    _, _, _, _, sh = fit(DD, j, cols_q=())
                    hyp[name] = sh
                print(f"  j{j+1}: DIAG fit J = {cd[0]:.4f} +/- {sd[0]:.4f} kg m^2 (R2 {r2d:.3f}, N {N}) | "
                      f"2x2 row: A_{j+1}2 = {cf[0]:+.4f}+/-{sf[0]:.4f}, A_{j+1}3 = {cf[1]:+.4f}+/-{sf[1]:.4f} (R2 {r2f:.3f}) | "
                      f"fc {cd[1]:.3f} mu {cd[2]:.3f} b {cd[3]:.3f} | resid std w/ J fixed: " +
                      ", ".join(f"{k} {v*1e3:.1f}" for k, v in hyp.items()) + " mN.m")
                res.setdefault((FC_NOW, j), []).append((nm, cd[0], sd[0], cf[0], cf[1]))
            if FC_NOW == 12.0:
                best = []
                for dl in range(-5, 11):
                    c, _, r2, _, _ = fit(D, 1, cols_q=(1,), delay=dl)
                    c3, _, r23, _, _ = fit(D, 2, cols_q=(2,), delay=dl)
                    best.append((dl * 4, c[0], r2, c3[0], r23))
                b2 = max(best, key=lambda x: x[2]); b3 = max(best, key=lambda x: x[4])
                print(f"  current-vs-encoder delay scan (-20..+40 ms): best R2 j2 at {b2[0]:+d} ms (J {b2[1]:.4f}), "
                      f"j3 at {b3[0]:+d} ms (J {b3[3]:.4f})")
    print("\n================ summary: diagonal J per joint [kg m^2], per flight ================")
    for (fc, j), rows in sorted(res.items()):
        vals = np.array([r[1] for r in rows])
        print(f"  {fc:4.0f} Hz j{j+1}: " + "  ".join(f"{r[0]} {r[1]:.4f}" for r in rows) +
              f"   mean {vals.mean():.4f}")
