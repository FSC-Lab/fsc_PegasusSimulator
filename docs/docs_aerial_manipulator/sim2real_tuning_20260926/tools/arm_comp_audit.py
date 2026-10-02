#!/usr/bin/env python3
"""Audit the ARM-CHANNEL compensation terms against the 0918 / 0924 flights.

What the real arm was given in DIRECT, per joint, every tick:
    tau_intended = tau_law (whole-body u3, streamed)
                 + (S - 1) g_urdf(q)            passthrough_gravity_correction
                 + [fc + mu |tau|] tanh(qd_REF / w)   passthrough friction FF
and what it delivered, tau_applied (Present Current x Kt). The plant then
needed, to move as it did:
    tau_applied = S_true g(q, R0) + J q_ddot + coupling + FRICTION(qd, load) + ...

So the residual  r = tau_applied - S g_model(q, R0) - J_diag q_ddot  is the
friction (+ the model error) the arm actually saw, on the MEASURED velocity.
This script fits it per joint and per flight -- Coulomb level, load
coefficient, viscous slope, static offset at holds, breakaway -- and puts it
beside (a) the feed-forward the arm controller applied (which acts on the
REFERENCE velocity, so it is exactly zero at every hold and during every
stick), and (b) what the whole-body observer's arm channel (d_hat_q) and the
u3 internal feedforward carried. Then it derives corrected coefficients.

    /usr/bin/python3 arm_comp_audit.py <data dir>   -> analysis/arm_comp_audit.{json,txt}
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, "/home/shiqi/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation")
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as CT  # noqa: E402

KT = np.array([162.4, 154.0, 150.5, 153.4])          # current counts per N.m
KPWM = np.array([169.47, 149.70, 135.25, 148.51])    # duty counts per N.m
SIGN = np.array([-1.0, 1.0, 1.0, -1.0])              # model <-> hardware joint sign
S_G = np.array([1.0, 0.897, 0.969, 1.0])             # gravity_scale as flown
FC = np.array([0.01711, 0.03143, 0.05751, 0.05237])  # friction_ff [N.m] as flown (duty / kappa_pwm)
MU = np.array([0.0, 0.246, 0.161, 0.0])              # friction_load_coeff as flown
W_FF = 0.015                                         # friction_ff_width [rad/s]
J_ARM = np.array([0.010, 0.0194, 0.0097, 0.0097])    # calibrated joint-diagonal armature (2026-09-26)
JS_ORDER = [2, 3, 1, 4]
R_NED_ENU = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
R_FRD_FLU = np.diag([1, -1, -1.0])
RZm = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.0]])   # Rz(-90): actual -> model
FLIGHTS = [("f18", "0918 mission"), ("a1", "0924 F1"), ("a2", "0924 F2"), ("a3", "0924 F3 (12 s)"), ("a4", "0924 F4 (6 s)")]

P = TP.make_params_t650(armature_diag=J_ARM)
N = P["n"]


def R_of(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def g_joint(q_model, Rm):
    X = np.zeros(18 + 2 * N)
    X[3:12] = Rm.reshape(9, order="F")
    X[12:12 + N] = q_model
    return CT.dynamics(X, P)["g"][6:6 + N]


def lp(x, k):
    return np.column_stack([np.convolve(x[:, j], np.ones(k) / k, "same") for j in range(x.shape[1])])


def analyse(nm, path):
    d = np.load(path, allow_pickle=True)
    t0 = d["wb__recv"][0]
    t = d["wb__recv"] - t0
    D = d["wb__data"]
    direct = D[:, 0] > 0.5
    ta, tb = t[direct][0], t[direct][-1]
    names = list(d["js__name"][0])
    order = [names.index(f"joint{k}") for k in (1, 2, 3, 4)]
    tj = d["js__recv"] - t0
    q_hw = d["js__position"][:, order]
    tau_app = d["js__effort"][:, order] / KT
    tv = d["velobs__recv"] - t0
    vn = list(d["velobs__name"][0])
    vo = [vn.index(f"joint{k}") for k in (1, 2, 3, 4)]
    v_obs = d["velobs__velocity"][:, vo]
    tl = d["law__recv"] - t0
    L = d["law__data"]
    aux = L[:, 18:22] / KPWM if L.shape[1] > 21 else np.zeros((len(tl), 4))
    gcorr = L[:, 22:26] / KPWM if L.shape[1] > 25 else np.zeros((len(tl), 4))
    tc = d["tcmd__recv"] - t0
    tau_law = d["tcmd__effort"]
    tq = d["vatt__recv"] - t0
    Qv = d["vatt__q"]
    # 50 Hz grid over DIRECT
    tt = np.arange(ta + 0.5, tb - 0.5, 0.02)
    I = lambda tx, y: np.column_stack([np.interp(tt, tx, y[:, k]) for k in range(y.shape[1])])  # noqa: E731
    q = I(tj, q_hw)
    app = lp(I(tj, tau_app), 5)
    v = I(tv, v_obs)
    acc = lp(np.gradient(v, tt, axis=0), 5)
    law = I(tc, tau_law)
    ffr = I(tl, aux)
    gc = I(tl, gcorr)
    qd_ref = I(d["armref__recv"] - t0, d["armref__points[0].velocities"]) if "armref__points[0].velocities" in d.files else np.zeros_like(v)
    # model gravity on the MEASURED attitude, in the hardware convention
    g = np.zeros((len(tt), 4))
    for i, ti in enumerate(tt):
        qq = Qv[max(np.searchsorted(tq, ti) - 1, 0)]
        Rm = R_NED_ENU @ R_of(qq) @ R_FRD_FLU @ RZm
        g[i] = SIGN * g_joint(SIGN * q[i], Rm)
    # observer arm channel + u3 internal feedforward (model convention -> hw)
    dq = I(t, D[:, 37:41]) * SIGN
    u3_est = I(t, D[:, 93:97]) * SIGN if D.shape[1] > 96 else np.zeros_like(dq)
    intended = law + gc + ffr
    res = app - S_G * g - J_ARM * acc            # friction + unmodelled, measured velocity
    out = {"flight": nm, "direct": [float(ta), float(tb)], "joints": {}}
    for j in range(4):
        vv = v[:, j]
        mv = np.abs(vv) > np.radians(1.0)
        still = np.abs(vv) < np.radians(0.3)
        load = np.abs(S_G[j] * g[:, j])
        row = {"frac_moving": float(mv.mean()),
               "applied_minus_intended_rms": float(np.sqrt(np.mean((app[:, j] - intended[:, j]) ** 2))),
               "applied_minus_intended_mean": float(np.mean(app[:, j] - intended[:, j])),
               "ff_applied_rms": float(np.sqrt(np.mean(ffr[:, j] ** 2))),
               "ff_applied_when_moving_mean_abs": float(np.mean(np.abs(ffr[mv, j]))) if mv.any() else 0.0,
               "gravity_model_mean": float(np.mean(S_G[j] * g[:, j])),
               "static_residual_mean": float(res[still, j].mean()) if still.any() else np.nan,
               "static_residual_std": float(res[still, j].std()) if still.any() else np.nan,
               "static_residual_p90_abs": float(np.percentile(np.abs(res[still, j]), 90)) if still.any() else np.nan,
               "dhat_q_mean": float(dq[:, j].mean()), "dhat_q_std": float(dq[:, j].std()),
               "u3_est_mean": float(u3_est[:, j].mean()), "u3_est_rms": float(np.sqrt(np.mean(u3_est[:, j] ** 2)))}
        if mv.sum() > 100:
            s = np.sign(vv[mv])
            # r = fc*sign(v) + mu*load*sign(v) + c_v*v + b
            A = np.column_stack([s, s * load[mv], vv[mv], np.ones(mv.sum())])
            c, *_ = np.linalg.lstsq(A, res[mv, j], rcond=None)
            fit_full = {"fc": float(c[0]), "mu": float(c[1]), "viscous": float(c[2]), "bias": float(c[3])}
            # constrained: mu fixed at the report's value, fit fc + viscous + bias
            A2 = np.column_stack([s, vv[mv], np.ones(mv.sum())])
            c2, *_ = np.linalg.lstsq(A2, res[mv, j] - MU[j] * load[mv] * s, rcond=None)
            fit_mu_fixed = {"fc": float(c2[0]), "viscous": float(c2[1]), "bias": float(c2[2])}
            kin = res[mv, j] * s
            row.update({"n_moving": int(mv.sum()), "kinetic_level_median": float(np.median(kin)),
                        "kinetic_level_p25_p75": [float(np.percentile(kin, 25)), float(np.percentile(kin, 75))],
                        "ff_level_at_load_median": float(np.median(FC[j] + MU[j] * load[mv])),
                        "fit_full": fit_full, "fit_mu_fixed": fit_mu_fixed,
                        "v_abs_mean_deg_s": float(np.degrees(np.abs(vv[mv]).mean())),
                        "load_mean": float(load[mv].mean())})
            # breakaway: residual just before slip onsets
            st = np.abs(np.degrees(vv)) < 1.0
            onsets = []
            i = 15
            while i < len(tt) - 5:
                if st[i - 15:i].all() and (np.abs(np.degrees(vv[i:i + 5])) > 3.0).any():
                    onsets.append(i); i += 25
                else:
                    i += 1
            if onsets:
                br = np.array([res[i - 2, j] * np.sign(vv[i + 4]) for i in onsets])
                row["breakaway_n"] = len(onsets)
                row["breakaway_median"] = float(np.median(br))
        out["joints"][f"j{j+1}"] = row
    return out


def main():
    ddir = sys.argv[1]
    adir = os.path.join(ddir, "..", "analysis")
    os.makedirs(adir, exist_ok=True)
    allr = {}
    lines = []
    for nm, lab in FLIGHTS:
        p = os.path.join(ddir, f"{nm}.npz")
        if not os.path.exists(p):
            continue
        r = analyse(nm, p)
        allr[nm] = r
        lines.append(f"\n===== {lab} ({nm}) DIRECT {r['direct'][0]:.1f}..{r['direct'][1]:.1f} s")
        for j in range(4):
            w = r["joints"][f"j{j+1}"]
            lines.append(f"  j{j+1}: moving {100*w['frac_moving']:3.0f}% | applied-intended mean {w['applied_minus_intended_mean']:+.3f} rms {w['applied_minus_intended_rms']:.3f} N.m"
                         f" | FF applied rms {w['ff_applied_rms']:.3f} (|FF| when moving {w['ff_applied_when_moving_mean_abs']:.3f})"
                         f" | static residual (holds) mean {w['static_residual_mean']:+.3f} std {w['static_residual_std']:.3f} p90|.| {w['static_residual_p90_abs']:.3f}"
                         f" | d_hat_q mean {w['dhat_q_mean']:+.3f} std {w['dhat_q_std']:.3f} | u3_est rms {w['u3_est_rms']:.3f}")
            if "fit_full" in w:
                f, m = w["fit_full"], w["fit_mu_fixed"]
                lines.append(f"       kinetic level median {w['kinetic_level_median']:+.3f} (p25/75 {w['kinetic_level_p25_p75'][0]:+.3f}/{w['kinetic_level_p25_p75'][1]:+.3f}) vs FF level {w['ff_level_at_load_median']:.3f} at load {w['load_mean']:.2f} N.m, |v| {w['v_abs_mean_deg_s']:.1f} deg/s, n {w['n_moving']}"
                             f" | fit fc {f['fc']:+.3f} mu {f['mu']:+.3f} visc {f['viscous']:+.3f} bias {f['bias']:+.3f} | mu fixed: fc {m['fc']:+.3f} visc {m['viscous']:+.3f} bias {m['bias']:+.3f}"
                             + (f" | breakaway median {w['breakaway_median']:+.3f} (n {w['breakaway_n']})" if "breakaway_n" in w else ""))
    # ---- pooled recommendation over the informative flights (F3/F4: the arm moves) ----
    lines.append("\n===== POOLED (moving samples, all flights weighted by n) =====")
    rec = {}
    for j in range(4):
        num = den = 0.0
        fcs, mus, vis, bks = [], [], [], []
        for nm, r in allr.items():
            w = r["joints"][f"j{j+1}"]
            if "fit_full" not in w or w["n_moving"] < 300:
                continue
            n = w["n_moving"]
            fcs.append((w["fit_mu_fixed"]["fc"], n)); vis.append((w["fit_mu_fixed"]["viscous"], n))
            mus.append((w["fit_full"]["mu"], n))
            if "breakaway_median" in w:
                bks.append(w["breakaway_median"])
        if fcs:
            fc = sum(a * n for a, n in fcs) / sum(n for _, n in fcs)
            vi = sum(a * n for a, n in vis) / sum(n for _, n in vis)
            mu = sum(a * n for a, n in mus) / sum(n for _, n in mus)
            rec[f"j{j+1}"] = {"fc_mu_fixed": fc, "viscous": vi, "mu_free_fit": mu, "breakaway_median": float(np.median(bks)) if bks else None,
                              "as_flown_fc": float(FC[j]), "as_flown_mu": float(MU[j])}
            lines.append(f"  j{j+1}: fc (mu fixed {MU[j]}) {fc:+.4f} N.m [as flown {FC[j]:.4f}] | viscous {vi:+.3f} N.m/(rad/s) | free-fit mu {mu:+.3f} [as flown {MU[j]}]"
                         + (f" | breakaway {np.median(bks):+.3f}" if bks else ""))
    txt = "\n".join(lines)
    print(txt)
    json.dump({"flights": allr, "recommend": rec, "as_flown": {"fc": FC.tolist(), "mu": MU.tolist(), "width": W_FF, "S": S_G.tolist(), "J": J_ARM.tolist()}},
              open(os.path.join(adir, "arm_comp_audit.json"), "w"), indent=1)
    open(os.path.join(adir, "arm_comp_audit.txt"), "w").write(txt + "\n")


if __name__ == "__main__":
    main()
