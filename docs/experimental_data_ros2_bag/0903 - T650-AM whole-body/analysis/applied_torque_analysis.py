#!/usr/bin/env python3
"""Applied-torque analysis for the whole-body DIRECT flights of 2 and 3 Sep 2026.

Three torque signals per joint, all in the ARM's own sign convention:

  tau_cmd   law_debug[9+j]        the whole-body law's joint torque, as received
                                  by the arm controller (== joint_torque_command)
  duty      law_debug[13+j]       the PWM duty count actually written to the servo
  tau_app   joint_states.effort/N Present Current ticks -> N.m through the
                                  calibrated per-joint constant

and the identified command chain

  duty = tau_cmd * N_j * rho_j  +  (PWM_FULL*Ke_j/Vs) * qd_ref_j     (then clipped)
  rho_j = PWM_FULL * R_j * CUR_LSB / Vs           [duty counts per current count]

Run with /usr/bin/python3 (numpy only; the npz files are already extracted).
"""
import json, os
import numpy as np

BASE = os.path.dirname(os.path.abspath(__file__))
D0902 = os.path.join(BASE, "..", "..", "0902 - T650-AM whole-body", "analysis")
D0903 = BASE

# ---------------------------------------------------------------- servo model
JN   = ["joint 1", "joint 2", "joint 3", "joint 4"]
NM2C = np.array([160.0, 173.8, 146.7, 160.0])   # counts/N.m, calibrated 2026-08-17
R    = np.array([4.90, 4.90, 4.30, 4.90])       # motor_resistance_ohm (config)
KE   = np.array([2.32, 2.86, 2.40, 2.32])       # back_emf_ke (config), V.s/rad
VS, PWMF, LSB = 12.0, 885.0, 0.00269            # supply_voltage_v, pwm_full_scale, current_lsb_a
RHO    = PWMF * R * LSB / VS                    # duty counts per current count
NM2PWM = NM2C * RHO                             # duty counts per N.m
KE2PWM = PWMF * KE / VS                         # duty counts per rad/s
KT     = 1.0 / (NM2C * LSB)                     # N.m/A implied by the calibration
BTH    = KT * KE / R                            # N.m per rad/s, droop with the CONFIGURED back_emf_ke
BKT    = KT ** 2 / R                            # N.m per rad/s, droop with Ke == Kt (SI identity)
CAP    = np.array([57.6117, 364.8743, 192.0391, 57.6117])   # max_effort, duty counts
TAUCAP = CAP / NM2PWM                           # the same cap in N.m
SIGN   = np.array([-1.0, 1.0, 1.0, -1.0])       # model convention = SIGN * arm convention

DT  = 0.004        # analysis grid, s
LAG = 1            # samples of transport lag applied to the measured side

FLIGHTS = [
    dict(key="f0903", tag="3 Sep 10:16", npz=os.path.join(D0903, "whole_body_test_20260903_101609.npz"),
         t0=6.65, t1=40.65, trip=40.38,
         stages=[(6.65, 24.44, "hold",       "arm folded at home, governor HOLD", False),
                 (24.44, 28.29, "transition", "commanded EE move, j3 40 -> 8.3 deg", False),
                 (28.29, 33.00, "osc onset", "pitch 3 -> 5 deg", True),
                 (33.00, 36.00, "osc growing", "pitch 5 -> 14 deg", True),
                 (36.00, 40.65, "osc large", "pitch 14 -> 23 deg, watchdog", True)]),
    dict(key="f0902", tag="2 Sep 12:46", npz=os.path.join(D0902, "wb_tracking_20260902_124652.npz"),
         t0=6.44, t1=103.94, trip=None,
         stages=[(6.44, 30.00, "hold",       "arm folded at home", False),
                 (30.00, 33.00, "transition 1", "j3 40 -> 14.5 deg", False),
                 (33.00, 56.37, "hold",       "at the new pose", False),
                 (56.37, 59.38, "transition 2", "j3 back to 40 deg", False),
                 (59.38, 80.73, "hold",       "at home", False),
                 (80.73, 86.10, "transition 3", "base target moved 0.6 m", False),
                 (86.10, 103.94, "hold",      "at home", False)]),
]


def resample(npz, t0, t1):
    d = np.load(npz, allow_pickle=True)
    L, lt = d["law"], d["law_t"]
    t = np.arange(t0 + 0.30, t1 - 0.05, DT)
    o = lambda ts, x: np.stack([np.interp(t, ts, x[:, i]) for i in range(x.shape[1])], 1)
    out = dict(t=t,
               tau=o(lt, L[:, 9:13]),
               duty=o(lt, L[:, 13:17]),
               I=o(d["js_t"], d["js_eff"][:, 2:6]),
               q=o(d["js_t"], d["js_pos"][:, 2:6]),
               qd_raw=o(d["js_t"], d["js_vel"][:, 2:6]),
               qd_obs=o(d["vo_t"], d["vo_vel"]),
               qd_ref=o(d["srt_t"], d["srt_vel"]),
               q_ref=o(d["srt_t"], d["srt_pos"]))
    pe, pt = d["px_eul"], d["px_eul_t"]
    out["roll"] = np.interp(t, pt, pe[:, 0])
    out["pitch"] = np.interp(t, pt, pe[:, 1])
    return out


def lagged(S):
    """Shift the measured signals LAG samples earlier so amplitude, not delay, is scored."""
    n = len(S["t"]) - LAG
    C = {k: (v[LAG:][:n] if k in ("I", "qd_raw", "q") else v[:n]) for k, v in S.items()}
    return C


def roll_ratio(tc, ta, t, half=1.0, floor=0.08):
    """Percentage curve: least-squares projection of delivered on commanded in a
    +/- `half` second window.  Undefined where the window carries no command."""
    w = int(round(half / DT))
    num = np.convolve(tc * ta, np.ones(2 * w + 1), "same")
    den = np.convolve(tc * tc, np.ones(2 * w + 1), "same")
    cnt = np.convolve(np.ones_like(tc), np.ones(2 * w + 1), "same")
    out = np.where(den > cnt * floor ** 2, 100.0 * num / np.maximum(den, 1e-12), np.nan)
    return out


def analyse():
    res = {}
    for F in FLIGHTS:
        S = lagged(resample(F["npz"], F["t0"], F["t1"]))
        t = S["t"]
        tau, duty, I = S["tau"], S["duty"], S["I"]
        tau_goal = duty / NM2PWM
        tau_app = I / NM2C
        sat = np.abs(duty) >= CAP * 0.999
        F["S"] = S
        F.setdefault("npz", F["npz"])
        F["tau_goal"], F["tau_app"], F["sat"] = tau_goal, tau_app, sat
        # ---- chain verification: duty vs the identified equation
        # The base class clamps the TORQUE term at max_effort; the back-EMF
        # feedforward is added afterwards and only the PWM full scale bounds it.
        pred = np.clip(tau * NM2PWM, -CAP, CAP) + KE2PWM * S["qd_ref"]
        pred = np.clip(pred, -PWMF, PWMF)
        F["chain_err"] = np.abs(duty - pred).max(0)
        F["chain_rms"] = (duty - pred).std(0)
        # ---- percentage curve
        F["eta"] = np.stack([roll_ratio(tau[:, j], tau_app[:, j], t) for j in range(4)], 1)
        res[F["key"]] = F
    return res


def fit(res):
    """Identify tau_app = tau_goal - b*qd on the unsaturated samples of both flights."""
    rows = []
    for key in ("f0903", "f0902"):
        F = res[key]
        for j in range(4):
            ok = ~F["sat"][:, j]
            r = F["tau_app"][ok, j] - F["tau_goal"][ok, j]
            for lbl, v in (("obs", F["S"]["qd_obs"][ok, j]), ("raw", F["S"]["qd_raw"][ok, j])):
                b = -(v @ r) / (v @ v) if v @ v > 1e-9 else np.nan
                res_ = r + b * v
                r2 = 1 - res_.var() / r.var() if r.var() > 0 else np.nan
                rows.append((key, j, lbl, b, r2, res_.std(), r.std(), ok.sum()))
    return rows


if __name__ == "__main__":
    res = analyse()
    print("SERVO CONSTANTS")
    print("  rho (duty/count)     ", np.round(RHO, 6))
    print("  N*rho (duty per N.m) ", np.round(NM2PWM, 4))
    print("  Ke*PWMF/Vs (duty per rad/s)", np.round(KE2PWM, 3))
    print("  Kt = 1/(N*LSB) N.m/A ", np.round(KT, 4))
    print("  b_th = Kt*Ke/R       ", np.round(BTH, 4))
    print("  b_kt = Kt^2/R        ", np.round(BKT, 4))
    print("  torque cap (N.m)     ", np.round(TAUCAP, 4))
    for key in ("f0903", "f0902"):
        F = res[key]
        print(f"\n== {F['tag']} ==   DIRECT {F['t0']:.2f}-{F['t1']:.2f} s, n={len(F['S']['t'])}")
        print("  chain max|err| (duty counts):", np.round(F["chain_err"], 3),
              " rms:", np.round(F["chain_rms"], 4))
        print("  saturated fraction:", np.round(F["sat"].mean(0), 4))
        print("  |qd_obs| max:", np.round(np.abs(F["S"]["qd_obs"]).max(0), 3),
              " |qd_ref| max:", np.round(np.abs(F["S"]["qd_ref"]).max(0), 4))
        print("  |tau_cmd| rms/peak:", np.round(F["S"]["tau"].std(0), 3), np.round(np.abs(F["S"]["tau"]).max(0), 3))
    print("\nFIT  tau_app = tau_goal - b*qd   (unsaturated samples)")
    print("  flight  joint  vel     b        R2      resid   raw     n")
    for k, j, lbl, b, r2, rs, r0, n in fit(res):
        print(f"  {k}  j{j+1}     {lbl}   {b:7.3f}  {r2:7.3f}  {rs:.4f}  {r0:.4f}  {n}")


# ============================================================ report payload
def dec(n_samples, hz):
    """Bin boundaries for an anti-aliased decimation to `hz`: every trace in a
    figure is averaged over the SAME bins, so nothing is selectively smoothed."""
    step = max(1, int(round(1.0 / (hz * DT))))
    n = (n_samples // step) * step
    return n, step


def binmean(x, n, step):
    xr = x[:n].reshape(-1, step)
    if np.isnan(xr).any():
        with np.errstate(invalid="ignore"):
            out = np.full(xr.shape[0], np.nan)
            good = ~np.isnan(xr).all(axis=1)
            out[good] = np.nanmean(xr[good], axis=1)
        return out
    return xr.mean(axis=1)


def r3(a, p=4):
    return [None if not np.isfinite(v) else round(float(v), p) for v in a]


def payload():
    res = analyse()
    out = {"const": {
        "N": list(NM2C), "R": list(R), "Ke": list(KE), "Kt": r3(KT), "rho": r3(RHO, 6),
        "nm2pwm": r3(NM2PWM, 3), "ke2pwm": r3(KE2PWM, 2), "b_th": r3(BTH, 3), "b_kt": r3(BKT, 4),
        "taucap": r3(TAUCAP, 4), "cap": list(CAP), "vs": VS, "pwmf": PWMF, "lsb": LSB}}
    flights = {}
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]; t = S["t"]
        hz = 50.0 if key == "f0903" else 25.0
        fl = {"tag": F["tag"], "t0": float(F["t0"]), "t1": float(F["t1"]),
              "trip": F["trip"],
              "stages": [{"t0": a, "t1": b, "label": l, "note": nn, "bad": bd}
                         for a, b, l, nn, bd in F["stages"]]}
        n, step = dec(len(t), hz)
        B = lambda v: binmean(v, n, step)
        ne, se = dec(len(t), 10.0)
        Be = lambda v: binmean(v, ne, se)
        fl["t"] = r3(B(t), 3)
        fl["te"] = r3(Be(t), 3)
        for j in range(4):
            model = F["tau_goal"][:, j] - BKT[j] * S["qd_obs"][:, j]
            ok = ~F["sat"][:, j]
            fl[f"j{j+1}"] = {
                "cmd": r3(B(S["tau"][:, j]), 4),
                "app": r3(B(F["tau_app"][:, j]), 4),
                "mdl": r3(B(model), 4),
                "vel": r3(B(S["qd_obs"][:, j]), 4),
                "eta": r3(Be(F["eta"][:, j]), 1),
                "cap": round(float(TAUCAP[j]), 3),
                "rng": round(float(max(np.abs(S["tau"][:, j]).max(),
                                       np.abs(F["tau_app"][:, j]).max()) * 1.08), 2),
                "mdl_rms": round(float(np.std((F["tau_app"][:, j] - model)[ok])), 4),
                "ideal_rms": round(float(np.std((F["tau_app"][:, j] - F["tau_goal"][:, j])[ok])), 4),
                "meas_rms": round(float(np.std(F["tau_app"][ok, j])), 4),
                "cmd_rms": round(float(np.std(S["tau"][:, j])), 4),
                "cmd_pk": round(float(np.abs(S["tau"][:, j]).max()), 3),
                "sat": round(float(F["sat"][:, j].mean() * 100), 2),
                "vmax": round(float(np.abs(S["qd_obs"][:, j]).max()), 3),
            }
        flights[key] = fl
    out["flights"] = flights

    # ---- delivered fraction by |speed| bin, and the mean drop by signed speed
    sedges = [0.0, 0.02, 0.05, 0.10, 0.20, 0.35, 0.60, 1.00, 2.50]
    out["spdlab"] = ["0.01", "0.035", "0.075", "0.15", "0.27", "0.47", "0.8", "1.7"]
    out["spd"] = {}
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]
        for j in (1, 2):
            row = []
            for a, b in zip(sedges[:-1], sedges[1:]):
                sel = (~F["sat"][:, j]) & (np.abs(S["qd_obs"][:, j]) >= a) & \
                      (np.abs(S["qd_obs"][:, j]) < b) & (np.abs(F["tau_goal"][:, j]) > 0.15)
                if sel.sum() < 40:
                    row.append(None); continue
                g = (F["tau_goal"][sel, j] @ F["tau_app"][sel, j]) / (F["tau_goal"][sel, j] @ F["tau_goal"][sel, j])
                row.append(round(float(100 * g), 1))
            out["spd"][f"j{j+1}{'a' if key=='f0903' else 'b'}"] = row

    dedges = np.array([-1.5, -1.0, -0.7, -0.5, -0.35, -0.25, -0.15, -0.08, -0.04, -0.015,
                       0.015, 0.04, 0.08, 0.15, 0.25, 0.35, 0.5, 0.7, 1.0, 1.5])
    ctr = 0.5 * (dedges[:-1] + dedges[1:])
    out["droplab"] = [f"{v:+.2f}" for v in ctr]
    out["dropx"] = r3(ctr, 3)
    out["drop"] = {}
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]
        for j in (1, 2):
            row, xs = [], []
            for a, b in zip(dedges[:-1], dedges[1:]):
                sel = (~F["sat"][:, j]) & (S["qd_obs"][:, j] >= a) & (S["qd_obs"][:, j] < b)
                if sel.sum() < 30:
                    row.append(None); xs.append(None); continue
                row.append(round(float((F["tau_app"][sel, j] - F["tau_goal"][sel, j]).mean()), 4))
                xs.append(round(float(S["qd_obs"][sel, j].mean()), 4))
            out["drop"][f"j{j+1}{'a' if key=='f0903' else 'b'}"] = row
            out["drop"][f"x_j{j+1}{'a' if key=='f0903' else 'b'}"] = xs
    out["drop"]["mdl_j2"] = r3(-BKT[1] * ctr, 4)
    out["drop"]["mdl_j3"] = r3(-BKT[2] * ctr, 4)

    # ---- high-speed slope (regression dilution avoided)
    out["bfit"] = {}
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]
        for j in range(4):
            sel = (~F["sat"][:, j]) & (np.abs(S["qd_obs"][:, j]) > 0.15)
            if sel.sum() < 100:
                out["bfit"][f"j{j+1}{'a' if key=='f0903' else 'b'}"] = None; continue
            v = S["qd_obs"][sel, j]; r = F["tau_app"][sel, j] - F["tau_goal"][sel, j]
            out["bfit"][f"j{j+1}{'a' if key=='f0903' else 'b'}"] = [
                round(float(-(v @ r) / (v @ v)), 3), int(sel.sum()),
                round(float(1 - np.var(r + (-(v @ r) / (v @ v)) * v) / np.var(r)), 3)]

    # ---- zero-speed gain
    out["g0"] = {}
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]
        for j in range(4):
            z = (np.abs(S["qd_obs"][:, j]) < 0.005) & (np.abs(F["tau_goal"][:, j]) > 0.2) & (~F["sat"][:, j])
            out["g0"][f"j{j+1}{'a' if key=='f0903' else 'b'}"] = \
                [round(float(100 * (F["tau_goal"][z, j] @ F["tau_app"][z, j]) /
                             (F["tau_goal"][z, j] @ F["tau_goal"][z, j])), 1), int(z.sum())] if z.sum() > 100 else None

    # ---- per-phase table
    out["phase"] = {}
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]; t = S["t"]
        rows = []
        for a, b, lab, note, bad in F["stages"]:
            sel = (t >= a) & (t < b)
            row = {"label": lab, "t0": a, "t1": b, "bad": bad}
            for j in (1, 2):
                s2 = sel & (~F["sat"][:, j])
                g = (S["tau"][s2, j] @ F["tau_app"][s2, j]) / (S["tau"][s2, j] @ S["tau"][s2, j])
                row[f"j{j+1}"] = [round(float(np.mean(S["tau"][sel, j])), 3),
                                  round(float(np.std(S["tau"][sel, j])), 3),
                                  round(float(np.sqrt(np.mean(S["qd_obs"][sel, j] ** 2))), 3),
                                  round(float(np.abs(S["qd_obs"][sel, j]).max()), 2),
                                  round(float(100 * g), 0),
                                  round(float(np.mean(np.abs(F["tau_app"][sel, j] - S["tau"][sel, j]))), 3),
                                  round(float(F["sat"][sel, j].mean() * 100), 1)]
            rows.append(row)
        out["phase"][key] = rows

    # ---- the six extra terms: total, decomposition, and what they had to cancel
    QMIN = np.array([-0.610865, -1.396263, -0.698132, -1.570796])
    QMAX = np.array([0.610865, 0.872665, 0.872665, 1.570796])
    # After this time on 3 Sep joint 2 crosses its software limit and the
    # controller substitutes the PD pull-back, so `duty - conversion` is no
    # longer "the six extra terms" and the decomposition is meaningless.
    CLEAN_END = {"f0903": 36.5, "f0902": 1e9}
    # ---- reference-chain synchronisation: the governor's q_d against the arm
    # controller's own smoothed reference, and against the measured joint.
    out["sync"] = {}
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]; t = S["t"]
        d = np.load(F["npz"], allow_pickle=True)
        on = lambda ts, x: np.stack([np.interp(t, ts, x[:, i]) for i in range(x.shape[1])], 1)
        # WholeBodyReference is MODEL convention; SIGN maps it to the arm's.
        qd = on(d["ref_t"], d["ref_q_d"]) * SIGN
        vd = on(d["ref_t"], d["ref_qdot_d"]) * SIGN
        n, step = dec(len(t), 50.0 if key == "f0903" else 25.0)
        Bn = lambda v: binmean(v, n, step)
        ent = {"t": r3(Bn(t), 3)}
        for j in range(4):
            ent[f"qd{j+1}"] = r3(np.degrees(Bn(qd[:, j])), 3)
            ent[f"qc{j+1}"] = r3(np.degrees(Bn(S["q_ref"][:, j])), 3)
            ent[f"qm{j+1}"] = r3(np.degrees(Bn(S["q"][:, j])), 3)
            ent[f"vd{j+1}"] = r3(np.degrees(Bn(vd[:, j])), 3)
            ent[f"vc{j+1}"] = r3(np.degrees(Bn(S["qd_ref"][:, j])), 3)
            ent[f"vm{j+1}"] = r3(np.degrees(Bn(S["qd_obs"][:, j])), 3)
        # when did each reference MOVE, and where did it end up?
        def moves(v, thr):
            a = np.abs(v) > thr
            idx = np.where(a)[0]
            if not len(idx):
                return []
            return [(float(t[r[0]]), float(t[r[-1]])) for r in
                    np.split(idx, np.where(np.diff(idx) > 25)[0] + 1) if len(r) > 25]
        ent["rows"] = []
        for j in range(4):
            mg = moves(vd[:, j], np.radians(1.0))
            ma = moves(S["qd_ref"][:, j], np.radians(1.0))
            ent["rows"].append({
                "j": j + 1,
                "gov": [[round(a, 2), round(b, 2)] for a, b in mg],
                "arm": [[round(a, 2), round(b, 2)] for a, b in ma],
                "gov_end": round(float(np.degrees(qd[-1, j])), 2),
                "arm_end": round(float(np.degrees(S["q_ref"][-1, j])), 2),
                "gap_max": round(float(np.degrees(np.abs(qd[:, j] - S["q_ref"][:, j]).max())), 2),
                "vpk_gov": round(float(np.degrees(np.abs(vd[:, j]).max())), 2),
                "vpk_arm": round(float(np.degrees(np.abs(S["qd_ref"][:, j]).max())), 2),
                "lag": (round(ma[0][0] - mg[0][0], 2) if (mg and ma) else None)})
        out["sync"][key] = ent

    out["ff"] = {}
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]; t = S["t"]
        conv = np.clip(S["tau"] * NM2PWM, -CAP, CAP)
        sup = (S["duty"] - conv) / NM2PWM                 # supplied, N.m
        req = BKT * S["qd_obs"]                           # droop to cancel, N.m
        n, step = dec(len(t), 50.0 if key == "f0903" else 25.0)
        Bn = lambda v: binmean(v, n, step)
        te = CLEAN_END[key]
        mask = binmean((t < te).astype(float), n, step) > 0.999
        ent = {"t": r3(Bn(t), 3)}
        for j in (1, 2):
            sv = Bn(sup[:, j]).copy(); sv[~mask] = np.nan
            ent[f"sup{j+1}"] = r3(sv, 5)
            ent[f"req{j+1}"] = r3(Bn(req[:, j]), 5)
        ent["clean_end"] = None if te > 1e8 else te
        ent["terms"] = {}
        for j in (1, 2):
            ok = ((t < te) & (np.abs(S["duty"][:, j]) < CAP[j] * 0.999)
                  & (S["q"][:, j] > QMIN[j]) & (S["q"][:, j] < QMAX[j]))
            y = (S["duty"] - conv)[ok, j]                 # duty counts
            vr = S["qd_ref"][ok, j]; tt = t[ok]; k = NM2PWM[j]
            emf = KE2PWM[j] * vr                          # the one live term
            r = y - emf
            rows = [("back-EMF feedforward", "s·(885·Ke/Vs)·q̇_ref",
                     float(abs(emf).std() / k),
                     float((vr @ y) / (vr @ vr)) if vr @ vr > 1e-12 else 0.0,
                     float(KE2PWM[j]))]
            for nm, expr, b in (
                    ("Coulomb friction", "(f+λ|g_arm|)·tanh(q̇_ref/w)", np.tanh(vr / 0.015)),
                    ("viscous friction", "v·q̇_ref", None),
                    ("breakaway / stiction", "ε(t), ramped while stuck",
                     np.sign(vr) * (np.abs(vr) > 0.005)),
                    ("dither", "A·sin(2π·100·t)", np.sin(2 * np.pi * 100 * tt)),
                    ("integral", "kᵢ∫(q_ref−q)dt",
                     np.cumsum(S["q_ref"][ok, j] - S["q"][ok, j]) * DT)):
                if b is None:
                    rows.append((nm, expr, None, None, None)); continue
                c = float((b @ r) / (b @ b)) if b @ b > 1e-12 else 0.0
                rows.append((nm, expr, float(abs(c) * b.std() / k), c, None))
            ent["terms"][f"j{j+1}"] = {
                "rows": [{"name": a, "expr": e, "rms": rr, "coef": c, "cfg": cf}
                         for a, e, rr, c, cf in rows],
                "total": float(y.std() / k), "peak": float(np.abs(y).max() / k),
                "resid_rms": float(r.std() / k), "resid_max": float(np.abs(r).max() / k),
                "quant": float(1.0 / NM2C[j]), "req_rms": float(req[ok, j].std()),
                "corr": float(np.corrcoef(sup[ok, j], req[ok, j])[0, 1]),
                "var_removed": float(1 - np.var(req[ok, j] - sup[ok, j]) / np.var(req[ok, j])),
                "n": int(ok.sum())}
        ph = []
        for a, b, lab, note, bad in F["stages"]:
            sel = (t >= a) & (t < b) & (t < te) & (np.abs(S["duty"][:, 2]) < CAP[2] * 0.999)
            if sel.sum() < 50:
                continue
            ph.append({"label": lab, "bad": bad,
                       "sup": round(float(sup[sel, 2].std()), 4),
                       "req": round(float(req[sel, 2].std()), 4),
                       "pct": round(float(100 * sup[sel, 2].std() / max(req[sel, 2].std(), 1e-9)), 1)})
        ent["phase_j3"] = ph
        act = np.abs(S["qd_ref"][:, 2]) > 0.005
        idx = np.where(act)[0]
        runs = np.split(idx, np.where(np.diff(idx) > 3)[0] + 1) if len(idx) else []
        ent["ref_active_pct"] = round(float(100 * act.mean()), 1)
        ent["ref_runs"] = [{"t0": round(float(t[rr[0]]), 2), "t1": round(float(t[rr[-1]]), 2),
                            "vref": round(float(np.abs(S["qd_ref"][rr, 2]).max()), 3),
                            "v": round(float(np.abs(S["qd_obs"][rr, 2]).max()), 3),
                            "q0": round(float(np.degrees(S["q"][rr[0], 2])), 1),
                            "q1": round(float(np.degrees(S["q"][rr[-1], 2])), 1)}
                           for rr in runs if len(rr) > 25]
        ent["travel_ref"] = round(float(np.degrees(np.abs(np.diff(S["q_ref"][:, 2])).sum())), 1)
        ent["travel_q"] = round(float(np.degrees(np.abs(np.diff(S["q"][:, 2])).sum())), 1)
        out["ff"][key] = ent

    # ---- chain verification numbers
    out["chain"] = {}
    QMIN = np.array([-0.610865, -1.396263, -0.698132, -1.570796])
    QMAX = np.array([0.610865, 0.872665, 0.872665, 1.570796])
    for key in ("f0903", "f0902"):
        F = res[key]; S = F["S"]
        pred = np.clip(np.clip(S["tau"] * NM2PWM, -CAP, CAP) + KE2PWM * S["qd_ref"], -PWMF, PWMF)
        ok = (S["q"] > QMIN) & (S["q"] < QMAX)
        out["chain"][key] = {"n": int(len(S["t"])),
                             "max": [round(float(np.abs(S["duty"][ok[:, j], j] - pred[ok[:, j], j]).max()), 3) for j in range(4)],
                             "rms": [round(float(np.std(S["duty"][ok[:, j], j] - pred[ok[:, j], j])), 4) for j in range(4)],
                             "nout": [int((~ok[:, j]).sum()) for j in range(4)]}
    return out


if __name__ == "__main__" and "--json" in __import__("sys").argv:
    if True:
        p = os.path.join(BASE, "applied_torque_payload.json")
        with open(p, "w") as f:
            json.dump(payload(), f, separators=(",", ":"))
        print("wrote", p, os.path.getsize(p), "bytes")
