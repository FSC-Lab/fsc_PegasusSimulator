#!/usr/bin/env python3
"""Score one or more wb_l1_campaign_driver runs.

    /usr/bin/python3 wb_l1_metrics.py <run.npz> [run2.npz ...]

Everything is measured over the SOAK window -- DIRECT, after the settle
allowance -- except the entry transient, which is measured from the DIRECT
edge. Both rigs publish the same debug array, so the same numbers come out of
a GMO and an L1 run and the columns line up.

THE HEADLINE NUMBER IS THE PHANTOM FORCE. In free flight the true interaction
wrench is exactly zero, so |F_hat_y| is entirely fictitious -- it is the force
the impedance law is rendering compliance against for no reason. The GMO can
only report the lumped (J_y^#)^T d_hat there; the L1 path with attribution on
is supposed to report ~0. That comparison is the working note's own proposed
experiment.

Debug array layout (fsc_autopilot_ros2, wb_control_debug):
  [0] mode        [2] gmo_active   [13..16] tau_joint  [17] u1
  [24..27] e_y    [28..30] e_R     [31..40] d_e_hat (FILTERED, what the loops
                                              see)
  [41..44] motors [45..47] x_cd    [48..50] x_c        [51] n_sat
  [52..55] unallocated wrench      [56] streamed reference fresh
  [57] L1 active  [58..61] F_hat_y [62..71] d_hat^c (UNFILTERED)
  [72..77] w_hat_e                 [78..87] w_hat      [88] outputs on a bound
"""

import sys

import numpy as np

D_MODE, D_GMO, D_U1, D_NSAT, D_STREAM = 0, 2, 17, 51, 56
D_TAU = slice(13, 17)
D_EY = slice(24, 28)
D_ER = slice(28, 31)
D_DHAT = slice(31, 41)
D_MOT = slice(41, 45)
D_XCD = slice(45, 48)
D_XC = slice(48, 51)
D_L1 = 57
D_FY = slice(58, 62)
D_DC = slice(62, 72)
D_WE = slice(72, 78)
D_WHAT = slice(78, 88)
D_NCLAMP = 88


def score(path, settle=20.0):
    z = np.load(path, allow_pickle=True)
    log, dbg = z["log"], z["dbg"]
    aborted = bool(z["aborted"])
    reason = str(z["abort_reason"])
    out = {"file": path.split("/")[-1], "aborted": aborted, "reason": reason}

    if dbg.size == 0 or dbg.shape[1] < 2:
        out["error"] = "no debug samples"
        return out
    t = dbg[:, 0]
    d = dbg[:, 1:]
    if d.shape[1] <= D_NCLAMP:
        # SAFETY-only rows are short; pad so the slices below are defined.
        d = np.hstack([d, np.full((d.shape[0], D_NCLAMP + 1 - d.shape[1]),
                                  np.nan)])

    direct = np.nan_to_num(d[:, D_MODE]) > 0.5
    if not direct.any():
        out["error"] = "never entered DIRECT"
        return out
    t_dir = t[direct][0]
    t_end = t[direct][-1]
    out["t_direct"] = t_dir
    out["direct_s"] = t_end - t_dir
    out["observer"] = ("L1" if np.nanmax(d[direct, D_L1]) > 0.5 else "GMO")

    soak = direct & (t >= t_dir + settle)
    if soak.sum() < 50:
        soak = direct
        out["note"] = "soak window shorter than requested"
    out["soak_s"] = t[soak][-1] - t[soak][0]

    # ---- entry transient: how fast the estimate takes up the mismatch ----
    ent = direct & (t <= t_dir + settle)
    e_com = np.linalg.norm(d[:, D_XC] - d[:, D_XCD], axis=1)
    out["entry_peak_com_mm"] = 1e3 * np.nanmax(e_com[ent])
    # time from the DIRECT edge until the CoM error stays under 20 mm
    idx = np.where(direct)[0]
    settled = None
    for k in range(len(idx)):
        if np.all(e_com[idx[k:]] < 0.020):
            settled = t[idx[k]] - t_dir
            break
    out["entry_settle_s"] = settled if settled is not None else np.inf

    # The last 30 s of DIRECT: the 90 s soak is not automatically steady.
    late = direct & (t >= t_end - 30.0)

    # ---- ESTIMATE RISE, which is NOT the position recovery -------------
    # Measured 2026-09-06: at DIRECT entry the observer starts at d_hat = 0, so
    # the law commands m*g = 36.75 N while the plant needs 47.5 N. The vehicle
    # SAGS ~44 cm in 3 s, picks up lateral velocity during the sag, and the
    # soft CoM position loop then takes ~50 s to wash that out. So
    # entry_settle_s is the POSITION recovery and says almost nothing about
    # the estimator; the time for d_hat_z to reach 90% of its final value is
    # the observer bandwidth, and it is the quantity omega_c / K_o sets.
    dz = d[:, D_DHAT][:, 2]
    dz_final = np.nanmean(dz[late]) if late.any() else np.nan
    rise = np.nan
    if np.isfinite(dz_final) and abs(dz_final) > 1e-6:
        for k in idx:
            if abs(dz[k]) >= 0.9 * abs(dz_final):
                rise = t[k] - t_dir
                break
    out["dhat_rise90_s"] = rise
    if log.size:
        lt0 = log[:, 0]
        w = (lt0 >= t_dir) & (lt0 <= t_dir + 20.0)
        if w.any():
            out["entry_sag_mm"] = 1e3 * (log[w, 3][0] - np.min(log[w, 3]))
            out["entry_xy_max_mm"] = 1e3 * np.nanmax(
                np.linalg.norm(log[w][:, 1:3] - log[w][0, 1:3], axis=1))

    # ---- LATE window ---------------------------------------------------
    # The position recovery runs ~50 s on this plant, so a mean over the whole
    # soak scores the transient as if it were the hold. Report both.
    out["late_com_mm"] = 1e3 * np.nanmean(e_com[late])
    out["late_com_max_mm"] = 1e3 * np.nanmax(e_com[late])

    # ---- steady state ----
    out["com_err_mean_mm"] = 1e3 * np.nanmean(e_com[soak])
    out["com_err_rms_mm"] = 1e3 * np.sqrt(np.nanmean(e_com[soak] ** 2))
    out["com_err_max_mm"] = 1e3 * np.nanmax(e_com[soak])
    e_y = np.linalg.norm(d[:, D_EY][:, :3], axis=1)
    out["ee_err_mean_mm"] = 1e3 * np.nanmean(e_y[soak])
    out["ee_err_max_mm"] = 1e3 * np.nanmax(e_y[soak])
    out["eR_max"] = np.nanmax(np.linalg.norm(d[:, D_ER], axis=1)[soak])
    out["u1_mean_N"] = np.nanmean(d[soak, D_U1])
    out["n_sat_frac"] = float(np.nanmean(d[soak, D_NSAT] > 0))
    out["tau_max_Nm"] = np.nanmax(np.abs(d[:, D_TAU])[soak])
    out["stream_fresh_frac"] = float(np.nanmean(d[soak, D_STREAM] > 0.5))

    # tilt from the odometry log, over the same window
    if log.size:
        lt = log[:, 0]
        m = (lt >= t_dir + settle) & (lt <= t_end)
        if m.sum() > 10:
            out["tilt_pp_deg"] = float(np.ptp(log[m, 7]))
            out["tilt_mean_deg"] = float(np.mean(log[m, 7]))

    # ---- what the estimator says ----
    out["dhat_z_N"] = np.nanmean(d[soak, D_DHAT][:, 2])
    out["dhat_xy_N"] = np.nanmean(np.linalg.norm(d[:, D_DHAT][:, :2], axis=1)[soak])
    out["dhat_rot_Nm"] = np.nanmean(np.linalg.norm(d[:, D_DHAT][:, 3:6], axis=1)[soak])

    # THE HEADLINE: in free flight this is entirely phantom.
    fy_f = np.linalg.norm(d[:, D_FY][:, :3], axis=1)
    out["phantom_Fy_force_N"] = np.nanmean(fy_f[soak])
    out["phantom_Fy_late_N"] = np.nanmean(fy_f[late])
    out["phantom_Fy_max_N"] = np.nanmax(fy_f[soak])
    out["phantom_Fy_psi_Nm"] = np.nanmean(np.abs(d[:, D_FY][:, 3])[late])

    # ---- per-leg scoring: the x / y / yaw steps and the compatible
    # trajectory. Each leg is bounded by its own mark and the next one.
    marks = []
    if "leg_marks" in z.files:
        for entry in z["leg_marks"]:
            tt, _, nm = str(entry).partition(" ")
            marks.append((float(tt), nm))
    if marks:
        out["legs"] = len(marks)
        e_all = e_com
        for k, (t_leg, nm) in enumerate(marks):
            t_next = marks[k + 1][0] if k + 1 < len(marks) else t_end
            w = direct & (t >= t_leg) & (t < t_next)
            if w.sum() < 20:
                continue
            # settle = the last third of the leg, i.e. after the transition
            w_end = direct & (t >= t_next - (t_next - t_leg) / 3.0) & (t < t_next)
            out[f"leg.{nm}.peak_mm"] = 1e3 * np.nanmax(e_all[w])
            out[f"leg.{nm}.settle_mm"] = (1e3 * np.nanmean(e_all[w_end])
                                          if w_end.sum() > 5 else np.nan)
            out[f"leg.{nm}.tilt_max"] = (
                float(np.nanmax(log[(log[:, 0] >= t_leg) & (log[:, 0] < t_next), 7]))
                if log.size else np.nan)
            out[f"leg.{nm}.s"] = t_next - t_leg
    if "leg_fail" in z.files and str(z["leg_fail"]):
        out["leg_fail"] = str(z["leg_fail"])

    if out["observer"] == "L1":
        out["we_force_N"] = np.nanmean(
            np.linalg.norm(d[:, D_WE][:, :3], axis=1)[soak])
        out["we_moment_Nm"] = np.nanmean(
            np.linalg.norm(d[:, D_WE][:, 3:], axis=1)[soak])
        out["what_thrust_N"] = np.nanmean(d[soak, D_WHAT][:, 2])
        out["n_clamped_frac"] = float(np.nanmean(d[soak, D_NCLAMP] > 0))
        # The deadbeat estimate is a 250 Hz momentum difference: this is the
        # noise the filter has to remove, and the reason Steps 2-4 use it
        # unfiltered while the control loops do not.
        out["dc_z_std_N"] = float(np.nanstd(d[soak, D_DC][:, 2]))
        out["dhat_z_std_N"] = float(np.nanstd(d[soak, D_DHAT][:, 2]))
    return out


ROWS = [
    ("observer", "{}", ""), ("aborted", "{}", ""),
    ("direct_s", "{:.1f}", "s"), ("soak_s", "{:.1f}", "s"),
    ("dhat_rise90_s", "{:.2f}", "s"),
    ("entry_sag_mm", "{:.0f}", "mm"), ("entry_xy_max_mm", "{:.0f}", "mm"),
    ("entry_peak_com_mm", "{:.1f}", "mm"), ("entry_settle_s", "{:.1f}", "s"),
    ("com_err_mean_mm", "{:.2f}", "mm"), ("com_err_rms_mm", "{:.2f}", "mm"),
    ("com_err_max_mm", "{:.2f}", "mm"),
    ("ee_err_mean_mm", "{:.2f}", "mm"), ("ee_err_max_mm", "{:.2f}", "mm"),
    ("late_com_mm", "{:.2f}", "mm"), ("late_com_max_mm", "{:.2f}", "mm"),
    ("tilt_pp_deg", "{:.3f}", "deg"), ("tilt_mean_deg", "{:.3f}", "deg"),
    ("eR_max", "{:.4f}", ""),
    ("u1_mean_N", "{:.3f}", "N"), ("tau_max_Nm", "{:.3f}", "N.m"),
    ("n_sat_frac", "{:.3f}", ""), ("stream_fresh_frac", "{:.3f}", ""),
    ("dhat_z_N", "{:.3f}", "N"), ("dhat_xy_N", "{:.3f}", "N"),
    ("dhat_rot_Nm", "{:.4f}", "N.m"),
    ("phantom_Fy_force_N", "{:.4f}", "N"),
    ("phantom_Fy_late_N", "{:.4f}", "N"),
    ("phantom_Fy_max_N", "{:.4f}", "N"),
    ("phantom_Fy_psi_Nm", "{:.4f}", "N.m"),
    ("we_force_N", "{:.4f}", "N"), ("we_moment_Nm", "{:.4f}", "N.m"),
    ("what_thrust_N", "{:.3f}", "N"), ("n_clamped_frac", "{:.3f}", ""),
    ("dc_z_std_N", "{:.3f}", "N"), ("dhat_z_std_N", "{:.4f}", "N"),
]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    res = [score(p) for p in sys.argv[1:]]
    names = [r["file"] for r in res]
    w = max(24, max(len(n) for n in names) + 2)
    print(f"{'metric':<24s}" + "".join(f"{n:>{w}s}" for n in names))
    print("-" * (24 + w * len(names)))
    for key, fmt, unit in ROWS:
        if not any(key in r for r in res):
            continue
        cells = []
        for r in res:
            v = r.get(key, None)
            cells.append("-" if v is None else
                         (fmt.format(v) if not isinstance(v, str) else v))
        label = f"{key} [{unit}]" if unit else key
        print(f"{label:<24s}" + "".join(f"{c:>{w}s}" for c in cells))
    # per-leg block, printed after the summary so the main table stays fixed
    leg_keys = []
    for r in res:
        for k in r:
            if k.startswith("leg.") and k not in leg_keys:
                leg_keys.append(k)
    if leg_keys:
        print()
        print("per-leg (x/y/yaw steps and the compatible-trajectory leg)")
        print("-" * (24 + w * len(names)))
        for k in leg_keys:
            cells = []
            for r in res:
                v = r.get(k)
                cells.append("-" if v is None else
                             ("nan" if isinstance(v, float) and np.isnan(v)
                              else f"{v:.2f}"))
            print(f"{k[4:]:<24s}" + "".join(f"{c:>{w}s}" for c in cells))

    for r in res:
        if r.get("leg_fail"):
            print(f"\n{r['file']}: LEG PROBLEM -- {r['leg_fail']}")
        if r.get("error"):
            print(f"\n{r['file']}: ERROR {r['error']}")
        if r.get("aborted"):
            print(f"\n{r['file']}: ABORTED -- {r['reason']}")
        if r.get("note"):
            print(f"\n{r['file']}: note -- {r['note']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
