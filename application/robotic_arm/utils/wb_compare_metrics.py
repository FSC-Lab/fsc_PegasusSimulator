#!/usr/bin/env python3
"""Leg-by-leg tracking comparison of two whole-body observers.

    /usr/bin/python3 wb_compare_metrics.py <run.npz> [run2.npz ...]
    /usr/bin/python3 wb_compare_metrics.py --table <gmo.npz> <l1.npz>

wb_l1_metrics.py scores a run as a whole (entry transient, soak, phantom
force). THIS one segments DIRECT by the driver's leg marks and scores each
manoeuvre on its own, which is what a step-response / trajectory-tracking
table needs.

WHAT A "STEP" ACTUALLY IS HERE, and it matters for reading the numbers: the
driver publishes a step CHANGE OF TARGET, but the whole_body_planner then
solves a compatible min-snap transition to it. So every leg is a smooth
tracked trajectory, not a commanded discontinuity -- "peak error" is a
TRACKING error along that trajectory, not step overshoot. The two `traj_ee`
legs differ only in that the arm moves too.

The reference is the law's own x_cd (debug [45..47]), not the raw driver
setpoint, so the comparison is like-for-like between the two observers.

Debug layout: see wb_l1_metrics.py.
"""

import sys

import numpy as np

D_MODE, D_U1, D_NSAT = 0, 17, 51
D_TAU = slice(13, 17)
D_EY = slice(24, 28)
D_ER = slice(28, 31)
D_DHAT = slice(31, 41)
D_XCD = slice(45, 48)
D_XC = slice(48, 51)
D_FY = slice(58, 62)

SETTLE_WINDOW = 2.0   # s at the end of a leg that counts as "settled"


def load(path):
    z = np.load(path, allow_pickle=True)
    dbg = z["dbg"]
    if dbg.size == 0:
        raise SystemExit(f"{path}: no debug samples")
    t, d = dbg[:, 0], dbg[:, 1:]
    if d.shape[1] <= D_FY.stop:
        d = np.hstack([d, np.full((d.shape[0], D_FY.stop + 1 - d.shape[1]),
                                  np.nan)])
    marks = []
    for s in z["leg_marks"]:
        a, b = str(s).split(" ", 1)
        marks.append((float(a), b))
    log = z["log"]
    return dict(path=path, name=path.split("/")[-1].replace(".npz", ""),
                t=t, d=d, marks=marks, log=log,
                aborted=bool(z["aborted"]), reason=str(z["abort_reason"]),
                t_direct=float(z["t_direct"]))


def legs(run):
    """(name, t0, t1) per leg, clipped to DIRECT."""
    t, d, marks = run["t"], run["d"], run["marks"]
    direct = np.nan_to_num(d[:, D_MODE]) > 0.5
    if not direct.any():
        return []
    t_end = t[direct][-1]
    out = []
    for i, (t0, name) in enumerate(marks):
        t1 = marks[i + 1][0] if i + 1 < len(marks) else t_end
        if name == "post_hold":
            continue
        out.append((name, t0, min(t1, t_end)))
    return out


def score_leg(run, t0, t1):
    t, d = run["t"], run["d"]
    m = (t >= t0) & (t < t1)
    if m.sum() < 5:
        return None
    xc, xcd = d[m][:, D_XC], d[m][:, D_XCD]
    err = np.linalg.norm(xc - xcd, axis=1)
    tt = t[m]
    settled = tt >= (tt[-1] - SETTLE_WINDOW)
    ey = np.linalg.norm(d[m][:, D_EY], axis=1)
    tau = np.abs(d[m][:, D_TAU])
    log = run["log"]
    lm = (log[:, 0] >= t0) & (log[:, 0] < t1)
    tilt = log[lm, 7] if lm.any() else np.array([np.nan])
    return dict(
        dur=float(tt[-1] - tt[0]),
        peak_mm=float(np.nanmax(err) * 1e3),
        rms_mm=float(np.sqrt(np.nanmean(err ** 2)) * 1e3),
        settled_mm=float(np.nanmean(err[settled]) * 1e3),
        ee_peak_mm=float(np.nanmax(ey) * 1e3),
        ee_rms_mm=float(np.sqrt(np.nanmean(ey ** 2)) * 1e3),
        ee_settled_mm=float(np.nanmean(ey[settled]) * 1e3),
        tilt_max=float(np.nanmax(tilt)),
        eR_max=float(np.nanmax(np.linalg.norm(d[m][:, D_ER], axis=1))),
        tau_max=float(np.nanmax(tau)),
        clamp_pct=float(100.0 * np.mean(tau.max(axis=1) > 2.999)),
        sat_pct=float(100.0 * np.nanmean(np.nan_to_num(d[m][:, D_NSAT]) > 0.5)),
        fy=float(np.nanmean(np.linalg.norm(d[m][:, D_FY], axis=1))),
        u1=float(np.nanmean(d[m][:, D_U1])),
        dz=float(np.nanmean(d[m][:, D_DHAT][:, 2])),
    )


def report(run):
    print(f"\n=== {run['name']}"
          + ("  *** ABORTED: " + run["reason"] + " ***" if run["aborted"]
             else ""))
    ls = legs(run)
    if not ls:
        # Distinguish the two ways this happens: a run that never reached
        # DIRECT at all, and one that entered and then failed before the
        # mission's first leg. They mean completely different things.
        t, d = run["t"], run["d"]
        direct = np.nan_to_num(d[:, D_MODE]) > 0.5
        if not direct.any():
            print("  never entered DIRECT")
        else:
            dur = t[direct][-1] - t[direct][0]
            print(f"  entered DIRECT and held {dur:.1f} s, but no mission leg "
                  f"ever started -- see the run's own log")
        return {}
    hdr = (f"  {'leg':<14}{'dur':>6}{'peakCoM':>9}{'rms':>7}{'settled':>9}"
           f"{'peakEE':>8}{'settEE':>8}{'tilt':>7}{'tau':>6}{'clamp%':>8}"
           f"{'|Fy|':>7}")
    print(hdr)
    print("  " + "-" * (len(hdr) - 2))
    out = {}
    for name, t0, t1 in ls:
        s = score_leg(run, t0, t1)
        if s is None:
            continue
        out[name] = s
        print(f"  {name:<14}{s['dur']:>6.1f}{s['peak_mm']:>9.1f}"
              f"{s['rms_mm']:>7.1f}{s['settled_mm']:>9.1f}"
              f"{s['ee_peak_mm']:>8.1f}{s['ee_settled_mm']:>8.1f}"
              f"{s['tilt_max']:>7.2f}{s['tau_max']:>6.2f}"
              f"{s['clamp_pct']:>8.1f}{s['fy']:>7.3f}")
    return out


def table(runs):
    """Markdown table, one row per leg, one column pair per run."""
    per = [(r["name"], report(r)) for r in runs]
    names = []
    for _, o in per:
        for k in o:
            if k not in names:
                names.append(k)
    print("\n\n### Markdown\n")
    head = "| leg |" + "".join(
        f" {n} peak / settled |" for n, _ in per)
    print(head)
    print("|---|" + "---|" * len(per))
    for leg in names:
        row = f"| {leg} |"
        for _, o in per:
            s = o.get(leg)
            row += (f" {s['peak_mm']:.0f} / {s['settled_mm']:.1f} mm |"
                    if s else " — |")
        print(row)


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        raise SystemExit(__doc__)
    runs = [load(a) for a in args]
    if "--table" in sys.argv:
        table(runs)
    else:
        for r in runs:
            report(r)


if __name__ == "__main__":
    main()
