#!/usr/bin/env python3
"""Score whole-body runs on the DIRECT-ENTRY transient and its dominant mode.

    /usr/bin/python3 wb_tune_score.py <run.npz> [run2.npz ...]

wb_l1_metrics.py scores a whole flight and wb_compare_metrics.py scores each
mission leg. This one exists for GAIN TUNING, where the thing being minimised
is the entry transient: the observer starts at zero, has ~10.8 N of mismatch
to find, and the vehicle swings while it finds it. Everything here is measured
from the DIRECT edge.

THE HEADLINE IS `osc`, the amplitude of the dominant lateral mode. On the
2026-09-06 shipped tune that mode is 0.128 Hz (0.81 rad/s) at ~94 mm -- slower
than the position loop (1.97 rad/s) and the attitude loop (3.9 rad/s), which
is why it is attributed to an observer channel rather than to a law gain.
`f_osc` is reported alongside it because a tune that moves the FREQUENCY has
changed which element dominates, and one that only moves the AMPLITUDE has not.
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

ENTRY = 40.0    # s of DIRECT that count as "entry"
TAIL = 25.0     # s at the end that count as "settled"


def score(path):
    z = np.load(path, allow_pickle=True)
    dbg, log = z["dbg"], z["log"]
    o = {"run": path.split("/")[-1].replace(".npz", ""),
         "abort": bool(z["aborted"])}
    d = dbg[:, 1:]
    if d.shape[1] <= D_FY.stop:
        d = np.hstack([d, np.full((d.shape[0], D_FY.stop + 1 - d.shape[1]),
                                  np.nan)])
    m = np.nan_to_num(d[:, D_MODE]) > 0.5
    if not m.any():
        o["err"] = "never entered DIRECT"
        return o
    t = dbg[m, 0] - dbg[m, 0][0]
    dd = d[m]
    e = dd[:, D_XC] - dd[:, D_XCD]
    en = np.linalg.norm(e, axis=1)
    dt = float(np.median(np.diff(t)))

    o["direct_s"] = float(t[-1])
    w = (t > 1.0) & (t < ENTRY)
    o["peak_mm"] = float(en[t < ENTRY].max() * 1e3)
    below = (en < 0.05) & (t > 3)
    o["settle_s"] = float(t[np.argmax(below)]) if below.any() else np.inf

    # Dominant lateral mode over the entry window, detrended so the decay of
    # the transient itself does not masquerade as a low-frequency peak.
    if w.sum() > 64:
        seg = e[w][:, :2]
        tt = t[w]
        amp, fpk = 0.0, np.nan
        for k in (0, 1):
            y = seg[:, k]
            y = y - np.polyval(np.polyfit(tt, y, 3), tt)   # remove the decay
            n = len(y)
            A = np.abs(np.fft.rfft(y * np.hanning(n)))
            fr = np.fft.rfftfreq(n, dt)
            j = np.argmax(A[2:]) + 2
            a = 2 * A[j] / n * 1e3
            if a > amp:
                amp, fpk = a, fr[j]
        o["osc_mm"], o["f_osc_hz"] = float(amp), float(fpk)
        o["entry_rms_mm"] = float(np.sqrt(np.mean(en[w] ** 2)) * 1e3)

    tail = t > t[-1] - TAIL
    o["tail_mm"] = float(np.mean(en[tail]) * 1e3)
    o["tail_rms_mm"] = float(np.sqrt(np.mean(en[tail] ** 2)) * 1e3)
    o["eR_max"] = float(np.nanmax(np.linalg.norm(dd[:, D_ER], axis=1)))
    o["tau_max"] = float(np.nanmax(np.abs(dd[:, D_TAU])))
    o["clamp_pct"] = float(100 * np.mean(
        np.abs(dd[:, D_TAU]).max(axis=1) > 2.999))
    o["sat_pct"] = float(100 * np.nanmean(
        np.nan_to_num(dd[:, D_NSAT]) > 0.5))
    o["fy_N"] = float(np.mean(np.linalg.norm(dd[tail][:, D_FY][:, :3], axis=1)))
    lm = log[:, 17] > 0.5
    o["tilt_max"] = float(log[lm, 7].max()) if lm.any() else np.nan
    return o


def main():
    rows = [score(p) for p in sys.argv[1:]]
    hdr = (f"{'run':<12}{'DIRECT':>8}{'peak':>8}{'osc':>8}{'f_osc':>8}"
           f"{'entryRMS':>10}{'settle':>8}{'tail':>8}{'tilt':>7}{'|e_R|':>8}"
           f"{'tau':>6}{'clamp%':>8}{'|Fy|':>7}")
    print(hdr)
    print("-" * len(hdr))
    for o in rows:
        if "err" in o:
            print(f"{o['run']:<12}  {o['err']}")
            continue
        flag = "  *** ABORTED" if o["abort"] else ""
        print(f"{o['run']:<12}{o['direct_s']:>7.0f}s{o['peak_mm']:>8.0f}"
              f"{o.get('osc_mm', float('nan')):>8.1f}"
              f"{o.get('f_osc_hz', float('nan')):>8.3f}"
              f"{o.get('entry_rms_mm', float('nan')):>10.0f}"
              f"{o['settle_s']:>8.1f}{o['tail_mm']:>8.1f}{o['tilt_max']:>7.2f}"
              f"{o['eR_max']:>8.4f}{o['tau_max']:>6.2f}{o['clamp_pct']:>8.1f}"
              f"{o['fy_N']:>7.3f}{flag}")
    print("\npeak/osc/entryRMS/tail in mm, f_osc in Hz, settle = first time "
          "|x_c-x_cd| < 50 mm")


if __name__ == "__main__":
    main()
