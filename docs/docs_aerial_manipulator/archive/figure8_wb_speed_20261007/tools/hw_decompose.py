#!/usr/bin/env python3
"""hw_decompose.py -- the 1005 hardware figure-8 EE error split into the part that REPEATS between the two
whole-body runs at one speed (driven by the trajectory) and the part that does not (wander, noise), on the
1005 analysis' own definitions (metrics.analyse via f1005: odometry + encoders through the planner's FK vs
the planner stream, whole EXECUTING span).

Two runs at one speed: e_i(tau) = d(tau) + n_i(tau), n independent of d and of each other, so
  var((e_a - e_b)/2) = sigma_n^2 / 2,   var((e_a + e_b)/2) = sigma_d^2 + sigma_n^2 / 2
on the run-phase grid tau = t - t_start. Errors are resolved on the reference path (along / cross / z).

    AM_NPZ=<dir with w3 w4 w5 w6 .npz> PYTHONNOUSERSITE=1 /usr/bin/python3 hw_decompose.py [--json out.json]
"""
import argparse, json, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_vs_decoupled_figure8_flight_20261005", "tools"))
import f1005 as F          # noqa: E402
import metrics as M        # noqa: E402

rms = lambda x: float(np.sqrt(np.mean(np.square(x))))


def path_frame(t, red, e):
    tan = np.gradient(red[:, :2], t, axis=0); sp = np.linalg.norm(tan, axis=1)
    tan = tan / (sp[:, None] + 1e-12); left = np.column_stack([-tan[:, 1], tan[:, 0]])
    return np.column_stack([np.sum(e[:, :2] * tan, 1), np.sum(e[:, :2] * left, 1), e[:, 2]]), sp


def one(nm):
    o, S = M.analyse(nm)
    t = S["t"] - S["t"][0]
    e = S["re"] - S["red"]
    pf, sp = path_frame(S["t"], S["red"], e)
    return dict(t=t, e=e, pf=pf, sp=sp, m=o["metrics"])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--json", default=""); a = ap.parse_args()
    out = {}
    for v, (na, nb) in ((0.10, ("w3", "w4")), (0.13, ("w5", "w6"))):
        A, B = one(na), one(nb)
        n = min(len(A["t"]), len(B["t"]))
        ea, eb = A["e"][:n], B["e"][:n]
        ok = A["sp"][:n] > 0.02
        half_d = (ea - eb) / 2.0; mean_e = (ea + eb) / 2.0
        # per-axis world xyz variances (incl. mean): sum over axes
        sn2 = 2.0 * np.mean(np.sum(half_d ** 2, 1))
        sd2 = np.mean(np.sum(mean_e ** 2, 1)) - sn2 / 2.0
        r = dict(rms_a=rms(np.linalg.norm(ea, axis=1)) * 1e3, rms_b=rms(np.linalg.norm(eb, axis=1)) * 1e3,
                 repeatable_rms_mm=1e3 * float(np.sqrt(max(sd2, 0.0))), random_rms_mm=1e3 * float(np.sqrt(sn2)),
                 corr_xy=[float(np.corrcoef(ea[:, i], eb[:, i])[0, 1]) for i in range(2)])
        pa, pb = A["pf"][:n][ok], B["pf"][:n][ok]
        for i, k in enumerate(("along", "cross", "z")):
            hd = (pa[:, i] - pb[:, i]) / 2; me = (pa[:, i] + pb[:, i]) / 2
            sn = 2 * np.mean(hd ** 2); sd = np.mean(me ** 2) - sn / 2
            r[k] = dict(mean_a=1e3 * float(pa[:, i].mean()), mean_b=1e3 * float(pb[:, i].mean()),
                        rms_a=1e3 * rms(pa[:, i]), rms_b=1e3 * rms(pb[:, i]),
                        repeatable=1e3 * float(np.sqrt(max(sd, 0))), random=1e3 * float(np.sqrt(sn)),
                        corr=float(np.corrcoef(pa[:, i], pb[:, i])[0, 1]))
        # spectrum of the world-xy error (mean of the two runs): power below 0.06 Hz (slower than one
        # q2 cycle at 0.10 m/s) vs above
        f = np.fft.rfftfreq(n, 0.01)
        P = sum(np.abs(np.fft.rfft(ea[:, i] - ea[:, i].mean())) ** 2 + np.abs(np.fft.rfft(eb[:, i] - eb[:, i].mean())) ** 2 for i in range(2))
        r["psd_peak_hz"] = float(f[1 + np.argmax(P[1:])])
        r["frac_below_0p06Hz"] = float(P[(f > 0) & (f < 0.06)].sum() / P[f > 0].sum())
        r["frac_0p06_0p3Hz"] = float(P[(f >= 0.06) & (f < 0.3)].sum() / P[f > 0].sum())
        r["frac_above_0p3Hz"] = float(P[f >= 0.3].sum() / P[f > 0].sum())
        out[f"{v:.2f}"] = r
        print(f"v {v:.2f}  runs {r['rms_a']:.1f} / {r['rms_b']:.1f} mm  repeatable {r['repeatable_rms_mm']:.1f}  random {r['random_rms_mm']:.1f}"
              f"  corr x/y {r['corr_xy'][0]:.2f}/{r['corr_xy'][1]:.2f}  PSD peak {r['psd_peak_hz']:.3f} Hz"
              f"  power <0.06 {r['frac_below_0p06Hz']:.2f} 0.06-0.3 {r['frac_0p06_0p3Hz']:.2f} >0.3 {r['frac_above_0p3Hz']:.2f}")
        for k in ("along", "cross", "z"):
            q = r[k]
            print(f"     {k:5s} mean {q['mean_a']:+6.1f}/{q['mean_b']:+6.1f}  rms {q['rms_a']:5.1f}/{q['rms_b']:5.1f}  repeatable {q['repeatable']:5.1f}  random {q['random']:5.1f}  corr {q['corr']:+.2f}")
    if a.json:
        json.dump(out, open(a.json, "w"), indent=1); print("wrote", a.json)


if __name__ == "__main__":
    main()
