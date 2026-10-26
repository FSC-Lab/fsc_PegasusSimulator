#!/usr/bin/env python3
"""Is the pitch moment the box's force acting at the claw, 0.113 m below the CoM?
Fits the deadbeat rotational residual about the push-normal axis (wb_control_debug [62..71] rows
3..5, model frame; model x = body -y = the pitch axis while the nose points along the push) against
the force the box needs: F = m (a_box + mu g sgn v_box), along the push, from the box's mocap.
Also prints the box-speed statistics of the push (reversals, stuck share).
Usage: moment_fit.py <tag> <t0> <t1> <box mass kg>"""
import sys
import numpy as np
from bp_common import Bag

tag, t0, t1, mb = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
b = Bag(tag)
S = np.load(f"../npz/push_{tag}.npz")
T = S["T"]; m = (T >= t0) & (T <= t1)
dt = 0.02

def lp(x, fc):
    a = np.exp(-2 * np.pi * fc * dt); y = np.empty_like(x); y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = a * y[i - 1] + (1 - a) * x[i]
    return y

def zlp(x, fc):        # zero-phase
    return lp(lp(x, fc)[::-1], fc)[::-1]

tw, W = b.t("wb"), b.f("wb", "data")
ok = W[:, 0] > 0.5
Mraw = np.interp(T, tw[ok], W[ok, 65])      # deadbeat d^c, rotational x (model)
Mfil = np.interp(T, tw[ok], W[ok, 34])      # filtered (what the law cancels)
x = zlp(S["box_al"], 4.0)
v = np.gradient(x, dt); a = np.gradient(zlp(v, 3.0), dt)
mu = 0.25
Fbox = mb * (a + mu * 9.81 * np.tanh(v / 0.01))     # force the claw must apply ALONG the push
Mz = zlp(Mraw, 1.5)
Fbox = zlp(Fbox, 1.5)
X = np.column_stack([np.ones(m.sum()), Fbox[m]])
c, *_ = np.linalg.lstsq(X, Mz[m], rcond=None)
r = np.corrcoef(Fbox[m], Mz[m])[0, 1]
print(f"{tag} {t0}..{t1}: pitch moment (deadbeat, model x) = {c[0]:+.3f} + {c[1]:+.4f} * F_box   corr {r:+.2f}  "
      f"(claw 0.113 m below the CoM predicts slope ~ -/+0.11 m)")
print(f"  moment std: deadbeat {np.std(Mz[m]):.3f}, filtered {np.std(Mfil[m]):.3f} N.m; F_box std {np.std(Fbox[m]):.2f} N")
vv = v[m] * 1e3
plan = 21.0
print(f"  box speed along push: mean {vv.mean():.1f} mm/s (plan {plan}), std {vv.std():.1f}, min {vv.min():.0f}, max {vv.max():.0f}")
print(f"  share of time moving BACKWARD (< -5 mm/s) {np.mean(vv < -5):.0%}, ~still (|v|<5) {np.mean(np.abs(vv) < 5):.0%}, forward {np.mean(vv > 5):.0%}")
sgn = np.sign(np.where(np.abs(vv) < 5, 0, vv)); nz = sgn[sgn != 0]
print(f"  direction reversals {int(np.sum(np.diff(nz) != 0))} in {t1 - t0:.1f} s")
# dominant frequency of the vehicle's along-push error
e = S["ecom_al"][m] - S["ecom_al"][m].mean()
f = np.fft.rfftfreq(len(e), dt); P = np.abs(np.fft.rfft(e * np.hanning(len(e)))) ** 2
k = np.argmax(P[(f > 0.05)]) + np.sum(f <= 0.05)
print(f"  CoM along-push error: rms {1e3*np.std(e):.0f} mm, p-p {1e3*(e.max()-e.min()):.0f} mm, peak at {f[k]:.2f} Hz")
eR = S["eR0"][m]
print(f"  pitch error (model x): rms {1e3*np.std(eR):.0f} mrad, range {1e3*eR.min():.0f}..{1e3*eR.max():.0f}")
# lag between the force and the CoM error
nL = min(100, m.sum() // 2)
cc = [np.corrcoef(Fbox[m][:m.sum() - L], S['ecom_al'][m][L:])[0, 1] for L in range(0, nL)]
L = int(np.argmax(np.abs(cc)))
print(f"  CoM along-error follows F_box by {L*dt:.2f} s, corr {cc[L]:+.2f}")
