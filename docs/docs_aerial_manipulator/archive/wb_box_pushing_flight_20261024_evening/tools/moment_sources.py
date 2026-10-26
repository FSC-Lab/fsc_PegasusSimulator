#!/usr/bin/env python3
"""Which source explains the pitch moment the observer sees in contact? Regress the deadbeat rotational
residual about the pitch axis (wb_control_debug [65], model x = the j2/j3 axis) on: the box's force at the
claw's lever (moment_fit.py's F_box) and the slip direction of q2 and q3 (tanh(qdot / 3 deg/s): a friction
torque's sign). All low-passed 1.5 Hz (zero phase). Prints R^2 per model.
Usage: moment_sources.py <tag> <t0> <t1> <box mass kg>"""
import sys
import numpy as np
from bp_common import Bag

tag, a, c, mb = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
b = Bag(tag)
S = np.load(f"../npz/push_{tag}.npz"); T = S["T"]; m = (T >= a) & (T <= c); dt = 0.02
def lp(x, fc):
    al = np.exp(-2 * np.pi * fc * dt); y = np.empty_like(x); y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = al * y[i - 1] + (1 - al) * x[i]
    return y
def zlp(x, fc):
    return lp(lp(x, fc)[::-1], fc)[::-1]
tw, W = b.t("wb"), b.f("wb", "data"); ok = W[:, 0] > 0.5
M = zlp(np.interp(T, tw[ok], W[ok, 65]), 1.5)
x = zlp(S["box_al"], 4.0); v = np.gradient(x, dt); acc = np.gradient(zlp(v, 3.0), dt)
Fb = zlp(mb * (acc + 0.25 * 9.81 * np.tanh(v / 0.01)), 1.5)
s2 = zlp(np.tanh(np.gradient(zlp(S["q2"], 5.0), dt) / 3.0), 1.5)
s3 = zlp(np.tanh(np.gradient(zlp(S["q3"], 5.0), dt) / 3.0), 1.5)
y = M[m]
def fit(cols, nm):
    X = np.column_stack([np.ones(m.sum())] + [cc[m] for cc in cols])
    cf, *_ = np.linalg.lstsq(X, y, rcond=None)
    r2 = 1 - np.var(y - X @ cf) / np.var(y)
    print(f"  {nm:28s} R^2 {r2:5.2f}  coefficients {np.round(cf[1:], 3)}")
print(f"{tag} {a}-{c}s: deadbeat pitch moment std {np.std(y):.3f} N.m")
fit([Fb], "box force (lever)")
fit([s2, s3], "q2/q3 slip direction")
fit([Fb, s2, s3], "both")
