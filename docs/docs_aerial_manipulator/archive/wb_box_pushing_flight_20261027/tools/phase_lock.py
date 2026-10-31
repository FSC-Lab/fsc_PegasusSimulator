#!/usr/bin/env python3
"""Lock-in at the contact cycle's frequency: amplitude and phase (deg, relative to the vehicle's
along-push CoM error) of every signal that could drive it. Usage: phase_lock.py <tag> <t0> <t1> [f]"""
import sys
import numpy as np
from bp_common import Bag

tag, a, c = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
b = Bag(tag)
S = np.load(f"../npz/push_{tag}.npz"); T = S["T"]; m = (T >= a) & (T <= c); t = T[m] - T[m][0]
tw, W = b.t("wb"), b.f("wb", "data"); ok = W[:, 0] > 0.5
def wc(i):
    return np.interp(T, tw[ok], W[ok, i])[m]
dt = 0.02
def dom(x):
    x = x - np.polyval(np.polyfit(t, x, 1), t)
    P = np.abs(np.fft.rfft(x * np.hanning(len(x)))) ** 2; f = np.fft.rfftfreq(len(x), dt)
    k = (f > 0.15) & (f < 1.5); return f[k][np.argmax(P[k])]
e = S["ecom_al"][m]
f = float(sys.argv[4]) if len(sys.argv) > 4 else dom(e)
A = np.column_stack([np.cos(2*np.pi*f*t), np.sin(2*np.pi*f*t), np.ones_like(t), t])
def lk(x):
    cc, *_ = np.linalg.lstsq(A, x, rcond=None); return np.hypot(cc[0], cc[1]), np.angle(cc[0] - 1j*cc[1])
ae, pe = lk(e)
box_v = np.gradient(S["box_al"], dt)[m]
veh_v = np.gradient(S["veh_al"], dt)[m]
mu, mb = 0.25, float(sys.argv[5]) if len(sys.argv) > 5 else 0.28
fric = -mu * mb * 9.81 * np.tanh(box_v / 0.01)       # friction ON THE BOX along the push
rows = [("CoM along error [m]", e), ("vehicle along velocity [m/s]", veh_v), ("box along velocity [m/s]", box_v),
        ("friction on box [N]", fric),
        ("pitch error e_R x (model) [rad]", wc(28)), ("roll error e_R y (model) [rad]", wc(29)),
        ("pitch moment deadbeat (model x) [N.m]", wc(65)), ("pitch moment filtered d_hat_r x [N.m]", wc(34)),
        ("force deadbeat along nose (model y) [N]", wc(63)), ("force filtered d_hat_t y [N]", wc(32)),
        ("force deadbeat z [N]", wc(64)),
        ("free-axis offset [m]", wc(117)), ("q2 [deg]", S["q2"][m]), ("q3 [deg]", S["q3"][m]),
        ("tau2 [N.m]", S["tau2"][m]), ("tau3 [N.m]", S["tau3"][m]),
        ("EE task error along (model y) [m]", wc(25)), ("EE task error z [m]", wc(26)),
        ("F_hat_y consumed along [N]", wc(59)), ("F_hat_y consumed z [N]", wc(60))]
print(f"{tag} {a}-{c}s: lock-in at {f:.3f} Hz (period {1/f:.2f} s); phase relative to the CoM along error, + = leads")
for nm, x in rows:
    am, ph = lk(x)
    print(f"  {nm:42s} amp {am:9.4f}  phase {np.degrees(ph - pe + np.pi) % 360 - 180:+7.1f} deg")
