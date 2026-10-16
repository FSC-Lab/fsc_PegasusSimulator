#!/usr/bin/env python3
"""Replay each flight's DIRECT demand through OLD and NEW caps: seconds the torque term would be clipped by
max_effort, and seconds the total duty (clipped term + the flown back-EMF feed-forward + trim, i.e. what the
duty carried beyond the torque term) would reach the servo PWM Limit.
PYTHONNOUSERSITE=1 /usr/bin/python3 caps_replay.py <npz> ..."""
import sys
import numpy as np
from caps_audit import KPWM

OLD = dict(cap=np.array([57.6117, 364.8743, 192.0391, 57.6117]), pwm=np.array([180.0, 545.0, 330.0, 180.0]))
NEW = dict(cap=np.array([85.0, 420.0, 260.0, 57.6117]), pwm=np.array([212.0, 587.0, 414.0, 189.0]))

print(f"{'flight':44s} | {'term clipped s (j1..j4) OLD':>28s} | {'NEW':>22s} | {'duty at PWM Limit s OLD':>24s} | {'NEW':>22s}")
for p in sys.argv[1:]:
    d = np.load(p)
    L = d["law__data"]; tl = d["law__recv"]; pt = L[:, 0] > 0.5
    dt = np.median(np.diff(tl))
    T = (L[:, 9:13] * KPWM + L[:, 18:22] + L[:, 22:26])[pt]
    duty = L[pt, 13:17]
    beyond = duty - np.clip(T, -OLD["cap"], OLD["cap"])          # FF + trim actually written
    f = lambda v: " ".join(f"{x:5.2f}" for x in v)
    res = []
    for C in (OLD, NEW):
        clipped = (np.abs(T) > C["cap"]).sum(axis=0) * dt
        tot = np.clip(T, -C["cap"], C["cap"]) + beyond
        atlim = (np.abs(tot) >= C["pwm"] - 0.5).sum(axis=0) * dt
        res.append((clipped, atlim))
    print(f"{'/'.join(p.split('/')[-3:]):44s} | {f(res[0][0]):>28s} | {f(res[1][0]):>22s} | {f(res[0][1]):>24s} | {f(res[1][1]):>22s}")
