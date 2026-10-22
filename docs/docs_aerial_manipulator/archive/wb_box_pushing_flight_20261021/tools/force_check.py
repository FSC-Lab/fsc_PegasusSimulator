#!/usr/bin/env python3
"""The law's contact-force estimate (wb_control_debug [97..99], the 4-D raw F_hat, low-passed) against
the Isaac contact truth, then the same estimate on the hardware push: is F_hat a usable force reading,
and what friction does it imply on the real table?
Windows: free = the hover above the handle (jaws open, no contact); settle; slide (second half: the
box is moving); post."""
import numpy as np
from phases import hw_run, sim_run, win, phases


def lp(t, x, fc=1.0):
    """first-order low-pass at fc [Hz], causal, on an irregular time base"""
    y = np.zeros_like(x)
    y[0] = x[0]
    for i in range(1, len(t)):
        a = 1 - np.exp(-2 * np.pi * fc * max(t[i] - t[i - 1], 0.0))
        y[i] = y[i - 1] + a * (x[i] - y[i - 1])
    return y


def mean_in(t, x, a, b):
    m = (t >= a) & (t <= b)
    return x[m].mean(axis=0), x[m].std(axis=0)


def run(r, truth=None):
    d = r["dbg"]
    t = d[:, 0]
    F = d[:, 2 + 97:2 + 100].astype(float)
    Fl = lp(t, F, 1.0)
    P = phases(r)
    m = r["marks"]
    rows = {"free": P["hover"], "onhand": P["onhand"], "settle": P["settle"],
            "slide 2nd half": (0.5 * (P["slide"][0] + P["slide"][1]), P["slide"][1]),
            "slide": P["slide"], "post": P["post"]}
    print(f"\n{r['name']}")
    for k, (a, b) in rows.items():
        mu, sd = mean_in(t, F, a, b)
        line = f"  {k:15s} F_hat raw mean [{mu[0]:+5.2f} {mu[1]:+5.2f} {mu[2]:+5.2f}] N (std {sd.max():4.1f})"
        if truth is not None:
            tm, _ = mean_in(truth[:, 0], truth[:, 1:4], a, b)
            line += f" | truth F_ext [{tm[0]:+5.2f} {tm[1]:+5.2f} {tm[2]:+5.2f}] N"
        print(line)


def sim_truth(s):
    ct = s["contact"]
    tn = ct[:, 7:10]
    sn = 1.0 if np.nanmean(tn[np.abs(tn[:, 2]) > 0.5, 2]) > 0 else -1.0
    # friction-anchor sign as pl_metrics calibrates it (opposes the slide); the push is -y
    F_box = sn * ct[:, 1:4] + sn * ct[:, 4:7]
    return np.column_stack([ct[:, 0], -F_box])


run(hw_run())
for tag in ("v21_1", "v21_2", "v21_3"):
    s = sim_run(tag)
    run(s, sim_truth(s))
