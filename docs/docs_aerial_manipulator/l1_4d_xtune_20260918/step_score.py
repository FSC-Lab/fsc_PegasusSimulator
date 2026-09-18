#!/usr/bin/env python3
"""Per-axis settling metrics of the step legs of a wb_l1_campaign_driver run.

    /usr/bin/python3 step_score.py run1.npz [run2.npz ...]

For each step leg: the CoM error component ALONG the step (x for step_x*, y
for step_y*), from the end of the planned move (the planner's HOLD event):
peak during the move, t20/t50 = last time |e| exceeds 20/50 mm after the move
ends [s], the envelope ratio between hold seconds 8-12 and 0-4 (1 = no decay),
the settled rms (last 2 s), the leaked error on the other horizontal axis, the
attitude error peak and the EE error. The slow ~0.15 Hz mode the tune targets
shows up in t20 and the envelope ratio, not in the settled rms alone.
"""
import sys
import numpy as np


def legs_of(d):
    marks = [(float(m.split()[0]), m.split()[1]) for m in d["leg_marks"]]
    ev = [str(e) for e in d["events"]]
    # planner -> HOLD after each leg start = end of the move
    holds = []
    for e in ev:
        if "planner -> HOLD" in e:
            holds.append(float(e.split("s]")[0].strip("[ ")))
    out = []
    for i, (t0, name) in enumerate(marks):
        if not name.startswith("step_"):
            continue
        t1 = marks[i + 1][0] if i + 1 < len(marks) else d["log"][-1, 0]
        te = [h for h in holds if t0 < h < t1]
        if not te:
            continue
        out.append((name, t0, te[0], t1))
    return out


def score(path):
    d = np.load(path, allow_pickle=True)
    D = d["dbg"]; t = D[:, 0]
    col = lambda i: D[:, 1 + i]
    e = np.column_stack([col(48) - col(45), col(49) - col(46), col(50) - col(47)])
    eR = np.sqrt(col(28) ** 2 + col(29) ** 2 + col(30) ** 2)
    ey = np.sqrt(col(24) ** 2 + col(25) ** 2 + col(26) ** 2)
    nsat = col(51)
    rows = []
    for name, t0, te, t1 in legs_of(d):
        ax = 0 if "x" in name else 1
        oth = 1 - ax
        mv = (t >= t0) & (t < te)
        hd = (t >= te) & (t < t1)
        th = t[hd] - te
        ea = e[hd][:, ax]
        peak = np.abs(e[mv][:, ax]).max() if mv.any() else np.nan
        exc = np.where(np.abs(ea) > 0.020)[0]; t20 = th[exc[-1]] if exc.size else 0.0
        exc = np.where(np.abs(ea) > 0.050)[0]; t50 = th[exc[-1]] if exc.size else 0.0
        a = np.abs(ea[(th >= 0) & (th < 4)]).max()
        b = np.abs(ea[(th >= 8) & (th < 12)]).max() if (th >= 8).any() else np.nan
        dec = b / a if a > 0 else np.nan
        sett = np.sqrt(np.mean(ea[th > th[-1] - 2.0] ** 2))
        rows.append((name, peak, t20, t50, dec, sett, np.abs(e[mv | hd][:, oth]).max(),
                     eR[mv | hd].max(), ey[mv | hd].max(), ey[hd][th > th[-1] - 2.0].mean(),
                     100.0 * np.mean(nsat[mv | hd] > 0)))
    return rows


def main():
    print(f"{'run':28s} {'leg':9s} {'peak':>5s} {'t20':>5s} {'t50':>5s} {'env8-12/0-4':>11s} "
          f"{'settled':>7s} {'other':>5s} {'|eR|':>6s} {'ey pk':>5s} {'ey st':>5s} {'clamp':>5s}")
    for p in sys.argv[1:]:
        tag = p.split("/")[-1].replace(".npz", "")
        for r in score(p):
            print(f"{tag:28s} {r[0]:9s} {r[1]*1e3:5.0f} {r[2]:5.1f} {r[3]:5.1f} {r[4]:11.2f} "
                  f"{r[5]*1e3:7.1f} {r[6]*1e3:5.0f} {r[7]:6.3f} {r[8]*1e3:5.1f} {r[9]*1e3:5.1f} {r[10]:5.1f}")


if __name__ == "__main__":
    main()
