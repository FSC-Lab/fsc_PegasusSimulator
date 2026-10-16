"""Figure: the drop moment. Top: the planner's claw error (arrival_error) from 4 s before the drop leg ends to
2 s after the click, D-1 (basket) and E-1 (no payload), aligned on the leg end, with the 50 mm tolerance and a
20 mm candidate gate. Middle: D-1's basket position minus the drop point (x, y) and the ballistic landing
point an instantaneous release would give. Bottom: wrist roll q4 and the basket tilt.  -> ../figures/drop.png"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pd15_common import *   # noqa: F401,F403
from release import vel, drop_point

C_D1, C_E1, C_REF, C3 = "#2a78d6", "#eb6834", "#8a8984", "#1baf7a"


def main():
    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for run, col, lab in (("e1", C_E1, "E-1 no payload"), ("d1", C_D1, "D-1 basket")):
        d, t0 = load(run)
        a, b = leg_windows(d, t0)["execute_place (approach)"]
        t_open = [t for t, n, m in rosout(d, t0, contains="gripper OPEN") if t > a][0]
        ta = d["pp_arr__recv"] - t0; A = d["pp_arr__data"]
        m = (ta >= b - 4) & (ta <= t_open + 0.12)
        ax[0].plot(ta[m] - b, A[m, 2] * 1e3, color=col, lw=1.6, label=lab)
        ax[0].plot([t_open - b], [np.interp(t_open, ta, A[:, 2]) * 1e3], "o", color=col, ms=8)
        ax[0].annotate(f"{RUNS[run][0]} Drop click\n+{t_open - b:.2f} s", (t_open - b, np.interp(t_open, ta, A[:, 2]) * 1e3),
                       xytext=(8, 10 if run == "d1" else -28), textcoords="offset points", fontsize=8, color="#52514e")
        if run == "d1":
            pt, ee, yaw = drop_point(d, t0)
            to, Po, Qo = obj_pose(d, t0)
            Vo = vel(to, Po)
            mo = (to >= b - 4) & (to <= t_open + 1.0)
            rest_z = Po[-1, 2]
            h = np.clip(Po[mo, 2] - rest_z, 0, None)
            land = Po[mo, :2] + Vo[mo, :2] * np.sqrt(2 * h / 9.80665)[:, None]
            x = to[mo] - b
            ax[1].plot(x, (Po[mo, 0] - pt[0]) * 1e3, color=C_D1, lw=1.4, label="basket x - drop point")
            ax[1].plot(x, (Po[mo, 1] - pt[1]) * 1e3, color=C3, lw=1.4, label="basket y - drop point")
            mm = to[mo] <= t_open
            ax[1].plot(x[mm], np.linalg.norm(land[mm] - pt[:2], axis=1) * 1e3, color="#0b0b0b", lw=1.0, ls="--",
                       label="|ballistic landing - drop point|")
            rest = Po[-1, :2] - pt[:2]
            ax[1].annotate(f"rest: ({rest[0]*1e3:.0f}, {rest[1]*1e3:.0f}) mm, tilt {tilt_of(Qo[-1:])[0]:.0f}°",
                           (x[-1], rest[1] * 1e3), xytext=(-170, 12), textcoords="offset points", fontsize=8, color="#52514e")
            tw, D = wbdebug(d, t0)
            mw = (tw >= b - 4) & (tw <= t_open + 1.0)
            ax[2].plot(tw[mw] - b, np.degrees(D[mw, 8]), color=C_D1, lw=1.4, label="wrist roll q4 (D-1)")
            from pp_common import quat_R
            roll_b = np.array([np.degrees(np.arctan2(R[2, 1], R[2, 2])) for R in (quat_R(*q) for q in Qo[mo])])
            ax[2].plot(x, -roll_b, color=C3, lw=1.4, label="basket roll about the claw axis (D-1, world)")
            for a_ in ax:
                a_.axvline(t_open - b, color=C_D1, lw=0.8, ls=":")
    ax[0].axhline(50, color=C_REF, lw=1.0, ls="--"); ax[0].text(-3.9, 52, "planner tolerance 50 mm", fontsize=8, color="#52514e")
    ax[0].axhline(20, color=C_REF, lw=1.0, ls=":"); ax[0].text(-3.9, 22, "candidate trigger gate 20 mm", fontsize=8, color="#52514e")
    for a_ in ax:
        a_.axvline(0, color="#c3c2b7", lw=0.8)
        a_.grid(True, color="#e6e5e0", lw=0.6)
        a_.legend(loc="upper right", fontsize=8, frameon=False)
    ax[0].set_ylabel("claw error |target - claw| (mm)", fontsize=9)
    ax[1].set_ylabel("basket - drop point (mm)", fontsize=9)
    ax[2].set_ylabel("deg", fontsize=9)
    ax[2].set_xlabel("time from the end of the drop leg (s)")
    fig.suptitle("The drop: claw error at the click, where the basket was, and the wrist roll the basket swings with", fontsize=11)
    fig.tight_layout()
    out = os.path.join(FIG, "drop.png")
    fig.savefig(out, dpi=110)
    print("wrote", out)


if __name__ == "__main__":
    main()
