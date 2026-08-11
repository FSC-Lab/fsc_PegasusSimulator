#!/usr/bin/env python3
"""Yaw-gain sweep against the full autopilot flight stack, compared to real flight C.

Four gain sets flown back to back in one flight, each given an identical +20 deg / -20 deg
yaw step pair. The real reference is flight C's own two yaw steps, measured the same way.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from plot_odom import detect_steps, load_real, load_sim

CONFIGS = [
    ("baseline  K0.3 P0.2 D0     YAWP1.4", 0.0, "#c0392b"),
    ("Kd up     K0.3 P0.2 D0.004 YAWP1.4", 5.0, "#e08214"),
    ("Kd up++   K0.3 P0.2 D0.01  YAWP1.4", 10.0, "#7570b3"),
    ("autotune  K1.0 P0.264 D~0  YAWP3.62", 15.0, "#1b7837"),
]
STEP_T = [(5.0, 20.0), (20.0, 0.0), (35.0, 20.0), (50.0, 0.0),
          (65.0, 20.0), (80.0, 0.0), (95.0, 20.0), (110.0, 0.0)]


def yaw_of(side):
    return np.degrees(np.unwrap(np.radians(side["eul"][:, 2])))


def metrics(t, y, target, t_step):
    m = (t >= t_step - 1.0) & (t <= t_step + 13.0)
    tw, yw = t[m] - t_step, y[m]
    if len(tw) < 20:
        return None
    y0 = yw[tw < -0.1].mean()
    size = target - y0
    if abs(size) < 1.0:
        return None
    s = np.sign(size)
    post = tw >= 0
    tp, yp = tw[post], yw[post]

    def cross(frac):
        idx = np.where(s * (yp - (y0 + frac * size)) >= 0)[0]
        return tp[idx[0]] if len(idx) else None

    t10, t90 = cross(0.1), cross(0.9)
    peak = np.max(s * (yp - y0))
    over = 100.0 * (peak - abs(size)) / abs(size)
    settle = None
    if t10 is not None:
        tol = 0.05 * abs(size)
        mm = tp >= t10
        bad = np.where(np.abs(yp[mm] - target) > tol)[0]
        if len(bad) == 0:
            settle = 0.0
        elif bad[-1] + 1 < mm.sum():
            settle = tp[mm][bad[-1] + 1] - t10
    return dict(rise=(t90 - t10) if (t10 and t90) else None, over=over,
                settle=settle, t10=t10, y0=y0, size=size, tw=tw, yw=yw)


def main():
    sim = load_sim("sim_yawsweep.npz")
    ts, ys = sim["t"], yaw_of(sim)

    # real reference: flight C's own two yaw steps
    real = load_real("exp_C.npz")
    tr, yr = real["t"], yaw_of(real)
    rsteps = [s for s in detect_steps(real["ref_t"], real["ref_pos"], real["ref_yaw"])
              if s["axis"] == "yaw"]
    real_m = [metrics(tr, yr, s["to"], s["t"]) for s in rsteps]
    real_m = [m for m in real_m if m]

    print(f"{'config':38} {'step':>6} | {'rise[s]':>8} {'over[%]':>8} {'settle[s]':>10}")
    print("-" * 80)
    rows = []
    for ci, (name, _, _) in enumerate(CONFIGS):
        per = []
        for k in (0, 1):
            t_step, target = STEP_T[2 * ci + k]
            m = metrics(ts, ys, target, t_step)
            if m:
                per.append(m)
                print(f"{name:38} {target:6.0f} | "
                      f"{(m['rise'] if m['rise'] else np.nan):8.3f} {m['over']:8.1f} "
                      f"{(m['settle'] if m['settle'] is not None else np.nan):10.2f}")
        rows.append(per)
    print("-" * 80)
    for k, m in enumerate(real_m):
        print(f"{'REAL flight C':38} {'':>6} | "
              f"{(m['rise'] if m['rise'] else np.nan):8.3f} {m['over']:8.1f} "
              f"{(m['settle'] if m['settle'] is not None else np.nan):10.2f}")

    def agg(ms, key):
        v = [m[key] for m in ms if m and m.get(key) is not None]
        return np.mean(v) if v else np.nan

    print(f"\n{'config':38} {'rise[s]':>9} {'over[%]':>9} {'settle[s]':>10}")
    print("-" * 80)
    for (name, _, _), per in zip(CONFIGS, rows):
        print(f"{name:38} {agg(per,'rise'):9.3f} {agg(per,'over'):9.1f} "
              f"{agg(per,'settle'):10.2f}")
    print(f"{'REAL flight C':38} {agg(real_m,'rise'):9.3f} {agg(real_m,'over'):9.1f} "
          f"{agg(real_m,'settle'):10.2f}")

    # ---- figure ----
    fig, axes = plt.subplots(2, 4, figsize=(19, 7.5), sharex=True)
    for ci, (name, _, colour) in enumerate(CONFIGS):
        for k in (0, 1):
            ax = axes[k][ci]
            t_step, target = STEP_T[2 * ci + k]
            m = metrics(ts, ys, target, t_step)
            if m:
                ax.plot(m["tw"], m["yw"] - m["y0"], color=colour, lw=1.7, label="sim")
            rm = real_m[k] if k < len(real_m) else None
            if rm:
                ax.plot(rm["tw"], rm["yw"] - rm["y0"], color="0.25", lw=1.3, ls="--",
                        label="real (flight C)")
            ax.axhline(m["size"] if m else 0, color="0.7", lw=0.8, ls=":")
            ax.axvline(0, color="0.85", lw=0.8)
            ax.grid(alpha=0.25)
            ax.tick_params(labelsize=8)
            if k == 0:
                ax.set_title(name, fontsize=9)
            if ci == 0:
                ax.set_ylabel(f"$\\Delta$yaw [deg]\n({'+20' if k==0 else '-20'} step)",
                              fontsize=9)
            ax.set_xlabel("t since step [s]", fontsize=8)
            if ci == 0 and k == 0:
                ax.legend(fontsize=8)
    fig.suptitle("T650 yaw step response: MC_YAWRATE_D sweep and the autotune gain set, "
                 "against real flight C (full autopilot stack, closed loop)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig("figures/yaw_gain_sweep.png", dpi=110)
    print("\nwrote figures/yaw_gain_sweep.png")


if __name__ == "__main__":
    main()
