#!/usr/bin/env python3
"""Overlay the runs of a wb_l1 campaign.

    /usr/bin/python3 wb_l1_plot.py out.png run1.npz run2.npz ...

Three panels, all against time since the DIRECT edge so the runs line up:

  1. CoM tracking error -- the entry transient is the whole story of the
     estimate's bandwidth: the observer has to take up ~10.8 N of injected
     mismatch before the vehicle holds station, and how long that takes is
     set by the filter the estimate reaches the control loops through.
  2. The estimate itself, d_hat_z -- where the 10.8 N goes.
  3. The PHANTOM end-effector force. In free flight the true interaction
     wrench is EXACTLY zero, so every newton here is fictitious: it is what
     the impedance law is rendering compliance against for no reason. This is
     the panel the attribution step exists for.

Run it as

    PYTHONNOUSERSITE=1 /usr/bin/python3 wb_l1_plot.py ...

on this machine: ~/.local carries numpy 2.2.6 while the apt matplotlib was
built against numpy 1.x, and the user site-packages wins on sys.path, so a
plain `python3` fails at import with "numpy.core.multiarray failed to import".
Disabling user site-packages picks up the apt pair (numpy 1.21.5 +
matplotlib 3.5.1), which agree. No ROS needed either way.
"""

import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402

D_MODE, D_L1 = 0, 57
D_XCD, D_XC = slice(45, 48), slice(48, 51)
D_DHAT = slice(31, 41)
D_FY = slice(58, 62)


def load(path):
    z = np.load(path, allow_pickle=True)
    dbg = z["dbg"]
    t, d = dbg[:, 0], dbg[:, 1:]
    if d.shape[1] <= D_L1:
        return None
    m = np.nan_to_num(d[:, D_MODE]) > 0.5
    if not m.any():
        return None
    t0 = t[m][0]
    obs = "L1" if np.nanmax(d[m, D_L1]) > 0.5 else "GMO"
    return dict(
        name=f"{path.split('/')[-1].replace('.npz','')} [{obs}]",
        t=t[m] - t0,
        e_com=1e3 * np.linalg.norm(d[m][:, D_XC] - d[m][:, D_XCD], axis=1),
        dhat_z=d[m][:, D_DHAT][:, 2],
        fy=np.linalg.norm(d[m][:, D_FY][:, :3], axis=1),
        aborted=bool(z["aborted"]))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    out, paths = sys.argv[1], sys.argv[2:]
    runs = [r for r in (load(p) for p in paths) if r]
    if not runs:
        print("no usable runs")
        return 1

    fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for r in runs:
        lab = r["name"] + (" ABORTED" if r["aborted"] else "")
        ax[0].plot(r["t"], r["e_com"], lw=1.0, label=lab)
        ax[1].plot(r["t"], r["dhat_z"], lw=1.0, label=lab)
        ax[2].plot(r["t"], r["fy"], lw=1.0, label=lab)

    ax[0].set_ylabel("CoM tracking error [mm]")
    ax[0].set_yscale("log")
    ax[0].axhline(20, color="k", ls=":", lw=0.8)
    ax[0].set_title("Whole-body DIRECT hover, +15% kf / mass x1.10 / CoM shift "
                    "/ rotor lag -- time from the DIRECT edge")
    ax[1].set_ylabel(r"$\hat{d}_z$ fed to $f_d$ [N]")
    ax[2].set_ylabel(r"phantom $|\hat{F}_y|$ force [N]")
    ax[2].set_xlabel("t since DIRECT [s]")
    ax[2].text(0.01, 0.92,
               "free flight: the TRUE interaction wrench is exactly zero",
               transform=ax[2].transAxes, fontsize=8, color="0.35")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
