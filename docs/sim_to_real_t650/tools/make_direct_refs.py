#!/usr/bin/env python3
"""Build a DIRECT-mode-only reference sequence from an extracted direct-actuation bag.

Both 2026-08-07 bags begin in SAFETY (baseline) mode and switch to DIRECT partway through;
the newer one runs several step responses in baseline first. Only the DIRECT segment is
comparable to a DIRECT simulation, so everything before the mode switch is dropped.

t = 0 is the DIRECT engagement instant, and waypoint 0 is the reference the vehicle was
already holding at that moment. The driver flies to waypoint 0 and settles before starting
the clock, so the simulation reaches the same initial condition the real vehicle was in
when DIRECT took over -- the same discipline the baseline study used for its mid-flight
bags.

  make_direct_refs.py <real_npz> <out_ref_npz>
"""
import sys

import numpy as np


def main(src, out):
    d = np.load(src, allow_pickle=True)
    t0 = d["odom_t"][0]

    mode_t, mode_v = d["mode_t"], d["mode_v"]
    direct = np.where(mode_v == "DIRECT")[0]
    if len(direct) == 0:
        raise SystemExit(f"{src}: no DIRECT segment in direct_actuation/mode")
    t_direct = mode_t[direct[0]]
    # End of the DIRECT segment: first non-DIRECT sample after it, else end of log.
    after = np.where((mode_t > t_direct) & (mode_v != "DIRECT"))[0]
    t_end = mode_t[after[0]] if len(after) else d["odom_t"][-1]

    ref_t, ref_p = d["ref_t"], d["ref_pos"]
    ref_yaw, ref_unit = d["ref_yaw"], d["ref_yaw_unit"]

    # Waypoint 0: whatever was being held when DIRECT engaged (the initial condition).
    prior = np.where(ref_t <= t_direct)[0]
    if len(prior) == 0:
        raise SystemExit(f"{src}: no reference published before DIRECT engaged")
    i0 = prior[-1]

    # Steps: references issued while DIRECT was active.
    during = np.where((ref_t > t_direct) & (ref_t < t_end))[0]

    idx = np.concatenate(([i0], during))
    t_rel = np.concatenate(([0.0], ref_t[during] - t_direct))

    np.savez_compressed(out, t_rel=t_rel, position=ref_p[idx], yaw=ref_yaw[idx],
                        yaw_unit=ref_unit[idx],
                        direct_window=np.array([t_direct - t0, t_end - t0]))

    print(f"wrote {out}")
    print(f"  DIRECT window in bag: {t_direct-t0:.2f} .. {t_end-t0:.2f}s "
          f"({t_end-t_direct:.1f}s)")
    print(f"  dropped {int((ref_t <= t_direct).sum())} pre-DIRECT reference(s), "
          f"kept {len(during)} step(s) + 1 initial hold")
    for k in range(len(t_rel)):
        p, y = ref_p[idx[k]], ref_yaw[idx[k]]
        tag = "  <- initial condition (held at DIRECT entry)" if k == 0 else ""
        print(f"    t={t_rel[k]:7.2f}s  pos=({p[0]:6.2f},{p[1]:6.2f},{p[2]:5.2f})  "
              f"yaw={y:7.2f}{tag}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
