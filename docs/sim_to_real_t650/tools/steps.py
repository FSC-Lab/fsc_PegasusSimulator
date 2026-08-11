#!/usr/bin/env python3
"""Step detection and timing for the position-controller reference sequence.

Timing note (this cost the earlier X650 campaign real time -- see
fsc_autopilot_ros2/docs/sim_to_real_fidelity.md section 7): the rosbag2 `messages.timestamp`
column is the RECEIVE time and is bursty, and the reference topic's own header stamps sit on
a different clock epoch from the PX4 topics. Neither can be used directly for step onsets.

The PX4 topics carry a clean hrt `timestamp` field, so this module builds the
receive-time -> PX4-time map from the attitude-setpoint stream (which has both) and applies
it to the reference messages' receive times. Residual jitter is a few ms rather than the
~300 ms the raw receive times carry.
"""
import numpy as np

G = 9.80665


def recv_to_px4_map(exp_npz):
    """Least-squares affine fit recv_ns -> px4_us over the attitude setpoint stream."""
    d = np.load(exp_npz)
    recv = d["sp_bag_t"].astype(np.float64)
    px4 = d["sp_ts"].astype(np.float64)
    A = np.stack([recv, np.ones_like(recv)], axis=1)
    coef, *_ = np.linalg.lstsq(A, px4, rcond=None)
    resid = px4 - A @ coef
    return coef, float(np.sqrt(np.mean(resid**2)))


def ref_onsets(exp_npz, ref_npz):
    """Reference waypoints on the PX4/replay time axis (t=0 at the first attitude setpoint)."""
    d = np.load(exp_npz)
    r = np.load(ref_npz)
    coef, rms = recv_to_px4_map(exp_npz)
    # ref_*.npz stored t_rel already relative to sp_bag_t[0], in seconds; undo to raw ns
    recv_ns = r["t_rel"] * 1e9 + float(d["sp_bag_t"][0])
    px4_us = coef[0] * recv_ns + coef[1]
    t = (px4_us - float(d["sp_ts"][0])) * 1e-6
    return t, r, rms


def detect_steps(t, r, min_change=0.05, min_yaw_change=2.0):
    """Turn the reference waypoint sequence into discrete single-axis steps.

    Consecutive waypoints are differenced; a step is recorded for whichever of x / y / z /
    yaw actually moved. A waypoint that moves two axes at once yields two entries (none
    occur in these two flights). The first waypoint has no predecessor and is skipped unless
    an explicit prior state is supplied by the caller.
    """
    pos = r["position"]
    yaw = np.asarray(r["yaw"], dtype=np.float64)
    # yaw is in degrees here (yaw_unit == 0 for both flights); unwrap 360 -> 0
    yaw = (yaw + 180.0) % 360.0 - 180.0

    steps = []
    for i in range(1, len(t)):
        dx = pos[i] - pos[i - 1]
        dyaw = (yaw[i] - yaw[i - 1] + 180.0) % 360.0 - 180.0
        hold = (t[i + 1] - t[i]) if i + 1 < len(t) else None
        for k, name in enumerate("xyz"):
            if abs(dx[k]) >= min_change:
                steps.append(dict(idx=len(steps), t=t[i], axis=name,
                                  frm=float(pos[i - 1, k]), to=float(pos[i, k]),
                                  size=float(dx[k]), hold=hold))
        if abs(dyaw) >= min_yaw_change:
            steps.append(dict(idx=len(steps), t=t[i], axis="yaw",
                              frm=float(yaw[i - 1]), to=float(yaw[i]),
                              size=float(dyaw), hold=hold))
    return steps


def quat_to_rot(q):
    """PX4 q=[w,x,y,z] -> R (body->NED), shape (N,3,3)."""
    q = np.asarray(q, dtype=np.float64)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - w * z)
    R[:, 0, 2] = 2 * (x * z + w * y)
    R[:, 1, 0] = 2 * (x * y + w * z)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - w * x)
    R[:, 2, 0] = 2 * (x * z - w * y)
    R[:, 2, 1] = 2 * (y * z + w * x)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def horizontal_accel_ned(q):
    """Horizontal specific acceleration implied by the vehicle's tilt, in NED [m/s^2].

    The thrust axis is body -z; in a quasi-hover the horizontal acceleration it produces is
    a_h = -g * (R[0,2], R[1,2]) / R[2,2]. This is the single scalar pair that a horizontal
    position step actually commands, and unlike raw roll/pitch it is independent of heading,
    so it can be projected straight onto the commanded step direction.
    """
    R = quat_to_rot(q)
    denom = np.clip(R[:, 2, 2], 0.2, None)
    return -G * np.stack([R[:, 0, 2], R[:, 1, 2]], axis=1) / denom[:, None]


def euler_deg(q):
    q = np.asarray(q, dtype=np.float64)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(np.stack([roll, pitch, yaw], axis=1))


def step_direction_ned(step):
    """Unit vector in NED for a horizontal step. The reference frame is ENU
    (x=East, y=North), so ENU (dx,dy) maps to NED (dy, dx)."""
    if step["axis"] == "x":
        v = np.array([0.0, np.sign(step["size"])])
    elif step["axis"] == "y":
        v = np.array([np.sign(step["size"]), 0.0])
    else:
        return None
    return v


if __name__ == "__main__":
    import sys
    tag = sys.argv[1] if len(sys.argv) > 1 else "A"
    t, r, rms = ref_onsets(f"exp_{tag}.npz", f"ref_{tag}.npz")
    print(f"recv->px4 affine fit residual RMS: {rms*1e-3:.2f} ms")
    print(f"{'i':>3} {'t[s]':>8}  {'axis':>4} {'from':>8} {'to':>8} {'size':>7} {'hold[s]':>8}")
    for s in detect_steps(t, r):
        h = f"{s['hold']:8.2f}" if s["hold"] else f"{'-':>8}"
        print(f"{s['idx']:3d} {s['t']:8.2f}  {s['axis']:>4} {s['frm']:8.2f} {s['to']:8.2f} "
              f"{s['size']:+7.2f} {h}")
