"""Box pose around a tilt trip: raw VRPN and /obj_0/mocap, tilt from its rest pose, the claw (current_ee)."""
import sys
import numpy as np

z = np.load(sys.argv[1])
t_a, t_b = float(sys.argv[2]), float(sys.argv[3])
step = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1


def R(q):  # w x y z
    w, x, y, zz = q
    return np.array([[1 - 2 * (y * y + zz * zz), 2 * (x * y - w * zz), 2 * (x * zz + w * y)],
                     [2 * (x * y + w * zz), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - w * x)],
                     [2 * (x * zz - w * y), 2 * (y * zz + w * x), 1 - 2 * (x * x + y * y)]])


def at(a, t):
    i = np.searchsorted(a[:, 0], t)
    i = min(max(i, 0), len(a) - 1)
    return a[i]


for key in ('vrpn_box', 'obj'):
    a = z[key]
    ref = at(a, t_a)
    R0 = R(ref[4:8])
    print(f'== {key}  rest at t={t_a:.2f}: p=({ref[1]:.3f},{ref[2]:.3f},{ref[3]:.3f})')
    for t in np.arange(t_a, t_b + 1e-9, step):
        s = at(a, t)
        Rs = R(s[4:8])
        tilt = np.degrees(np.arccos(np.clip(Rs[:, 2] @ R0[:, 2], -1, 1)))
        yaw = np.degrees(np.arctan2(Rs[1, 0], Rs[0, 0]))
        e = at(z['ee'], t)
        d = s[1:4] - ref[1:4]
        print(f'  t={s[0]:7.2f}  box dp=({d[0]*1e3:6.0f},{d[1]*1e3:6.0f},{d[2]*1e3:6.0f}) mm  tilt {tilt:5.1f}  yaw {yaw:7.1f}'
              f'   claw-box(rest) = ({(e[1]-ref[1])*1e3:5.0f},{(e[2]-ref[2])*1e3:5.0f},{(e[3]-ref[3])*1e3:5.0f}) mm')
