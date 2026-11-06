"""Box mocap quality: tilt noise vs its own median orientation, position jitter, flips; at rest (vehicle far) and
in the hold over the handle (claw within 0.35 m of the box, box not moving)."""
import sys
import numpy as np


def R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


for f in sys.argv[1:]:
    z = np.load(f)
    b = z['vrpn_box']
    ee = z['ee']
    zax = np.array([R(q)[:, 2] for q in b[:, 4:8]])
    xax = np.array([R(q)[:, 0] for q in b[:, 4:8]])
    p = b[:, 1:4]
    # the box's rest pose = the median over the first 5 s of the bag
    m0 = b[:, 0] < b[0, 0] + 5
    p0 = np.median(p[m0], axis=0)
    z0 = np.median(zax[m0], axis=0); z0 /= np.linalg.norm(z0)
    tilt = np.degrees(np.arccos(np.clip(zax @ z0, -1, 1)))
    flips = np.sum(np.abs(np.diff(np.degrees(np.unwrap(np.arctan2(xax[:, 1], xax[:, 0]))))) > 90)
    still = np.linalg.norm(p - p0, axis=1) < 0.005
    ie = np.clip(np.searchsorted(ee[:, 0], b[:, 0]), 0, len(ee) - 1)
    near = np.linalg.norm(ee[ie, 1:4] - p, axis=1) < 0.35
    far = np.linalg.norm(ee[ie, 1:4] - p, axis=1) > 1.0
    dt = np.diff(b[:, 0])
    print(f'== {f.split("/")[-1]}: {len(b)} samples, {1/np.median(dt):.0f} Hz, gaps > 50 ms: {np.sum(dt > 0.05)}, '
          f'180-deg yaw flips: {flips}, box at rest z {p0[2]:.3f} m')
    for name, sel in (('rest, vehicle > 1 m away', still & far & (tilt < 30)), ('hold, claw < 0.35 m away', still & near & (tilt < 30))):
        if sel.sum() < 20:
            print(f'   {name:28s} n={sel.sum()}')
            continue
        t = tilt[sel]
        hp = p[sel] - np.median(p[sel], axis=0)
        print(f'   {name:28s} n={sel.sum():5d}  tilt median {np.median(t):.2f}  p95 {np.percentile(t,95):.2f}  max {t.max():.2f} deg'
              f'  | samples > 1.5 deg {100*np.mean(t>1.5):4.1f} %  > 2.0 deg {100*np.mean(t>2.0):4.1f} %'
              f'  | pos jitter rms {1e3*np.sqrt(np.mean(np.sum(hp**2,1))):.1f} mm')
