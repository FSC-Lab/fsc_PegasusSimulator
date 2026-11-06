"""Recover the SAFETY thrust map (normalized = scaling * F / 4 + idle) from a bag.

F = |position_controller/state.output| (SAFETY's force command, N); normalized = -thrust_body[2]
of the SAFETY attitude setpoint sent to PX4. Pairs matched by nearest receive time (<= 5 ms).
"""
import os, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bagdir = sys.argv[1]
db = os.path.join(bagdir, [x for x in os.listdir(bagdir) if x.endswith('.db3')][0])
con = sqlite3.connect(db)
topics = {r[1]: (r[0], r[2]) for r in con.execute('select id,name,type from topics')}

def series(name, f):
    tid, typ = topics[name]
    cls = get_message(typ)
    out = []
    for ts, data in con.execute('select timestamp,data from messages where topic_id=? order by timestamp', (tid,)):
        out.append((ts * 1e-9, f(deserialize_message(data, cls))))
    return np.array(out)

t0 = con.execute('select min(timestamp) from messages').fetchone()[0] * 1e-9
F = series('/uav_0/fsc_autopilot_ros2/position_controller/state',
           lambda m: np.linalg.norm([m.output.x, m.output.y, m.output.z]))
U = series('/uav_0/fmu/in/vehicle_attitude_setpoint', lambda m: -m.thrust_body[2])
i = np.searchsorted(F[:, 0], U[:, 0])
i = np.clip(i, 1, len(F) - 1)
j = np.where(np.abs(F[i - 1, 0] - U[:, 0]) < np.abs(F[i, 0] - U[:, 0]), i - 1, i)
ok = np.abs(F[j, 0] - U[:, 0]) < 0.005
f, u, tt = F[j[ok], 1], U[ok, 1], U[ok, 0] - t0
sel = (u > 0.05) & (u < 0.95)          # off the clamp
f, u, tt = f[sel], u[sel], tt[sel]
A = np.c_[f / 4.0, np.ones_like(f)]
coef, *_ = np.linalg.lstsq(A, u, rcond=None)
res = u - A @ coef
print(f'{os.path.basename(bagdir)}: n={len(u)} samples, t {tt.min():.1f}..{tt.max():.1f} s, F {f.min():.1f}..{f.max():.1f} N')
print(f'  fit: scaling {coef[0]:.6f}, idle {coef[1]:.6f}, residual rms {np.sqrt(np.mean(res**2)):.2e}')
for name, (s, b) in {'10-25 matched (a316625)': (0.037947, 0.252449),
                     '10-24 bench (fe5bc49)': (0.036758, 0.241528)}.items():
    r = u - (s * f / 4.0 + b)
    print(f'  {name:26s} residual rms {np.sqrt(np.mean(r**2)):.2e}  max {np.abs(r).max():.2e}')
