import sys, numpy as np
def yaw_of(q):
    w,x,y,z = q.T
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
f = sys.argv[1]
d = np.load(f, allow_pickle=True)
o = np.asarray(d["odom"], float)
t, z = o[:,0], o[:,3]; yaw = yaw_of(o[:,7:11])
ev = d["events"] if "events" in d else []
z0 = np.median(z[:20]); i = np.argmax(z > z0 + 0.03); tl = t[i]
print("first odom t", t[0], "lift-off t", tl, "z0", z0)
for e in ev[:25]: print("  EV", e)
tt = np.arange(t[0], tl + 8, 0.25)
for x in tt:
    k = np.argmin(abs(t-x))
    print(f"t {t[k]-tl:+6.2f}s  z {z[k]:.3f}  yaw {yaw[k]:+7.2f}  vxy {np.hypot(o[k,4],o[k,5]):.3f}")
