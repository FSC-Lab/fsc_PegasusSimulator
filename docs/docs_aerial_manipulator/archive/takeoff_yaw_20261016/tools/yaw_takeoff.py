import sys, numpy as np, glob
def yaw_of(q):  # w x y z
    w,x,y,z = q.T
    return np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
for f in sys.argv[1:]:
    try:
        d = np.load(f, allow_pickle=True)
        o = np.asarray(d["odom"], float)
    except Exception as e:
        print(f, "ERR", e); continue
    t, z = o[:,0], o[:,3]
    yaw = yaw_of(o[:,7:11]); direct = o[:,11]
    z0 = np.median(z[:20])
    i = np.argmax(z > z0 + 0.03)   # lift-off
    if z[i] <= z0 + 0.03: print(f, "no lift-off"); continue
    j = np.argmax(direct > 0.5) if np.any(direct>0.5) else len(t)-1
    tl = t[i]
    yaw0 = np.median(yaw[max(0,i-50):i])
    seg = slice(i, j)
    dy = yaw[seg] - yaw0
    k = np.argmax(np.abs(dy))
    # time back within 20% of peak
    back = np.where((np.arange(len(dy))>k) & (np.abs(dy) < 0.2*abs(dy[k])))[0]
    tb = t[seg][back[0]]-tl if len(back) else np.nan
    print(f"{f.split('/')[-1]:14s} yaw_ground {yaw0:7.2f}  peak {dy[k]:+6.2f} deg at +{t[seg][k]-tl:5.2f}s (z {z[seg][k]:.2f})  back<20% at +{tb:5.2f}s  final-before-DIRECT {dy[-1]:+6.2f}  SAFETY span {t[j]-tl:5.1f}s")
