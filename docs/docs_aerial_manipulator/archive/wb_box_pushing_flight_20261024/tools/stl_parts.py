import sys, numpy as np, struct
p = sys.argv[1]
d = open(p, 'rb').read()
n = struct.unpack('<I', d[80:84])[0]
if 84 + 50 * n == len(d):
    a = np.frombuffer(d, dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]), count=n, offset=84)
    V = a['v'].reshape(-1, 3).astype(float)
else:
    import re
    V = np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', d), dtype=float)
print(p, 'triangles', len(V)//3, 'extent', V.min(0).round(4), V.max(0).round(4))
# connected components by shared (rounded) vertices
key = np.round(V, 6)
uniq, inv = np.unique(key, axis=0, return_inverse=True)
inv = inv.ravel()
parent = np.arange(len(uniq))
def find(i):
    while parent[i] != i:
        parent[i] = parent[parent[i]]; i = parent[i]
    return i
for t in range(len(V)//3):
    a_, b_, c_ = inv[3*t:3*t+3]
    for x in (b_, c_):
        ra, rx = find(a_), find(x)
        if ra != rx: parent[rx] = ra
roots = np.array([find(i) for i in range(len(uniq))])
comp = roots[inv[0::3]]
for r in np.unique(comp):
    idx = np.where(comp == r)[0]
    P = V[np.concatenate([3*idx, 3*idx+1, 3*idx+2])]
    lo, hi = P.min(0), P.max(0)
    print(f"  part tris {len(idx):6d}  centre {((lo+hi)/2).round(4)}  size {(hi-lo).round(4)}")
