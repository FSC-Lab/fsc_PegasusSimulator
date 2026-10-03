"""Which armature STRUCTURE do the flights prefer? One scalar J per hypothesis, j2/j3 rows fitted together
(friction per joint), on the moving samples of the two fast flights. Uses armature_id.prepare().
    PYTHONNOUSERSITE=1 /usr/bin/python3 armature_structure.py <dir with a3/a4.npz>"""
import sys, numpy as np
import armature_id as AI
S = {"joint-diagonal": np.eye(2), "model (J h h^T on link)": np.array([[2.0, 1.0], [1.0, 1.0]])}
for fc in (8.0, 12.0):
    AI.FC_NOW = fc
    for nm in ("a3", "a4"):
        D = AI.prepare(nm, fc)
        rows = []
        for j in (1, 2):
            qd = D["qd"][:, j]; mv = (np.abs(qd) > np.radians(3.0)) & (D["qdeg"][:, j] < 49.0); mv[:20] = mv[-20:] = False
            rows.append((j, mv))
        out = []
        for name, H in S.items():
            Ys, As = [], []
            for r, (j, mv) in enumerate(rows):
                sg = np.sign(D["qd"][:, j])
                arm = (H[r, 0] * D["qdd"][:, 1] + H[r, 1] * D["qdd"][:, 2])[mv]
                fr = np.column_stack([sg, sg * np.abs(D["tg"][:, j]), D["qd"][:, j], np.ones_like(sg)])[mv]
                blk = np.zeros((mv.sum(), 1 + 8)); blk[:, 0] = arm; blk[:, 1 + 4 * r:5 + 4 * r] = fr
                As.append(blk); Ys.append(D["y"][mv, j])
            A = np.vstack(As); y = np.concatenate(Ys)
            c, *_ = np.linalg.lstsq(A, y, rcond=None); res = y - A @ c
            out.append(f"{name}: J = {c[0]:.4f}, resid {res.std()*1e3:.2f} mN.m")
        print(f"{fc:4.0f} Hz {nm}: " + " | ".join(out))
