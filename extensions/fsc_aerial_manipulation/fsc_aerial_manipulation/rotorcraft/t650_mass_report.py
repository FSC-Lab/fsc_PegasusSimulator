"""
| File: t650_mass_report.py
| Description: Report the mass properties PhysX actually simulates for the T650 (every link of the
|   articulation: body + 4 rotor links), and their composite about the composite CoM in the /body
|   frame, next to the controller design inertia. Values come from the PhysX tensor API at run time,
|   not from the USD authoring, so they include how PhysX resolved the authored mass properties.
"""
import json

import numpy as np
from scipy.spatial.transform import Rotation

# The CCM controller's design inertia (CCM_quad configs/config_quad_torque.py, FLU) for comparison.
DESIGN_INERTIA = np.array([[0.077, 0.0089, 0.0], [0.0089, 0.06408172, 0.0], [0.0, 0.0, 0.065004565]])
DESIGN_MASS = 3.033921


def _view(sim_view, stage_prefix):
    for pattern in (stage_prefix, stage_prefix + "/body"):
        try:
            v = sim_view.create_articulation_view(pattern)
            if v is not None and v.count > 0:
                return v, "articulation"
        except Exception:
            pass
    v = sim_view.create_rigid_body_view(stage_prefix + "/*")
    return v, "rigid_body"


def report_mass_properties(stage_prefix, out_path=None):
    import omni.physics.tensors as tensors

    sim_view = tensors.create_simulation_view("numpy")
    view, kind = _view(sim_view, stage_prefix)
    if kind == "articulation":
        paths = list(view.link_paths[0])
        n = view.max_links
        masses = np.asarray(view.get_masses()).reshape(view.count, n)[0]
        coms = np.asarray(view.get_coms()).reshape(view.count, n, 7)[0]
        inertias = np.asarray(view.get_inertias()).reshape(view.count, n, 9)[0]
        tfs = np.asarray(view.get_link_transforms()).reshape(view.count, n, 7)[0]
    else:
        paths = list(view.prim_paths)
        masses = np.asarray(view.get_masses()).reshape(-1)
        coms = np.asarray(view.get_coms()).reshape(-1, 7)
        inertias = np.asarray(view.get_inertias()).reshape(-1, 9)
        tfs = np.asarray(view.get_transforms()).reshape(-1, 7)

    links = []
    for i, p in enumerate(paths):
        R = Rotation.from_quat(tfs[i, 3:7]).as_matrix()          # link prim -> world
        # omni.physics.tensors: get_inertias is the tensor at the CoM expressed in the link prim
        # frame; get_coms carries the principal-axes rotation (x, y, z, qx, qy, qz, qw), reported
        # separately. A product of inertia authored via principalAxes shows up as off-diagonal terms
        # here -- and as a non-identity quaternion -- if PhysX applied it.
        I_link = inertias[i].reshape(3, 3, order="F")
        com_w = tfs[i, 0:3] + R @ coms[i, 0:3]
        links.append(dict(path=p, mass=float(masses[i]), R=R, pos=tfs[i, 0:3], com_w=com_w, I_w=R @ I_link @ R.T,
                          com_local=coms[i, 0:3], com_quat=coms[i, 3:7], I_local=I_link))
    body = next(l for l in links if l["path"].rstrip("/").endswith("/body"))
    M = sum(l["mass"] for l in links)
    c = sum(l["mass"] * l["com_w"] for l in links) / M
    I = np.zeros((3, 3))
    for l in links:
        d = l["com_w"] - c
        I += l["I_w"] + l["mass"] * (d @ d * np.eye(3) - np.outer(d, d))
    Rb = body["R"]
    I_body_frame = Rb.T @ I @ Rb
    com_body_frame = Rb.T @ (c - body["pos"])
    ratio = np.diag(I_body_frame) / np.diag(DESIGN_INERTIA)

    lines = [f"[T650 MASS] PhysX {kind} view of {stage_prefix}: {len(links)} links, total mass {M:.6f} kg "
             f"(design {DESIGN_MASS:.6f})"]
    for l in links:
        lines.append(f"[T650 MASS]   {l['path']}: m {l['mass']:.6f} kg, CoM(link) {np.round(l['com_local'], 6).tolist()}, "
                     f"principal-axes q(xyzw) {np.round(l['com_quat'], 5).tolist()}, "
                     f"I(link frame, at CoM) diag {np.round(np.diag(l['I_local']), 8).tolist()} "
                     f"xy {l['I_local'][0, 1]:.6f}")
    lines.append(f"[T650 MASS] composite CoM in the /body frame {np.round(com_body_frame, 6).tolist()} m")
    lines.append("[T650 MASS] composite inertia about the composite CoM, /body frame (FLU) [kg m^2]:")
    for r in I_body_frame:
        lines.append("[T650 MASS]   " + "  ".join(f"{v: .6f}" for v in r))
    lines.append(f"[T650 MASS] diag / design diag = {np.round(ratio, 4).tolist()}  "
                 f"(design diag {np.diag(DESIGN_INERTIA).tolist()})")
    for ln in lines:
        print(ln, flush=True)
    rep = dict(view=kind, total_mass=M, design_mass=DESIGN_MASS, com_body_frame=com_body_frame.tolist(),
               inertia_body_frame=I_body_frame.tolist(), design_inertia=DESIGN_INERTIA.tolist(),
               diag_ratio=ratio.tolist(),
               links=[dict(path=l["path"], mass=l["mass"], com_local=l["com_local"].tolist(),
                           com_quat_xyzw=l["com_quat"].tolist(), inertia_local=l["I_local"].tolist(),
                           pos_in_body=(body["R"].T @ (l["pos"] - body["pos"])).tolist()) for l in links])
    if out_path:
        with open(out_path, "w") as f:
            json.dump(rep, f, indent=2)
        print(f"[T650 MASS] written to {out_path}", flush=True)
    return rep
