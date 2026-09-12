#!/usr/bin/env python3
"""
Split the FSC VTOL CAD STL into articulated parts for the config-driven VTOL.

A single fused STL has no part structure, so this recovers the moving parts by
connected-component ("island") analysis and geometric rules, then recenters each
part on the correct joint so it drops straight into vtol.quad_tiltrotor_config()
(via fsc_cad_vtol_config). Output = OBJ parts; run them through
convert_meshes_to_usd.py to get the USDs the scene references.

    body                     -> body link (fuselage, wings, V-tail, booms, servos,
                                 mounts, rear-rotor mounts — everything fixed)
    front_{left,right}_arm   -> nacelle link (bracket + motor; TILTS, no spin)
    front_{left,right}_prop  -> prop link  (propeller; tilts + spins)
    rear_{left,right}_prop   -> prop link  (propeller; spins)

Requires trimesh (NOT in Isaac's python) — run in a plain venv:
    python3 -m venv /tmp/meshsplit_venv
    /tmp/meshsplit_venv/bin/pip install numpy networkx trimesh
    /tmp/meshsplit_venv/bin/python application/vtol/split_cad_stl.py \\
        --stl "~/Documents/PhDStudies/VTOL_Design/UAV VTOL Fixedwing.STL" \\
        --out ~/vtol_cad_usd/parts2

The rules below were tuned against that specific model (mm units, nose along CAD
+Z, span along CAD X). The hinge is placed at the servo body, then nudged by
HINGE_OFFSET_MM (found interactively against the pivot marker in 06_vtol_rotor_physics.py).
"""

import argparse
import os

import numpy as np
import trimesh

# --- geometry rules (CAD mm; X=span, Y=height, Z=length, nose at high Z) -----
FRONT_Z_MIN = 905.0     # front rotor region (tilting): CAD Z above this
REAR_Z = (250.0, 500.0)  # rear rotor region (static)
LEFT_X_MIN = 1050.0      # left rotor column
RIGHT_X_MAX = 820.0      # right rotor column
BOOM_Y = 62.0            # below this height = boom-level = servo/mount -> fixed
SERVO_MIN_X = 25.0       # servo body: chunky boom-level block
SERVO_MIN_Z = 45.0
# thin horizontal mounting rail (fixed): thin in Y, boom-ish, large footprint
PLATE_MAX_Y = 8.0
PLATE_MAX_YC = 95.0
PLATE_MIN_XZ = 30.0
# hinge = servo-body centre + this offset (down 2.5mm, forward 40mm), the
# interactively-tuned centre of rotation (against the pivot marker in scene 06).
HINGE_OFFSET_MM = np.array([0.0, -2.5, 40.0])


def _region(cx, cz):
    left, right = cx > LEFT_X_MIN, cx < RIGHT_X_MAX
    front, rear = cz > FRONT_Z_MIN, REAR_Z[0] < cz < REAR_Z[1]
    if front and left:  return "front_left"
    if front and right: return "front_right"
    if rear and left:   return "rear_left"
    if rear and right:  return "rear_right"
    return "body"


def _is_fixed(p):
    """A front-region island that should NOT tilt: the boom-level mounting rail
    (thin plate) OR anything at/below boom height (servo body, mount, bracket base)."""
    s = p.extents; c = p.bounds.mean(axis=0)
    plate = s[1] < PLATE_MAX_Y and c[1] < PLATE_MAX_YC and (s[0] > PLATE_MIN_XZ or s[2] > PLATE_MIN_XZ)
    return plate or c[1] < BOOM_Y


def _to_flu(v_mm):
    """CAD-centred mm -> FLU metres: world(X,Y,Z) = (cadZ, cadX, cadY) / 1000."""
    x, y, z = v_mm
    return np.array([z, x, y]) * 0.001


def main():
    ap = argparse.ArgumentParser(description="Split the FSC VTOL STL into articulated parts.")
    ap.add_argument("--stl", required=True, help="input fused STL")
    ap.add_argument("--out", required=True, help="output directory for OBJ parts")
    args = ap.parse_args()

    stl = os.path.expanduser(args.stl)
    out = os.path.expanduser(args.out)
    os.makedirs(out, exist_ok=True)

    mesh = trimesh.load(stl, force="mesh")
    center = mesh.bounds.mean(axis=0)
    islands = mesh.split(only_watertight=False)

    groups = {k: [] for k in ("body", "front_left", "front_right", "rear_left", "rear_right")}
    for p in islands:
        c = p.bounds.mean(axis=0)
        groups[_region(c[0], c[2])].append(p)

    body = list(groups["body"])
    exports, positions = {}, {}
    for name in ("front_left", "front_right", "rear_left", "rear_right"):
        ps = sorted(groups[name], key=lambda p: -len(p.faces))
        prop, rest = ps[0], ps[1:]
        prop_hub = prop.bounds.mean(axis=0)
        pm = prop.copy(); pm.apply_translation(-prop_hub)          # prop spins on its shaft
        exports[f"{name}_prop"] = pm
        if name.startswith("front"):
            fixed = [p for p in rest if _is_fixed(p)]
            motor = [p for p in rest if not _is_fixed(p)]
            body.extend(fixed)
            servo_blocks = [p for p in fixed if p.bounds.mean(0)[2] > FRONT_Z_MIN
                            and p.extents[0] > SERVO_MIN_X and p.extents[2] > SERVO_MIN_Z]
            servo = trimesh.util.concatenate(servo_blocks or fixed)
            tilt_hub = servo.bounds.mean(axis=0) + HINGE_OFFSET_MM   # centre of rotation
            arm = trimesh.util.concatenate(motor); arm.apply_translation(-tilt_hub)
            exports[f"{name}_arm"] = arm
            positions[name] = (_to_flu(tilt_hub - center), _to_flu(prop_hub - tilt_hub))
        else:
            body.extend(rest)                                        # rear mounts -> body
            positions[name] = (_to_flu(prop_hub - center), np.zeros(3))

    b = trimesh.util.concatenate(body); b.apply_translation(-center)
    exports["body"] = b
    for n, m in exports.items():
        m.export(os.path.join(out, n + ".obj"))

    print(f"exported {len(exports)} parts to {out}")
    print("# quad_tiltrotor_config front-rotor values (FLU m):")
    for n in ("front_left", "front_right"):
        pos, spin = positions[n]
        print(f"#   {n}: position=({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})  "
              f"spin_offset=({spin[0]:.3f},{spin[1]:.3f},{spin[2]:.3f})")


if __name__ == "__main__":
    main()
