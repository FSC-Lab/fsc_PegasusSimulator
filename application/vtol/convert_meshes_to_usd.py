#!/usr/bin/env python
"""
Headless mesh -> USD converter for VTOL CAD components.

Turns tessellated CAD exports (STL / OBJ / FBX / glTF) into USD so they can be
referenced as link geometry by the config-driven VTOL (see vtol.py mesh support).

Because each articulated link needs its OWN geometry, convert each component
separately: body + one file per rotor (2 tilting + 2 static).

Run with the Isaac python (same as the scenes):

    env -u PYTHONHOME -u PYTHONPATH -u CONDA_PREFIX -u VIRTUAL_ENV \\
      PYTHONEXE=~/workspaces/isaacsim/kit/python/bin/python3 \\
      ~/workspaces/isaacsim/python.sh application/vtol/convert_meshes_to_usd.py \\
      --input /path/to/cad_dir  --output /path/to/usd_dir

    # or a single file:
    ... convert_meshes_to_usd.py --input body.stl --output body.usd

Notes
-----
* A whole directory of meshes is converted to a matching directory of .usd files.
* The converter preserves the mesh's own local origin. For articulation, each
  part's origin should sit at its JOINT PIVOT (tilt axis for a nacelle, hub for a
  prop). If a part is off-origin, fix it in your mesh tool, or use the per-link
  mesh_offset/mesh_scale in the VTOL config to compensate.
"""

import argparse
import asyncio
import os
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as asset_converter  # noqa: E402
import carb  # noqa: E402

MESH_EXTS = {".stl", ".obj", ".fbx", ".gltf", ".glb", ".dae", ".ply"}


async def _convert_one(in_path: str, out_path: str) -> bool:
    """Convert a single mesh file to USD. Returns True on success."""
    ctx = asset_converter.get_instance()
    # Default options are fine for static geometry; keep materials + up-axis as-is.
    options = asset_converter.AssetConverterContext()
    task = ctx.create_converter_task(in_path, out_path, None, options)
    ok = await task.wait_until_finished()
    if not ok:
        code = task.get_status()
        detail = task.get_detailed_error()
        carb.log_error(f"convert FAILED: {in_path} -> {out_path} (status={code}) {detail}")
    return ok


def _iter_jobs(input_path: str, output_path: str):
    """Yield (in_file, out_usd) pairs for a file or a directory of meshes."""
    if os.path.isdir(input_path):
        os.makedirs(output_path, exist_ok=True)
        for name in sorted(os.listdir(input_path)):
            stem, ext = os.path.splitext(name)
            if ext.lower() in MESH_EXTS:
                yield (os.path.join(input_path, name),
                       os.path.join(output_path, stem + ".usd"))
    else:
        out = output_path
        if os.path.isdir(output_path) or output_path.endswith(os.sep):
            os.makedirs(output_path, exist_ok=True)
            stem = os.path.splitext(os.path.basename(input_path))[0]
            out = os.path.join(output_path, stem + ".usd")
        else:
            os.makedirs(os.path.dirname(os.path.abspath(out)) or ".", exist_ok=True)
        yield (input_path, out)


def main():
    ap = argparse.ArgumentParser(description="Convert CAD mesh(es) to USD for the VTOL.")
    ap.add_argument("--input", required=True, help="mesh file OR directory of meshes")
    ap.add_argument("--output", required=True, help="output .usd file OR directory")
    args = ap.parse_args()

    jobs = list(_iter_jobs(args.input, args.output))
    if not jobs:
        carb.log_error(f"No convertible meshes found at {args.input} (exts: {sorted(MESH_EXTS)})")
        simulation_app.close()
        sys.exit(1)

    loop = asyncio.get_event_loop()
    failures = 0
    for in_file, out_usd in jobs:
        carb.log_warn(f"[convert] {in_file} -> {out_usd}")
        ok = loop.run_until_complete(_convert_one(in_file, out_usd))
        failures += 0 if ok else 1

    carb.log_warn(f"[convert] done: {len(jobs) - failures}/{len(jobs)} succeeded")
    simulation_app.close()
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
