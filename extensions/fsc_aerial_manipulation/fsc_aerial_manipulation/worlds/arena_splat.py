"""
| File: arena_splat.py
| Description: Path resolution and scene geometry for the reconstructed flight-arena
|   scene (foam-mat floor, green pillars) -- a 3D Gaussian splat captured on a phone
|   (IMG_7141), reconstructed with spirulae-splat, and rendered natively by Isaac
|   Sim's NuRec renderer.
|
|   Unlike the gate scene, metric scale, gravity and origin were NOT measured off the
|   reconstruction afterwards: three AprilTags (tag36h11, ids 0-2) with tape-measured
|   centres were in the capture, and spirulae-splat's export_ply_3dgs.py solved the
|   similarity transform onto them directly (markers.yaml). The exported splat is
|   therefore already in this frame; arena_metric.usda only cancels the USDZ
|   exporter's axis swap.
|
|   The USD asset itself is NOT in git -- see ../worlds/README.md.
|
| Scene frame (right-handed, Z up, metres) = the AprilTag board frame:
|
|       +Z  up
|        |        tag 0 at the origin, tag 1 at (0.315, 0, 0), tag 2 at (0, 0.8, 0)
|        |        the tag plane (top face of the board) is z = 0
|        o------- +Y   (toward tag 2)
|       /
|     +X   (toward tag 1)
"""
__all__ = [
    "ARENA_SPLAT_USD",
    "arena_splat_usd_path",
    "TAG_CENTRES",
    "FLOOR_Z",
    "DEFAULT_SPAWN_POS",
    "DEFAULT_SPAWN_EULER",
]

import os

import numpy as np

# --------------------------------------------------------------------------------
# Asset location
# --------------------------------------------------------------------------------

_ASSET_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "assets")
_DEFAULT_USD = os.path.join(_ASSET_DIR, "arena_metric.usda")

#: Resolved path to the metric arena scene. Override with the ARENA_SPLAT_USD env var.
ARENA_SPLAT_USD = os.path.expanduser(os.environ.get("ARENA_SPLAT_USD", _DEFAULT_USD))


def arena_splat_usd_path(path: str = None) -> str:
    """Return a validated path to the arena splat scene USD.

    Resolution order: explicit ``path`` argument, then the ``ARENA_SPLAT_USD``
    environment variable, then the in-repo asset directory.

    Raises:
        FileNotFoundError: with instructions, if the asset has not been downloaded.
    """
    resolved = os.path.expanduser(path) if path else ARENA_SPLAT_USD
    if os.path.isfile(resolved):
        # Same trap as gate_metric.usda: the wrapper references @./arena.usdz@
        # RELATIVELY, and a missing payload composes to an empty Xform with no error.
        payload = os.path.join(os.path.dirname(resolved), "arena.usdz")
        if not os.path.isfile(payload):
            raise FileNotFoundError(
                f"Found {resolved} but its payload is missing: {payload}\n"
                "arena_metric.usda references arena.usdz relatively; both files must "
                "sit in the same directory or the scene loads silently empty."
            )
        return resolved

    raise FileNotFoundError(
        f"Arena splat scene not found: {resolved}\n"
        "\n"
        "This asset is not stored in git, the same as gate.usdz and the other FSC\n"
        "USD assets. To get it:\n"
        f"  * download arena.usdz + arena_metric.usda into {_ASSET_DIR}/, or\n"
        "  * point ARENA_SPLAT_USD at an existing copy.\n"
        "See extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/README.md"
    )


# --------------------------------------------------------------------------------
# Scene geometry
#
# Mirrored here so code can import it without the (ungitted) asset present.
# Re-exporting the splat from a new capture or new markers.yaml changes these.
# --------------------------------------------------------------------------------

#: AprilTag centres (tag36h11 id -> xyz, m) as given in the capture's markers.yaml.
#: The reconstruction reproduces all three pairwise distances to 0.1 mm.
TAG_CENTRES = {
    0: np.array([0.0, 0.0, 0.0]),
    1: np.array([0.315, 0.0, 0.0]),
    2: np.array([0.0, 0.8, 0.0]),
}

#: Height of the mat floor, measured off the splat by tools/arena_splat/make_arena_usd.py
#: (densest 5 mm bin of opaque gaussians 1.5-4 m from the origin). It is below z = 0
#: because the tags lie on a ~12 mm board on top of the mats.
FLOOR_Z = -0.0125

#: On the mats where the capture started, 1.5 m in front of the tag board, yawed
#: 180 deg so body-forward (world -X) looks at the board, the green pillars and the
#: white cooler stack -- the best-covered view in the capture.
DEFAULT_SPAWN_POS = (1.5, 0.4, FLOOR_Z + 0.07)
DEFAULT_SPAWN_EULER = (0.0, 0.0, 180.0)
