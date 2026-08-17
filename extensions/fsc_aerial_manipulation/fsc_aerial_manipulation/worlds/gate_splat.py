"""
| File: gate_splat.py
| Description: Path resolution and scene geometry for the reconstructed A2RL x DCL
|   drone-gate scene -- a 3D Gaussian splat captured on a phone, reconstructed with
|   spirulae-splat, and rendered natively by Isaac Sim's NuRec renderer.
|
|   The USD asset itself is NOT in git. Like every other FSC custom USD
|   (x650_new.usd, AM_xfwd.usda, ...) it lives under an ``assets/`` directory that
|   .gitignore excludes, and is distributed out-of-band. See ../worlds/README.md
|   for how to obtain or regenerate it.
|
|   This module IS in git, so a fresh clone can import the scene constants (gate
|   position, opening size, fly-through axis) even before the asset is downloaded --
|   controller and trajectory code should import them from here rather than
|   hardcoding numbers.
|
| Scene frame (right-handed, Z up, metres):
|
|       +Z  up
|        |        the gate plane is the YZ plane at x = 0
|        |        flying THROUGH the gate is motion along X
|        o------- +Y
|       /
|     +X   the side the capture was made from; the drone spawns here
|
|   floor            z = 0
|   opening centre   (0, 0, 1.746)
|   opening          1.493 m wide x 1.165 m tall
|   sill (lower lip) z = 1.102
|   lintel (upper)   z = 2.328
"""
__all__ = [
    "GATE_SPLAT_USD",
    "gate_splat_usd_path",
    "OPENING_CENTRE",
    "OPENING_WIDTH",
    "OPENING_HEIGHT",
    "SILL_HEIGHT",
    "LINTEL_HEIGHT",
    "FLY_THROUGH_AXIS",
    "APPROACH_SIDE",
    "SCALE_M_PER_UNIT",
    "DEFAULT_SPAWN_POS",
    "DEFAULT_SPAWN_EULER",
]

import os

import numpy as np

# --------------------------------------------------------------------------------
# Asset location
# --------------------------------------------------------------------------------

_ASSET_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), "assets")
_DEFAULT_USD = os.path.join(_ASSET_DIR, "gate_metric.usda")

#: Resolved path to the metric gate scene. Override with the GATE_SPLAT_USD env var.
GATE_SPLAT_USD = os.path.expanduser(os.environ.get("GATE_SPLAT_USD", _DEFAULT_USD))


def gate_splat_usd_path(path: str = None) -> str:
    """Return a validated path to the gate splat scene USD.

    Resolution order: explicit ``path`` argument, then the ``GATE_SPLAT_USD``
    environment variable, then the in-repo asset directory.

    Raises:
        FileNotFoundError: with instructions, if the asset has not been downloaded.
            The asset is deliberately not in git (105 MB); see worlds/README.md.
    """
    resolved = os.path.expanduser(path) if path else GATE_SPLAT_USD
    if os.path.isfile(resolved):
        # gate_metric.usda references @./gate.usdz@ RELATIVELY - a missing payload
        # composes to an empty Xform and renders nothing, with no USD error at all,
        # so check for it here where the failure is still explainable.
        payload = os.path.join(os.path.dirname(resolved), "gate.usdz")
        if not os.path.isfile(payload):
            raise FileNotFoundError(
                f"Found {resolved} but its payload is missing: {payload}\n"
                "gate_metric.usda references gate.usdz relatively; both files must sit "
                "in the same directory or the scene loads silently empty."
            )
        return resolved

    raise FileNotFoundError(
        f"Gate splat scene not found: {resolved}\n"
        "\n"
        "This asset is not stored in git (105 MB), the same as x650_new.usd and the\n"
        "other FSC USD assets. To get it:\n"
        f"  * download gate.usdz + gate_metric.usda into {_ASSET_DIR}/\n"
        "    (same out-of-band asset bundle as the vehicle USDs), or\n"
        "  * regenerate from a capture with tools/gate_splat/make_metric_usd.py, or\n"
        "  * point GATE_SPLAT_USD at an existing copy.\n"
        "See extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/README.md"
    )


# --------------------------------------------------------------------------------
# Scene geometry
#
# Produced by tools/gate_splat/make_metric_usd.py and mirrored here so that code can
# import them without the (ungitted) asset present. Regenerating the scene from a new
# capture will change these -- keep them in step with the generated gate_metric.json.
# --------------------------------------------------------------------------------

#: Metric scale applied to the reconstruction, from the gate's measured 1.50 m inner
#: opening WIDTH. NOTE the opening is landscape (1.49 x 1.17 m); if the 1.50 m field
#: measurement was actually the HEIGHT, everything rescales by ~1.22 and this whole
#: block must be regenerated.
SCALE_M_PER_UNIT = 0.8130081300813008

OPENING_CENTRE = np.array([0.0, 0.0, 1.7460040559440315])
OPENING_WIDTH = 1.4934363914511652
OPENING_HEIGHT = 1.1654694727319996
SILL_HEIGHT = 1.1016222741403174
LINTEL_HEIGHT = 2.3283871430440226

#: Motion through the gate is along world X; the gate plane is the YZ plane at x=0.
FLY_THROUGH_AXIS = np.array([1.0, 0.0, 0.0])
#: The gate face the capture (and hence the best reconstruction quality) is on.
APPROACH_SIDE = +1.0

#: Spawn 3 m out on the approach side, yawed 180 deg so body-forward faces the gate.
DEFAULT_SPAWN_POS = (3.0, 0.0, 0.07)
DEFAULT_SPAWN_EULER = (0.0, 0.0, 180.0)
