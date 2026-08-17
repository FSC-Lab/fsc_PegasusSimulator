# Gate splat tooling

Regenerates the reconstructed drone-gate scene
(`extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/assets/gate_metric.usda`)
from a phone capture.

You only need this if you re-capture or re-train. To just *fly* the scene, download
the prebuilt asset — see the `worlds/README.md` linked above.

## Prerequisites

* A capture dataset: COLMAP `transforms.json` + `images/` + an exported `splat.ply`,
  produced by [spirulae-splat](https://github.com/harry7557558/spirulae-splat)
  (`extract_frames.py` -> `run_colmap.bash` -> `ns-train spirulae` ->
  `export_ply_3dgs.py`).
* `GATE_DATASET_DIR` pointing at it (default `~/Downloads/IMG_4701`).
* Two envs: one with `numpy/opencv/plyfile` (nerfstudio env works) and one with
  `pxr` (usd-core, e.g. the 3dgrut env).

## Steps

```bash
export GATE_DATASET_DIR=~/Downloads/IMG_4701

# 1. Measure the gate opening in reconstruction units -> gate_scale.json
#    Writes annotated overlays to $GATE_DATASET_DIR/measure_debug/ - LOOK AT THEM.
~/miniconda3/envs/py312/bin/python measure_gate_opening.py

# 2. Bake metric scale + gravity alignment + origin -> gate_metric.usda / .json
~/miniconda3/envs/3dgrut/bin/python make_metric_usd.py

# 3. Install into the repo
cp $GATE_DATASET_DIR/gate_metric.usda $GATE_DATASET_DIR/gate.usdz \
   ../../extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/assets/

# 4. Update the constants in worlds/gate_splat.py from the new gate_metric.json
```

Step 4 matters: `gate_splat.py` mirrors the scene geometry so it is importable
without the asset. If you regenerate and skip it, controller code will use stale
numbers while the renderer uses the new ones.

## Two traps, both hit during the original build

**Measure the banner cutout, not the frame.** The gate's timber frame has depth. All
capture views were from one side, so plain grayscale edge detection locks onto the
*back* edge of the reveal and inflates the opening by ~0.2 units. `measure_gate_opening.py`
therefore fits the **blue banner boundary** via an HSV mask — the front-face aperture.

**Verify on held-out views.** Reprojection error on the frames used in the solve is
not evidence. Project the triangulated corners into frames that were *not* used; the
first attempt looked fine on solve views and was 140 px out on held-out ones.

## Which dimension is 1.50 m?

The opening is landscape (1.49 x 1.17 m), so "the opening is 1.50 m" is ambiguous.
Current scale assumes **width**. The discriminator was camera height: width=1.50 m
puts the phone at 1.61 m above the floor (a standing person), whereas height=1.50 m
would demand 1.95 m. If a field measurement contradicts this, set
`KNOWN_WIDTH_M`/`MEASURED_WIDTH_UNITS` in `make_metric_usd.py` accordingly and
regenerate.

The robust replacement is an AprilTag capture: >= 3 non-collinear tags with measured
centres plus a `markers.yaml`, which makes spirulae-splat's `export_ply_3dgs.py`
solve scale, rotation and translation directly.
