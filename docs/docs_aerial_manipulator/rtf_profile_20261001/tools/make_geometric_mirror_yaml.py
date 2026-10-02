#!/usr/bin/env python3
"""make_geometric_mirror_yaml.py -- the geometric+L1 (Cai et al., CEP 2025) controller
yaml for the SAME-CONDITION RTF-1 comparison against the whole-body and modular rigs.

The whole-body rig flies its HARDWARE controller config on the flight-identified MIRROR
plant (params_..._whole_body_l1_4d_..._t650_sim.yaml), and the modular yaml is generated
from that file. The committed geometric _sim.yaml instead carries the older stress plant
(mass/inertia x1.10, CoM 10/10/5 mm, arm x1.05) and a sim allocator (+17.6 % kf, sim km),
with law gains retuned on that plant at RTF 0.48. This script builds the equivalent:

  start           the committed geometric _sim.yaml (the Isaac topology)
  controller      every key it shares with the geometric HARDWARE yaml takes the
                  hardware value (the config flown on 2026-09-28): allocator kf/km,
                  SAFETY thrust map, UDE gate, k_R/k_w 1.0/0.55, armff_com_trim_x
                  -14.8 mm, measured-stamp L1 sample time
  guards          system_wd_* as the whole-body mirror (20 deg / 0.75 m)
  plant           every sim_* key replaced by the whole-body mirror's, verbatim
                  (rotor lag, kf scale/sag, km scale, force/torque bias, CoM, arm
                  friction/current noise/velocity lag, mocap feed noise,
                  wall-clock compensation)
  vehicle_name    "AM-T650-L1-MIRROR"

and CHECKS all four. Nothing committed is modified.

    /usr/bin/python3 make_geometric_mirror_yaml.py [--out ../variants/geometric_l1_mirror_sim.yaml]
"""
import argparse
import os
import re

CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
G_SIM = os.path.join(CFG, "params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim.yaml")
G_HW = os.path.join(CFG, "params_single_aerial_manipulator_geometric_l1_direct_actuation_t650.yaml")
W_MIR = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml")
KEY = re.compile(r"^(\s+)([a-z][a-z0-9_]*):\s*([^#\n]*?)\s*(#.*)?$")


def scalars(path, section="/**/fsc_autopilot_ros2:"):
    """key -> value string inside the node's ros__parameters block (first occurrence)."""
    out, inside = {}, False
    for line in open(path):
        if line.startswith("/**/"):
            inside = line.strip() == section
            continue
        m = KEY.match(line)
        if inside and m and m.group(2) not in out and m.group(2) != "ros__parameters":
            out[m.group(2)] = m.group(3)
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(here, "..", "variants", "geometric_l1_mirror_sim.yaml"))
    a = ap.parse_args()
    gs, gh, wm = scalars(G_SIM), scalars(G_HW), scalars(W_MIR)
    ctrl = {k: gh[k] for k in gs if k in gh and not k.startswith(("sim_", "system_wd_")) and k != "vehicle_name"}
    guards = {k: wm[k] for k in wm if k.startswith("system_wd_")}
    plant = {k: v for k, v in wm.items() if k.startswith("sim_")}

    lines, out, plant_done, changed = open(G_SIM).read().splitlines(), [], False, []
    for line in lines:
        m = KEY.match(line)
        if not m:
            out.append(line)
            continue
        ind, k, v = m.group(1), m.group(2), m.group(3)
        if k.startswith("sim_"):
            if not plant_done:   # the whole mirror plant block replaces the first sim_ key
                out.append(f"{ind}# ── PLANT = the whole-body MIRROR's sim_* keys, verbatim "
                           f"(rationale and per-flight numbers: section 1 of")
                out.append(f"{ind}# {os.path.basename(W_MIR)}) ──")
                out += [f"{ind}{pk}: {pv}" for pk, pv in plant.items()]
                plant_done = True
            continue
        if k == "vehicle_name":
            new = '"AM-T650-L1-MIRROR"'
        elif k in guards:
            new = guards[k]
        elif k in ctrl:
            new = ctrl[k]
        else:
            new = v
        if new != v:
            changed.append((k, v, new))
        out.append(f"{ind}{k}: {new}" + (f"  {m.group(4)}" if m.group(4) and new == v else ""))
    for k in guards:   # guard keys the sim file lacks
        if k not in gs:
            raise SystemExit(f"guard {k} missing from {G_SIM}")
    header = [
        "# " + "=" * 76,
        "# GENERATED -- do not hand-edit. fsc_PegasusSimulator",
        "#   docs/docs_aerial_manipulator/rtf_profile_20261001/tools/make_geometric_mirror_yaml.py",
        "# The geometric+L1 controller on the whole-body MIRROR plant, for the RTF-1",
        "# same-condition comparison: geometric HARDWARE controller values, the mirror's",
        "# guards and sim_* plant. Sources:",
        f"#   {os.path.basename(G_SIM)} (topology)",
        f"#   {os.path.basename(G_HW)} (controller values)",
        f"#   {os.path.basename(W_MIR)} (plant + guards)",
        "# " + "=" * 76, ""]
    open(a.out, "w").write("\n".join(header + out) + "\n")

    # ---- checks on the written file ----
    v = scalars(a.out)
    assert {k: x for k, x in v.items() if k.startswith("sim_")} == plant, "plant differs from the mirror"
    for k, x in ctrl.items():
        assert v[k] == x, f"controller key {k}: {v[k]} != hardware {x}"
    for k, x in guards.items():
        assert v[k] == x, f"guard {k}"
    shared = [k for k in v if k in wm and not k.startswith("sim_") and k != "vehicle_name"]
    diff = [(k, v[k], wm[k]) for k in shared if v[k] != wm[k]]
    print(f"wrote {os.path.normpath(a.out)}")
    print(f"  plant: {len(plant)} sim_ keys == the whole-body mirror's")
    print(f"  controller: {len(ctrl)} keys == the geometric hardware yaml; guards {guards}")
    print(f"  shared non-plant keys vs the whole-body mirror: {len(shared)}, differing: {diff or 'none'}")
    print("  changed from the committed _sim.yaml:")
    for k, old, new in changed:
        print(f"    {k}: {old} -> {new}")


if __name__ == "__main__":
    main()
