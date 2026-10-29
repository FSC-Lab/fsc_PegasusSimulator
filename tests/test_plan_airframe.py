"""The build plan's aircraft is ONE description, read the same way by the plant and every controller.

    ~/envs/isaac_env/bin/python3 tests/test_plan_airframe.py

2026-10-27. scripts/plan_airframe.py writes the plan's airframe file (airframe_plan_A.cfg in the
controller package) from VTOL_Building_Plan.xlsx; the simulator BUILDS its plant from that file
(vtol_utils.airframe_vtol_config) and the controllers read it through FSC_AIRFRAME_ID. Checked here:

  1. the file is current with the workbook (regenerated, it is identical but for the timestamp line);
  2. the loader: the plan file has its rear pair and [layout]; the package's own airframe_id.cfg is
     uniform, has no [layout], and the CAD path is untouched;
  3. the PLANT built from the file assembles to exactly the file's mass, CG and inertia; each rotor
     carries its pair's propeller, speed, lag and torque ratio; the aero mapping is exact (the
     default file's [aero] rebuilds VTOLAerodynamicsConfig()'s own defaults field for field);
  4. the CONTROLLERS on the same file (a subprocess with FSC_AIRFRAME_ID set): the mass with the
     water, the hubs, the per-pair thrust<->throttle inverse of the PLANT's momentum model for each
     pair (so3 and the cascade, front and rear), the safe controller's allocation arms, the tilt
     servo scale (command x the plant's tilt_max = the angle), and the uniform-rotor guard;
  5. the tank fold: the plant's CG and inertia with the water at VTOL_TANK_DZ match so3's;
  6. [tuning] (2026-10-28): the file's retuned gains reach the safe controller and the cascade's roll
     law, the package's airframe_id.cfg has only the C++ autopilot's fsc_* keys (every Python node on its
     shipped literal), a mistyped key
     is refused, and the cascade's ROS default and its gate default both read the file.
Isaac is stubbed exactly as tests/test_slosh_import.py does; nothing here proves USD or PhysX.
"""
import importlib.util
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from controller_path import inner_dir, load_airframe, controller_dir     # noqa: E402

# the file under test: FSC_AIRFRAME_ID when it names a complete (layout) file -- the launcher runs this as
# its pre-flight gate for such a file, with the launch's own tank -- else the generated config A
_env_af = os.environ.get("FSC_AIRFRAME_ID")
PLAN = _env_af if _env_af and load_airframe(_env_af).has_layout else os.path.join(inner_dir(), "airframe_plan_A.cfg")
DEFAULT = os.path.join(inner_dir(), "airframe_id.cfg")
XLSX = os.path.expanduser("~/Downloads/VTOL_Building_Plan.xlsx")
bad = []


def check(ok, msg):
    print(("  ok    " if ok else "  FAIL  ") + msg)
    if not ok:
        bad.append(msg)


def plant_modules():
    spec = importlib.util.spec_from_file_location("tsi", os.path.join(HERE, "test_slosh_import.py"))
    tsi = importlib.util.module_from_spec(spec); spec.loader.exec_module(tsi)
    tsi.stub_isaac()
    aero = tsi.load("pegasus.simulator.logic.dynamics.vtol_aerodynamics", "dynamics/vtol_aerodynamics.py")
    tsi.load("pegasus.simulator.logic.dynamics.slosh", "dynamics/slosh.py")
    vtol = tsi.load("pegasus.simulator.logic.vehicles.vtol", "vehicles/vtol.py")
    tsi.load("pegasus.simulator.logic.vehicles.vtol_slosh", "vehicles/vtol_slosh.py")
    spec = importlib.util.spec_from_file_location(
        "vtol_utils", os.path.join(ROOT, "extensions", "fsc_aerial_manipulation", "fsc_aerial_manipulation",
                                   "rotorcraft", "vtol_utils.py"))
    vu = importlib.util.module_from_spec(spec); spec.loader.exec_module(vu)
    return vtol, aero, vu


def plant_thrust(T_or_none, rotor, cfg, thr, v_ax):
    """vtol.py's momentum thrust for `rotor` at throttle `thr` (omega settled), per its own params."""
    om_max = cfg.rotor_omega_max if rotor.omega_max is None else rotor.omega_max
    D, J0 = rotor.prop_D, max(rotor.prop_J0, 1e-3)
    n_max = om_max / (2 * math.pi)
    ct0 = rotor.max_thrust / (1.225 * n_max ** 2 * D ** 4)
    n = thr * om_max / (2 * math.pi)
    return max(0.0, 1.225 * D * D * (ct0 * (n * D) ** 2 - (ct0 / J0) * (n * D) * v_ax))


def main():
    print("1. the plan file against the workbook")
    if os.path.exists(XLSX):
        with tempfile.TemporaryDirectory() as td:
            out = os.path.join(td, "a.cfg")
            cfgletter = str(load_airframe(PLAN).layout.name)[-1]           # plan_vtol_<A|B|C>
            r = subprocess.run([sys.executable, os.path.join(ROOT, "scripts", "plan_airframe.py"), "--out", out,
                                "--config", cfgletter],
                               capture_output=True, text=True)
            check(r.returncode == 0, f"scripts/plan_airframe.py runs ({r.stderr.strip()[-200:]})")
            if r.returncode == 0:
                strip = lambda t: [l for l in t.splitlines() if not l.startswith("#   written")   # noqa: E731
                                   and not l.startswith("#   launch") and not l.startswith("#            VTOL_")]
                new = strip(open(out).read().replace(out, PLAN))
                cur = strip(open(PLAN).read())
                check(new == cur, f"{os.path.basename(PLAN)} is current with {XLSX} (else re-run the generator)")
    else:
        print(f"  skip  {XLSX} not present")

    print("2. the loader")
    af, a0 = load_airframe(PLAN), load_airframe(DEFAULT)
    check(not af.rotors.uniform and af.has_layout, "the plan file: a separate rear pair and a [layout]")
    check(a0.rotors.uniform and not a0.has_layout, "airframe_id.cfg: uniform, no [layout] (its plant stays the CAD)")
    check(all(getattr(a0.rotors, "rear_" + k) == getattr(a0.rotors, k)
              for k in ("max_thrust", "cq", "prop_d", "prop_j0", "omega_max", "tau_spin")),
          "airframe_id.cfg's rear_* default to its front values")

    print("3. the plant built from the file")
    vtol, aero_mod, vu = plant_modules()
    cfg, _ = vu.airframe_vtol_config(PLAN)
    m, cg, I = vtol.assembled_mass_properties(cfg)
    check(abs(m - af.mass.airframe_kg) < 1e-9 and np.allclose(cg, [af.mass.cg_x, 0, 0], atol=1e-12)
          and np.allclose(I, [af.mass.ixx, af.mass.iyy, af.mass.izz], atol=1e-12),
          f"assembly = the file: M {m:.4f} kg, CG {cg.round(5)}, I {I.round(4)}")
    for i, rc in enumerate(cfg.rotors):
        want = [float(af.rotors.per_rotor(k)[i]) for k in ("max_thrust", "prop_d", "prop_j0", "omega_max", "tau_spin", "cq")]
        got = [rc.max_thrust, rc.prop_D, rc.prop_J0, rc.omega_max, rc.tau_spin, rc.moment_ratio]
        check(np.allclose(want, got) and np.allclose(rc.position, af.rotors.hubs[i]),
              f"rotor {rc.name}: {rc.max_thrust:.1f} N, D {rc.prop_D:.3f}, J0 {rc.prop_J0:.2f}, "
              f"omega {rc.omega_max:.0f}, tau {rc.tau_spin:.3f}, cq {rc.moment_ratio:.4f} at {np.round(rc.position, 3)}")
    if cfg.body_mesh:                                   # dressed in the CAD quad's meshes (visual only)
        cad = vu.fsc_cad_vtol_config()
        _, cg_cad, _ = vtol.assembled_mass_properties(cad)
        by = {r.name: r for r in cad.rotors}
        check(np.allclose(cfg.body_mesh_offset, -cg_cad) and cfg.body_mesh == cad.body_mesh
              and all(rc.mesh == by[rc.name].mesh and rc.nacelle_mesh == by[rc.name].nacelle_mesh for rc in cfg.rotors)
              and cfg.body_collision_size is not None,
              f"the CAD meshes: body offset {np.round(cfg.body_mesh_offset, 4)} (the CAD's CG onto the file's), "
              f"each rotor's arm + prop on the file's hub, a box collider {cfg.body_collision_size}")
        box, _ = vu.airframe_vtol_config(PLAN, cad_meshes=False)
        check(box.body_mesh is None and np.allclose(vtol.assembled_mass_properties(box)[2], I),
              "cad_meshes=False gives the procedural boxes with the same mass properties")
    else:
        print("  skip  the CAD parts are not on this machine: procedural boxes")
    tilting = [rc for rc in cfg.rotors if rc.tiltable]
    check(all(rc.tilt_min_deg == af.tilt.min_deg and rc.tilt_max_deg == af.tilt.max_deg for rc in tilting),
          f"the tilting pair's travel {af.tilt.min_deg:+.0f}..{af.tilt.max_deg:+.0f} deg = the file's")
    d0, dd = aero_mod.VTOLAerodynamicsConfig(), vu.aero_config_from_airframe(a0)
    diff = [f for f in vars(d0) if not math.isclose(getattr(d0, f), getattr(dd, f), rel_tol=1e-12, abs_tol=1e-15)]
    check(not diff, f"the [aero] mapping is exact: airframe_id.cfg rebuilds VTOLAerodynamicsConfig() (differs: {diff})")
    ap = cfg.aerodynamics
    check((ap.wing_area, ap.wing_span, ap.mean_chord) == (af.aero.s_wing, af.aero.b_wing, af.aero.c_wing),
          f"the plan's wing in the plant: S {ap.wing_area} b {ap.wing_span} c {ap.mean_chord}")

    print("4. the controllers on the same file")
    water = float(os.environ.get("VTOL_TANK_KG") or 0.8)            # the launch's tank when run as its gate
    dz = float(os.environ.get("VTOL_TANK_DZ") or -0.076)
    env = dict(os.environ, FSC_AIRFRAME_ID=PLAN, VTOL_TANK_KG=str(water), VTOL_TANK_DX="0", VTOL_TANK_DZ=str(dz))
    probe = r"""
import json, math, sys
sys.path.insert(0, %r)
from px4_vtol_direct_actuator import nmpc_so3_transition_node as so3, nmpc_cascade_node as casc
from px4_vtol_direct_actuator import geometric_hover_node as gh
out = dict(M=so3.M, sep=so3.REAR_SEPARATE, rpos=so3.RPOS.tolist(), cgs=so3.CG_SHIFT.tolist(), I=[so3.IXX, so3.IYY, so3.IZZ],
           servo90=so3.tilt_to_servo(math.pi / 2), A=gh.allocation_matrix(0.0, so3.CQ).tolist(), af=so3._AF.digest(),
           tuning=so3._AF.tuning, safe=[gh.Gains().kR, gh.Gains().kW],
           casc=[so3._AF.tune("cascade_kR_roll", 3.0), so3._AF.tune("cascade_kW_roll", 1.2)])
pts = []
for rear in (False, True):
    for T in (6.0, 12.0, 18.0):
        for v in (0.0, 10.0):
            pts.append([rear, T, v, so3.thrust_to_throttle(T, v, rear=rear), casc.thrust_to_throttle(T, v, rear=rear)])
out["pts"] = pts
try:
    so3._AF.require_uniform("probe"); out["guard"] = False
except SystemExit:
    out["guard"] = True
print("JSON" + json.dumps(out))
""" % controller_dir()
    r = subprocess.run([os.path.expanduser("~/envs/isaac_env/bin/python3"), "-c", probe], env=env,
                       capture_output=True, text=True, timeout=600)
    line = next((l for l in r.stdout.splitlines() if l.startswith("JSON")), None)
    check(line is not None, f"the controllers import on the plan file ({r.stderr.strip()[-300:]})")
    if line:
        c = json.loads(line[4:])
        check(c["af"] == af.digest(), f"the controllers loaded the same file (af={c['af']})")
        check(abs(c["M"] - (af.mass.airframe_kg + water)) < 1e-12, f"so3.M {c['M']:.4f} = airframe + {water:g} kg of water")
        check(c["sep"] and c["guard"], "per-pair rotors on; a uniform-rotor node refuses the file")
        hubs_cg = af.rotors.hubs - np.array([af.mass.cg_x, 0.0, c["cgs"][2]])
        check(np.allclose(c["rpos"], hubs_cg), f"so3.RPOS = the file's hubs about the wet CG (z shift {c['cgs'][2]:+.4f})")
        rmax = max(abs(p[3] - p[4]) for p in c["pts"])
        check(rmax < 1e-12, f"the cascade's thrust_to_throttle == so3's, front and rear (max diff {rmax:.1e})")
        err = 0.0
        for rear, T, v, thr, _ in c["pts"]:
            rc = cfg.rotors[2 if rear else 0]
            if 0.0 < thr < 1.0:
                err = max(err, abs(plant_thrust(None, rc, cfg, thr, v) - T))
        check(err < 1e-9, f"the controllers' throttle makes the demanded thrust in the PLANT, per pair (max err {err:.1e} N)")
        tmax = cfg.rotors[0].tilt_max_deg
        check(abs(c["servo90"] * tmax - 90.0) < 1e-9,
              f"tilt: a 90 deg command is servo {c['servo90']:.4f}, which the plant ({tmax:g} deg travel) flies at 90")
        A = np.array(c["A"])
        check(np.allclose(A[1], af.rotors.hubs[:, 1]) and np.allclose(-A[2], af.rotors.hubs[:, 0] - 0.0),
              "the safe controller's allocation arms are the file's hubs")
        t = c["tuning"]
        check(t == af.tuning, f"the file's [tuning] reaches the controllers: {t}")
        check(c["safe"] == [t.get("safe_kR", 9.0), t.get("safe_kW", 1.8)],
              f"the safe controller flies the file's gains (kR, kW = {c['safe']})")
        check(c["casc"] == [t.get("cascade_kR_roll", 3.0), t.get("cascade_kW_roll", 1.2)],
              f"the cascade's roll law takes the file's gains (kR_roll, kW_roll = {c['casc']})")
        # 5. the tank fold, plant vs so3
        print("5. the water at VTOL_TANK_DZ, plant vs so3")
        import importlib
        slosh = sys.modules["pegasus.simulator.logic.dynamics.slosh"]
        os.environ.update(VTOL_TANK_KG=str(water), VTOL_TANK_R=os.environ.get("VTOL_TANK_R") or "0.10",
                          VTOL_TANK_F1=os.environ.get("VTOL_TANK_F1") or "1.10")
        tank = slosh.default_tank(slosh=False)
        scfg, _ = vu.airframe_vtol_slosh_config(PLAN, tank=tank, tank_dx=0.0, tank_dz=dz)
        vs = sys.modules["pegasus.simulator.logic.vehicles.vtol_slosh"]
        wet, before, after = vs.fold_tank_into_body(scfg, scfg.tank)
        m2, cg2, I2 = after
        shift = cg2 - before[1]
        check(abs(m2 - c["M"]) < 1e-9 and np.allclose(shift, c["cgs"], atol=1e-9),
              f"wet CG shift plant {shift.round(5)} = so3 {np.round(c['cgs'], 5)}")
        rel = np.abs(np.array(I2) / np.array(c["I"]) - 1.0)
        check(np.all(rel < 0.01), f"wet inertia plant {np.round(I2, 4)} vs so3 {np.round(c['I'], 4)} (the water's own rigid part: {np.round(100 * rel, 2)} %)")

    # 6. [tuning]: the CAD file keeps every Python node's shipped literal (the identity hashes ride on it; its
    #    own [tuning] is the C++ autopilot's fsc_* keys), a mistyped key is refused, and every site that sets
    #    these gains reads the file
    print("6. [tuning]")
    probe0 = r"""
import json, sys
sys.path.insert(0, %r)
from px4_vtol_direct_actuator import nmpc_so3_transition_node as so3, geometric_hover_node as gh
print("JSON" + json.dumps(dict(tuning=so3._AF.tuning, safe=[gh.Gains().kR, gh.Gains().kW],
                               casc=[so3._AF.tune("cascade_kR_roll", 3.0), so3._AF.tune("cascade_kW_roll", 1.2)])))
""" % controller_dir()
    env0 = {k: v for k, v in os.environ.items() if k != "FSC_AIRFRAME_ID"}
    r = subprocess.run([os.path.expanduser("~/envs/isaac_env/bin/python3"), "-c", probe0], env=env0,
                       capture_output=True, text=True, timeout=600)
    line = next((l for l in r.stdout.splitlines() if l.startswith("JSON")), None)
    c0 = json.loads(line[4:]) if line else None
    # the CAD file's [tuning] (2026-10-28 night) carries the C++ autopilot's fsc_* YAML gains only: no Python node
    # reads them, so every one stays on its shipped literal
    check(c0 is not None and all(k.startswith("fsc_") for k in c0["tuning"]) and c0["safe"] == [9.0, 1.8]
          and c0["casc"] == [3.0, 1.2],
          f"airframe_id.cfg: [tuning] holds fsc_* keys only, every Python node on its shipped gains ({c0})")
    with tempfile.TemporaryDirectory() as td:
        bad_cfg = os.path.join(td, "typo.cfg")
        with open(bad_cfg, "w") as f:
            f.write(open(PLAN).read().replace("safe_kR =", "safe_kr ="))
        try:
            load_airframe(bad_cfg); refused = False
        except ValueError:
            refused = True
    check(refused, "a mistyped [tuning] key is refused, not silently ignored")
    src = open(os.path.join(controller_dir(), "px4_vtol_direct_actuator", "nmpc_cascade_node.py")).read()
    check('P("kR_roll", so3._AF.tune("cascade_kR_roll", 3.0))' in src and 'so3._AF.tune("cascade_kR_roll", kR)' in src
          and 'P("kW_roll", so3._AF.tune("cascade_kW_roll", 1.2))' in src and 'so3._AF.tune("cascade_kW_roll", kW)' in src,
          "the cascade's ROS default AND its gate default read [tuning]")
    gsrc = open(os.path.join(controller_dir(), "px4_vtol_direct_actuator", "geometric_hover_node.py")).read()
    check('P("kR", _AF.tune("safe_kR", 9.0)); P("kW", _AF.tune("safe_kW", 1.8))' in gsrc,
          "the safe controller's ROS default reads [tuning] (Gains() is checked above)")

    print(f"\n{'PASS' if not bad else 'FAIL'}" + ("" if not bad else f" -- {len(bad)}: " + "; ".join(bad)))
    return 0 if not bad else 1


if __name__ == "__main__":
    sys.exit(main())
