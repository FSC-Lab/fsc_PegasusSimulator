#!/usr/bin/env python3
"""Closed-loop tests for the SAFE geometric hover controller, against the plant double.

Unlike the controller self-tests in ros2/px4_vtol_direct_actuator/ -- which integrate each
controller's OWN model and therefore cannot see a model error -- this drives the real
control law against `tests/plant_double.py`, which is built from the simulator's own
aerodynamics module and the assembled mass properties of the CAD airframe.

    ~/envs/isaac_env/bin/python3 tests/test_geometric_hover.py [--latency 0.1]

No Isaac, no ROS.
"""
import math
import os
import sys
import importlib.util

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from plant_double import VTOLPlant, MASS, G                       # noqa: E402

_spec = importlib.util.spec_from_file_location(
    "geometric_hover_node",
    os.path.join(_HERE, "..", "ros2", "px4_vtol_direct_actuator", "px4_vtol_direct_actuator",
                 "geometric_hover_node.py"))
_gh = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_gh)

FAILURES = []


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"   {detail}" if detail else ""))
    if not ok:
        FAILURES.append(name)


def fly(targets, duration, start=(0.0, 0.0, 5.0), start_yaw=0.0, ctrl_hz=200.0,
        plant_hz=1000.0, latency=0.0, disturb=None, gains=None, ramp_kw=None, log_every=None):
    """Run the closed loop. `targets` = [(t_switch, x, y, z, yaw_deg), ...].

    Returns a dict of time series. `latency` [s] delays the actuator command reaching the
    plant, which is what the HIL link does.
    """
    gains = gains or _gh.Gains()
    ramp_kw = dict(v_max=4.0, vz_up=2.0, vz_dn=1.0, a_max=1.5, az_max=1.5,
                   yaw_rate_max=math.radians(45.0), **(ramp_kw or {}))
    Ainv = np.linalg.pinv(_gh.allocation_matrix(gains.cg_x, gains.cQ))
    ramp = _gh.SetpointRamp()
    ramp.seed(start, start_yaw)
    plant = VTOLPlant()
    R0 = np.array([[math.cos(start_yaw), -math.sin(start_yaw), 0],
                   [math.sin(start_yaw), math.cos(start_yaw), 0], [0, 0, 1]])
    plant.reset(pos=start, R=R0)
    if disturb:
        for k, val in disturb.items():
            setattr(plant, k, np.array(val, dtype=float))

    dt_c = 1.0 / ctrl_hz
    dt_p = 1.0 / plant_hz
    sub = max(1, int(round(dt_c / dt_p)))
    ei = np.zeros(3)
    hover = float(np.clip(MASS * G / 4.0 / 18.0, 0.0, 1.0))
    delay_n = max(0, int(round(latency / dt_c)))
    queue = [[hover] * 4] * (delay_n + 1)
    out = {k: [] for k in ("t", "p", "sp", "tgt", "v", "rpy", "mot", "err")}

    n_steps = int(duration * ctrl_hz)
    for k in range(n_steps):
        t = k * dt_c
        tx = [g for g in targets if g[0] <= t][-1]
        target = np.array(tx[1:4], dtype=float)
        yaw_t = math.radians(tx[4])
        p_sp, v_ff, a_ff, yaw_sp = ramp.step(target, yaw_t, dt_c, ramp_kw["v_max"],
                                             ramp_kw["vz_up"], ramp_kw["vz_dn"],
                                             ramp_kw["a_max"], ramp_kw["az_max"],
                                             ramp_kw["yaw_rate_max"])
        mot, ei, _ = _gh.geometric_command(
            plant.pos, plant.vel, plant.R, plant.omega, p_sp, v_ff, a_ff, yaw_sp,
            ramp.yaw_rate, ei, dt_c, gains, Ainv, integrate=True)
        queue.append(mot)
        applied = queue[-(delay_n + 1)]
        queue = queue[-(delay_n + 2):]
        for _ in range(sub):
            plant.step(applied, 0.0, [0.0, 0.0, 0.0], dt_p)
        out["t"].append(t); out["p"].append(plant.pos.copy()); out["sp"].append(p_sp)
        out["tgt"].append(target.copy()); out["v"].append(plant.vel.copy())
        out["rpy"].append(plant.rpy); out["mot"].append(list(applied))
        out["err"].append(float(np.linalg.norm(plant.pos - target)))
        if log_every and k % int(log_every * ctrl_hz) == 0:
            r, pi, y = plant.rpy
            print(f"      t={t:5.1f} p=({plant.pos[0]:7.2f},{plant.pos[1]:6.2f},{plant.pos[2]:6.2f}) "
                  f"tgt=({target[0]:6.1f},{target[1]:5.1f},{target[2]:5.1f}) "
                  f"|v|={np.linalg.norm(plant.vel):4.1f} rpy=({r:+5.1f},{pi:+5.1f},{y:+6.1f}) "
                  f"err={out['err'][-1]:5.2f}")
    for k in out:
        out[k] = np.array(out[k])
    return out


# ---------------------------------------------------------------------------
def test_hover_hold(lat):
    print("\n1. hover hold — no target change, 20 s")
    r = fly([(0.0, 0.0, 0.0, 5.0, 0.0)], 20.0, start=(0.0, 0.0, 5.0), latency=lat)
    settled = r["err"][-int(len(r["err"]) * 0.25):]
    check("holds within 5 cm", float(np.max(settled)) < 0.05, f"max {np.max(settled):.3f} m")
    check("attitude stays level", float(np.max(np.abs(r["rpy"][:, :2]))) < 5.0,
          f"max |roll|,|pitch| {np.max(np.abs(r['rpy'][:, :2])):.1f} deg")
    # ★ HOVER THROTTLE DEPENDS ON THE THRUST MODEL, so derive it rather than hard-coding 0.47.
    # Flat model: throttle = W/4/MAX_THRUST = 0.470. Momentum model: thrust is QUADRATIC in
    # throttle, so the same force needs sqrt(0.470) = 0.686. Pinning the old number here failed a
    # controller that had been correctly updated for the sim's momentum model.
    import plant_double as _pd
    _hov = (3.45 * 9.81 / 4.0) / _pd.MAX_THRUST
    if getattr(_pd, "THRUST_MODEL", "instant") == "momentum":
        _hov = math.sqrt(_hov)
    check("motors near hover throttle", abs(float(np.mean(r["mot"])) - _hov) < 0.12,
          f"mean {np.mean(r['mot']):.3f}")


def test_step_translate(lat):
    print("\n2. operator types (25, 12, 12) from a hover at (0,0,5)")
    r = fly([(0.0, 0.0, 0.0, 5.0, 0.0), (1.0, 25.0, 12.0, 12.0, 0.0)], 45.0, latency=lat,
            log_every=7.5)
    final = r["err"][-int(len(r["err"]) * 0.1):]
    check("arrives within 10 cm", float(np.max(final)) < 0.10, f"max {np.max(final):.3f} m")
    # overshoot: never travel PAST the target along the approach direction
    u = np.array([25.0, 12.0]) / np.linalg.norm([25.0, 12.0])
    beyond = float(np.max((r["p"][:, 0:2] - np.array([25.0, 12.0])) @ u))
    check("no overshoot past the target", beyond < 0.30, f"max {beyond:+.2f} m beyond")
    check("speed respects v_max", float(np.max(np.linalg.norm(r["v"][:, 0:2], axis=1))) < 4.6,
          f"peak {np.max(np.linalg.norm(r['v'][:, 0:2], axis=1)):.2f} m/s")
    check("lean stays inside the cap", float(np.max(np.abs(r["rpy"][:, :2]))) < 36.0,
          f"max {np.max(np.abs(r['rpy'][:, :2])):.1f} deg")


def test_altitude_hold_during_traverse(lat):
    print("\n3. altitude hold through a 40 m traverse at constant z")
    r = fly([(0.0, 0.0, 0.0, 10.0, 0.0), (1.0, 40.0, 0.0, 10.0, 0.0)], 45.0,
            start=(0.0, 0.0, 10.0), latency=lat)
    dz = np.abs(r["p"][:, 2] - 10.0)
    check("altitude held within 0.3 m", float(np.max(dz)) < 0.30, f"max deviation {np.max(dz):.3f} m")
    check("lands on the target", r["err"][-1] < 0.10, f"final error {r['err'][-1]:.3f} m")


def test_yaw(lat):
    print("\n4. yaw to 120 deg while holding position")
    r = fly([(0.0, 0.0, 0.0, 6.0, 0.0), (1.0, 0.0, 0.0, 6.0, 120.0)], 20.0, latency=lat)
    yaw_err = abs((r["rpy"][-1, 2] - 120.0 + 180) % 360 - 180)
    check("reaches the commanded yaw", yaw_err < 3.0, f"final yaw {r['rpy'][-1, 2]:.1f} deg")
    drift = float(np.max(np.linalg.norm(r["p"][:, 0:2], axis=1)))
    check("position holds through the yaw", drift < 0.35, f"max drift {drift:.3f} m")


def test_descend_and_land(lat):
    print("\n5. descend from 12 m to the ground over a point")
    r = fly([(0.0, 8.0, 4.0, 12.0, 0.0), (1.0, 8.0, 4.0, 0.35, 0.0)], 40.0, latency=lat)
    check("descends to the commanded height", abs(r["p"][-1, 2] - 0.35) < 0.20,
          f"final z {r['p'][-1, 2]:.2f} m")
    xy = float(np.linalg.norm(r["p"][-1, 0:2] - np.array([8.0, 4.0])))
    check("holds the point through the descent", xy < 0.20, f"lateral error {xy:.3f} m")
    check("descent rate respects vz_down", float(np.max(-r["v"][:, 2])) < 1.3,
          f"peak {np.max(-r['v'][:, 2]):.2f} m/s")


def test_upset_recovery(lat):
    print("\n6. recovery from a 25 deg roll + 4 m/s sideways at engage")
    roll = math.radians(25.0)
    R0 = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    r = fly([(0.0, 0.0, 0.0, 10.0, 0.0)], 20.0, start=(0.0, 0.0, 10.0), latency=lat,
            disturb={"vel": [0.0, 4.0, 0.0], "R": R0})
    check("recovers to level", float(np.max(np.abs(r["rpy"][-200:, :2]))) < 4.0,
          f"final |roll|,|pitch| {np.max(np.abs(r['rpy'][-200:, :2])):.2f} deg")
    check("returns to the hold point", r["err"][-1] < 0.20, f"final error {r['err'][-1]:.3f} m")
    check("altitude never drops below 8 m", float(np.min(r["p"][:, 2])) > 8.0,
          f"min z {np.min(r['p'][:, 2]):.2f} m")


def test_wind(lat):
    print("\n7. holds position in a 6 m/s crosswind")
    gains = _gh.Gains()
    Ainv = np.linalg.pinv(_gh.allocation_matrix(gains.cg_x, gains.cQ))
    ramp = _gh.SetpointRamp(); ramp.seed((0.0, 0.0, 12.0), 0.0)
    plant = VTOLPlant(wind_world=[0.0, 6.0, 0.0]); plant.reset(pos=(0.0, 0.0, 12.0))
    ei = np.zeros(3); dt = 1.0 / 200.0
    for k in range(200 * 40):
        p_sp, v_ff, a_ff, yaw_sp = ramp.step([0.0, 0.0, 12.0], 0.0, dt, 4.0, 2.0, 1.0,
                                             1.5, 1.5, math.radians(45.0))
        mot, ei, _ = _gh.geometric_command(plant.pos, plant.vel, plant.R, plant.omega,
                                           p_sp, v_ff, a_ff, yaw_sp, ramp.yaw_rate,
                                           ei, dt, gains, Ainv)
        for _ in range(5):
            plant.step(mot, 0.0, [0.0, 0.0, 0.0], 1.0 / 1000.0)
    err = float(np.linalg.norm(plant.pos - np.array([0.0, 0.0, 12.0])))
    print(f"      final pos ({plant.pos[0]:.2f},{plant.pos[1]:.2f},{plant.pos[2]:.2f})  "
          f"roll {plant.rpy[0]:+.1f} deg into the wind")
    check("holds station in wind (integral nulls the offset)", err < 0.35, f"error {err:.3f} m")


def test_latency_margin():
    """How much loop delay the attitude loop tolerates.

    This is the number the old self-tests could never produce, because they ran the
    controller against its own model with no transport delay. The HIL link (mocap -> DDS ->
    controller -> DDS -> PX4 -> Isaac) is the delay that matters, and the attitude loop --
    not the position loop -- is what it destabilises first.
    """
    print("\n8. loop-latency margin (hover hold + a 15 m step)")
    last_ok = 0.0
    for ms in (0, 50, 100, 125, 150):
        r = fly([(0.0, 0.0, 0.0, 8.0, 0.0), (2.0, 15.0, 0.0, 8.0, 0.0)], 25.0,
                start=(0.0, 0.0, 8.0), latency=ms / 1000.0)
        att = float(np.max(np.abs(r["rpy"][:, :2])))
        err = float(np.max(r["err"][-400:]))
        ok = att < 40.0 and err < 0.5
        print(f"      {ms:4d} ms -> max |roll|,|pitch| {att:6.1f} deg, settled error "
              f"{err:5.2f} m   {'stable' if ok else 'DIVERGED'}")
        if ok:
            last_ok = ms / 1000.0
        else:
            break
    check("stable with at least 100 ms of loop latency", last_ok >= 0.100,
          f"margin {last_ok*1000:.0f} ms")


def test_envelope_is_a_hover_controller():
    print("\n9. it is structurally a multicopter")
    src = open(os.path.join(_HERE, "..", "ros2", "px4_vtol_direct_actuator",
                            "px4_vtol_direct_actuator", "geometric_hover_node.py")).read()
    check("no tilt parameter exists", "tilt_max_deg" not in src and "tilt_for_v" not in src)
    check("servo output is hard-coded to zero", "sc[i] = 0.0" in src)
    check("the reference velocity is ignored (targets only)", "m.velocity" not in src)


if __name__ == "__main__":
    lat = 0.0
    if "--latency" in sys.argv:
        lat = float(sys.argv[sys.argv.index("--latency") + 1])
    print("geometric hover controller vs the simulator-derived plant double")
    print(f"latency = {lat*1000:.0f} ms" + ("  (HIL-like)" if lat else "  (ideal link)"))
    print("=" * 72)
    test_hover_hold(lat)
    test_step_translate(lat)
    test_altitude_hold_during_traverse(lat)
    test_yaw(lat)
    test_descend_and_land(lat)
    test_upset_recovery(lat)
    test_wind(lat)
    test_latency_margin()
    test_envelope_is_a_hover_controller()
    print("\n" + "=" * 72)
    if FAILURES:
        print(f"{len(FAILURES)} CHECK(S) FAILED:")
        for f in FAILURES:
            print("  -", f)
        sys.exit(1)
    print("all checks passed")
