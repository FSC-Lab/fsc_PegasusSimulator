#!/usr/bin/env python3
"""Tank-plan phase 3 gate: the pendulum analogue matches the closed form and adds no mass.

    python3 tests/test_slosh_pendulum.py

Imports the SHIPPED dynamics/slosh.py by file path -- not a re-derivation.
"""
import importlib.util
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SLOSH_PY = os.path.join(HERE, "..", "extensions", "pegasus.simulator", "pegasus", "simulator",
                        "logic", "dynamics", "slosh.py")
_spec = importlib.util.spec_from_file_location("slosh", SLOSH_PY)
slosh = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(slosh)

DT = 1.0 / 250.0          # the sim's physics step


def free_decay(params, seconds, f_axial=slosh.G, x0=0.01):
    pend = slosh.PendulumSlosh(params)
    pend.x = [x0, 0.0]
    zc, peak = [], 0.0
    for k in range(int(seconds / DT)):
        xp = pend.x[0]
        pend.step([0.0, 0.0, f_axial], DT)
        if xp * pend.x[0] < 0.0:
            zc.append(k * DT)
        peak = max(peak, abs(pend.x[0]))
    f = 1.0 / (2.0 * (zc[-1] - zc[0]) / (len(zc) - 1)) if len(zc) > 2 else 0.0
    return pend, f


def main():
    bad = []
    tank = slosh.TankConfig()
    p = slosh.slosh_parameters(tank)

    # 1. frequency against the closed form, to < 2% (the plan's gate)
    pend, f = free_decay(p, 20.0)
    print(f"  free decay {f:.3f} Hz vs closed form {p.freq_hz:.3f} Hz")
    if abs(f / p.freq_hz - 1.0) > 0.02:
        bad.append(f"frequency {f:.3f} vs closed form {p.freq_hz:.3f}: > 2%")

    # 2. damping: half amplitude every half_amplitude_cycles
    n_cycles = 20.0 * p.freq_hz
    expect = 0.01 * 0.5 ** (n_cycles / p.half_amplitude_cycles)
    # amplitude now: read the envelope from energy (robust to phase)
    amp = math.sqrt(2.0 * pend.energy() / (p.slosh_mass * p.omega ** 2))
    print(f"  after 20 s: amplitude {1000 * amp:.2f} mm, expected {1000 * expect:.2f} (zeta {100 * p.zeta:.2f}%)")
    if abs(amp / expect - 1.0) > 0.15:
        bad.append(f"decay rate off: {1000 * amp:.2f} mm vs {1000 * expect:.2f} expected")

    # 3. undamped: energy bounded over 200 cycles (symplectic integrator)
    p0 = slosh.slosh_parameters(tank.baffled(0.0))
    pu = slosh.PendulumSlosh(p0); pu.x = [0.01, 0.0]; E0 = pu.energy()
    for _ in range(int(60.0 / DT)):
        pu.step([0.0, 0.0, slosh.G], DT)
    drift = pu.energy() / E0 - 1.0
    print(f"  undamped, 60 s: energy drift {100 * drift:+.2f}%")
    if abs(drift) > 0.05:
        bad.append(f"undamped pendulum gains/loses {100 * drift:.1f}% of its energy in 60 s")

    # 4. NO DOUBLE COUNTING: a steady lateral specific force must leave zero slosh force
    #    (the dead-mass fold already carries it), with the bob hung at -f/w^2
    ps = slosh.PendulumSlosh(p); F = [0.0, 0.0, 0.0]
    for _ in range(int(30.0 / DT)):
        F = ps.step([2.0, 0.0, slosh.G], DT)
    x_exp = -2.0 / p.omega ** 2
    print(f"  steady 2 m/s^2 lateral: F_slosh {F[0]:+.5f} N, bob {1000 * ps.x[0]:+.2f} mm (expect {1000 * x_exp:+.2f})")
    if abs(F[0]) > 1e-3:
        bad.append(f"steady lateral acceleration leaves {F[0]:.4f} N of slosh force -- double counting")
    if abs(ps.x[0] - x_exp) > 0.02 * abs(x_exp):
        bad.append("static bob offset does not match -f/omega^2")

    # 5. the restoring field is the AXIAL SPECIFIC FORCE: 2 g rings sqrt(2) faster, 0 g not at all
    _, f2 = free_decay(p, 10.0, f_axial=2.0 * slosh.G)
    print(f"  at 2 g: {f2:.3f} Hz (expect {p.freq_hz * math.sqrt(2):.3f})")
    if abs(f2 / (p.freq_hz * math.sqrt(2)) - 1.0) > 0.02:
        bad.append("frequency does not scale with sqrt(axial specific force)")
    if slosh.PendulumSlosh(p).omega(0.0) != 0.0:
        bad.append("a tank in free fall should have no slosh mode")

    # 6. an empty tank is inert
    pe = slosh.PendulumSlosh(slosh.slosh_parameters(slosh.TankConfig(fill_frac=0.0)))
    if any(abs(v) > 0 for v in pe.step([3.0, 3.0, slosh.G], DT)):
        bad.append("an empty tank produced a slosh force")

    # 7. the force is the bob's inertia: after a lateral STEP the first-step force is -m1*x_ddot = +m1*f
    pk = slosh.PendulumSlosh(p)
    F1 = pk.step([1.0, 0.0, slosh.G], DT)
    print(f"  first step after a 1 m/s^2 lateral step: F {F1[0]:+.4f} N (expect +m1 = {p.slosh_mass:+.4f})")
    if abs(F1[0] - p.slosh_mass) > 1e-3:
        bad.append("first-step slosh force is not m1 * f")

    # E. the wall: a lateral push far beyond what the linear model can carry parks the bob at
    #    R/2 (plus the contact's static compression), at rest, and the reaction is finite --
    #    not an unbounded pendulum (an 80 mm nose tank on the gate integrated to 1e12 mm
    #    before the wall existed)
    pw = slosh.PendulumSlosh(p)
    for _ in range(2000):
        pw.step([20.0, 0.0, 9.81], 1e-3)          # 2 g sideways, 2 s
    print(f"  wall: bob at {1000 * pw.x[0]:+.1f} mm after 2 s at 2 g sideways (wall {1000 * p.wall:.1f} mm, "
          f"static compression {1000 * 20.0 / p.wall_omega ** 2:.1f} mm), v {pw.v[0]:+.4f} m/s")
    if not (p.wall <= abs(pw.x[0]) <= p.wall + 20.0 / p.wall_omega ** 2 + 1e-6) or abs(pw.v[0]) > 1e-3:
        bad.append("the bob is not resting on the wall")
    if p.wall <= 0.0 or p.wall > 0.5 * 0.0414 + 1e-9:
        bad.append(f"wall {p.wall} is not half the radius")
    # E2. the wall is COMPLIANT (2026-09-14): an impact is a finite force over tens of ms, not
    #     m1 v / dt in one step (17 N at 1/240 s from the 100 mm tank's 0.41 kg bob at 0.17 m/s,
    #     bag ..234537 -- the hover stop rang at ~1 Hz on those jolts). Drive the bob into the
    #     wall at 0.5 m/s at the sim's physics step and require: peak reaction under half the
    #     one-step arrest, contact longer than 5 steps, penetration under 10 mm, and the bob
    #     leaves the wall (repulsive-only contact, the pendulum's restoring force acts throughout).
    dt = 1.0 / 240.0; v0 = 0.5
    pi_ = slosh.PendulumSlosh(p); pi_.x[0] = p.wall - 0.002; pi_.v[0] = v0
    pk = pen = 0.0; n_c = 0; left = False; was_in = False
    for _ in range(int(0.4 / dt)):
        F = pi_.step([0.0, 0.0, 9.81], dt)
        inside = pi_.x[0] > p.wall
        n_c += inside; left = left or (was_in and not inside); was_in = inside
        pk = max(pk, abs(F[0])); pen = max(pen, pi_.x[0] - p.wall)
    arrest = p.slosh_mass * v0 / dt
    print(f"  impact at {v0} m/s: peak reaction {pk:.2f} N (one-step arrest {arrest:.1f}), contact {n_c} steps = {1000 * n_c * dt:.0f} ms, "
          f"penetration {1000 * pen:.2f} mm, bob left the wall: {left}")
    if not (pk < 0.5 * arrest and n_c >= 5 and pen < 0.010 and left):
        bad.append("the wall is not a compliant, finite contact")
    if p.wall_omega <= 0.0:
        bad.append("wall_omega must be > 0 (the one-step arrest is the model this replaced)")
    # F. rotation terms: a pure yaw rate about a tank 0.3 m ahead is a centripetal specific
    #    force toward the CG (-x), i.e. the bob is thrown forward
    pr = slosh.PendulumSlosh(p)
    for _ in range(200):
        pr.step([0.0, 0.0, 9.81], 1e-3, omega=[0.0, 0.0, 2.0], omega_dot=[0.0, 0.0, 0.0], r_tank=[0.3, 0.0, 0.0])
    print(f"  rotation: 2 rad/s of yaw with the tank 0.3 m ahead -> f_x seen by the tank {pr.f_last[0]:+.2f} m/s^2 (expect -1.20), bob x {1000 * pr.x[0]:+.2f} mm")
    if abs(pr.f_last[0] + 1.2) > 1e-6 or pr.x[0] <= 0.0:
        bad.append("omega x (omega x r) is not reaching the tank")

    print()
    if bad:
        print(f"FAIL ({len(bad)}):")
        for b in bad:
            print("  -", b)
        return 1
    print("PASS -- the pendulum is the closed-form mode and adds no mass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
