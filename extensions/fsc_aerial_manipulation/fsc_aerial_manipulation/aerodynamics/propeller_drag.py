#!/usr/bin/env python
"""
| File: propeller_drag.py
| License: BSD-3-Clause
| Description: Propeller-level aerodynamic effects not covered by
|   pegasus.simulator's existing dynamics/ package -- that package only has
|   `Drag`/`LinearDrag` (a single whole-body linear drag force,
|   `-diag(dx,dy,dz) @ body_velocity`), with no rotor-level model at all.
|   Two DISTINCT effects are implemented here, both standard in the
|   multirotor-dynamics literature (e.g. Mahony/Kumar/Corke, "Multirotor
|   Aerial Vehicles", IEEE RAM 2012; Bouabdallah's thesis), neither of which
|   this repo has bench data for yet -- both are first-cut, clearly-flagged
|   placeholders pending real characterization, same status as e.g.
|   `gains_quad_att` was in the companion Automatica2026Simulation project
|   before it got tuned against a real closed loop.

1. `PropellerParasiticDrag` -- the classical rotor "H-force": each rotor's
   hub sees an apparent airflow velocity (from the vehicle's own
   translational velocity AND the rotor's own position sweeping through the
   air as the vehicle rotates, v_i = v_body + omega_body x r_i), and the
   spinning blades generate a drag force in the rotor's OWN disk plane
   (perpendicular to its thrust axis) opposing the IN-PLANE component of
   that velocity, scaled by rotor speed: F_drag,i = -k_h * |omega_i| *
   v_perp,i. This is what "propeller parasitic drag" ordinarily refers to.

   Important, verified finding (see CLAUDE.md): for a rotor mounted flat in
   the body XY-plane (r_i has zero Z-component, matching X650's layout) and
   v_body=0 (attitude-only, no translational velocity -- CoM held fixed),
   v_i = omega_body x r_i is ALWAYS purely along the rotor's own spin axis
   (Z) for a pure body rotation, not in its disk plane -- so v_perp,i = 0
   identically, and this term contributes ~zero net torque in the
   attitude-only case. That's not a bug in the model; it's the correct
   physical statement that H-force fundamentally requires horizontal
   airspeed THROUGH the rotor disk, which a body that's only rotating (not
   translating) doesn't produce. It becomes non-negligible once translation
   is reintroduced -- kept general (accepts v_body) for exactly that future
   extension, per CLAUDE.md's "Known simplifications".

2. `RotorGyroscopicTorque` -- the reaction torque from each spinning rotor's
   own angular momentum being carried around by the vehicle's body rotation:
   tau_gyro = sum_i omega_body x (J_rotor * omega_i * rot_dir_i * e_z). This
   IS non-zero for pure attitude motion (unlike the H-force term above) and
   is the standard "propeller gyroscopic coupling" term appearing in most
   textbook quadrotor rigid-body EOM derivations, on top of the vehicle's
   own -omega x J*omega gyroscopic term (which is about the WHOLE vehicle's
   inertia, not the individual spinning rotors).

k_h and J_rotor are NOT bench-measured (no such data exists in this repo's
docs/propeller_testing/ reports, which only cover Step 1 thrust/torque and
Step 3 spin-up lag). J_rotor is a rough physical estimate for the MN4014+
15x5" combo (thin-blade-pair approximation, prop mass ~15g, radius 0.1905m
-- (2/3)*0.0075kg*0.1905^2*2blades ~ 1.8e-4 kg*m^2, motor's own rotor
inertia assumed small relative to this and neglected). k_h has no
comparable estimate available and is left at a conservative placeholder;
see CLAUDE.md for the numeric sanity checks run against both.
"""
import numpy as np


def skew(v):
    v = np.asarray(v).flatten()
    return np.array([[0., -v[2], v[1]],
                      [v[2], 0., -v[0]],
                      [-v[1], v[0], 0.]])


class PropellerParasiticDrag:
    """H-force (in-rotor-plane drag) model, per rotor.

    rotor_positions : (N,3) rotor hub positions relative to the vehicle CoM,
                       body frame.
    spin_axis        : (3,) unit vector, the rotor thrust/spin axis in the
                        body frame (same for all rotors -- coaxial-tilt
                        rotors are not modeled). Defaults to body +Z.
    k_h               : drag coefficient, N / ((rad/s)*(m/s)) -- force
                         magnitude per unit (rotor speed * in-plane speed).
    """

    def __init__(self, rotor_positions, k_h, spin_axis=(0.0, 0.0, 1.0)):
        self.rotor_positions = np.asarray(rotor_positions, dtype=float)
        self.num_rotors = self.rotor_positions.shape[0]
        self.k_h = float(k_h)
        self.spin_axis = np.asarray(spin_axis, dtype=float)
        self.spin_axis = self.spin_axis / np.linalg.norm(self.spin_axis)

    def compute(self, rotor_speeds, v_body=None, omega_body=None):
        """
        rotor_speeds : (N,) rotor angular speeds [rad/s] (sign-agnostic --
                       drag opposes velocity regardless of spin direction).
        v_body        : (3,) vehicle CoM velocity in the body frame [m/s],
                         defaults to zero (attitude-only usage).
        omega_body    : (3,) vehicle body angular velocity [rad/s], defaults
                        to zero.

        Returns (forces (N,3) [N], net_force (3,) [N], net_torque (3,) [N.m]).
        """
        v_body = np.zeros(3) if v_body is None else np.asarray(v_body, dtype=float)
        omega_body = np.zeros(3) if omega_body is None else np.asarray(omega_body, dtype=float)
        omega_x = skew(omega_body)

        forces = np.zeros((self.num_rotors, 3))
        for i in range(self.num_rotors):
            r_i = self.rotor_positions[i]
            v_i = v_body + omega_x @ r_i
            v_perp = v_i - (v_i @ self.spin_axis) * self.spin_axis
            forces[i] = -self.k_h * abs(rotor_speeds[i]) * v_perp

        net_force = forces.sum(axis=0)
        net_torque = sum(skew(self.rotor_positions[i]) @ forces[i] for i in range(self.num_rotors))
        return forces, net_force, net_torque


class RotorGyroscopicTorque:
    """Reaction torque from each spinning rotor's angular momentum being
    carried around by the vehicle's own body rotation.

    rot_dir    : (N,) +-1 per rotor, spin direction sign (matches
                 ThrustCurve's rot_dir convention).
    J_rotor     : scalar or (N,) rotor+prop spin inertia about its own axis
                  [kg*m^2] -- same value for all rotors by default (assumes
                  identical motor+prop units).
    spin_axis   : (3,) unit vector, same convention as PropellerParasiticDrag.
    """

    def __init__(self, rot_dir, J_rotor, spin_axis=(0.0, 0.0, 1.0)):
        self.rot_dir = np.asarray(rot_dir, dtype=float)
        self.num_rotors = self.rot_dir.shape[0]
        self.J_rotor = np.broadcast_to(np.asarray(J_rotor, dtype=float), (self.num_rotors,))
        self.spin_axis = np.asarray(spin_axis, dtype=float)
        self.spin_axis = self.spin_axis / np.linalg.norm(self.spin_axis)

    def compute(self, rotor_speeds, omega_body):
        """rotor_speeds: (N,) [rad/s] (unsigned rotor speed; rot_dir supplies
        the sign of the resulting angular momentum). omega_body: (3,)
        [rad/s]. Returns net_torque (3,) [N.m]."""
        rotor_speeds = np.asarray(rotor_speeds, dtype=float)
        h_total = np.zeros(3)
        for i in range(self.num_rotors):
            h_i = self.J_rotor[i] * self.rot_dir[i] * rotor_speeds[i] * self.spin_axis
            h_total += h_i
        return skew(omega_body) @ h_total


# ── First-cut placeholders (NOT bench-measured; see module docstring) ──────
K_H_PLACEHOLDER = 1.0e-6      # N / ((rad/s)*(m/s)) -- conservative, unvalidated
J_ROTOR_PLACEHOLDER = 1.8e-4  # kg*m^2 -- thin-blade-pair estimate for MN4014+15x5"
