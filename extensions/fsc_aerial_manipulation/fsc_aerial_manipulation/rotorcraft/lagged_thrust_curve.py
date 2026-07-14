#!/usr/bin/env python
"""
| File: lagged_thrust_curve.py
| Author: Longhao Qian (longhao.qian@mail.utoronto.ca)
| License: BSD-3-Clause
| Copyright (c) 2026, Longhao Qian. All rights reserved.
| Description: QuadraticThrustCurve variant with first-order rotor spin-up dynamics, calibrated
|   from the MN4014+15x5" propeller bench test's Step 3 lambda model
|   (docs/propeller_testing/MN_4014_15x5_report.pdf). Subclasses the stock
|   pegasus.simulator QuadraticThrustCurve (not modified) so Iris and every other vehicle using
|   the stock class are completely unaffected.
"""

import numpy as np

from pegasus.simulator.logic.thrusters.quadratic_thrust_curve import QuadraticThrustCurve


class LaggedQuadraticThrustCurve(QuadraticThrustCurve):
    """
    Same force/torque model as QuadraticThrustCurve (force = rotor_constant*omega^2, yaw torque
    via rolling_moment_coefficient*omega^2*rot_dir), but the rotor angular velocity converges to
    its commanded value through a first-order lag instead of jumping to it instantaneously:

        omega_dot = rotor_lambda * (omega_cmd - omega)

    omega_cmd is the same min/max-clamped input_reference QuadraticThrustCurve itself would have
    applied directly. This matches the bench-measured first-order RPM step-response model,
    `RPM_rate = lambda*(RPM_cmd - RPM)` (Step 3, docs/propeller_testing/ reports) - lambda is
    scale-invariant between RPM and rad/s (both sides of the equation scale by the same
    rpm<->rad/s conversion factor), so the same numeric lambda applies directly to omega here.

    The per-step update is the *exact* discrete solution (zero-order hold on omega_cmd/
    rotor_lambda over each step), not a first-order Euler approximation:

        omega[k+1] = omega_cmd[k] + (omega[k] - omega_cmd[k]) * exp(-rotor_lambda[k] * dt)

    This is unconditionally stable for any rotor_lambda, dt > 0 (Euler integration is only
    conditionally stable - it can overshoot/oscillate numerically if rotor_lambda*dt isn't
    small), and stays exact even if rotor_lambda is later made to vary with operating point
    (as long as it's treated as constant within a single step, i.e. evaluated once at the start
    of the step - the usual zero-order-hold assumption for a per-physics-step update).

    Force is always computed from the lagged omega (never from omega_cmd/input_reference
    directly) - the whole point of this class.

    Default rotor_lambda=10.51 rad/s is the mean of the MN4014+15x5" report's `lag_all`/
    `lag_filtered` variants (10.519/10.501) - the two system-ID estimates derived directly from
    the measured step-response timing (lambda = 1/tau from the throttle-crossing lag), not
    `direct_all`/`direct_filtered` (noisier, up to ~24% unphysical negative values, NRMSE>1 - see
    the report's own "Lambda Fit Metrics Interpretation" guide). `lag_all`/`lag_filtered` are
    0% negative and NRMSE=0.355 ("well-conditioned", <0.5 per the same guide), landing squarely
    in the report's own physically-expected 5-15 rad/s range (tau ~ 70-200ms).
    """

    def __init__(self, config={}):
        super().__init__(config=config)

        self.rotor_lambda = config.get("rotor_lambda", [10.51] * self._num_rotors)
        assert len(self.rotor_lambda) == self._num_rotors

    def update(self, state, dt: float):
        """
        Note: see QuadraticThrustCurve.update() - state is unused here too, kept for interface
        compatibility.

        Args:
            state (State): The current state of the vehicle (unused).
            dt (float): The time elapsed between the previous and current function calls (s).
        """

        rolling_moment = 0.0

        for i in range(self._num_rotors):

            # Commanded (clamped) rotor velocity - identical clamp QuadraticThrustCurve applies.
            target_velocity = np.maximum(
                self.min_rotor_velocity[i], np.minimum(self._input_reference[i], self.max_rotor_velocity[i])
            )

            # First-order lag toward the commanded velocity - exact discrete (zero-order-hold)
            # solution, not Euler integration: unconditionally stable for any rotor_lambda[i],
            # dt > 0, and exact even if rotor_lambda[i] varies step-to-step with operating point.
            decay = np.exp(-self.rotor_lambda[i] * dt)
            self._velocity[i] = target_velocity + (self._velocity[i] - target_velocity) * decay
            # Defensive clamp - the exact solution can't overshoot past target_velocity for a
            # fixed target/lambda within one step, but this still guards against min/max having
            # changed between calls (or target_velocity itself moving on the next call).
            self._velocity[i] = np.maximum(
                self.min_rotor_velocity[i], np.minimum(self._velocity[i], self.max_rotor_velocity[i])
            )

            # Set the force using a quadratic thrust curve
            self._force[i] = self._rotor_constant[i] * np.power(self._velocity[i], 2)

            # Compute the rolling moment coefficient
            rolling_moment += self._rolling_moment_coefficient[i] * np.power(self._velocity[i], 2.0) * self._rot_dir[i]

        # Update the rolling moment variable
        self._rolling_moment = rolling_moment

        # Return the forces and velocities on each rotor and total torque applied on the body frame
        return self._force, self._velocity, self._rolling_moment
