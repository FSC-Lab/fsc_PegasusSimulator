"""
| File: mt2216_quadrotor_thrust_curve.py
| Propeller : T-Motor MT2216 / 10×4.5 (10 inch diameter, 4.5 inch pitch)
| Description: Physics-based thrust curve implementing
|     (1) steady-state RPM as a linear function of throttle % (bench test, R²=0.998)
|     (2) first-order RPM lag whose bandwidth λ depends on current RPM and RPM_rate,
|         fitted as a bivariate polynomial surface from step-response bench data
|         (lag_filtered variant: |RPM_rate| ≤ 5 000 RPM/s, R²=0.526, RMSE=2.64 s⁻¹)
|
| Model equation (from bench test report MT_2216_10x4.5_report.pdf):
|     dRPM/dt = λ(RPM, RPM_rate) · (RPM_cmd − RPM)
|
| Source data:  extensions/.../thrusters/propeller_data/MT_2216_10x4p5/
"""

import numpy as np
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.thrusters.thrust_curve import ThrustCurve


# ---------------------------------------------------------------------------
# Unit helpers
# ---------------------------------------------------------------------------
_RPM_TO_RADS = np.pi / 30.0   # ω [rad/s] = RPM × 2π/60
_RADS_TO_RPM = 30.0 / np.pi   # RPM       = ω  × 60/2π

# ---------------------------------------------------------------------------
# Static throttle % → steady-state RPM  (global weighted fit, degree 1)
#   RPM_ss(x) = _RPM_A1·x + _RPM_A0,   x = throttle [%]
#   R² = 0.9976   (file: MT_2216_10x4.5_global_fit_coeffs.csv)
# ---------------------------------------------------------------------------
_RPM_A0 = 2991.67    # RPM at 0 % throttle  (motor idle speed)
_RPM_A1 = 70.394     # RPM gained per percent throttle,

# ---------------------------------------------------------------------------
# Backend input-reference decoding  (must match PX4MavlinkBackendConfig)
#   input_reference [rad/s] = controls × input_scaling + zero_position_armed
#   controls ∈ [0, 1]   →   throttle [%] = controls × 100
#   throttle_pct = (input_ref − _DEFAULT_ZERO_POS) / _DEFAULT_SCALING × 100
# ---------------------------------------------------------------------------
_DEFAULT_SCALING  = 1000.0   # rad/s per unit control  (PX4MavlinkBackendConfig default)
_DEFAULT_ZERO_POS = 100.0    # rad/s at armed / 0 % throttle (PX4MavlinkBackendConfig default)

# ---------------------------------------------------------------------------
# Lambda bivariate polynomial — lag_filtered variant
#   Fit: λ = Σ c_ij · R̃^i · r̃^j
#   R̃ = RPM / _RPM_SCALE        (RPM_SCALE spans the 0–10 000 RPM flight envelope)
#   r̃ = RPM_rate / _RATE_SCALE  (RATE_SCALE matches the |rate| ≤ 5 000 RPM/s filter)
#
#   Source: MT_2216_10x4.5_lambda_fit_coeffs_lag_filtered.csv
# ---------------------------------------------------------------------------
_RPM_SCALE  = 10000.0   # RPM normalization denominator
_RATE_SCALE = 5000.0    # RPM/s normalization denominator

# Polynomial coefficients  [term  →  coeff]
_C00 =  15.228347576925923    # RPM^0 · rate^0
_C01 =   0.5264782604975088   # RPM^0 · rate^1
_C02 =  -0.22065976224058959  # RPM^0 · rate^2
_C10 =  -2.971293749482538    # RPM^1 · rate^0
_C11 =   0.14653515702744768  # RPM^1 · rate^1
_C20 =  -0.5215607193970192   # RPM^2 · rate^0

# Physical lower bound on λ: prevents τ > 1 s (λ < 1 s⁻¹ is unphysical
# for a spinning propeller and would destabilise the integrator)
_LAMBDA_MIN = 1.0  # s⁻¹


class MT2216ThrustCurve(ThrustCurve):
    """
    Physics-based thrust curve for the T-Motor MT2216 / 10×4.5 propeller.

    Replaces the instantaneous QuadraticThrustCurve with a two-stage model:
      · Static map  : throttle % → steady-state RPM (measured polynomial)
      · Dynamic lag : first-order ODE dRPM/dt = λ(RPM, RPM_rate) · (RPM_cmd − RPM)
                      where λ is a bivariate polynomial in current RPM and RPM_rate.

    Each of the four rotors is integrated independently with its own state
    (current RPM and previous-step RPM for the finite-difference rate estimate).

    Interface is identical to QuadraticThrustCurve:
        set_input_reference(input_reference)  — target ω [rad/s] per rotor
        update(state, dt)                     — returns (forces, velocities, moment)
        .force / .velocity / .rolling_moment / .rot_dir  properties
    """

    def __init__(self, config: dict = {}):
        """
        Args:
            config (dict): Optional parameter overrides.

                num_rotors (int):                    default 4
                rotor_constant (list[float]):         k_T  [N/(rad/s)²]
                rolling_moment_coefficient (list):    k_M  [Nm/(rad/s)²]
                rot_dir (list[int]):                  +1 CW, -1 CCW per rotor
                min_rotor_velocity (list[float]):     [rad/s]
                max_rotor_velocity (list[float]):     [rad/s]
                backend_scaling  (float):             rad/s per unit control
                backend_zero_pos (float):             rad/s offset at 0 % throttle

        Default rotor_constant / rolling_moment_coefficient are the Iris (3DR)
        values already used in QuadraticThrustCurve and IrisConfig.
        """
        self._num_rotors = config.get("num_rotors", 4)

        self._rotor_constant = config.get(
            "rotor_constant", [8.54858e-6] * self._num_rotors
        )
        assert len(self._rotor_constant) == self._num_rotors

        self._rolling_moment_coefficient = config.get(
            "rolling_moment_coefficient", [1e-6] * self._num_rotors
        )
        assert len(self._rolling_moment_coefficient) == self._num_rotors

        self._rot_dir = config.get("rot_dir", [-1, -1, 1, 1])
        assert len(self._rot_dir) == self._num_rotors

        self.min_rotor_velocity = config.get(
            "min_rotor_velocity", [0.0] * self._num_rotors
        )
        assert len(self.min_rotor_velocity) == self._num_rotors

        self.max_rotor_velocity = config.get(
            "max_rotor_velocity", [1100.0] * self._num_rotors
        )
        assert len(self.max_rotor_velocity) == self._num_rotors

        # Backend scaling (must match PX4MavlinkBackendConfig used at runtime)
        self._backend_scaling  = config.get("backend_scaling",  _DEFAULT_SCALING)
        self._backend_zero_pos = config.get("backend_zero_pos", _DEFAULT_ZERO_POS)

        # ---- Per-rotor state variables (one per rotor, independent) ----
        # Initialise at motor idle speed so there is no step transient on arm
        idle_rpm  = _RPM_A0
        idle_rads = np.clip(
            idle_rpm * _RPM_TO_RADS,
            self.min_rotor_velocity[0],
            self.max_rotor_velocity[0],
        )
        self._rpm      = np.full(self._num_rotors, idle_rpm)   # current RPM  [RPM]
        self._rpm_prev = np.full(self._num_rotors, idle_rpm)   # previous step RPM  [RPM]

        # Outputs (rad/s, N, Nm)
        self._velocity       = np.full(self._num_rotors, idle_rads)
        self._force          = np.zeros(self._num_rotors)
        self._rolling_moment = 0.0

        # Commanded reference (set by set_input_reference each step)
        self._input_reference = [idle_rads] * self._num_rotors

    # ------------------------------------------------------------------
    # ThrustCurve interface
    # ------------------------------------------------------------------

    def set_input_reference(self, input_reference):
        """Receive target angular velocities [rad/s] from the control backend."""
        self._input_reference = input_reference

    def update(self, state: State, dt: float):
        """
        Advance each rotor's RPM by one physics step using the first-order lag.

        Per-rotor sequence (independent for each i ∈ {0,1,2,3}):
          1. Decode throttle % from input_reference [rad/s].
          2. Compute steady-state RPM_cmd from the measured polynomial.
          3. Estimate RPM_rate from the finite difference over dt.
          4. Evaluate λ(RPM, RPM_rate) from the bivariate polynomial surface.
          5. Integrate:  RPM[k+1] = RPM[k] + dt · λ · (RPM_cmd − RPM[k])
          6. Clamp to velocity limits; compute thrust and rolling moment.

        Args:
            state (State): Current vehicle state (not used here; kept for interface).
            dt    (float): Physics timestep [s].

        Returns:
            tuple(list, list, float): (forces_z [N], velocities [rad/s], rolling_moment [Nm])
        """
        rolling_moment = 0.0

        for i in range(self._num_rotors):

            # 1. Throttle % from backend input_reference [rad/s]
            #    input_ref = controls × scaling + zero_pos  →  controls ∈ [0, 1]
            #    throttle_pct = controls × 100
            throttle_pct = np.clip(
                (self._input_reference[i] - self._backend_zero_pos)
                / self._backend_scaling * 100.0,
                0.0, 100.0,
            )

            # 2. Steady-state RPM from measured polynomial (degree 1, R²=0.998)
            rpm_cmd = _RPM_A0 + _RPM_A1 * throttle_pct

            # 3. RPM_rate estimate: finite difference from previous step
            rpm_current = self._rpm[i]
            rpm_rate    = (rpm_current - self._rpm_prev[i]) / dt if dt > 0.0 else 0.0

            # 4. Bandwidth coefficient λ from bivariate polynomial surface
            lam = self._compute_lambda(rpm_current, rpm_rate)
            lam = 13; # average value for testing

            # 5. First-order lag integration (Euler forward step)
            #    dRPM/dt = λ · (RPM_cmd − RPM)
            rpm_next  = rpm_current + dt * lam * (rpm_cmd - rpm_current)
            omega_next = np.clip(
                rpm_next * _RPM_TO_RADS,
                self.min_rotor_velocity[i],
                self.max_rotor_velocity[i],
            )
            rpm_next = omega_next * _RADS_TO_RPM

            # Advance state
            self._rpm_prev[i] = rpm_current
            self._rpm[i]      = rpm_next

            # 6. Outputs
            self._velocity[i] = omega_next
            self._force[i]    = self._rotor_constant[i] * omega_next ** 2
            rolling_moment   += (
                self._rolling_moment_coefficient[i]
                * omega_next ** 2
                * self._rot_dir[i]
            )

        self._rolling_moment = rolling_moment
        return list(self._force), list(self._velocity), self._rolling_moment

    # ------------------------------------------------------------------
    # Lambda evaluation
    # ------------------------------------------------------------------

    def _compute_lambda(self, rpm: float, rpm_rate: float) -> float:
        """
        Evaluate the bandwidth coefficient λ [s⁻¹] from the fitted surface.

        Surface model (lag_filtered, |RPM_rate| ≤ 5 000 RPM/s):
            λ = c00
              + c01 · r̃  + c02 · r̃²
              + c10 · R̃  + c11 · R̃·r̃
              + c20 · R̃²
        where  R̃ = RPM / 10 000   and   r̃ = RPM_rate / 5 000

        The result is clamped to ≥ _LAMBDA_MIN to prevent unphysical behaviour.
        """
        R = rpm      / _RPM_SCALE
        r = rpm_rate / _RATE_SCALE

        lam = (
            _C00
            + _C01 * r  + _C02 * r * r
            + _C10 * R  + _C11 * R * r
            + _C20 * R * R
        )
        return max(lam, _LAMBDA_MIN)

    # ------------------------------------------------------------------
    # Properties  (same names/types as QuadraticThrustCurve)
    # ------------------------------------------------------------------

    @property
    def force(self):
        """Per-rotor thrust [N]."""
        return list(self._force)

    @property
    def velocity(self):
        """Per-rotor angular velocity [rad/s] (actual, after lag)."""
        return list(self._velocity)

    @property
    def rolling_moment(self):
        """Net body-Z yaw moment from spinning propellers [Nm]."""
        return self._rolling_moment

    @property
    def rot_dir(self):
        """Rotation direction per rotor (+1 CW, -1 CCW)."""
        return self._rot_dir
