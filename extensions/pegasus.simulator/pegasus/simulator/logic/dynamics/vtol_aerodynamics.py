"""
Basic VTOL aerodynamic model.

============================================================================
 FRAME CONVENTION  — read this before touching a single coefficient.
============================================================================
Pegasus body frame is **FLU** (x-forward, y-LEFT, z-UP).  Aircraft stability
derivatives are universally published in the **FRD** / stability-axis
convention (x-forward, y-RIGHT, z-DOWN), where

    alpha = atan2(w, u)          w = DOWNWARD body velocity
    beta  = asin(v / V)          v = RIGHTWARD body velocity
    +Cm   = nose-UP,  +Cn = nose-RIGHT,  +Cl = right-wing-DOWN

FLU -> FRD is a 180 deg rotation about the shared x-axis, so

    u, p, Mx        are INVARIANT
    v, w, q, r      flip sign   -> alpha, beta flip sign
    Fy, Fz, My, Mz  flip sign

A derivative is only safe to copy verbatim if its inputs and its output flip
an EVEN number of times.  Historically this model computed alpha/beta from the
FLU velocity and then fed them into FRD-published coefficients, sign-patching
individual terms by hand; six of them ended up wrong, including the
lift-curve slope (the wing lifted only at NEGATIVE true angle of attack) and
the static pitch stability (which was divergent).  See the 2026-09-06 audit.

The fix is structural, not per-coefficient: **this model converts to FRD on
the way in and back to FLU on the way out.**  Every coefficient below is
therefore the STANDARD published value with its STANDARD sign, and no term
needs a frame-related sign patch ever again.  If you add a derivative, copy it
from the literature unchanged.

All returned forces and moments are expressed in the Pegasus body FLU frame.
============================================================================
"""

from dataclasses import dataclass, field
import math
from typing import Mapping, Sequence

import numpy as np


@dataclass
class VTOLAerodynamicsConfig:
    """Aerodynamic derivatives in the STANDARD (FRD / stability-axis)
    convention -- see the frame note at the top of this file.

    Derivatives ported from the AVTOL SO(3) design (AVTOL_Work-main/.../
    config.py AeroParams + fixedwing_aero.py).  The coefficients are
    dimensionless (they characterise the airfoil/layout, so they transfer
    across scale); the geometry below scales the forces to THIS ~3.4 kg
    airframe.

    GEOMETRY SCALED to this ~3.4 kg / ~0.6 m tilt-quadplane (2026-08-09). The
    old S=0.55/b=2.0 were the AVTOL's (a 1325 kg aircraft) -> a 2 m span on a
    0.6 m fuselage is unphysical and made the wing 5x too powerful.  Sensible
    small-quadplane wing: span 1.4 m, chord 0.25 m, S=0.35 m^2 (AR 5.6) ->
    wing loading ~98 N/m^2, stall ~11.5 m/s, wing-borne cruise ~18 m/s.
    """

    rho: float = 1.225
    wing_area: float = 0.35         # S [m^2]
    wing_span: float = 1.4          # b [m]
    mean_chord: float = 0.25        # cbar [m]

    # --- longitudinal lift (small low-AR wing: CL_alpha ~ 2*pi*AR/(AR+2)) ---
    cl0: float = 0.15
    cl_alpha: float = 4.8           # +: more lift at more (nose-up) AoA
    cl_q: float = 7.0               # pitch-rate lift (nose-up rate -> more lift)
    # ALL-MOVING SURFACES (2026-09-06 audit). vtol.py builds each control surface as a
    # separate hinged box at the tail -- there is no fixed stabiliser or fin, so the whole
    # surface rotates and the flap-effectiveness discount does not apply (tau = 1). The
    # AVTOL values assumed conventional flapped surfaces and were 30-40% low. Geometric
    # values, from the surfaces quad_tiltrotor_config actually builds:
    #   CL_de = a_t*S_t/S = 0.70      Cm_de = -V_H*a_t = -1.75   (0.45 x 0.14 m at l_t=0.626)
    #   CY_dr = a_v*S_v/S = 0.24      Cn_dr = -V_V*a_v = -0.109  (0.22 x 0.14 m at l_v=0.626)
    # This also makes the fin exactly self-consistent: |Cn_dr|/|CY_dr| = l_v/b = 0.447.
    # It matters for pitch: at Cm_de = -1.1 the elevator only matches rotor pitch authority
    # at 14.1 m/s -- above the speed at which the rotors have tilted away and stopped
    # providing it. At -1.75 the hand-over is at 11.2 m/s, below stall.
    cl_elevator: float = 0.70       # CL_de (per rad; +de = trailing edge DOWN)

    # --- drag polar  (CD0 + k_induced*CL^2) ---
    cd0: float = 0.04
    # AUDIT FIX (2026-08-11): k=0.04 implied an Oswald efficiency e=1.42
    # (impossible, e<=1) -> induced drag was ~2x under-modeled.
    # k = 1/(pi*e*AR) with AR=5.6, e=0.8 -> 0.071 (physical).
    k_induced: float = 0.071
    cd_beta2: float = 0.20          # extra sideslip drag (kept from Isaac)

    # --- side force  (AVTOL: CY_beta / CY_dr) ---
    cy_beta: float = -0.98          # -: sideslip right -> force left (restoring)
    cy_rudder: float = 0.24         # CY_dr (all-moving fin: a_v*S_v/S)

    # --- roll moment  (+Cl = right wing DOWN) ---
    cl_roll_beta: float = -0.12     # dihedral: slip right -> roll LEFT (restoring)
    # AUDIT FIX (2026-09-06): cl_aileron was 0.08, inherited from the AVTOL -- a 1325 kg
    # aircraft with small outboard ailerons. THIS airframe's aileron, as built by
    # quad_tiltrotor_config, is a 0.35 m x 0.12 m panel on a 1.4 m span / 0.25 m chord wing:
    # 50% of span and 48% of chord, i.e. an ELEVON. A strip estimate over that geometry,
    #     Cl_da = 2*CL_alpha*tau/(S*b) * integral(c*y dy) over the aileron span,
    # with tau ~ 0.65 for cf/c = 0.48, gives 0.49 -- 6x the inherited value.
    #
    # This was not academic: at 0.08 the ailerons NEVER overtake rotor differential-thrust
    # roll (1.5 N.m vs 5.1 N.m at 12 m/s), so as the rotors tilt to 90 deg -- where they stop
    # producing roll at all -- there is no continuous hand-over and roll authority genuinely
    # disappears. At 0.49 the cross-over is ~9 m/s, below the 11.5 m/s stall and well below
    # wing-borne speed. The lateral fragility that defeated every 90 deg transition attempt
    # was, at least in part, this one number. See tests/test_aero_consistency.py.
    cl_aileron: float = 0.49
    cl_p: float = -0.5              # roll damping
    cl_r: float = 0.1               # roll from yaw rate
    cl_rudder: float = 0.02

    # --- pitch moment  (+Cm = nose-UP) ---
    cm0: float = 0.02
    cm_alpha: float = -1.2          # -: static stability (nose-up AoA -> nose-down moment)
    cm_elevator: float = -1.75      # +de (TE down) -> nose-down (all-moving: -V_H*a_t)
    cm_q: float = -12.0             # pitch damping

    # --- yaw moment  (+Cn = nose-RIGHT) ---
    # AUDIT FIX (2026-09-06): cn_beta was 0.25 (AVTOL). The fin this airframe models
    # (0.22 x 0.14 m at l_v = 0.626 m) gives a fin volume V_V = 0.039 and a_v = 2.76 ->
    # Cn_beta = V_V*a_v = 0.109, and a fuselage only REDUCES weathercock stability from
    # there. The inconsistency is visible without any geometry estimate: for the SAME fin
    # the ratio Cn/CY must equal l_v/b = 0.45 by definition, and the pair (0.25, -0.98)
    # gives 0.26 while the rudder pair (-0.12, 0.17) gives 0.71. cy_beta = -0.98 is left
    # alone -- it is defensible once fuselage side area is counted -- but then cn_beta must
    # sit BELOW the fin-alone value, not above it. Side effect: full rudder now trims ~24 deg
    # of sideslip instead of ~12, which is a healthier authority balance.
    cn_beta: float = 0.11           # weathercock: slip right -> nose right (into the wind)
    cn_rudder: float = -0.109       # all-moving fin: -V_V*a_v
    cn_r: float = -0.2              # yaw damping
    cn_p: float = -0.06             # yaw from roll rate
    cn_aileron: float = 0.02        # adverse yaw

    max_surface_deflection_rad: float = math.radians(25.0)

    # --- stall / high-alpha blending -------------------------------------
    # With CL0=0.15, CL_alpha=4.8 and a small low-Re wing, the linear lift
    # peaks near 13 deg, which is where the stall break belongs.
    #
    # Beyond it the model blends to a NEWTONIAN FLAT PLATE
    # (CL = 2*sign(a)*sin^2(a)*cos(a), CD = 2*sin^2(a)) with a sigmoid of
    # sharpness `stall_blend_M`.  A VTOL spends a lot of time at attitudes a
    # linear wing model cannot describe -- descending, sliding sideways,
    # backing up -- and the blend keeps those regimes merely approximate
    # instead of actively wrong.  The previous model CLIPPED alpha for the
    # coefficients but used the RAW alpha for the lift DIRECTION, which
    # produced 15 N of downforce while backing up at 10 m/s and a 10 N
    # backward force in a pure vertical descent (2026-09-06 audit).
    stall_alpha_rad: float = math.radians(13.0)
    # sideslip: where the fin's linear term gives way to a flat plate (see update())
    sideslip_break_rad: float = math.radians(20.0)
    cy_max: float = 0.6             # broadside |CY| on the wing reference area
    stall_blend_M: float = 25.0     # sigmoid sharpness of the linear->flat-plate blend
    min_airspeed: float = 0.25
    max_coefficient_speed: float = 80.0


@dataclass
class VTOLAeroState:
    """Diagnostics from the last update.

    NOTE: `alpha` and `beta` are reported in the STANDARD convention
    (alpha > 0 = nose-up / wind from below), NOT the raw FLU atan2(w, u) that
    older logs and analysis scripts printed -- those had the opposite sign.
    """
    airspeed: float = 0.0
    alpha: float = 0.0
    beta: float = 0.0
    dynamic_pressure: float = 0.0
    coefficients: dict = field(default_factory=dict)


class VTOLAerodynamics:
    """Compute lift, drag, sideslip force, and aerodynamic moments."""

    def __init__(self, config: VTOLAerodynamicsConfig | Mapping | None = None):
        if config is None:
            self.config = VTOLAerodynamicsConfig()
        elif isinstance(config, VTOLAerodynamicsConfig):
            self.config = config
        else:
            self.config = VTOLAerodynamicsConfig(**dict(config))

        self.last_state = VTOLAeroState()

    # ------------------------------------------------------------------
    def _stall_blend(self, angle: float, break_angle: float = None) -> float:
        """0 in the linear regime, 1 past the break angle.

        This is the Beard & McLain blending sigmoid written as a tanh -- the two agree to
        3e-3 everywhere, and the tanh form saturates instead of overflowing, so the SAME
        expression can be written in CasADi inside a controller's model. The controllers
        must mirror this model exactly (see tests/test_controller_aero_match.py), and the
        cheapest way to guarantee that is for both sides to evaluate the same formula.
        """
        cfg = self.config
        a0 = cfg.stall_alpha_rad if break_angle is None else break_angle
        return 0.5 * (1.0 + math.tanh(0.5 * cfg.stall_blend_M * (abs(angle) - a0)))

    def update(self, state, actuators=None, wind_body=None):
        """`state.linear_body_velocity` / `state.angular_velocity` are FLU;
        the returned force and moment are FLU.  Everything in between is FRD."""
        cfg = self.config
        controls = self._coerce_actuators(actuators)

        body_velocity = np.array(state.linear_body_velocity, dtype=float)
        if wind_body is not None:
            body_velocity = body_velocity - np.array(wind_body, dtype=float)

        airspeed = float(np.linalg.norm(body_velocity))
        if airspeed < cfg.min_airspeed:
            self.last_state = VTOLAeroState(airspeed=airspeed)
            return np.zeros(3), np.zeros(3)

        # ---- FLU -> FRD (180 deg about x): y and z components flip ----
        u = body_velocity[0]
        v = -body_velocity[1]
        w = -body_velocity[2]
        omega_flu = np.array(state.angular_velocity, dtype=float)
        p, q, r = omega_flu[0], -omega_flu[1], -omega_flu[2]

        alpha = math.atan2(w, u)                        # standard AoA (+ = nose-up)
        beta = math.asin(float(np.clip(v / airspeed, -1.0, 1.0)))

        # One speed drives BOTH the dynamic pressure and the rate
        # non-dimensionalisation, so the model stays self-consistent when the
        # cap bites (they used to disagree above max_coefficient_speed).
        v_coeff = min(airspeed, cfg.max_coefficient_speed)
        qbar = 0.5 * cfg.rho * v_coeff * v_coeff

        elevator = controls["elevator"] * cfg.max_surface_deflection_rad
        aileron = controls["aileron"] * cfg.max_surface_deflection_rad
        rudder = controls["rudder"] * cfg.max_surface_deflection_rad

        p_hat = p * cfg.wing_span / (2.0 * v_coeff)
        q_hat = q * cfg.mean_chord / (2.0 * v_coeff)
        r_hat = r * cfg.wing_span / (2.0 * v_coeff)

        # ---- lift / drag / side force: linear model blended into a flat plate ----
        # SIDE FORCE. The linear fin term is only meaningful while the flow is attached to
        # the fin. Extrapolated to beta = 90 deg it claims |CY| = 1.54 -- 16 N of side force
        # on this airframe in a 6 m/s crosswind, about half its weight -- and it is collinear
        # with the cd_beta2 sideslip drag out there, so the two double-count. Past the break
        # angle both fade into a single flat plate referenced to the airframe's side area
        # (~0.10 m^2 at Cd ~ 2 -> CY_max ~ 0.6 on the 0.35 m^2 wing reference). Symmetric
        # with what the lift curve does in alpha. A VTOL sits sideways-on to the wind in
        # every hover, so this is an operating point, not a corner case.
        sigma_b = self._stall_blend(beta, cfg.sideslip_break_rad)
        cy_linear = cfg.cy_beta * beta + cfg.cy_rudder * rudder
        cy_plate = -cfg.cy_max * math.copysign(1.0, beta) * math.sin(beta) ** 2
        cy = (1.0 - sigma_b) * cy_linear + sigma_b * cy_plate
        sigma = self._stall_blend(alpha)
        sa, ca = math.sin(alpha), math.cos(alpha)
        cl_linear = cfg.cl0 + cfg.cl_alpha * alpha
        cl_plate = 2.0 * math.copysign(1.0, alpha) * sa * sa * ca
        cl = ((1.0 - sigma) * cl_linear + sigma * cl_plate
              + cfg.cl_q * q_hat + cfg.cl_elevator * elevator)
        cd = (cfg.cd0
              + (1.0 - sigma) * cfg.k_induced * cl_linear * cl_linear
              + sigma * 2.0 * sa * sa
              + (1.0 - sigma_b) * cfg.cd_beta2 * beta * beta)

        # The moment model is only meaningful in the attached-flow regime; past
        # stall the alpha term is frozen at its stall value rather than
        # extrapolated (the damping and control terms stay live).
        alpha_att = float(np.clip(alpha, -cfg.stall_alpha_rad, cfg.stall_alpha_rad))
        c_l = (cfg.cl_roll_beta * beta + cfg.cl_aileron * aileron + cfg.cl_p * p_hat
               + cfg.cl_r * r_hat + cfg.cl_rudder * rudder)
        c_m = cfg.cm0 + cfg.cm_alpha * alpha_att + cfg.cm_elevator * elevator + cfg.cm_q * q_hat
        c_n = (cfg.cn_beta * beta + cfg.cn_rudder * rudder + cfg.cn_r * r_hat
               + cfg.cn_p * p_hat + cfg.cn_aileron * aileron)

        lift = qbar * cfg.wing_area * cl
        drag = qbar * cfg.wing_area * cd
        side = qbar * cfg.wing_area * cy

        # Wind axes -> FRD body axes.  Drag opposes the relative wind; lift is
        # perpendicular to it in the plane of symmetry, positive "upward"
        # (= -z in FRD).  Both stay exact at any alpha, so the force can never
        # point the wrong way when the linear model saturates.
        velocity_frd = np.array([u, v, w])
        drag_force = -drag * (velocity_frd / airspeed)
        lift_force = lift * np.array([sa, 0.0, -ca])
        side_force = np.array([0.0, side, 0.0])
        force_frd = drag_force + lift_force + side_force

        moment_frd = np.array(
            [
                qbar * cfg.wing_area * cfg.wing_span * c_l,
                qbar * cfg.wing_area * cfg.mean_chord * c_m,
                qbar * cfg.wing_area * cfg.wing_span * c_n,
            ],
            dtype=float,
        )

        # ---- FRD -> FLU on the way out ----
        force_body = np.array([force_frd[0], -force_frd[1], -force_frd[2]])
        moment_body = np.array([moment_frd[0], -moment_frd[1], -moment_frd[2]])

        self.last_state = VTOLAeroState(
            airspeed=airspeed,
            alpha=alpha,
            beta=beta,
            dynamic_pressure=qbar,
            coefficients={"CL": cl, "CD": cd, "CY": cy, "Cl": c_l, "Cm": c_m,
                          "Cn": c_n, "stall": sigma},
        )

        return force_body, moment_body

    def _coerce_actuators(self, actuators):
        controls = {
            "aileron": 0.0,
            "elevator": 0.0,
            "rudder": 0.0,
        }

        if actuators is None:
            return controls

        if isinstance(actuators, Mapping):
            for key in controls:
                controls[key] = float(np.clip(actuators.get(key, 0.0), -1.0, 1.0))
            return controls

        if isinstance(actuators, Sequence) and not isinstance(actuators, (str, bytes)):
            values = list(actuators)
            if len(values) > 5:
                controls["aileron"] = float(np.clip(values[5], -1.0, 1.0))
            if len(values) > 6:
                controls["elevator"] = float(np.clip(values[6], -1.0, 1.0))
            if len(values) > 7:
                controls["rudder"] = float(np.clip(values[7], -1.0, 1.0))

        return controls
