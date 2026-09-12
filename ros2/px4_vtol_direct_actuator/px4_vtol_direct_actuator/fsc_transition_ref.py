#!/usr/bin/env python3
"""FSC-scaled wrapper around the AVTOL SO(3) path-following REFERENCE GENERATORS.

The user's AVTOL work (~/Documents/PhDStudies/ControlDesign_VTOL/NMPC_Design/AVTOL_Work-main/
Aircraft_Model_SO3_pathfollowing) contains the hover->cruise transition reference generators:

  * transition_reference.py            -- TransitionRefInterp (original)
  * transition_reference_consistent.py -- TransitionRefInterp (consistency-fixed: re-trims
        alpha/thrust after smoothing, DCM-consistent velocity, body-rate refs)  <- PREFERRED
  * Build_TransitionProfile.py / run_transitionProfile.py -- open-loop profile builders
        (write openloop_transition_profile.npz; need the kane_plant model, offline only)

They are scaled to the 1325 kg AVTOL (S=21.7 m^2, T_max=24 kN, Vc=40 m/s). This module imports
them UNMODIFIED and re-parameterises for THIS 3.5 kg FSC tilt-quadplane (the same trim math is
scale-free; only mass/geometry/thrust change). The result is a force-balanced transition schedule
    V -> (tilt, alpha, theta, Ttot, de_trim)
i.e. for every airspeed on the hover->cruise path: the total-thrust TILT (converted to the Isaac
convention), the trim angle-of-attack, the body pitch reference, the total thrust, and the
elevator trim. The dimensionless aero derivatives here MATCH vtol_aerodynamics.py exactly
(which itself ports the AVTOL AeroParams), so the schedule is consistent with the sim plant.

CONVENTIONS
  * AVTOL beta: 90 deg = hover (thrust vertical), 0 deg = cruise (thrust forward).
  * Isaac tilt: 0 = up, 90 deg = forward.       tilt_isaac = 90 deg - beta_avtol.
  * theta (body pitch reference, nose-up +) = alpha + gamma (gamma ~ 0 on a level transition).
  * de_trim is in rad; the sim elevator channel is de/25 deg (max_surface_deflection_rad).

Used by nmpc_cruise_node.py (NMPC fixed-wing cruise, GS button D) and
nmpc_so3_cascade_node.py (geometric fixed-wing cruise, GS button E).
Env: AVTOL_REF_DIR overrides the generator location.
"""
import math
import os
import sys

import numpy as np

DEFAULT_AVTOL_DIR = os.path.expanduser(
    "~/Documents/PhDStudies/ControlDesign_VTOL/NMPC_Design/AVTOL_Work-main/"
    "Aircraft_Model_SO3_pathfollowing")

# ---- FSC quad-tiltrotor parameters (single source; matches vtol_aerodynamics.py +
#      quad_tiltrotor_config + the HIL-measured 34.3 N weight) ----
FSC = dict(
    m=3.45, g=9.81, rho=1.225,   # 3.45 = the ASSEMBLED vehicle (body link + 10 articulated
    #                              links); 3.5 left a steady 0.14 m/s^2 bias in every reference
    S=0.35,                      # wing area [m^2]  (scaled small-quadplane wing)
    CL0=0.15, CL_alpha=4.8,      # lift (CL_max handled via alpha_max below)
    # AERO SYNCED TO THE CORRECTED SIM (2026-08-11 audit): this trim generator is the 4th aero copy
    # (sim + SO(3) NMPC model + nmpc_cruise + HERE). It MUST match vtol_aerodynamics.py or the trim
    # schedule V->(tilt,alpha,theta,Ttot,de) is infeasible for the plant and the NMPC solve fails.
    CD0=0.04, k_induced=0.071,
    Cm0=-0.02, Cm_alpha=1.2, Cm_de=1.1,
    T_max_total=60.0,            # 4x18 N, derated: mid-transition vertical support is bounded
    alpha_max_deg=8.0,           # keeps CL <= 0.82 (CL_max 1.2, stall 13 deg) -> below the stall break
    accel_cap=2.0,               # forward accel the schedule may assume [m/s^2]
    a_margin=0.15,               # W_eff = W + m*a_margin (small maneuver margin)
    cruise_thrust_margin=1.15,
)
DE_MAX = math.radians(25.0)      # sim max_surface_deflection_rad (elevator channel scaling)

_avtol_mod = None


def avtol_dir():
    return os.environ.get("AVTOL_REF_DIR", DEFAULT_AVTOL_DIR)


def list_generators():
    """The reference-generator entry points found in the AVTOL directory."""
    d = avtol_dir()
    names = ["transition_reference_consistent.py", "transition_reference.py",
             "Build_TransitionProfile.py", "run_transitionProfile.py"]
    return {n: os.path.isfile(os.path.join(d, n)) for n in names}


def _import_avtol(consistent=True):
    """Import TransitionRefInterp from the AVTOL directory (unmodified sources)."""
    global _avtol_mod
    if _avtol_mod is not None:
        return _avtol_mod
    d = avtol_dir()
    if not os.path.isdir(d):
        raise FileNotFoundError(f"AVTOL reference dir not found: {d} (set AVTOL_REF_DIR)")
    sys.path.insert(0, d)
    try:
        if consistent:
            try:
                import transition_reference_consistent as mod
            except Exception:
                import transition_reference as mod
        else:
            import transition_reference as mod
    finally:
        try:
            sys.path.remove(d)
        except ValueError:
            pass
    _avtol_mod = mod
    return mod


class FSCTransitionProfile:
    """FSC-scaled hover->cruise transition schedule, indexed by AIRSPEED (path-following:
    the reference advances only as the vehicle actually gains speed, so tilt/pitch/thrust
    stay force-balanced with the current aerodynamic state -- the same idea as the AVTOL
    s_from_beta path parameterisation, but keyed on the airspeed the wing actually sees)."""

    def __init__(self, V_cruise=15.0, tf=20.0, dt=0.02, consistent=True, tilt_cap_deg=None,
                 **overrides):
        mod = _import_avtol(consistent=consistent)
        p = dict(FSC); p.update(overrides)
        self._tilt_cap_deg = tilt_cap_deg          # None -> env FSC_TILT_CAP_DEG (default 65)
        self.m = p["m"]; self.g = p["g"]; self.W = p["m"] * p["g"]
        self.V_cruise = float(V_cruise)
        self.ref = mod.TransitionRefInterp.from_params(
            tf=float(tf), dt=float(dt),
            beta_hover_deg=90.0, beta_cruise_deg=0.0,
            V_start=0.0, V_cruise=float(V_cruise),
            z_start=0.0, z_final=0.0, heading_change_deg=0.0,
            m=p["m"], g=p["g"], rho=p["rho"], S=p["S"],
            CL0=p["CL0"], CL_alpha=p["CL_alpha"], CD0=p["CD0"], k_induced=p["k_induced"],
            Cm0=p["Cm0"], Cm_alpha=p["Cm_alpha"], Cm_de=p["Cm_de"],
            T_max_total=p["T_max_total"], alpha_max_deg=p["alpha_max_deg"],
            a_margin=p["a_margin"], accel_cap=p["accel_cap"],
            cruise_thrust_margin=p["cruise_thrust_margin"],
        )
        # ---- V-indexed lookup tables (monotonic airspeed axis) ----
        t = np.asarray(self.ref.t_arr, float)
        V = np.maximum.accumulate(np.asarray(self.ref.Vref_arr, float))  # monotonic for interp
        beta_av = np.asarray(self.ref.beta_arr, float)                   # AVTOL convention [rad]
        alpha = np.asarray(self.ref.alpha_arr, float)
        Ttot = np.asarray(self.ref.Ttot_arr, float)
        de = np.asarray(getattr(self.ref, "de_trim_arr", np.zeros_like(t)), float)
        gamma = np.asarray(getattr(self.ref, "gamma_arr", np.zeros_like(t)), float)
        self._t, self._V = t, V
        # Isaac convention; capped at 65 deg by default. Every HIL bag (fwD4..fwD8) shows the
        # loop ROCK-SOLID through ~65 deg tilt / wing ~80%, and fragile in the last 15% of the
        # hand-off: once the rear pair fully unloads (~72 deg+) the rotor pitch authority is
        # ONE-SIDED (can't push below zero), the slow pitch trim + elevator can't stop an
        # uncommanded pitch-up, and the cruise phugoid cycles into wing STALL (AoA +40 in fwD8).
        # At 65 deg the thrust is still mostly forward + the wing carries ~75-85% via the alpha
        # trim = genuine fixed-wing cruise, with enough reserve authority to damp the phugoid.
        # Env FSC_TILT_CAP_DEG pushes toward 75-90 for envelope research.
        cap_deg = (self._tilt_cap_deg if self._tilt_cap_deg is not None
                   else float(os.environ.get("FSC_TILT_CAP_DEG", "65.0")))
        tilt_cap = math.radians(float(cap_deg))
        self._tilt = np.clip(math.pi / 2 - beta_av, 0.0, tilt_cap)
        self._alpha = alpha
        self._theta = alpha + gamma                                      # body pitch ref (nose-up +)
        self._Ttot = Ttot
        self._de = de

    # ---- lookups (V clipped to the schedule range) ----
    def _iV(self, V, arr):
        return float(np.interp(float(V), self._V, arr))

    def by_speed(self, V):
        """Schedule at airspeed V: dict(tilt, alpha, theta, theta_cmd, Ttot, de, de_cmd, wing_share).

        theta_cmd is the SPEED-GATED pitch command: the trim solver pins alpha at alpha_max all
        the way down to V=0 (it minimises thrust), but below ~2 m/s the wing does nothing and the
        body should hover LEVEL -- so the commanded pitch ramps in over 2..10 m/s (fully in well
        before wing-borne, so the cruise trim itself is untouched)."""
        tilt = self._iV(V, self._tilt)
        alpha = self._iV(V, self._alpha)
        theta = self._iV(V, self._theta)
        Ttot = self._iV(V, self._Ttot)
        de = self._iV(V, self._de)
        gate = min(max((float(V) - 2.0) / 8.0, 0.0), 1.0)
        return dict(tilt=tilt, alpha=alpha, theta=theta, theta_cmd=theta * gate,
                    Ttot=Ttot, de=de,
                    de_cmd=max(min(de / DE_MAX, 1.0), -1.0) * gate,
                    wing_share=self.wing_share(V))

    def by_time(self, t):
        return self.by_speed(float(np.interp(float(t), self._t, self._V)))

    def speed_at_tilt(self, tilt):
        """Invert the tilt schedule: the trim airspeed at a given (Isaac) tilt angle [rad]."""
        return float(np.interp(float(tilt), self._tilt, self._V))

    def wing_share(self, V):
        """Fraction of weight the schedule expects the WING to carry at airspeed V."""
        Ttot = self._iV(V, self._Ttot)
        beta_av = math.pi / 2 - self._iV(V, self._tilt)
        vert_thrust = Ttot * math.sin(beta_av)
        return float(np.clip(1.0 - vert_thrust / self.W, 0.0, 1.0))

    @property
    def v_wing_borne(self):
        """Airspeed from which the schedule is essentially airplane-mode (tilt > 80 deg)."""
        idx = np.argmax(self._tilt > math.radians(80.0))
        if self._tilt[idx] <= math.radians(80.0):
            return self.V_cruise
        return float(self._V[idx])

    def table(self, n=21):
        rows = []
        for tt in np.linspace(self._t[0], self._t[-1], n):
            V = float(np.interp(tt, self._t, self._V))
            s = self.by_speed(V)
            rows.append((tt, V, math.degrees(s["tilt"]), math.degrees(s["alpha"]),
                         math.degrees(s["theta"]), s["Ttot"], s["de_cmd"], s["wing_share"]))
        return rows


def _selftest():
    print(f"AVTOL dir: {avtol_dir()}")
    for name, found in list_generators().items():
        print(f"  {'ok ' if found else 'MISSING'} {name}")
    prof = FSCTransitionProfile(V_cruise=15.0, tf=20.0)
    W = prof.W
    print(f"\nFSC-scaled transition schedule (m={prof.m} kg, W={W:.1f} N, Vc={prof.V_cruise}):")
    print(f"{'t':>5} {'V':>6} {'tilt':>6} {'alpha':>6} {'theta':>6} {'Ttot':>6} {'de_cmd':>7} {'wing%':>6}")
    for r in prof.table():
        print(f"{r[0]:5.1f} {r[1]:6.2f} {r[2]:6.1f} {r[3]:6.2f} {r[4]:6.2f} {r[5]:6.1f} {r[6]:7.3f} {100*r[7]:5.1f}%")
    s0 = prof.by_speed(0.0); sc = prof.by_speed(prof.V_cruise)
    print(f"\nv_wing_borne (tilt>80deg) = {prof.v_wing_borne:.1f} m/s")
    ok = True
    def chk(cond, msg):
        nonlocal ok
        print(("PASS " if cond else "FAIL ") + msg)
        ok = ok and cond
    chk(abs(s0["Ttot"] - W * (1 + FSC["a_margin"] / FSC["g"])) < 0.15 * W,
        f"hover thrust ~= weight ({s0['Ttot']:.1f} vs {W:.1f} N)")
    chk(s0["tilt"] < math.radians(10.0), f"hover tilt ~ 0 ({math.degrees(s0['tilt']):.1f} deg)")
    chk(sc["tilt"] > math.radians(60.0), f"cruise tilt > 60 deg ({math.degrees(sc['tilt']):.1f} deg; default cap 65)")
    chk(math.radians(2.0) < sc["alpha"] < math.radians(9.0),
        f"cruise alpha in 2..9 deg ({math.degrees(sc['alpha']):.2f} deg)")
    chk(sc["Ttot"] < 0.35 * W, f"cruise thrust << weight ({sc['Ttot']:.1f} vs {W:.1f} N)")
    chk(sc["wing_share"] > 0.85, f"cruise wing share > 85% ({100*sc['wing_share']:.0f}%)")
    # force balance at cruise trim: L(V,alpha) ~ W_eff - Ttot*sin(beta)
    qS = 0.5 * FSC["rho"] * prof.V_cruise ** 2 * FSC["S"]
    L = qS * (FSC["CL0"] + FSC["CL_alpha"] * sc["alpha"])
    beta_av = math.pi / 2 - sc["tilt"]
    resid = L + sc["Ttot"] * math.sin(beta_av) - W * (1 + FSC["a_margin"] / FSC["g"])
    chk(abs(resid) < 0.15 * W, f"cruise vertical balance residual {resid:+.1f} N")
    print("\nSELFTEST " + ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(_selftest())
