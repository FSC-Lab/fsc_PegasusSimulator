"""
control_params.py — the ONE place a control parameter comes from.

Every scenario flies the SAME law (controller.py, the coupled feedback-
linearizing controller + generalized-momentum observer). What differs between a
free-flight tracking run, a grasp and a box push is the NUMBERS — so the numbers
live in `robotic_arm/config/<scenario>.yaml` and nowhere else. That folder sits
beside utils_controller/ and utils_planner/ rather than inside either: the
parameters belong to the SCENARIO, not to the code that happens to read them.

WHY THERE ARE NO DEFAULTS IN THIS FILE
    Before this module the three scenarios were three copies of the controller,
    and the copies silently drifted: DLS_LAMBDA was 0.3 in one and `0#.3`
    (i.e. zero, commented out mid-token) in the other two, and only one of them
    ever got the saturation-consistent coupling. Nobody decided that twice.
    A default here would recreate exactly that failure — the YAML says 0.2, the
    dataclass says 0.3, and "which one flew?" needs a debugger to answer.
    So: a key missing from the YAML is a STARTUP ERROR listing what is missing.
    The file is the answer, always.

LAYOUT OF A SCENARIO FILE

    default:            # required — the scenario's baseline, all keys present
      k_x: 16.0
      ...
    circle_drone:       # optional variant sections, sparse overrides
      K_y: [8, 8, 8, 1]

`load("free", variant="circle_drone")` merges the variant over `default`. A
variant that does not exist is NOT an error (most trajectories are happy with
the baseline) — it is reported, so a typo'd section name is visible rather than
silently ignored. An UNKNOWN KEY is always an error, in either section.

Precedence, lowest to highest:  default  ->  variant  ->  overrides (AM_SWEEP).

Pure numpy + pyyaml: importable and testable without Isaac.
"""

import os
from dataclasses import dataclass, field, fields

import numpy as np
import yaml

# robotic_arm/config/ — a sibling of utils_controller/, not a child of it (the
# parameters describe the scenario, not this module).
CONFIG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")

# Every key a scenario file must define, by kind. The controller reads NOTHING
# that is not in one of these lists, and the loader accepts nothing else — so
# this block is both the schema and the documentation of what is tunable.
_SCALARS = ("k_x", "k_v", "k_R", "k_w", "dls_lambda")
_FLAGS = ("use_gmo", "omega_e_ff")
_DIAGS = {"M_r_d": 3, "K_y": 4, "D_y": 4, "K_o": 10}   # stored as diagonal matrices
_SPECIAL = ("impedance_mode", "M_Y", "tau_max")        # validated case by case
REQUIRED = tuple(_SCALARS) + tuple(_FLAGS) + tuple(_DIAGS) + tuple(_SPECIAL)


@dataclass
class ControlParams:
    """One scenario's control parameters, exactly as loaded.

    Attribute names match what the law reads (`g.k_x`, `g.K_y`, ...), so the
    control code is untouched by where the numbers came from.
    """

    # --- geometric outer loop (base position + attitude) --------------------
    k_x: float
    k_v: float
    k_R: float
    k_w: float
    M_r_d: np.ndarray            # 3x3 diagonal — desired rotational inertia

    # --- end-effector task impedance ---------------------------------------
    K_y: np.ndarray              # 4x4 diagonal — [x, y, z, EE heading]
    D_y: np.ndarray              # 4x4 diagonal

    # --- generalized-momentum disturbance observer --------------------------
    use_gmo: bool
    K_o: np.ndarray              # (6+n)x(6+n) diagonal — per-channel bandwidth

    # --- arm task inertia ---------------------------------------------------
    impedance_mode: str          # "natural" (M_y = Lambda_y) | "shaped"
    M_Y: np.ndarray = None       # 4x4 diagonal in shaped mode, None in natural

    # --- solver / actuator --------------------------------------------------
    dls_lambda: float = 0.0      # damped-least-squares reg. of the J_3y solve
    # tau_max: per-joint servo saturation [N.m]. None = NO clamp inside the law
    # (the caller may still clamp what it applies). Not the same thing: with a
    # value, the law clamps AND rebuilds the base moment from the realized u3,
    # so tau_body never pre-compensates a reaction the servos cannot deliver.
    tau_max: float = None
    omega_e_ff: bool = False     # EE angular-accel feedforward (off = filtered)

    # provenance, for the startup banner and the run log — which file, which
    # section. A run whose gains cannot be traced to a file is a run you cannot
    # repeat.
    source: str = ""
    scenario: str = ""
    variant: str = ""

    def summary(self):
        """One line for the startup banner / npz metadata."""
        m_y = "natural" if self.M_Y is None else "shaped"
        tau = "off" if self.tau_max is None else f"{self.tau_max:g}"
        return (f"{self.scenario}"
                + (f":{self.variant}" if self.variant else "")
                + f"  k_x={self.k_x:g} k_v={self.k_v:g} k_R={self.k_R:g} "
                f"k_w={self.k_w:g}  K_y={np.diag(self.K_y)} "
                f"D_y={np.diag(self.D_y)}  GMO={'on' if self.use_gmo else 'off'} "
                f"K_o={np.diag(self.K_o)[0]:g}..  M_y={m_y} "
                f"dls={self.dls_lambda:g} tau_max={tau}")

    def as_dict(self):
        """Plain-python view (npz/JSON friendly): matrices back to diagonals."""
        out = {}
        for f in fields(self):
            v = getattr(self, f.name)
            if isinstance(v, np.ndarray):
                v = np.diag(v).tolist()
            out[f.name] = v
        return out


class ConfigError(ValueError):
    """A scenario file that cannot be trusted — raised eagerly at startup."""


def scenario_path(scenario):
    return os.path.join(CONFIG_DIR, f"{scenario}.yaml")


def available_scenarios():
    if not os.path.isdir(CONFIG_DIR):
        return []
    return sorted(f[:-5] for f in os.listdir(CONFIG_DIR) if f.endswith(".yaml"))


def _diag(name, value, n):
    v = np.asarray(value, float).ravel()
    if v.shape != (n,):
        raise ConfigError(f"{name!r} must be a list of {n} numbers "
                          f"(the diagonal), got {len(v)}: {value!r}")
    if not np.all(np.isfinite(v)):
        raise ConfigError(f"{name!r} has a non-finite entry: {value!r}")
    return np.diag(v)


def _build(merged, where, scenario, variant):
    """Validated mapping -> ControlParams. Every error names the file."""
    unknown = sorted(set(merged) - set(REQUIRED))
    if unknown:
        raise ConfigError(f"{where}: unknown key(s) {unknown}; valid keys are "
                          f"{sorted(REQUIRED)}")
    missing = sorted(set(REQUIRED) - set(merged))
    if missing:
        raise ConfigError(f"{where}: missing required key(s) {missing}. There "
                          f"are no defaults in code — every key must be in the "
                          f"file (see control_params.py).")

    kw = {}
    for k in _SCALARS:
        try:
            kw[k] = float(merged[k])
        except (TypeError, ValueError):
            raise ConfigError(f"{where}: {k!r} must be a number, got {merged[k]!r}")
    for k in _FLAGS:
        if not isinstance(merged[k], bool):
            raise ConfigError(f"{where}: {k!r} must be true/false, got {merged[k]!r}")
        kw[k] = bool(merged[k])
    for k, n in _DIAGS.items():
        kw[k] = _diag(f"{where}: {k}", merged[k], n)

    mode = merged["impedance_mode"]
    if mode not in ("natural", "shaped"):
        raise ConfigError(f"{where}: impedance_mode must be 'natural' or "
                          f"'shaped', got {mode!r}")
    kw["impedance_mode"] = mode
    # natural mode resolves M_y to Lambda_y online, so a matrix here would be
    # silently ignored — refuse it rather than let the file lie about the run.
    if mode == "natural":
        if merged["M_Y"] is not None:
            raise ConfigError(f"{where}: impedance_mode 'natural' uses the "
                              f"robot's own task inertia; M_Y must be null")
        kw["M_Y"] = None
    else:
        if merged["M_Y"] is None:
            raise ConfigError(f"{where}: impedance_mode 'shaped' needs M_Y "
                              f"(4 numbers); got null")
        kw["M_Y"] = _diag(f"{where}: M_Y", merged["M_Y"], 4)
        if not kw["use_gmo"]:
            # NOT an error — this is the GMO A/B baseline. Worth saying out
            # loud, though, because only PART of shaped mode switches off with
            # the observer: the disturbance term −(Λ_y·M_y⁻¹−I)·F̂_y vanishes
            # with F̂_y, but Λ_y·M_y⁻¹ still scales the D_y/K_y impedance, so
            # this is NOT the same run as impedance_mode 'natural'.
            print(f"[control_params] NOTE {where}: shaped impedance with "
                  f"use_gmo false — the GMO disturbance term drops out, but "
                  f"M_Y still shapes the D_y/K_y impedance (this is the "
                  f"observer A/B, not 'natural' mode)", flush=True)

    tau = merged["tau_max"]
    if tau is not None:
        tau = float(tau)
        if not (tau > 0):
            raise ConfigError(f"{where}: tau_max must be > 0 or null "
                              f"(null = no clamp inside the law), got {tau}")
    kw["tau_max"] = tau

    return ControlParams(scenario=scenario, variant=variant or "",
                         source=where, **kw)


def load(scenario, variant=None, path=None, overrides=None, verbose=True):
    """Load `config/<scenario>.yaml`, optionally merging a variant section.

    scenario   file stem, e.g. "free" / "pick" / "push" / "px4_direct"
    variant    optional section merged over `default` — the free-flight demo
               passes its trajectory type, so a mode can carry its own gains
    path       explicit file, bypassing CONFIG_DIR (per-run experiments)
    overrides  final dict merged on top (AM_SWEEP), same validation
    """
    p = path or scenario_path(scenario)
    if not os.path.exists(p):
        raise ConfigError(f"no control-parameter file {p!r}; available "
                          f"scenarios: {available_scenarios()}")
    with open(p) as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ConfigError(f"{p}: expected a mapping of sections, got "
                          f"{type(data).__name__}")
    if "default" not in data or not isinstance(data["default"], dict):
        raise ConfigError(f"{p}: needs a 'default:' section holding every key "
                          f"(variants are sparse overrides of it)")

    merged = dict(data["default"])
    used_variant = ""
    if variant:
        if variant in data:
            if not isinstance(data[variant], dict):
                raise ConfigError(f"{p}: section {variant!r} must be a mapping")
            merged.update(data[variant])
            used_variant = variant
        elif verbose:
            # NOT an error: most trajectories are happy on the baseline. Said
            # out loud so a typo'd section name is visible instead of silent.
            print(f"[control_params] {os.path.basename(p)}: no '{variant}' "
                  f"section — using 'default' (sections: "
                  f"{[k for k in data if k != 'default']})", flush=True)
    where = f"{p} [default" + (f"+{used_variant}" if used_variant else "") + "]"

    if overrides:
        bad = sorted(set(overrides) - set(REQUIRED))
        if bad:
            raise ConfigError(f"override(s) {bad} are not control parameters; "
                              f"valid keys are {sorted(REQUIRED)}")
        merged.update(overrides)
        where += " +overrides"

    cfg = _build(merged, where, scenario, used_variant)
    if verbose:
        print(f"[control_params] {cfg.summary()}", flush=True)
        print(f"[control_params] source: {where}", flush=True)
    return cfg
