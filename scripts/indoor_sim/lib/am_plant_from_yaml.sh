#!/usr/bin/env bash
# am_plant_from_yaml.sh -- the aerial manipulator's PLANT knobs, read from a
# controller yaml's section-1 REALITY MODEL and exported as the PEGASUS_* env
# the Isaac plant script (06) consumes (2026-09-18).
#
# Sourced, not executed. Set WB_SIM_YAML before sourcing. Exports:
#   PEGASUS_ARM_SERVO_MODEL, PEGASUS_ARM_CURRENT_NOISE_{A,BW_HZ,SEED},
#   PEGASUS_PLANT_{MASS,INERTIA}_SCALE, PEGASUS_PLANT_COM_SHIFT_{X,Y,Z},
#   PEGASUS_PLANT_KF_SCALE, PEGASUS_ARM_COUNTS_{ENABLE,NOMINAL,TRUE},
#   PEGASUS_ARM_FRICTION_{SCALE,WIDTH}, PEGASUS_ARM_MASS_SCALE,
#   PEGASUS_ARM_FRICTION_COMP, and sets ARM_PT_CORR (true|false) for the
#   torque-mode arm stack's passthrough_corrections:= argument.
# Precedence everywhere: environment > yaml > built-in default.
#
# THIS IS A VERBATIM COPY of the block in
# start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh
# (lines "ARM SERVO MODEL" .. "ARM GRAVITY MISMATCH"), lifted so that the
# DECOUPLED rig (start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh)
# reads its own yaml's identical section with identical semantics. The WB
# launchers still carry the block inline on purpose -- they fly tomorrow and
# were not touched; fold them onto this file after the flight test. Keep the
# two in step until then (diff them).
# -- ARM SERVO MODEL: the current loop's residual, in the plant (2026-09-09) ---
# THE SWITCH LIVES HERE, not in the controller YAML: this is a property of the
# PLANT, and the paired fsc_autopilot_ros2 node never sees it -- that mismatch
# is the effect under test. The arm controller now closes a 1.5 Hz software
# CURRENT LOOP around Dynamixel Mode 16, which REMOVES the back-EMF droop this
# block used to carry (j2 torque delivery 93.3 -> 99.4 %, j3 87.5 -> 91.8 %) and
# leaves a zero-mean residual current error in its place. Design and bench
# numbers: fsc_open_manipulator/doc/"Current Loop Design.md" and
# doc/current_error_all.png; implementation in .../robotic_arm/servo_model.py.
#
#   current  ON  - the arm as it is today: residual current noise, band-limited
#                  at 5 Hz, made torque through Kt
#   ideal    OFF - the commanded effort applied exactly, for the A/B
#
# THE RESIDUAL IS PER JOINT (2026-09-11), and the asymmetry is the plant: the
# arm ships current_loop_bandwidth_hz_joints [0.0, 1.5, 1.5, 0.0], because the
# trim helps the two loaded joints and HURTS j1/j4, whose commanded current is
# mostly noise. So the two joints with NO trim are the two NOISIEST -- in-band
# bench rms 11 / 4.4 / 6.8 / 11 mA, which this file's yaml carries as the total
# rms [15.6, 6.2, 9.6, 15.6] mA (a first-order low pass puts half its variance
# below its own corner, so total = sqrt(2) x in-band).
#
# Override without editing this file:
#   PEGASUS_ARM_SERVO_MODEL=ideal <this script> <config>
# The SOURCE OF TRUTH is the paired controller yaml, so one file describes the
# whole run (user request, 2026-09-04). The whole-body NODE never declares these
# keys -- rclcpp ignores them -- they are here for the plant, which is why they
# are prefixed sim_ and why this launcher is what reads them.
# Precedence: environment > yaml > built-in default.
WB_SIM_YAML="${WB_SIM_YAML:-$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml}"
yaml_scalar() {  # $1 = key; prints the value, or nothing if absent/commented
  [[ -r "$WB_SIM_YAML" ]] || return 0
  sed -nE "s/^[[:space:]]*$1:[[:space:]]*([^#[:space:]]+).*/\\1/p" "$WB_SIM_YAML" | head -1
}
if [[ -z "${PEGASUS_ARM_SERVO_MODEL:-}" ]]; then
  case "$(yaml_scalar sim_arm_current_noise_enable)" in
    true|True|TRUE|1)    PEGASUS_ARM_SERVO_MODEL=current ;;
    false|False|FALSE|0) PEGASUS_ARM_SERVO_MODEL=ideal ;;
    "") PEGASUS_ARM_SERVO_MODEL=current
        echo "NOTE: sim_arm_current_noise_enable not found in $WB_SIM_YAML -- defaulting to 'current'." ;;
    *)  echo "ERROR: sim_arm_current_noise_enable must be true or false in $WB_SIM_YAML" >&2; exit 2 ;;
  esac
  if [[ "$PEGASUS_ARM_SERVO_MODEL" != "ideal" ]]; then
    for KV in "PEGASUS_ARM_CURRENT_NOISE_BW_HZ:sim_arm_current_noise_bw_hz" \
              "PEGASUS_ARM_CURRENT_NOISE_SEED:sim_arm_current_noise_seed"; do
      VAR="${KV%%:*}"; KEY="${KV##*:}"
      if [[ -z "${!VAR:-}" ]]; then
        V="$(yaml_scalar "$KEY")"
        [[ -n "$V" ]] && export "$VAR=$V"
      fi
    done
    # The amplitude is PER JOINT: sim_arm_current_noise_a_j1..j4, joined into
    # the "a1,a2,a3,a4" form 06 parses. ALL FOUR OR NONE -- a partial list
    # would silently model three joints, the same rule the counts block uses.
    # The pre-2026-09-11 scalar sim_arm_current_noise_a still works as a
    # fallback, so an older yaml keeps its meaning.
    if [[ -z "${PEGASUS_ARM_CURRENT_NOISE_A:-}" ]]; then
      NOISE_LIST=""
      for J in j1 j2 j3 j4; do
        V="$(yaml_scalar "sim_arm_current_noise_a_$J")"
        [[ -n "$V" ]] || { NOISE_LIST=""; break; }
        NOISE_LIST="${NOISE_LIST:+$NOISE_LIST,}$V"
      done
      [[ -z "$NOISE_LIST" ]] && NOISE_LIST="$(yaml_scalar sim_arm_current_noise_a)"
      [[ -n "$NOISE_LIST" ]] && export PEGASUS_ARM_CURRENT_NOISE_A="$NOISE_LIST"
    fi
  fi
  echo "Arm servo model taken from $(basename "$WB_SIM_YAML")"
fi
export PEGASUS_ARM_SERVO_MODEL="${PEGASUS_ARM_SERVO_MODEL:-current}"

# ── ROBUSTNESS INJECTION: plant-side model uncertainty ──────────────────────
# Same precedence rule as the servo model: environment > yaml > built-in.
# Absent keys leave the plant nominal, so a yaml without them behaves as before.
for KV in "PEGASUS_PLANT_MASS_SCALE:sim_plant_mass_scale" \
          "PEGASUS_PLANT_INERTIA_SCALE:sim_plant_inertia_scale" \
          "PEGASUS_PLANT_COM_SHIFT_X:sim_plant_com_shift_x" \
          "PEGASUS_PLANT_COM_SHIFT_Y:sim_plant_com_shift_y" \
          "PEGASUS_PLANT_COM_SHIFT_Z:sim_plant_com_shift_z" \
          "PEGASUS_PLANT_KF_SCALE:sim_plant_kf_scale"; do
  VAR="${KV%%:*}"; KEY="${KV##*:}"
  if [[ -z "${!VAR:-}" ]]; then
    V="$(yaml_scalar "$KEY")"
    [[ -n "$V" ]] && export "$VAR=$V"
  fi
done
if [[ -n "${PEGASUS_PLANT_MASS_SCALE:-}${PEGASUS_PLANT_INERTIA_SCALE:-}${PEGASUS_PLANT_COM_SHIFT_X:-}${PEGASUS_PLANT_COM_SHIFT_Y:-}${PEGASUS_PLANT_COM_SHIFT_Z:-}" ]]; then
  echo -e "\033[1;31mMODEL-UNCERTAINTY INJECTION ACTIVE: mass x${PEGASUS_PLANT_MASS_SCALE:-1.0}, inertia x${PEGASUS_PLANT_INERTIA_SCALE:-1.0}, CoM shift (${PEGASUS_PLANT_COM_SHIFT_X:-0},${PEGASUS_PLANT_COM_SHIFT_Y:-0},${PEGASUS_PLANT_COM_SHIFT_Z:-0}) m\033[0m"
fi
if [[ -n "${PEGASUS_PLANT_KF_SCALE:-}" && "${PEGASUS_PLANT_KF_SCALE}" != "1.0" ]]; then
  # A PLANT-side k_f error is seen by SAFETY and DIRECT alike, unlike an
  # alloc_thrust_coeff detune which only DIRECT goes through.
  echo -e "\033[1;31mTHRUST-LOSS INJECTION ACTIVE (PLANT SIDE, both modes): k_f x${PEGASUS_PLANT_KF_SCALE}\033[0m"
  echo -e "\033[1;33mThe PLANT is perturbed; the controller believes the nominal model.\033[0m"
fi

# ── ARM COUNT <-> TORQUE: the digital command path ──────────────────────────
# The real arm command is an int16 Goal PWM register, not a torque. Same
# precedence rule again: environment > yaml > built-in. Absent keys leave the
# arm a perfectly calibrated continuous torque source, i.e. the pre-2026-09-08
# plant, so a yaml without them behaves exactly as before.
if [[ -z "${PEGASUS_ARM_COUNTS_ENABLE:-}" ]]; then
  case "$(yaml_scalar sim_arm_counts_enable)" in
    true|True|TRUE|1)    export PEGASUS_ARM_COUNTS_ENABLE=1 ;;
    false|False|FALSE|0) export PEGASUS_ARM_COUNTS_ENABLE=0 ;;
    "")                  export PEGASUS_ARM_COUNTS_ENABLE=0 ;;
    *)  echo "ERROR: sim_arm_counts_enable must be true or false in $WB_SIM_YAML" >&2; exit 2 ;;
  esac
fi
for KV in "PEGASUS_ARM_COUNTS_NOMINAL:sim_arm_counts_nominal" \
          "PEGASUS_ARM_COUNTS_TRUE:sim_arm_counts_true"; do
  VAR="${KV%%:*}"; KEY="${KV##*:}"
  [[ -n "${!VAR:-}" ]] && continue
  LIST=""
  for J in j1 j2 j3 j4; do
    V="$(yaml_scalar "${KEY}_$J")"
    # All four or none: a partial list would silently model three joints.
    [[ -n "$V" ]] || { LIST=""; break; }
    LIST="${LIST:+$LIST,}$V"
  done
  [[ -n "$LIST" ]] && export "$VAR=$LIST"
done
# Only claim ACTIVE when something actually changes: quantization on, or the
# two calibrations genuinely differ. Setting TRUE to the nominal value is a
# MATCHED run and must not look like an injection -- a banner that fires either
# way is worth nothing.
ARM_COUNTS_MISMATCH=0
if [[ -n "${PEGASUS_ARM_COUNTS_TRUE:-}" ]]; then
  ARM_COUNTS_MISMATCH=$(awk -v a="${PEGASUS_ARM_COUNTS_NOMINAL:-162.4,154.0,150.5,153.4}" \
                            -v b="$PEGASUS_ARM_COUNTS_TRUE" '
    BEGIN { n=split(a,A,","); split(b,B,",");
            for (i=1;i<=n;i++) if ((A[i]-B[i])^2 > 1e-18) { print 1; exit } print 0 }')
fi
if [[ "${PEGASUS_ARM_COUNTS_ENABLE:-0}" == "1" || "$ARM_COUNTS_MISMATCH" == "1" ]]; then
  echo -e "\033[1;31mARM COUNT<->TORQUE PATH ACTIVE: register quantized=${PEGASUS_ARM_COUNTS_ENABLE:-0}, counts/N.m nominal [${PEGASUS_ARM_COUNTS_NOMINAL:-calibrated}] vs true [${PEGASUS_ARM_COUNTS_TRUE:-calibrated}]\033[0m"
  if [[ "$ARM_COUNTS_MISMATCH" == "1" ]]; then
    echo -e "\033[1;33mDelivered torque scales as nominal/true; the controller is not told. Isaac prints the percentage.\033[0m"
  else
    echo -e "\033[1;33mCalibration MATCHED: register quantization only, no gain error.\033[0m"
  fi
else
  echo "Arm count<->torque path off: continuous torque, perfect calibration."
fi

# ── GEARBOX FRICTION + ARM GRAVITY MISMATCH (2026-09-14) ─────────────────────
# The calibration report's friction (eq. 5) on the PLANT, scaled, and the arm
# link masses scaled -- while the flight law, the arm controller's correction
# and 06's own hold keep the nominal arm. 1.05 / 1.05 = the requested 5 %
# imperfect friction + gravity compensation. Same precedence: env > yaml >
# built-in (0 / 1.0 = the pre-2026-09-14 plant).
for KV in "PEGASUS_ARM_FRICTION_SCALE:sim_arm_friction_scale" \
          "PEGASUS_ARM_FRICTION_WIDTH:sim_arm_friction_width" \
          "PEGASUS_ARM_MASS_SCALE:sim_arm_mass_scale"; do
  VAR="${KV%%:*}"; KEY="${KV##*:}"
  if [[ -z "${!VAR:-}" ]]; then
    V="$(yaml_scalar "$KEY")"
    [[ -n "$V" ]] && export "$VAR=$V"
  fi
done
# THE COMPENSATION IS COUPLED TO THE PLANT. The arm controller's friction
# feed-forward (torque_controller_isaac_aerial.yaml, passthrough_auxiliary_terms)
# is switched ON exactly when the plant has friction, and OFF when it has none
# -- a feed-forward against a frictionless plant is a pure disturbance.
# PEGASUS_ARM_FRICTION_COMP=0|1 overrides (the "uncompensated" A/B).
if [[ -z "${PEGASUS_ARM_FRICTION_COMP:-}" ]]; then
  if awk -v s="${PEGASUS_ARM_FRICTION_SCALE:-0}" 'BEGIN{exit !(s+0 > 0)}'; then
    export PEGASUS_ARM_FRICTION_COMP=1
  else
    export PEGASUS_ARM_FRICTION_COMP=0
  fi
fi
case "$PEGASUS_ARM_FRICTION_COMP" in 0|1) ;;
  *) echo "ERROR: PEGASUS_ARM_FRICTION_COMP must be 0 or 1 (got '$PEGASUS_ARM_FRICTION_COMP')" >&2; exit 2 ;;
esac
ARM_PT_CORR=$([[ "$PEGASUS_ARM_FRICTION_COMP" == 1 ]] && echo true || echo false)
if awk -v s="${PEGASUS_ARM_FRICTION_SCALE:-0}" 'BEGIN{exit !(s+0 > 0)}'; then
  echo -e "\033[1;31mGEARBOX FRICTION ACTIVE on the plant: x${PEGASUS_ARM_FRICTION_SCALE} the report's fc/mu (tanh width ${PEGASUS_ARM_FRICTION_WIDTH:-0.015} rad/s); arm-side friction compensation ${ARM_PT_CORR} (passthrough_auxiliary_terms).\033[0m"
else
  echo "Gearbox friction off (frictionless plant); arm-side friction compensation ${ARM_PT_CORR}."
fi
if [[ -n "${PEGASUS_ARM_MASS_SCALE:-}" && "${PEGASUS_ARM_MASS_SCALE}" != "1.0" ]]; then
  echo -e "\033[1;31mARM GRAVITY MISMATCH ACTIVE: arm link masses x${PEGASUS_ARM_MASS_SCALE}; every model keeps the nominal arm.\033[0m"
fi
