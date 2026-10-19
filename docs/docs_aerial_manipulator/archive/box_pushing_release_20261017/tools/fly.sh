#!/usr/bin/env bash
# One box-pushing flight for the 2026-10-17 RELEASE-WINDOW campaign: the 10-16
# campaign's fly.sh (box_pushing_campaign_20261016/tools/fly.sh -- the final
# configuration: 500 g, 0.50 m / 24 s min-snap, pose [0, 32, 28, 0], the box guards)
# with the driver flying the arm GS's new Exit To Push sequence: push_pull/release
# (the claw's reference back to the CoM anchor), then the gripper opens, then the
# exit on its OPEN report (pl_mission.py --release-at-open).
#   fly.sh <tag> [key=val ...]          (clean slate first, in a SEPARATE call)
# Environment:
#   RELEASE=1      0 = the old sequence (open; exit on the 3 s slot), for an A/B
#   MASS=0.500 DIST=0.50 SPEED= MU_S=0.29 MU_K=0.29 BOX_Y0=   as on 10-16
#   PL_DRIVER_ARGS adds to the driver's arguments; PEGASUS_PUSH_* pass through.
set -o pipefail
TAG="${1:?tag}"; shift
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
P="$C/../push_pull_20261003"
Y=~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_box_pushing.yaml
mkdir -p "$C/runs"
MASS=${MASS:-0.500}; DIST=${DIST:-0.50}; SPEED=${SPEED:-$(python3 -c 'print(0.5/24)')}
MU_S=${MU_S:-0.29}; MU_K=${MU_K:-0.29}
REL=""; [[ "${RELEASE:-1}" == 1 ]] && REL="--release-at-open"
read T Y0 SLOT < <(python3 -c "d=$DIST; v=$SPEED; T=d/v; print(f'{T:.2f} {d/2:.4f} {T+3.0:.2f}')")
Y0=${BOX_Y0:-$Y0}   # BOX_Y0 overrides the centred start (the 0.70 m push: 0.36, 4 cm off the far edge)
/usr/bin/python3 "$P/tools/make_push_pull_yaml.py" > /dev/null
ARGS=("push_pull_push_distance=$DIST" "push_pull_push_time_s=$T" "$@")   # later args win
LAW=(); for kv in "${ARGS[@]}"; do
  if [[ $kv == push_pull_* ]]; then k=${kv%%=*}; v=${kv#*=}
    [[ $v =~ ^-?[0-9]+$ ]] && v="$v.0"   # rclcpp refuses an integer for a double
    grep -q "^    $k:" $Y || { echo "no planner key $k"; exit 1; }
    sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" $Y; echo "  $k -> $v"
  else LAW+=("$kv"); fi; done
[[ ${#LAW[@]} -gt 0 ]] && /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py --file $Y "${LAW[@]}" | grep -- "->"
cp $Y "$C/runs/$TAG.yaml"
{ printf '%s\n' "${ARGS[@]}"; echo "MASS=$MASS"; echo "DIST=$DIST"; echo "MU_S=$MU_S"; echo "MU_K=$MU_K"; echo "BOX_Y0=$Y0"; echo "RELEASE=${RELEASE:-1}"; } > "$C/runs/$TAG.args"
cd "$P" && PL_RUNS_DIR="$C/runs" PUSH_FEEDBACK=${PUSH_FEEDBACK:-raw} PEGASUS_PUSH_BOX_MASS=$MASS \
  PEGASUS_PUSH_FRICTION_STATIC=$MU_S PEGASUS_PUSH_FRICTION_DYNAMIC=$MU_K \
  PEGASUS_PUSH_BOX_XY="${PEGASUS_PUSH_BOX_XY:-1.275,$Y0}" \
  PL_DRIVER_ARGS="--timetable --slot-release $SLOT --slot-land-step 27.0 $REL ${PL_DRIVER_ARGS:-}" \
  bash tools/run_pl.sh $TAG > "$C/runs/$TAG.console.log" 2>&1
/usr/bin/python3 "$P/tools/make_push_pull_yaml.py" > /dev/null
echo "[$TAG] $(grep -E '^completed|^ABORTED' "$C/runs/$TAG.console.log" | head -1 | cut -c1-150)"
