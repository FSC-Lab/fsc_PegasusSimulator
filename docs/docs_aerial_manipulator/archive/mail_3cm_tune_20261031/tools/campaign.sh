#!/usr/bin/env bash
# Back-to-back run_mail_3cm.sh flights, a clean slate before and after each, every bag processed
# into runs/work for compare.py:
#
#   MAIL_REALTIME=1 MAIL_SIM_RTF=0.51 campaign.sh <tag>=<yaml|default> [<tag>=<yaml|default> ...]
#
# MAIL_OTHER_CPUS=<list>   run everything this script starts (stack, PX4, ground stations, driver,
#                          recorder) on these CPUs; Isaac goes where PEGASUS_ISAAC_CPUS (in MAIL_SCENE_ENV)
#                          or the machine config's ISAAC_CPUS puts it. Unset = no pinning.
#
# <yaml> = a candidate from make_gain_yaml.py (default = the shipped sim mail yaml). A progress line per
# flight goes to stdout; each flight's own logs stay in runs/.
set -o pipefail
if [[ -n "${MAIL_OTHER_CPUS:-}" && -z "${_MAIL_PINNED:-}" ]]; then
  export _MAIL_PINNED=1
  exec taskset -c "$MAIL_OTHER_CPUS" "$0" "$@"
fi
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$(cd -- "$HERE/.." && pwd)/runs"
source "$HERE/../../../../../scripts/config/fsc_lab_machine.conf"
clean() {
  "$FSC_AUTOPILOT_WS/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh" > /dev/null 2>&1
  "$FSC_PEGASUS_ROOT/scripts/kill_stale_sim_processes.sh" -y > /dev/null 2>&1
  for p in $(pgrep -f "[k]onsole --noclose -e bash -lc $FSC_PEGASUS_ROOT/scripts/indoor_sim"); do kill "$p" 2>/dev/null; done
  sleep 3
}
for spec in "$@"; do
  tag="${spec%%=*}"; yaml="${spec#*=}"
  clean
  if [[ "$yaml" == default ]]; then unset MAIL_YAML; else export MAIL_YAML="$yaml"; fi
  case "$tag" in c4_*) export MAIL_LAYOUT=4cm ;; *) export MAIL_LAYOUT=3cm ;; esac   # the tag names the layout
  echo "[$(date +%T)] $tag start ($MAIL_LAYOUT, ${MAIL_YAML:-shipped yaml})"
  "$HERE/run_mail_3cm.sh" "$tag" > "$RUNS/$tag.runner.log" 2>&1
  rc=$?
  clean
  if [[ -d "$RUNS/$tag.bag" ]]; then
    "$HERE/analysis/process_bag.sh" "$RUNS/$tag.bag" "$tag" "$RUNS/work" > "$RUNS/$tag.process.log" 2>&1
  fi
  echo "[$(date +%T)] $tag done (runner rc $rc; $(grep -c -E 'ABORT|Abort' "$RUNS/$tag.log" 2>/dev/null) abort lines; result: $(grep -E -m1 'COMPLETE|aborted|ABORTED|FAILED' "$RUNS/$tag.log" 2>/dev/null | cut -c1-80))"
done
