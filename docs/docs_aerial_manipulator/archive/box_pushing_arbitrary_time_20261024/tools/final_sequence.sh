#!/usr/bin/env bash
# 2026-10-25: the arbitrary-time rig's final results campaign (the tuned yaml, its own guards incl.
# the sideways box guard), then the K 9 A/B at the same guards (two flights). Strictly sequential.
PEG="$HOME/fsc_PegasusSimulator"; HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
echo "[$(date +%T)] campaign"
METHOD=arbitrary_time MAX_ATTEMPTS=4 TAG_PREFIX=final bash "$PEG/results/utils/run_box_pushing_campaign.sh" 3 \
  > "$PEG/results/simulation_results/Isaac_simulation_data/box_pushing/campaign_atc.out" 2>&1 < /dev/null
echo "[$(date +%T)] campaign done"
[[ -f "$HERE/../runs/STOP" ]] && { echo "STOP"; exit 0; }
bash "$HERE/atc_chain.sh" k9a_final_guards:variants/t_v7_k9_final_guards.yaml k9b_final_guards:variants/t_v7_k9_final_guards.yaml
echo "[$(date +%T)] sequence done"
