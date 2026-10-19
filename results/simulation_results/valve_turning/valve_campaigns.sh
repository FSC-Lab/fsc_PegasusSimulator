#!/bin/bash
# 2026-10-18: the valve-turning results on the final config (README 15.10), both rigs, back to back
cd /home/shiqi/fsc_PegasusSimulator
O=results/simulation_results/valve_turning
TAG_PREFIX=final_wb bash results/utils/run_valve_turning_campaign.sh 3 > $O/campaign.out 2>&1 < /dev/null
METHOD=coordinated_admittance TAG_PREFIX=final_ca bash results/utils/run_valve_turning_campaign.sh 3 > $O/campaign_cadm.out 2>&1 < /dev/null
echo "BOTH CAMPAIGNS DONE $(date +%T)" >> $O/campaign_cadm.out
