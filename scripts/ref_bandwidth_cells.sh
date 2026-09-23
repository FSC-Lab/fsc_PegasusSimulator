#!/bin/bash
# IS THE REFERENCE'S LOOP-DEPENDENCE A BANDWIDTH EFFECT?  (the user's hypothesis, 2026-09-23)
#
# The ttr sweep showed e = (static bias) + (velocity-error constant) x d(theta_ref)/dt, R^2 0.999:
#   geometric  +5.03 deg, 3.77 s      INDI  -0.04 deg, 0.82 s      designed 2 zeta/wn = 0.45 s
# So the geometric loop misses its own design by 8.4x while INDI nearly meets it AT THE SAME
# NOMINAL wn (3.15 vs 3.1).  The node's own note says why it cannot simply be sped up: "the inner
# loop CANNOT damp the short period -- the ~65 ms actuator lag forbids speeding it up (see
# pitch_ratio)", which is why pitch_ratio ships at 0.  Four falsifiable predictions:
#   geo --kR 9            if pitch attitude is really the OUTER's, 3x the inner gain does NOT
#                         fix 3.77 s.  If it collapses toward 0.26 s, the inner owned it after all.
#   geo --pitch_ratio 2   turns the disabled gain schedule ON (kR_p ~ qbar).  The recorded reason
#                         says speeding the inner up is forbidden -> predict NO better, maybe worse.
#   indi --indi_wn 6      if INDI's constant is genuinely its OWN designed bandwidth it should fall
#                         ~2x (0.82 -> ~0.45).  If it saturates, the actuator is the wall.
#   indi --indi_wn 2      and it should RISE ~1.55x (-> ~1.27 s).  A knob that moves BOTH ways is
#                         the difference between a mechanism and a coincidence.
S=${OUT:-$HOME/flight_bags/ref_bw}; mkdir -p "$S"
IP=~/envs/isaac_env/bin/python3
HERE=$(cd "$(dirname "$0")" && pwd)
BASE="--est mdob --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 --noise_acc 0.02 --l1 --decel_at 60 --dur 90"
run () { local tag=$1 kind=$2; shift 2
  [ -s "$S/$tag.csv" ] && return
  timeout 1200 $IP "$HERE/ref_probe.py" $kind "$S/$tag.csv" -- $BASE "$@" > "$S/$tag.log" 2>&1; }
for s in 1 2; do
  for t in 6 14; do
    run "kR9_ttr${t}_s$s"   geo  --kR 9        --ttr $t --seed $s &
    run "pr2_ttr${t}_s$s"   geo  --pitch_ratio 2.0 --ttr $t --seed $s &
    run "wn6_ttr${t}_s$s"   indi --indi_wn 3.4,6.0,3.2 --ttr $t --seed $s &
    run "wn2_ttr${t}_s$s"   indi --indi_wn 3.4,2.0,3.2 --ttr $t --seed $s &
    wait
  done
done
echo BW_DONE
