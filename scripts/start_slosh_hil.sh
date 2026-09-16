#!/usr/bin/env bash
#
# Start the HIL stack on the FREE-SURFACE TANK model (branch `slosh`).
#
#   scripts/start_slosh_hil.sh                 # full restart, tank model
#   scripts/start_slosh_hil.sh --if-needed     # skip the restart if the stack is fit
#   scripts/start_slosh_hil.sh --baseline      # the SAME path, baseline VTOL -- the A/B
#   scripts/start_slosh_hil.sh --dry-run       # checks only; launches nothing
#
# WHY THIS EXISTS AND IS NOT JUST `VTOL_SLOSH=1 restart_hil.sh`
# ------------------------------------------------------------
# Because the env var can silently fail to arrive.  The launchers pass Isaac's
# environment INLINE on a tmux send-keys command line -- a pane does not inherit
# the exporting shell's environment -- so a forgotten forward means the stack
# comes up on the BASELINE aircraft while the operator believes the tank is
# flying, and every number from that flight is mislabelled.  This project has
# already paid for mislabelled flights: a glide read as wing-borne flight, a
# frozen sim read as a successful manoeuvre, a whole run flown by the wrong
# controller after a stale DDS discovery.  So this script does not trust the
# variable -- it VERIFIES that the model it asked for is the model that came up,
# and refuses to report READY otherwise.
#
# It also runs the phase-1 gates first (2 s, no Isaac) and puts bags somewhere
# that survives a reboot -- /tmp/bags is the default everywhere else and is how
# the post-extraction verification bag was lost.
set -u
ROOT=/home/pieper/envs/fsc_PegasusSimulator
SESSION=${SESSION:-vtol_smc_hil}          # ONE HIL stack at a time: PX4 4560 and the DDS
                                          # domain are fixed, so the tank stack reuses the
                                          # session (and all the tooling keyed to that name).
                                          # The MODEL is what differs, and that is verified below.
# overridable so the guard itself can be tested without switching branches
MODEL_PY=${MODEL_PY:-$ROOT/extensions/pegasus.simulator/pegasus/simulator/logic/vehicles/vtol_slosh.py}
LAUNCH_LOG=${LAUNCH_LOG:-$HOME/flight_bags/LAUNCH_LOG.txt}

say(){ echo "[slosh_hil] $*"; }

WANT_SLOSH=1
DRY=0
ARGS=()
for a in "$@"; do
  case "$a" in
    --baseline) WANT_SLOSH=0 ;;
    # --dry-run runs every check that does not need a stack -- the branch guard,
    # the gates, the bag path -- and stops before touching Isaac or PX4.  It is
    # how this script itself is tested without burning four minutes, and how you
    # check a machine is ready to fly before committing to a run.
    --dry-run) DRY=1 ;;
    *) ARGS+=("$a") ;;
  esac
done

# --- 1. am I even on a branch that HAS the tank model? ----------------------
# `git checkout dev_branch_logical` and this file is gone; without this check the
# run would fall back to the baseline aircraft and look completely normal.
if [ "$WANT_SLOSH" = "1" ] && [ ! -f "$MODEL_PY" ]; then
  echo "[slosh_hil] !! $MODEL_PY does not exist."
  echo "[slosh_hil]    The tank model lives on branch 'slosh'. You are on "
  echo "[slosh_hil]    '$(git -C "$ROOT" branch --show-current)'.  git checkout slosh"
  exit 1
fi
BRANCH=$(git -C "$ROOT" branch --show-current 2>/dev/null)
COMMIT=$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null)
DIRTY=$(git -C "$ROOT" status --porcelain 2>/dev/null | wc -l)

# --- 2. the phase-1 gates, before burning 4 minutes on a stack --------------
if [ "${SKIP_GATES:-0}" != "1" ]; then
  say "1/4 gates: slosh scaffold + import"
  for t in test_slosh_scaffold test_slosh_import; do
    if ! OUT=$(/usr/bin/python3 "$ROOT/tests/$t.py" 2>&1); then
      echo "$OUT" | tail -12
      echo "[slosh_hil] !! $t FAILED -- not launching. (SKIP_GATES=1 overrides.)"
      exit 1
    fi
  done
  say "    both gates pass"
else
  say "1/4 gates SKIPPED (SKIP_GATES=1)"
fi

# --- 3. hand off to the one real launcher -----------------------------------
# Durable bag directory. Everything else defaults to /tmp/bags, which is wiped on
# reboot -- that is exactly how the 72.7 s post-extraction verification flight was
# lost, and a tank campaign that cannot be compared against its baseline is worthless.
export BAG_DIR=${BAG_DIR:-$HOME/flight_bags/slosh}
export BAG=${BAG:-$([ "$WANT_SLOSH" = 1 ] && echo slosh || echo base)_$(date +%Y%m%d_%H%M%S)}
export VTOL_SLOSH=$WANT_SLOSH
export VTOL_TANK=${VTOL_TANK:-1}
export VTOL_TANK_KG=${VTOL_TANK_KG:-0.502}    # water mass: 0.1 / 0.502 / 0.7 flown; plant AND controllers
export VTOL_TANK_R=${VTOL_TANK_R:-}           # tank radius [m], empty = the plan's 41 mm (0.08 / 0.10 gated)
export VTOL_TANK_DX=${VTOL_TANK_DX:-0}
export VTOL_CG_DX=${VTOL_CG_DX:-0}
export CASC_L1=${CASC_L1:-true}
# The tank AIRCRAFT flies the tank CASCADE (nmpc_cascade_tank_node: the base node with
# the back-transition schedule replaced -- rotors upright before the aft tilt, flare
# capped under the CL peak). Same node name and controller_id, so nothing downstream
# changes. CASC_NODE_PY=<path> overrides; CASC_NODE_PY=base forces the base node.
CTRL_DIR=${FSC_CONTROLLER_DIR:-$HOME/ros2_ws/src/px4_vtol_direct_actuator}
# THE IDENTIFIED AIRFRAME (piece 3, 2026-09-15): every controller constant comes from
# $CTRL_DIR/px4_vtol_direct_actuator/airframe_id.cfg. The READY checks below compare each banner's
# M= / CG_X= against THIS file (no 3.449572 / 0.176 literals here any more), and every banner must
# carry the file's digest (af=): the proof that the node loaded the same file this launcher read.
AF_LINE=$(cd "$CTRL_DIR/px4_vtol_direct_actuator" && /usr/bin/python3 -c "from airframe_id import AIRFRAME as A; print(A.mass.airframe_kg, A.mass.cg_x, A.digest())" 2>/dev/null) || AF_LINE=""
if [ -z "$AF_LINE" ]; then
  echo "[slosh_hil] !! could not load $CTRL_DIR/px4_vtol_direct_actuator/airframe_id.cfg through airframe_id.py (the controller's identified airframe). Not READY."; exit 1
fi
AF_M=${AF_LINE%% *}; AF_REST=${AF_LINE#* }; AF_CG=${AF_REST%% *}; AF_DIGEST=${AF_REST#* }
echo "[slosh_hil] airframe: airframe_id.cfg digest $AF_DIGEST (M_airframe $AF_M kg, CG_X $AF_CG m)${FSC_AIRFRAME_ID:+  OVERRIDE FSC_AIRFRAME_ID=$FSC_AIRFRAME_ID}"
if [ "$WANT_SLOSH" = "1" ] && [ "${VTOL_TANK:-1}" != "0" ]; then
  case "${CASC_NODE_PY:-}" in
    "")   export CASC_NODE_PY="$CTRL_DIR/px4_vtol_direct_actuator/nmpc_cascade_tank_node.py" ;;
    base) unset CASC_NODE_PY ;;
  esac
fi
say "    cascade: $(basename "${CASC_NODE_PY:-nmpc_cascade_node.py}")  l1_on=$CASC_L1  water=$VTOL_TANK_KG kg  tank_R=${VTOL_TANK_R:-0.0414} m  tank_dx=$VTOL_TANK_DX  cg_dx=$VTOL_CG_DX"
if [ "${ADAPTIVE:-1}" != "0" ]; then
  say "    adaptive: $(basename "${ADAPTIVE_NODE_PY:-nmpc_cascade_steso_node.py}") as cascade_adaptive (window 6i-adaptive; GS 'Adaptive NMPC + Geo'; driver CTRL=cascade_adaptive)"
else
  say "    adaptive: OFF (ADAPTIVE=0)"
fi
mkdir -p "$BAG_DIR"

say "2/4 model: $([ "$WANT_SLOSH" = 1 ] && echo "VTOLSlosh (VTOL_TANK=$VTOL_TANK)" || echo 'baseline VTOL')"
say "    branch $BRANCH @ $COMMIT$([ "$DIRTY" -gt 0 ] && echo " (+$DIRTY uncommitted)")"
say "    bags -> $BAG_DIR/$BAG"
if [ "$DRY" = "1" ]; then
  say "DRY RUN -- stopping here. Nothing was launched and NOTHING IS VERIFIED:"
  say "  would run: VTOL_SLOSH=$WANT_SLOSH BAG_DIR=$BAG_DIR BAG=$BAG \\"
  say "             bash $ROOT/scripts/restart_hil.sh ${ARGS[*]-}"
  say "  then verify Isaac logged the model banner before reporting READY."
  exit 0
fi

# ★ DETACHED, THEN POLL -- do not run restart_hil.sh as a plain child.
# On a COLD machine it hands off to start_vtol_smc_hil.sh, whose last line is
# `tmux attach-session`, and whose teardown kills processes in this process
# group: run as a child it takes THIS script down with it, so the verification
# below would never execute and the run would look like it simply vanished
# (night_campaign.sh hit exactly this at 22:59 and solved it the same way).
# setsid puts it in its own session; we then wait for its own READY line.
RESTART_LOG=${RESTART_LOG:-$BAG_DIR/restart_$(date +%H%M%S).log}
WAIT_S=${WAIT_S:-1200}
# ★ NO GPU, NO ISAAC. 2026-09-13 21:45: the NVIDIA driver had been swapped (595-open installed
# 17:43, removed 17:47 via PackageKit) and nouveau had the GA102; Isaac's renderer and PhysX GPU
# both fail at boot ("nvidia kernel module is not loaded"), the pane exits, and this launcher
# then polled for a banner for 12 minutes across three attempts. Refuse in one second instead.
if [ ! -e /dev/nvidia0 ]; then
  echo "[slosh_hil] !! /dev/nvidia0 does not exist: the NVIDIA kernel module is not loaded, so Isaac"
  echo "[slosh_hil]    cannot render or run PhysX on the GPU. Check 'lsmod | grep nvidia', the apt"
  echo "[slosh_hil]    history (/var/log/apt/history.log) for a driver change, reinstall and reboot."
  exit 1
fi
say "3/4 restarting the stack, detached (~4 min cold) -> $RESTART_LOG"
setsid nohup timeout "$WAIT_S" bash "$ROOT/scripts/restart_hil.sh" \
  ${ARGS[@]+"${ARGS[@]}"} > "$RESTART_LOG" 2>&1 < /dev/null &
RPID=$!

# WAIT ON HEALTH, NOT ON THE PROCESS, and accept either signal:
#   * warm path: restart_hil.sh prints "READY." after its own gates (link up, zero
#     poll timeouts, controller nodes alive, no traceback);
#   * COLD path: it hands off to start_vtol_smc_hil.sh, whose last line is
#     `tmux attach-session` -- which cannot succeed without a terminal, so the
#     launcher exits NON-ZERO and never prints READY even when the stack is
#     perfectly up. Waiting on the exit status there would fail every cold start.
# hil_health.sh is the authority in both cases: it checks the stack (link, poll
# timeouts, both controllers, no traceback) AND the vehicle (upright, low,
# stopped, near origin).
OK=0
for _ in $(seq 1 $((WAIT_S / 15))); do
  sleep 15
  if grep -q "READY\." "$RESTART_LOG" 2>/dev/null; then
    OK=1; say "    restart_hil.sh reported READY"; break
  fi
  if H=$(timeout 90 bash "$ROOT/scripts/hil_health.sh" 2>/dev/null); then
    OK=1; say "    healthy: $H"; break
  fi
  if grep -q "HIL LINK NOT HEALTHY\|threw on startup" "$RESTART_LOG" 2>/dev/null; then
    echo "[slosh_hil] !! the restart reported a fault:"
    grep -n "!!" "$RESTART_LOG" | tail -5
    exit 1
  fi
done
if [ "$OK" != "1" ]; then
  echo "[slosh_hil] !! timed out after ${WAIT_S}s with no healthy stack. Last health: ${H:-<none>}"
  echo "[slosh_hil]    restart log tail:"; tail -20 "$RESTART_LOG"
  exit 1
fi

# --- 4. VERIFY the model that actually came up ------------------------------
# 03_vtol_px4_hil.py logs this banner when, and only when, it spawns the tank model.
say "4/4 verifying which model Isaac spawned"
PANE=$(tmux capture-pane -p -J -S -3000 -t "$SESSION:2-isaac" 2>/dev/null)   # -J: join wrapped lines
BANNER=$(echo "$PANE" | grep -c "VTOL_SLOSH set")
if [ "$WANT_SLOSH" = "1" ] && [ "$BANNER" -lt 1 ]; then
  echo "[slosh_hil] !! ASKED FOR THE TANK MODEL AND DID NOT GET IT."
  echo "[slosh_hil]    Isaac never logged 'VTOL_SLOSH set -> VTOLSlosh model'."
  echo "[slosh_hil]    Check, in order:"
  echo "[slosh_hil]      * VTOL_SLOSH is forwarded on the Isaac command line in BOTH"
  echo "[slosh_hil]        restart_hil.sh and start_vtol_smc_hil.sh (a pane does not"
  echo "[slosh_hil]        inherit this shell's environment)"
  echo "[slosh_hil]      * --if-needed skipped the restart, so the RUNNING Isaac is an"
  echo "[slosh_hil]        older baseline one: re-run without --if-needed"
  echo "[slosh_hil]      * the scene imported fsc_cad_vtol_slosh_config (tmux window 2-isaac)"
  echo "[slosh_hil]    DO NOT FLY THIS AND CALL IT A TANK RESULT."
  exit 1
fi
# Phase 2: with the tank mounted the vehicle logs its mass fold at construction.
# Verify THAT too -- "VTOLSlosh with no tank" and "VTOLSlosh with the tank" are
# different aircraft, and the launcher must say which one it verified.
TANK_LINE=$(echo "$PANE" | grep -o "\[TANK\] dead mass.*" | tail -1 | cut -c1-90)
if [ "$WANT_SLOSH" = "1" ]; then
  if [ "${VTOL_TANK:-1}" != "0" ] && [ -z "$TANK_LINE" ]; then
    echo "[slosh_hil] !! the tank model came up but never logged '[TANK] dead mass' --"
    echo "[slosh_hil]    the tank was NOT mounted. Check VTOL_TANK and the scene."
    exit 1
  fi
  [ -n "$TANK_LINE" ] && say "    $TANK_LINE"
  # THE MASS: the scene prints "water=<kg> kg" from the tank it built; it must be the one asked
  # for (VTOL_TANK_KG travels inline on the Isaac line -- a stale launcher would drop it and
  # Isaac would fly the default 0.502 under a controller told 0.7).
  if [ "${VTOL_TANK:-1}" != "0" ]; then
    WKG=$(echo "$PANE" | grep -o "water=[0-9.]* kg" | tail -1 | grep -o "[0-9.]*")
    if [ -z "$WKG" ] || ! awk -v a="$WKG" -v b="$VTOL_TANK_KG" 'BEGIN{exit (a-b<0.002 && b-a<0.002)?0:1}'; then
      echo "[slosh_hil] !! asked for VTOL_TANK_KG=$VTOL_TANK_KG but Isaac's tank banner says water=${WKG:-<none>} kg."
      echo "[slosh_hil]    The plant is not the aircraft the controllers model. Not READY."
      exit 1
    fi
    say "    water: $WKG kg confirmed in Isaac's log"
    # THE SHAPE: the same banner prints "R=<mm> mm" from the tank it built (VTOL_TANK_R
    # travels inline on the Isaac line like the mass; a dropped forward flies the 41 mm tank
    # under a campaign that believes it flew the 80 mm one)
    RMM=$(echo "$PANE" | grep -o "R=[0-9]* mm" | tail -1 | grep -o "[0-9]*")
    WANT_MM=$(awk -v r="${VTOL_TANK_R:-0.0414}" 'BEGIN{printf "%d", r*1000+0.5}')
    if [ -z "$RMM" ] || [ "$RMM" != "$WANT_MM" ]; then
      echo "[slosh_hil] !! asked for a ${WANT_MM} mm tank (VTOL_TANK_R=${VTOL_TANK_R:-unset}) but Isaac's banner says R=${RMM:-<none>} mm. Not READY."
      exit 1
    fi
    say "    tank radius: $RMM mm confirmed in Isaac's log"
  fi
  # phase 3: asked for the free surface -> the banner must say so
  if [ "${VTOL_TANK:-1}" = "slosh" ]; then
    P3=$(echo "$PANE" | grep -o "SLOSH (phase 3, pendulum)" | tail -1)
    if [ -z "$P3" ]; then
      echo "[slosh_hil] !! VTOL_TANK=slosh but Isaac logged a dead-mass tank, not the pendulum."
      exit 1
    fi
    say "    free surface: $P3 confirmed in Isaac's log"
  fi
fi
if [ "$WANT_SLOSH" = "0" ] && [ "$BANNER" -ge 1 ]; then
  echo "[slosh_hil] !! ASKED FOR THE BASELINE AND GOT THE TANK MODEL (banner present)."
  echo "[slosh_hil]    Re-run without --if-needed to force a fresh Isaac."
  exit 1
fi

# ★ PIPE THE PANES TO FILES.  When the stack died on 2026-09-12 there was nothing
# to look at: tmux was gone and with it every pane's scrollback, so the cause is
# still unknown. A pane log costs nothing and is the only forensic trail a dead
# stack leaves.
for w in 2-isaac 1-px4 6h-cascade 6i-adaptive; do
  tmux pipe-pane -o -t "$SESSION:$w" "cat >> $BAG_DIR/pane_${w}_$(date +%H%M%S).log" 2>/dev/null
done
say "    pane logs -> $BAG_DIR/pane_*.log"

# and VERIFY which cascade came up: the variant prints its schedule banner on start.
# -J joins the lines tmux wrapped at the pane width: the tank banner is 128 characters and its
# "M=<kg>" tail sat on the next captured line, so the mass check saw nothing (2026-09-13 17:12).
CPANE=$(tmux capture-pane -p -J -S -400 -t "$SESSION:6h-cascade" 2>/dev/null)
CBANNER=$(echo "$CPANE" | grep -o "\[cascade_tank\] schedule:.*" | tail -1 | cut -c1-240)
case "${CASC_NODE_PY:-}" in
  *tank*)
    if [ -z "$CBANNER" ]; then
      echo "[slosh_hil] !! asked for the TANK cascade and its banner never appeared in 6h-cascade."
      echo "[slosh_hil]    The base cascade (or nothing) is running. Not READY."
      exit 1
    fi
    say "    $CBANNER"
    # The banner proves which node STARTED, not that it is still running: the 2026-09-13
    # guidance wiring died in the constructor two lines after the banner (a float() on the
    # namespace string), the launcher reported READY on the banner alone, and the mission was
    # flown by the geometric controller. A pane traceback or a missing node process is a fault.
    if echo "$CPANE" | grep -qi traceback; then
      echo "[slosh_hil] !! the TANK cascade printed its banner and then THREW:"
      echo "$CPANE" | grep -i -A3 traceback | tail -8; exit 1
    fi
    if ! pgrep -f "nmpc_cascade_tank_node" >/dev/null 2>&1; then
      echo "[slosh_hil] !! the TANK cascade printed its banner but no nmpc_cascade_tank_node process is alive. Not READY."; exit 1
    fi
    # and its MASS: the banner's M must be the airframe (3.4496) plus the water asked for
    CM=$(echo "$CBANNER" | grep -o "M=[0-9.]* kg" | grep -o "[0-9.]*")
    if [ -z "$CM" ] || ! awk -v a="$CM" -v b="$VTOL_TANK_KG" -v m="$AF_M" 'BEGIN{exit ((a-m-b)<0.002 && (m+b-a)<0.002)?0:1}'; then
      echo "[slosh_hil] !! the cascade models M=${CM:-<none>} kg but VTOL_TANK_KG=$VTOL_TANK_KG asks for $(awk -v b="$VTOL_TANK_KG" -v m="$AF_M" 'BEGIN{printf "%.4f", m+b}')."
      echo "[slosh_hil]    The env var did not reach the 6h-cascade pane. Not READY."
      exit 1
    fi
    # and its CG: a tank mounted off the CG (VTOL_TANK_DX) moves it, in the plant's fold and
    # (since 2026-09-14) in so3.CG_X alike -- the banner's CG_X must be the file's cg_x + dx * water / M
    CCG=$(echo "$CBANNER" | grep -o "CG_X=[0-9.]* m" | grep -o "[0-9.]*")
    WANT_CG=$(awk -v dx="${VTOL_TANK_DX:-0}" -v b="$VTOL_TANK_KG" -v m="$AF_M" -v c="$AF_CG" 'BEGIN{printf "%.4f", c + dx*b/(m+b)}')
    if [ -z "$CCG" ] || ! awk -v a="$CCG" -v w="$WANT_CG" 'BEGIN{exit ((a-w)<0.0015 && (w-a)<0.0015)?0:1}'; then
      echo "[slosh_hil] !! the cascade models CG_X=${CCG:-<none>} but VTOL_TANK_DX=${VTOL_TANK_DX:-0} with $VTOL_TANK_KG kg puts it at $WANT_CG. Not READY."; exit 1
    fi
    CAF=$(echo "$CBANNER" | grep -o "af=[0-9a-f]*" | cut -d= -f2)
    if [ "$CAF" != "$AF_DIGEST" ]; then
      echo "[slosh_hil] !! the cascade loaded airframe file ${CAF:-<none>} but this launcher read $AF_DIGEST. Not READY."; exit 1
    fi
    say "    cascade mass: $CM kg = airframe + $VTOL_TANK_KG kg of water; CG_X $CCG (tank_dx ${VTOL_TANK_DX:-0})" ;;
  *steso*)
    # the STESO variant: the tank schedule inside, its own banner, its own process name
    SBANNER=$(echo "$CPANE" | grep -o "\[cascade+steso\] estimator=.*" | tail -1 | cut -c1-240)
    if [ -z "$SBANNER" ]; then
      echo "[slosh_hil] !! asked for the STESO cascade and its banner never appeared in 6h-cascade. Not READY."; exit 1
    fi
    say "    $SBANNER"
    if echo "$CPANE" | grep -qi traceback; then
      echo "[slosh_hil] !! the STESO cascade printed its banner and then THREW:"
      echo "$CPANE" | grep -i -A3 traceback | tail -8; exit 1
    fi
    if ! pgrep -f "nmpc_cascade_steso_node" >/dev/null 2>&1; then
      echo "[slosh_hil] !! the STESO cascade printed its banner but no nmpc_cascade_steso_node process is alive. Not READY."; exit 1
    fi
    CM=$(echo "$SBANNER" | grep -o "M=[0-9.]* kg" | grep -o "[0-9.]*")
    if [ -z "$CM" ] || ! awk -v a="$CM" -v b="$VTOL_TANK_KG" -v m="$AF_M" 'BEGIN{exit ((a-m-b)<0.002 && (m+b-a)<0.002)?0:1}'; then
      echo "[slosh_hil] !! the STESO cascade models M=${CM:-<none>} kg but VTOL_TANK_KG=$VTOL_TANK_KG asks for $(awk -v b="$VTOL_TANK_KG" -v m="$AF_M" 'BEGIN{printf "%.4f", m+b}'). Not READY."; exit 1
    fi
    # and its CG: a tank mounted off the CG (VTOL_TANK_DX) moves it, in the plant's fold and
    # (since 2026-09-14) in so3.CG_X alike -- the banner's CG_X must be the file's cg_x + dx * water / M
    CCG=$(echo "$SBANNER" | grep -o "CG_X=[0-9.]* m" | grep -o "[0-9.]*")
    WANT_CG=$(awk -v dx="${VTOL_TANK_DX:-0}" -v b="$VTOL_TANK_KG" -v m="$AF_M" -v c="$AF_CG" 'BEGIN{printf "%.4f", c + dx*b/(m+b)}')
    if [ -z "$CCG" ] || ! awk -v a="$CCG" -v w="$WANT_CG" 'BEGIN{exit ((a-w)<0.0015 && (w-a)<0.0015)?0:1}'; then
      echo "[slosh_hil] !! the cascade models CG_X=${CCG:-<none>} but VTOL_TANK_DX=${VTOL_TANK_DX:-0} with $VTOL_TANK_KG kg puts it at $WANT_CG. Not READY."; exit 1
    fi
    CAF=$(echo "$CBANNER" | grep -o "af=[0-9a-f]*" | cut -d= -f2)
    if [ "$CAF" != "$AF_DIGEST" ]; then
      echo "[slosh_hil] !! the cascade loaded airframe file ${CAF:-<none>} but this launcher read $AF_DIGEST. Not READY."; exit 1
    fi
    say "    cascade mass: $CM kg = airframe + $VTOL_TANK_KG kg of water; CG_X $CCG (tank_dx ${VTOL_TANK_DX:-0}) (STESO estimator, tank schedule)" ;;
  *)
    [ -n "$CBANNER" ] && { echo "[slosh_hil] !! the TANK cascade is running but was not asked for."; exit 1; }
    say "    cascade: base node confirmed" ;;
esac

# THE ADAPTIVE CASCADE (window 6i-adaptive, id cascade_adaptive; 2026-09-14): the same checks --
# banner, no traceback, process alive, mass and CG paired -- when the launcher started it.
if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx 6i-adaptive; then
  APANE=$(tmux capture-pane -p -J -S -400 -t "$SESSION:6i-adaptive" 2>/dev/null)
  ABANNER=$(echo "$APANE" | grep -o "\[cascade+steso\] estimator=.*" | tail -1 | cut -c1-260)
  if [ -z "$ABANNER" ]; then
    echo "[slosh_hil] !! the ADAPTIVE cascade window exists but its [cascade+steso] banner never appeared. Not READY."; exit 1
  fi
  say "    $ABANNER"
  if echo "$APANE" | grep -qi traceback; then
    echo "[slosh_hil] !! the ADAPTIVE cascade printed its banner and then THREW:"; echo "$APANE" | grep -i -A3 traceback | tail -8; exit 1
  fi
  if ! ros2 node list 2>/dev/null | grep -qx "/nmpc_cascade_control_adaptive"; then
    echo "[slosh_hil] !! the ADAPTIVE cascade printed its banner but /nmpc_cascade_control_adaptive is not a live node. Not READY."; exit 1
  fi
  # the ESTIMATOR the banner names must be the one asked for (CASC_EST; 2026-09-15: steso | leso |
  # mdob | pwc) -- the piece-2 audit found this check missing: a stack running the wrong kind
  # would have been reported READY and the comparison flights would carry no proof of their kind
  AEST=$(echo "$ABANNER" | grep -o "estimator=[a-z]*" | cut -d= -f2)
  if [ "$AEST" != "${CASC_EST:-steso}" ]; then
    echo "[slosh_hil] !! the ADAPTIVE cascade runs estimator=${AEST:-<none>} but CASC_EST=${CASC_EST:-steso} was asked for. Not READY."; exit 1
  fi
  AM=$(echo "$ABANNER" | grep -o "M=[0-9.]* kg" | grep -o "[0-9.]*")
  if [ -z "$AM" ] || ! awk -v a="$AM" -v b="$VTOL_TANK_KG" -v m="$AF_M" 'BEGIN{exit ((a-m-b)<0.002 && (m+b-a)<0.002)?0:1}'; then
    echo "[slosh_hil] !! the ADAPTIVE cascade models M=${AM:-<none>} kg but VTOL_TANK_KG=$VTOL_TANK_KG asks for $(awk -v b="$VTOL_TANK_KG" -v m="$AF_M" 'BEGIN{printf "%.4f", m+b}'). Not READY."; exit 1
  fi
  ACG=$(echo "$ABANNER" | grep -o "CG_X=[0-9.]* m" | grep -o "[0-9.]*")
  WANT_CG=$(awk -v dx="${VTOL_TANK_DX:-0}" -v b="$VTOL_TANK_KG" -v m="$AF_M" -v c="$AF_CG" 'BEGIN{printf "%.4f", c + dx*b/(m+b)}')
  if [ -z "$ACG" ] || ! awk -v a="$ACG" -v w="$WANT_CG" 'BEGIN{exit ((a-w)<0.0015 && (w-a)<0.0015)?0:1}'; then
    echo "[slosh_hil] !! the ADAPTIVE cascade models CG_X=${ACG:-<none>} but the tank puts it at $WANT_CG. Not READY."; exit 1
  fi
  AAF=$(echo "$ABANNER" | grep -o "af=[0-9a-f]*" | cut -d= -f2)
  if [ "$AAF" != "$AF_DIGEST" ]; then
    echo "[slosh_hil] !! the ADAPTIVE cascade loaded airframe file ${AAF:-<none>} but this launcher read $AF_DIGEST. Not READY."; exit 1
  fi
  # the IMU-force-twin copy (ADAPTIVE_NODE_PY=.../nmpc_cascade_imu_node.py, 2026-09-15): its own banner
  # must precede the variant's, and the accelerometer subscription line must have been logged --
  # without it the node flies with NO force channel and no cancellation, silently
  if [ "$(basename "${ADAPTIVE_NODE_PY:-x}")" = nmpc_cascade_imu_node.py ]; then
    IBANNER=$(echo "$APANE" | grep -o "\[cascade+imu\] force residual.*" | tail -1 | cut -c1-200)
    if [ -z "$IBANNER" ]; then
      echo "[slosh_hil] !! ADAPTIVE_NODE_PY names the IMU node but its [cascade+imu] banner never appeared. Not READY."; exit 1
    fi
    if ! echo "$APANE" | grep -q "\[cascade+imu\] accelerometer: /"; then
      echo "[slosh_hil] !! the IMU node never attached its sensor_combined subscription. Not READY."; exit 1
    fi
    say "    $IBANNER"
  fi
  say "    adaptive cascade: estimator=$AEST, M $AM kg, CG_X $ACG, standby as cascade_adaptive (GS: Adaptive NMPC + Geo)"
fi

# THE STATE SOURCE (2026-09-15): CASC_STATE=px4 flies the cascades on EKF2's odometry; EKF_SOURCE=gps
# leaves EKF2 on GPS position/velocity (the airframe defaults) instead of the mocap bridge's full EV;
# VTOL_GPS_HZ sets the sim GPS's rate. Each is verified on the live stack, not assumed from the env:
if [ -n "${VTOL_GPS_HZ:-}" ]; then
  GHZ=$(timeout 12 ros2 topic hz /uav_0/fmu/out/vehicle_gps_position -w 40 2>/dev/null | grep -o "average rate: [0-9.]*" | tail -1 | grep -o "[0-9.]*")
  if [ -z "$GHZ" ] || ! awk -v a="$GHZ" -v b="$VTOL_GPS_HZ" 'BEGIN{exit (a > 0.6*b && a < 1.6*b) ? 0 : 1}'; then
    echo "[slosh_hil] !! VTOL_GPS_HZ=$VTOL_GPS_HZ asked for but PX4's vehicle_gps_position runs at ${GHZ:-<none>} Hz. Not READY."; exit 1
  fi
  say "    sim GPS: $GHZ Hz on PX4's vehicle_gps_position (asked $VTOL_GPS_HZ)"
fi
if [ "${CASC_STATE:-mocap}" = px4 ]; then
  OHZ=$(timeout 12 ros2 topic hz /uav_0/fmu/out/vehicle_odometry -w 40 2>/dev/null | grep -o "average rate: [0-9.]*" | tail -1 | grep -o "[0-9.]*")
  if [ -z "$OHZ" ] || ! awk -v a="$OHZ" 'BEGIN{exit (a > 30) ? 0 : 1}'; then
    echo "[slosh_hil] !! CASC_STATE=px4 but EKF2's vehicle_odometry runs at ${OHZ:-<none>} Hz. Not READY."; exit 1
  fi
  for W in 6h-cascade 6i-adaptive; do
    if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx "$W"; then
      if ! tmux capture-pane -p -J -S -400 -t "$SESSION:$W" 2>/dev/null | grep -q "state source: px4"; then
        echo "[slosh_hil] !! CASC_STATE=px4 but the $W node did not report 'state source: px4'. Not READY."; exit 1
      fi
    fi
  done
  if [ "${GEO_STATE:-mocap}" = px4 ]; then
    if ! tmux capture-pane -p -J -S - -t "$SESSION:6-geometric" 2>/dev/null | grep -q "state source: px4"; then   # whole scrollback: the pane logs at 2 Hz
      echo "[slosh_hil] !! GEO_STATE=px4 but the safe controller did not report 'state source: px4'. Not READY."; exit 1
    fi
    say "    safe controller: state source px4"
  fi
  if [ "${EKF_SOURCE:-ev}" = gps ] || [ "${EKF_SOURCE:-ev}" = gpsmag ]; then
    GPHZ=$(timeout 12 ros2 topic hz /uav_0/fmu/out/vehicle_global_position -w 40 2>/dev/null | grep -o "average rate: [0-9.]*" | tail -1 | grep -o "[0-9.]*")
    if [ -z "$GPHZ" ]; then
      echo "[slosh_hil] !! EKF_SOURCE=gps but EKF2 publishes no vehicle_global_position (no GPS aiding). Not READY."; exit 1
    fi
    say "    EKF2 on GPS: vehicle_global_position at $GPHZ Hz"
  fi
  say "    state source: px4 (EKF2 vehicle_odometry at $OHZ Hz, EKF_SOURCE=${EKF_SOURCE:-ev})"
fi
# HW=1 (piece 5, 2026-09-15): the hardware preflight. PX4's failsafes are read back over MAVLink
# against scripts/preflight_hw.cfg; one refused line = not READY. (In HIL, HW=1 is expected to
# refuse: the sim battery parameter alone is a refusal -- that is the test that the check works.)
if [ "${HW:-0}" = 1 ]; then
  if ! /usr/bin/python3 "$ROOT/scripts/preflight_hw.py" --url "${PREFLIGHT_URL:-udpin:0.0.0.0:14550}" --timeout 20; then
    echo "[slosh_hil] !! HW=1: the hardware preflight REFUSED (above). Not READY."; exit 1
  fi
  say "    hardware preflight: every failsafe armed (preflight_hw.cfg)"
fi
# CASC_ASPD=px4 (piece 4, 2026-09-15): the cascade's airspeed source is PX4's airspeed_validated (the
# sim's synthesised pitot, air-relative since VTOL_WIND). The topic must be alive and each cascade pane
# must have reported 'airspeed source: px4'; VTOL_WIND / VTOL_TURB must have reached the Isaac pane.
if [ "${CASC_ASPD:-none}" = px4 ]; then
  AHZ=$(timeout 12 ros2 topic hz /uav_0/fmu/out/airspeed_validated -w 40 2>/dev/null | grep -o "average rate: [0-9.]*" | tail -1 | grep -o "[0-9.]*")
  if [ -z "$AHZ" ] || ! awk -v a="$AHZ" 'BEGIN{exit (a > 5) ? 0 : 1}'; then
    echo "[slosh_hil] !! CASC_ASPD=px4 but PX4's airspeed_validated runs at ${AHZ:-<none>} Hz (SYS_HAS_NUM_ASPD?). Not READY."; exit 1
  fi
  for W in 6h-cascade 6i-adaptive; do
    if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx "$W"; then
      if ! tmux capture-pane -p -J -S -400 -t "$SESSION:$W" 2>/dev/null | grep -q "airspeed source: px4"; then
        echo "[slosh_hil] !! CASC_ASPD=px4 but the $W node did not report 'airspeed source: px4'. Not READY."; exit 1
      fi
    fi
  done
  say "    airspeed source: px4 (airspeed_validated at $AHZ Hz)"
fi
if [ -n "${VTOL_WIND:-}${VTOL_TURB:-}" ]; then
  if ! tmux capture-pane -p -J -S - -t "$SESSION:2-isaac" 2>/dev/null | grep -q "\[VTOL\] wind:"; then
    echo "[slosh_hil] !! VTOL_WIND=${VTOL_WIND:-} VTOL_TURB=${VTOL_TURB:-} asked for but Isaac never logged '[VTOL] wind:' (the env did not reach the pane, or the vehicle is not stepping). Not READY."; exit 1
  fi
  say "    wind: $(tmux capture-pane -p -J -S - -t "$SESSION:2-isaac" 2>/dev/null | grep -o "\[VTOL\] wind:.*" | tail -1 | cut -c1-120)"
fi

# THE BAG MUST EXIST (2026-09-15): the cold-start launcher runs under `set -u` and an unbound variable
# aborts it half-way -- Isaac, PX4 and the controllers were up (started before the abort), every
# check above passed, the flight landed, and there was NO bag: the 10-bag window is started near the
# end of that script. ros2 bag creates its directory at start; the results row falls back to the
# NEWEST bag, i.e. the previous flight's. Refuse READY without it, and require the recorder window.
if ! tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx 10-bag; then
  echo "[slosh_hil] !! no 10-bag recorder window (the launcher aborted before starting it?). Not READY."; exit 1
fi
if [ ! -d "$BAG_DIR/$BAG" ]; then
  echo "[slosh_hil] !! the bag directory $BAG_DIR/$BAG does not exist -- the recorder is not recording. Not READY."; exit 1
fi
say "    recording: $BAG_DIR/$BAG"

# provenance, next to the bags, because the tmux pane is gone by the time anyone asks
printf '%s  model=%-10s branch=%-18s commit=%-9s dirty=%-3s bag=%s\n' \
  "$(date '+%Y-%m-%d %H:%M:%S')" \
  "$([ "$WANT_SLOSH" = 1 ] && echo VTOLSlosh || echo VTOL)" \
  "$BRANCH" "$COMMIT" "$DIRTY" "$BAG_DIR/$BAG  cascade=$(basename "${CASC_NODE_PY:-nmpc_cascade_node.py}")  water=${VTOL_TANK_KG}kg tank=${VTOL_TANK} tank_R=${VTOL_TANK_R:-0.0414} tank_dx=${VTOL_TANK_DX} l1=${CASC_L1} state=${CASC_STATE:-mocap}${GEO_STATE:+ geo_state=$GEO_STATE} af=$AF_DIGEST ekf=${EKF_SOURCE:-ev}${VTOL_GPS_HZ:+ gps_hz=$VTOL_GPS_HZ}${VTOL_GPS_SIGMA:+ gps_sigma=$VTOL_GPS_SIGMA} aspd=${CASC_ASPD:-none}${HW:+ hw=$HW}${VTOL_WIND:+ wind=$VTOL_WIND}${VTOL_TURB:+ turb=$VTOL_TURB}${ADAPTIVE_NODE_PY:+ anode=$(basename "$ADAPTIVE_NODE_PY")}${ABANNER:+  adaptive: $ABANNER}${IBANNER:+  $IBANNER}" >> "$LAUNCH_LOG"

say "READY -- $([ "$WANT_SLOSH" = 1 ] && echo "TANK MODEL (VTOLSlosh, tank $([ -n "$TANK_LINE" ] && echo MOUNTED || echo none))" || echo 'baseline VTOL') confirmed in Isaac."
say "        bag $BAG_DIR/$BAG   provenance appended to $LAUNCH_LOG"
say "        tear down with: bash $ROOT/scripts/kill_slosh_hil.sh"
