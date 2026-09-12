#!/usr/bin/env bash
# Rebuild the campaign table by RE-SCORING every bag with the current metrics, rather than
# trusting rows written earlier in the night. The scoring rules changed several times as
# instrumentation bugs surfaced (log-string parsing that missed a whole flight, a landing error
# reported next to a crash, a verdict order that called a 139 deg roll a landing), so rows written
# under older rules are not comparable with each other. The bags are the durable record; re-derive
# everything from them.
set -u
cd "$(dirname "$0")/.."
OUT=${OUT:-/tmp/night_campaign}
set +u; source scripts/rosenv.sh 2>/dev/null; set -u

# recover each trial's parameters from the results file (bag names carry a timestamp suffix)
# NB: `IFS=$'\t' read` collapses CONSECUTIVE tabs, because tab is an IFS whitespace character.
# A trial with no parameters has an empty second field, so read skipped it and shifted the
# VERDICT into the params column -- the table then claimed baseline's parameter was "CRASHED".
# cut keeps empty fields.
declare -A P
while IFS= read -r line; do
  trial=$(printf '%s' "$line" | cut -f1)
  [ "$trial" = "trial" ] && continue
  params=$(printf '%s' "$line" | cut -f2)
  P["$trial"]="${params:--}"
done < "$OUT/results.tsv"

printf 'trial\tparams\tverdict\twb_s\tpeakV\ttilt\troll\tyaw\tzmin\tzmax\tfails\tland_err\n'
for b in "$OUT"/bags/*/; do
  raw=$(basename "$b")
  name=$(echo "$raw" | sed -E 's/_[0-9]{6}$//')       # strip the _HHMMSS suffix
  # ★ REFUSE A LOG THAT HOLDS MORE THAN ONE FLIGHT.
  # Two campaign instances briefly wrote the same path, leaving bt_flare20.flight as a binary
  # file with NUL bytes and TWO summaries -- one landing 0.00 m from the target and one 707.12 m
  # away. Whichever you read is a coin toss, so read neither.
  flt="$OUT/$name.flight"
  nsum=$(tr -d '\0' < "$flt" 2>/dev/null | grep -ac '===== SUMMARY =====' || echo 0)
  if [ "${nsum:-0}" -gt 1 ]; then
    printf '%s\t%s\tCORRUPT_LOG\t\t\t\t\t\t\t\t\t\n' "$name" "${P[$name]:--}"
    continue
  fi
  lerr=$(tr -d '\0' < "$flt" 2>/dev/null | grep -aoE "LANDING ERROR +[0-9.]+" | tail -1 | tr -dc '0-9.')
  /usr/bin/python3 scripts/night_metrics.py "$b" "$name" "${P[$name]:--}" "${lerr:-}" 2>/dev/null \
    | cut -f1-12
done | sort -k3,3 -k4,4nr
