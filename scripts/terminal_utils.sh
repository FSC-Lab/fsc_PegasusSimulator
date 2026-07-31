# terminal_utils.sh
# shellcheck shell=bash

[[ -n "${_TERMINAL_UTILS_SH_LOADED:-}" ]] && return 0
_TERMINAL_UTILS_SH_LOADED=1

open_new_terminal() {
  local executable="${1:?open_new_terminal requires a command}"
  shift

  # Launchers commonly pass "$0". Resolve it before changing terminal/CWD so
  # scripts in indoor_sim/ and outdoor_sim/ remain callable from any directory.
  if [[ "$executable" == */* && -e "$executable" ]]; then
    executable="$(cd -- "$(dirname -- "$executable")" && pwd)/$(basename -- "$executable")"
  fi

  local cmd
  printf -v cmd '%q ' "$executable" "$@"

  if command -v konsole >/dev/null 2>&1; then
    konsole --noclose -e bash -lc "$cmd" &
    return 0
  fi

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal -- bash -lc "$cmd" &
    return 0
  fi

  if command -v x-terminal-emulator >/dev/null 2>&1; then
    x-terminal-emulator -e bash -lc "$cmd" &
    return 0
  fi

  if command -v xterm >/dev/null 2>&1; then
    xterm -e bash -lc "$cmd" &
    return 0
  fi

  echo "ERROR: No supported terminal found (konsole/gnome-terminal/x-terminal-emulator/xterm)." >&2
  return 1
}
