# terminal_utils.sh
# shellcheck shell=bash

[[ -n "${_TERMINAL_UTILS_SH_LOADED:-}" ]] && return 0
_TERMINAL_UTILS_SH_LOADED=1

open_new_terminal() {
  local cmd
  printf -v cmd '%q ' "$@"

  # Launch the GUI terminal with a PRISTINE environment, mimicking how the desktop
  # shell (GNOME) starts terminals. A sourced ROS/Isaac/venv environment (this
  # repo's users source ROS + a venv like isaac_env from ~/.bashrc) injects vars
  # that make gnome-terminal load mismatched snap libs and crash on startup
  # ("undefined symbol: __libc_pthread_init"). We keep only what a GUI terminal
  # needs; the inner `bash -lc` re-sources the user's profile, so the actual work
  # command still runs with the full environment.
  local uid; uid="$(id -u)"
  local -a launch=(
    env -i
    HOME="$HOME"
    USER="${USER:-$(id -un)}"
    LOGNAME="${LOGNAME:-${USER:-$(id -un)}}"
    SHELL="${SHELL:-/bin/bash}"
    LANG="${LANG:-C.UTF-8}"
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    DISPLAY="${DISPLAY:-:0}"
    XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$uid}"
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-unix:path=/run/user/$uid/bus}"
  )
  [ -n "${WAYLAND_DISPLAY:-}" ] && launch+=(WAYLAND_DISPLAY="$WAYLAND_DISPLAY")

  if command -v konsole >/dev/null 2>&1; then
    "${launch[@]}" konsole --noclose -e bash -lc "$cmd" &
    return 0
  fi

  if command -v gnome-terminal >/dev/null 2>&1; then
    "${launch[@]}" gnome-terminal -- bash -lc "$cmd" &
    return 0
  fi

  if command -v x-terminal-emulator >/dev/null 2>&1; then
    "${launch[@]}" x-terminal-emulator -e bash -lc "$cmd" &
    return 0
  fi

  if command -v xterm >/dev/null 2>&1; then
    "${launch[@]}" xterm -e bash -lc "$cmd" &
    return 0
  fi

  echo "ERROR: No supported terminal found (konsole/gnome-terminal/x-terminal-emulator/xterm)." >&2
  return 1
}