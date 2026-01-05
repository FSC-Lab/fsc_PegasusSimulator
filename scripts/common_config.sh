#!/usr/bin/env bash
# Common config loader (source this file from launch scripts)

set -euo pipefail

cfg_usage() {
  local prog="${1:-$0}"
  echo "Usage:"
  echo "  ${prog} <config_name>"
  echo
  echo "Looks for configs in: <repo>/scripts/config/"
  echo "Example:"
  echo "  ${prog} longhao_machine"
  echo "  ${prog} longhao_machine.conf"
}

# Resolve repo root based on this file location:
#   <repo>/scripts/common_config.sh -> <repo>
_get_repo_root() {
  local this_dir
  this_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  cd -- "$this_dir/.." && pwd
}

resolve_config_path() {
  # $1: config name (e.g. longhao_machine or longhao_machine.conf) OR a path
  local name="${1:-}"
  local repo_root config_dir candidate

  if [[ -z "$name" ]]; then
    echo ""
    return 1
  fi

  repo_root="$(_get_repo_root)"
  config_dir="$repo_root/scripts/config"

  # If user passed an existing path, accept it (optional behavior).
  if [[ -f "$name" ]]; then
    echo "$name"
    return 0
  fi

  # If user passed just a name, search in scripts/config
  if [[ "$name" != *.conf ]]; then
    candidate="$config_dir/${name}.conf"
  else
    candidate="$config_dir/$name"
  fi

  if [[ -f "$candidate" ]]; then
    echo "$candidate"
    return 0
  fi

  # Not found
  echo ""
  return 1
}

load_machine_config() {
  local prog="${1:-$0}"
  local cfg_name="${2:-}"

  if [[ -z "$cfg_name" ]]; then
    echo "ERROR: config name is required."
    cfg_usage "$prog"
    return 2
  fi

  local cfg_file=""
  if ! cfg_file="$(resolve_config_path "$cfg_name")"; then
    echo "ERROR: could not resolve config: $cfg_name"
    echo "Looked in: $(_get_repo_root)/scripts/config/"
    cfg_usage "$prog"
    return 2
  fi

  echo "[Config] Using: $cfg_file"

  # shellcheck source=/dev/null
  source "$cfg_file"

  # Required variables (only the 3 you want)
  : "${PX4_DIR:?PX4_DIR must be set in config}"
  : "${ISAAC_PY:?ISAAC_PY must be set in config}"
  : "${FSC_PEGASUS_ROOT:?FSC_PEGASUS_ROOT must be set in config}"

  [[ -d "$PX4_DIR" ]] || { echo "ERROR: PX4_DIR not found: $PX4_DIR" >&2; return 1; }
  [[ -f "$ISAAC_PY" ]] || { echo "ERROR: ISAAC_PY not found: $ISAAC_PY" >&2; return 1; }
  [[ -d "$FSC_PEGASUS_ROOT" ]] || { echo "ERROR: FSC_PEGASUS_ROOT not found: $FSC_PEGASUS_ROOT" >&2; return 1; }

  return 0
}
