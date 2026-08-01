#!/usr/bin/env bash

# Source ROS/colcon-generated environment files without leaking nounset state.
safe_source() {
  local setup_file="${1:?safe_source requires a setup file}"
  if [[ ! -r "${setup_file}" ]]; then
    printf 'ERROR: setup file not readable: %s\n' "${setup_file}" >&2
    return 1
  fi

  local restore_nounset=0
  local source_rc=0
  case $- in
    *u*)
      restore_nounset=1
      set +u
      ;;
  esac

  # The conditional prevents errexit from skipping nounset restoration.
  # shellcheck disable=SC1090
  if source "${setup_file}"; then
    source_rc=0
  else
    source_rc=$?
  fi

  if ((restore_nounset)); then
    set -u
  fi
  return "${source_rc}"
}
