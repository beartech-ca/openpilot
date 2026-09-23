#!/usr/bin/env bash

# Regression gate for the beartransit-star branch (2022 Ford Transit MK5 LKA_STEERING
# port). Enumerates the three suites every fix on this branch is checked against:
# the Ford car interface, the Ford panda safety implementation, and the StarPilot /
# openpilot regression suites the port touches.
#
# This list used to live only in the session ledger
# (BearTransit/main/.superpowers/sdd/progress.md) and could not survive a session on
# its own; this script is the part of it that can. Update it whenever a task adds,
# renames or moves a suite the branch depends on -- in particular,
# starpilot/common/tests/test_starpilot_process.py lives under starpilot/common/tests/,
# not starpilot/tests/.
#
# Usage: scripts/test_transit_gate.sh
# Assumes .venv is already synced (uv sync); this script does not run uv sync itself,
# matching scripts/test_all.sh's convention.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." >/dev/null && pwd)"
PY="$ROOT/.venv/bin/python"
FAILED=0

run_group() {
  local name="$1"
  shift
  echo "[$name]"
  if ! "$@"; then
    FAILED=1
  fi
}

cd "$ROOT"

# 1. Ford car interface: opendbc/car/ford, including TransitLkaState and the
# Transit-specific CAN construction.
run_group ford-car "$PY" -m pytest opendbc_repo/opendbc/car/ford/tests/test_ford.py -q -o addopts=""

# 2. Ford panda safety: opendbc/safety/modes/ford.h. Rebuild libsafety.so first --
# a stale .so silently tests the previous version of ford.h.
run_group ford-safety-build bash -c "cd '$ROOT/opendbc_repo' && '$PY' -m SCons -j8 opendbc/safety/tests/libsafety/libsafety.so"
run_group ford-safety bash -c "cd '$ROOT/opendbc_repo' && '$PY' -m pytest opendbc/safety/tests/test_ford.py -q -n 4"

# 3. StarPilot/openpilot regression suites this branch's tasks added or touched
# (Transit params, lane centering, device settings layout, athenad/uploads being
# disabled, driver monitoring, manager, toggles, the always-run process).
run_group starpilot-regression "$PY" -m pytest \
  starpilot/common/tests/test_transit_params.py \
  selfdrive/controls/tests/test_lane_centering.py \
  starpilot/common/tests/test_lane_centering_galaxy.py \
  starpilot/common/tests/test_safe_mode_lane_centering.py \
  starpilot/system/the_galaxy/tests/test_device_settings_layout.py \
  system/manager/test/test_manager.py \
  selfdrive/ui/tests/test_athena_disabled.py \
  system/loggerd/tests/test_uploader.py \
  selfdrive/modeld/tests/test_dmonitoringmodeld.py \
  selfdrive/ui/layouts/settings/tests/test_toggles.py \
  starpilot/common/tests/test_starpilot_process.py \
  -q -o addopts=""

exit "$FAILED"
