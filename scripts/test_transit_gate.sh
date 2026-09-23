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
# Each group's pytest output is captured and printed, and its reported "passed" count
# is checked against a floor (the second argument to run_group_with_floor below). A
# suite that starts silently skipping en masse (a collection error, a fixture that
# makes every test self-skip, a bad path that collects nothing) fails the gate loudly
# instead of exiting 0 with fewer tests run. Raise a floor whenever a task deliberately
# adds tests; lower one only with a comment explaining why fewer tests is now correct.
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

# Runs pytest, prints its output, and fails the gate if the reported "passed" count
# (summed with any explicitly-allowed skips) is below the given floor -- catching a
# suite that starts collecting/running fewer tests than expected without needing an
# exact, brittle count.
run_group_with_floor() {
  local name="$1" min_passed="$2"
  shift 2
  echo "[$name]"
  local out
  out="$("$@" 2>&1)"
  local status=$?
  echo "$out"
  if [ "$status" -ne 0 ]; then
    FAILED=1
  fi
  # pytest's summary line ends with e.g. "83 passed in 1.23s" or
  # "309 passed, 118 skipped in 4.56s". Grab the passed count; treat a missing
  # summary line (e.g. a hard collection crash) as zero.
  local passed
  passed="$(echo "$out" | grep -Eo '[0-9]+ passed' | tail -1 | grep -Eo '[0-9]+')"
  passed="${passed:-0}"
  echo "[$name] passed=$passed (floor $min_passed)"
  if [ "$passed" -lt "$min_passed" ]; then
    echo "[$name] FAIL: only $passed passed, expected at least $min_passed -- a suite likely started skipping, erroring, or failing collection"
    FAILED=1
  fi
}

cd "$ROOT"

# 1. Ford car interface: opendbc/car/ford, including TransitLkaState and the
# Transit-specific CAN construction.
run_group_with_floor ford-car 83 "$PY" -m pytest opendbc_repo/opendbc/car/ford/tests/test_ford.py -q -o addopts=""

# 2. Ford panda safety: opendbc/safety/modes/ford.h. Rebuild libsafety.so first --
# a stale .so silently tests the previous version of ford.h.
run_group ford-safety-build bash -c "cd '$ROOT/opendbc_repo' && '$PY' -m SCons -j8 opendbc/safety/tests/libsafety/libsafety.so"
run_group_with_floor ford-safety 309 bash -c "cd '$ROOT/opendbc_repo' && '$PY' -m pytest opendbc/safety/tests/test_ford.py -q -n 4"

# 3. StarPilot/openpilot regression suites this branch's tasks added or touched
# (Transit params, lane centering, device settings layout, athenad/uploads being
# disabled, driver monitoring, manager, toggles, the always-run process). Runs under
# the project's own pytest defaults (pyproject.toml), including -Werror via the
# "error" filterwarnings entry -- no -o addopts="" override. That override used to be
# required only because starpilot/common/tests/test_starpilot_process.py failed
# *collection* under -Werror (a DeprecationWarning raised at import time by the
# vendored, unmodified starpilot/third_party/reactivex package); pyproject.toml now
# carries a filterwarnings entry scoped to that exact module instead.
STARPILOT_REGRESSION_SUITES=(
  starpilot/common/tests/test_transit_params.py
  selfdrive/controls/tests/test_lane_centering.py
  starpilot/common/tests/test_lane_centering_galaxy.py
  starpilot/common/tests/test_safe_mode_lane_centering.py
  starpilot/system/the_galaxy/tests/test_device_settings_layout.py
  system/manager/test/test_manager.py
  selfdrive/ui/tests/test_athena_disabled.py
  system/loggerd/tests/test_uploader.py
  selfdrive/modeld/tests/test_dmonitoringmodeld.py
  selfdrive/ui/layouts/settings/tests/test_toggles.py
  starpilot/common/tests/test_starpilot_process.py
)
# system/manager/test/test_manager.py::TestManager::{test_manager_prepare,
# test_set_params_with_default_value} spawn a background thread that shells out to
# ./bootlog (system/manager/helpers.py). ./bootlog is a Linux binary; on this macOS
# dev machine that subprocess.call raises OSError: [Errno 8] Exec format error
# *inside that thread*, which pytest reports as a PytestUnhandledThreadExceptionWarning
# rather than a test failure -- so under the old -o addopts="" override (no -Werror)
# both tests silently pass with a warning. Under the project's own -Werror that warning
# is promoted to an error and both tests fail, independent of anything this branch
# touches: it is a macOS-vs-Linux-binary environment limitation, not a regression, and
# was already flagged as an expected -Werror finding in
# BearTransit/main/.superpowers/sdd/final-review-fixes-report.md's own "-Werror run"
# section. Deselect exactly these two so the gate reports real regressions instead of
# this known environmental one; a device/CI run on real Linux hardware would not need
# this exclusion, so remove it if this script is ever run there.
BOOTLOG_EXEC_FORMAT_DESELECTS=(
  --deselect "system/manager/test/test_manager.py::TestManager::test_manager_prepare"
  --deselect "system/manager/test/test_manager.py::TestManager::test_set_params_with_default_value"
)
# 146 passed, 1 skipped: the 148 passed, 1 skipped documented baseline
# (final-review-fixes-report.md) minus the 2 tests deselected above.
run_group_with_floor starpilot-regression 146 "$PY" -m pytest "${STARPILOT_REGRESSION_SUITES[@]}" "${BOOTLOG_EXEC_FORMAT_DESELECTS[@]}" -q

exit "$FAILED"
