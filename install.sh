#!/usr/bin/env bash
# ford-lka install script
# Handles switching from stock prebuilt branches to our source branch cleanly.
#
# Usage (on comma device):
#   cd /data/openpilot
#   git remote set-url origin https://github.com/ghostdev137/openpilot.git
#   git fetch origin ford-lka:ford-lka --force
#   git checkout -f ford-lka
#   bash install.sh

set -e

cd "$(dirname "$0")"

echo "=== ford-lka install ==="

# 1. Verify we're on the right branch
branch=$(git rev-parse --abbrev-ref HEAD)
if [ "$branch" != "ford-lka" ]; then
  echo "ERROR: not on ford-lka branch (on $branch)"
  exit 1
fi

# 2. Clean orphan submodule directories left over from prebuilt branches
#    The prebuilt/release branches ship submodule contents as flat directories.
#    When we switch to a source branch, git refuses to clone into non-empty dirs.
echo "[1/4] Cleaning orphan submodule dirs..."
for d in msgq_repo tinygrad_repo teleoprtc_repo rednose_repo; do
  # Only remove if NOT a proper submodule (has .git file/dir) and is populated
  if [ -d "$d" ] && [ ! -e "$d/.git" ]; then
    echo "  removing orphan $d/"
    rm -rf "$d"
  fi
done

# 3. Sync submodule URLs in case .gitmodules changed
echo "[2/4] Syncing submodule URLs..."
git submodule sync 2>&1 | tail -3

# 4. Init all submodules
echo "[3/4] Initializing submodules..."
if ! git submodule update --init --recursive --force 2>&1 | tail -10; then
  echo "ERROR: submodule update failed"
  exit 1
fi

# 5. Verify critical files are present
echo "[4/4] Verifying..."
for f in opendbc_repo/opendbc/car/ford/values.py panda/board/main.c tinygrad_repo/tinygrad/__init__.py; do
  if [ ! -f "$f" ]; then
    echo "ERROR: missing $f"
    exit 1
  fi
done

# 6. Set update target and clear cached carparams so fingerprint re-runs
echo -n "ford-lka" > /data/params/d/UpdaterTargetBranch
rm -f /data/params/d/CarParamsCache /data/params/d/CarParamsPersistent

echo ""
echo "=== install OK ==="
echo "All submodules present. Safe to reboot."
echo "Run: sudo reboot"
