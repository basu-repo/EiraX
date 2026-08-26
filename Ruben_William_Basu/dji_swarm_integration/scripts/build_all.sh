#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon --log-base log_dji_lrs build --symlink-install \
  --base-paths dji_stack --build-base build_dji_lrs \
  --install-base install_dji_lrs --packages-select lrs_halmstad

set +u
source "$ROOT/install_dji_lrs/setup.bash"
set -u
colcon --log-base log_dji build --symlink-install \
  --base-paths . --build-base build_dji \
  --install-base install_dji --packages-select dji_swarm_integration

echo "[BUILT] DJI perception and decentralized integration packages are ready."
