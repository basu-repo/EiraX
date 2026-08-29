#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="${SCRIPT_DIR}/baylands_editable.world"
MODEL_DIR="$(cd -- "${SCRIPT_DIR}/../.." && pwd)/models"

if ! command -v gz >/dev/null 2>&1; then
  echo "Error: 'gz' was not found. Source ROS 2 Jazzy first:" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "Error: world file not found: ${WORLD_FILE}" >&2
  exit 1
fi

if gz service -l 2>/dev/null | grep -qx '/server_control'; then
  echo "Error: another Gazebo server is already running." >&2
  echo "Close the existing Gazebo window before opening the editor again." >&2
  exit 1
fi

export GZ_SIM_RESOURCE_PATH="${MODEL_DIR}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"

echo "Opening persistent Baylands world: ${WORLD_FILE}"
echo "After editing, save from another terminal with:"
echo "  ${SCRIPT_DIR}/save_baylands_world.sh"
exec gz sim "${WORLD_FILE}"
