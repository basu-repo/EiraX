#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="${SCRIPT_DIR}/baylands_editable.world"

if ! command -v gz >/dev/null 2>&1; then
  echo "Error: 'gz' was not found. Source ROS 2 Jazzy first:" >&2
  echo "  source /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi

WORLD_NAME="baylands_editable"
SAVE_SERVICE="/world/${WORLD_NAME}/generate_world_sdf"

if ! gz service -l 2>/dev/null | grep -qx "${SAVE_SERVICE}"; then
  echo "Error: the Baylands Gazebo session is not running." >&2
  echo "Start it first with: ${SCRIPT_DIR}/open_baylands_editor.sh" >&2
  exit 1
fi

echo "Saving current models and poses to: ${WORLD_FILE}"
python3 "${SCRIPT_DIR}/save_running_world.py" \
  --service "${SAVE_SERVICE}" \
  --output "${WORLD_FILE}"

echo "World saved and verified. The next launch will open this state."
