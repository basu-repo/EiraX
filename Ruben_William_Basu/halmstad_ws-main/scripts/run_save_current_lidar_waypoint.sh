#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

WAYPOINT=""
CONFIG_FILE="$WS_ROOT/src/lrs_halmstad/config/baylands_route_lidar.yaml"
PC2LS_NODE="/a201_0000/pointcloud_to_laserscan"
DRY_RUN="false"

usage() {
  cat <<'EOF'
Usage: ./run.sh save_current_lidar_waypoint waypoint:=road_to_west_8 [node:=/a201_0000/pointcloud_to_laserscan] [file:=...] [dry_run:=true|false]

Reads the current pointcloud_to_laserscan min_height/max_height ROS params and
writes them into baylands_route_lidar.yaml under:

  waypoints:
    <waypoint>:
      pc2ls_min_height: ...
      pc2ls_max_height: ...
EOF
}

coerce_bool() {
  case "$1" in
    true|false)
      printf '%s\n' "$1"
      ;;
    *)
      echo "Invalid boolean value: $1" >&2
      exit 2
      ;;
  esac
}

param_value() {
  local param_name="$1"
  local output=""

  output="$(ros2 param get "$PC2LS_NODE" "$param_name")"
  python3 - "$output" <<'PY'
import re
import sys

text = sys.argv[1]
match = re.search(r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)", text)
if not match:
    raise SystemExit(f"Could not parse numeric parameter value from: {text}")
print(float(match.group(1)))
PY
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    help|-h|--help)
      usage
      exit 0
      ;;
    waypoint:=*)
      WAYPOINT="${1#waypoint:=}"
      ;;
    node:=*|pc2ls_node:=*)
      PC2LS_NODE="${1#*:=}"
      ;;
    file:=*|config:=*)
      CONFIG_FILE="${1#*:=}"
      ;;
    dry_run:=*)
      DRY_RUN="$(coerce_bool "${1#dry_run:=}")"
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [ -z "$WAYPOINT" ]; then
  echo "Missing waypoint:=..." >&2
  usage >&2
  exit 2
fi

if [ ! -f "$CONFIG_FILE" ]; then
  echo "Config file not found: $CONFIG_FILE" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

MIN_HEIGHT="$(param_value min_height)"
MAX_HEIGHT="$(param_value max_height)"

if [ "$DRY_RUN" = "true" ]; then
  cat <<EOF
[dry-run] Would update $CONFIG_FILE:
waypoints:
  $WAYPOINT:
    pc2ls_min_height: $MIN_HEIGHT
    pc2ls_max_height: $MAX_HEIGHT
EOF
  exit 0
fi

python3 - "$CONFIG_FILE" "$WAYPOINT" "$MIN_HEIGHT" "$MAX_HEIGHT" <<'PY'
from pathlib import Path
import sys

import yaml

path = Path(sys.argv[1])
waypoint = sys.argv[2]
min_height = float(sys.argv[3])
max_height = float(sys.argv[4])

data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
waypoints = data.setdefault("waypoints", {})
entry = waypoints.setdefault(waypoint, {})
entry["pc2ls_min_height"] = min_height
entry["pc2ls_max_height"] = max_height

path.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY

echo "Saved live lidar settings for $WAYPOINT:"
echo "  pc2ls_min_height: $MIN_HEIGHT"
echo "  pc2ls_max_height: $MAX_HEIGHT"
echo "  file: $CONFIG_FILE"
