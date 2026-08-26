#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

PC2LS_NODE="/a201_0000/pointcloud_to_laserscan"
SWEEP=""
MIN_START="-0.7"
MAX_END="10.0"
STEP="0.05"
DWELL_S="2.0"
MIN_HEIGHT=""
MAX_HEIGHT=""
DRY_RUN="false"

usage() {
  cat <<'EOF'
Usage:
  ./run.sh pc2ls_sweep sweep:=min [node:=/a201_0000/pointcloud_to_laserscan] [min_start:=-0.7] [max:=current] [step:=0.05] [dwell_s:=2.0]
  ./run.sh pc2ls_sweep sweep:=max [node:=/a201_0000/pointcloud_to_laserscan] [min:=current] [max_end:=10.0] [step:=0.05] [dwell_s:=2.0]

Behavior:
  sweep:=min  keeps max_height fixed and sweeps min_height from min_start up to max_height.
  sweep:=max  keeps min_height fixed and sweeps max_height from min_height up to max_end.

Examples:
  ./run.sh pc2ls_sweep sweep:=min max:=0.4 step:=0.05 dwell_s:=3
  ./run.sh pc2ls_sweep sweep:=max min:=-0.4 max_end:=10.0 step:=0.1 dwell_s:=2
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

  output="$(ros2 service call "${PC2LS_NODE}/get_parameters" rcl_interfaces/srv/GetParameters "{names: ['$param_name']}")"
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

float_values() {
  local start="$1"
  local end="$2"
  local step="$3"

  python3 - "$start" "$end" "$step" <<'PY'
import math
import sys

start = float(sys.argv[1])
end = float(sys.argv[2])
step = abs(float(sys.argv[3]))
if step <= 0.0:
    raise SystemExit("step must be > 0")

value = start
epsilon = step * 1e-6
while value <= end + epsilon:
    print(f"{min(value, end):.6g}")
    value += step
PY
}

set_param() {
  local name="$1"
  local value="$2"
  local service="${PC2LS_NODE}/set_parameters"
  local request

  if [ "$DRY_RUN" = "true" ]; then
    echo "[dry-run] ros2 service call $service rcl_interfaces/srv/SetParameters ..."
    return 0
  fi

  request="{parameters: [{name: '$name', value: {type: 3, double_value: $value}}]}"
  ros2 service call "$service" rcl_interfaces/srv/SetParameters "$request" >/dev/null
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    help|-h|--help)
      usage
      exit 0
      ;;
    sweep:=min|sweep:=min_height)
      SWEEP="min"
      ;;
    sweep:=max|sweep:=max_height)
      SWEEP="max"
      ;;
    node:=*|pc2ls_node:=*)
      PC2LS_NODE="${1#*:=}"
      ;;
    min_start:=*)
      MIN_START="${1#min_start:=}"
      ;;
    max_end:=*)
      MAX_END="${1#max_end:=}"
      ;;
    min:=*|min_height:=*|pc2ls_min_height:=*)
      MIN_HEIGHT="${1#*:=}"
      ;;
    max:=*|max_height:=*|pc2ls_max_height:=*)
      MAX_HEIGHT="${1#*:=}"
      ;;
    step:=*)
      STEP="${1#step:=}"
      ;;
    dwell_s:=*|sleep_s:=*)
      DWELL_S="${1#*:=}"
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

if [ -z "$SWEEP" ]; then
  echo "Missing sweep:=min or sweep:=max" >&2
  usage >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

if [ "$DRY_RUN" != "true" ]; then
  service="${PC2LS_NODE}/set_parameters"
  services="$(timeout 5s ros2 service list --no-daemon || true)"
  if ! grep -Fxq "$service" <<<"$services"; then
    echo "pc2ls set_parameters service not found: $service" >&2
    if grep -Fq "pointcloud_to_laserscan" <<<"$services"; then
      echo "Available pointcloud_to_laserscan services:" >&2
      grep -F "pointcloud_to_laserscan" <<<"$services" >&2
    else
      echo "No pointcloud_to_laserscan services found. Start localization with lidar:=3d first." >&2
    fi
    exit 1
  fi
fi

case "$SWEEP" in
  min)
    if [ -z "$MAX_HEIGHT" ]; then
      MAX_HEIGHT="$(param_value max_height)"
    fi
    echo "[pc2ls_sweep] sweeping min_height from $MIN_START to $MAX_HEIGHT, fixed max_height=$MAX_HEIGHT"
    for value in $(float_values "$MIN_START" "$MAX_HEIGHT" "$STEP"); do
      echo "[pc2ls_sweep] min_height=$value max_height=$MAX_HEIGHT"
      set_param max_height "$MAX_HEIGHT"
      set_param min_height "$value"
      sleep "$DWELL_S"
    done
    ;;
  max)
    if [ -z "$MIN_HEIGHT" ]; then
      MIN_HEIGHT="$(param_value min_height)"
    fi
    echo "[pc2ls_sweep] sweeping max_height from $MIN_HEIGHT to $MAX_END, fixed min_height=$MIN_HEIGHT"
    for value in $(float_values "$MIN_HEIGHT" "$MAX_END" "$STEP"); do
      echo "[pc2ls_sweep] min_height=$MIN_HEIGHT max_height=$value"
      set_param min_height "$MIN_HEIGHT"
      set_param max_height "$value"
      sleep "$DWELL_S"
    done
    ;;
esac

echo "[pc2ls_sweep] done"
