#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

UAV_NODE="/follow_uav"
WAIT_TIMEOUT_S="${UAV_POSITION_WAIT_TIMEOUT_S:-3}"
COMMAND="${1:-}"
shift || true

DISTANCE=""
ANGLE_DELTA=""
ANGLE_ABS=""

usage() {
  cat <<EOF
Usage:
  ./run.sh uav_position <angle|distance> <value>
  ./run.sh uav_position angle:=<delta_deg> distance:=<meters>
  ./run.sh uav_position angle_abs:=<deg> distance:=<meters>

Examples:
  ./run.sh uav_position angle 45
  ./run.sh uav_position angle -45
  ./run.sh uav_position distance 12
  ./run.sh uav_position angle:=45 distance:=12
  ./run.sh uav_position angle_abs:=0 distance:=8 wait_s:=1

Notes:
  - angle is relative: current leader_heading_offset_deg + value.
  - angle_abs is absolute: leader_heading_offset_deg = value.
  - distance is absolute: d_target = value.
  - waits up to ${WAIT_TIMEOUT_S}s for ${UAV_NODE} before setting parameters.
EOF
}

wait_for_follow_node() {
  if awk -v timeout="$WAIT_TIMEOUT_S" 'BEGIN { exit !(timeout <= 0.0) }'; then
    return 0
  fi
  local deadline
  deadline="$(awk -v now="$(date +%s)" -v timeout="$WAIT_TIMEOUT_S" 'BEGIN { printf "%.0f\n", now + timeout }')"
  echo "[uav_position] Waiting for ${UAV_NODE} parameters..."
  while [ "$(date +%s)" -le "$deadline" ]; do
    if ros2 param get --no-daemon "$UAV_NODE" d_target >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  echo "[uav_position] ${UAV_NODE} is not active or has no d_target parameter after ${WAIT_TIMEOUT_S}s" >&2
  return 1
}

set_param() {
  local name="$1"
  local value="$2"
  echo "[uav_position] Setting ${UAV_NODE}.${name}=${value}"
  ros2 param set --no-daemon "$UAV_NODE" "$name" "$value"
}

get_param_number() {
  local name="$1"
  local output=""
  output="$(ros2 param get --no-daemon "$UAV_NODE" "$name")"
  awk '{print $NF}' <<< "$output"
}

normalize_angle_deg() {
  awk -v angle="$1" 'BEGIN {
    while (angle > 180.0) angle -= 360.0
    while (angle <= -180.0) angle += 360.0
    printf "%.6f\n", angle
  }'
}

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

if [[ "$COMMAND" == *":="* || "$COMMAND" == *=* ]]; then
  set -- "$COMMAND" "$@"
  COMMAND="set"
fi

case "$COMMAND" in
  angle)
    if [ "$#" -lt 1 ]; then
      usage >&2
      exit 2
    fi
    ANGLE_DELTA="$1"
    shift
    ;;
  angle_abs|heading|heading_offset)
    if [ "$#" -lt 1 ]; then
      usage >&2
      exit 2
    fi
    ANGLE_ABS="$1"
    shift
    ;;
  distance|dist)
    if [ "$#" -lt 1 ]; then
      usage >&2
      exit 2
    fi
    DISTANCE="$1"
    shift
    ;;
  set)
    ;;
  help|-h|--help|"")
    usage
    exit 0
    ;;
  *)
    echo "Unknown UAV position command: $COMMAND" >&2
    usage >&2
    exit 2
    ;;
esac

for arg in "$@"; do
  case "$arg" in
    angle:=*|delta:=*)
      ANGLE_DELTA="${arg#*:=}"
      ;;
    angle_abs:=*|heading:=*|heading_offset:=*)
      ANGLE_ABS="${arg#*:=}"
      ;;
    distance:=*|dist:=*)
      DISTANCE="${arg#*:=}"
      ;;
    node:=*)
      UAV_NODE="${arg#node:=}"
      ;;
    wait_s:=*|timeout_s:=*)
      WAIT_TIMEOUT_S="${arg#*:=}"
      ;;
    no_wait:=true|wait:=false)
      WAIT_TIMEOUT_S="0"
      ;;
    "")
      ;;
    *)
      echo "Unknown UAV position argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [ -z "$ANGLE_ABS" ] && [ -z "$ANGLE_DELTA" ] && [ -z "$DISTANCE" ]; then
  echo "No UAV position value was provided." >&2
  usage >&2
  exit 2
fi

wait_for_follow_node

if [ -n "$ANGLE_ABS" ]; then
  next_angle="$(normalize_angle_deg "$ANGLE_ABS")"
  set_param leader_heading_offset_deg "$next_angle"
fi

if [ -n "$ANGLE_DELTA" ]; then
  current_angle="$(get_param_number leader_heading_offset_deg)"
  next_angle="$(awk -v current="$current_angle" -v delta="$ANGLE_DELTA" 'BEGIN { printf "%.6f\n", current + delta }')"
  next_angle="$(normalize_angle_deg "$next_angle")"
  echo "[uav_position] Current angle=${current_angle}, delta=${ANGLE_DELTA}, next=${next_angle}"
  set_param leader_heading_offset_deg "$next_angle"
fi

if [ -n "$DISTANCE" ]; then
  set_param d_target "$DISTANCE"
fi
