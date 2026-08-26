#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
FOLLOW_SIM=false
SCAN_PID=""
WATCH_PID=""
EXTRA_ARGS=()
ROS_ARGS=()

UAV_NAMES="dji1,dji2"
YAW_CENTER_DEG="0.0"
YAW_AMPLITUDE_DEG="120.0"
PERIOD_S="10.0"
PAN_PHASE_OFFSETS_DEG="0,180"
PITCH_DEG="-20.0"
PITCH_AMPLITUDE_DEG="0.0"
PITCH_PERIOD_S="0.0"
PITCH_PHASE_OFFSETS_DEG=""
PUBLISH_PITCH="true"
RATE_HZ="10.0"
STATUS_TOPIC="/coord/support/camera_scan_status"

sim_helper_running() {
  if [ ! -f "$SIM_PID_FILE" ]; then
    return 1
  fi

  local sim_pid
  sim_pid="$(cat "$SIM_PID_FILE" 2>/dev/null || true)"
  if [ -z "$sim_pid" ]; then
    return 1
  fi

  kill -0 "$sim_pid" 2>/dev/null
}

scan_running() {
  if [ -z "$SCAN_PID" ]; then
    return 1
  fi
  kill -0 "$SCAN_PID" 2>/dev/null
}

stop_scan_group() {
  local signal="$1"
  local timeout_s="$2"
  local waited_s=0
  local scan_pgid=""

  if ! scan_running; then
    return 0
  fi

  scan_pgid="$(ps -o pgid= -p "$SCAN_PID" 2>/dev/null | tr -d ' ')"
  if [ -n "$scan_pgid" ]; then
    /bin/kill "-$signal" -- "-$scan_pgid" 2>/dev/null || true
  else
    kill "-$signal" "$SCAN_PID" 2>/dev/null || true
  fi

  while scan_running && [ "$waited_s" -lt "$timeout_s" ]; do
    sleep 1
    waited_s=$((waited_s + 1))
  done
}

cleanup() {
  if [ -n "$WATCH_PID" ] && kill -0 "$WATCH_PID" 2>/dev/null; then
    kill "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true
  fi

  if scan_running; then
    stop_scan_group INT 5
    stop_scan_group TERM 3
    if scan_running; then
      stop_scan_group KILL 0
    fi
  fi

  if [ -n "$SCAN_PID" ]; then
    wait "$SCAN_PID" 2>/dev/null || true
  fi
}

trap cleanup INT TERM EXIT

if sim_helper_running; then
  FOLLOW_SIM=true
  echo "[run_support_camera_scan] Gazebo helper detected; stopping the scan helper when the sim helper exits."
fi

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  shift
fi

for arg in "$@"; do
  case "$arg" in
    uavs:=*|uav_names:=*)
      UAV_NAMES="${arg#*:=}"
      ;;
    yaw_center_deg:=*)
      YAW_CENTER_DEG="${arg#yaw_center_deg:=}"
      ;;
    yaw_amplitude_deg:=*)
      YAW_AMPLITUDE_DEG="${arg#yaw_amplitude_deg:=}"
      ;;
    period_s:=*)
      PERIOD_S="${arg#period_s:=}"
      ;;
    pan_phase_offsets_deg:=*)
      PAN_PHASE_OFFSETS_DEG="${arg#pan_phase_offsets_deg:=}"
      ;;
    pitch_deg:=*)
      PITCH_DEG="${arg#pitch_deg:=}"
      ;;
    pitch_amplitude_deg:=*)
      PITCH_AMPLITUDE_DEG="${arg#pitch_amplitude_deg:=}"
      ;;
    pitch_period_s:=*)
      PITCH_PERIOD_S="${arg#pitch_period_s:=}"
      ;;
    pitch_phase_offsets_deg:=*)
      PITCH_PHASE_OFFSETS_DEG="${arg#pitch_phase_offsets_deg:=}"
      ;;
    publish_pitch:=*)
      PUBLISH_PITCH="${arg#publish_pitch:=}"
      ;;
    rate_hz:=*)
      RATE_HZ="${arg#rate_hz:=}"
      ;;
    status_topic:=*)
      STATUS_TOPIC="${arg#status_topic:=}"
      ;;
    *)
      EXTRA_ARGS+=("-p" "$arg")
      ;;
  esac
done

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

echo "[run_support_camera_scan] Starting support camera sweep:"
echo "  uav_names=$UAV_NAMES"
echo "  yaw_center_deg=$YAW_CENTER_DEG yaw_amplitude_deg=$YAW_AMPLITUDE_DEG period_s=$PERIOD_S"
echo "  pan_phase_offsets_deg=$PAN_PHASE_OFFSETS_DEG"
echo "  pitch_deg=$PITCH_DEG pitch_amplitude_deg=$PITCH_AMPLITUDE_DEG pitch_period_s=$PITCH_PERIOD_S"
echo "  pitch_phase_offsets_deg=${PITCH_PHASE_OFFSETS_DEG:-<same-phase>}"

ROS_ARGS+=(
  -p "use_sim_time:=true"
  -p "uav_names:=$UAV_NAMES"
  -p "yaw_center_deg:=$YAW_CENTER_DEG"
  -p "yaw_amplitude_deg:=$YAW_AMPLITUDE_DEG"
  -p "period_s:=$PERIOD_S"
  -p "pitch_deg:=$PITCH_DEG"
  -p "pitch_amplitude_deg:=$PITCH_AMPLITUDE_DEG"
  -p "pitch_period_s:=$PITCH_PERIOD_S"
  -p "publish_pitch:=$PUBLISH_PITCH"
  -p "rate_hz:=$RATE_HZ"
  -p "status_topic:=$STATUS_TOPIC"
)

if [ -n "$PAN_PHASE_OFFSETS_DEG" ]; then
  ROS_ARGS+=(-p "pan_phase_offsets_deg:=$PAN_PHASE_OFFSETS_DEG")
fi
if [ -n "$PITCH_PHASE_OFFSETS_DEG" ]; then
  ROS_ARGS+=(-p "pitch_phase_offsets_deg:=$PITCH_PHASE_OFFSETS_DEG")
fi

setsid ros2 run lrs_halmstad support_camera_scanner --ros-args \
  "${ROS_ARGS[@]}" \
  "${EXTRA_ARGS[@]}" &
SCAN_PID=$!

if [ "$FOLLOW_SIM" = true ]; then
  (
    while scan_running; do
      if ! sim_helper_running; then
        echo "[run_support_camera_scan] Gazebo helper exited; stopping the scan helper."
        stop_scan_group INT 5
        stop_scan_group TERM 3
        if scan_running; then
          stop_scan_group KILL 0
        fi
        exit 0
      fi
      sleep 1
    done
  ) &
  WATCH_PID=$!
fi

set +e
wait "$SCAN_PID"
STATUS=$?
set -e
trap - INT TERM EXIT
cleanup
exit "$STATUS"
