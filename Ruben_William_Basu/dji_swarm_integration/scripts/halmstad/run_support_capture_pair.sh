#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
FOLLOW_SIM=false
WATCH_PID=""
A_PID=""
B_PID=""
COMMON_ARGS=()

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

process_running() {
  local pid="$1"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

stop_process_group() {
  local pid="$1"
  local signal="$2"
  local timeout_s="$3"
  local waited_s=0
  local pgid=""

  if ! process_running "$pid"; then
    return 0
  fi

  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
  if [ -n "$pgid" ]; then
    /bin/kill "-$signal" -- "-$pgid" 2>/dev/null || true
  else
    kill "-$signal" "$pid" 2>/dev/null || true
  fi

  while process_running "$pid" && [ "$waited_s" -lt "$timeout_s" ]; do
    sleep 1
    waited_s=$((waited_s + 1))
  done
}

cleanup() {
  if [ -n "$WATCH_PID" ] && kill -0 "$WATCH_PID" 2>/dev/null; then
    kill "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true
  fi

  for pid in "$A_PID" "$B_PID"; do
    if process_running "$pid"; then
      stop_process_group "$pid" INT 5
      stop_process_group "$pid" TERM 3
      if process_running "$pid"; then
        stop_process_group "$pid" KILL 0
      fi
    fi
  done

  for pid in "$A_PID" "$B_PID"; do
    if [ -n "$pid" ]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}

trap cleanup INT TERM EXIT

if sim_helper_running; then
  FOLLOW_SIM=true
  echo "[run_support_capture_pair] Gazebo helper detected; stopping the capture pair when the sim helper exits."
fi

WORLD="baylands"
if sim_helper_running && [ -f "$SIM_WORLD_FILE" ]; then
  sim_world="$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)"
  if [ -n "$sim_world" ]; then
    WORLD="$sim_world"
  fi
fi

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

UAV_A_NAME="dji1"
UAV_B_NAME="dji2"
OUTPUT_ROOT="$WS_ROOT/datasets/${WORLD}_support_pair"

for arg in "$@"; do
  case "$arg" in
    out:=*|root:=*)
      OUTPUT_ROOT="${arg#*:=}"
      ;;
    uav_a:=*)
      UAV_A_NAME="${arg#uav_a:=}"
      ;;
    uav_b:=*)
      UAV_B_NAME="${arg#uav_b:=}"
      ;;
    uav_names:=*)
      IFS=',' read -r UAV_A_NAME UAV_B_NAME <<<"${arg#uav_names:=}"
      UAV_A_NAME="${UAV_A_NAME:-dji1}"
      UAV_B_NAME="${UAV_B_NAME:-dji2}"
      ;;
    hz:=*)
      COMMON_ARGS+=("-p" "capture_hz:=${arg#hz:=}")
      ;;
    class:=*)
      COMMON_ARGS+=("-p" "class_name:=${arg#class:=}")
      ;;
    id:=*)
      COMMON_ARGS+=("-p" "class_id:=${arg#id:=}")
      ;;
    negatives:=*)
      COMMON_ARGS+=("-p" "save_negative_examples:=${arg#negatives:=}")
      ;;
    overlay:=*)
      COMMON_ARGS+=("-p" "save_overlay:=${arg#overlay:=}")
      ;;
    save_metadata:=*|save_overlay:=*|save_negative_examples:=*|capture_hz:=*|class_id:=*|class_name:=*|val_every_n:=*|min_bbox_pixels:=*|min_bbox_area_px:=*|target_length_m:=*|target_width_m:=*|target_height_m:=*|target_base_z_m:=*|camera_pose_timeout_s:=*|image_timeout_s:=*|target_pose_topic:=*)
      COMMON_ARGS+=("-p" "$arg")
      ;;
    *)
      COMMON_ARGS+=("-p" "$arg")
      ;;
  esac
done

OUTPUT_A="$(python3 -c 'import os,sys; print(os.path.abspath(os.path.expanduser(sys.argv[1])))' "$OUTPUT_ROOT/$UAV_A_NAME")"
OUTPUT_B="$(python3 -c 'import os,sys; print(os.path.abspath(os.path.expanduser(sys.argv[1])))' "$OUTPUT_ROOT/$UAV_B_NAME")"

if [ "$OUTPUT_A" = "$OUTPUT_B" ]; then
  echo "[run_support_capture_pair] Output directories must be unique." >&2
  exit 2
fi

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
set -u

echo "[run_support_capture_pair] Starting paired capture:"
echo "  world=$WORLD"
echo "  $UAV_A_NAME -> $OUTPUT_A"
echo "  $UAV_B_NAME -> $OUTPUT_B"

setsid ros2 run lrs_halmstad sim_dataset_capture --ros-args \
  -p use_sim_time:=true \
  -p uav_name:="$UAV_A_NAME" \
  -p output_dir:="$OUTPUT_A" \
  -p dataset_name:="${WORLD}_${UAV_A_NAME}" \
  "${COMMON_ARGS[@]}" &
A_PID=$!

setsid ros2 run lrs_halmstad sim_dataset_capture --ros-args \
  -p use_sim_time:=true \
  -p uav_name:="$UAV_B_NAME" \
  -p output_dir:="$OUTPUT_B" \
  -p dataset_name:="${WORLD}_${UAV_B_NAME}" \
  "${COMMON_ARGS[@]}" &
B_PID=$!

if [ "$FOLLOW_SIM" = true ]; then
  (
    while process_running "$A_PID" || process_running "$B_PID"; do
      if ! sim_helper_running; then
        echo "[run_support_capture_pair] Gazebo helper exited; stopping both capture nodes."
        for pid in "$A_PID" "$B_PID"; do
          if process_running "$pid"; then
            stop_process_group "$pid" INT 5
            stop_process_group "$pid" TERM 3
            if process_running "$pid"; then
              stop_process_group "$pid" KILL 0
            fi
          fi
        done
        exit 0
      fi
      sleep 1
    done
  ) &
  WATCH_PID=$!
fi

set +e
wait "$A_PID"
STATUS_A=$?
wait "$B_PID"
STATUS_B=$?
set -e

STATUS=$STATUS_A
if [ "$STATUS" -eq 0 ] && [ "$STATUS_B" -ne 0 ]; then
  STATUS=$STATUS_B
fi

trap - INT TERM EXIT
cleanup
exit "$STATUS"
