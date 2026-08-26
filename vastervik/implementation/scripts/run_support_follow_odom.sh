#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"
SIM_WORLD_FILE="$STATE_DIR/gazebo_sim.world"
FOLLOW_SIM=false
LAUNCH_PID=""
WATCH_PID=""
HAVE_LEADER_START_X=false
HAVE_LEADER_START_Y=false
HAVE_LEADER_START_YAW=false
HAVE_LEADER_NOMINAL_Z=false
EXTRA_ARGS=()

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

launch_running() {
  if [ -z "$LAUNCH_PID" ]; then
    return 1
  fi

  kill -0 "$LAUNCH_PID" 2>/dev/null
}

stop_launch_group() {
  local signal="$1"
  local timeout_s="$2"
  local waited_s=0
  local launch_pgid=""

  if ! launch_running; then
    return 0
  fi

  launch_pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
  if [ -n "$launch_pgid" ]; then
    /bin/kill "-$signal" -- "-$launch_pgid" 2>/dev/null || true
  else
    kill "-$signal" "$LAUNCH_PID" 2>/dev/null || true
  fi

  while launch_running && [ "$waited_s" -lt "$timeout_s" ]; do
    sleep 1
    waited_s=$((waited_s + 1))
  done
}

cleanup() {
  if [ -n "$WATCH_PID" ] && kill -0 "$WATCH_PID" 2>/dev/null; then
    kill "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true
  fi

  if launch_running; then
    stop_launch_group INT 5
    stop_launch_group TERM 3
    if launch_running; then
      stop_launch_group KILL 0
    fi
  fi

  if [ -n "$LAUNCH_PID" ]; then
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}

trap cleanup INT TERM EXIT

if sim_helper_running; then
  FOLLOW_SIM=true
  echo "[run_support_follow_odom] Gazebo helper detected; stopping this overlay when the sim helper exits."
fi

DEFAULT_WORLD="baylands"
if sim_helper_running && [ -f "$SIM_WORLD_FILE" ]; then
  sim_world="$(cat "$SIM_WORLD_FILE" 2>/dev/null || true)"
  if [ -n "$sim_world" ]; then
    DEFAULT_WORLD="$sim_world"
  fi
fi

WORLD="$DEFAULT_WORLD"
if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

for arg in "$@"; do
  case "$arg" in
    leader_start_x:=*)
      HAVE_LEADER_START_X=true
      EXTRA_ARGS+=("$arg")
      ;;
    leader_start_y:=*)
      HAVE_LEADER_START_Y=true
      EXTRA_ARGS+=("$arg")
      ;;
    leader_start_yaw_deg:=*)
      HAVE_LEADER_START_YAW=true
      EXTRA_ARGS+=("$arg")
      ;;
    leader_nominal_z:=*)
      HAVE_LEADER_NOMINAL_Z=true
      EXTRA_ARGS+=("$arg")
      ;;
    *)
      EXTRA_ARGS+=("$arg")
      ;;
  esac
done

ORIG_ROS_DOMAIN_ID="${ROS_DOMAIN_ID-}"

set +u
source "$WS_ROOT/scripts/slam_state_common.sh"
set -u

set +u
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

if [ -n "$ORIG_ROS_DOMAIN_ID" ]; then
  export ROS_DOMAIN_ID="$ORIG_ROS_DOMAIN_ID"
fi

echo "[run_support_follow_odom] Overlaying dji1+dji2 support-follow on top of the trusted 1-to-1 baseline."
if ! ros2 topic list 2>/dev/null | grep -qx '/dji0/pose'; then
  echo "[run_support_follow_odom] /dji0/pose is not visible yet; support UAVs will still spawn now and their controllers will wait for the main UAV pose." >&2
fi

add_live_leader_start_from_dji0_pose() {
  local timeout_s="${1:-5}"
  local pose_env=""
  local leader_yaw_deg=""

  pose_env="$(slam_state_capture_gazebo_named_pose_env "$WORLD" "dji0" "$timeout_s")" || return 1
  eval "$pose_env"
  leader_yaw_deg="$(
    python3 - "$spawn_yaw" <<'PY'
import math
import sys

print(f"{math.degrees(float(sys.argv[1])):.9f}")
PY
  )" || return 1

  if [ "$HAVE_LEADER_START_X" = false ]; then
    EXTRA_ARGS+=("leader_start_x:=$spawn_x")
  fi
  if [ "$HAVE_LEADER_START_Y" = false ]; then
    EXTRA_ARGS+=("leader_start_y:=$spawn_y")
  fi
  if [ "$HAVE_LEADER_START_YAW" = false ]; then
    EXTRA_ARGS+=("leader_start_yaw_deg:=$leader_yaw_deg")
  fi
  if [ "$HAVE_LEADER_NOMINAL_Z" = false ]; then
    EXTRA_ARGS+=("leader_nominal_z:=$spawn_z")
  fi

  echo "[run_support_follow_odom] Using live dji0 pose for support spawn x=${spawn_x} y=${spawn_y} z=${spawn_z} yaw_deg=${leader_yaw_deg}"
}

if ! add_live_leader_start_from_dji0_pose 5; then
  echo "[run_support_follow_odom] Warning: could not capture live dji0 pose; support UAVs will use launch defaults for slot placement." >&2
fi

setsid ros2 launch lrs_halmstad support_follow_odom.launch.py world:="$WORLD" "${EXTRA_ARGS[@]}" &
LAUNCH_PID=$!

if [ "$FOLLOW_SIM" = true ]; then
  (
    while launch_running; do
      if ! sim_helper_running; then
        echo "[run_support_follow_odom] Gazebo helper exited; stopping overlay."
        stop_launch_group INT 5
        stop_launch_group TERM 3
        if launch_running; then
          echo "[run_support_follow_odom] Overlay ignored shutdown signals; forcing exit."
          stop_launch_group KILL 0
        fi
        exit 0
      fi
      sleep 1
    done
  ) &
  WATCH_PID=$!
fi

set +e
wait "$LAUNCH_PID"
STATUS=$?
set -e
trap - INT TERM EXIT
cleanup
exit "$STATUS"
