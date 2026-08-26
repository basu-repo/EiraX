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
WAIT_FOR_GAZEBO="${WAIT_FOR_GAZEBO:-true}"
GAZEBO_READY_TIMEOUT_S="${GAZEBO_READY_TIMEOUT_S:-180}"

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

launch_group_running() {
  local launch_pgid

  if ! launch_running; then
    return 1
  fi

  launch_pgid="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')"
  if [ -z "$launch_pgid" ]; then
    return 1
  fi

  /bin/kill -0 -- "-$launch_pgid" 2>/dev/null
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
  echo "[run_spawn_uav] Gazebo helper detected; stopping this launcher when the sim helper exits."
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

ARGS=()
for arg in "$@"; do
  case "$arg" in
    wait_for_gazebo:=*)
      WAIT_FOR_GAZEBO="${arg#wait_for_gazebo:=}"
      ;;
    gazebo_ready_timeout_s:=*|sim_ready_timeout_s:=*)
      GAZEBO_READY_TIMEOUT_S="${arg#*:=}"
      ;;
    camera:=*)
      camera_mode="${arg#camera:=}"
      case "$camera_mode" in
        attached|integrated|integrated_joint)
          ARGS+=("uav_camera_mode:=integrated_joint")
          ;;
        detached|detached_model)
          echo "Detached camera mode has been removed from simulation. Use camera:=attached." >&2
          exit 2
          ;;
        *)
          ARGS+=("uav_camera_mode:=$camera_mode")
          ;;
      esac
      ;;
    name:=*)
      ARGS+=("uav_name:=${arg#name:=}")
      ;;
    height:=*)
      ARGS+=("z:=${arg#height:=}")
      ;;
    mount_pitch_deg:=*)
      ARGS+=("camera_pitch_offset_deg:=${arg#mount_pitch_deg:=}")
      ;;
    *)
      ARGS+=("$arg")
      ;;
  esac
done

ORIG_ROS_DOMAIN_ID="${ROS_DOMAIN_ID-}"

set +u
# ROS setup scripts may read unset variables while initializing the environment.
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/src/lrs_halmstad/clearpath/setup.bash"
set -u

if [ -n "$ORIG_ROS_DOMAIN_ID" ]; then
  export ROS_DOMAIN_ID="$ORIG_ROS_DOMAIN_ID"
fi

gazebo_world_name() {
  case "$1" in
    construction)
      printf '%s\n' "office_construction"
      ;;
    *)
      printf '%s\n' "$1"
      ;;
  esac
}

wait_for_gazebo_ready() {
  local gz_world
  local deadline
  gz_world="$(gazebo_world_name "$WORLD")"
  deadline=$((SECONDS + GAZEBO_READY_TIMEOUT_S))

  echo "[run_spawn_uav] Waiting for Gazebo clock and /world/${gz_world}/create"
  while (( SECONDS < deadline )); do
    if timeout 4s ros2 topic echo --no-daemon --once /clock >/dev/null 2>&1; then
      echo "[run_spawn_uav] Gazebo clock is publishing."
      break
    fi
    sleep 2
  done
  if (( SECONDS >= deadline )); then
    echo "[run_spawn_uav] Timed out waiting for /clock after ${GAZEBO_READY_TIMEOUT_S}s" >&2
    return 1
  fi

  while (( SECONDS < deadline )); do
    if command -v gz >/dev/null 2>&1 && timeout 4s gz service -l 2>/dev/null | grep -qx "/world/${gz_world}/create"; then
      echo "[run_spawn_uav] Gazebo create service is ready."
      return 0
    fi
    sleep 2
  done

  echo "[run_spawn_uav] Timed out waiting for /world/${gz_world}/create after ${GAZEBO_READY_TIMEOUT_S}s" >&2
  return 1
}

case "$WAIT_FOR_GAZEBO" in
  true)
    wait_for_gazebo_ready
    ;;
  false)
    ;;
  *)
    echo "[run_spawn_uav] Invalid wait_for_gazebo option: $WAIT_FOR_GAZEBO (use true or false)" >&2
    exit 2
    ;;
esac

setsid ros2 launch lrs_halmstad spawn_uav_1to1.launch.py world:="$WORLD" "${ARGS[@]}" &
LAUNCH_PID=$!

if [ "$FOLLOW_SIM" = true ]; then
  (
    while launch_running; do
      if ! sim_helper_running; then
        echo "[run_spawn_uav] Gazebo helper exited; stopping UAV launcher."
        stop_launch_group INT 5
        stop_launch_group TERM 3
        if launch_running; then
          echo "[run_spawn_uav] UAV launcher ignored shutdown signals; forcing exit."
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
