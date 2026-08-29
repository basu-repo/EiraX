#!/usr/bin/env bash
# Reset a running yolo/follow experiment without restarting Gazebo or Nav2.
# Kills the follow (and optionally record) stack, teleports the UAV back to its
# spawn position, and restarts the same follow command in the existing tmux pane.
#
# Usage: ./run.sh reset_experiment [world] [session:=name] [reset_ugv:=false]
#                                   [kill_grace_s:=3] [dry_run:=false]
#
# Prerequisites: the session must have been started with run_tmux_1to1.sh so
# that FOLLOW_CMD_STR and pane IDs are saved in the state file.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
TMUX_STATE_DIR="$STATE_DIR/tmux_sessions"

# Defaults — overridden by the loaded state file.
SESSION=""
WORLD="baylands"
MODE="yolo"
UAV_NAME="dji0"
UGV_NAMESPACE="a201_0000"
ROS_DOMAIN_ID_EFFECTIVE="${ROS_DOMAIN_ID:-3}"
RMW_IMPLEMENTATION_EFFECTIVE="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
RECORD="false"
FOLLOW_CMD_STR=""
RECORD_CMD_STR=""
LAYOUT=""
EFFECTIVE_GUI="false"
GAZEBO_PANE_ID=""
SPAWN_PANE_ID=""
LOCALIZATION_PANE_ID=""
NAV2_PANE_ID=""
FOLLOW_PANE_ID=""
RECORD_PANE_ID=""

RESET_UAV=true
RESET_UGV=false
KILL_GRACE_S=3
DRY_RUN=false

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  SESSION="halmstad-${WORLD}-1to1"
  shift
fi

for arg in "$@"; do
  case "$arg" in
    session:=*)
      SESSION="${arg#session:=}"
      ;;
    reset_uav:=*)
      RESET_UAV="${arg#reset_uav:=}"
      ;;
    reset_ugv:=*)
      RESET_UGV="${arg#reset_ugv:=}"
      ;;
    kill_grace_s:=*)
      KILL_GRACE_S="${arg#kill_grace_s:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      echo "Usage: $0 [world] [session:=name] [reset_uav:=true|false] [reset_ugv:=true|false] [kill_grace_s:=3] [dry_run:=true|false]" >&2
      exit 2
      ;;
  esac
done

# ---------------------------------------------------------------------------
# Load session state file
# ---------------------------------------------------------------------------

SESSION_SAFE="$(printf '%s' "${SESSION:-halmstad-${WORLD}-1to1}" | tr -c 'A-Za-z0-9_.-' '_')"
SESSION_STATE_FILE="$TMUX_STATE_DIR/${SESSION_SAFE}.env"

discover_fallback_state_file() {
  local candidate=""
  local matches=()
  candidate="$TMUX_STATE_DIR/halmstad-1to1.env"
  if [ -f "$candidate" ]; then
    printf '%s\n' "$candidate"
    return 0
  fi
  if [ -d "$TMUX_STATE_DIR" ]; then
    while IFS= read -r candidate; do
      matches+=("$candidate")
    done < <(find "$TMUX_STATE_DIR" -maxdepth 1 -type f -name '*.env' | sort)
  fi
  if [ "${#matches[@]}" -eq 1 ]; then
    printf '%s\n' "${matches[0]}"
    return 0
  fi
  return 1
}

if [ -f "$SESSION_STATE_FILE" ]; then
  # shellcheck disable=SC1090
  source "$SESSION_STATE_FILE"
  SESSION_SAFE="$(printf '%s' "$SESSION" | tr -c 'A-Za-z0-9_.-' '_')"
  SESSION_STATE_FILE="$TMUX_STATE_DIR/${SESSION_SAFE}.env"
else
  fallback_state_file="$(discover_fallback_state_file || true)"
  if [ -n "${fallback_state_file:-}" ] && [ -f "$fallback_state_file" ]; then
    # shellcheck disable=SC1090
    source "$fallback_state_file"
    SESSION_SAFE="$(printf '%s' "$SESSION" | tr -c 'A-Za-z0-9_.-' '_')"
    SESSION_STATE_FILE="$TMUX_STATE_DIR/${SESSION_SAFE}.env"
  else
    echo "No session state file found. Was the session started with run_tmux_1to1.sh?" >&2
    exit 1
  fi
fi

if [ -z "$FOLLOW_CMD_STR" ]; then
  echo "State file does not contain FOLLOW_CMD_STR. Re-run the session with a newer run_tmux_1to1.sh to populate it." >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# Verify tmux session exists
# ---------------------------------------------------------------------------

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session not found: $SESSION" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------

pane_exists() {
  local pane_id="$1"
  [ -n "$pane_id" ] || return 1
  tmux list-panes -a -t "$SESSION" -F '#{pane_id}' 2>/dev/null | grep -Fxq "$pane_id"
}

send_ctrl_c_pane() {
  local pane_id="$1"
  local label="$2"
  if pane_exists "$pane_id"; then
    echo "Sending Ctrl-C to $label ($pane_id)"
    if [ "$DRY_RUN" != true ]; then
      tmux send-keys -t "$pane_id" C-c
    fi
    return 0
  fi
  echo "Skipping Ctrl-C for $label (pane not found)"
  return 1
}

signal_processes_by_pattern() {
  local pattern="$1"
  if pkill -INT -f "$pattern" 2>/dev/null; then
    sleep 1
    pkill -TERM -f "$pattern" 2>/dev/null || true
    sleep 1
    pkill -KILL -f "$pattern" 2>/dev/null || true
  fi
}

signal_named_nodes() {
  local names_regex="$1"
  signal_processes_by_pattern "__node:=($names_regex)(\\s|\$)"
}

# ---------------------------------------------------------------------------
# Step 1 — Stop follow and record panes
# ---------------------------------------------------------------------------

echo "=== Stopping follow stack ==="
send_ctrl_c_pane "$FOLLOW_PANE_ID" "follow" || true

if [ "$RECORD" = true ] && [ -n "$RECORD_PANE_ID" ]; then
  send_ctrl_c_pane "$RECORD_PANE_ID" "record" || true
fi

if [ "$KILL_GRACE_S" != "0" ]; then
  echo "Waiting ${KILL_GRACE_S}s for graceful shutdown..."
  if [ "$DRY_RUN" != true ]; then
    sleep "$KILL_GRACE_S"
  fi
fi

# ---------------------------------------------------------------------------
# Step 2 — Kill any lingering follow-related ROS nodes
# ---------------------------------------------------------------------------

echo "=== Killing follow-related ROS nodes ==="
FOLLOW_NODES='camera_tracker|ugv_nav2_driver|pose_cmd_to_odom|gazebo_pose_tcp_bridge|omnet_metrics_bridge|visual_actuation_bridge|leader_detector|leader_tracker|leader_estimator|selected_target_filter|visual_target_estimator|follow_point_generator|follow_point_planner|visual_follow_controller|uav_simulator|follow_uav_odom|follow_uav|pose_cov_to_odom|odom_to_tf'

if [ "$DRY_RUN" != true ]; then
  signal_named_nodes "$FOLLOW_NODES"
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_follow\.launch\.py'
  signal_processes_by_pattern 'ros2 launch lrs_halmstad run_1to1_follow\.launch\.py'
  sleep 2
fi

# ---------------------------------------------------------------------------
# Step 3 — Teleport UAV back to spawn position
# ---------------------------------------------------------------------------

if [ "$RESET_UAV" = true ]; then
  echo "=== Teleporting UAV to spawn position ==="

  # UAV spawn: djiN → (0, 0, 7+N)
  uav_z=7
  if [[ "$UAV_NAME" =~ ^dji([0-9]+)$ ]]; then
    uav_z=$(( 7 + BASH_REMATCH[1] ))
  fi

  echo "  Entity: $UAV_NAME  z: $uav_z  (identity rotation)"

  if [ "$DRY_RUN" != true ]; then
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    # shellcheck disable=SC1091
    source "$WS_ROOT/install/setup.bash" 2>/dev/null || true
    set -u

    ros2 service call "/world/${WORLD}/set_pose" ros_gz_interfaces/srv/SetEntityPose \
      "{entity: {name: '${UAV_NAME}', type: 2}, pose: {position: {x: 0.0, y: 0.0, z: ${uav_z}.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" \
      2>&1 || echo "  Warning: UAV teleport service call failed — is Gazebo running?"
  else
    echo "  [dry-run] ros2 service call /world/${WORLD}/set_pose ... UAV z=${uav_z}"
  fi
fi

# ---------------------------------------------------------------------------
# Step 4 — Optionally teleport UGV back to spawn position
# ---------------------------------------------------------------------------

if [ "$RESET_UGV" = true ]; then
  echo "=== Teleporting UGV to spawn position ==="

  # World-specific UGV spawn poses (from run_follow.launch.py defaults).
  case "$WORLD" in
    warehouse)
      ugv_x=0.449; ugv_y=0.139
      # yaw=-4.6° → qz=sin(-2.3°)=-0.04013, qw=cos(-2.3°)=0.99919
      ugv_qz=-0.04013; ugv_qw=0.99919
      ;;
    construction)
      ugv_x=-0.048; ugv_y=-0.179
      # yaw=-53.9° → qz=sin(-26.95°)=-0.45306, qw=cos(-26.95°)=0.89144
      ugv_qz=-0.45306; ugv_qw=0.89144
      ;;
    *)
      ugv_x=0.0; ugv_y=0.0; ugv_qz=0.0; ugv_qw=1.0
      ;;
  esac

  # Clearpath robot_spawn.launch.py spawns with -name <namespace>/robot, so the
  # Gazebo entity name is "<namespace>/robot", not just the namespace.
  ugv_entity="${UGV_NAMESPACE}/robot"

  echo "  Entity: $ugv_entity  x:$ugv_x y:$ugv_y z:0.3  qz:$ugv_qz qw:$ugv_qw"

  if [ "$DRY_RUN" != true ]; then
    set +u
    source /opt/ros/jazzy/setup.bash 2>/dev/null || true
    source "$WS_ROOT/install/setup.bash" 2>/dev/null || true
    set -u

    ros2 service call "/world/${WORLD}/set_pose" ros_gz_interfaces/srv/SetEntityPose \
      "{entity: {name: '${ugv_entity}', type: 2}, pose: {position: {x: ${ugv_x}, y: ${ugv_y}, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: ${ugv_qz}, w: ${ugv_qw}}}}" \
      2>&1 || echo "  Warning: UGV teleport service call failed"
  else
    echo "  [dry-run] ros2 service call /world/${WORLD}/set_pose ... UGV entity=$ugv_entity x=$ugv_x y=$ugv_y z=0.3"
  fi
fi

# ---------------------------------------------------------------------------
# Step 5 — Restart follow in the follow pane
# ---------------------------------------------------------------------------

echo "=== Restarting follow stack ==="

follow_restart_line="cd $(printf '%q' "$WS_ROOT") && export ROS_DOMAIN_ID=$(printf '%q' "$ROS_DOMAIN_ID_EFFECTIVE") && export RMW_IMPLEMENTATION=$(printf '%q' "$RMW_IMPLEMENTATION_EFFECTIVE") && $FOLLOW_CMD_STR"

if [ "$DRY_RUN" = true ]; then
  echo "[dry-run] follow pane ($FOLLOW_PANE_ID):"
  echo "  $follow_restart_line"
else
  if pane_exists "$FOLLOW_PANE_ID"; then
    tmux send-keys -t "$FOLLOW_PANE_ID" "$follow_restart_line" C-m
  else
    echo "Warning: follow pane $FOLLOW_PANE_ID not found — cannot restart follow" >&2
  fi
fi

# Optionally restart recording (3 s delay to let follow nodes come up)
if [ "$RECORD" = true ] && [ -n "$RECORD_CMD_STR" ] && [ -n "$RECORD_PANE_ID" ]; then
  echo "=== Restarting recording in 3s ==="
  record_restart_line="sleep 3 && cd $(printf '%q' "$WS_ROOT") && export ROS_DOMAIN_ID=$(printf '%q' "$ROS_DOMAIN_ID_EFFECTIVE") && export RMW_IMPLEMENTATION=$(printf '%q' "$RMW_IMPLEMENTATION_EFFECTIVE") && $RECORD_CMD_STR"
  if [ "$DRY_RUN" = true ]; then
    echo "[dry-run] record pane ($RECORD_PANE_ID):"
    echo "  $record_restart_line"
  else
    if pane_exists "$RECORD_PANE_ID"; then
      tmux send-keys -t "$RECORD_PANE_ID" "$record_restart_line" C-m
    else
      echo "Warning: record pane $RECORD_PANE_ID not found — cannot restart recording" >&2
    fi
  fi
fi

echo "=== Reset complete. ==="
