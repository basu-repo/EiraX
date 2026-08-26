#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws"
SIM_PID_FILE="$STATE_DIR/gazebo_sim.pid"

WORLD="baylands"
WAYPOINT="rotundan_0"
LIDAR="3d"
GUI="false"
MODE="localization"
SESSION="halmstad-waypoint-localization"
TMUX_ATTACH="true"
DRY_RUN="false"
GAZEBO_READY_TIMEOUT_S="180"
GAZEBO_READY_SETTLE_S="10"
LOCALIZATION_DELAY_S="20"
NAV2_DELAY_S="24"

LOCALIZATION_ARGS=()
NAV2_ARGS=()
GAZEBO_ARGS=()

usage() {
  cat <<EOF
Usage:
  ./run.sh waypoint_localization [world] [args...]

Starts:
  - Gazebo at waypoint
  - localization
  - Nav2 only when mode:=nav2

Options:
  waypoint:=name              Default rotundan_0
  lidar:=2d|3d                Default 3d
  mode:=localization|nav2     Default localization
  gui:=true|false             Default false
  session:=name               Default halmstad-waypoint-localization
  tmux_attach:=true|false     Default true
  gazebo_ready_settle_s:=10
  localization_delay_s:=20
  nav2_delay_s:=24
  dry_run:=true|false

pc2ls_*/scan_relay_* args are forwarded to localization.
scan_topic:=... is forwarded to localization and Nav2.
EOF
}

shell_join() {
  local out=""
  local part=""
  for part in "$@"; do
    printf -v out '%s%q ' "$out" "$part"
  done
  printf '%s' "${out% }"
}

pane_by_title() {
  local title="$1"
  tmux list-panes -t "$SESSION:waypoint-localization" -F '#{pane_title} #{pane_id}' 2>/dev/null \
    | awk -v title="$title" '$1 == title { print $2; exit }'
}

send_pane_line() {
  local pane_id="$1"
  local line="$2"
  tmux send-keys -t "$pane_id" C-c
  tmux send-keys -t "$pane_id" "clear" C-m
  tmux send-keys -t "$pane_id" "$line" C-m
}

build_gazebo_ready_cmd() {
  cat <<EOF
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
set -u
deadline=\$((SECONDS + $GAZEBO_READY_TIMEOUT_S))
while (( SECONDS < deadline )); do
  if timeout 4s ros2 topic echo --no-daemon --once /clock >/dev/null 2>&1; then
    echo "[gazebo_ready] clock ready"
    break
  fi
  echo "[gazebo_ready] waiting for /clock"
  sleep 2
done
if (( SECONDS >= deadline )); then
  echo "[gazebo_ready] timed out waiting for /clock" >&2
  exit 1
fi
if [ "$GAZEBO_READY_SETTLE_S" != "0" ] && [ "$GAZEBO_READY_SETTLE_S" != "0.0" ]; then
  echo "[gazebo_ready] settling for $GAZEBO_READY_SETTLE_S seconds"
  sleep "$GAZEBO_READY_SETTLE_S"
fi
EOF
}

build_line() {
  local delay_s="$1"
  local wait_for_gazebo="$2"
  shift 2
  local line=""
  printf -v line 'cd %q && ' "$WS_ROOT"
  printf -v line '%sexport ROS_DOMAIN_ID=%q && export RMW_IMPLEMENTATION=%q && ' "$line" "${ROS_DOMAIN_ID:-3}" "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  if [ "$wait_for_gazebo" = "true" ]; then
    printf -v line '%swhile [ ! -f %q ]; do sleep 1; done && ' "$line" "$SIM_PID_FILE"
  fi
  if [ "$delay_s" != "0" ] && [ "$delay_s" != "0.0" ]; then
    printf -v line '%ssleep %q && ' "$line" "$delay_s"
  fi
  if [ "$wait_for_gazebo" = "true" ]; then
    printf -v line '%sbash -lc %q && ' "$line" "$(build_gazebo_ready_cmd)"
  fi
  printf -v line '%s%s' "$line" "$(shell_join "$@")"
  printf '%s' "$line"
}

if [ "$#" -gt 0 ] && [[ "$1" != *":="* ]] && [[ "$1" != *=* ]]; then
  WORLD="$1"
  shift
fi

for arg in "$@"; do
  case "$arg" in
    help|-h|--help)
      usage
      exit 0
      ;;
    waypoint:=*)
      WAYPOINT="${arg#waypoint:=}"
      ;;
    lidar:=2d|scan_sensor:=2d)
      LIDAR="2d"
      ;;
    lidar:=3d|scan_sensor:=3d)
      LIDAR="3d"
      ;;
    mode:=localization|mode:=nav2)
      MODE="${arg#mode:=}"
      ;;
    gui:=*)
      GUI="${arg#gui:=}"
      ;;
    session:=*)
      SESSION="${arg#session:=}"
      ;;
    tmux_attach:=*)
      TMUX_ATTACH="${arg#tmux_attach:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    gazebo_ready_timeout_s:=*|sim_ready_timeout_s:=*)
      GAZEBO_READY_TIMEOUT_S="${arg#*:=}"
      ;;
    gazebo_ready_settle_s:=*|sim_ready_settle_s:=*)
      GAZEBO_READY_SETTLE_S="${arg#*:=}"
      ;;
    localization_delay_s:=*)
      LOCALIZATION_DELAY_S="${arg#localization_delay_s:=}"
      ;;
    nav2_delay_s:=*)
      NAV2_DELAY_S="${arg#nav2_delay_s:=}"
      ;;
    pc2ls_*|scan_relay_*|use_scan_relay:=*)
      LOCALIZATION_ARGS+=("$arg")
      ;;
    scan_topic:=*|pointcloud_topic:=*)
      LOCALIZATION_ARGS+=("$arg")
      case "$arg" in
        scan_topic:=*)
          NAV2_ARGS+=("$arg")
          ;;
      esac
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      usage >&2
      exit 2
      ;;
  esac
done

GAZEBO_CMD=(./run.sh gazebo_sim "$WORLD" "gui:=$GUI" "waypoint:=$WAYPOINT" "${GAZEBO_ARGS[@]}")
LOCALIZATION_CMD=(./run.sh localization "$WORLD" "lidar:=$LIDAR" "${LOCALIZATION_ARGS[@]}")
NAV2_CMD=(./run.sh nav2 "lidar:=$LIDAR" "${NAV2_ARGS[@]}")

GAZEBO_LINE="$(build_line 0 false "${GAZEBO_CMD[@]}")"
LOCALIZATION_LINE="$(build_line "$LOCALIZATION_DELAY_S" true "${LOCALIZATION_CMD[@]}")"
NAV2_LINE="$(build_line "$NAV2_DELAY_S" true "${NAV2_CMD[@]}")"

if [ "$DRY_RUN" = "true" ]; then
  echo "Session: $SESSION"
  echo "Mode: $MODE"
  echo "[gazebo]       $GAZEBO_LINE"
  echo "[localization] $LOCALIZATION_LINE"
  if [ "$MODE" = "nav2" ]; then
    echo "[nav2]         $NAV2_LINE"
  fi
  exit 0
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
  if ! tmux list-windows -t "$SESSION" -F '#{window_name}' | grep -Fxq waypoint-localization; then
    echo "tmux session exists but has no waypoint-localization window: $SESSION" >&2
    echo "Attach with: tmux attach -t $SESSION" >&2
    exit 1
  fi

  GAZEBO_PANE="$(pane_by_title gazebo)"
  LOCALIZATION_PANE="$(pane_by_title localization)"
  NAV2_PANE="$(pane_by_title nav2)"

  if [ -z "$GAZEBO_PANE" ] || [ -z "$LOCALIZATION_PANE" ]; then
    echo "tmux session exists but does not have the expected gazebo/localization panes: $SESSION" >&2
    echo "Attach with: tmux attach -t $SESSION" >&2
    exit 1
  fi

  if [ "$MODE" = "nav2" ] && [ -z "$NAV2_PANE" ]; then
    NAV2_PANE="$(tmux split-window -d -P -F '#{pane_id}' -t "$LOCALIZATION_PANE" -v -l 50%)"
    tmux select-pane -t "$NAV2_PANE" -T nav2
  fi

  echo "Reusing $SESSION for waypoint $WAYPOINT"
  if [ -n "$NAV2_PANE" ]; then
    if [ "$MODE" = "nav2" ]; then
      send_pane_line "$NAV2_PANE" "$NAV2_LINE"
    else
      send_pane_line "$NAV2_PANE" "printf 'nav2 disabled for waypoint_localization mode=%s\n' '$MODE'; exec bash"
    fi
  fi
  send_pane_line "$LOCALIZATION_PANE" "$LOCALIZATION_LINE"
  send_pane_line "$GAZEBO_PANE" "$GAZEBO_LINE"
  tmux select-layout -t "$SESSION:waypoint-localization" tiled >/dev/null 2>&1 || true
  tmux select-pane -t "$GAZEBO_PANE"

  echo "Restarted $SESSION"
  echo "Attach with: tmux attach -t $SESSION"

  if [ "$TMUX_ATTACH" = "true" ]; then
    exec tmux attach -t "$SESSION"
  fi
  exit 0
fi

tmux new-session -d -s "$SESSION" -n waypoint-localization
GAZEBO_PANE="$(tmux display-message -p -t "$SESSION:waypoint-localization.0" '#{pane_id}')"
LOCALIZATION_PANE="$(tmux split-window -d -P -F '#{pane_id}' -t "$GAZEBO_PANE" -h -l 50%)"
tmux setw -t "$SESSION" automatic-rename off
tmux setw -t "$SESSION" allow-rename off
tmux select-pane -t "$GAZEBO_PANE" -T gazebo
tmux select-pane -t "$LOCALIZATION_PANE" -T localization
tmux send-keys -t "$GAZEBO_PANE" "$GAZEBO_LINE" C-m
tmux send-keys -t "$LOCALIZATION_PANE" "$LOCALIZATION_LINE" C-m
if [ "$MODE" = "nav2" ]; then
  NAV2_PANE="$(tmux split-window -d -P -F '#{pane_id}' -t "$LOCALIZATION_PANE" -v -l 50%)"
  tmux select-pane -t "$NAV2_PANE" -T nav2
  tmux send-keys -t "$NAV2_PANE" "$NAV2_LINE" C-m
fi
tmux select-layout -t "$SESSION:waypoint-localization" tiled >/dev/null 2>&1 || true
tmux select-pane -t "$GAZEBO_PANE"

echo "Started $SESSION"
echo "Attach with: tmux attach -t $SESSION"

if [ "$TMUX_ATTACH" = "true" ]; then
  exec tmux attach -t "$SESSION"
fi
