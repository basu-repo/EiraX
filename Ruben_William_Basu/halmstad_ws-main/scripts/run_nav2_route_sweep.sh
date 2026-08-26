#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
STATE_DIR="/tmp/halmstad_ws/nav2_route_sweep"
TMUX_STATE_DIR="/tmp/halmstad_ws/tmux_sessions"

WORLD="baylands"
MODE="follow"
LIDAR="3d"
ROUTES_RAW="all"
EXCLUDE_ROUTES_RAW="${EXCLUDE_ROUTES_RAW:-art,playground_1,playground_2,playground_3}"
SWEEP_MODE="waypoints"
SESSION_PREFIX="halmstad-nav2-route-sweep"
TMUX_ATTACH="false"
DRY_RUN="false"
STOP_BEFORE_EACH="true"
STOP_AFTER_EACH="true"
STOP_GROUP_GRACE_S="3"
STOP_FINAL_GRACE_S="3"
START_TIMEOUT_S=300
ROUTE_TIMEOUT_S=2000
BETWEEN_S=3
CHECK_UAV_FOLLOW="true"
UAV_NAME="dji0"
UAV_CHECK_TIMEOUT_S=120
UAV_CHECK_TOPIC_TIMEOUT_S=10
UAV_CHECK_MIN_Z=5.0
UAV_CHECK_HOLD_UGV_S=0
UAV_CHECK_REQUIRE_COMMAND="false"
SKIP_REMAINING_ON_COMPLETE="true"
CHAIN_ROUTE_STARTS="false"
FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE="false"
EXTRA_TMUX_ARGS=()
HAVE_UGV_START_DELAY_ARG="false"

source "$SCRIPT_DIR/baylands_route_lidar_common.sh"
source "$SCRIPT_DIR/baylands_waypoint_common.sh"
WAYPOINT_CONFIG_DIR="$(baylands_waypoint_config_dir)"
GROUP_CSV="$(baylands_group_waypoint_csv)"

usage() {
  cat <<EOF
Usage:
  ./run.sh nav2_route_sweep [world] [routes:=all|route_a,route_b] [args...]

Examples:
  ./run.sh nav2_route_sweep routes:=rotundan
  ./run.sh nav2_route_sweep routes:=rotundan,parkinglot_west
  ./run.sh nav2_route_sweep sweep:=routes
  ./run.sh nav2_route_sweep routes:=strip gui:=false
  ./run.sh nav2_route_sweep dry_run:=true

What it does:
  In sweep:=waypoints mode, start tmux_1to1 at each waypoint in a route,
  give Nav2 the remaining sliced route from that waypoint onward, wait for
  completion/failure/timeout, stop the stack, then teleport to the next waypoint.

  In sweep:=routes mode, run each route once from its first waypoint.

Options:
  routes:=all|a,b             Route YAML stems, e.g. rotundan,strip
  exclude_routes:=a,b          Routes skipped when routes:=all, default art/playground routes
  sweep:=waypoints|routes     Default waypoints
  mode:=follow|yolo           tmux_1to1 mode, default follow
  lidar:=2d|3d                Nav2/localization lidar mode, default 3d
  session_prefix:=name        tmux session prefix
  start_timeout_s:=seconds    Time to wait for ugv_nav2_driver to start
  route_timeout_s:=seconds    Max time per route after driver starts, 0/off disables
  between_s:=seconds          Pause between routes
  stop_before_each:=true|false
  stop_after_each:=true|false
  check_uav_follow:=true|false
  uav_check_require_command:=true|false  Require UAV cmd topic before route, default false
  uav_check_timeout_s:=seconds
  uav_check_hold_ugv_s:=seconds  Optional UGV start delay while checking
  skip_remaining_on_complete:=true|false
  chain_route_starts:=true|false  In sweep:=routes, start each route at the previous route's last waypoint
  fallback_waypoints_on_route_failure:=true|false  In sweep:=routes, try sliced waypoint starts if the full route fails
  dry_run:=true|false

Any other tmux_1to1 argument is forwarded, such as gui:=false, rtf:=1.0,
map:=/path/map.yaml, delay_s:=20, layout:=windows, or omnet:=false.
EOF
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "$value"
}

sanitize_name() {
  printf '%s' "$1" | tr -c 'A-Za-z0-9_.-' '_'
}

print_cmd() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

arg_value_or_default() {
  local prefix="$1"
  local default_value="$2"
  local arg=""
  shift 2
  for arg in "$@"; do
    if [[ "$arg" == "$prefix"* ]]; then
      printf '%s\n' "${arg#"$prefix"}"
      return 0
    fi
  done
  printf '%s\n' "$default_value"
}

result_context() {
  local -a lidar_args=("$@")
  local pc2ls_min pc2ls_max pc2ls_range_min pc2ls_range_max
  pc2ls_min="$(arg_value_or_default "pc2ls_min_height:=" "default" "${EXTRA_TMUX_ARGS[@]}" "${lidar_args[@]}")"
  pc2ls_max="$(arg_value_or_default "pc2ls_max_height:=" "default" "${EXTRA_TMUX_ARGS[@]}" "${lidar_args[@]}")"
  pc2ls_range_min="$(arg_value_or_default "pc2ls_range_min:=" "default" "${EXTRA_TMUX_ARGS[@]}" "${lidar_args[@]}")"
  pc2ls_range_max="$(arg_value_or_default "pc2ls_range_max:=" "default" "${EXTRA_TMUX_ARGS[@]}" "${lidar_args[@]}")"
  printf 'lidar=%s pc2ls_min_height=%s pc2ls_max_height=%s pc2ls_range_min=%s pc2ls_range_max=%s' \
    "$LIDAR" "$pc2ls_min" "$pc2ls_max" "$pc2ls_range_min" "$pc2ls_range_max"
}

discover_routes() {
  python3 - "$WAYPOINT_CONFIG_DIR" "$GROUP_CSV" <<'PY'
import csv
import os
import sys

waypoint_config_dir, group_csv = sys.argv[1:]
seen = set()

def emit(route: str) -> None:
    route = str(route).strip()
    if not route or route in {"none", "rviz"} or route in seen:
        return
    if not os.path.exists(os.path.join(waypoint_config_dir, f"baylands_waypoints_{route}.yaml")):
        return
    seen.add(route)
    print(route)

try:
    with open(group_csv, "r", encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            emit(row.get("group", ""))
except FileNotFoundError:
    pass

PY
}

split_routes() {
  local raw="$1"
  local part route
  if [ "$raw" = "all" ]; then
    discover_routes
    return
  fi
  IFS=',' read -ra parts <<< "$raw"
  for part in "${parts[@]}"; do
    route="$(trim "$part")"
    [ -n "$route" ] && printf '%s\n' "$route"
  done
}

route_is_excluded() {
  local route="$1"
  local part excluded
  IFS=',' read -ra parts <<< "$EXCLUDE_ROUTES_RAW"
  for part in "${parts[@]}"; do
    excluded="$(trim "$part")"
    if [ "$route" = "$excluded" ]; then
      return 0
    fi
  done
  return 1
}

first_waypoint_for_route() {
  local route="$1"
  python3 - "$route" "$WAYPOINT_CONFIG_DIR" "$GROUP_CSV" <<'PY'
import csv
import os
import sys

route, waypoint_config_dir, group_csv = sys.argv[1:]

def emit(value: str) -> None:
    value = str(value).strip()
    if value:
        print(value)
        raise SystemExit(0)

try:
    import yaml
except Exception:
    yaml = None

route_yaml = os.path.join(waypoint_config_dir, f"baylands_waypoints_{route}.yaml")
if yaml is not None and os.path.exists(route_yaml):
    with open(route_yaml, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    for waypoint in data.get("waypoints") or []:
        if isinstance(waypoint, dict):
            emit(waypoint.get("name", ""))

rows = []
try:
    with open(group_csv, "r", encoding="utf-8", newline="") as handle:
        rows.extend(csv.DictReader(handle))
except FileNotFoundError:
    pass

for row in rows:
    if str(row.get("group", "")).strip() == route:
        emit(row.get("place", ""))

preferred = f"{route}_0"
for row in rows:
    if str(row.get("place", "")).strip() == preferred:
        emit(preferred)

prefix = f"{route}_"
for row in rows:
    name = str(row.get("place", "")).strip()
    if name.startswith(prefix):
        emit(name)

raise SystemExit(f"No first waypoint found for route '{route}'")
PY
}

route_yaml_path() {
  baylands_route_yaml_path "$1"
}

route_waypoint_names() {
  local route="$1"
  local route_yaml
  route_yaml="$(route_yaml_path "$route")"
  python3 - "$route" "$route_yaml" <<'PY'
import sys

route, route_yaml = sys.argv[1:]
try:
    import yaml
except Exception as exc:
    raise SystemExit(f"PyYAML is required for waypoint sweep mode: {exc}")

with open(route_yaml, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}

for waypoint in data.get("waypoints") or []:
    if not isinstance(waypoint, dict):
        continue
    name = str(waypoint.get("name", "")).strip()
    if name:
        print(name)
PY
}

last_waypoint_for_route() {
  local route="$1"
  local last=""
  while IFS= read -r waypoint_name; do
    [ -n "$waypoint_name" ] && last="$waypoint_name"
  done < <(route_waypoint_names "$route")

  if [ -n "$last" ]; then
    printf '%s\n' "$last"
    return 0
  fi

  echo "No last waypoint found for route '$route'" >&2
  return 1
}

write_route_slice() {
  local route="$1"
  local start_index="$2"
  local out_path="$3"
  local route_yaml
  route_yaml="$(route_yaml_path "$route")"
  python3 - "$route_yaml" "$start_index" "$out_path" <<'PY'
import os
import sys

route_yaml, start_index, out_path = sys.argv[1:]
start_index = int(start_index)

import yaml

with open(route_yaml, "r", encoding="utf-8") as handle:
    data = yaml.safe_load(handle) or {}

waypoints = data.get("waypoints") or []
if start_index < 0 or start_index >= len(waypoints):
    raise SystemExit(f"Waypoint slice index {start_index} is out of range for {route_yaml}")

data["waypoints"] = waypoints[start_index:]
os.makedirs(os.path.dirname(out_path), exist_ok=True)
with open(out_path, "w", encoding="utf-8") as handle:
    yaml.safe_dump(data, handle, sort_keys=False)
PY
}

append_waypoint_jobs() {
  local route="$1"
  local label_prefix="$2"
  local route_file waypoint_name slice_path index
  local -a waypoint_names

  route_file="$(route_yaml_path "$route")"
  if [ ! -f "$route_file" ]; then
    echo "Skipping route '$route': missing route file $route_file" >&2
    return 1
  fi

  mapfile -t waypoint_names < <(route_waypoint_names "$route")
  if [ "${#waypoint_names[@]}" -eq 0 ]; then
    echo "Skipping route '$route': no named waypoints in $route_file" >&2
    return 1
  fi

  for index in "${!waypoint_names[@]}"; do
    waypoint_name="${waypoint_names[$index]}"
    slice_path="$RUN_DIR/slices/$(sanitize_name "${label_prefix}${route}")_${index}_$(sanitize_name "$waypoint_name").yaml"
    if [ "$DRY_RUN" != true ]; then
      write_route_slice "$route" "$index" "$slice_path"
    fi
    JOB_START_WAYPOINTS+=("$waypoint_name")
    JOB_NAV2_GOALS+=("$slice_path")
    JOB_LABELS+=("${label_prefix}${route}_${index}_${waypoint_name}")
  done
}

session_state_file() {
  local session_safe
  session_safe="$(sanitize_name "$1")"
  printf '%s/%s.env\n' "$TMUX_STATE_DIR" "$session_safe"
}

capture_follow_pane() {
  local session="$1"
  local out_file="$2"
  local state_file pane_id
  state_file="$(session_state_file "$session")"
  pane_id=""
  if [ -f "$state_file" ]; then
    pane_id="$(
      # Keep the tmux state file variables from leaking into this sweep script.
      # shellcheck disable=SC1090
      source "$state_file"
      printf '%s' "${FOLLOW_PANE_ID:-}"
    )"
  fi
  if [ -n "$pane_id" ]; then
    tmux capture-pane -p -S -20000 -t "$pane_id" > "$out_file" 2>/dev/null || true
  else
    tmux capture-pane -p -S -20000 -t "$session" > "$out_file" 2>/dev/null || true
  fi
}

route_driver_running() {
  pgrep -f '[u]gv_nav2_driver' >/dev/null 2>&1
}

route_timeout_disabled() {
  case "${1:-}" in
    0|0.0|false|False|FALSE|off|Off|OFF|none|None|NONE|disabled|Disabled|DISABLED)
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

ros_topic_echo_once() {
  local topic="$1"
  local out_file="$2"
  local timeout_s="${3:-$UAV_CHECK_TOPIC_TIMEOUT_S}"
  timeout "${timeout_s}s" bash -lc "
set +u
source /opt/ros/jazzy/setup.bash >/dev/null 2>&1
source '$WS_ROOT/install/setup.bash' >/dev/null 2>&1
export ROS_DOMAIN_ID='${ROS_DOMAIN_ID:-3}'
export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}'
set -u
ros2 topic echo --no-daemon --once '$topic'
" > "$out_file" 2>&1
}

pose_z_is_valid() {
  local pose_file="$1"
  local min_z="$2"
  python3 - "$pose_file" "$min_z" <<'PY'
import math
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1])
min_z = float(sys.argv[2])
docs = [doc for doc in yaml.safe_load_all(path.read_text(encoding="utf-8")) if isinstance(doc, dict)]

for doc in reversed(docs):
    pose = doc.get("pose") or {}
    if isinstance(pose, dict) and "pose" in pose:
        pose = pose.get("pose") or {}
    position = pose.get("position") or {}
    try:
        z = float(position["z"])
    except Exception:
        continue
    if math.isfinite(z) and z >= min_z:
        raise SystemExit(0)

raise SystemExit(1)
PY
}

check_uav_follow_ready() {
  local session="$1"
  local label="$2"
  local check_log="$3"
  local deadline=$((SECONDS + UAV_CHECK_TIMEOUT_S))
  local pose_tmp cmd_tmp pane_tmp
  pose_tmp="$(mktemp)"
  cmd_tmp="$(mktemp)"
  pane_tmp="$(mktemp)"

  {
    echo "[$label] checking UAV follow health"
    echo "[$label] require pose=/$(printf '%s' "$UAV_NAME")/pose z>=${UAV_CHECK_MIN_Z}"
    if [ "$UAV_CHECK_REQUIRE_COMMAND" = true ]; then
      echo "[$label] require command=/$(printf '%s' "$UAV_NAME")/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
    else
      echo "[$label] command topic is optional; pose/spawn health is enough"
    fi
  } > "$check_log"

  while (( SECONDS < deadline )); do
    capture_follow_pane "$session" "$pane_tmp"
    if grep -Eq 'process has died.*(follow_uav|uav_simulator|simulator)|\[(follow_uav|follow_uav_odom|simulator)-[0-9]+\].*Traceback' "$pane_tmp" 2>/dev/null; then
      cat "$pane_tmp" >> "$check_log"
      echo "[$label] UAV follow health failed: follow/simulator process died" | tee -a "$check_log" >&2
      rm -f "$pose_tmp" "$cmd_tmp" "$pane_tmp"
      return 1
    fi

    if ! ros_topic_echo_once "/$UAV_NAME/pose" "$pose_tmp" "$UAV_CHECK_TOPIC_TIMEOUT_S"; then
      echo "[$label] waiting for /$UAV_NAME/pose" >> "$check_log"
      sleep 2
      continue
    fi

    if ! pose_z_is_valid "$pose_tmp" "$UAV_CHECK_MIN_Z"; then
      echo "[$label] /$UAV_NAME/pose exists but z is below ${UAV_CHECK_MIN_Z}" >> "$check_log"
      sleep 2
      continue
    fi

    if [ "$UAV_CHECK_REQUIRE_COMMAND" = true ]; then
      if ! ros_topic_echo_once "/$UAV_NAME/psdk_ros2/flight_control_setpoint_ENUposition_yaw" "$cmd_tmp" "$UAV_CHECK_TOPIC_TIMEOUT_S"; then
        echo "[$label] waiting for UAV command topic" >> "$check_log"
        sleep 2
        continue
      fi
    elif ros_topic_echo_once "/$UAV_NAME/psdk_ros2/flight_control_setpoint_ENUposition_yaw" "$cmd_tmp" "$UAV_CHECK_TOPIC_TIMEOUT_S"; then
      echo "[$label] optional UAV command topic already visible" >> "$check_log"
    else
      echo "[$label] optional UAV command topic not visible yet; continuing because pose is healthy" >> "$check_log"
    fi

    echo "[$label] UAV follow health OK" | tee -a "$check_log"
    rm -f "$pose_tmp" "$cmd_tmp" "$pane_tmp"
    return 0
  done

  capture_follow_pane "$session" "$pane_tmp"
  cat "$pane_tmp" >> "$check_log"
  echo "[$label] UAV follow health timed out after ${UAV_CHECK_TIMEOUT_S}s" | tee -a "$check_log" >&2
  rm -f "$pose_tmp" "$cmd_tmp" "$pane_tmp"
  return 1
}

stop_session() {
  local session="$1"
  local cmd=(./stop.sh tmux_1to1 "$WORLD" "session:=$session" "group_grace_s:=$STOP_GROUP_GRACE_S" "final_grace_s:=$STOP_FINAL_GRACE_S")
  if [ "$DRY_RUN" = true ]; then
    cmd+=("dry_run:=true")
    echo "[dry-run] stop:"
    print_cmd "${cmd[@]}"
    return 0
  fi
  "${cmd[@]}" || true
}

classify_route_log() {
  local log_file="$1"
  if grep -q "UGV Nav2 motion complete" "$log_file" 2>/dev/null; then
    printf 'completed'
  elif grep -Eq "Nav2 goal failed|Goal failed|Aborting handle|Resulting plan has 0 poses|Timed out" "$log_file" 2>/dev/null; then
    printf 'failed'
  else
    printf 'finished'
  fi
}

wait_for_route_driver_start() {
  local session="$1"
  local route="$2"
  local tmp_log="$3"
  local deadline=$((SECONDS + START_TIMEOUT_S))
  while (( SECONDS < deadline )); do
    if route_driver_running; then
      return 0
    fi
    capture_follow_pane "$session" "$tmp_log"
    if grep -Eq "UGV Nav2 motion|Generated .* motion waypoints|UGV Nav2 motion complete|Nav2 goal failed" "$tmp_log" 2>/dev/null; then
      return 0
    fi
    sleep 3
  done
  echo "Route '$route' did not start ugv_nav2_driver within ${START_TIMEOUT_S}s" >&2
  return 1
}

wait_for_route_done() {
  local session="$1"
  local route="$2"
  local route_log="$3"

  if route_timeout_disabled "$ROUTE_TIMEOUT_S"; then
    while route_driver_running; do
      sleep 5
    done
    capture_follow_pane "$session" "$route_log"
    return 0
  fi

  local deadline=$((SECONDS + ROUTE_TIMEOUT_S))

  while route_driver_running; do
    if (( SECONDS >= deadline )); then
      capture_follow_pane "$session" "$route_log"
      echo "Route '$route' timed out after ${ROUTE_TIMEOUT_S}s" >&2
      return 124
    fi
    sleep 5
  done

  capture_follow_pane "$session" "$route_log"
  return 0
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
    routes:=*|nav2_goals:=*)
      ROUTES_RAW="${arg#*:=}"
      ;;
    exclude_routes:=*)
      EXCLUDE_ROUTES_RAW="${arg#exclude_routes:=}"
      ;;
    sweep:=*|sweep_mode:=*)
      SWEEP_MODE="${arg#*:=}"
      ;;
    mode:=*|stack:=*)
      MODE="${arg#*:=}"
      ;;
    lidar:=2d|scan_sensor:=2d)
      LIDAR="2d"
      ;;
    lidar:=3d|scan_sensor:=3d)
      LIDAR="3d"
      ;;
    session_prefix:=*)
      SESSION_PREFIX="${arg#session_prefix:=}"
      ;;
    tmux_attach:=*|attach:=*)
      TMUX_ATTACH="${arg#*:=}"
      ;;
    dry_run:=*)
      DRY_RUN="${arg#dry_run:=}"
      ;;
    start_timeout_s:=*)
      START_TIMEOUT_S="${arg#start_timeout_s:=}"
      ;;
    route_timeout_s:=*)
      ROUTE_TIMEOUT_S="${arg#route_timeout_s:=}"
      ;;
    between_s:=*)
      BETWEEN_S="${arg#between_s:=}"
      ;;
    check_uav_follow:=*)
      CHECK_UAV_FOLLOW="${arg#check_uav_follow:=}"
      ;;
    uav_name:=*)
      UAV_NAME="${arg#uav_name:=}"
      EXTRA_TMUX_ARGS+=("$arg")
      ;;
    uav_check_timeout_s:=*)
      UAV_CHECK_TIMEOUT_S="${arg#uav_check_timeout_s:=}"
      ;;
    uav_check_topic_timeout_s:=*)
      UAV_CHECK_TOPIC_TIMEOUT_S="${arg#uav_check_topic_timeout_s:=}"
      ;;
    uav_check_min_z:=*)
      UAV_CHECK_MIN_Z="${arg#uav_check_min_z:=}"
      ;;
    uav_check_hold_ugv_s:=*)
      UAV_CHECK_HOLD_UGV_S="${arg#uav_check_hold_ugv_s:=}"
      ;;
    uav_check_require_command:=*)
      UAV_CHECK_REQUIRE_COMMAND="${arg#uav_check_require_command:=}"
      ;;
    skip_remaining_on_complete:=*)
      SKIP_REMAINING_ON_COMPLETE="${arg#skip_remaining_on_complete:=}"
      ;;
    chain_route_starts:=*|chain_routes:=*)
      CHAIN_ROUTE_STARTS="${arg#*:=}"
      ;;
    fallback_waypoints_on_route_failure:=*|fallback_waypoints:=*)
      FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE="${arg#*:=}"
      ;;
    stop_before_each:=*)
      STOP_BEFORE_EACH="${arg#stop_before_each:=}"
      ;;
    stop_after_each:=*|stop_after:=*)
      STOP_AFTER_EACH="${arg#*:=}"
      ;;
    stop_group_grace_s:=*)
      STOP_GROUP_GRACE_S="${arg#stop_group_grace_s:=}"
      ;;
    stop_final_grace_s:=*)
      STOP_FINAL_GRACE_S="${arg#stop_final_grace_s:=}"
      ;;
    waypoint_config_dir:=*)
      WAYPOINT_CONFIG_DIR="${arg#waypoint_config_dir:=}"
      ;;
    group_csv:=*)
      GROUP_CSV="${arg#group_csv:=}"
      ;;
    waypoint_csv:=*)
      echo "waypoint_csv:= is deprecated; use group_csv:= with waypoints_baylands_groups.csv as the single source." >&2
      exit 2
      ;;
    session:=*)
      SESSION_PREFIX="${arg#session:=}"
      ;;
    ugv_start_delay_s:=*)
      HAVE_UGV_START_DELAY_ARG="true"
      EXTRA_TMUX_ARGS+=("$arg")
      ;;
    *)
      EXTRA_TMUX_ARGS+=("$arg")
      ;;
  esac
done

if [[ "$WORLD" == baylands* ]]; then
  baylands_sync_waypoints "$DRY_RUN"
fi

mapfile -t ROUTES < <(split_routes "$ROUTES_RAW")
FILTERED_ROUTES=()
for route in "${ROUTES[@]}"; do
  if route_is_excluded "$route"; then
    continue
  fi
  FILTERED_ROUTES+=("$route")
done
ROUTES=("${FILTERED_ROUTES[@]}")
if [ "${#ROUTES[@]}" -eq 0 ]; then
  echo "No routes selected." >&2
  exit 2
fi

RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$STATE_DIR/$RUN_ID"
if [ "$DRY_RUN" != true ]; then
  mkdir -p "$RUN_DIR"
fi

RESULTS=()
CURRENT_SESSION=""
CURRENT_JOB_LABEL=""
CURRENT_START_WAYPOINT=""
CURRENT_RUN_CONTEXT=""

write_summary_file() {
  local path="$1"
  local state="$2"
  {
    printf 'state: %s\n' "$state"
    printf 'run_id: %s\n' "$RUN_ID"
    printf 'world: %s\n' "$WORLD"
    printf 'mode: %s\n' "$MODE"
    printf 'lidar: %s\n' "$LIDAR"
    printf 'sweep: %s\n' "$SWEEP_MODE"
    printf 'chain_route_starts: %s\n' "$CHAIN_ROUTE_STARTS"
    printf 'fallback_waypoints_on_route_failure: %s\n' "$FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE"
    printf 'exclude_routes: %s\n' "$EXCLUDE_ROUTES_RAW"
    printf 'routes: %s\n' "${ROUTES[*]}"
    if [ -n "$CURRENT_SESSION" ]; then
      printf 'active_session: %s\n' "$CURRENT_SESSION"
      printf 'active_job: %s\n' "$CURRENT_JOB_LABEL"
      printf 'active_start_waypoint: %s\n' "$CURRENT_START_WAYPOINT"
      printf 'active_context: %s\n' "$CURRENT_RUN_CONTEXT"
    fi
    printf '\nresults:\n'
    if [ "${#RESULTS[@]}" -gt 0 ]; then
      printf '%s\n' "${RESULTS[@]}"
    fi
  } > "$path"
}

cleanup_on_interrupt() {
  echo
  echo "Interrupted."
  if [ "$DRY_RUN" != true ]; then
    partial_summary="$RUN_DIR/summary.partial.txt"
    write_summary_file "$partial_summary" "interrupted"
    echo "Saved partial summary: $partial_summary"
  fi
  if [ -n "$CURRENT_SESSION" ]; then
    stop_session "$CURRENT_SESSION"
  fi
  exit 130
}
trap cleanup_on_interrupt INT TERM

echo "Nav2 route sweep"
echo "  world: $WORLD"
echo "  mode: $MODE"
echo "  lidar: $LIDAR"
echo "  sweep: $SWEEP_MODE"
if route_timeout_disabled "$ROUTE_TIMEOUT_S"; then
  echo "  route timeout: disabled"
else
  echo "  route timeout: ${ROUTE_TIMEOUT_S}s"
fi
echo "  uav follow check: $CHECK_UAV_FOLLOW"
echo "  uav command required: $UAV_CHECK_REQUIRE_COMMAND"
echo "  skip remaining on complete: $SKIP_REMAINING_ON_COMPLETE"
echo "  chain route starts: $CHAIN_ROUTE_STARTS"
echo "  fallback waypoints on route failure: $FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE"
echo "  exclude routes: $EXCLUDE_ROUTES_RAW"
echo "  routes: ${ROUTES[*]}"
if [ "$DRY_RUN" != true ]; then
  echo "  logs: $RUN_DIR"
fi

PREVIOUS_ROUTE_LAST_WAYPOINT=""
for route in "${ROUTES[@]}"; do
  JOB_START_WAYPOINTS=()
  JOB_NAV2_GOALS=()
  JOB_LABELS=()
  route_last_waypoint=""
  route_completed="false"

  case "$SWEEP_MODE" in
    routes|route)
      if [ "$CHAIN_ROUTE_STARTS" = true ] && [ -n "$PREVIOUS_ROUTE_LAST_WAYPOINT" ]; then
        first_waypoint="$PREVIOUS_ROUTE_LAST_WAYPOINT"
      else
        first_waypoint="$(first_waypoint_for_route "$route")" || {
          echo "Skipping route '$route': could not resolve first waypoint" >&2
          RESULTS+=("$route skipped no_first_waypoint")
          continue
        }
      fi
      route_last_waypoint="$(last_waypoint_for_route "$route" 2>/dev/null || true)"
      JOB_START_WAYPOINTS+=("$first_waypoint")
      JOB_NAV2_GOALS+=("$route")
      JOB_LABELS+=("$route")
      ;;
    waypoints|waypoint)
      if ! append_waypoint_jobs "$route" ""; then
        RESULTS+=("$route skipped no_route_file")
        continue
      fi
      ;;
    *)
      echo "Invalid sweep mode: $SWEEP_MODE" >&2
      echo "Use sweep:=waypoints or sweep:=routes" >&2
      exit 2
      ;;
  esac

  job_index=0
  while [ "$job_index" -lt "${#JOB_START_WAYPOINTS[@]}" ]; do
    first_waypoint="${JOB_START_WAYPOINTS[$job_index]}"
    nav2_goals_arg="${JOB_NAV2_GOALS[$job_index]}"
    job_label="${JOB_LABELS[$job_index]}"
    session="${SESSION_PREFIX}-$(sanitize_name "$job_label")"
    CURRENT_SESSION="$session"
    route_log="$RUN_DIR/$(sanitize_name "$job_label").follow.log"
    tmp_log="${route_log}.startup"
    mapfile -t route_lidar_args < <(route_lidar_preset_args "$route" "$LIDAR" "${EXTRA_TMUX_ARGS[@]}")
    run_context="$(result_context "${route_lidar_args[@]}")"
    CURRENT_JOB_LABEL="$job_label"
    CURRENT_START_WAYPOINT="$first_waypoint"
    CURRENT_RUN_CONTEXT="$run_context"

    echo
    echo "=== Route: $route (start waypoint: $first_waypoint, nav2_goals: $nav2_goals_arg, session: $session) ==="

    if [ "$STOP_BEFORE_EACH" = true ]; then
      stop_session "$session"
    fi

    run_cmd=(
      ./run.sh tmux_1to1 "$WORLD"
      "mode:=$MODE"
      "session:=$session"
      "waypoint:=$first_waypoint"
      "nav2_goals:=$nav2_goals_arg"
      "lidar:=$LIDAR"
      "tmux_attach:=$TMUX_ATTACH"
      "${EXTRA_TMUX_ARGS[@]}"
    )
    if [ "${#route_lidar_args[@]}" -gt 0 ]; then
      echo "Route '$route' lidar preset: ${route_lidar_args[*]}"
      run_cmd+=("${route_lidar_args[@]}")
    fi
    if [ "$CHECK_UAV_FOLLOW" = true ] && [ "$HAVE_UGV_START_DELAY_ARG" != "true" ] && [ "$UAV_CHECK_HOLD_UGV_S" != "0" ] && [ "$UAV_CHECK_HOLD_UGV_S" != "0.0" ]; then
      run_cmd+=("ugv_start_delay_s:=$UAV_CHECK_HOLD_UGV_S")
    fi
    if [ "$DRY_RUN" = true ]; then
      run_cmd+=("dry_run:=true")
      echo "[dry-run] start:"
      print_cmd "${run_cmd[@]}"
      RESULTS+=("$job_label dry_run $first_waypoint $run_context")
      if [ "$SWEEP_MODE" = "routes" ] || [ "$SWEEP_MODE" = "route" ]; then
        route_completed="true"
      fi
      job_index=$((job_index + 1))
      continue
    fi

    if ! "${run_cmd[@]}"; then
      echo "Route '$job_label' failed to launch." >&2
      RESULTS+=("$job_label launch_failed $first_waypoint $run_context")
      stop_session "$session"
      CURRENT_SESSION=""
      CURRENT_JOB_LABEL=""
      CURRENT_START_WAYPOINT=""
      CURRENT_RUN_CONTEXT=""
      job_index=$((job_index + 1))
      continue
    fi

    if [ "$CHECK_UAV_FOLLOW" = true ]; then
      uav_check_log="$RUN_DIR/$(sanitize_name "$job_label").uav_check.log"
      if ! check_uav_follow_ready "$session" "$job_label" "$uav_check_log"; then
        capture_follow_pane "$session" "$route_log"
        status="uav_check_failed"
        echo "Route '$job_label' result: $status"
        RESULTS+=("$job_label $status $first_waypoint $run_context")
        if [ "$STOP_AFTER_EACH" = true ]; then
          stop_session "$session"
        fi
        CURRENT_SESSION=""
        CURRENT_JOB_LABEL=""
        CURRENT_START_WAYPOINT=""
        CURRENT_RUN_CONTEXT=""
        if [ "$BETWEEN_S" != "0" ] && [ "$BETWEEN_S" != "0.0" ]; then
          sleep "$BETWEEN_S"
        fi
        job_index=$((job_index + 1))
        continue
      fi
    fi

    if wait_for_route_driver_start "$session" "$job_label" "$tmp_log"; then
      if wait_for_route_done "$session" "$job_label" "$route_log"; then
        status="$(classify_route_log "$route_log")"
      else
        status="timeout"
      fi
    else
      capture_follow_pane "$session" "$route_log"
      status="start_timeout"
    fi

    echo "Route '$job_label' result: $status"
    RESULTS+=("$job_label $status $first_waypoint $run_context")
    if [ "$status" = "completed" ]; then
      route_completed="true"
    fi

    if [ "$STOP_AFTER_EACH" = true ]; then
      stop_session "$session"
    fi
    CURRENT_SESSION=""
    CURRENT_JOB_LABEL=""
    CURRENT_START_WAYPOINT=""
    CURRENT_RUN_CONTEXT=""

    if [ "$SWEEP_MODE" = "routes" ] || [ "$SWEEP_MODE" = "route" ]; then
      if [ "$FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE" = true ] && [ "$CHAIN_ROUTE_STARTS" != true ] && [ "$job_index" -eq 0 ] && [ "$status" != "completed" ]; then
        echo "Route '$route' failed as a full route; falling back to sliced waypoint starts."
        append_waypoint_jobs "$route" "fallback_" || true
      fi
    fi

    if [ "$BETWEEN_S" != "0" ] && [ "$BETWEEN_S" != "0.0" ]; then
      sleep "$BETWEEN_S"
    fi

    if [ "$SWEEP_MODE" != "routes" ] && [ "$SWEEP_MODE" != "route" ] && [ "$SKIP_REMAINING_ON_COMPLETE" = true ] && [ "$status" = "completed" ]; then
      echo "Route '$route' completed from '$first_waypoint'; skipping remaining start waypoints for this route."
      break
    fi
    if [ "$SWEEP_MODE" = "routes" ] || [ "$SWEEP_MODE" = "route" ]; then
      if [ "$FALLBACK_WAYPOINTS_ON_ROUTE_FAILURE" = true ] && [[ "$job_label" == fallback_* ]] && [ "$SKIP_REMAINING_ON_COMPLETE" = true ] && [ "$status" = "completed" ]; then
        echo "Route '$route' completed from fallback '$first_waypoint'; skipping remaining fallback waypoints for this route."
        break
      fi
    fi
    job_index=$((job_index + 1))
  done

  if [ "$SWEEP_MODE" = "routes" ] || [ "$SWEEP_MODE" = "route" ]; then
    if [ "$route_completed" = "true" ] && [ -n "$route_last_waypoint" ]; then
      PREVIOUS_ROUTE_LAST_WAYPOINT="$route_last_waypoint"
    else
      PREVIOUS_ROUTE_LAST_WAYPOINT=""
    fi
  fi
done

CURRENT_SESSION=""

echo
echo "Route sweep summary:"
for row in "${RESULTS[@]}"; do
  printf '  %s\n' "$row"
done

if [ "$DRY_RUN" != true ]; then
  write_summary_file "$RUN_DIR/summary.txt" "finished"
  echo "Saved summary: $RUN_DIR/summary.txt"
fi
