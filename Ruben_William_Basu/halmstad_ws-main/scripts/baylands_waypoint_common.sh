#!/usr/bin/env bash

BAYLANDS_WAYPOINT_COMMON_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

baylands_group_waypoint_csv() {
  printf '%s/maps/waypoints_baylands_groups.csv\n' "$WS_ROOT"
}

baylands_waypoint_config_dir() {
  printf '%s/src/lrs_halmstad/config/baylands_waypoints\n' "$WS_ROOT"
}

baylands_config_dir() {
  printf '%s/src/lrs_halmstad/config\n' "$WS_ROOT"
}

baylands_all_waypoints_yaml() {
  printf '%s/baylands_waypoints.yaml\n' "$(baylands_config_dir)"
}

baylands_sync_waypoints() {
  local dry_run="${1:-false}"
  local cmd=(
    bash "$BAYLANDS_WAYPOINT_COMMON_DIR/map-making/run_sync_waypoints_csv.sh"
    "input:=$(baylands_group_waypoint_csv)"
    "route_output:=$(baylands_all_waypoints_yaml)"
    "rviz_dir:=$(baylands_waypoint_config_dir)"
    "dry_run:=$dry_run"
  )
  if [ "$dry_run" = "true" ]; then
    "${cmd[@]}" >/dev/null
    echo "[baylands_waypoints] dry-run: waypoint YAML sync would use $(baylands_group_waypoint_csv)"
    return 0
  fi
  "${cmd[@]}" >/dev/null
  echo "[baylands_waypoints] synced generated waypoint YAMLs from $(baylands_group_waypoint_csv)"
}

baylands_route_yaml_path() {
  local route="$1"
  local ws_name
  ws_name="$(basename "$WS_ROOT")"
  if [[ "$route" == "$ws_name/"* ]]; then
    route="${route#"$ws_name"/}"
  fi

  local name
  name="$(basename "$route")"
  if [[ "$name" == *.yaml ]]; then
    if [[ "$route" = /* ]]; then
      printf '%s\n' "$route"
    elif [ -f "$route" ]; then
      readlink -f "$route"
    elif [ -f "$WS_ROOT/$route" ]; then
      printf '%s/%s\n' "$WS_ROOT" "$route"
    elif [ -f "$(baylands_config_dir)/$route" ]; then
      printf '%s/%s\n' "$(baylands_config_dir)" "$route"
    elif [[ "$route" == */* ]]; then
      printf '%s/%s\n' "$WS_ROOT" "$route"
    else
      printf '%s/%s\n' "$(baylands_waypoint_config_dir)" "$route"
    fi
    return 0
  fi
  if [ "$name" = "baylands_waypoints" ]; then
    baylands_all_waypoints_yaml
    return 0
  fi
  if [ -f "$(baylands_config_dir)/$name.yaml" ]; then
    printf '%s/%s.yaml\n' "$(baylands_config_dir)" "$name"
    return 0
  fi
  printf '%s/baylands_waypoints_%s.yaml\n' "$(baylands_waypoint_config_dir)" "$name"
}
