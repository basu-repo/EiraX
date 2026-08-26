#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
SIM_BIN="$SCRIPT_DIR/UAV_UGV"
INI_FILE="$SCRIPT_DIR/omnetpp.ini"
OMNET_SETENV="${OMNETPP_SETENV:-$WORKSPACE_DIR/omnetpp-6.2.0/setenv}"
DEFAULT_RESULT_DIR="$SCRIPT_DIR/results"

usage() {
  cat <<'EOF'
Usage: ./run_qtenv.sh [wifi|5g|lora|lora-duplex|lora-sweep] [extra OMNeT args...]

Examples:
  ./run_qtenv.sh
  ./run_qtenv.sh wifi
  ./run_qtenv.sh 5g
  ./run_qtenv.sh lora
  ./run_qtenv.sh lora-duplex
  ./run_qtenv.sh lora-sweep
  ./run_qtenv.sh wifi --result-dir=/tmp/omnet-test
EOF
}

if [[ $# -gt 0 && ( "$1" == "-h" || "$1" == "--help" || "$1" == "help" ) ]]; then
  usage
  exit 0
fi

NETWORK_TYPE="wifi"
if [[ $# -gt 0 && "${1#-}" == "$1" ]]; then
  NETWORK_TYPE="${1,,}"
  shift
fi

case "$NETWORK_TYPE" in
  wifi)
    CONFIG_NAME="Communication-GazeboBridge-WiFi"
    NED_PATH="$SCRIPT_DIR/src:$SCRIPT_DIR/../inet4.5/src"
    ;;
  5g)
    CONFIG_NAME="Communication-GazeboBridge-5G"
    NED_PATH="$SCRIPT_DIR/src:$SCRIPT_DIR/../inet4.5/src"
    ;;
  lora)
    CONFIG_NAME="Communication-GazeboBridge-LoRa-Simplex"
    NED_PATH="$SCRIPT_DIR/src:$SCRIPT_DIR/../inet4.5/src:$SCRIPT_DIR/../flora/src"
    ;;
  lora-duplex)
    CONFIG_NAME="Communication-GazeboBridge-LoRa-Duplex"
    NED_PATH="$SCRIPT_DIR/src:$SCRIPT_DIR/../inet4.5/src:$SCRIPT_DIR/../flora/src"
    ;;
  lora-sweep)
    CONFIG_NAME="Communication-GazeboBridge-LoRa-Sweep"
    NED_PATH="$SCRIPT_DIR/src:$SCRIPT_DIR/../inet4.5/src:$SCRIPT_DIR/../flora/src"
    ;;
  *)
    echo "Unsupported network type: $NETWORK_TYPE" >&2
    usage >&2
    exit 1
    ;;
esac

if [[ ! -f "$OMNET_SETENV" ]]; then
  echo "Missing OMNeT setenv: $OMNET_SETENV" >&2
  echo "Set OMNETPP_SETENV to your OMNeT setenv path if it lives elsewhere." >&2
  exit 1
fi

if [[ ! -x "$SIM_BIN" ]]; then
  echo "Missing executable: $SIM_BIN" >&2
  exit 1
fi

mkdir -p "$DEFAULT_RESULT_DIR"

RESULT_DIR_ARG=( "--result-dir=$DEFAULT_RESULT_DIR" )
for arg in "$@"; do
  if [[ "$arg" == --result-dir=* ]]; then
    RESULT_DIR_ARG=()
    break
  fi
done

# shellcheck source=/dev/null
set +u
source "$OMNET_SETENV"
set -u

exec "$SIM_BIN" \
  -u Qtenv \
  -f "$INI_FILE" \
  -c "$CONFIG_NAME" \
  -n "$NED_PATH" \
  "${RESULT_DIR_ARG[@]}" \
  "$@"
