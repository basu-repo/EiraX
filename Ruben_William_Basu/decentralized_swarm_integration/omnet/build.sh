#!/usr/bin/env bash
set -eo pipefail

OMNET_ROOT="${OMNET_ROOT:-/home/basudeo/omnetpp-6.0.1}"
INET_ROOT="${INET_ROOT:-/home/basudeo/inet}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source "${OMNET_ROOT}/setenv"
set -u
cd "${SCRIPT_DIR}"
opp_makemake -f --deep -O out \
  -KINET_PROJ="${INET_ROOT}" \
  -I. -I"${INET_ROOT}/src" \
  -L"${INET_ROOT}/src" -lINET
make -j2
