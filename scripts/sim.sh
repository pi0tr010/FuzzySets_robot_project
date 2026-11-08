#!/usr/bin/env bash
# Inside the container: build the robot program, then start the simulator.
set -euo pipefail
cd "$(dirname "$0")/.."
make -C firmware sim
exec python3 -m sim "$@"
