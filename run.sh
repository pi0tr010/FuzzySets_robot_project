#!/usr/bin/env bash
# Starts the simulator in Docker. Linux: run directly. Windows: run inside WSL2
# (Windows 11 WSLg provides the X server). Extra arguments go to the simulator,
# e.g. ./run.sh --map maps/my.json --new 40x40
set -euo pipefail
cd "$(dirname "$0")"

export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
export DISPLAY="${DISPLAY:-:0}"

if [ -z "${WSL_DISTRO_NAME:-}" ] && command -v xhost >/dev/null; then
    xhost "+SI:localuser:$(id -un)" >/dev/null
fi

exec docker compose run --rm --build sim ./scripts/sim.sh "$@"
