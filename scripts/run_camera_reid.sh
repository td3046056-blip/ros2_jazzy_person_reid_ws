#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

source .venv/bin/activate

export COLCON_TRACE="${COLCON_TRACE:-0}"
set +u
source install/setup.bash
set -u

ros2 launch person_reid_tracker camera_reid.launch.py "$@"
