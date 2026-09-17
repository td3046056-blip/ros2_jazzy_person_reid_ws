#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

CAMERA_SOURCE="${1:-/dev/video2}"
if [[ "$CAMERA_SOURCE" =~ ^[0-9]+$ ]]; then
  CAMERA_SOURCE="/dev/video${CAMERA_SOURCE}"
fi

if [ ! -f .venv/bin/activate ]; then
  echo "[ERROR] .venv not found. Run: ./scripts/setup_jazzy_env.sh" >&2
  exit 1
fi

if [ ! -f install/setup.bash ]; then
  echo "[ERROR] install/setup.bash not found. Run: ./scripts/setup_jazzy_env.sh" >&2
  exit 1
fi

source .venv/bin/activate

# Keep colcon setup quiet and safe even when shell uses strict mode.
export COLCON_TRACE=""
set +u
source install/setup.bash
set -u

echo "[INFO] Using USB camera source: $CAMERA_SOURCE"
ros2 launch person_reid_tracker usb_camera_reid.launch.py camera_source:="$CAMERA_SOURCE" show_window:=true draw_all_tracks:=true
