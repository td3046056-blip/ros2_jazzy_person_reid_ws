#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

source .venv/bin/activate
export COLCON_TRACE="${COLCON_TRACE:-0}"
set +u
source install/setup.bash
set -u

python - <<'PY'
import sys
print('python:', sys.executable)
import torch
print('torch:', torch.__version__)
import cv2
print('cv2:', cv2.__version__)
import rclpy
print('rclpy: ok')
from person_reid_tracker.yolo_deepsort_pipeline import YoloDeepSortPipeline
print('person_reid_tracker import: ok')
PY
