#!/usr/bin/env bash
set -euo pipefail

# Run from workspace root: ./scripts/setup_jazzy_env.sh
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_DIR"

sudo apt update
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-opencv \
  ros-jazzy-rqt-image-view \
  v4l-utils \
  python3-opencv \
  python3-numpy \
  python3-scipy \
  python3-yaml \
  python3-pil \
  python3-psutil \
  python3-matplotlib \
  python3-pandas \
  python3-seaborn \
  python3-venv \
  python3-full \
  python3-dev \
  build-essential

# Avoid colcon scanning vendored third-party repos as ROS/Python packages.
touch third_party/COLCON_IGNORE
[ -d third_party/deep-person-reid-master ] && touch third_party/deep-person-reid-master/COLCON_IGNORE

python3 -m venv --system-site-packages .venv
source .venv/bin/activate

# Keep setuptools compatible with colcon-core on Ubuntu/ROS2 Jazzy.
python -m pip install --upgrade "pip<26" "setuptools<80" wheel

# Install CPU-only torch by default. This avoids pulling 4GB+ CUDA 13 wheels.
# If you need GPU later, replace this with the matching PyTorch CUDA command.
python -m pip install --index-url https://download.pytorch.org/whl/cpu \
  "torch==2.3.1+cpu" "torchvision==0.18.1+cpu"

python -m pip install -r requirements_pip.txt

# Install colcon inside venv and build using the venv python, not /usr/bin/python3.
python -m pip install "colcon-common-extensions"

rm -rf build install log
python -m colcon build --symlink-install --packages-select person_reid_tracker

echo ""
echo "Setup done. Run with:"
echo "  cd $WS_DIR"
echo "  ./scripts/run_camera_reid.sh"
echo ""
echo "Check interpreter/deps with:"
echo "  source .venv/bin/activate"
echo "  python - <<'PY'"
echo "import sys, torch, cv2, rclpy"
echo "print(sys.executable)"
echo "print(torch.__version__)"
echo "print(cv2.__version__)"
echo "PY"
