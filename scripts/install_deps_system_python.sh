#!/usr/bin/env bash
set -euo pipefail
# Alternative quick install for Ubuntu 24.04/ROS2 Jazzy if you do NOT want a venv.
# This uses --break-system-packages because Ubuntu blocks global pip installs by default.
python3 -m pip install --break-system-packages -r requirements_pip.txt
