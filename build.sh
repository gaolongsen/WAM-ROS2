#!/usr/bin/env bash
set -euo pipefail

# Humble's generators must run with the system ROS 2 Python, not Anaconda.
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${PATH}"

colcon --log-base log_ros2 build \
  --symlink-install \
  --build-base build_ros2 \
  --install-base install_ros2 \
  "$@"
