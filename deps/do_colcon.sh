#!/usr/bin/env bash
set -o pipefail
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH:-}
export PKG_CONFIG_PATH=/usr/local/lib/aarch64-linux-gnu/pkgconfig:${PKG_CONFIG_PATH:-}
cd /home/pi/ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
