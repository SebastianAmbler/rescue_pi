#!/usr/bin/env bash
# Launches camera_ros (IMX708 -> ROS topics) + the MJPEG bridge (browser feed).
# Used by the camera-ros.service systemd unit; can also be run by hand.
source /opt/ros/humble/setup.bash
source /home/pi/ros2_ws/install/setup.bash
# our self-built libcamera lives in /usr/local
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}
exec ros2 launch camera_bridge camera.launch.py "$@"
