#!/usr/bin/env bash
# rescue_pi post-setup verification. Run after reboot:  bash ~/ros2_ws/verify_stack.sh
set +e
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/pi/ros2_ws/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH:-}

echo "════════════ DEVICES ════════════"
for d in ydlidar imu esp32hat teensy; do printf "  /dev/%-9s -> %s\n" "$d" "$(readlink /dev/$d 2>/dev/null || echo MISSING)"; done

echo "════════════ CAMERA (IMX708) ════════════"
echo "  kernel imx708 bound : $(sudo dmesg 2>/dev/null | grep -i imx708 | tail -1 || echo '(none in dmesg)')"
echo "  v4l-subdev          : $(ls /dev/v4l-subdev* 2>/dev/null | tr '\n' ' ' || echo none)"
echo "  --- cam -l ---"; cam -l 2>/dev/null | grep -iE "imx708|Available" || echo "  cam found no camera"

echo "════════════ camera-ros.service ════════════"
systemctl is-active camera-ros.service
echo "  /camera topics:"; ros2 topic list 2>/dev/null | grep -E "/camera" | sed 's/^/    /'
echo "  compressed rate (3s):"; timeout 4 ros2 topic hz /camera/image_raw/compressed 2>/dev/null | grep -m1 average || echo "    (no frames)"
echo "  MJPEG snapshot:"; curl -s -o /tmp/snap.jpg -w "    http 8000 -> %{http_code}, %{size_download} bytes\n" http://localhost:8000/snapshot.jpg 2>/dev/null
  echo "  color cast (magenta_idx ~1.0 = accurate, >1.15 = purple):"
  python3 - <<'PY' 2>/dev/null || echo "    (cv2 not available)"
import cv2
img=cv2.imread('/tmp/snap.jpg')
if img is not None:
    b,g,r=[float(img[:,:,i].mean()) for i in range(3)]
    print(f"    R={r:.0f} G={g:.0f} B={b:.0f}  magenta_idx={(r+b)/(2*g):.2f}")
PY

echo "════════════ SLAM stack (manual launch) ════════════"
echo "  Run:  ros2 launch lidar_slam_bringup slam.launch.py"
echo "  Then: ros2 service call /slam/start std_srvs/srv/Trigger"
echo "  Expect /scan /imu/data /odom_rf2o /map, TF map->base_link, rosbridge :9090"
