# lidar_slam_bringup

SLAM for the **YDLIDAR T-MINI-Plus (12 m)** lidar on a headless Raspberry Pi
(ROS 2 Humble), visualised on a Windows 11 PC over **rosbridge**.

```
map ──(slam_toolbox)──> odom ──(EKF: rf2o + IMU yaw-rate)──> base_link
                                                              ├─(static)─ laser_frame  (from ydlidar_launch)
                                                              └─(static)─ imu_link
```

* **Lidar:** `ydlidar_ros2_driver` (T-mini Plus, `TminiPro.yaml`, 230400 baud) → `/scan` (best-effort QoS, frame `laser_frame`)
* **Odometry:** `rf2o_laser_odometry` (scan matching) → `/odom_rf2o`
* **IMU:** WitMotion BWT901CL → `/imu/data` (only the yaw *rate* is fused;
  the drifting absolute yaw is ignored)
* **Fusion:** `robot_localization` EKF → `odom → base_link`
* **Mapping:** `slam_toolbox` (async) → `/map`, `map → odom`
* **Bridge:** `rosbridge_server` websocket on `ws://<pi-ip>:9090`

## Devices

`udev` rules create stable names:

| Device | Symlink       | Chip   | Rule |
|--------|---------------|--------|------|
| Lidar  | `/dev/ydlidar`| CP2102 | `ydlidar*.rules` (installed by the ydlidar driver) |
| IMU    | `/dev/imu`    | CH340  | `/etc/udev/rules.d/99-lidar-imu.rules` |

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

**One command launches the whole stack** (lidar + rf2o + EKF + slam_toolbox +
IMU + rosbridge):

```bash
ros2 launch lidar_slam_bringup slam.launch.py
```

If the IMU is unplugged, disable it:

```bash
ros2 launch lidar_slam_bringup slam.launch.py start_imu:=false
```

Useful launch arguments:

| Arg               | Default     | Meaning |
|-------------------|-------------|---------|
| `start_lidar`     | `true`      | Include `ydlidar_launch.py`. Set `false` ONLY if you run the lidar in its own terminal — never run both, they fight over `/dev/ydlidar`. |
| `use_ekf`         | `true`      | Fuse IMU yaw-rate with rf2o. `false` → rf2o publishes `odom→base_link` directly (no IMU). |
| `start_imu`       | `true`      | Start the WitMotion IMU node. Set `false` if the IMU is unplugged. |
| `start_rosbridge` | `true`      | Start the rosbridge websocket. |
| `imu_port`        | `/dev/imu`  | IMU serial port. |

Advanced: run the lidar separately (debugging the driver):

```bash
# terminal 1 — lidar only
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
# terminal 2 — everything else (lidar disabled here)
ros2 launch lidar_slam_bringup slam.launch.py start_lidar:=false
```

## Visualise on Windows 11 (Foxglove Studio + rosbridge)

1. Install **Foxglove Studio** for Windows (https://foxglove.dev/download).
2. Make sure the Pi and PC are on the same network. Pi IP: **192.168.1.167**.
3. In Foxglove: **Open connection → Rosbridge (ROS 1 & 2)** →
   `ws://192.168.1.167:9090` → **Open**.
4. Add panels:
   * **3D** panel: enable `/map`, `/scan`, and the **TF** tree.
   * Set the 3D panel's *Frame* to `map`.
5. Drive/move the lidar around — the occupancy grid builds live.

> Tip: if Foxglove can't connect, check the Pi firewall and that
> `rosbridge_websocket` is listening: `ss -ltnp | grep 9090`.

## Save the map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map      # if nav2 installed
# or via slam_toolbox service:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/pi/my_map'}}"
```

## Tuning notes

* **Lidar mount:** the `base_link → laser_frame` static transform is published
  by `ydlidar_launch.py` (default `z=0.02 m`). Edit it in
  `~/ros2_ws/src/ydlidar_ros2_driver-humble/launch/ydlidar_launch.py` to match
  your robot. If the map looks mirrored, toggle `reversion`/`inverted` in
  `params/TminiPro.yaml`.
* **Lidar config:** `params/TminiPro.yaml` (`lidar_type: 1`, `baudrate: 230400`,
  `range_max: 12.0`). The driver reports `Model: Tmini Plus` on startup.
* **IMU heading:** by design only the gyro yaw-rate enters the EKF
  (`config/ekf.yaml`, `imu0_config`). rf2o owns absolute heading.
* **QoS:** the YDLidar driver publishes `/scan` best-effort; rf2o and
  slam_toolbox both subscribe compatibly (verified).
