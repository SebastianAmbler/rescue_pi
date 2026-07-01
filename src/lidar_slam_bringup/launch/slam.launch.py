#!/usr/bin/env python3
"""
Full SLAM bringup for the YDLIDAR T-MINI-Plus (12 m) lidar.

Pipeline / TF tree:
    map  --(slam_toolbox)-->  odom  --(EKF or rf2o)-->  base_link
    base_link --(static, from ydlidar_launch)--> laser_frame
    base_link --(static)--> imu_link

Nodes:
  * ydlidar_ros2_driver_node -> /scan          (YDLidar T-mini Plus, 230400 baud)
  * rf2o_laser_odometry_node -> /odom_rf2o     (scan-matching odometry)
  * witmotion imu_node       -> /imu/data      (yaw-rate only into the EKF)
  * ekf_filter_node          -> odom->base_link tf  (rf2o + IMU yaw-rate)
  * slam_toolbox async       -> /map, map->odom tf
  * rosbridge websocket      -> ws://<pi-ip>:9090  (for the Windows viewer)
  * imu_udp_bridge           -> /imu/data as JSON to PC:3392 (Windows UI sim)
  * UDP.py  (ExecuteProcess) -> ESP32 track-drive bridge (non-ROS)
  * UDPS.py (ExecuteProcess) -> Teensy/Mega servo bridge (non-ROS)

The lidar is driven by ydlidar_ros2_driver's own ydlidar_launch.py
(params: TminiPro.yaml), which also publishes the base_link -> laser_frame TF.

Launch args (override with name:=value):
  start_lidar:=true (default) include ydlidar_launch.py so this one file
                     launches the whole stack. Set false ONLY if you run the
                     lidar separately (ros2 launch ydlidar_ros2_driver
                     ydlidar_launch.py) — never run both, they fight over the port.
  use_ekf:=true      fuse IMU yaw-rate with rf2o via robot_localization.
                     false -> rf2o publishes odom->base_link directly.
  start_imu:=true    start the WitMotion IMU node.
  start_rosbridge:=true        imu_port:=/dev/imu
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("lidar_slam_bringup")
    slam_params = os.path.join(pkg, "config", "slam_toolbox.yaml")
    ekf_params = os.path.join(pkg, "config", "ekf.yaml")
    ydlidar_launch = os.path.join(
        get_package_share_directory("ydlidar_ros2_driver"),
        "launch", "ydlidar_launch.py")

    use_ekf = LaunchConfiguration("use_ekf")
    start_imu = LaunchConfiguration("start_imu")
    start_rosbridge = LaunchConfiguration("start_rosbridge")
    start_lidar = LaunchConfiguration("start_lidar")
    imu_port = LaunchConfiguration("imu_port")
    start_imu_bridge = LaunchConfiguration("start_imu_bridge")
    pc_ip = LaunchConfiguration("pc_ip")
    start_esp32_bridge = LaunchConfiguration("start_esp32_bridge")
    start_servo_bridge = LaunchConfiguration("start_servo_bridge")
    start_arm_guard = LaunchConfiguration("start_arm_guard")
    start_qr_markers = LaunchConfiguration("start_qr_markers")
    scripts_dir = LaunchConfiguration("scripts_dir")

    args = [
        DeclareLaunchArgument("use_ekf", default_value="true"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("start_rosbridge", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("imu_port", default_value="/dev/imu"),
        # Forward /imu/data to the Windows UI over UDP (broadcast if pc_ip empty).
        DeclareLaunchArgument("start_imu_bridge", default_value="true"),
        DeclareLaunchArgument("pc_ip", default_value="192.168.1.67"),
        # Standalone (non-ROS) bridges in scripts_dir: UDP.py (ESP32 drive),
        # UDPS.py (Teensy/Mega servos). Set false if you run them by hand.
        DeclareLaunchArgument("start_esp32_bridge", default_value="true"),
        DeclareLaunchArgument("start_servo_bridge", default_value="true"),
        # Pause SLAM while the arm is out of its home pose (taps UDPS.py's
        # local arm-angle forward on 127.0.0.1:3393).
        DeclareLaunchArgument("start_arm_guard", default_value="true"),
        # Drop a marker on the SLAM map when the PC reports a QR detection
        # (UDP 3393, 0xCC-framed JSON) -> /qr_markers (latched MarkerArray).
        DeclareLaunchArgument("start_qr_markers", default_value="true"),
        DeclareLaunchArgument("scripts_dir",
                              default_value=os.path.expanduser("~")),
    ]

    # --- Lidar: YDLIDAR T-MINI-Plus via its own launch (publishes /scan
    #     and the base_link -> laser_frame static TF) ---
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch),
        condition=IfCondition(start_lidar),
    )

    # --- rf2o scan-matching odometry ---
    # With EKF: rf2o only publishes /odom_rf2o (no TF); EKF owns odom->base_link.
    # Without EKF: rf2o publishes odom->base_link directly.
    def rf2o_node(publish_tf, condition):
        return Node(
            package="rf2o_laser_odometry",
            executable="rf2o_laser_odometry_node",
            name="rf2o_laser_odometry",
            output="screen",
            condition=condition,
            parameters=[{
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_rf2o",
                "publish_tf": publish_tf,
                "base_frame_id": "base_link",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 10.0,  # match the YDLidar T-mini Plus 10 Hz scan rate
            }],
        )

    rf2o_with_ekf = rf2o_node(False, IfCondition(use_ekf))
    rf2o_no_ekf = rf2o_node(True, UnlessCondition(use_ekf))

    # --- IMU (WitMotion BWT901CL) ---
    imu = Node(
        package="lidar_slam_bringup",
        executable="imu_node",
        name="witmotion_imu",
        output="screen",
        condition=IfCondition(start_imu),
        parameters=[{
            "port": imu_port,
            "fallback_port": "/dev/ttyUSB0",
            "baudrate": 115200,
            "frame_id": "imu_link",
        }],
    )

    # --- IMU -> Windows UI UDP bridge (broadcasts to PC:3392) ---
    imu_bridge = Node(
        package="lidar_slam_bringup",
        executable="imu_udp_bridge",
        name="imu_udp_bridge",
        output="screen",
        condition=IfCondition(start_imu_bridge),
        parameters=[{
            "pc_ip": pc_ip,
            "pc_port": 3392,
            "rate_hz": 30.0,
        }],
    )

    # --- Standalone (non-ROS) UDP bridges run via ExecuteProcess ---
    # UDP.py  = ESP32 track-drive bridge (UDP 3390 in / 3391 out)
    esp32_bridge = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([scripts_dir, "UDP.py"])],
        name="esp32_bridge",
        output="screen",
        condition=IfCondition(start_esp32_bridge),
    )
    # UDPS.py = Teensy/Mega servo bridge (UDP 3391 in / angles -> PC 3392)
    servo_bridge = ExecuteProcess(
        cmd=["python3", PathJoinSubstitution([scripts_dir, "UDPS.py"])],
        name="servo_bridge",
        output="screen",
        condition=IfCondition(start_servo_bridge),
    )

    # --- EKF: rf2o + IMU yaw-rate -> odom->base_link ---
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        condition=IfCondition(use_ekf),
        parameters=[ekf_params],
    )

    # --- slam_toolbox (async mapping) -> map, map->odom ---
    # NOTE: slam_toolbox is no longer auto-started here. The slam_manager node
    # owns its process lifecycle so the UI's SLAM button can start a *fresh* map
    # on every toggle-on (this build has no reset service). slam_manager spawns
    # `ros2 run slam_toolbox async_slam_toolbox_node` with this same params file.

    # --- slam_manager: /slam/{start,stop,save_map} services (UI via rosbridge) ---
    slam_manager = Node(
        package="lidar_slam_bringup",
        executable="slam_manager",
        name="slam_manager",
        output="screen",
        parameters=[{
            "slam_params_file": slam_params,
            "maps_dir": os.path.expanduser("~/Desktop/Maps"),
            "saved_maps_dir": os.path.expanduser("~/savedMaps"),
            # Push each exported map to the control PC's HTTP receiver.
            "pc_ip": pc_ip,
            "pc_port": 8780,
        }],
    )

    # --- arm_home_guard: pause SLAM while the arm is out of its home pose ---
    # Listens to UDPS.py's local arm-angle tap and toggles slam_toolbox's
    # pause_new_measurements so scans aren't integrated while the arm is
    # deployed (it occludes the lidar / changes the footprint).
    arm_home_guard = Node(
        package="lidar_slam_bringup",
        executable="arm_home_guard",
        name="arm_home_guard",
        output="screen",
        condition=IfCondition(start_arm_guard),
        parameters=[{
            "listen_port": 3393,
            "tolerance": 5,
            "resume_stable_sec": 1.0,
        }],
    )

    # --- qr_marker_node: PC QR detections -> /qr_markers on the SLAM map ---
    # Receives 0xCC-framed JSON on UDP 3393 (shares the port with arm_home_guard,
    # which only binds 127.0.0.1), places a marker at the robot's map->base_link
    # pose, and logs each hit to ~/savedMaps/qr_markers.csv.
    qr_markers = Node(
        package="lidar_slam_bringup",
        executable="qr_marker_node",
        name="qr_marker_node",
        output="screen",
        condition=IfCondition(start_qr_markers),
        parameters=[{
            "udp_port": 3393,
        }],
    )

    # --- Static transforms ---
    # NOTE: base_link -> laser_frame is published by ydlidar_launch.py
    #       (default z=0.02). Adjust it there to match your mount.
    # base_link -> imu_link : adjust to where the IMU is mounted.
    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu",
        arguments=["0", "0", "0.0", "0", "0", "0", "base_link", "imu_link"],
    )

    # --- rosbridge websocket (for Foxglove on the Windows PC) ---
    rosbridge = GroupAction(
        condition=IfCondition(start_rosbridge),
        actions=[
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[{"port": 9090}],
            ),
            Node(
                package="rosapi",
                executable="rosapi_node",
                name="rosapi",
                output="screen",
            ),
        ],
    )

    return LaunchDescription(args + [
        lidar, rf2o_with_ekf, rf2o_no_ekf, imu, imu_bridge, ekf, slam_manager,
        arm_home_guard, qr_markers, tf_imu, rosbridge, esp32_bridge,
        servo_bridge,
    ])
