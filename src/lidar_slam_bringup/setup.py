import os
from glob import glob

from setuptools import setup

package_name = "lidar_slam_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="claudecode67420@gmail.com",
    description="Bringup for T-MINI-Plus lidar SLAM",
    license="MIT",
    entry_points={
        "console_scripts": [
            "imu_node = lidar_slam_bringup.imu_node:main",
            "imu_udp_bridge = lidar_slam_bringup.imu_udp_bridge:main",
            "slam_manager = lidar_slam_bringup.slam_manager:main",
            "arm_home_guard = lidar_slam_bringup.arm_home_guard:main",
            "qr_marker_node = lidar_slam_bringup.qr_marker_node:main",
        ],
    },
)
