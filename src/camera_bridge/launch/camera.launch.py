"""Launch camera_ros (owns the IMX708) + the MJPEG bridge for the control UI.

camera_ros publishes:
  /camera/image_raw              (raw, for your CV / ROS nodes)
  /camera/image_raw/compressed   (JPEG)
  /camera/camera_info
The bridge re-serves the compressed topic at http://<pi>:8000/stream.mjpg.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fmt = LaunchConfiguration("format")
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument("width", default_value="1920"),
        DeclareLaunchArgument("height", default_value="1080"),
        # RGB888 -> /camera/image_raw is rgb8, convenient for OpenCV/CV nodes.
        DeclareLaunchArgument("format", default_value="RGB888"),
        DeclareLaunchArgument("port", default_value="8000"),

        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            parameters=[{
                "width": width,
                "height": height,
                "format": fmt,
                # libcamera AF control: 0=manual 1=auto 2=continuous
                "AfMode": 2,
                # compressed_image_transport JPEG quality (1-100); flat param name.
                # Lowered 90->75 to cut per-frame CPU encode at 1080p (encode was
                # the fps bottleneck: camera_node pinned ~1 core, stream ~15fps).
                "jpeg_quality": 85,
                "FrameDurationLimits": [20000, 20000],
            }],
            output="screen",
        ),

        Node(
            package="camera_bridge",
            executable="mjpeg_bridge",
            name="mjpeg_bridge",
            parameters=[{
                "topic": "/camera/image_raw/compressed",
                "port": port,
            }],
            output="screen",
        ),
    ])
