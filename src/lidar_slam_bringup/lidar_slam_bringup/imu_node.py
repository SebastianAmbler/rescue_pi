#!/usr/bin/env python3
"""
ROS2 node for the WitMotion BWT901CL IMU (CH340 USB-serial).

Publishes sensor_msgs/Imu on /imu/data.

Protocol (default UART output mode): 11-byte packets
    0x55, <type>, d0..d7, <checksum>
    0x51 = acceleration (g)        -> linear_acceleration (m/s^2)
    0x52 = angular velocity (deg/s)-> angular_velocity (rad/s)
    0x53 = angle roll/pitch/yaw    -> orientation quaternion
    0x54 = magnetic field          -> ignored (we run 6-axis, no mag fusion)

At startup we switch the IMU to 6-axis fusion (gyro + accel only) so the
magnetometer is excluded from the yaw solution -> immune to magnetic
interference. Yaw then drifts slowly with no absolute reference, so we still
mark it low-confidence (large covariance) and the downstream EKF uses only the
gyro yaw-rate, not absolute yaw, for heading.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial

PACKET_HEADER = 0x55
PACKET_LEN = 11

# WitMotion config commands (format: FF AA <reg> <lo> <hi>).
# We switch the onboard fusion to 6-axis (gyro + accel only) so the
# magnetometer is excluded from the yaw solution -> immune to magnetic
# interference (yaw then drifts slowly instead of jumping). Sent without the
# SAVE command, so it applies for the session but doesn't write flash; it is
# re-applied on every startup.
WIT_UNLOCK = bytes([0xFF, 0xAA, 0x69, 0x88, 0xB5])
WIT_SET_6AXIS = bytes([0xFF, 0xAA, 0x24, 0x01, 0x00])  # reg 0x24 ALG: 1 = 6-axis

G = 9.80665
DEG2RAD = math.pi / 180.0


def to_signed_short(low, high):
    value = (high << 8) | low
    if value >= 32768:
        value -= 65536
    return value


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * cp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


class ImuNode(Node):
    def __init__(self):
        super().__init__("witmotion_imu")
        self.declare_parameter("port", "/dev/imu")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "imu_link")
        # Fallback port if the primary (e.g. udev symlink) is missing.
        self.declare_parameter("fallback_port", "/dev/ttyUSB0")

        self.frame_id = self.get_parameter("frame_id").value
        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baudrate").value)
        fallback = self.get_parameter("fallback_port").value

        self.ser = self._open(port, baud, fallback)
        self._set_6axis()

        self.pub = self.create_publisher(Imu, "imu/data", 50)

        # Latest values cached between packet types.
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]

        self._stop = False
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()

    def _open(self, port, baud, fallback):
        for p in (port, fallback):
            try:
                s = serial.Serial(p, baud, timeout=1)
                self.get_logger().info(f"Opened IMU on {p} @ {baud}")
                return s
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"Could not open {p}: {e}")
        raise RuntimeError("Could not open IMU serial port")

    def _set_6axis(self):
        """Disable the magnetometer in the onboard fusion (9-axis -> 6-axis).

        Yaw becomes pure gyro integration: no magnetic interference, but it
        drifts slowly with no absolute heading reference. Verify with a magnet
        near the sensor -- yaw should NOT react in 6-axis mode.
        """
        try:
            self.ser.write(WIT_UNLOCK)
            time.sleep(0.2)
            self.ser.write(WIT_SET_6AXIS)
            time.sleep(0.2)
            self.get_logger().info("IMU set to 6-axis fusion (magnetometer excluded)")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Failed to set 6-axis mode: {e}")

    def _read_loop(self):
        while not self._stop and rclpy.ok():
            try:
                byte = self.ser.read(1)
                if not byte or byte[0] != PACKET_HEADER:
                    continue
                packet = byte + self.ser.read(PACKET_LEN - 1)
                if len(packet) != PACKET_LEN:
                    continue
                if (sum(packet[0:10]) & 0xFF) != packet[10]:
                    continue
                self._handle(packet[1], packet[2:10])
            except Exception:  # noqa: BLE001 - port closed on shutdown, etc.
                if self._stop:
                    break

    def _handle(self, ptype, data):
        if ptype == 0x51:  # acceleration (g)
            self.accel = [
                to_signed_short(data[0], data[1]) / 32768.0 * 16 * G,
                to_signed_short(data[2], data[3]) / 32768.0 * 16 * G,
                to_signed_short(data[4], data[5]) / 32768.0 * 16 * G,
            ]
        elif ptype == 0x52:  # angular velocity (deg/s)
            self.gyro = [
                to_signed_short(data[0], data[1]) / 32768.0 * 2000 * DEG2RAD,
                to_signed_short(data[2], data[3]) / 32768.0 * 2000 * DEG2RAD,
                to_signed_short(data[4], data[5]) / 32768.0 * 2000 * DEG2RAD,
            ]
        elif ptype == 0x53:  # angle (deg) -> publish a full Imu message
            roll = to_signed_short(data[0], data[1]) / 32768.0 * 180 * DEG2RAD
            pitch = to_signed_short(data[2], data[3]) / 32768.0 * 180 * DEG2RAD
            yaw = to_signed_short(data[4], data[5]) / 32768.0 * 180 * DEG2RAD
            self._publish(roll, pitch, yaw)

    def _publish(self, roll, pitch, yaw):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        # roll/pitch are gravity-referenced (trustworthy); yaw drifts -> huge cov.
        msg.orientation_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 1e6,
        ]

        msg.angular_velocity.x = self.gyro[0]
        msg.angular_velocity.y = self.gyro[1]
        msg.angular_velocity.z = self.gyro[2]
        msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001,
        ]

        msg.linear_acceleration.x = self.accel[0]
        msg.linear_acceleration.y = self.accel[1]
        msg.linear_acceleration.z = self.accel[2]
        msg.linear_acceleration_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05,
        ]

        self.pub.publish(msg)

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
