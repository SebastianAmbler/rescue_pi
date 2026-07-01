#!/usr/bin/env python3
"""
Bridges /imu/data (sensor_msgs/Imu) to the Windows control UI.

The Windows ws_server.py listens on UDP 3392 (its `angles_sock`) and broadcasts
every line it receives verbatim to all connected WebSocket/browser clients. So
we just need to send a JSON line there; no change to ws_server.py is required.

We publish the orientation as roll/pitch/yaw in DEGREES (UI/sim friendly) plus
the yaw rate, under cmd "imu":

    {"cmd": "imu", "roll": -1.3, "pitch": 2.0, "yaw": 178.4, "yaw_rate": 0.01}

Finding the PC: UDPS.py learns the PC's IP because the PC sends commands to it
first (it reads the sender address off the incoming packet). This node is
send-only -- nothing arrives from the PC -- so there is no address to learn.
Instead, if pc_ip is left empty we UDP-broadcast to the subnet (e.g.
192.168.1.255:3392); ws_server.py binds 0.0.0.0:3392 so it receives the
broadcast without us knowing its exact IP. Set pc_ip to force unicast.

The browser UI's websocket.js needs a handler for cmd:"imu" to feed the sim and
light the IMU status dot (see README / chat notes).
"""

import json
import math
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

RAD2DEG = 180.0 / math.pi


def subnet_broadcast_addr():
    """Best-effort subnet broadcast address (assumes /24), e.g. 192.168.1.255.

    Uses a UDP 'connect' to a public IP to discover the Pi's primary local
    address -- this sets the socket's local end without sending any packet, so
    it works offline and doesn't touch IPv6 (see Pi IPv6 quirk). Falls back to
    the limited broadcast 255.255.255.255 if the local IP can't be determined.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        parts = ip.split(".")
        parts[3] = "255"
        return ".".join(parts)
    except OSError:
        return "255.255.255.255"
    finally:
        s.close()


def quaternion_to_euler(x, y, z, w):
    """Return (roll, pitch, yaw) in radians from a quaternion."""
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ImuUdpBridge(Node):
    def __init__(self):
        super().__init__("imu_udp_bridge")
        # PC running ws_server.py. Leave empty to auto-broadcast to the subnet.
        self.declare_parameter("pc_ip", "")
        self.declare_parameter("pc_port", 3392)
        # Throttle so we don't flood the UI; IMU may publish faster than this.
        self.declare_parameter("rate_hz", 30.0)

        pc_ip = self.get_parameter("pc_ip").value
        self.pc_port = int(self.get_parameter("pc_port").value)
        rate = float(self.get_parameter("rate_hz").value)
        self.min_period = 1.0 / rate if rate > 0 else 0.0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if pc_ip:
            self.dest_ip = pc_ip
            self.get_logger().info(
                f"Forwarding /imu/data -> {self.dest_ip}:{self.pc_port} "
                f"(unicast) @ {rate:.0f} Hz max"
            )
        else:
            self.dest_ip = subnet_broadcast_addr()
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            self.get_logger().info(
                f"Forwarding /imu/data -> {self.dest_ip}:{self.pc_port} "
                f"(broadcast, no pc_ip set) @ {rate:.0f} Hz max"
            )

        self._last_sent = 0.0

        self.create_subscription(Imu, "imu/data", self._on_imu, 10)

    def _on_imu(self, msg: Imu):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.min_period and (now - self._last_sent) < self.min_period:
            return
        self._last_sent = now

        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        payload = json.dumps({
            "cmd": "imu",
            "roll": round(roll * RAD2DEG, 2),
            "pitch": round(pitch * RAD2DEG, 2),
            "yaw": round(yaw * RAD2DEG, 2),
            "yaw_rate": round(msg.angular_velocity.z, 4),
        }).encode()
        try:
            self.sock.sendto(payload, (self.dest_ip, self.pc_port))
        except OSError as e:  # network down, bad address, etc.
            self.get_logger().warn(f"UDP send failed: {e}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = ImuUdpBridge()
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
