#!/usr/bin/env python3
"""
Drop a marker on the SLAM map whenever the Windows PC detects a QR code.

The Windows PC runs the QR detector and, on each hit, sends a UDP packet to the
Pi on port 3393 framed as a single 0xCC byte followed by JSON:

    {"cmd":"qr_marker","n":<int>,"id":"<qr string>","time":"HH:MM:SS"}

n is a detection counter; id is the decoded QR text. We take the robot's
current pose from the SLAM TF tree (map -> base_link) as the marker location --
i.e. the marker is placed where the robot was standing when it saw the code.

Port sharing with arm_home_guard
--------------------------------
arm_home_guard also uses 3393, but it binds 127.0.0.1:3393 (it only ever
receives UDPS.py's local arm-angle tap). We bind 0.0.0.0:<udp_port> with
SO_REUSEADDR so both sockets can sit on the same port: Linux delivers loopback
packets (the arm-angle tap) to the more specific 127.0.0.1 socket and the PC's
LAN packets to our wildcard socket. The 0xCC frame byte and the "qr_marker"
cmd are an extra guard against acting on anything that isn't a QR packet.

Markers
-------
A persistent visualization_msgs/MarkerArray is published latched
(TRANSIENT_LOCAL) on /qr_markers so a web/Foxglove client that connects later
still receives every marker. Each detection appends two markers (unique,
incrementing ids):

  * a SPHERE in namespace "qr" at (x, y, 0.1). A web viewer reads the marker's
    .text field for the label f"#{n} {id}".
  * a TEXT_VIEW_FACING in namespace "qr_text" at (x, y, 0.35) with the same
    label so it is readable in RViz/Foxglove.

We re-publish the full array after each addition.

Each detection is also appended to ~/savedMaps/qr_markers.csv (same dir as
slam_manager's saved maps) so the QR hits persist alongside the maps.
"""

import csv
import json
import math
import os
import socket
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

import tf2_ros
from std_msgs.msg import ColorRGBA, Empty
from visualization_msgs.msg import Marker, MarkerArray

# Frame byte the PC prefixes every command packet with.
FRAME_BYTE = 0xCC
# Distinct colour for the QR sphere markers (orange).
QR_COLOR = ColorRGBA(r=1.0, g=0.55, b=0.0, a=1.0)
TEXT_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)


def quaternion_yaw(x, y, z, w):
    """Yaw (rad) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class QrMarkerNode(Node):
    def __init__(self):
        super().__init__("qr_marker_node")

        self.declare_parameter("udp_port", 3393)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter(
            "csv_path", os.path.expanduser("~/savedMaps/qr_markers.csv"))

        self._port = int(self.get_parameter("udp_port").value)
        self._map_frame = self.get_parameter("map_frame").value
        self._base_frame = self.get_parameter("base_frame").value
        self._csv_path = self.get_parameter("csv_path").value

        # --- TF: latest map -> base_link pose of the robot ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- latched MarkerArray so late web clients get every marker ---
        # KEEP_LAST with a large depth: each publish is the *full* array, so a
        # depth comfortably above the expected marker count keeps history intact.
        marker_qos = QoSProfile(
            depth=100,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(MarkerArray, "/qr_markers", marker_qos)

        # Markers are appended from the UDP rx thread and cleared from the
        # executor thread (new-map signal), so guard the shared state.
        self._lock = threading.Lock()
        self._markers = MarkerArray()
        self._next_id = 0

        # slam_manager publishes here whenever it starts a fresh map; the old
        # markers belong to the previous map's frame, so drop them. Latched
        # (TRANSIENT_LOCAL) to match the publisher so we still get the most
        # recent signal if this node (re)starts after a new map began.
        new_map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Empty, "/slam/new_map", self._on_new_map, new_map_qos)

        # --- UDP listener thread ---
        # Bind the wildcard so we get the PC's LAN packets; SO_REUSEADDR lets us
        # coexist with arm_home_guard's 127.0.0.1:<port> socket (see module doc).
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", self._port))
        self._sock.settimeout(1.0)
        self._stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.get_logger().info(
            "qr_marker_node ready (udp 0.0.0.0:%d, tf %s->%s, csv %s)"
            % (self._port, self._map_frame, self._base_frame, self._csv_path))

    # ------------------------------------------------------------------ rx
    def _rx_loop(self):
        while not self._stop:
            try:
                data, _ = self._sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data or data[0] != FRAME_BYTE:
                continue
            try:
                msg = json.loads(data[1:].decode("utf-8", errors="ignore"))
            except ValueError:
                continue  # malformed JSON -> ignore
            if not isinstance(msg, dict) or msg.get("cmd") != "qr_marker":
                continue
            self._on_qr(msg)

    def _on_qr(self, msg):
        n = msg.get("n")
        qr_id = msg.get("id")
        when = msg.get("time", "")
        if n is None or qr_id is None:
            return  # incomplete frame -> ignore

        # Robot pose at detection time: latest map -> base_link.
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(
                "no %s->%s transform yet; skipping QR #%s (%s): %s"
                % (self._map_frame, self._base_frame, n, qr_id, e))
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        x, y = t.x, t.y
        yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
        label = "#%s %s" % (n, qr_id)

        self.get_logger().info(
            "QR %s @ map (%.2f, %.2f) yaw %.1f deg"
            % (label, x, y, math.degrees(yaw)))

        self._append_markers(x, y, label)
        self._append_csv(n, qr_id, when, x, y, yaw)

    # ------------------------------------------------------------- markers
    def _on_new_map(self, _msg):
        """A fresh SLAM map was started -> wipe every QR marker.

        We publish a single DELETEALL marker (clears all namespaces in
        RViz/Foxglove and our web viewer) and reset the array so late-joining
        clients no longer replay the previous map's markers off the latched
        topic. The id counter restarts so the new map's markers start at 0.
        """
        with self._lock:
            clear = MarkerArray()
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.action = Marker.DELETEALL
            clear.markers.append(m)
            self._pub.publish(clear)
            self._markers = MarkerArray()
            self._next_id = 0
        self.get_logger().info("new SLAM map -> cleared QR markers")

    def _append_markers(self, x, y, label):
        stamp = self.get_clock().now().to_msg()

        sphere = Marker()
        sphere.header.frame_id = self._map_frame
        sphere.header.stamp = stamp
        sphere.ns = "qr"
        sphere.id = self._next_id
        self._next_id += 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = x
        sphere.pose.position.y = y
        sphere.pose.position.z = 0.1
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.2
        sphere.scale.y = 0.2
        sphere.scale.z = 0.2
        sphere.color = QR_COLOR
        # A web viewer reads the label off the sphere's .text field.
        sphere.text = label

        text = Marker()
        text.header.frame_id = self._map_frame
        text.header.stamp = stamp
        text.ns = "qr_text"
        text.id = self._next_id
        self._next_id += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 0.35
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2  # text height
        text.color = TEXT_COLOR
        text.text = label

        with self._lock:
            self._markers.markers.append(sphere)
            self._markers.markers.append(text)
            self._pub.publish(self._markers)

    # ----------------------------------------------------------------- csv
    def _append_csv(self, n, qr_id, when, x, y, yaw):
        try:
            os.makedirs(os.path.dirname(self._csv_path), exist_ok=True)
            new_file = not os.path.exists(self._csv_path)
            with open(self._csv_path, "a", newline="") as f:
                w = csv.writer(f)
                if new_file:
                    w.writerow(["n", "id", "time", "map_x", "map_y", "yaw"])
                w.writerow([n, qr_id, when,
                            "%.4f" % x, "%.4f" % y, "%.4f" % yaw])
        except OSError as e:
            self.get_logger().warn("failed to write CSV %s: %s"
                                   % (self._csv_path, e))

    # ------------------------------------------------------------ shutdown
    def destroy_node(self):
        self._stop = True
        try:
            self._sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
