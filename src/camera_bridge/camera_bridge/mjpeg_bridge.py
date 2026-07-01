#!/usr/bin/env python3
"""
MJPEG HTTP bridge for the control UI.

Subscribes to a ROS 2 sensor_msgs/CompressedImage topic (JPEG, as produced by
camera_ros + compressed_image_transport) and re-serves the bytes to browsers as
multipart/x-mixed-replace. The JPEG is NOT re-encoded -- the compressed frames
from the camera are forwarded as-is, so this adds almost no CPU.

This node does NOT touch the camera; camera_ros owns /dev/video0. Any number of
browsers (and on-Pi consumers) can pull frames here while ROS nodes subscribe to
the raw image topic for processing.

Endpoints (default port 8000):
  /              -> full-bleed HTML page wrapping the stream (CAM 1/CAM 2 iframes)
  /stream.mjpg   -> raw multipart MJPEG (for <img src=...>, e.g. webcam overlay)
  /snapshot.jpg  -> single latest JPEG frame

Parameters:
  ~topic (str)   default /camera/image_raw/compressed
  ~port  (int)   default 8000
  ~host  (str)   default 0.0.0.0
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

PAGE = (
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<style>html,body{margin:0;height:100%;background:#000;overflow:hidden}"
    "img{width:100%;height:100%;object-fit:cover;display:block}</style></head>"
    "<body><img src='/stream.mjpg'></body></html>"
).encode()


class FrameHub:
    """Holds the latest JPEG frame and wakes waiting HTTP clients."""

    def __init__(self):
        self.frame = None
        self.seq = 0
        self.cond = threading.Condition()

    def publish(self, data: bytes):
        with self.cond:
            self.frame = data
            self.seq += 1
            self.cond.notify_all()

    def get(self, last_seq, timeout=5.0):
        with self.cond:
            if self.seq == last_seq:
                self.cond.wait(timeout)
            return self.frame, self.seq


hub = FrameHub()


def make_handler():
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass

        def _cors(self):
            self.send_header("Access-Control-Allow-Origin", "*")

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self._cors()
                self.end_headers()
                self.wfile.write(PAGE)
            elif path == "/snapshot.jpg":
                frame, _ = hub.get(-1)
                if frame is None:
                    self.send_error(503, "no frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.send_header("Cache-Control", "no-cache")
                self._cors()
                self.end_headers()
                self.wfile.write(frame)
            elif path == "/stream.mjpg":
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private, max-age=0")
                self.send_header("Pragma", "no-cache")
                self._cors()
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()
                seq = -1
                try:
                    while True:
                        frame, seq = hub.get(seq)
                        if frame is None:
                            continue
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            ("Content-Length: %d\r\n\r\n" % len(frame)).encode())
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)
    return Handler


class MjpegBridge(Node):
    def __init__(self):
        super().__init__("mjpeg_bridge")
        topic = self.declare_parameter(
            "topic", "/camera/image_raw/compressed").value
        self.port = self.declare_parameter("port", 8000).value
        self.host = self.declare_parameter("host", "0.0.0.0").value
        self.create_subscription(
            CompressedImage, topic, self._on_image, qos_profile_sensor_data)
        self.httpd = ThreadingHTTPServer((self.host, self.port), make_handler())
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(
            "MJPEG bridge serving %s on http://%s:%d/stream.mjpg"
            % (topic, self.host, self.port))

    def _on_image(self, msg: CompressedImage):
        hub.publish(bytes(msg.data))

    def destroy_node(self):
        try:
            self.httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MjpegBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
