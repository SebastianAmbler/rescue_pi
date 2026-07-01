#!/usr/bin/env python3
"""
Pause SLAM while the robot arm is out of its home pose.

When the arm is deployed it occludes the lidar and changes the robot's
footprint, so any scans integrated while it moves corrupt the map. This node
watches the arm joint angles and pauses slam_toolbox's new-measurement
integration whenever the arm leaves home, then resumes once it has settled
back home.

Angle source
------------
UDPS.py (the Teensy/Mega servo bridge) reads the arm angles off the serial
port and, besides forwarding them to the Windows UI, also forwards every
angles line to 127.0.0.1:<listen_port> (default 3393) for us. The line is the
same JSON the Teensy emits:

    {"type":"angles","front":90,"back":90,"arm1":50,"arm2":100,
     "arm3":0,"arm4":55,"arm5":90,"grip":90}

We can't read the serial port ourselves (UDPS.py owns it exclusively), hence
the UDP tap.

Pause mechanism
---------------
slam_toolbox exposes /slam_toolbox/pause_new_measurements (slam_toolbox/srv/
Pause). It is a *toggle* that returns the resulting `status`, so we drive it
toward the state we want and re-check on a reconcile timer. This keeps the map
intact (unlike stopping the slam process, which slam_manager uses for a fresh
map). If SLAM is not running there is nothing to pause; we just remember the
desired state and apply it when SLAM comes up (or restarts).
"""

import json
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Bool

from slam_toolbox.srv import Pause

DEFAULT_HOME = {
    "front": 90, "back": 90,
    "arm1": 130, "arm2": 35, "arm3": 0, "arm4": 90, "arm5": 90,
    "grip": 90, 
}


class ArmHomeGuard(Node):
    def __init__(self):
        super().__init__("arm_home_guard")

        self.declare_parameter("listen_port", 3393)
        # Home pose as a JSON object string (joint -> angle).
        self.declare_parameter("home_pose", json.dumps(DEFAULT_HOME))
        # Only arm1 and arm2 swing the arm into the lidar plane; the other
        # joints (base pan/tilt, wrist, gripper) don't occlude it, so they are
        # not part of the home check.
        self.declare_parameter("monitored_joints", ["arm1", "arm2"])
        self.declare_parameter("tolerance", 5)          # degrees, per joint
        self.declare_parameter("resume_stable_sec", 1.0)  # settle before resume
        self.declare_parameter("reconcile_period", 1.0)
        self.declare_parameter(
            "pause_service", "/slam_toolbox/pause_new_measurements")

        self._port = self.get_parameter("listen_port").value
        self._home = json.loads(self.get_parameter("home_pose").value)
        self._joints = list(self.get_parameter("monitored_joints").value)
        self._tol = int(self.get_parameter("tolerance").value)
        self._resume_stable = float(
            self.get_parameter("resume_stable_sec").value)
        pause_srv = self.get_parameter("pause_service").value

        # --- shared state (guarded by _lock) ---
        self._lock = threading.Lock()
        self._desired_paused = False   # True => want measurements paused
        self._is_home = None           # last computed home-state (None=unknown)
        self._home_since = None        # monotonic ts of continuous home start
        self._last_msg_ts = 0.0

        # --- pause-service plumbing ---
        self._applied = None           # last pause state we confirmed on slam
        self._call_in_flight = False
        self._was_ready = False
        self._cli = self.create_client(Pause, pause_srv)

        # --- /slam/active: latched Bool the web UI can subscribe to ---------
        # true  => actively mapping (slam_toolbox up AND measurements unpaused)
        # false => paused (by us / anyone) or SLAM stopped/not running
        # Transient-local so a client that connects later immediately gets the
        # current value. We drive it from the pause-service readiness and the
        # *confirmed* pause state (_applied) rather than a standalone flag, so a
        # pause toggled from elsewhere is reflected too.
        active_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._active_pub = self.create_publisher(Bool, "/slam/active", active_qos)
        self._last_active = None

        # --- UDP listener thread ---
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", self._port))
        self._sock.settimeout(1.0)
        self._stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        self.create_timer(
            float(self.get_parameter("reconcile_period").value),
            self._reconcile)

        self.get_logger().info(
            "arm_home_guard ready (udp 127.0.0.1:%d, joints=%s, tol=%d deg, "
            "pause via %s)" % (self._port, self._joints, self._tol, pause_srv))

        # Latch the initial state (SLAM not yet known up -> not active).
        self._publish_active()

    # -------------------------------------------------------- /slam/active
    def _publish_active(self):
        """Publish /slam/active iff the computed state changed.

        active = pause service ready (slam_toolbox up) AND measurements not
        paused. Republishing only on change keeps the log quiet; the latched
        QoS still delivers the held value to any new subscriber."""
        active = bool(self._was_ready and self._applied is False)
        if active != self._last_active:
            self._last_active = active
            self._active_pub.publish(Bool(data=active))
            self.get_logger().info("/slam/active -> %s" % active)

    # ------------------------------------------------------------------ rx
    def _rx_loop(self):
        while not self._stop:
            try:
                data, _ = self._sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            line = data.decode(errors="ignore").strip()
            if line.startswith("ANGLES:"):
                # legacy "ANGLES:" prefix -> strip to the JSON/body if any
                line = line[len("ANGLES:"):].strip()
            if not line.startswith("{"):
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if msg.get("type") != "angles":
                continue
            self._on_angles(msg)

    def _on_angles(self, msg):
        # All monitored joints must be present to judge the pose.
        try:
            is_home = all(
                abs(int(msg[j]) - int(self._home[j])) <= self._tol
                for j in self._joints)
        except (KeyError, TypeError, ValueError):
            return  # incomplete/garbled frame -> leave state unchanged

        now = time.monotonic()
        with self._lock:
            self._last_msg_ts = now
            self._is_home = is_home

            if not is_home:
                self._home_since = None
                if not self._desired_paused:
                    self._desired_paused = True
                    self.get_logger().info(
                        "arm left home -> pausing SLAM measurements")
            else:
                if self._home_since is None:
                    self._home_since = now
                # Resume only after the arm has held home long enough.
                if (self._desired_paused
                        and now - self._home_since >= self._resume_stable):
                    self._desired_paused = False
                    self.get_logger().info(
                        "arm back home -> resuming SLAM measurements")

    # ----------------------------------------------------------- reconcile
    def _reconcile(self):
        # Resume can be time-gated even without a new angles frame.
        with self._lock:
            if (self._is_home and self._desired_paused
                    and self._home_since is not None
                    and time.monotonic() - self._home_since
                    >= self._resume_stable):
                self._desired_paused = False
                self.get_logger().info(
                    "arm back home -> resuming SLAM measurements")
            desired = self._desired_paused

        ready = self._cli.service_is_ready()
        if not ready:
            # SLAM not running (or restarting): nothing to drive, and we no
            # longer know its pause state, so force a re-apply when it returns.
            if self._was_ready:
                self.get_logger().info(
                    "SLAM pause service gone; will re-apply on restart")
            self._was_ready = False
            self._applied = None
            self._publish_active()
            return

        if not self._was_ready:
            self.get_logger().info("SLAM pause service available")
            self._was_ready = True
            # slam_toolbox (re)starts with measurements unpaused, so we know its
            # state without toggling -- this avoids a spurious pause/resume on
            # startup when the arm is already home, and forces a re-pause after
            # a slam restart while the arm is still deployed.
            self._applied = False
            self._publish_active()

        if self._call_in_flight or self._applied == desired:
            return
        self._send_toggle(desired)

    def _send_toggle(self, desired):
        self._call_in_flight = True
        future = self._cli.call_async(Pause.Request())

        def _done(fut):
            self._call_in_flight = False
            try:
                status = bool(fut.result().status)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn("pause toggle failed: %s" % e)
                return
            # Pause toggles; if it landed on the desired state we're done,
            # otherwise the next reconcile tick toggles again.
            self._applied = status
            self._publish_active()
            if status == desired:
                self.get_logger().info(
                    "SLAM measurements %s"
                    % ("paused" if status else "resumed"))

        future.add_done_callback(_done)

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
    node = ArmHomeGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
