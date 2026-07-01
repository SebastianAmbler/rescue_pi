#!/usr/bin/env python3
"""
SLAM manager: lets the control UI start/stop slam_toolbox and export the map.

slam_toolbox is NOT started by the launch file anymore -- this node owns its
process lifecycle so the UI's SLAM button can give a *fresh* map on every
toggle-on (this slam_toolbox build has no reset/start-fresh service, so the only
reliable way to start a new map is to restart the node).

Exposed services (all std_srvs/srv/Trigger, so they are trivial to call from the
browser via rosbridge with an empty request):

  /slam/start      start async_slam_toolbox_node (fresh map). No-op if running.
  /slam/stop       export the current /map to a GeoTIFF in ~/Desktop/Maps, then
                   stop slam_toolbox.
  /slam/save_map   export the current /map to a GeoTIFF in ~/savedMaps, without
                   stopping mapping.

The map is exported as a GeoTIFF (GDAL) when available; if GDAL/osgeo is missing
we fall back to a map_server-style .pgm + .yaml pair so a map is never lost.

After each successful local export the artifact(s) are pushed to the control PC
via HTTP POST http://<pc_ip>:<pc_port>/api/upload-map (raw bytes in the body,
X-Filename + X-Subdir headers). The local save is the source of truth: an upload
failure (PC down / timeout / non-200) is logged loudly but never fails the
export, so a map is still kept on the Pi.

The node subscribes to /map with transient_local (latched) QoS so it always
holds the most recent occupancy grid, even one published before SLAM-side topics
were discovered.
"""

import os
import signal
import subprocess
import urllib.error
import urllib.request
from datetime import datetime

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

# Occupancy thresholds (match nav2 map_server defaults) and the trinary palette
# used when rasterising the grid into a viewable single-band image.
OCC_THRESH = 65          # >= -> occupied
FREE_THRESH = 25         # <  -> free
PX_OCCUPIED = 0          # black
PX_FREE = 254            # white
PX_UNKNOWN = 205         # gray


class SlamManager(Node):
    def __init__(self):
        super().__init__("slam_manager")

        self.declare_parameter("maps_dir", os.path.expanduser("~/Desktop/Maps"))
        self.declare_parameter("saved_maps_dir", os.path.expanduser("~/savedMaps"))
        self.declare_parameter(
            "slam_params_file",
            os.path.join(get_package_share_directory("lidar_slam_bringup"),
                         "config", "slam_toolbox.yaml"))
        # Control PC receiver for exported maps (HTTP POST /api/upload-map).
        self.declare_parameter("pc_ip", "192.168.1.67")
        self.declare_parameter("pc_port", 8780)
        self.declare_parameter("upload_timeout_s", 10.0)

        self.maps_dir = self.get_parameter("maps_dir").value
        self.saved_maps_dir = self.get_parameter("saved_maps_dir").value
        self.slam_params_file = self.get_parameter("slam_params_file").value
        self.pc_ip = self.get_parameter("pc_ip").value
        self.pc_port = int(self.get_parameter("pc_port").value)
        self.upload_timeout_s = float(self.get_parameter("upload_timeout_s").value)

        self._slam_proc = None       # subprocess.Popen for slam_toolbox, or None
        self._last_map = None         # latest nav_msgs/OccupancyGrid

        # Latched/transient_local so we receive the most recently published grid.
        map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)

        # Tells qr_marker_node to drop the previous map's QR markers whenever we
        # start a fresh map. Latched so a (re)starting qr_marker_node still sees
        # the most recent new-map signal.
        new_map_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._new_map_pub = self.create_publisher(
            Empty, "/slam/new_map", new_map_qos)

        self.create_service(Trigger, "/slam/start", self._srv_start)
        self.create_service(Trigger, "/slam/stop", self._srv_stop)
        self.create_service(Trigger, "/slam/save_map", self._srv_save_map)

        self.get_logger().info(
            "slam_manager ready (maps_dir=%s, saved_maps_dir=%s, "
            "upload->http://%s:%d/api/upload-map)"
            % (self.maps_dir, self.saved_maps_dir, self.pc_ip, self.pc_port))

    # --- /map cache --------------------------------------------------------
    def _on_map(self, msg):
        self._last_map = msg

    # --- process management -----------------------------------------------
    def _slam_running(self):
        return self._slam_proc is not None and self._slam_proc.poll() is None

    def _start_slam(self):
        cmd = ["ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
               "--ros-args",
               "--params-file", self.slam_params_file,
               "-r", "__node:=slam_toolbox"]
        # New process group so we can signal the whole tree on stop.
        self._slam_proc = subprocess.Popen(cmd, start_new_session=True)
        self._last_map = None
        # Fresh map -> clear any QR markers left over from the previous map.
        self._new_map_pub.publish(Empty())
        self.get_logger().info("started slam_toolbox (pid %d)" % self._slam_proc.pid)

    def _stop_slam(self):
        if not self._slam_running():
            self._slam_proc = None
            return
        pid = self._slam_proc.pid
        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
            try:
                self._slam_proc.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("slam_toolbox did not exit on SIGINT; killing")
                os.killpg(os.getpgid(pid), signal.SIGKILL)
                self._slam_proc.wait(timeout=5.0)
        except ProcessLookupError:
            pass
        self.get_logger().info("stopped slam_toolbox")
        self._slam_proc = None

    # --- services ----------------------------------------------------------
    def _srv_start(self, request, response):
        if self._slam_running():
            response.success = True
            response.message = "SLAM already running"
            return response
        try:
            self._start_slam()
            response.success = True
            response.message = "SLAM started (new map)"
        except Exception as e:  # noqa: BLE001 - surface any spawn failure to UI
            response.success = False
            response.message = "failed to start SLAM: %s" % e
            self.get_logger().error(response.message)
        return response

    def _srv_stop(self, request, response):
        running = self._slam_running()
        saved_path, err = self._export_map(self.maps_dir)
        self._stop_slam()
        if saved_path:
            response.success = True
            response.message = "SLAM stopped, map saved to %s" % saved_path
        else:
            response.success = False
            response.message = (
                "SLAM stopped, but map not saved: %s" % err
                if running else "SLAM was not running; %s" % err)
        return response

    def _srv_save_map(self, request, response):
        saved_path, err = self._export_map(self.saved_maps_dir)
        if saved_path:
            response.success = True
            response.message = "map saved to %s" % saved_path
        else:
            response.success = False
            response.message = "map not saved: %s" % err
            self.get_logger().warn(response.message)
        return response

    # --- map export --------------------------------------------------------
    def _export_map(self, out_dir):
        """Write the cached /map to out_dir. Returns (path, None) or (None, err)."""
        grid = self._last_map
        if grid is None:
            return None, "no map available yet"
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.join(out_dir, "map_%s" % stamp)

        info = grid.info
        w, h = info.width, info.height
        if w == 0 or h == 0:
            return None, "map is empty (0x0)"
        # ROS grid: row-major, row 0 at the bottom. Flip for top-down raster.
        raw = np.array(grid.data, dtype=np.int16).reshape(h, w)
        img = np.full((h, w), PX_UNKNOWN, dtype=np.uint8)
        img[(raw >= 0) & (raw < FREE_THRESH)] = PX_FREE
        img[raw >= OCC_THRESH] = PX_OCCUPIED
        img = np.flipud(img)

        subdir = os.path.basename(base)  # e.g. map_20260625_143000
        try:
            path = base + ".tif"
            self._write_geotiff(img, info, path)
            self.get_logger().info("saved GeoTIFF %s" % path)
            self._upload_export([path], subdir)
            return path, None
        except ImportError:
            path = self._write_pgm_yaml(img, info, base)
            msg = "GDAL not installed; saved PGM/YAML instead"
            self.get_logger().warn("%s (%s)" % (msg, path))
            self._upload_export([path, base + ".yaml"], subdir)
            return path, None
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("GeoTIFF export failed: %s" % e)
            try:
                path = self._write_pgm_yaml(img, info, base)
                self._upload_export([path, base + ".yaml"], subdir)
                return path, None
            except Exception as e2:  # noqa: BLE001
                return None, "%s / fallback failed: %s" % (e, e2)

    # --- upload to control PC ----------------------------------------------
    def _upload_export(self, paths, subdir):
        """Push each exported artifact to the PC. Never raises: a failed upload
        is logged but must not undo the (successful) local save above."""
        for p in paths:
            try:
                self._upload_one_file(p, subdir)
            except Exception as e:  # noqa: BLE001 - belt-and-suspenders
                self.get_logger().error(
                    "upload of %s raised unexpectedly: %s" % (p, e))

    def _upload_one_file(self, path, subdir):
        """POST one file's raw bytes to http://pc_ip:pc_port/api/upload-map.
        Returns True on HTTP 200, False otherwise (logging the reason)."""
        url = "http://%s:%d/api/upload-map" % (self.pc_ip, self.pc_port)
        fname = os.path.basename(path)
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError as e:
            self.get_logger().error("upload: cannot read %s: %s" % (path, e))
            return False

        req = urllib.request.Request(url, data=data, method="POST")
        req.add_header("X-Filename", fname)
        req.add_header("X-Subdir", subdir)
        req.add_header("Content-Type", "application/octet-stream")
        try:
            with urllib.request.urlopen(req, timeout=self.upload_timeout_s) as resp:
                code = resp.getcode()
                body = resp.read(512).decode("utf-8", "replace").strip()
            if code == 200:
                self.get_logger().info(
                    "uploaded %s (%d B) -> %s %s" % (fname, len(data), url, body))
                return True
            self.get_logger().error(
                "upload of %s failed: HTTP %s %s" % (fname, code, body))
            return False
        except urllib.error.HTTPError as e:  # non-2xx response
            detail = e.read(512).decode("utf-8", "replace").strip()
            self.get_logger().error(
                "upload of %s failed: HTTP %s %s %s"
                % (fname, e.code, e.reason, detail))
            return False
        except urllib.error.URLError as e:  # conn refused / DNS / timeout
            self.get_logger().error(
                "upload of %s failed: %s -- is the PC receiver up at %s ?"
                % (fname, e.reason, url))
            return False
        except Exception as e:  # noqa: BLE001 - socket.timeout etc.
            self.get_logger().error("upload of %s failed: %s" % (fname, e))
            return False

    def _write_geotiff(self, img, info, path):
        from osgeo import gdal  # raises ImportError if GDAL is absent

        h, w = img.shape
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        # Top-left corner of the (flipped, top-down) raster, north-up.
        gt = (ox, res, 0.0, oy + h * res, 0.0, -res)

        drv = gdal.GetDriverByName("GTiff")
        ds = drv.Create(path, w, h, 1, gdal.GDT_Byte, options=["COMPRESS=LZW"])
        ds.SetGeoTransform(gt)
        # Indoor SLAM map -> local metric coords, no geographic CRS.
        ds.GetRasterBand(1).WriteArray(img)
        ds.FlushCache()
        ds = None

    def _write_pgm_yaml(self, img, info, base):
        """Fallback: nav2 map_server-style .pgm + .yaml. Returns the .pgm path."""
        pgm_path = base + ".pgm"
        h, w = img.shape
        with open(pgm_path, "wb") as f:
            f.write(b"P5\n%d %d\n255\n" % (w, h))
            f.write(img.tobytes())
        yaml_path = base + ".yaml"
        with open(yaml_path, "w") as f:
            f.write(
                "image: %s\n"
                "resolution: %f\n"
                "origin: [%f, %f, %f]\n"
                "negate: 0\n"
                "occupied_thresh: 0.65\n"
                "free_thresh: 0.25\n"
                % (os.path.basename(pgm_path), info.resolution,
                   info.origin.position.x, info.origin.position.y, 0.0))
        return pgm_path

    # --- shutdown ----------------------------------------------------------
    def destroy_node(self):
        self._stop_slam()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SlamManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
