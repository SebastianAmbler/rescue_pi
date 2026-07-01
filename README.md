# rescue_pi

Raspberry Pi 4 side code for the RoboCup RMRC (Rescue) competition, Incheon.

ROS 2 Humble workspace for a tracked rescue robot: YDLidar T-mini Plus + WitMotion IMU
+ rf2o laser odometry → SLAM, a Raspberry Pi Camera Module 3 NoIR (IMX708, **autofocus**)
video stack, and UDP telemetry/control bridges to the operator PC.

## Repo layout

| Path                | What it is |
|---------------------|------------|
| `src/`              | ROS 2 packages (vendored — camera_ros, rf2o, ydlidar driver, slam bringup, camera_bridge) |
| `deps/`             | Things that had to be built outside colcon: kernel driver sources (`kernel/imx708`, `kernel/dw9807-vcm`) + DKMS confs, device-tree overlays (`overlays/`), and `do_colcon.sh` |
| `system/`           | Config that lives outside the workspace: udev rules, `boot/config.txt`, systemd service, tuned libcamera `imx708_noir.json` |
| `savedMaps/`        | Saved SLAM maps (`.pgm` + `.yaml`) |
| `UDP.py`, `UDPS.py` | Telemetry / control bridges |
| `docs/INSTALL.md`   | **Full from-scratch setup & reproduction guide** |
| `SETUP_STATUS.md`   | As-verified state of the running system |

## Rebuild from scratch

See **[`docs/INSTALL.md`](docs/INSTALL.md)** — covers ROS deps, the YDLidar SDK and
Raspberry Pi libcamera fork (CMake/meson builds into `/usr/local`), the IMX708 + dw9807
autofocus kernel modules via **DKMS**, the device-tree overlay, udev rules, camera colour
tuning, and the systemd service.

Generated `build/`, `install/`, and `log/` are intentionally **not** tracked — regenerate
with `bash deps/do_colcon.sh`.

## Quick start (already-provisioned Pi)

```bash
cd ~/ros2_ws
bash deps/do_colcon.sh          # build
bash verify_stack.sh            # sanity-check sensors + camera
```
