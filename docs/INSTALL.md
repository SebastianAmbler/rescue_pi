# rescue_pi — full setup & reproduction guide

Everything needed to rebuild this robot from a fresh OS install. Target hardware/OS:

- **Raspberry Pi 4 Model B (4/8 GB)**
- **Ubuntu 22.04 (aarch64)**, kernel `5.15.0-1103-raspi` (also built for `-1061`)
- **ROS 2 Humble**

The workspace itself (`~/ros2_ws`) is this repo. Generated `build/`, `install/`, and
`log/` directories are **not** tracked — regenerate them with `deps/do_colcon.sh`.

Sensors / boards (all USB serial, stable names via udev — see below):

| Device                         | Chip            | VID:PID       | udev symlink   |
|--------------------------------|-----------------|---------------|----------------|
| YDLidar T-mini Plus            | CP2102          | `10c4:ea60`   | `/dev/ydlidar` |
| WitMotion BWT901CL IMU         | CH340           | `1a86:7523`   | `/dev/imu`     |
| ESP32 track-drive board        | CH9102          | `1a86:55d3`   | `/dev/esp32hat`|
| Teensy servo controller        | Teensy          | `16c0:0483`   | `/dev/teensy`  |
| Raspberry Pi Camera Module 3 NoIR (IMX708 + autofocus) | CSI | — | `/dev/video0`  |

---

## 0. Base system

```bash
sudo apt update && sudo apt upgrade -y
# ROS 2 Humble desktop (follow official ROS docs if not already installed)
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions \
                    python3-rosdep git build-essential cmake
sudo rosdep init 2>/dev/null; rosdep update
```

Passwordless sudo (used by helper scripts) — optional:
```bash
echo "pi ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010-pi-nopasswd
sudo chmod 440 /etc/sudoers.d/010-pi-nopasswd
```

---

## 1. Clone the workspace

```bash
git clone https://github.com/SebastianAmbler/rescue_pi.git ~/ros2_ws
cd ~/ros2_ws
```

The ROS packages under `src/` are **vendored** (committed directly), so no submodule
init is needed. Upstream sources, for reference:

| Package                       | Upstream                                            | Pinned commit |
|-------------------------------|-----------------------------------------------------|---------------|
| `camera_ros` (v0.7.0)         | https://github.com/christianrauch/camera_ros        | `f4023dc`     |
| `rf2o_laser_odometry`         | https://github.com/MAPIRlab/rf2o_laser_odometry     | `b38c68e`     |
| `ldlidar_ros2` *(COLCON_IGNORE'd — unused)* | https://github.com/ldrobotSensorTeam/ldlidar_ros2 | `0f6101c` |
| `ldlidar_sl_ros2` *(COLCON_IGNORE'd — unused)* | https://github.com/ldrobotSensorTeam/ldlidar_sl_ros2 | `d70802a` |
| `ydlidar_ros2_driver-humble`  | YDLIDAR ROS2 driver (Humble branch)                 | vendored      |

We drive the lidar with `ydlidar_ros2_driver` (needs the YDLidar SDK, step 2), not the
`ldlidar_*` packages.

Install ROS dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 2. YDLidar SDK (CMake build → `/usr/local`)

Required by `ydlidar_ros2_driver`.

```bash
cd ~/build_deps          # or anywhere
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK           # pinned/tested at commit 6bd7763
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install        # installs /usr/local/lib/libydlidar_sdk.a + headers
sudo ldconfig
```

---

## 3. libcamera — Raspberry Pi fork (needed by `camera_ros`)

Ubuntu's stock libcamera does **not** carry the RPi vc4 pipeline or IMX708 tuning.
Build the RPi fork into `/usr/local`.

```bash
sudo apt install -y meson ninja-build pkg-config libyaml-dev python3-yaml \
     python3-ply python3-jinja2 libgnutls28-dev openssl libboost-dev \
     libevent-dev libdrm-dev

cd ~/build_deps
git clone https://github.com/raspberrypi/libcamera.git
cd libcamera             # pinned/tested at commit 06c3856 ("Merge branch 'master' into next")
meson setup build --buildtype=release \
      -Dpipelines=rpi/vc4 -Dipas=rpi/vc4 \
      -Dv4l2=true -Dgstreamer=disabled -Dtest=false -Dlc-compliance=disabled \
      -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled \
      --prefix=/usr/local
ninja -C build
sudo ninja -C build install
sudo ldconfig
```

### 3a. Camera colour tuning (purple/blue-shadow fix)

The NoIR sensor's effective black pedestal sits above the nominal 4096, so residual
floor was amplified by AWB blue gain → blue/purple blacks. Fix: raise `rpi.black_level`
from `4096` → **`4400`** in the NoIR tuning file. The corrected file is backed up here:

```bash
# backup first, then install the tuned file from this repo
sudo cp /usr/local/share/libcamera/ipa/rpi/vc4/imx708_noir.json{,.orig}
sudo cp ~/ros2_ws/system/libcamera/imx708_noir.json \
        /usr/local/share/libcamera/ipa/rpi/vc4/imx708_noir.json
sudo systemctl restart camera-ros.service   # after step 6
```

To retune, edit only that one `black_level` number and restart the service.
(`system/libcamera/imx708_noir.json.orig` is the unmodified upstream file.)

---

## 4. IMX708 camera driver **with autofocus** (kernel modules via DKMS) ⭐

This is the fiddly part. The Camera Module 3 needs **two** kernel modules that aren't in
the stock `-raspi` kernel on this Ubuntu image:

1. **`imx708`** — the Sony IMX708 sensor driver.
2. **`dw9807-vcm`** — the Dongwoon VCM (voice-coil motor) **autofocus** driver.

> **Critical:** the device-tree overlay wires the sensor to the `dw9817` VCM via
> `lens-focus`. Without the `dw9807-vcm` module loaded, the v4l2-async bind never
> completes → **no `/dev/video0`** and libcamera sees nothing. Building + loading the VCM
> driver is what creates the `imx708 → unicam` media links and `/dev/video0`.

Sources are in `deps/kernel/` (`imx708/` and `dw9807-vcm/`), each with its `dkms.conf`.

```bash
sudo apt install -y dkms linux-headers-$(uname -r) device-tree-compiler

# imx708 sensor
sudo cp -r ~/ros2_ws/deps/kernel/imx708       /usr/src/imx708-1.0
sudo dkms add    -m imx708 -v 1.0
sudo dkms build  -m imx708 -v 1.0
sudo dkms install -m imx708 -v 1.0

# dw9807-vcm autofocus
sudo cp -r ~/ros2_ws/deps/kernel/dw9807-vcm   /usr/src/dw9807-vcm-1.0
sudo dkms add    -m dw9807-vcm -v 1.0
sudo dkms build  -m dw9807-vcm -v 1.0
sudo dkms install -m dw9807-vcm -v 1.0
```

DKMS rebuilds both automatically on every kernel update (`AUTOINSTALL="yes"`), and the
OF aliases in `modules.alias` auto-load them at boot. Verify:
```bash
dkms status          # expect imx708/1.0 and dw9807-vcm/1.0 "installed" for your kernel(s)
```

### 4a. Device-tree overlay

The compiled overlay `deps/overlays/imx708.dtbo` (source: `imx708-overlay.dts` +
`imx708.dtsi`, which sets `dongwoon,dw9817-vcm` at `0x0c` with `lens-focus`) goes into the
firmware overlays directory:

```bash
sudo cp ~/ros2_ws/deps/overlays/imx708.dtbo /boot/firmware/overlays/imx708.dtbo
```

### 4b. `config.txt`

Enable the overlay in `/boot/firmware/config.txt` (full reference copy at
`system/boot/config.txt`). The key line:

```ini
dtoverlay=imx708
```

`camera_auto_detect=1` may stay on; the explicit overlay is what pulls in our DKMS pair.
Back up before editing: `sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.bak`

**Reboot** after installing the overlay + config change.

---

## 5. udev rules (stable device names)

```bash
sudo cp ~/ros2_ws/system/udev/99-rescue-pi.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/ydlidar /dev/imu /dev/esp32hat /dev/teensy   # all 4 should resolve
```

> Note: the ESP32 rule is keyed by serial `5B61038521`. If you swap the ESP32 board,
> update the `ATTRS{serial}` in the rule (`system/udev/99-rescue-pi.rules`).

---

## 6. Build the ROS workspace

```bash
cd ~/ros2_ws
bash deps/do_colcon.sh      # sources ROS, points CMAKE/PKG_CONFIG at /usr/local, colcon build
```

This builds all 5 active packages; `camera_ros` links against
`/usr/local/lib/.../libcamera.so.0.7`.

---

## 7. Camera service (auto-start on boot)

```bash
sudo cp ~/ros2_ws/system/systemd/camera-ros.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now camera-ros.service
```

Publishes `/camera/image_raw{,/compressed}` + `/camera/camera_info`, and an MJPEG stream
at `http://<pi>:8000/snapshot.jpg`.

---

## 8. Verify

```bash
bash ~/ros2_ws/verify_stack.sh
cam -l                       # should list "imx708_noir"
```

Expected: `cam -l` lists the IMX708, `camera-ros.service` active,
`/camera/image_raw/compressed` publishing, MJPEG returns a valid 1920x1080 JPEG.

If the camera is **not** found after reboot:
```bash
sudo dmesg | grep -i "imx708\|unicam\|dw9807\|dw9817"
dkms status
ls -l /dev/video0
```

---

## Network / SLAM notes

- Control PC assumed `192.168.1.67` (launch arg `pc_ip` default); Pi `eth0` `192.168.1.167`.
- Full SLAM stack (tested): `/scan`, `/imu/data`, `/odom_rf2o`, `/map` (after `/slam/start`),
  TF `map→base_link`, rosbridge `:9090`, UDP `3390/3391/3393`.
- SLAM is launched on demand (no `rescue-slam.service` autostart installed).
- Telemetry / control bridges: `UDP.py`, `UDPS.py` (in repo root).
- Saved maps are in `savedMaps/` (`.pgm` + `.yaml` pairs).

See `SETUP_STATUS.md` for the as-verified running-system state.
