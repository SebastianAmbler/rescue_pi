# rescue_pi setup status (Claude Code)

Workspace restored at `~/ros2_ws` on this Pi 4 (Ubuntu 22.04, ROS 2 Humble).

## DONE & VERIFIED on the running system (no reboot needed)
- Workspace cloned; submodules pinned: `camera_ros` 0.7.0 (`f4023dc`), `rf2o` (`b38c68e`). `ldlidar_ros2`/`ldlidar_sl_ros2` COLCON_IGNORE'd.
- `UDP.py`/`UDPS.py` copied to `~`; `~/Desktop/Maps`, `~/savedMaps` created.
- ROS/system deps + rosdep installed.
- YDLidar SDK built/installed (`/usr/local/lib/libydlidar_sdk.a`).
- libcamera **0.7.1** (RPi fork, rpi/vc4 + IMX708 tuning) built into `/usr/local`.
- `colcon build` OK — all 5 pkgs. `camera_ros` links `/usr/local/libcamera.so.0.7`.
- **udev** `/etc/udev/rules.d/99-rescue-pi.rules**: ydlidar=CP2102, imu=CH340(1a86:7523), esp32hat=CH9102(1a86:55d3 ser 5B61038521), teensy=16c0:0483. All 4 symlinks resolve. (ESP32 is **USB**, not GPIO.)
- **Full SLAM stack tested**: /scan, /imu/data, /odom_rf2o, /map (after `/slam/start`), TF map→base_link, rosbridge :9090, UDP 3390/3391/3393 — all good.
- `camera-ros.service` installed + **enabled** (auto-starts on boot).

## CAMERA — WORKING ✅ (verified after reboot into kernel 1103)
- imx708 kernel driver → **DKMS** for kernels 1061 & 1103.
- **dw9807-vcm** (autofocus VCM, `dongwoon,dw9817-vcm`) → **DKMS** for both kernels.
  *Critical:* the overlay enables the dw9817 VCM (`lens-focus`); without this driver the
  v4l2-async bind never completes → no `/dev/video0`, libcamera sees nothing. Building +
  loading it created the `imx708→unicam` links and `/dev/video0`.
- `imx708.dtbo` in `/boot/firmware/overlays/`; `dtoverlay=imx708` in `config.txt` (backup `config.txt.bak.*`).
- Both DKMS modules auto-load on boot (OF aliases in `modules.alias`).
- Verified: `cam -l` lists `imx708_noir`; libcamera captures 1920x1080 @ ~30 fps;
  `camera_ros` publishes `/camera/image_raw{,/compressed}` + `/camera/camera_info`;
  MJPEG at `http://<pi>:8000/snapshot.jpg` returns a valid 1920x1080 JPEG.
- Sensor self-reports as **NoIR** (module-ID register bit 0x80) → libcamera uses
  `imx708_noir.json` tuning.
- **Purple/blue shadow cast FIXED**: raised `rpi.black_level` from `4096` → **`4400`** in
  `/usr/local/share/libcamera/ipa/rpi/vc4/imx708_noir.json` (backup `.orig` alongside).
  The NoIR sensor's effective black pedestal sits slightly above the nominal 4096 (16-bit
  scale); the leftover floor was amplified by the AWB blue gain → blue/purple blacks.
  Empirically swept 4096→10000; 4400 makes mid-shadows neutral (blue/G≈1.00, red/G≈0.95)
  and the whole frame neutral, without crushing shadow detail (8000+ clipped blacks to 0).
  Edit that one number and `sudo systemctl restart camera-ros.service` to retune.

## AFTER REBOOT
Run: `bash ~/ros2_ws/verify_stack.sh`
- Expect `cam -l` to list IMX708 and `camera-ros.service` active, `/camera/image_raw/compressed` publishing, MJPEG at http://<pi>:8000/snapshot.jpg.
- If camera NOT found: `sudo dmesg | grep -i imx708`; check overlay applied: `sudo vcdbg log msg 2>&1 | grep -i imx708` or `dmesg | grep -i "imx708\|unicam"`.

## Notes / assumptions to confirm
- Control PC assumed `192.168.1.67` (launch `pc_ip` default); Pi eth0 `192.168.1.167`.
- Passwordless sudo enabled via `/etc/sudoers.d/010-pi-nopasswd`.
- Optional: a `rescue-slam.service` to autostart the SLAM launch on boot is NOT yet installed (SLAM is launched on demand).
