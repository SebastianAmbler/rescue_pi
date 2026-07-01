#!/usr/bin/env python3
"""
esp32_bridge.py  —  Raspberry Pi serial/UDP bridge for DDSM115 track drive
runs alongside ROS2 (for arm/flippers) but handles ESP32 comms independently.

UDP IN  (from PC)  port 3390:
    motion packet  8 bytes  struct.pack('ff', linear, angular)
    lock packet    2 bytes  struct.pack('Bb', 0xFF, state)   state=1 lock, 0=release

UDP OUT (to PC)    port 3391:
    JSON string    {"T":3,"id":...,"tor":...,"spd":...,"temp":...,"err":...}\n
    forwarded as-is from ESP32 Serial

Serial (ESP32):
    TX  →  {"T":1,"l":<n>,"r":<n>}\n   or   {"T":2,"lock":<0|1>}\n
    RX  ←  newline-terminated JSON from ESP32
"""

import socket
import serial
import struct
import threading
import json
import sys
import os
import glob
import subprocess
import time

# ═══════════════════════════════════════════════════════════════
#  ★ SET THESE ★
# ═══════════════════════════════════════════════════════════════
ESP32_PORT   = "/dev/esp32hat"
BAUD_RATE    = 115200

LISTEN_IP    = "0.0.0.0"
LISTEN_PORT  = 3390          # UDP in  — PC sends here
FEEDBACK_PORT = 3391         # UDP out — feedback sent back to PC

# Speed scaling: linear/angular floats from PC → integer RPM for DDSM115
# Tune these to match your robot's geometry and desired max speed
MAX_SPEED    = 200           # max RPM (DDSM115 tops out at 200)

# Watchdog: stop motors if no UDP received for this many seconds
WATCHDOG_SEC = 1

# ═══════════════════════════════════════════════════════════════
#  --list helper
# ═══════════════════════════════════════════════════════════════
def _get_port_description(port):
    try:
        result = subprocess.run(
            ['udevadm', 'info', '--name=' + port, '--query=property'],
            capture_output=True, text=True, timeout=2
        )
        props = {}
        for line in result.stdout.splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                props[k] = v
        parts = []
        if 'ID_VENDOR'       in props: parts.append(props['ID_VENDOR'])
        if 'ID_MODEL'        in props: parts.append(props['ID_MODEL'])
        if 'ID_SERIAL_SHORT' in props: parts.append(f"S/N:{props['ID_SERIAL_SHORT']}")
        if parts:
            return ' | '.join(parts)
    except Exception:
        pass
    dev_name = port.split('/')[-1]
    try:
        base = f'/sys/class/tty/{dev_name}/device/..'
        vid = open(f'{base}/idVendor').read().strip()
        pid = open(f'{base}/idProduct').read().strip()
        return f"VID:0x{vid}  PID:0x{pid}"
    except Exception:
        pass
    return "(no description)"

def list_serial_ports():
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    print("\nSerial ports available:")
    print("─" * 62)
    if not ports:
        print("  (none found — is the ESP32 plugged in?)")
    else:
        for port in ports:
            print(f"  {port:<20}  {_get_port_description(port)}")
    print("─" * 62)
    print()


# ═══════════════════════════════════════════════════════════════
#  Bridge
# ═══════════════════════════════════════════════════════════════
class ESP32Bridge:

    MOTION_PACKET_SIZE = 8    # struct.pack('ff', linear, angular)
    LOCK_PACKET_SIZE   = 2    # struct.pack('Bb', 0xFF, state)
    LOCK_MARKER        = 0xFF

    def __init__(self):
        # Serial to ESP32
        print(f"[SERIAL] Opening {ESP32_PORT} @ {BAUD_RATE}...")
        self.ser = serial.Serial(ESP32_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)   # wait for ESP32 to boot
        print("[SERIAL] Connected.")

        # UDP in — receive commands from PC
        self.udp_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_in.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_in.bind((LISTEN_IP, LISTEN_PORT))
        self.udp_in.settimeout(1.0)
        print(f"[UDP IN]  Listening on {LISTEN_IP}:{LISTEN_PORT}")

        # UDP out — send feedback to PC
        self.udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.pc_addr = None   # set on first received packet

        self._is_locked    = False
        self._last_recv    = time.time()
        self._running      = True

    # ── convert linear/angular → left/right RPM ───────────────────────────────
    @staticmethod
    def _twist_to_lr(linear, angular):
        left  = (linear - angular / 2.0) * MAX_SPEED
        right = (linear + angular / 2.0) * MAX_SPEED
        left  = max(-MAX_SPEED, min(MAX_SPEED, int(left)))
        right = max(-MAX_SPEED, min(MAX_SPEED, int(right)))
        return left, right

    # ── send JSON command to ESP32 ────────────────────────────────────────────
    def _send(self, obj):
        line = json.dumps(obj) + '\n'
        self.ser.write(line.encode())

    # ── handle incoming UDP packets ───────────────────────────────────────────
    def _handle_motion(self, data, addr):
        linear, angular = struct.unpack('ff', data)
        left, right = self._twist_to_lr(linear, angular)
        is_stop = left == 0 and right == 0
        if is_stop:
            self.ser.reset_output_buffer()
            print(f"[STOP SENT] {time.time():.3f}")
        self._send({"T": 1, "l": left, "r": right})

    def _handle_lock(self, data, addr):
        marker, state = struct.unpack('Bb', data)
        if marker != self.LOCK_MARKER:
            print(f"[UDP IN]  Bad lock marker from {addr}")
            return
        self._is_locked = bool(state)
        self._send({"T": 2, "lock": int(state)})
        print(f"[LOCK]    {'ENGAGED' if state else 'RELEASED'}")

    # ── UDP receive loop ──────────────────────────────────────────────────────
    def _udp_recv_loop(self):
        while self._running:
            try:
                data, addr = self.udp_in.recvfrom(64)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[UDP IN]  Error: {e}")
                continue

            # Drain any additional packets that arrived in the same burst.
            # Motion packets are coalesced (keep only the latest — this prevents
            # stale motion queuing ahead of a stop), but lock packets must NEVER
            # be dropped: a brake-release sent just before a motion packet would
            # otherwise be discarded, leaving the motors stuck locked.
            batch = [(data, addr)]
            while True:
                try:
                    self.udp_in.setblocking(False)
                    newer, newer_addr = self.udp_in.recvfrom(64)
                    batch.append((newer, newer_addr))
                except BlockingIOError:
                    break
                except Exception:
                    break
                finally:
                    self.udp_in.settimeout(1.0)

            self._last_recv = time.time()

            # Process lock packets in arrival order; defer motion to the latest one
            # so unlock-then-move (and stop-then-brake) sequences apply correctly.
            latest_motion = None
            for pkt, paddr in batch:
                self.pc_addr = (paddr[0], FEEDBACK_PORT)
                if len(pkt) == self.LOCK_PACKET_SIZE:
                    self._handle_lock(pkt, paddr)
                elif len(pkt) == self.MOTION_PACKET_SIZE:
                    latest_motion = (pkt, paddr)
                else:
                    print(f"[UDP IN]  Unknown packet: {len(pkt)} bytes from {paddr}")

            if latest_motion is not None:
                self._handle_motion(*latest_motion)

    # ── watchdog: stop motors if PC goes silent ───────────────────────────────
    def _watchdog_loop(self):
        while self._running:
            time.sleep(0.1)
            if self._is_locked:
                continue
            if time.time() - self._last_recv > WATCHDOG_SEC:
                self._send({"T": 1, "l": 0, "r": 0})

    # ── serial read loop: forward T:3 feedback to PC over UDP ─────────────────
    def _serial_read_loop(self):
        while self._running:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # only forward T:3 feedback packets to PC
                try:
                    obj = json.loads(line)
                    if obj.get("T") == 3 and self.pc_addr:
                        self.udp_out.sendto(line.encode(), self.pc_addr)
                except json.JSONDecodeError:
                    pass

                # always print to console for debugging
                print(f"[ESP32]   {line}")

            except Exception as e:
                if self._running:
                    print(f"[SERIAL]  Read error: {e}")
                break

    # ── start / stop ──────────────────────────────────────────────────────────
    def run(self):
        threads = [
            threading.Thread(target=self._udp_recv_loop,   daemon=True),
            threading.Thread(target=self._watchdog_loop,   daemon=True),
            threading.Thread(target=self._serial_read_loop, daemon=True),
        ]
        for t in threads:
            t.start()

        print("[BRIDGE]  Running. Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()

    def stop(self):
        print("\n[BRIDGE]  Stopping...")
        self._running = False
        # stop motors before closing
        try:
            self._send({"T": 1, "l": 0, "r": 0})
        except Exception:
            pass
        self.ser.close()
        self.udp_in.close()
        self.udp_out.close()
        print("[BRIDGE]  Stopped.")


# ═══════════════════════════════════════════════════════════════
def main():
    if '--list' in sys.argv:
        list_serial_ports()
        return

    if not os.path.exists(ESP32_PORT):
        print(f"\nERROR: Port '{ESP32_PORT}' does not exist.")
        list_serial_ports()
        sys.exit(1)

    ESP32Bridge().run()

if __name__ == '__main__':
    main()
