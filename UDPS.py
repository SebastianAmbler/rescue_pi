#!/usr/bin/env python3
import socket
import serial
import glob
import subprocess
import sys
import os
import time
import threading


# ═══════════════════════════════════════════════════════════════
#  ★ SET THESE MANUALLY ★
# ═══════════════════════════════════════════════════════════════
MEGA_PORT  = "/dev/teensy"
BAUD_RATE  = 115200

# ── UDP ───────────────────────────────────────────────────────
LISTEN_IP   = "0.0.0.0"
LISTEN_PORT = 3391

# ── Angles readback ───────────────────────────────────────────
PC_ANGLES_PORT = 3392

# ── Local arm-angle tap ───────────────────────────────────────
# Arm angle lines are also forwarded to localhost here so the arm_home_guard
# ROS node can pause SLAM while the arm is out of its home pose. Independent of
# PC discovery (always on) — the serial port can only be opened once, by us.
LOCAL_ANGLES_IP   = "127.0.0.1"
LOCAL_ANGLES_PORT = 3393

# ── Packet constants ──────────────────────────────────────────
SERVO_MARKER = 0xAA


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
    return "(no description — try: udevadm info --name=" + port + ")"


def list_serial_ports():
    ports = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
    print()
    print("Serial ports currently available on this Pi:")
    print("─" * 62)
    if not ports:
        print("  (none found — is the Arduino Mega plugged in?)")
    else:
        for port in ports:
            desc = _get_port_description(port)
            tag = "  ← MEGA_PORT" if port == MEGA_PORT else ""
            print(f"  {port:<20}  {desc}{tag}")
    print("─" * 62)
    print()


# ═══════════════════════════════════════════════════════════════
#  Arduino Mega serial connection
#
#  FIX: Separate read and write locks so readline() in the read
#  thread never blocks send() in the UDP thread, and vice versa.
#  Previously both shared one lock — send() holding it would
#  stall readline(), causing angles to queue up or be dropped.
# ═══════════════════════════════════════════════════════════════
class MegaSerial:
    def __init__(self):
        self._ser       = None
        self._write_lock = threading.Lock()   # guards serial writes only
        self._ser_lock   = threading.Lock()   # guards _ser assignment
        if not self._connect():
            print(f"[MEGA]  WARN: {MEGA_PORT} not found — servo packets will be dropped, will keep retrying")

    def _connect(self):
        if not os.path.exists(MEGA_PORT):
            return False
        try:
            ser = serial.Serial(MEGA_PORT, BAUD_RATE, timeout=1)
            time.sleep(1.5)
            with self._ser_lock:
                self._ser = ser
            print(f"[MEGA]  OK → {MEGA_PORT} @ {BAUD_RATE}")
            return True
        except serial.SerialException as e:
            print(f"[MEGA]  ERROR: {e}")
            return False

    def send(self, json_bytes: bytes):
        """Forward JSON + newline to Mega serial (write-lock only)."""
        with self._write_lock:
            ser = self._ser
            if ser is None:
                print("[MEGA]  WARN: not connected — packet dropped")
                return
            try:
                ser.write(json_bytes + b'\n')
            except serial.SerialException as e:
                print(f"[MEGA]  ERROR write: {e}")
                with self._ser_lock:
                    self._ser = None

    def start_read_loop(self, forward_sock, pc_addr):
        """Background thread: read lines from Mega, forward ANGLES/DATA to PC."""
        threading.Thread(
            target=self._read_loop,
            args=(forward_sock, pc_addr),
            daemon=True
        ).start()

    def _read_loop(self, forward_sock, pc_addr):
        """
        FIX: readline() runs completely independently of send().
        No shared lock means writes never stall reads.
        """
        print(f"[MEGA]  Read loop started — will forward ANGLES:/DATA:/typed-JSON lines to PC:{PC_ANGLES_PORT}")
        last_reconnect_attempt = 0
        while True:
            try:
                with self._ser_lock:
                    ser = self._ser

                if ser is None:
                    now = time.monotonic()
                    if now - last_reconnect_attempt >= 2.0:
                        last_reconnect_attempt = now
                        if self._connect():
                            print("[MEGA]  reconnected")
                    time.sleep(0.1)
                    continue

                # readline() blocks up to serial timeout (1 s) — no lock held
                line = ser.readline().decode(errors='ignore').strip()

                if not line:
                    continue

                # ── DEBUG: print every line from Mega so you can see what it sends ──
                print(f"[MEGA RX] {repr(line)}")

                # ── Local tap: forward arm-angle lines to the SLAM arm guard ──
                # (arm_home_guard pauses SLAM while the arm is out of home).
                if line.startswith('{"type":"angles"') or line.startswith("ANGLES:"):
                    try:
                        forward_sock.sendto(
                            line.encode(), (LOCAL_ANGLES_IP, LOCAL_ANGLES_PORT))
                    except OSError:
                        pass

                # Forward the legacy ANGLES:/DATA: readback AND the new typed
                # JSON broadcasts ({"type":"angles"|"compass"|"thermal",...}) the
                # Teensy now emits. Everything else (boot/IK debug) stays local.
                if (line.startswith("ANGLES:") or line.startswith("DATA:")
                        or line.startswith('{"type":')):
                    if pc_addr[0] is not None:
                        dest = (pc_addr[0], PC_ANGLES_PORT)
                        forward_sock.sendto(line.encode(), dest)
                        print(f"[FWD]   → {dest}  {line}")
                    else:
                        print(f"[FWD]   WARN: PC not yet discovered, dropping: {line}")
                else:
                    # All other Mega output (boot messages, IK debug, errors…)
                    print(f"[MEGA]  {line}")

            except serial.SerialException as e:
                print(f"[MEGA]  serial error in read loop: {e}")
                with self._ser_lock:
                    self._ser = None
                time.sleep(0.5)
            except Exception as e:
                print(f"[MEGA]  unexpected read error: {e}")
                time.sleep(0.1)

    def close(self):
        with self._ser_lock:
            if self._ser:
                self._ser.close()


# ═══════════════════════════════════════════════════════════════
#  UDP listener
# ═══════════════════════════════════════════════════════════════
def run(mega: MegaSerial):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.settimeout(1.0)

    fwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pc_addr  = [None]   # shared with read loop thread

    mega.start_read_loop(fwd_sock, pc_addr)

    print(f"[UDP]   Listening on {LISTEN_IP}:{LISTEN_PORT}")
    print(f"[UDP]   PC IP auto-discovered → angles sent to :{PC_ANGLES_PORT}")
    print()

    while True:
        try:
            data, addr = sock.recvfrom(512)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[UDP]   recv error: {e}")
            continue

        if not data:
            continue

        if pc_addr[0] != addr[0]:
            pc_addr[0] = addr[0]
            print(f"[UDP]   PC discovered → {addr[0]}")

        if data[0] == SERVO_MARKER and len(data) > 1:
            json_bytes = data[1:]
            mega.send(json_bytes)
            print(f"[MEGA TX] → {json_bytes.decode(errors='replace')}")

    sock.close()
    fwd_sock.close()


# ═══════════════════════════════════════════════════════════════
def main():
    if '--list' in sys.argv:
        list_serial_ports()
        return

    mega = MegaSerial()

    try:
        run(mega)
    except KeyboardInterrupt:
        pass
    finally:
        mega.close()
        print("Mega bridge stopped.")


if __name__ == '__main__':
    main()
