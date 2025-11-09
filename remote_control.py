# remote_control.py — launches qr_stream_app.py, ingests pose (yaw + distance), and drives the Arduino

import socket
import time
import keyboard  # pip install keyboard
import threading
import json
import subprocess, sys, atexit
from pathlib import Path

try:
    from autopilot import run_autopilot
except Exception:
    run_autopilot = None

# ----- Arduino connection -----
ARDUINO_IP = "192.168.4.1"
ARDUINO_PORT = 8080

# ----- Pose process settings -----
POSE_UDP_HOST = "127.0.0.1"
POSE_UDP_PORT = 5005
POSE_URL = "http://192.168.4.4/capture"   # your ESP32 snapshot endpoint
POSE_TAG_SIZE_M = 0.10
POSE_SHOW_WINDOW = True  # set False to hide camera window
POSE_POLL_FPS = 10       # snapshot poll rate

# Path to qr_stream_app.py (repo root -> src/qr_stream_app.py)
THIS_DIR = Path(__file__).resolve().parent
POSE_SCRIPT_PATH = THIS_DIR / "src" / "qr_stream_app.py"
# If this file also lives in src/, use: POSE_SCRIPT_PATH = THIS_DIR / "qr_stream_app.py"

# ----- Manual driving defaults -----
power = 8
duration = 1
stop_command = 's'

key_to_command = {
    'w': f'f {power} {duration}',
    's': f'b {power} {duration}',
    'a': f'a {power} {duration}',
    'd': f'd {power} {duration}',
    'q': f'l {power} {duration}',
    'e': f'r {power} {duration}',
    'o': 'o',
    'p': 'p',
}

autopilot_enabled = False
pressed_keys = set()

# latest readings
latest_distance_ultra = None          # from Arduino (cm)
latest_skew_deg = None                # from camera yaw
latest_cam_dist_cm = None             # from camera distance_m * 100
_skew_lock = threading.Lock()

# autopilot thread control
autopilot_thread = None
autopilot_stop_event = None

# ----- Pose subprocess management -----
_pose_proc = None

def start_pose_process():
    """Launch qr_stream_app.py with UDP output to our listener."""
    global _pose_proc
    if _pose_proc and _pose_proc.poll() is None:
        return  # already running
    if not POSE_SCRIPT_PATH.exists():
        raise FileNotFoundError(f"qr_stream_app.py not found at: {POSE_SCRIPT_PATH}")

    args = [
        sys.executable, str(POSE_SCRIPT_PATH),
        "--url", POSE_URL,
        "--no-record",
        "--tag-size", str(POSE_TAG_SIZE_M),
        "--udp", f"{POSE_UDP_HOST}:{POSE_UDP_PORT}",
        "--poll-fps", str(POSE_POLL_FPS),
    ]
    args.append("--display" if POSE_SHOW_WINDOW else "--no-display")

    _pose_proc = subprocess.Popen(args)
    print(f"[POSE] Launched: {' '.join(args)} (pid={_pose_proc.pid})")

def stop_pose_process():
    """Terminate the qr_stream_app.py child process."""
    global _pose_proc
    if _pose_proc and _pose_proc.poll() is None:
        print("[POSE] Stopping pose process…")
        try:
            _pose_proc.terminate()
            try:
                _pose_proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                _pose_proc.kill()
        except Exception:
            pass
    _pose_proc = None

atexit.register(stop_pose_process)

# ----- Arduino RX (distance) -----
def receive_data(sock):
    """Continuously receive data from Arduino (distance)."""
    global latest_distance_ultra
    buffer = b""
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                break
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line.decode().strip()
                if line.startswith("DIST:"):
                    try:
                        val = line.split(":")[1]
                        if val != "N/A":
                            latest_distance_ultra = int(val)
                            print(f"[DIST] {latest_distance_ultra} cm")
                        else:
                            latest_distance_ultra = None
                    except ValueError:
                        pass
        except socket.timeout:
            continue
        except Exception as e:
            print("Receive error:", e)
            break

# ----- UDP listener for pose (yaw + distance) -----
def pose_udp_listener(host: str, port: int):
    """
    Listens for JSON from qr_stream_app.py (with --udp host:port).
    Expects keys: seen (bool), yaw_deg (float), distance_m (float).
    """
    global latest_skew_deg, latest_cam_dist_cm
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    print(f"[POSE] Listening on UDP {host}:{port}")
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            msg = json.loads(data.decode("utf-8"))
            with _skew_lock:
                if msg.get("seen"):
                    # yaw -> skew
                    yd = msg.get("yaw_deg", None)
                    latest_skew_deg = float(yd) if isinstance(yd, (int, float)) else None
                    # camera distance (m -> cm)
                    dm = msg.get("distance_m", None)
                    latest_cam_dist_cm = float(dm) * 100.0 if isinstance(dm, (int, float)) else None
                else:
                    latest_skew_deg = None
                    latest_cam_dist_cm = None
        except Exception:
            # ignore malformed packets and keep listening
            pass

# ----- Autopilot hooks -----
def get_distance():
    """Prefer camera distance (cm), fallback to ultrasonic."""
    with _skew_lock:
        if latest_cam_dist_cm is not None:
            return int(latest_cam_dist_cm)
    return latest_distance_ultra

def get_skew():
    """Return skew angle in degrees (camera yaw). None if not seen."""
    with _skew_lock:
        return latest_skew_deg

# ===== Main =====
print("Starting pose UDP listener thread…")
threading.Thread(target=pose_udp_listener, args=(POSE_UDP_HOST, POSE_UDP_PORT), daemon=True).start()

print("Launching QR stream process…")
start_pose_process()

print("Connecting to Arduino…")
try:
    s = socket.socket()
    s.settimeout(3)
    s.connect((ARDUINO_IP, ARDUINO_PORT))
    print("Connected!")
    print("Hold WASD/QE to move. Release to stop. Press 'X' to toggle autopilot. ESC to exit.")

    # Start receiver thread
    threading.Thread(target=receive_data, args=(s,), daemon=True).start()

    while True:
        # Keyboard control
        for key in key_to_command.keys():
            if keyboard.is_pressed(key) and key not in pressed_keys:
                pressed_keys.add(key)
                cmd = key_to_command[key]
                # Manual override: stop autopilot if running
                if autopilot_thread is not None and autopilot_stop_event is not None:
                    print("[MANUAL] Manual input detected - stopping autopilot")
                    try:
                        autopilot_stop_event.set()
                        autopilot_thread.join(timeout=1.0)
                    except Exception:
                        pass
                    autopilot_thread = None
                print(f"[MANUAL] {cmd}")
                s.send(cmd.encode())

        # Stop when key released
        keys_to_remove = []
        for key in list(pressed_keys):
            if not keyboard.is_pressed(key):
                print(f"[MANUAL] Stop ({key} released)")
                s.send(stop_command.encode())
                keys_to_remove.append(key)
        for key in keys_to_remove:
            pressed_keys.remove(key)

        # Toggle autopilot
        if keyboard.is_pressed('x'):
            time.sleep(0.1)  # debounce
            if run_autopilot is None:
                print("Autopilot module not available (can't import).")
            else:
                if autopilot_thread is None:
                    print("Starting autopilot...")
                    autopilot_stop_event = threading.Event()
                    def _run():
                        try:
                            # run_autopilot is blocking so run it in a thread
                            # signature: run_autopilot(sock, get_distance, get_skew, stop_event, ...)
                            run_autopilot(s, get_distance, get_skew, autopilot_stop_event, keep_pushing_after_dock=True)
                        except Exception as e:
                            print("Autopilot error:", e)
                    autopilot_thread = threading.Thread(target=_run, daemon=True)
                    autopilot_thread.start()
                else:
                    print("Stopping autopilot...")
                    try:
                        autopilot_stop_event.set()
                        autopilot_thread.join(timeout=1.0)
                    except Exception:
                        pass
                    autopilot_thread = None
            time.sleep(0.5)  # debounce

        # Exit
        if keyboard.is_pressed('esc'):
            print("Exiting…")
            break

        # (Optional) heartbeat showing skew & distances
        if int(time.time() * 10) % 10 == 0:
            with _skew_lock:
                dbg_skew = f"{latest_skew_deg:+.1f}°" if latest_skew_deg is not None else "N/A"
                dbg_cam_d = f"{latest_cam_dist_cm:.0f} cm" if latest_cam_dist_cm is not None else "N/A"
            print(f"[DBG] skew={dbg_skew} cam_dist={dbg_cam_d} ultra={latest_distance_ultra}", end="\r")

        time.sleep(0.05)

    s.close()

except Exception as e:
    print("Connection failed:", e)
finally:
    stop_pose_process()
