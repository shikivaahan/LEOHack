import socket
import time
import keyboard  # pip install keyboard
import threading
try:
    from autopilot import run_autopilot
except Exception:
    run_autopilot = None

ARDUINO_IP = "192.168.4.1"
ARDUINO_PORT = 8080

# Default values setup
power = 8
duration = 1
stop_command = 's'

# Mapping the movements of keys to Arduino command
key_to_command = {
    'w': f'f {power} {duration}',  # forward
    's': f'b {power} {duration}',  # backward
    'a': f'a {power} {duration}',  # left strafe
    'd': f'd {power} {duration}',  # right strafe
    'q': f'l {power} {duration}',  # turn left
    'e': f'r {power} {duration}',  # turn right
    'o': 'o',                      # servo open
    'p': 'p',                      # servo close
}

autopilot_enabled = False
pressed_keys = set()
autopilot_enabled = False
latest_distance = None

# autopilot thread control
autopilot_thread = None
autopilot_stop_event = None


def receive_data(sock):
    """Continuously receive data from Arduino."""
    global latest_distance
    buffer = b""
    while True:
        try:
            data = sock.recv(1024)
            if not data:
                break
            buffer += data
            # Split by newlines
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line.decode().strip()
                if line.startswith("DIST:"):
                    try:
                        val = line.split(":")[1]
                        if val != "N/A":
                            latest_distance = int(val)
                            print(f"[DIST] {latest_distance} cm")
                        else:
                            latest_distance = None
                    except ValueError:
                        pass
        except socket.timeout:
            continue
        except Exception as e:
            print("Receive error:", e)
            break


def get_distance():
    """Callable used by autopilot: return latest_distance or None."""
    return latest_distance


def get_skew():
    """Callable used by autopilot: return None unless you wire a camera/skew estimator."""
    return None


# === Main Control ===
print("Connecting to Arduino...")

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
                # manual override: stop autopilot if running
                if autopilot_thread is not None and autopilot_stop_event is not None:
                    print("[MANUAL] Manual input detected - stopping autopilot")
                    try:
                        autopilot_stop_event.set()
                        autopilot_thread.join(timeout=1.0)
                    except Exception:
                        pass
                    autopilot_thread = None
                autopilot_enabled = False
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
            # debounce
            time.sleep(0.1)
            if run_autopilot is None:
                print("Autopilot module not available (can't import).")
            else:
                if autopilot_thread is None:
                    print("Starting autopilot...")
                    autopilot_stop_event = threading.Event()
                    # run_autopilot is blocking so run it in a thread
                    def _run():
                        try:
                            # keep pushing after dock to hold contact (until stop_event set)
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
            print("Exiting...")
            break

        time.sleep(0.05)

    s.close()

except Exception as e:
    print("Connection failed:", e)
