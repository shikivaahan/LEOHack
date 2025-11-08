# run_tests.py
import socket
import threading
import time
import csv
import json
import statistics
import datetime
from collections import deque

# ==== Configuration ====
ARDUINO_IP = "192.168.4.1"
ARDUINO_PORT = 8080
STOP_COMMAND = 's'

# Test plan groups: (name, list of commands to test)
TEST_GROUPS = [
    ("forward_backward", ['f', 'b']),
    ("strafe_left_right", ['a', 'd']),
    ("turn_left_right", ['l', 'r']),
]

POWERS = [8]  # add more if supported

# Recommended: two durations that still allow a line fit and keep runtime < 15 min
DURATIONS_SEC = [0.2, 0.8]

# Recommended: three trials per setting
TRIALS_PER_SETTING = 3

# Stationary wait requirement (10 seconds before and after each trial)
STATIONARY_WAIT_SEC = 10  # keep at 10s

# Sensor stability parameters
STABLE_WINDOW_SAMPLES = 10
STABLE_STD_THRESHOLD_CM = 1.5

# Recommended: reduce stability timeout to cap measurement overhead
STABLE_TIMEOUT_SEC = 2.0

SETTLE_TIME_SEC = 0.3  # after stop, allow sensor to settle

# Break resume word
BREAK_RESUME_WORD = "resume"

# Output files
RUN_TAG = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
CSV_PATH = f"control_tests_{RUN_TAG}.csv"
JSON_PATH = f"control_tests_{RUN_TAG}.json"
EVENTS_PATH = f"control_tests_events_{RUN_TAG}.jsonl"

# ==== Distance receiver ====
class DistanceReceiver:
    def __init__(self, sock):
        self.sock = sock
        self._buf = b""
        self.latest_distance_cm = None
        self._lock = threading.Lock()
        self._recent = deque(maxlen=200)  # (timestamp, distance_cm)
        self._running = True

    def start(self):
        t = threading.Thread(target=self._run, daemon=True)
        t.start()

    def stop(self):
        self._running = False

    def _run(self):
        while self._running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    break
                self._buf += data
                while b"\n" in self._buf:
                    line, self._buf = self._buf.split(b"\n", 1)
                    line = line.decode().strip()
                    if line.startswith("DIST:"):
                        val = line.split(":")[1].strip()
                        ts = time.time()
                        if val != "N/A":
                            try:
                                d = int(val)
                            except ValueError:
                                d = None
                        else:
                            d = None
                        with self._lock:
                            self.latest_distance_cm = d
                            self._recent.append((ts, d))
                        if d is not None:
                            print(f"[DIST] {d} cm")
            except socket.timeout:
                continue
            except Exception as e:
                print("Receive error:", e)
                break

    def get_recent_values(self, max_age_sec=2.0):
        now = time.time()
        with self._lock:
            return [d for (ts, d) in self._recent if d is not None and (now - ts) <= max_age_sec]

    def wait_for_stable(self, timeout_sec=STABLE_TIMEOUT_SEC):
        t0 = time.time()
        while time.time() - t0 < timeout_sec:
            vals = self.get_recent_values(max_age_sec=2.0)
            vals = [v for v in vals if v is not None]
            if len(vals) >= STABLE_WINDOW_SAMPLES:
                std = statistics.pstdev(vals[-STABLE_WINDOW_SAMPLES:])
                if std <= STABLE_STD_THRESHOLD_CM:
                    median = statistics.median(vals[-STABLE_WINDOW_SAMPLES:])
                    return median
            time.sleep(0.05)
        return None

def connect():
    print("Connecting to Arduino...")
    s = socket.socket()
    s.settimeout(3)
    s.connect((ARDUINO_IP, ARDUINO_PORT))
    print("Connected!")
    return s

# Opposite command mapping (return to start)
OPPOSITE_CMD = {
    'f': 'b', 'b': 'f',
    'a': 'd', 'd': 'a',
    'l': 'r', 'r': 'l',
}

def log_event(fp, event_type, details=None):
    rec = {
        "timestamp": datetime.datetime.now().isoformat(),
        "event": event_type,
    }
    if details:
        rec["details"] = details
    fp.write(json.dumps(rec) + "\n")
    fp.flush()

def wait_stationary_and_measure(receiver, label, events_fp, seconds=STATIONARY_WAIT_SEC):
    print(f"[STATIONARY] {label}: leave robot stationary for {seconds} seconds...")
    log_event(events_fp, "stationary_start", {"label": label, "seconds": seconds})
    t_start = time.time()
    remaining = seconds
    # Progress countdown (per second)
    while remaining > 0:
        mins = remaining // 60
        secs = remaining % 60
        print(f"  Time remaining: {int(mins):02d}:{int(secs):02d}", end="\r", flush=True)
        time.sleep(1.0)
        remaining = seconds - int(time.time() - t_start)
    print("\n[STATIONARY] Measuring distance after stationary period...")
    # Try to get a stable reading after the stationary period
    meas = receiver.wait_for_stable(timeout_sec=STABLE_TIMEOUT_SEC)
    log_event(events_fp, "stationary_end", {"label": label, "measured_cm": meas})
    if meas is None:
        print("[WARN] Stable measurement not obtained; using last known distance as fallback.")
        meas = receiver.latest_distance_cm
    print(f"[MEASURE] {label} distance = {meas} cm")
    return meas

def send_and_wait(sock, cmd_str, duration_sec, events_fp, label=None):
    if label:
        print(f"[SEND] {label}: {cmd_str}")
    else:
        print(f"[SEND] {cmd_str}")
    log_event(events_fp, "command_send", {"cmd": cmd_str, "duration_sec": duration_sec, "label": label})
    sock.send(cmd_str.encode())
    time.sleep(duration_sec)
    print(f"[SEND] {STOP_COMMAND}")
    log_event(events_fp, "command_stop", {"label": label})
    sock.send(STOP_COMMAND.encode())
    time.sleep(SETTLE_TIME_SEC)

def run_single_trial(sock, receiver, command, power, duration_sec, group_name, events_fp):
    # Pre-stationary measurement (10 seconds)
    pre_label = f"{group_name}:{command}:pre_stationary"
    pre = wait_stationary_and_measure(receiver, pre_label, events_fp, seconds=STATIONARY_WAIT_SEC)
    if pre is None:
        print("[WARN] Failed to get pre distance (None)")

    # Execute command
    cmd_str = f"{command} {power} {duration_sec}"
    send_and_wait(sock, cmd_str, duration_sec, events_fp, label=f"{group_name}:{command}:forward_move")

    # Return to start with opposite command
    opposite = OPPOSITE_CMD.get(command)
    if opposite:
        opp_cmd_str = f"{opposite} {power} {duration_sec}"
        send_and_wait(sock, opp_cmd_str, duration_sec, events_fp, label=f"{group_name}:{command}:return_move")
    else:
        print(f"[WARN] No opposite command defined for '{command}'")

    # Post-stationary measurement (10 seconds)
    post_label = f"{group_name}:{command}:post_stationary"
    post = wait_stationary_and_measure(receiver, post_label, events_fp, seconds=STATIONARY_WAIT_SEC)
    if post is None:
        print("[WARN] Failed to get post distance (None)")

    rec = {
        "timestamp": datetime.datetime.now().isoformat(),
        "group": group_name,
        "command": command,
        "power": power,
        "duration_sec": duration_sec,
        "pre_distance_cm": pre,
        "post_distance_cm": post,
        "delta_cm": None,  # filled below
    }
    if pre is not None and post is not None:
        rec["delta_cm"] = post - pre
    print(f"[RESULT] group={group_name} cmd={command} power={power} duration={duration_sec}s | pre={pre} cm, post={post} cm, delta={rec['delta_cm']}")
    return rec

def prompt_break_and_resume(events_fp, group_name):
    print(f"\n=== BREAK after group '{group_name}' ===")
    print(f"Reorient/setup as needed. Type the word '{BREAK_RESUME_WORD}' and press Enter to resume.")
    log_event(events_fp, "break_start", {"group": group_name})
    while True:
        try:
            text = input(f"Type '{BREAK_RESUME_WORD}' to continue: ").strip().lower()
        except EOFError:
            text = ""
        if text == BREAK_RESUME_WORD:
            print("[BREAK] Resuming.")
            log_event(events_fp, "break_resume", {"group": group_name})
            break
        else:
            print(f"[BREAK] Invalid input. Please type '{BREAK_RESUME_WORD}'.")

def estimate_runtime_minutes():
    # Approximate estimate including stationary waits, movements (uses max duration),
    # and settle time. Measurement stabilization time varies; capped by STABLE_TIMEOUT_SEC.
    per_trial_sec = (2 * STATIONARY_WAIT_SEC) + (2 * max(DURATIONS_SEC)) + (2 * SETTLE_TIME_SEC)
    # If stabilization often hits the timeout, add its cap here:
    per_trial_sec += (2 * min(STABLE_TIMEOUT_SEC, 1.0))  # assume ~1s average each pre/post
    total_trials = sum(len(cmds) for _, cmds in TEST_GROUPS) * len(POWERS) * len(DURATIONS_SEC) * TRIALS_PER_SETTING
    breaks_sec = 40  # conservative budget for two quick breaks
    return ((per_trial_sec * total_trials) + breaks_sec) / 60.0

def main():
    s = None
    all_records = []
    events_fp = None
    try:
        s = connect()
        receiver = DistanceReceiver(s)
        receiver.start()
        time.sleep(0.5)  # warm-up

        events_fp = open(EVENTS_PATH, "w")
        log_event(events_fp, "session_start", {"run_tag": RUN_TAG})

        est_min = estimate_runtime_minutes()
        print(f"[INFO] Estimated run time ~ {est_min:.1f} minutes with current settings")

        for group_name, commands in TEST_GROUPS:
            print(f"\n=== GROUP START: {group_name} ===")
            log_event(events_fp, "group_start", {"group": group_name, "commands": commands})

            for command in commands:
                for power in POWERS:
                    for duration_sec in DURATIONS_SEC:
                        for trial in range(1, TRIALS_PER_SETTING + 1):
                            print(f"\n--- {group_name} | {command} power={power} duration={duration_sec}s trial={trial}/{TRIALS_PER_SETTING} ---")
                            log_event(events_fp, "trial_start", {
                                "group": group_name,
                                "command": command,
                                "power": power,
                                "duration_sec": duration_sec,
                                "trial": trial
                            })
                            rec = run_single_trial(s, receiver, command, power, duration_sec, group_name, events_fp)
                            if rec is not None:
                                rec["trial"] = trial
                                all_records.append(rec)
                                log_event(events_fp, "trial_end", {"group": group_name, "command": command, "trial": trial})
                            else:
                                print("[WARN] Trial returned no record.")
                                log_event(events_fp, "trial_error", {"group": group_name, "command": command, "trial": trial})
                                # Attempt to stop anyway
                                try:
                                    s.send(STOP_COMMAND.encode())
                                except Exception:
                                    pass

            # BREAK after each group
            prompt_break_and_resume(events_fp, group_name)
            log_event(events_fp, "group_end", {"group": group_name})

        # Save CSV
        fieldnames = ["timestamp", "group", "command", "power", "duration_sec", "trial", "pre_distance_cm", "post_distance_cm", "delta_cm"]
        with open(CSV_PATH, "w", newline=True) as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            for r in all_records:
                w.writerow({k: r.get(k) for k in fieldnames})

        # Save JSON
        with open(JSON_PATH, "w") as f:
            json.dump(all_records, f, indent=2)

        log_event(events_fp, "session_end", {"run_tag": RUN_TAG})
        print(f"\n[DONE] Saved {len(all_records)} records to:")
        print(f"  CSV:   {CSV_PATH}")
        print(f"  JSON:  {JSON_PATH}")
        print(f"  EVENTS:{EVENTS_PATH}")

    except Exception as e:
        print("Connection/run failed:", e)
    finally:
        if events_fp is not None:
            try:
                events_fp.close()
            except Exception:
                pass
        if s is not None:
            try:
                s.send(STOP_COMMAND.encode())
            except Exception:
                pass
            try:
                s.close()
            except Exception:
                pass

if __name__ == "__main__":
    main()