"""
Autopilot state machine for docking.

Usage:
- Call run_autopilot(sock, get_distance, get_skew, stop_event) to run.

Assumptions:
- get_distance() -> int|None (cm)
- get_skew() -> float|None (deg)
- sock is a connected socket to the Arduino and send_cmd uses the same encoding as remote_control.py (no newline)

Commands understood by the Arduino (per project convention):
- 'f P T' : forward at power P for time T (P:0-9, T:seconds single digit)
- 'b P T' : backward
- 'l P T' : rotate left
- 'r P T' : rotate right
- 'a P T' / 'd P T' : translate left/right (if available)
- 's' : stop
- 'o' / 'p' : servo open/close (dock latch)

This module implements a simple state machine: initial_advance -> sweep_search -> align -> approach -> dock
"""

import time
import threading
from typing import Callable, Optional

# ------ PARAMETERS (tune these for your robot) ------
ULTRASONIC_RANGE_CM = 150        # sensor effective range (cm)
DOCKING_DISTANCE_CM = 15         # stop/dock when <= this
ALIGN_TOLERANCE_DEG = 5          # acceptable skew to consider aligned
INITIAL_ADVANCE_STEP_SEC = 1.0   # small forward pulses during initial advance
SWEEP_TURN_POWER = 4
SWEEP_TURN_TIME = 1              # seconds per sweep turn step
SWEEP_MAX_STEPS = 10             # how many sweep steps before giving up
APPROACH_FORWARD_POWER = 3
APPROACH_STEP_TIME = 1           # seconds per forward pulse during approach
APPROACH_SLOW_THRESHOLD = 40     # below this, slow down
APPROACH_SLOW_POWER = 2
# --------------------------------------------------


def send_cmd(sock, cmd: str):
    """Send a command string to the Arduino. Does NOT add newline (keeps parity with remote_control.py)."""
    try:
        sock.send(cmd.encode())
    except Exception as e:
        print("[AUTOPILOT] send_cmd error:", e)


def run_autopilot(sock,
                  get_distance: Callable[[], Optional[int]],
                  get_skew: Callable[[], Optional[float]],
                  stop_event: Optional[threading.Event] = None,
                  keep_pushing_after_dock: bool = False,
                  push_power: int = APPROACH_SLOW_POWER,
                  push_duration: float = 1.0) -> bool:
    """
    Blocking autopilot main loop.
    - sock: connected socket to Arduino.
    - get_distance: callable -> int (cm) or None
    - get_skew: callable -> float (deg) or None
    - stop_event: threading.Event() that stops autopilot if set

    Returns True if docking completed, False if aborted/stopped.
    """
    if stop_event is None:
        stop_event = threading.Event()

    state = "initial_advance"
    sweep_step = 0

    print("[AUTOPILOT] Starting autopilot...")

    def should_stop():
        return stop_event.is_set()

    while not should_stop():
        try:
            dist = get_distance()    # cm or None
            skew = get_skew()        # deg or None
        except Exception as e:
            print("[AUTOPILOT] Sensor read error:", e)
            dist = None
            skew = None

        # ---------- State machine ----------
        if state == "initial_advance":
            # Move forward in small pulses until the ultrasonic sees something < ULTRASONIC_RANGE_CM
            print("[AUTOPILOT] State: initial_advance - moving forward to enter sensor range")
            if dist is not None and dist <= ULTRASONIC_RANGE_CM:
                print("[AUTOPILOT] Sensor in range:", dist, "cm -> switch to sweep/search")
                state = "sweep_search"
                sweep_step = 0
                # stop briefly
                send_cmd(sock, 's')
                time.sleep(0.2)
                continue

            # If the sensor is N/A (out of range), advance forward a bit
            send_cmd(sock, f"f 5 {int(INITIAL_ADVANCE_STEP_SEC)}")
            time.sleep(INITIAL_ADVANCE_STEP_SEC + 0.05)
            send_cmd(sock, 's')
            time.sleep(0.1)

        elif state == "sweep_search":
            print("[AUTOPILOT] State: sweep_search, step", sweep_step)
            if skew is not None:
                print("[AUTOPILOT] QR detected (skew =", skew, "deg) -> alignment")
                state = "align"
                send_cmd(sock, 's')
                time.sleep(0.1)
                continue

            if sweep_step >= SWEEP_MAX_STEPS:
                print("[AUTOPILOT] Sweep exhausted, moving forward slightly and retrying sweep")
                send_cmd(sock, f"f 4 1")
                time.sleep(1.1)
                send_cmd(sock, 's')
                time.sleep(0.2)
                sweep_step = 0
                continue

            send_cmd(sock, f"l {SWEEP_TURN_POWER} {SWEEP_TURN_TIME}")
            time.sleep(SWEEP_TURN_TIME + 0.15)
            send_cmd(sock, 's')
            time.sleep(0.15)

            sweep_step += 1
            continue

        elif state == "align":
            if skew is None:
                print("[AUTOPILOT] Lost QR during align -> back to sweep")
                state = "sweep_search"
                sweep_step = 0
                continue

            if abs(skew) <= ALIGN_TOLERANCE_DEG:
                print("[AUTOPILOT] Aligned within ±", ALIGN_TOLERANCE_DEG, "deg. Switching to approach")
                send_cmd(sock, 's')
                time.sleep(0.1)
                state = "approach"
                continue

            if skew > 0:
                print("[AUTOPILOT] Align: skew", skew, "deg -> rotate right small")
                send_cmd(sock, f"r 3 1")
            else:
                print("[AUTOPILOT] Align: skew", skew, "deg -> rotate left small")
                send_cmd(sock, f"l 3 1")
            time.sleep(1.05)
            send_cmd(sock, 's')
            time.sleep(0.1)
            continue

        elif state == "approach":
            if dist is None:
                print("[AUTOPILOT] Approach: no ultrasonic reading, moving small step forward")
                send_cmd(sock, f"f {APPROACH_FORWARD_POWER} {APPROACH_STEP_TIME}")
                time.sleep(APPROACH_STEP_TIME + 0.05)
                send_cmd(sock, 's')
                time.sleep(0.1)
                continue

            print("[AUTOPILOT] Approach: distance", dist, "cm, skew", skew)

            if dist <= DOCKING_DISTANCE_CM:
                print("[AUTOPILOT] Within docking distance (", dist, "cm). STOP & DOCK")
                send_cmd(sock, 's')
                time.sleep(0.1)
                state = "dock"
                continue

            if skew is not None and abs(skew) > ALIGN_TOLERANCE_DEG:
                if skew > 0:
                    send_cmd(sock, f"r 3 1")
                else:
                    send_cmd(sock, f"l 3 1")
                time.sleep(1.05)
                send_cmd(sock, 's')
                time.sleep(0.05)

            if dist <= APPROACH_SLOW_THRESHOLD:
                send_cmd(sock, f"f {APPROACH_SLOW_POWER} {APPROACH_STEP_TIME}")
            else:
                send_cmd(sock, f"f {APPROACH_FORWARD_POWER} {APPROACH_STEP_TIME}")
            time.sleep(APPROACH_STEP_TIME + 0.05)
            send_cmd(sock, 's')
            time.sleep(0.05)
            continue

        elif state == "dock":
            print("[AUTOPILOT] Executing docking servo")
            send_cmd(sock, 'o')   # open/trigger latch; change to 'p' if needed
            time.sleep(1.0)
            send_cmd(sock, 's')
            print("[AUTOPILOT] Docking performed.")

            if keep_pushing_after_dock:
                # Keep issuing forward pulses to hold pressure against the dock until stopped.
                print("[AUTOPILOT] keep_pushing_after_dock enabled — holding forward command until stopped")
                try:
                    while not should_stop():
                        send_cmd(sock, f"f {push_power} {int(push_duration)}")
                        # wait duration plus small settle
                        time.sleep(push_duration + 0.05)
                        # ensure stop between pulses so manual override can interject
                        send_cmd(sock, 's')
                        time.sleep(0.05)
                except Exception as e:
                    print("[AUTOPILOT] Error during pushing-after-dock:", e)
                # final stop
                send_cmd(sock, 's')
                print("[AUTOPILOT] Pushing stopped — autopilot exiting")
                return True

            print("[AUTOPILOT] Autopilot done.")
            return True

        else:
            print("[AUTOPILOT] Unknown state:", state)
            return False

    print("[AUTOPILOT] Stopped by stop_event")
    send_cmd(sock, 's')
    return False


# ---------------- Example integration ----------------
if __name__ == "__main__":
    # Example runs a simulated autopilot loop.
    # Replace with your socket and callable providers that return live sensor values.
    latest_dist = None
    latest_skew = None

    def get_distance():
        return latest_dist

    def get_skew():
        return latest_skew

    print("Module loaded. Integrate get_distance/get_skew and call run_autopilot() from your main program.")
