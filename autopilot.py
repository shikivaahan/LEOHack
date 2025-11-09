"""
Autopilot state machine for docking.

Assumptions:
- get_distance() -> int|None (cm)
- get_skew() -> float|None (deg, + means target to RIGHT)
"""

import time
import threading
from typing import Callable, Optional

# ======= TUNABLE PARAMETERS =======
# Sweep pattern
SWEEP_TURN_POWER       = 3         # motor power used for sweep turns
SWEEP_TURN_TIME        = 0.05         # seconds per sweep step (integer for your Arduino parser)
TURN_DEG_PER_STEP      = 1        # ***approx*** degrees turned by one sweep step at the above power/time
MAX_SWEEP_LEFT_DEG     = 90        # hard limit for left sweep
MAX_SWEEP_RIGHT_DEG    = 80        # hard limit for right sweep
NUDGE_FORWARD_POWER    = 4         # forward power between sweeps
NUDGE_FORWARD_TIME     = 1         # seconds for nudge forward between sweeps

# Align
ALIGN_TOLERANCE_DEG    = 5         # acceptable skew to consider aligned
ALIGN_STEP_POWER       = 3
ALIGN_STEP_TIME        = 1

# Approach
APPROACH_FORWARD_POWER = 4
APPROACH_SLOW_POWER    = 3
APPROACH_STEP_TIME     = 1
APPROACH_SLOW_THRESHOLD= 40        # slow when closer than this (cm)
DOCKING_DISTANCE_CM    = 15        # dock threshold (cm)

# ==================================


def send_cmd(sock, cmd: str):
    """Send a command string to the Arduino. Does NOT add newline."""
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
    """
    if stop_event is None:
        stop_event = threading.Event()

    # Start directly in sweep mode (no ultrasonic gate)
    state = "sweep_search"
    print("[AUTOPILOT] Starting autopilot... (initial = sweep_search)")

    # Sweep bookkeeping
    sweep_dir = "left"  # alternate between 'left' and 'right'
    steps_in_dir = 0

    # Compute step limits from desired degrees and per-step degrees
    max_steps_left  = max(1, int(round(MAX_SWEEP_LEFT_DEG  / float(TURN_DEG_PER_STEP))))
    max_steps_right = max(1, int(round(MAX_SWEEP_RIGHT_DEG / float(TURN_DEG_PER_STEP))))

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

        # ------------- STATE MACHINE -------------
        if state == "sweep_search":
            # If we see the tag at any time, go align
            if skew is not None:
                print(f"[AUTOPILOT] QR detected during sweep (skew = {skew:.1f} deg) -> alignment")
                send_cmd(sock, 's')
                time.sleep(0.1)
                state = "align"
                continue

            # Decide current sweep limit
            current_limit = max_steps_left if sweep_dir == "left" else max_steps_right

            # If exceeded limit, nudge forward once and flip direction
            if steps_in_dir >= current_limit:
                print(f"[AUTOPILOT] Sweep {sweep_dir} limit reached (~{steps_in_dir*TURN_DEG_PER_STEP}°). Nudge forward & swap direction.")
                # Nudge forward
                send_cmd(sock, f"f {NUDGE_FORWARD_POWER} {NUDGE_FORWARD_TIME}")
                time.sleep(NUDGE_FORWARD_TIME + 0.05)
                send_cmd(sock, 's')
                time.sleep(0.15)

                # Flip direction and reset counter
                sweep_dir = "right" if sweep_dir == "left" else "left"
                steps_in_dir = 0
                continue

            # Perform one sweep step in the current direction
            if sweep_dir == "left":
                print(f"[AUTOPILOT] Sweep step {steps_in_dir+1}/{current_limit} LEFT "
                      f"(~{TURN_DEG_PER_STEP}° per step)")
                send_cmd(sock, f"l {SWEEP_TURN_POWER} {SWEEP_TURN_TIME}")
            else:
                print(f"[AUTOPILOT] Sweep step {steps_in_dir+1}/{current_limit} RIGHT "
                      f"(~{TURN_DEG_PER_STEP}° per step)")
                send_cmd(sock, f"r {SWEEP_TURN_POWER} {SWEEP_TURN_TIME}")

            time.sleep(SWEEP_TURN_TIME + 0.5)
            send_cmd(sock, 's')
            time.sleep(0.50)
            steps_in_dir += 1
            continue

        elif state == "align":
            if skew is None:
                print("[AUTOPILOT] Lost QR during align -> back to sweep")
                state = "sweep_search"
                # keep current direction; reset to start small arcs again
                steps_in_dir = 0
                continue

            if abs(skew) <= ALIGN_TOLERANCE_DEG:
                print(f"[AUTOPILOT] Aligned within ±{ALIGN_TOLERANCE_DEG}°. Switching to approach")
                send_cmd(sock, 's')
                time.sleep(0.1)
                state = "approach"
                continue

            if skew > 0:
                print(f"[AUTOPILOT] Align: skew {skew:.1f}° -> rotate RIGHT small")
                send_cmd(sock, f"r {ALIGN_STEP_POWER} {ALIGN_STEP_TIME}")
            else:
                print(f"[AUTOPILOT] Align: skew {skew:.1f}° -> rotate LEFT small")
                send_cmd(sock, f"l {ALIGN_STEP_POWER} {ALIGN_STEP_TIME}")
            time.sleep(ALIGN_STEP_TIME + 0.05)
            send_cmd(sock, 's')
            time.sleep(0.05)
            continue

        elif state == "approach":
            if dist is None:
                print("[AUTOPILOT] Approach: no distance reading, moving small step forward")
                send_cmd(sock, f"f {APPROACH_FORWARD_POWER} {APPROACH_STEP_TIME}")
                time.sleep(APPROACH_STEP_TIME + 0.05)
                send_cmd(sock, 's')
                time.sleep(0.1)
                continue

            print(f"[AUTOPILOT] Approach: distance {dist} cm, skew {skew}")

            if dist <= DOCKING_DISTANCE_CM:
                print(f"[AUTOPILOT] Within docking distance ({dist} cm). STOP & DOCK")
                send_cmd(sock, 's')
                time.sleep(0.1)
                state = "dock"
                continue

            # Keep alignment tight during approach
            if skew is not None and abs(skew) > ALIGN_TOLERANCE_DEG:
                if skew > 0:
                    send_cmd(sock, f"r {ALIGN_STEP_POWER} {ALIGN_STEP_TIME}")
                else:
                    send_cmd(sock, f"l {ALIGN_STEP_POWER} {ALIGN_STEP_TIME}")
                time.sleep(ALIGN_STEP_TIME + 0.05)
                send_cmd(sock, 's')
                time.sleep(0.05)

            # Slow down when close
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
            send_cmd(sock, 'o')   # or 'p' depending on your latch
            time.sleep(1.0)
            send_cmd(sock, 's')
            print("[AUTOPILOT] Docking performed.")

            if keep_pushing_after_dock:
                print("[AUTOPILOT] keep_pushing_after_dock enabled — holding forward command until stopped")
                try:
                    while not should_stop():
                        send_cmd(sock, f"f {push_power} {int(push_duration)}")
                        time.sleep(push_duration + 0.05)
                        send_cmd(sock, 's')
                        time.sleep(0.05)
                except Exception as e:
                    print("[AUTOPILOT] Error during pushing-after-dock:", e)
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
