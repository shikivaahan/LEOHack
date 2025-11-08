# Introduction

This repository holds the basic code for interfacing with the Active Satellite Robot (ASR) from your laptop. It is there to serve a baseline for you to connect to the Arduino and control the motors of the robot.

## System overview

The robot's onboard computer (the Arduino Nano IOT) is a microcontroller running on C++ compiled code. It has a WiFi/Bluetooth module on it and that's what is used to connect to it. The Nano creates its own network to which the ground station (laptop running python) is connected. The python script on the laptop then takes button presses from the user and transmits them as single characters which the Arduino then receives and triggers the corresponding functions.

# Installation Instructions

## Software Download

To edit, compile and run the code you will need two IDE environments. (In theory we could do only one, but it'll be easier if we use two separate ones).
- Arduino IDE
- VSCode (or any other python IDE like Spyder)

### Arduino IDE

To download Arduino IDE, use this link: https://www.arduino.cc/en/software/
If you're new to Arduino (then you're in for a treat!), follow this link for a basic getting-started guide: https://docs.arduino.cc/software/ide-v2/tutorials/getting-started-ide-v2/

Go to Tools/Manage Libraries and download *WiFiNINA* library.

### VS Code

VS Code is a very popular developer software for all types of languages. If you already have an IDE ready, feel free to use it. If not, download VS Code from here: https://code.visualstudio.com/download.

Next, you will need to install python and its dependencies (python add-ons). The exact steps will differ based on your operating system so please follow the steps outlined on this page to install python, and a python VS Code extension: https://code.visualstudio.com/docs/python/python-tutorial#_prerequisites

## Downloading the code

For quick-start, just download the repo from this webpage (top right button). Save it and start working in it. However it is recommended you fork this repository into your private GitHub (or equivalent) repository and work there. It is also totally fine if you don't want to use this code at all and start something from scratch.


# Running the code

## Arduino IDE

The robot's onboard computer runs on 'onboard_receiver.ino' code which uses the surrounding files in the 'onboard_receiver' folder. Open it with Arduino IDE. Towards the top, you will see:
```
char ssid[] = "Nano_[YOUR_NAME]_AP";
char pass[] = "nano1pwd";
```
This will be the name of your Nano's WiFi. Change the ssid so it includes your name in it, also change the password to something you prefer (8 characters minimum). Plug in the board and hit the 'upload' button to upload the code. It should start running straight away!

## Python Script

First, connect your laptop to the WiFi network created by your nano. Then, run the 'remote_control.py' file to start up the controller. If everything works, it should connect to your Arduino in a few seconds. Then you can use the following buttons to control the robot:
w -> go foward
s -> go backwards
q -> rotate left
e -> rotate right

And voilla! You are controlling the robot. Now you can make your own modifications to it :) 

# Breakout Board Pinout

- Black -> GND (3A max)
- Red -> +5V (3A max)
- Yellow -> D9 (PWM/White-Blue Cable)
- Orange -> D11 (PWM/White-Orange Cable)
- Brown -> D12 (PWM/White-Brown Cable)
- White -> A0 (Anaolgue/White-Green Cable)
- Grey -> +3.3V (150 mA max/Blue Cable)
- Purple -> A2 (Analogue/Green Cable)


## Data streaming protocol & control-law (design guide)

This section explains how the Nano/Arduino and the laptop exchange data and commands, the expected message formats, suggested parsing rules, and how to structure a control law on the laptop side (conceptual — no code here).

### Overview

- The system uses a simple ASCII/text protocol over a TCP socket between the laptop and the Nano. The laptop opens a connection to the Nano's IP (default in this project: `192.168.4.1:8080`).
- Communication is bidirectional:
  - Host -> Nano: control commands (single-line ASCII tokens)
  - Nano -> Host: telemetry or status messages (single-line ASCII tokens)
- Messages are newline-delimited (each message ends with a '\n'). Each message is treated as an atomic packet for parsing.

Assumptions
- The system uses ASCII tokens separated by spaces for human-readability.
- The Nano behaves as a simple TCP server; the host connects as a client. If your setup differs (serial, UDP, or different port), adapt accordingly.

### Control command format (Host -> Nano)

The existing `remote_control.py` shows the command mapping used for manual control. These are plain ASCII tokens with optional numeric parameters separated by spaces. Examples included in the project:

- Movement commands (format: <cmd> <power> <duration>):
  - `f <power> <duration>`  — forward
  - `b <power> <duration>`  — backward
  - `a <power> <duration>`  — left
  - `d <power> <duration>`  — right
  - `l <power> <duration>`  — rotate left
  - `r <power> <duration>`  — rotate right
- Servo commands (no args):
  - `o` — servo open
  - `p` — servo close
- Stop / safety:
  - `s` — stop (immediate stop / neutral)

Notes:
- `power` and `duration` are decimal integers (the code currently uses small integers for power and seconds for duration). The Nano should parse tokens by splitting on whitespace.
- Command messages should be terminated with a newline so the Nano can treat each line as a message.

### Recommended telemetry format (Nano -> Host)

The repo doesn't lock a single telemetry format. For interoperability, use newline-delimited ASCII lines with a short message-type token followed by comma- or space-separated fields. Two recommended forms (choose one and keep consistent):

1) Keyed CSV (preferred for human-readability):
   TELEMETRY lines start with `T,` then comma-separated values, e.g.:
   T,<timestamp_ms>,<left_pwm>,<right_pwm>,<left_enc>,<right_enc>,<servo_pos>,<battery_mv>
   Example: `T,1634567890,128,127,2345,2370,90,7400` — timestamp in ms, left PWM, right PWM, left encoder count, right encoder count, servo angle (deg), battery millivolts.

2) Space-separated key token (more flexible):
   `TL <timestamp_ms> L_PWM <val> R_PWM <val> L_ENC <val> R_ENC <val> BATT <mv>`
   Example: `TL 1634567890 L_PWM 128 R_PWM 127 L_ENC 2345 R_ENC 2370 BATT 7400`

Also include lightweight status/heartbeat lines:
- `HB <timestamp_ms>`  — heartbeat (indicates the Nano is alive)
- `ERR <code> <msg>`  — error or diagnostic

Parsing rules (host side):
- Read incoming bytes, split on newline (`\n`) to get messages.
- Trim whitespace and ignore empty lines.
- Inspect the first token to determine message type (`T`, `TL`, `HB`, `ERR`, etc.) and parse the rest according to that type.
- Treat unknown/invalid lines as diagnostics (log them) but do not crash your control loop.

### Contract for telemetry & commands (inputs / outputs)

- Inputs (to host control law): the most recent telemetry message containing state estimates (encoder counts, motor PWMs, battery level, timestamps).
- Outputs (from host): commands sent to the Nano using the control command format above (e.g., `f <power> <duration>` or a more explicit motion command if you extend the protocol).
- Error modes: loss of connection, malformed telemetry, stale data.
- Success criteria: host receives telemetry at an expected rate (e.g., 10–50 Hz), control commands result in expected actuation and telemetry reflecting the state.

### How to design the control loop (conceptual)

1. Sampling & Timing
  - Decide a control rate (e.g., 10–50 Hz). The loop must read the latest telemetry message, compute control output, and send commands.
  - Use timestamps sent by the Nano (or host-side reception time) to estimate latency and drop stale messages.

2. State estimation
  - From telemetry (encoder counts, PWMs), compute relevant states: wheel speeds (deltas over dt), heading, battery voltage, etc.
  - If encoder counts are cumulative, compute speed by differencing counts and dividing by dt.

3. Control law structure (high-level)
  - Error = desired_state - measured_state (for example, desired_speed - measured_speed)
  - Controller: implement a PI/PID or state-feedback controller conceptually. Keep integral anti-windup and output saturation in mind.
  - Output mapping: controller output must be translated to the command format the Nano accepts. If commands are coarse (power + duration), send frequent short-duration commands (e.g., 0.05–0.2 s) to approximate continuous control.

4. Command scheduling & acknowledgement
  - Prefer idempotent or short commands; e.g., rather than long-duration open-loop commands, send frequent small updates so changes can be reacted to.
  - If you add an acknowledgement (ACK) message to the protocol, the host can confirm the Nano received critical changes; otherwise assume best-effort and include a watchdog.

5. Safety & watchdog
  - If telemetry hasn't been received for N consecutive control cycles (N configurable, e.g., 3), send `s` (stop) and raise an alert.
  - Always include a local emergency stop command (`s`) for immediate halting.
  - Enforce saturation limits on control outputs and ensure PWM/power values are clamped to safe ranges.

### Edge cases & robustness

- Packet loss: TCP reduces this risk, but handle broken connections and reconnection logic.
- Latency and jitter: measure round-trip times and design controller gains conservatively to tolerate latency. If latency is large (>100 ms), prefer slower integrators and predictive elements.
- Out-of-order or duplicate data: if using timestamps, reject telemetry older than the latest processed.
- Battery sag: include battery voltage in telemetry and scale control output if voltage is low to avoid overload.

### Testing and debugging (no code examples)

- Logging: keep a timestamped log of raw telemetry messages and sent commands for offline analysis.
- Step tests: send known small commands and observe telemetry response (e.g., send small positive power for a short duration and confirm encoder delta).
- Monitor health: add `HB` heartbeat messages from the Nano. If missing, trigger a stop.

### Protocol extension suggestions (if you later update code)

- Add an explicit motor command that sets left & right PWM directly (e.g., `M <left_pwm> <right_pwm>`), which is better for closed-loop control than the current high-level `f/b/...` commands.
- Add `ACK` messages or sequence numbers so the host can match commands to observed behavior.
- Add a JSON-compact telemetry mode for more fields (binary is also possible for higher throughput, but requires a strict parser).

### Summary / Quick checklist

- Use newline-terminated ASCII messages.
- Host parses the first token to detect message type.
- Telemetry should include timestamp + encoder counts + actuator outputs + battery.
- The host control law computes an error, runs a controller (PI/PID), maps outputs to the Nano command format, and sends commands frequently in short durations.
- Implement watchdog safety: stop on stale telemetry or communication loss.

If you want, I can next (a) produce a short one-page spec file (protocol.md) extracted from this content, or (b) create a small example telemetry format and a table mapping fields to units — tell me which you'd prefer.
