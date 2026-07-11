"""
===================================================
  eLSI Sprint 1 - Task 2A : PID Line Following + Pick & Place
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 2A scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_v1_2a.py --eval
  3. Run this file:      python3 task2a.py

WHAT YOU IMPLEMENT
  control_loop()  — PID controller that returns (left_speed, right_speed).
  detect_color()  — identify the box color from RGB sensor values.
  should_pick()   — decide when to stop and pick the box.
  should_drop()   — decide when to drop the carried box.

Everything else (connecting, receiving sensors, sending motor/pick/drop
commands) is handled by CoppeliaClient.
Don't edit this file except the marked TODO sections.
You may add helper functions.

SENSOR PROTOCOL  (from 2a_wrapper.py):
  Line sensors:  'left_corner', 'left', 'middle', 'right', 'right_corner'
                  — float [0.0, 1.0];  higher = line detected.
  Proximity:     'proximity'  — metres to nearest object; 1.0 = nothing in range.
  Color sensor:  'color_r', 'color_g', 'color_b'  — float [0.0, 1.0].

TASK FLOW
  1. Robot drives the line following the PID controller.
  2. When the robot is close to the box (proximity low), read the color,
     stop, and send a PICK command.
  3. Robot carries the box and continues following the line.
  4. At the correct drop zone, send a DROP command.A

Team ID: [ 403 ]
"""

import time

from connector import CoppeliaClient

# The five line sensors, ordered left → right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']


# =============================================================================
#  TODO (participants): implement the four functions below.
#  You may add helper functions anywhere in this section.
# =============================================================================

# ---- Tunable constants -----------------------------------------------------
BASE_SPEED = 3.0            # rad/s, forward speed on straight line
KP = 6.0
KI = 0.02
KD = 1.8
MAX_CORRECTION = 3.5        # clamp so wheel speeds don't go haywire

PICK_PROXIMITY_THRESHOLD = 0.14   # metres — close enough to pick
DROP_PROXIMITY_THRESHOLD = 0.14   # metres — close enough to drop
COLOR_CONFIDENCE_THRESHOLD = 0.25

# Sensor position weights, left → right (symmetric around 0).
SENSOR_WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]

# ---- Internal PID / navigation state ---------------------------------------
# control_loop() is called once per cycle with only `sensors`, so PID history
# (integral/derivative terms) and the color/junction bias have to live at
# module scope rather than being passed in as arguments.
_integral_error = 0.0
_prev_error = 0.0
_last_line_seen_sign = 1.0   # remembers which way to search if the line is lost

# Set by should_pick()/should_drop() each cycle so control_loop() can bias
# steering at forks toward the correct drop zone for the carried box's color.
_carrying_box_state = False
_detected_color_state = None

DT = 0.05  # matches the ~20 Hz loop in main()


def _line_error(sensors):
    """Weighted-average line position error from the 5 line sensors.

    Returns a float roughly in [-2, 2]; negative = line is to the left,
    positive = line is to the right, 0 = centered. Returns None if no
    sensor currently sees the line (line lost).
    """
    total_weight = 0.0
    total_signal = 0.0
    for w, key in zip(SENSOR_WEIGHTS, SENSOR_ORDER):
        v = sensors.get(key, 0.0)
        total_weight += w * v
        total_signal += v

    if total_signal < 1e-3:
        return None

    return total_weight / total_signal


def control_loop(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

    Standard PID line follower:
      1. Compute weighted line-position error from the 5 sensors.
      2. Run PID on that error to get a steering correction.
      3. Apply the correction differentially: turn toward the line.
      4. At a junction (both corner sensors triggered) with a box on
         board, nudge the error toward the drop zone matching the
         carried box's color (red -> left, blue -> straight, green -> right).
    """
    global _integral_error, _prev_error, _last_line_seen_sign

    error = _line_error(sensors)

    if error is None:
        # Line lost: spin gently toward the side we last saw it on to
        # reacquire, rather than driving blind.
        search_speed = 1.5
        left = -search_speed * _last_line_seen_sign
        right = search_speed * _last_line_seen_sign
        return left, right

    if error != 0.0:
        _last_line_seen_sign = 1.0 if error > 0 else -1.0

    # Junction bias: only kicks in when both outer corner sensors see line
    # at once (a fork) and we're carrying a box we need to route.
    at_junction = sensors.get('left_corner', 0.0) > 0.5 and sensors.get('right_corner', 0.0) > 0.5
    if at_junction and _carrying_box_state:
        if _detected_color_state == "red":
            error -= 1.5   # steer left
        elif _detected_color_state == "green":
            error += 1.5   # steer right
        # "blue" (or unknown) -> no bias, go straight

    # --- PID ---
    _integral_error += error * DT
    derivative = (error - _prev_error) / DT
    _prev_error = error

    correction = KP * error + KI * _integral_error + KD * derivative
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    # Positive error (line to the right) -> speed up left wheel, slow right.
    left = BASE_SPEED + correction
    right = BASE_SPEED - correction

    return left, right


def detect_color(sensors):
    """Identify the box color from the color sensor RGB values.

    Picks the dominant channel, but only reports a detection once that
    channel clearly leads the other two and exceeds a confidence floor.
    """
    r = sensors.get('color_r', 0.0)
    g = sensors.get('color_g', 0.0)
    b = sensors.get('color_b', 0.0)

    channels = {"red": r, "green": g, "blue": b}
    best_name = max(channels, key=channels.get)
    best_val = channels[best_name]

    if best_val < COLOR_CONFIDENCE_THRESHOLD:
        return None

    others = [v for name, v in channels.items() if name != best_name]
    if best_val - max(others) < 0.08:   # not clearly dominant yet
        return None

    return best_name


def should_pick(sensors, carrying_box):
    """Pick the box once it's close and we're not already holding one."""
    global _carrying_box_state
    _carrying_box_state = carrying_box

    if carrying_box:
        return False

    return sensors.get('proximity', 1.0) < PICK_PROXIMITY_THRESHOLD


def should_drop(sensors, carrying_box, detected_color):
    """Drop the box once we're carrying it and have arrived at its zone.

    Zone routing itself happens in control_loop() (junction steering based
    on detected_color); this function just confirms arrival via proximity,
    the same way should_pick() confirms proximity to the box.
    """
    global _carrying_box_state, _detected_color_state
    _carrying_box_state = carrying_box
    _detected_color_state = detected_color

    if not carrying_box:
        return False

    return sensors.get('proximity', 1.0) < DROP_PROXIMITY_THRESHOLD


# =============================================================================
#  Main loop (Don't Edit this)
# =============================================================================
def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to 2a_wrapper. Running... (Ctrl+C to stop)")

    last_sensors   = None
    carrying_box   = False
    detected_color = None

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            # --- Color detection (once, before picking) ---
            if detected_color is None and not carrying_box:
                color = detect_color(last_sensors)
                if color is not None:
                    detected_color = color
                    print(f"Color detected: {color!r}")

            # --- Pick ---
            if not carrying_box and should_pick(last_sensors, carrying_box):
                success = client.send_pick()
                print(f"PICK attempted  — success={success}")
                if success:
                    carrying_box = True

            # --- Drop ---
            if carrying_box and should_drop(last_sensors, carrying_box, detected_color):
                success = client.send_drop()
                print(f"DROP attempted  — success={success}")
                if success:
                    carrying_box = False

            # --- Motor command ---
            left, right = control_loop(last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()