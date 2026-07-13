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

PICK_PROXIMITY_THRESHOLD = 0.16   # metres — close enough to pick
DROP_PROXIMITY_THRESHOLD = 0.16   # metres — close enough to drop
COLOR_CONFIDENCE_THRESHOLD = 0.12

# Sensor position weights, left → right (symmetric around 0).
SENSOR_WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]

# ---- Internal PID / navigation state ---------------------------------------
_integral_error = 0.0
_prev_error = 0.0
_last_line_seen_sign = 1.0

_carrying_box_state = False
_detected_color_state = None
_color_hist = []   # rolling RGB smoothing history

DT = 0.05  # matches the ~20 Hz loop in main()


def _line_error(sensors):
    """Weighted-average line position error from the 5 line sensors."""
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
    """Return (left_speed, right_speed) for the current sensor reading."""
    global _integral_error, _prev_error, _last_line_seen_sign

    error = _line_error(sensors)

    if error is None:
        # Line lost: rotate toward last seen side.
        search_speed = 1.2
        left = -search_speed * _last_line_seen_sign
        right =  search_speed * _last_line_seen_sign
        return left, right

    if abs(error) > 1e-6:
        _last_line_seen_sign = 1.0 if error > 0 else -1.0

    # Junction detection (slightly permissive to avoid missing fork)
    at_junction = (
        sensors.get('left_corner', 0.0) > 0.35 and
        sensors.get('right_corner', 0.0) > 0.35
    )

    # Color-based routing while carrying
    if at_junction and _carrying_box_state:
        if _detected_color_state == "red":
            error -= 1.6
        elif _detected_color_state == "green":
            error += 1.8
        # blue/unknown => straight

    # PID
    _integral_error += error * DT
    derivative = (error - _prev_error) / DT
    _prev_error = error

    correction = KP * error + KI * _integral_error + KD * derivative
    correction = max(-MAX_CORRECTION, min(MAX_CORRECTION, correction))

    # Positive error => line is right => turn right (left faster, right slower)
    left = BASE_SPEED + correction
    right = BASE_SPEED - correction

    return left, right


def detect_color(sensors):
    """Identify the box color from RGB sensor values with temporal smoothing."""
    global _color_hist

    r = sensors.get('color_r', 0.0)
    g = sensors.get('color_g', 0.0)
    b = sensors.get('color_b', 0.0)

    _color_hist.append((r, g, b))
    if len(_color_hist) > 5:
        _color_hist.pop(0)

    ar = sum(x[0] for x in _color_hist) / len(_color_hist)
    ag = sum(x[1] for x in _color_hist) / len(_color_hist)
    ab = sum(x[2] for x in _color_hist) / len(_color_hist)

    vals = {"red": ar, "green": ag, "blue": ab}
    best = max(vals, key=vals.get)
    m = vals[best]
    second = sorted(vals.values(), reverse=True)[1]

    if m < COLOR_CONFIDENCE_THRESHOLD:
        return None
    if (m - second) < 0.04:
        return None
    return best


def _is_object_close(sensors, threshold):
    """Per wrapper protocol: proximity is distance in metres; 1.0 means no object."""
    p = sensors.get('proximity', 1.0)
    return 0.0 < p < threshold


def should_pick(sensors, carrying_box):
    """Pick once close to box and not already carrying."""
    global _carrying_box_state
    _carrying_box_state = carrying_box

    if carrying_box:
        return False
    return _is_object_close(sensors, PICK_PROXIMITY_THRESHOLD)


def should_drop(sensors, carrying_box, detected_color):
    """Drop once close to drop zone and currently carrying."""
    global _carrying_box_state, _detected_color_state
    _carrying_box_state = carrying_box
    _detected_color_state = detected_color

    if not carrying_box:
        return False
    return _is_object_close(sensors, DROP_PROXIMITY_THRESHOLD)
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