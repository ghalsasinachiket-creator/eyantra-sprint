"""
===================================================
    eLSI Sprint 1 - Task 1A : PID Line Following
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 1A scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_task1a.py --eval
  3. Run this file:      python3 task1a_template.py

WHAT YOU IMPLEMENT
  Only control_loop(). Everything else (connecting, receiving sensors,
  sending motor commands) is handled for you by CoppeliaClient.
  Don't Edit this file except control_loop().
  You can add helper functions if you like.

Team ID: [ 403]
"""

import time

from connector_task1a import CoppeliaClient

# The five line sensors, ordered left -> right across the robot.
# Each value is in [0.0, 1.0]; the line may be brighter or darker than the floor.
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

# PID tuning parameters. Start here, then tune in CoppeliaSim if needed.
Kp = 1.5
Ki = 0.0
Kd = 0.6

BASE_SPEED = 1.8
MAX_SPEED = 3.0
INTEGRAL_LIMIT = 10.0
LINE_PRESENT_THRESHOLD = 0.05
CONTRAST_THRESHOLD = 0.08

WEIGHTS = {
    'left_corner': -2.0,
    'left': -1.0,
    'middle': 0.0,
    'right': 1.0,
    'right_corner': 2.0,
}

prev_error = 0.0
integral = 0.0


def get_line_strengths(sensors):
    """Return contrast-based line strengths, independent of line color."""
    values = [sensors[name] for name in SENSOR_ORDER]
    low = min(values)
    high = max(values)
    contrast = high - low

    if contrast < CONTRAST_THRESHOLD:
        return [0.0] * len(SENSOR_ORDER)

    bright_line = [(value - low) / contrast for value in values]
    dark_line = [(high - value) / contrast for value in values]

    # The actual line is narrow, so it activates fewer sensors than the floor.
    if sum(bright_line) <= sum(dark_line):
        return bright_line
    return dark_line


def control_loop(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

      2. Feed that error through a PID controller:
      3. Drive the wheels differentially:
    """
    # ----- placeholder: drive straight slowly. REPLACE THIS. -----
    global prev_error, integral

    line_strengths = get_line_strengths(sensors)
    weighted_sum = sum(value * WEIGHTS[name] for value, name in zip(line_strengths, SENSOR_ORDER))
    total = sum(line_strengths)

    if total > LINE_PRESENT_THRESHOLD:
        error = weighted_sum/total
    else:
        error = prev_error * 0.9  # decay and keep turning toward the last known line
        integral = 0.0  # reset to avoid windup
    
    integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, integral + error))
    derivative = error - prev_error

    # Slow down proportionally when the error is large, to avoid overshooting.
    speed_scale = 1.0 - 0.4* min(abs(error), 1.0)
    dynamic_base = BASE_SPEED * speed_scale

    correction = Kp*error + Ki*integral + Kd*derivative
    correction = max(-dynamic_base, min(dynamic_base, correction))
    prev_error = error

    left = dynamic_base + correction
    right = dynamic_base - correction
    left  = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    return left, right


def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_task1a. Running... (Ctrl+C to stop)")

    try:
        while True:
            # Send one command for each fresh sensor packet.
            sensors = client.receive_sensor_data()
            if sensors is None:
                time.sleep(0.02)
                continue

            left, right = control_loop(sensors)
            client.send_motor_command(left, right)

            time.sleep(0.08)   # keep commands slightly below the bridge read rate
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0)   # stop the robot
        except Exception:
            pass
        client.close()


if __name__ == "__main__":
    main()
