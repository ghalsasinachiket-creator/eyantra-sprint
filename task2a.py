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
  4. At the correct drop zone, send a DROP command.

Team ID: [ XXX ]
"""

import time

from task2_.task2a.connector import CoppeliaClient

# The five line sensors, ordered left → right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']


# =============================================================================
#  TODO (participants): implement the four functions below.
#  You may add helper functions anywhere in this section.
# =============================================================================

def control_loop(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

    Args:
        sensors (dict): latest sensor values with keys:
            'left_corner', 'left', 'middle', 'right', 'right_corner'  — [0, 1]
            'proximity'     — distance in metres (1.0 = nothing nearby)
            'color_r', 'color_g', 'color_b'                          — [0, 1]

    Returns:
        tuple[float, float]: (left_speed, right_speed) in rad/s.

    TODO (participants): replace the placeholder with your PID controller.
    Typical approach:
      1. Compute a weighted line-position error from the 5 sensors.
      2. Run a PID on that error.
      3. Apply the correction differentially to left / right wheel speeds.
    """
    # ----- placeholder: drive straight slowly. REPLACE THIS. -----
    base_speed = 2.0
    left  = base_speed
    right = base_speed
    return left, right


def detect_color(sensors):
    """Identify the box color from the color sensor RGB values.

    Args:
        sensors (dict): same dict as control_loop(); use
            sensors['color_r'], sensors['color_g'], sensors['color_b'].

    Returns:
        str | None: one of ``"red"``, ``"green"``, ``"blue"``,
                    or ``None`` if no confident detection yet.

    TODO (participants): implement color detection.
    Hint: compare which channel has the highest value, but only trigger
    when the maximum is above a confidence threshold (e.g. > 0.25).
    """
    # ----- placeholder: never detects. REPLACE THIS. -----
    return None


def should_pick(sensors, carrying_box):
    """Decide whether to attempt picking the box right now.

    Args:
        sensors      (dict): latest sensor values.
        carrying_box (bool): True if the robot is already holding a box.

    Returns:
        bool: True to send a PICK command this cycle.

    TODO (participants): use sensors['proximity'] to detect closeness.
    Only pick when NOT already carrying a box.
    Example threshold: proximity < 0.10 (10 cm).
    """
    # ----- placeholder: never picks. REPLACE THIS. -----
    return False


def should_drop(sensors, carrying_box, detected_color):
    """Decide whether to drop the box right now.

    Args:
        sensors        (dict):       latest sensor values.
        carrying_box   (bool):       True if the robot is holding a box.
        detected_color (str | None): color string from detect_color(), or None.

    Returns:
        bool: True to send a DROP command this cycle.

    TODO (participants): only drop when carrying_box is True and you are
    at the correct drop zone for detected_color.
    Color → zone:
        "red"   → left zone
        "blue"  → straight / center zone
        "green" → right zone
    """
    # ----- placeholder: never drops. REPLACE THIS. -----
    return False


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
