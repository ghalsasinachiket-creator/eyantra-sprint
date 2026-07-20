"""
===================================================
  eLSI Sprint 1 - Task 2B : PID Line Following + Pick & Place (dual line)
===================================================

Participant template (PID variant).

TASK 2B
  Follow the track (white line on black AND black line on white) through the
  checkpoints, pick the red and blue boxes near the circle, drop each in its
  matching colour drop zone, then finish at the white box.
  Boxes are handled ONE AT A TIME: pick one, deliver it, come back for the other.

HOW TO RUN
  1. Open the Task 2B scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_v1_2b.py --eval
  3. Run this file:      python3 task2b_pid_template.py

WHAT YOU IMPLEMENT
  control_loop()  - PID controller that returns (left_speed, right_speed).
  detect_color()  - identify the box colour from the RGB sensor.
  should_pick()   - decide when to pick a box (only when one is right next to you).
  should_drop()   - decide when to drop the carried box (at its matching zone).

Everything else (connecting, receiving sensors, sending motor/pick/drop
commands) is handled by CoppeliaClient. Don't edit outside the marked TODO
sections. You may add helper functions.

SENSOR PROTOCOL (from bridge_v1_2b.py):
  Line sensors:  'left_corner','left','middle','right','right_corner' — [0,1].
                 NOTE: this track has BOTH white-line-on-black and
                 black-line-on-white sections, so "on the line" is not always
                 "high" — design your error term to handle both.
  Proximity:     'proximity' — metres to nearest object; 1.0 = nothing in range.
  Color sensor:  'color_r','color_g','color_b' — [0,1].

Team ID: [ XXX ]
"""

import time

from connector_2b import CoppeliaClient

# The five line sensors, ordered left -> right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']


# =============================================================================
#  TODO (participants): implement the four functions below.
#  You may add helper functions anywhere in this section.
# =============================================================================

def control_loop(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

    Args:
        sensors (dict): 'left_corner','left','middle','right','right_corner' [0,1],
                        'proximity' (m), 'color_r','color_g','color_b' [0,1].

    Returns:
        tuple[float, float]: (left_speed, right_speed) in rad/s.

    TODO (participants): replace the placeholder with your PID controller.
    Because the track switches between white-on-black and black-on-white, a
    plain "higher = line" error will fail on one half. Consider detecting which
    regime you're in (e.g. from the overall brightness) and flipping the sign,
    or using an error term that works for both.
    """
    # ----- placeholder: drive straight slowly. REPLACE THIS. -----
    base_speed = 2.0
    return base_speed, base_speed


def detect_color(sensors):
    """Identify the colour of the box in front from the RGB sensor.

    Returns:
        str | None: "red", "blue", or None if no confident detection.

    TODO (participants): compare color_r / color_g / color_b and return the
    dominant colour once it is above a confidence threshold.
    """
    # ----- placeholder: never detects. REPLACE THIS. -----
    return None


def should_pick(sensors, carrying_color):
    """Decide whether to send a PICK this cycle.

    Args:
        sensors (dict): latest sensor values.
        carrying_color (str | None): colour currently held, or None if empty-handed.

    Returns:
        bool: True to attempt a PICK now.

    NOTE: the bridge only picks if a box is actually next to the gripper, so a
    PICK when nothing is close simply returns failure. Only pick when NOT
    already carrying a box (carrying_color is None).

    TODO (participants): use sensors['proximity'] to detect that a box is close.
    """
    # ----- placeholder: never picks. REPLACE THIS. -----
    return False


def should_drop(sensors, carrying_color):
    """Decide whether to send a DROP this cycle.

    Args:
        sensors (dict): latest sensor values.
        carrying_color (str | None): colour currently held, or None.

    Returns:
        bool: True to attempt a DROP now.

    TODO (participants): only drop when carrying_color is not None AND you have
    navigated to the drop zone that matches carrying_color
    ("red" -> red zone, "blue" -> blue zone).
    """
    # ----- placeholder: never drops. REPLACE THIS. -----
    return False


# =============================================================================
#  Main loop (Don't Edit this)
# =============================================================================
def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_v1_2b. Running... (Ctrl+C to stop)")

    last_sensors   = None
    carrying_color = None   # colour of the box currently held, or None
    delivered      = 0      # number of boxes released so far

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            # --- Pick (empty-handed only) ---
            if carrying_color is None and should_pick(last_sensors, carrying_color):
                colour_seen = detect_color(last_sensors)     # read BEFORE picking
                success = client.send_pick()
                print(f"PICK attempted (saw {colour_seen!r}) — success={success}")
                if success:
                    carrying_color = colour_seen

            # --- Drop (only while carrying) ---
            if carrying_color is not None and should_drop(last_sensors, carrying_color):
                success = client.send_drop()
                print(f"DROP attempted ({carrying_color!r}) — success={success}")
                if success:
                    delivered += 1
                    carrying_color = None
                    print(f"Delivered {delivered} box(es) so far.")

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
