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

Team ID: [ XXX ]
"""

import time

from connector_task1a import CoppeliaClient

# The five line sensors, ordered left -> right across the robot.
# Each value is in [0.0, 1.0]; a higher value means the line is detected.
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']


def control_loop(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

    `sensors` is a dict, e.g.:
        {'left_corner': 0.02, 'left': 0.41, 'middle': 0.95,
         'right': 0.05, 'right_corner': 0.01}

    ------------------------------------------------------------------
    TODO (participants): implement your PID line-following controller.
    ------------------------------------------------------------------
    A typical approach:
      1. Turn the 5 readings into ONE line-position error
      2. Feed that error through a PID controller:
      3. Drive the wheels differentially:
    """
    # ----- placeholder: drive straight slowly. REPLACE THIS. -----
    base_speed = 2.0
    left = base_speed
    right = base_speed
    return left, right


def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_task1a. Running... (Ctrl+C to stop)")

    last_sensors = None
    try:
        while True:
            # Pull the freshest sensor packet; reuse the last one between packets.
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            left, right = control_loop (last_sensors)
            client.send_motor_command(left, right)

            time.sleep(0.05)   # ~20 Hz control loop
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
