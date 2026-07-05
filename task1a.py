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
Kp = 1.2
Ki = 0.02
Kd = 0.18

BASE_SPEED = 3.6
MAX_SPEED = 5.0
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
prev_time = None

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
    """PID line follower with adaptive speed; edits only inside this function."""
    # persistent state inside function (so no global edits needed)
    if not hasattr(control_loop, "prev_error"):
        control_loop.prev_error = 0.0
        control_loop.integral = 0.0

    # ---- Tunables (kept local to satisfy 'only control_loop' rule) ----
    Kp = 1.35
    Ki = 0.01
    Kd = 0.40

    BASE_SPEED = 2.8     # higher cruise speed
    MAX_SPEED = 3.0      # keep within task limits
    INTEGRAL_LIMIT = 2.0
    LINE_PRESENT_THRESHOLD = 0.05
    CONTRAST_THRESHOLD = 0.08

    SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']
    WEIGHTS = {
        'left_corner': -2.0,
        'left': -1.0,
        'middle': 0.0,
        'right': 1.0,
        'right_corner': 2.0,
    }

    # ---- Contrast-normalized line extraction (works for dark/bright line) ----
    vals = [sensors[n] for n in SENSOR_ORDER]
    low, high = min(vals), max(vals)
    contrast = high - low

    if contrast < CONTRAST_THRESHOLD:
        strengths = [0.0] * 5
    else:
        bright = [(v - low) / contrast for v in vals]
        dark = [(high - v) / contrast for v in vals]
        strengths = bright if sum(bright) <= sum(dark) else dark

    total = sum(strengths)
    if total > LINE_PRESENT_THRESHOLD:
        error = sum(s * WEIGHTS[n] for s, n in zip(strengths, SENSOR_ORDER)) / total
    else:
        # keep steering in last known direction when line is weak/lost
        error = control_loop.prev_error * 0.85
        control_loop.integral = 0.0

    # ---- PID ----
    control_loop.integral += error
    control_loop.integral = max(-INTEGRAL_LIMIT, min(INTEGRAL_LIMIT, control_loop.integral))
    derivative = error - control_loop.prev_error

    correction = Kp * error + Ki * control_loop.integral + Kd * derivative

    # adaptive speed: fast on straights, slower in turns
    turn = min(abs(error), 1.5) / 1.5
    dynamic_base = BASE_SPEED * (1.0 - 0.35 * turn)

    # keep correction bounded so one wheel doesn't saturate too early
    correction = max(-dynamic_base, min(dynamic_base, correction))

    left = dynamic_base + correction
    right = dynamic_base - correction

    # clamp output
    left = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))

    control_loop.prev_error = error
    return left, right


def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_task1a. Running... (Ctrl+C to stop)")

    last_t = time.time()          # NEW
    dt_samples = []                # NEW

    try:
        while True:
            # Send one command for each fresh sensor packet.
            sensors = client.receive_sensor_data()
            if sensors is None:
                time.sleep(0.02)
                continue

            now = time.time()                      # NEW
            dt = now - last_t                      # NEW
            last_t = now                            # NEW
            dt_samples.append(dt)                   # NEW
            if len(dt_samples) % 1 == 0:           # NEW — print every 25 samples, not every frame
             avg = sum(dt_samples[-25:]) / 25    # NEW
             print(f"avg dt over last 25 samples: {avg:.4f}s  (~{1/avg:.1f} Hz)" , flush=True)  # NEW

            left, right = control_loop(sensors)
            client.send_motor_command(left, right)

            time.sleep(0.005)   # keep commands slightly below the bridge read rate
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
