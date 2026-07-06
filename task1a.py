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
    global prev_error, integral

    if not hasattr(control_loop, "lost_count"):
        control_loop.lost_count = 0
        control_loop.straight_count = 0
        control_loop.f_error = 0.0
        control_loop.last_sign = 1.0

    # --- Tuned for rough/dashed segments + speed ---
    Kp_l, Ki_l, Kd_l = 1.22, 0.0, 0.62
    BASE = 2.45
    BOOST = 2.95
    MAXV = 3.0
    I_LIM = 1.8

    LINE_T = 0.028
    CONTRAST_T = 0.042

    vals = [sensors[n] for n in SENSOR_ORDER]
    lo, hi = min(vals), max(vals)
    contrast = hi - lo

    if contrast < CONTRAST_T:
        strengths = [0.0] * 5
    else:
        bright = [(v - lo) / contrast for v in vals]
        dark = [(hi - v) / contrast for v in vals]
        strengths = bright if sum(bright) <= sum(dark) else dark

    total = sum(strengths)
    line_found = total > LINE_T

    if line_found:
        raw_error = sum(s * WEIGHTS[n] for s, n in zip(strengths, SENSOR_ORDER)) / total

        # confidence from how concentrated the line is on a few sensors
        peak = max(strengths)
        confidence = max(0.0, min(1.0, (peak - 0.35) / 0.65))  # 0..1

        # heavier smoothing in low confidence zones (dashed / disturbed)
        alpha = 0.55 if confidence > 0.6 else 0.35
        control_loop.f_error = (1 - alpha) * control_loop.f_error + alpha * raw_error
        error = control_loop.f_error

        control_loop.lost_count = 0
        control_loop.last_sign = 1.0 if error >= 0 else -1.0
    else:
        control_loop.lost_count += 1
        # bridge short gaps with heading memory, avoid violent search
        if control_loop.lost_count <= 5:
            error = prev_error * 0.96
        elif control_loop.lost_count <= 14:
            error = control_loop.last_sign * 0.85
        else:
            error = control_loop.last_sign * 1.05
        integral = 0.0
        confidence = 0.0

    # PID
    integral += error
    integral = max(-I_LIM, min(I_LIM, integral))
    derivative = error - prev_error
    correction = Kp_l * error + Ki_l * integral + Kd_l * derivative

    # Straight detector for burst speed
    if line_found and abs(error) < 0.09 and abs(derivative) < 0.05 and confidence > 0.75:
        control_loop.straight_count += 1
    else:
        control_loop.straight_count = 0

    in_boost = control_loop.straight_count >= 10

    # Speed policy:
    # - high on stable straights
    # - moderate on normal line
    # - keep momentum on dashed/rough zones (don't crawl)
    if line_found:
        e = min(abs(error), 1.5) / 1.5
        if in_boost:
            dynamic_base = BOOST * (1.0 - 0.18 * e)
        else:
            dynamic_base = BASE * (1.0 - 0.34 * e)
        # if confidence is low, slightly cap speed but don't over-slow
        if confidence < 0.45:
            dynamic_base = min(dynamic_base, 2.10)
    else:
        dynamic_base = 1.95 if control_loop.lost_count <= 10 else 1.70

    correction = max(-dynamic_base, min(dynamic_base, correction))

    left = dynamic_base + correction
    right = dynamic_base - correction

    left = max(-MAXV, min(MAXV, left))
    right = max(-MAXV, min(MAXV, right))

    prev_error = error
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
