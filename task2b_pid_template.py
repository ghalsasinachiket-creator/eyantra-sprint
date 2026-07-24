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

Team ID: [ 403]
"""

import time

from connector_2b import CoppeliaClient

# The five line sensors, ordered left -> right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']


# =============================================================================
#  TODO (participants): implement the four functions below.
#  You may add helper functions anywhere in this section.
# =============================================================================

# ---- tuning constants (start here, adjust after watching it run) ----------
SENSOR_WEIGHTS = {'left_corner': -2.0, 'left': -1.0, 'middle': 0.0,
                   'right': 1.0, 'right_corner': 2.0}

KP, KI, KD = 3.0, 0.0, 1.5     # PID gains
BASE_SPEED = 3.0               # rad/s, nominal forward speed
MAX_SPEED = 5.0                # rad/s, wheel speed clamp
CURVE_SLOWDOWN = 0.6           # fraction of BASE_SPEED shed when error is large

LINE_PRESENT_THRESHOLD = 0.35  # below this total "line-ness" => treat as a gap
PICK_PROXIMITY = 0.08          # metres — TUNE: distance that counts as "box is here"
DROP_PROXIMITY = 0.12          # metres — TUNE: distance that counts as "at the zone"
COLOR_CONFIDENCE = 0.15        # margin the dominant channel must lead by

# PID state carried between control_loop() calls (the function itself is
# only given the latest sensor reading, so the running terms live here).
_pid_state = {'integral': 0.0, 'last_error': 0.0, 'last_time': None}


def _line_signals(sensors):
    """Turn raw reflectance readings into a 'line-ness' value that is always
    HIGH when a sensor sits on the line — whether this stretch is a white
    line on black, or a black line on white.

    We guess the regime from the average of all five readings: if the arena
    is mostly dark (avg < 0.5) the line is the bright bit, so raw value IS
    the signal; if the arena is mostly bright, the line is the dark bit, so
    we invert.
    """
    raw = [sensors[name] for name in SENSOR_ORDER]
    #avg = sum(raw) / len(raw)
    #return raw if avg < 0.5 else [1.0 - v for v in raw]
    med = sorted(raw)[2] #median is 5
    if med < 0.5:
        return raw # background dark -> line is bright, use as-is
    else:
        return [1.0 - v for v in raw] #background bright -> invert




def control_loop(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

    TODO (participants): replace the placeholder with your PID controller.
    """
    signals = _line_signals(sensors)
    total = sum(signals)

    now = time.time()
    dt = (now - _pid_state['last_time']) if _pid_state['last_time'] else 0.05
    dt = max(dt, 1e-3)
    _pid_state['last_time'] = now

    if total > LINE_PRESENT_THRESHOLD:
        # weighted centroid of where the line sits under the sensor bar
        weights = [SENSOR_WEIGHTS[name] for name in SENSOR_ORDER]
        error = sum(w * s for w, s in zip(weights, signals)) / total
    else:
        # dashed-line gap: nothing detected this cycle. Coast on the last
        # known error instead of snapping to 0 (which would drive straight
        # off a curve) or stopping.
        error = _pid_state['last_error']

    _pid_state['integral'] += error * dt
    derivative = (error - _pid_state['last_error']) / dt
    _pid_state['last_error'] = error

    correction = KP * error + KI * _pid_state['integral'] + KD * derivative

    # shed speed on sharp curves (large |error|) so corrections can catch up
    speed = BASE_SPEED * (1.0 - min(abs(error), 1.0) * CURVE_SLOWDOWN)

    left_speed = max(min(speed + correction, MAX_SPEED), -MAX_SPEED)
    right_speed = max(min(speed - correction, MAX_SPEED), -MAX_SPEED)
    return left_speed, right_speed


def detect_color(sensors):
    """Identify the colour of the box/zone in front from the RGB sensor.

    TODO (participants): compare color_r / color_g / color_b and return the
    dominant colour once it is above a confidence threshold.
    """
    r, g, b = sensors['color_r'], sensors['color_g'], sensors['color_b']
    if r - max(g, b) > COLOR_CONFIDENCE:
        return "red"
    if b - max(r, g) > COLOR_CONFIDENCE:
        return "blue"
    return None


def should_pick(sensors, carrying_color):
    """Decide whether to send a PICK this cycle.

    TODO (participants): use sensors['proximity'] to detect that a box is close.
    """
    if carrying_color is not None:
        return False
    return sensors['proximity'] < PICK_PROXIMITY


def should_drop(sensors, carrying_color):
    """Decide whether to send a DROP this cycle.

    TODO (participants): only drop when carrying_color is not None AND you have
    navigated to the drop zone that matches carrying_color.
    """
    if carrying_color is None:
        return False
    if sensors['proximity'] >= DROP_PROXIMITY:
        return False
    return detect_color(sensors) == carrying_color


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