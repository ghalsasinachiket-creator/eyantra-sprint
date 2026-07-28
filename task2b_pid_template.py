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

from curses import raw
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

KP, KI, KD = 0.65, 0.0, 0.15    # PID gains (reduced from 3.0/1.5 — was causing violent oscillation)
BASE_SPEED = 2.0               # rad/s, nominal forward speed (reduced from 3.0)
MAX_SPEED = 4.0                # rad/s, wheel speed clamp
CURVE_SLOWDOWN = 0.5           # fraction of BASE_SPEED shed when error is large

LINE_PRESENT_THRESHOLD = 0.35  # below this total "line-ness" => treat as a gap
PICK_PROXIMITY = 0.08          # metres — TUNE: distance that counts as "box is here"
DROP_PROXIMITY = 0.12          # metres — TUNE: distance that counts as "at the zone"
COLOR_CONFIDENCE = 0.15        # margin the dominant channel must lead by

# Maximum rate of change of error per second (for derivative clamping).
# Prevents the D-term from exploding when the error jumps suddenly between
# cycles — e.g. when the robot re-acquires the line after a gap, or the
# sensor noise spikes. Units: error-units / second.
MAX_D_ERROR_PER_SEC = 5.0

# PID state carried between control_loop() calls (the function itself is
# only given the latest sensor reading, so the running terms live here).
_pid_state = {'integral': 0.0, 'last_error': 0.0, 'last_time': None}
_debug_state = {'n': 0}

def _line_signals(sensors):
    """Return per-sensor line strength independent of line colour.

    The useful signal is not the absolute reflectance value; it is the
    sensor that differs from the local background. A median baseline makes
    white-on-black and black-on-white look identical from frame 1, with no
    polarity state to get temporarily stuck in the wrong direction.
    """
    raw = [sensors[name] for name in SENSOR_ORDER]
    baseline = sorted(raw)[2]
    return [abs(value - baseline) for value in raw]

def control_loop(sensors):
    """Return (left_speed, right_speed) for the current sensor reading.

    TODO (participants): replace the placeholder with your PID controller.
    """
    signals = _line_signals(sensors)
    total = sum(signals)

    now = time.time()
    first_call = _pid_state['last_time'] is None
    if first_call:
        dt = 0.05
    else:
        dt = max(now - _pid_state['last_time'], 1e-3)
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

    if first_call:
        # First call: no previous error to differentiate against.
        # Reset integral and set derivative = 0 to avoid a massive kick.
        _pid_state['integral'] = 0.0
        _pid_state['last_error'] = error
        derivative = 0.0
    else:
        _pid_state['integral'] += error * dt
        # Clamp the derivative to prevent spikes when error jumps suddenly
        # (e.g. re-acquiring line after a gap, or sensor noise).
        raw_derivative = (error - _pid_state['last_error']) / dt
        derivative = max(min(raw_derivative, MAX_D_ERROR_PER_SEC), -MAX_D_ERROR_PER_SEC)
        _pid_state['last_error'] = error

    correction = KP * error + KI * _pid_state['integral'] + KD * derivative

    # shed speed on sharp curves (large |error|) so corrections can catch up
    speed = BASE_SPEED * (1.0 - min(abs(error), 1.0) * CURVE_SLOWDOWN)
    correction = max(min(correction, speed), -speed)

    left_speed = max(min(speed + correction, MAX_SPEED), 0.0)
    right_speed = max(min(speed - correction, MAX_SPEED), 0.0)
    if _debug_state['n'] < 30:
        raw = [sensors[name] for name in SENSOR_ORDER]
        print(f"[{_debug_state['n']:02d}] raw={[round(v, 3) for v in raw]} "
              f"med={sorted(raw)[2]:.3f} "
              f"signals={[round(v, 3) for v in signals]} total={total:.3f} "
              f"error={error:.3f} dt={dt:.3f} d={derivative:.3f} "
              f"corr={correction:.3f} speed={speed:.3f} "
              f"L={left_speed:.3f} R={right_speed:.3f}",
              flush=True)
        _debug_state['n'] += 1
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
