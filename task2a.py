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
BASE_SPEED = 3.0
KP = 5.2
KI = 0.0
KD = 1.4
MAX_CORRECTION = 3.2
MAX_SPEED = 4.0
BRANCH_STEER = 1.15
BRANCH_MAX_CORRECTION = 2.35
BRANCH_BASE_SPEED = 2.25

PICK_PROXIMITY_THRESHOLD = 0.22
DROP_PROXIMITY_THRESHOLD = 0.16
COLOR_CONFIDENCE_THRESHOLD = 0.05
LINE_THRESHOLD = 0.03

# stop-at-box behavior
PICK_HOLD_SECONDS = 0.35
PICK_TRY_FRAMES = 1          # one PICK command per approach window
MAX_PICK_ATTEMPTS = 3        # attempt windows before backing off and retrying approach
BACKOFF_SECONDS = 0.6        # how long to reverse when a pick keeps failing

# The arena has one branch node after pickup:
#   red = left branch, blue = straight branch, green = right branch.
TURN_BY_COLOR = {
    # Positive branch bias turns this robot toward the left arm at the node;
    # negative turns it toward the right arm.
    "red": 1,
    "blue": 0,
    "green": -1,
}

SENSOR_WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]

# ---- Internal state --------------------------------------------------------
_integral_error = 0.0
_prev_error = 0.0
_last_line_seen_sign = 1.0

_carrying_box_state = False
_detected_color_state = None
_color_hist = []

# pickup state machine: search -> settle -> pick_try -> backoff
_pick_state = "search"
_pick_timer = 0
_hold_until = 0.0
_drop_inhibit_timer = 0
_pick_attempts = 0
_backoff_until = 0.0

# junction tracking while carrying
_junction_count = 0
_was_at_junction = False
_branch_committed = False

_was_frozen = False  # set True whenever control_loop freezes the robot

DT = 0.05


def _line_error(sensors):
    vals = [sensors.get(key, 0.0) for key in SENSOR_ORDER]
    lo = min(vals)
    hi = max(vals)
    contrast = hi - lo
    if contrast < LINE_THRESHOLD:
        return None

    bright_strengths = [(v - lo) / contrast for v in vals]
    dark_strengths = [(hi - v) / contrast for v in vals]

    # Some scenes report "line = high"; others effectively report the dark
    # track as the strongest signal. Pick the narrower pattern each frame.
    strengths = (
        bright_strengths
        if sum(bright_strengths) <= sum(dark_strengths)
        else dark_strengths
    )

    total_signal = sum(strengths)
    if total_signal < LINE_THRESHOLD:
        return None

    return sum(w * v for w, v in zip(SENSOR_WEIGHTS, strengths)) / total_signal


def _is_object_close(sensors, threshold):
    # Protocol: proximity is distance in metres; 1.0 means no object
    p = sensors.get('proximity', 1.0)
    return 0.0 < p < threshold


def control_loop(sensors):
    global _integral_error, _prev_error, _last_line_seen_sign
    global _pick_state, _hold_until, _was_frozen, _backoff_until

    # --- Backing off after repeated failed pick attempts ---
    if not _carrying_box_state and time.time() < _backoff_until:
        _was_frozen = True
        return -1.0, -1.0  # reverse slightly to re-approach

    # --- Freeze while attempting a pick ---
    if (
        not _carrying_box_state
        and (_pick_state == "pick_try" or time.time() < _hold_until)
    ):
        _was_frozen = True  # BUGFIX: this flag was declared but never set,
        return 0.0, 0.0     # which let stale PID error terms cause a swerve
                             # the instant the robot resumed after a pick.

    error = _line_error(sensors)

    if error is None:
        search_speed = 1.2
        left = -search_speed * _last_line_seen_sign
        right = search_speed * _last_line_seen_sign
        return left, right

    if _was_frozen:
        _prev_error = error       # kills the derivative kick this frame
        _integral_error = 0.0     # don't carry pre-pick/backoff windup into
        _was_frozen = False       # the resumed drive

    if abs(error) > 1e-6:
        _last_line_seen_sign = 1.0 if error > 0 else -1.0

    line_vals = [sensors.get(key, 0.0) for key in SENSOR_ORDER]
    at_junction = sum(1 for v in line_vals if v > 0.35) >= 4

    branch_turn = 0
    if at_junction and _carrying_box_state and not _branch_committed:
        branch_turn = TURN_BY_COLOR.get(_detected_color_state, 0)

    _track_junction(at_junction)

    _integral_error += error * DT
    derivative = (error - _prev_error) / DT
    _prev_error = error

    correction = KP * error + KI * _integral_error + KD * derivative
    correction += BRANCH_STEER * branch_turn

    correction_limit = BRANCH_MAX_CORRECTION if branch_turn else MAX_CORRECTION
    correction = max(-correction_limit, min(correction_limit, correction))

    base_speed = BRANCH_BASE_SPEED if branch_turn else BASE_SPEED
    left = base_speed + correction
    right = base_speed - correction
    left = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    return left, right


def _track_junction(at_junction):
    """Count the first branch node and commit only after leaving it.

    Red/green need a steering bias for several frames while the robot is on
    the node. If we commit on the rising edge, the bias disappears instantly
    and the robot continues on the default blue/straight route.
    """
    global _junction_count, _was_at_junction, _branch_committed
    if not _carrying_box_state:
        _was_at_junction = at_junction
        return
    if at_junction and not _was_at_junction:
        _junction_count += 1
    if not at_junction and _was_at_junction and _junction_count >= 1:
        _branch_committed = True
    _was_at_junction = at_junction


def detect_color(sensors):
    global _color_hist

    r = sensors.get('color_r', 0.0)
    g = sensors.get('color_g', 0.0)
    b = sensors.get('color_b', 0.0)

    if max(r, g, b) < COLOR_CONFIDENCE_THRESHOLD:
        return None

    _color_hist.append((r, g, b))
    if len(_color_hist) > 8:
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
    if (m - second) < 0.02:
        return None
    return best


def should_pick(sensors, carrying_box):
    global _carrying_box_state, _pick_state, _pick_timer, _hold_until
    global _drop_inhibit_timer, _pick_attempts, _backoff_until
    global _junction_count, _was_at_junction

    _carrying_box_state = carrying_box

    if carrying_box:
        _pick_state = "done"
        return False

    if time.time() < _backoff_until:
        return False

    box_seen = _is_object_close(sensors, PICK_PROXIMITY_THRESHOLD)
    if box_seen and _pick_state == "search":
        _pick_state = "pick_try"
        _pick_timer = PICK_TRY_FRAMES
        _hold_until = time.time() + PICK_HOLD_SECONDS

    if _pick_state == "pick_try":
        _pick_timer -= 1
        _hold_until = time.time() + PICK_HOLD_SECONDS
        _drop_inhibit_timer = 20
        if _pick_timer <= 0:
            _pick_attempts += 1
            if _pick_attempts >= MAX_PICK_ATTEMPTS:
                # Repeated failed attempts: back off and re-approach instead
                # of looping search<->pick_try forever in place.
                _pick_attempts = 0
                _backoff_until = time.time() + BACKOFF_SECONDS
            _pick_state = "search"
        return True

    return False


def _on_pick_success():
    """Call this (see note in main-loop guidance) to reset attempt state
    and start counting junctions fresh for the drop-by-node logic."""
    global _pick_attempts, _junction_count, _was_at_junction, _branch_committed
    _pick_attempts = 0
    _junction_count = 0
    _was_at_junction = False
    _branch_committed = False


def should_drop(sensors, carrying_box, detected_color):
    global _carrying_box_state, _detected_color_state, _drop_inhibit_timer

    was_carrying = _carrying_box_state
    _carrying_box_state = carrying_box
    if detected_color is not None:
        _detected_color_state = detected_color

    if not carrying_box:
        return False

    if not was_carrying:
        # Just picked up this frame — reset node/attempt tracking.
        _on_pick_success()

    if _drop_inhibit_timer > 0:
        _drop_inhibit_timer -= 1
        return False

    # The first junction chooses the branch. Drop only after entering a branch
    # and reaching the dead-end/drop-zone object there.
    return _branch_committed and _is_object_close(sensors, DROP_PROXIMITY_THRESHOLD)

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
            if sensors is None:
                time.sleep(0.01)
                continue
            last_sensors = sensors

            if detected_color is None:
               p = last_sensors.get('proximity', 1.0)
               near_box = carrying_box or (0.0 < p < PICK_PROXIMITY_THRESHOLD)
               if near_box:
                 color = detect_color(last_sensors)
                 if color is not None:
                   detected_color = color
                   print(f"Color detected: {color!r}")

            # --- Pick ---
            if not carrying_box and should_pick(last_sensors, carrying_box):
              client.send_motor_command(0.0, 0.0)
              time.sleep(0.35)  # short settle
              success = client.send_pick()
              print(f"PICK attempted  — success={success}")
              if success:
               carrying_box = True
              time.sleep(0.05)
              continue

            # --- Drop ---
            if carrying_box and should_drop(last_sensors, carrying_box, detected_color):
                success = client.send_drop()
                print(f"DROP attempted  — success={success}")
                if success:
                    carrying_box = False
                    detected_color = None
                time.sleep(0.05)
                continue

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
