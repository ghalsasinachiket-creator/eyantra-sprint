"""
===================================================
    eLSI Sprint 1 - Task 1A : PID Line Following
    Team ID: [403] -- revised control_loop (v2)
===================================================

WHY THIS IS FASTER THAN v1
---------------------------
1. FIXED: dead tuning constants. The old control_loop() redefined its own
   local Kp_l/Ki_l/Kd_l/BASE/BOOST_BASE/MAXV/I_LIM/LINE_T/CONTRAST_T, which
   *shadowed* the module-level Kp/Ki/Kd/BASE_SPEED/MAX_SPEED/etc. Editing
   the top-of-file constants therefore had zero effect on the robot -- this
   is almost certainly why tuning "did nothing" before. Now there is exactly
   one place to change each parameter, and control_loop() actually uses it.

2. FIXED: wheels could never reverse. In v1, `correction` was clamped to
   +/- dynamic_base *before* being applied to left/right, so the inner
   wheel (dynamic_base - correction) had a hard floor of 0. On a near-180
   hairpin -- exactly the loops you circled -- that forces a wide, slow arc
   instead of a tight pivot turn. Now only the *final* wheel speeds are
   clamped (to +/- MAXV), so the inner wheel is free to go negative when
   the turn is sharp enough to need it.

3. CHANGED: the straight_count > 12-consecutive-clean-frames "boost" gate
   is gone. In a comb of back-to-back hairpins there's never enough clean
   straight track between turns to build up 12 frames, so that whole
   section ran stuck on the slow branch the entire time. Replaced with a
   continuous curvature-based target speed -- no gate to fail to unlock.

4. ADDED: gain scheduling (gentle Kp/Kd near center, aggressive Kp/Kd on
   sharp error) and an anticipatory term using the derivative, so the
   robot starts shedding speed *before* it's already deep in a turn, and
   corners more crisply once there. This is the anticipatory
   derivative-based speed scaling you were already planning to try.

IMPORTANT: contest rules say only control_loop() (and its constants) should
change for submission. The dt-logging fix in main() below is for your own
debugging of loop frequency / lost-line events in the comb section -- treat
it as local-only and revert main() before you submit if scoring diffs the
whole file.
"""

import time

from connector_task1a import CoppeliaClient

SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

WEIGHTS = {
    'left_corner': -2.0,
    'left': -1.0,
    'middle': 0.0,
    'right': 1.0,
    'right_corner': 2.0,
}

# ---- Single source of truth for every tunable. Nothing inside
#      control_loop() redefines these anymore -- edit here only. ----
KP_STRAIGHT, KD_STRAIGHT = 1.00, 0.35   # gentle gains near center -> smooth cruise
KP_TURN,     KD_TURN     = 1.70, 0.85   # aggressive gains on sharp error -> crisp cornering
KI = 0.0

CRUISE_SPEED   = 3.6   # target speed on straights / gentle curves
MIN_TURN_SPEED = 1.6   # floor speed even mid-hairpin -- keep moving, don't crawl
MAXV           = 5.0   # actual motor ceiling. Nudge this up/down to find the sim's real limit.

I_LIM = 2.0
LINE_T = 0.030
CONTRAST_T = 0.045
ERROR_FILTER = 0.35     # weight on the *new* reading; higher = less lag, more twitch

prev_error = 0.0
integral = 0.0
prev_time = None



def control_loop(sensors):
    global prev_error, integral

    if not hasattr(control_loop, "lost_count"):
        control_loop.lost_count = 0
        control_loop.filtered_error = 0.0

    vals = [sensors[n] for n in SENSOR_ORDER]
    lo, hi = min(vals), max(vals)
    c = hi - lo

    if c < CONTRAST_T:
        strengths = [0.0] * 5
    else:
        bright = [(v - lo) / c for v in vals]
        dark = [(hi - v) / c for v in vals]
        strengths = bright if sum(bright) <= sum(dark) else dark

    total = sum(strengths)
    line_found = total > LINE_T

    if line_found:
        raw_error = sum(s * WEIGHTS[n] for s, n in zip(strengths, SENSOR_ORDER)) / total
        control_loop.filtered_error = (
            (1 - ERROR_FILTER) * control_loop.filtered_error + ERROR_FILTER * raw_error
        )
        error = control_loop.filtered_error
        control_loop.lost_count = 0
    else:
        control_loop.lost_count += 1
        sign = 1.0 if prev_error >= 0 else -1.0
        error = prev_error * 0.94 if control_loop.lost_count <= 4 else sign * 1.05
        integral = 0.0

    # ---- gain scheduling: gentle on straights, aggressive in corners ----
    severity = min(abs(error), 1.5) / 1.5     # 0 = dead straight, 1 = full hairpin
    Kp = KP_STRAIGHT + (KP_TURN - KP_STRAIGHT) * severity
    Kd = KD_STRAIGHT + (KD_TURN - KD_STRAIGHT) * severity

    integral += error
    integral = max(-I_LIM, min(I_LIM, integral))
    derivative = error - prev_error
    correction = Kp * error + KI * integral + Kd * derivative

    # ---- continuous, anticipatory speed curve (replaces the boost gate) ----
    anticip = abs(error) + 0.6 * abs(derivative)   # brake *before* fully inside the turn
    curve_factor = min(anticip / 1.6, 1.0)
    target_speed = CRUISE_SPEED - (CRUISE_SPEED - MIN_TURN_SPEED) * curve_factor

    if not line_found:
        target_speed = MIN_TURN_SPEED if control_loop.lost_count <= 8 else MIN_TURN_SPEED * 0.85

    left = target_speed + correction
    right = target_speed - correction

    # Clip the wheels independently -- NOT the correction beforehand.
    # This is what allows the inner wheel to reverse for a tight pivot.
    left = max(-MAXV, min(MAXV, left))
    right = max(-MAXV, min(MAXV, right))

    left = max(-3.5, min(3.5, left))
    right = max(-3.5, min(3.5, right))

    prev_error = error
    return left, right



def main():
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_task1a. Running... (Ctrl+C to stop)")

    last_t = time.time()
    dt_samples = []

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is None:
                time.sleep(0.02)
                continue

            now = time.time()
            dt = now - last_t
            last_t = now
            dt_samples.append(dt)
            # v1 had `% 1 == 0`, which is always true (mod 1 is always 0), so it
            # printed every frame instead of every 25, and divided by a hardcoded
            # 25 even before 25 samples existed. Both fixed below -- debug only,
            # revert before submitting if the file is diffed for scoring.
            if len(dt_samples) % 25 == 0:
                window = dt_samples[-25:]
                avg = sum(window) / len(window)
                print(f"avg dt over last {len(window)} samples: {avg:.4f}s  (~{1/avg:.1f} Hz)", flush=True)

            left, right = control_loop(sensors)
            client.send_motor_command(left, right)

            time.sleep(0.005)
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