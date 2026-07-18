import time

from connector import CoppeliaClient

DEBUG = False
LOG_EVERY_N = 25
SLEEP_BETWEEN_CYCLES = 0.02
PICK_DROP_REPLY_HALT = True


SENSOR_ORDER = [
    "left_corner",
    "left",
    "middle",
    "right",
    "right_corner",
]

WEIGHTS = [-2, -1, 0, 1, 2]

# PID values from your successful Task 1A controller.
KP = 0.65
KI = 0.00
KD = 0.15

PICK_DISTANCE = 0.14
DROP_DISTANCE = 0.19
MIN_DROP_READY_CYCLES_STRAIGHT = 15  # blue: short straight run, proximity-based

# Red/green: proximity is unusable (proven flat/constant regardless of
# distance travelled). Instead we wait for the line to genuinely end,
# but only after this many post-turn cycles -- otherwise a brief
# mid-turn blip could fire it too early. User's own sim observation
# placed real arrival at ~105-120 cycles, so require most of that
# before trusting a "line lost" as the real end rather than a blip.
MIN_TRAVEL_BEFORE_LINE_END_DROP = 100

# Sharp turn at the junction for red/green branches.
TURN_CYCLES = 14
MAX_TURN_EXTENSION = 20  # hard cap so a bad turn can't run away forever

# Forced straight-through for blue, so the trident's side branches
# don't pull the PID sideways right at the split.
STRAIGHT_CYCLES = 8

JUNCTION_MIN_CYCLES = 40
OFF_CENTER_DEVIATION = 0.12  # how far from centre counts as "clearly on the branch"
JUNCTION_CONFIRM_CYCLES = 3  # consecutive matching frames needed before committing to a turn

# Real debug data: normal on-line total ~0.80, fully-lost total ~0.00,
# the actual junction reads a diffuse ~0.28. Band chosen with margin
# on both sides of that observed value.
DIFFUSE_TOTAL_MIN = 0.08
DIFFUSE_TOTAL_MAX = 0.55


previous_error = 0.0
integral = 0.0
lost_streak = 0
last_correction = 0.0
last_base_speed = 1.5

target_color = None
carrying_started = False
post_pick_cycles = 0

junction_taken = False
turn_cycles_left = 0
straight_cycles_left = 0
turn_extension_used = 0
drop_ready = False
drop_ready_cycles = 0
_drop_debug_count = 0
junction_confirm_streak = 0


def line_features(sensors):
    """Find the line from the local difference between the five sensors."""

    values = [sensors.get(name, 0.0) for name in SENSOR_ORDER]

    baseline = sorted(values)[2]
    deviations = [abs(value - baseline) for value in values]

    total = sum(deviations)
    max_deviation = max(deviations)

    weighted_sum = sum(
        deviation * weight
        for deviation, weight in zip(deviations, WEIGHTS)
    )

    position = weighted_sum / total if total > 0.0001 else 0.0

    active_count = sum(
        deviation > 0.10
        for deviation in deviations
    )

    uniform_surface = (max(values) - min(values)) < 0.06

    return {
        "total": total,
        "max_deviation": max_deviation,
        "position": position,
        "active_count": active_count,
        "uniform_surface": uniform_surface,
    }


def junction_detected(sensors):
    """Detect the centre three-way intersection.

    Real debug data showed normal single-line tracking ALWAYS produces
    active_count == 1 (only the middle sensor reads "on line"; the rest
    read the off-line background value). active_count never reaches 2
    or 3 during ordinary tracking, so any threshold >= 2 on active_count
    alone was unreachable — that's why junction detection kept failing.

    The real junction instead showed up as a DIFFUSE reading: all five
    sensors landing on moderate, blended values instead of the sharp
    binary on/off pattern (e.g. 0.735, 0.718, 0.664, 0.636, 0.533).
    That gives a "total" deviation sum in a mid-range band -- clearly
    below normal on-line total (~0.80) but clearly above fully-lost
    total (~0.00). That mid-range band is what we now key off.
    """

    features = line_features(sensors)

    diffuse_transition = (
        DIFFUSE_TOTAL_MIN < features["total"] < DIFFUSE_TOTAL_MAX
    )

    return features["active_count"] >= 2 or diffuse_transition


def dominant_color(sensors):
    """Return red, green, blue, or None."""

    r = sensors.get("color_r", 0.0)
    g = sensors.get("color_g", 0.0)
    b = sensors.get("color_b", 0.0)

    colors = {
        "red": r,
        "green": g,
        "blue": b,
    }

    if max(colors.values()) < 0.02:
        return None

    return max(colors, key=colors.get)


def detect_color(sensors):
    """Read the box colour while the package is near the robot."""

    global target_color

    if sensors.get("proximity", 1.0) > 0.25:
        return None

    color = dominant_color(sensors)

    if DEBUG:
        print(
            f"Box RGB: "
            f"R={sensors['color_r']:.3f}, "
            f"G={sensors['color_g']:.3f}, "
            f"B={sensors['color_b']:.3f} "
            f"-> {color}"
        )

    if color is not None:
        target_color = color

    return color


def should_pick(sensors, carrying_box):
    """Pick only after box colour is known."""

    if carrying_box:
        return False

    if target_color is None:
        return False

    return sensors.get("proximity", 1.0) < PICK_DISTANCE


def should_drop(sensors, carrying_box, detected_color):
    """
    Drop only after the chosen branch is entered and the robot has
    genuinely reached the end of it.

    Real debug data proved proximity is unusable for red/green: it sat
    at a perfectly constant ~0.123 for 200+ cycles no matter how far
    the robot travelled along the curve -- almost certainly because
    the sensor is picking up the curve's own central post at a fixed
    radius, not the actual end-of-branch marker. No proximity
    threshold can ever distinguish "cruising the curve" from "arrived"
    there.

    What DOES mark arrival for red/green: the drawn trajectory line
    physically ends at the bracket, so "the line is genuinely lost
    after a solid stretch of post-turn travel" is the real signal --
    confirmed by the user watching the sim (drop should happen right
    around when the line disappears for good, ~cycle 105-120).

    Blue keeps the original proximity-based check since that one
    already works correctly.
    """

    global carrying_started
    global drop_ready_cycles

    if not carrying_box:
        return False

    carrying_started = True

    if not drop_ready:
        return False

    drop_ready_cycles += 1

    if target_color == "blue":
        return _should_drop_proximity(sensors)

    return _should_drop_line_end(sensors)


def _should_drop_proximity(sensors):
    """Blue: proximity-based drop, unchanged from the working version."""

    if drop_ready_cycles < MIN_DROP_READY_CYCLES_STRAIGHT:
        return False

    proximity = sensors.get("proximity", 1.0)

    global _drop_debug_count
    _drop_debug_count += 1
    if DEBUG and _drop_debug_count % 20 == 0:
        print(f"[drop_ready] cycles={drop_ready_cycles}, "
              f"proximity={proximity:.3f} (need < {DROP_DISTANCE})")

    if proximity < DROP_DISTANCE:
        if DEBUG:
            print(f"Final marker detected: proximity={proximity:.3f}, "
                  f"drop_ready_cycles={drop_ready_cycles}")
        return True

    return False


def _should_drop_line_end(sensors):
    """Red/green: drop when the line is genuinely lost after enough
    post-turn travel to have plausibly reached the end of the branch.
    Proximity is not used at all here -- proven unreliable above.
    """

    if DEBUG and drop_ready_cycles % 10 == 0:
        print(f"[buffer wait] cycles={drop_ready_cycles}, "
              f"proximity={sensors.get('proximity', 1.0):.3f} (unused for this color)")

    if drop_ready_cycles < MIN_TRAVEL_BEFORE_LINE_END_DROP:
        return False

    features = line_features(sensors)
    line_lost_now = features["total"] < 0.08 or features["max_deviation"] < 0.08

    if line_lost_now:
        if DEBUG:
            print(f"Line end reached after {drop_ready_cycles} post-turn cycles "
                  f"(total={features['total']:.3f}, max_dev={features['max_deviation']:.3f}). "
                  f"Dropping here.")
        return True

    return False


def control_loop(sensors):
    """PID line-following plus red/blue/green branch selection."""

    global previous_error
    global integral
    global lost_streak
    global last_correction
    global last_base_speed
    global post_pick_cycles
    global junction_taken
    global turn_cycles_left
    global straight_cycles_left
    global turn_extension_used
    global drop_ready
    global drop_ready_cycles
    global junction_confirm_streak

    if carrying_started:
        post_pick_cycles += 1

        if DEBUG and post_pick_cycles % 15 == 0 and not junction_taken:
            debug_f = line_features(sensors)
            print(f"[post_pick] cycle={post_pick_cycles}, "
                  f"active_count={debug_f['active_count']}, "
                  f"total={debug_f['total']:.3f}, "
                  f"max_dev={debug_f['max_deviation']:.3f}, "
                  f"values={[round(sensors.get(n, 0.0), 3) for n in SENSOR_ORDER]}")

    # -------------------------------------------------------------
    # Detect the junction and choose the target route.
    #
    # A single matching frame isn't enough to commit to a turn — a
    # narrow bracket-shaped marker along the trunk can momentarily
    # look like a junction (extra edges / uniform low reading) for
    # a cycle or two as the sensor sweeps past it. A real trident is
    # wide, so the robot will keep reading "junction-like" for many
    # consecutive cycles as it approaches. Require several in a row
    # before committing, and reset the streak the moment a frame
    # doesn't match.
    # -------------------------------------------------------------
    if (
        carrying_started
        and target_color is not None
        and not junction_taken
        and post_pick_cycles >= JUNCTION_MIN_CYCLES
    ):
        if junction_detected(sensors):
            junction_confirm_streak += 1
        else:
            junction_confirm_streak = 0

        if junction_confirm_streak >= JUNCTION_CONFIRM_CYCLES:
            junction_taken = True

            if target_color == "blue":
                turn_cycles_left = 0
                straight_cycles_left = STRAIGHT_CYCLES
            else:
                turn_cycles_left = TURN_CYCLES
                straight_cycles_left = 0

            if DEBUG:
                debug_features = line_features(sensors)
                print(f"Junction detected. Taking {target_color} route. "
                      f"active_count={debug_features['active_count']}, "
                      f"total={debug_features['total']:.3f}, "
                      f"values={[sensors.get(n, 0.0) for n in SENSOR_ORDER]}")

        elif junction_confirm_streak > 0:
            # Mid-confirmation: we're inside (or just entering) the
            # diffuse zone but haven't hit JUNCTION_CONFIRM_CYCLES yet.
            # Slow to a crawl instead of running full-speed PID, so we
            # don't sail past the (brief) diffuse zone and out into the
            # open gap before we've had a chance to commit to the turn.
            if DEBUG:
                print(f"[confirming junction] streak={junction_confirm_streak}, "
                      f"total={line_features(sensors)['total']:.3f}")
            return 0.5, 0.5

    # -------------------------------------------------------------
    # Blue: force straight through the trident so side-branch lines
    # don't drag the PID off-centre right at the split.
    # -------------------------------------------------------------
    if junction_taken and straight_cycles_left > 0:
        straight_cycles_left -= 1
        return 1.7, 1.7

    # -------------------------------------------------------------
    # Sharp turn into the selected red/green branch.
    #   red   -> left turn  (red branch is on the left of the map)
    #   green -> right turn (green branch is on the right of the map)
    # -------------------------------------------------------------
    if junction_taken and turn_cycles_left > 0:
        if turn_cycles_left == TURN_CYCLES and DEBUG:
            print(f"Starting {target_color} turn. cycles={TURN_CYCLES}, "
                  f"position={line_features(sensors)['position']:.3f}")

        turn_cycles_left -= 1

        if turn_cycles_left == 0 and DEBUG:
            print(f"Turn cycles finished. position={line_features(sensors)['position']:.3f}")

        if target_color == "red":
            # Sharp left.
            return 0.5, 2.3

        if target_color == "green":
            # Sharp right.
            return 2.3, 0.5

    # -------------------------------------------------------------
    # Before handing back to PID, confirm the robot is actually
    # off-centre on the curved branch. If it's still reading close
    # to straight-on, the turn was too short — extend it a little
    # rather than silently falling through to plain PID (which would
    # just re-lock onto the trunk and look like "going straight").
    # -------------------------------------------------------------
    if (
        junction_taken
        and turn_cycles_left == 0
        and straight_cycles_left == 0
        and not drop_ready
        and target_color in ("red", "green")
    ):
        features_check = line_features(sensors)

        if (
            features_check["max_deviation"] < OFF_CENTER_DEVIATION
            and turn_extension_used < MAX_TURN_EXTENSION
        ):
            turn_extension_used += 1
            if target_color == "red":
                return 0.5, 2.3
            return 2.3, 0.5

    if junction_taken and turn_cycles_left == 0 and straight_cycles_left == 0:
        if not drop_ready:
            if DEBUG:
                print("Turn complete. drop_ready=True, resuming PID toward marker.")
            drop_ready_cycles = 0
        drop_ready = True

    # -------------------------------------------------------------
    # PID line following from Task 1A.
    # -------------------------------------------------------------
    features = line_features(sensors)

    line_lost = (
        features["total"] < 0.08
        or features["max_deviation"] < 0.08
    )

    if line_lost:
        lost_streak += 1

        if lost_streak == 1 and junction_taken and DEBUG:
            print(
                f"Line lost after junction. target={target_color}, "
                f"turn_cycles_left={turn_cycles_left}, drop_ready={drop_ready}"
            )

        if drop_ready:
            # We already know we're on the right branch, close to the
            # marker — don't oscillate searching for the line, just creep
            # forward. Taper the speed down as we get closer so we don't
            # fly past the marker before the drop actually fires.
            proximity_now = sensors.get("proximity", 1.0)

            if proximity_now < 0.30:
                creep = 0.35
            elif proximity_now < 0.55:
                creep = 0.55
            else:
                creep = 0.8

            return creep, creep

        if lost_streak <= 6:
            base_speed = last_base_speed
            correction = last_correction
        else:
            search_error = 0.6 if previous_error >= 0 else -0.6
            base_speed = 1.0
            correction = search_error * 0.6

        correction = max(-0.9, min(0.9, correction))

        left = max(0.0, min(4.0, base_speed - correction))
        right = max(0.0, min(4.0, base_speed + correction))

        return left, right

    lost_streak = 0

    position = max(-1.5, min(1.5, features["position"]))

    # Same wheel-direction convention as the working Task 1A code.
    error = -position

    integral += error
    integral = max(-5.0, min(5.0, integral))

    derivative = error - previous_error
    correction = (KP * error) + (KI * integral) + (KD * derivative)

    correction = max(-1.0, min(1.0, correction))

    previous_error = error

    base_speed = 1.7 - (0.45 * abs(error))
    base_speed = max(1.0, min(1.7, base_speed))

    last_correction = correction
    last_base_speed = base_speed

    left = max(0.0, min(4.0, base_speed - correction))
    right = max(0.0, min(4.0, base_speed + correction))

    return left, right


def main():
    global carrying_started, post_pick_cycles, junction_taken, turn_cycles_left, straight_cycles_left, turn_extension_used, drop_ready, drop_ready_cycles, junction_confirm_streak, lost_streak, previous_error, integral, target_color
    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print("Connected to bridge_v2_2a. Running... (Ctrl+C to stop)")

    carrying_box = False
    detected_color = None

    pick_in_progress = False
    drop_in_progress = False
    last_log_cycle = 0
    cycle = 0

    try:
        while True:
            cycle += 1

            # When waiting for PICK/DROP replies, DO NOT read sensors or
            # continue the control loop (prevents socket/reentrant issues).
            if pick_in_progress or drop_in_progress:
                time.sleep(SLEEP_BETWEEN_CYCLES)
                continue

            # Use only new sensor packets.
            sensors = client.receive_sensor_data()
            if sensors is None:
                time.sleep(SLEEP_BETWEEN_CYCLES)
                continue

            # Detect package colour.
            if (not carrying_box) and detected_color is None:
                color = detect_color(sensors)
                if color is not None:
                    detected_color = color
                    if DEBUG:
                        print(f"Target color set to: {detected_color}")

            # Pick package (stop motors first).
            if (not carrying_box) and (detected_color is not None) and should_pick(sensors, carrying_box):
                client.send_motor_command(0.0, 0.0)
                pick_in_progress = True
                try:
                    success = client.send_pick()
                finally:
                    pick_in_progress = False

                if success:
                    carrying_box = True
                    if DEBUG:
                        print(f"Carrying {detected_color} box.")
                # If pick failed, keep searching (do not call control_loop this cycle).
                time.sleep(SLEEP_BETWEEN_CYCLES)
                continue

            # Drop package (stop motors first).
            if carrying_box and should_drop(sensors, carrying_box, detected_color):
                client.send_motor_command(0.0, 0.0)
                drop_in_progress = True
                try:
                    success = client.send_drop()
                finally:
                    drop_in_progress = False

                if success:
                    if DEBUG:
                        print("Task completed.")
                    break

                if DEBUG:
                    print("DROP rejected. Continuing on the line.")
                time.sleep(SLEEP_BETWEEN_CYCLES)
                continue

            # Drive robot.
            left, right = control_loop(sensors)
            client.send_motor_command(left, right)

            if DEBUG and cycle - last_log_cycle >= LOG_EVERY_N:
                last_log_cycle = cycle
                f = line_features(sensors)
                print(f"[drive] cycle={cycle} pos={f['position']:.3f} total={f['total']:.3f}")

            time.sleep(SLEEP_BETWEEN_CYCLES)

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