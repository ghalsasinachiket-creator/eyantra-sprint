"""
===================================================
    eLSI Sprint 1 - Task 1B : Q-Learning
===================================================
 
Participant template.
 
HOW TO RUN
  1. Open the Task 1B scene in CoppeliaSim and press Play.
  2. Start the bridge:   ./bridge_v1_task1b --eval   (or bridge_task1b.py --eval)
  3. Train:              python task1b.py --mode train
     Test (no learning): python task1b.py --mode test
 
MODES
  train : choose actions with exploration AND update the Q-table.
          The Q-table is saved to disk on exit (including on disconnect/Ctrl+C).
  test  : load the saved Q-table, act greedily, and DO NOT update it.
 
WHAT YOU IMPLEMENT
  get_state()     - how to turn the 5 sensor values into a discrete state.
  get_reward()    - how good the latest reading is.
  choose_action() - which action to take in a given state (the policy).
 
Team ID: [ 403]
"""
 
import time
import os
import pickle
import random
import argparse
 
from connector_task1b import CoppeliaClient
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']
ACTIONS = [
    (0.6, 0.6),    # 0: straight
    (0.2, 0.7),    # 1: soft left
    (0.7, 0.2),    # 2: soft right
    (-0.3, 0.9),   # 3: sharp left (small reverse bias for a tighter pivot)
    (0.9, -0.3),   # 4: sharp right
]
 
# Hyperparameters for tuning
ALPHA = 0.2
GAMMA = 0.95
EPSILON = 0.3
 
DEVIATION_THRESH = 0.03
Q_TABLE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "q_table.pkl"
)
 
# =============================================================================
# TODO (participants): implement get_state(), get_reward()
# and choose_action().
# You may also add your own helper functions in this section.
# =============================================================================
 
def get_state(sensors):
    values = [sensors[name] for name in SENSOR_ORDER]
    low = min(values)
    high = max(values)
    contrast = high - low

    if contrast < DEVIATION_THRESH:
        return (0, 0)

    bright_strengths = [(value - low) / contrast for value in values]
    dark_strengths = [(high - value) / contrast for value in values]
    position_weights = [0.55, 1.05, 1.35, 1.05, 0.55]

    def build_candidate(strengths):
        peak = max(strengths)
        if peak < 0.45:
            return [False] * len(strengths), peak
        active = [strength >= max(0.42, peak * 0.58) for strength in strengths]
        return active, peak

    def score_candidate(active, peak):
        count = sum(active)
        if count == 0:
            return -1.0
        weighted_position = sum(
            weight for is_active, weight in zip(active, position_weights)
            if is_active
        ) / count
        wide_penalty = 0.28 * max(0, count - 2)
        edge_penalty = 0.22 * int(active[0] or active[4])
        return peak + weighted_position - wide_penalty - edge_penalty

    bright_active, bright_peak = build_candidate(bright_strengths)
    dark_active, dark_peak = build_candidate(dark_strengths)

    if score_candidate(bright_active, bright_peak) >= score_candidate(dark_active, dark_peak):
        active = bright_active
        peak = bright_peak
    else:
        active = dark_active
        peak = dark_peak

    if peak < 0.45:
        return (0, 0)

    mask = 0
    for active_sensor in active:
        mask = (mask << 1) | int(active_sensor)

    return (mask, 1)
 
 
def get_reward(sensors, state):
    mask, detected = state
 
    if not detected:
        return -100.0
 
    lc = (mask >> 4) & 1
    l  = (mask >> 3) & 1
    m  = (mask >> 2) & 1
    r  = (mask >> 1) & 1
    rc = mask & 1

    if m and not (l or r or lc or rc):
        return 60.0
    if m and (l or r) and not (lc or rc):
        return 35.0
    if l and not (r or lc or rc):
        return 15.0
    if r and not (l or lc or rc):
        return 15.0
    if lc and not rc:
        return -20.0
    if rc and not lc:
        return -20.0
    if l and r and not m:
        return -35.0
    return -50.0
 
 
def heuristic_action(mask):
    lc = (mask >> 4) & 1
    l  = (mask >> 3) & 1
    r  = (mask >> 1) & 1
    rc = mask & 1
 
    if lc:
        return 3   # sharp left
    if rc:
        return 4   # sharp right
    if l and not r:
        return 1   # soft left
    if r and not l:
        return 2   # soft right
    return 0        # straight
 
 
def choose_action(agent, state, training):
    mask, detected = state

    if not detected:
        choose_action._lost_count = getattr(choose_action, "_lost_count", 0) + 1
        last = getattr(choose_action, "last_turn", 0)

        if choose_action._lost_count > 30:   # ~0.6s of continuous no-detection
            choose_action._lost_count = 0
            new_dir = 4 if last == 3 else 3
            choose_action.last_turn = new_dir
            return new_dir

        return last

    choose_action._lost_count = 0

    agent._ensure(state)

    safe_action = heuristic_action(mask)
    lc = (mask >> 4) & 1
    l  = (mask >> 3) & 1
    m  = (mask >> 2) & 1
    r  = (mask >> 1) & 1
    rc = mask & 1

    if lc or rc:
        action = safe_action
    elif (l and not r) or (r and not l):
        action = safe_action
    elif training and random.random() < min(agent.epsilon, 0.08):
        if m:
            action = random.choice([0, 0, 1, 2])
        elif l and not r:
            action = random.choice([1, 3])
        elif r and not l:
            action = random.choice([2, 4])
        else:
            action = safe_action
    else:
        q_values = agent.q_table[state]
        best_q = max(q_values)
        safe_q = q_values[safe_action]
        action = q_values.index(best_q)
        if best_q == min(q_values) or best_q < safe_q + 5.0:
            action = safe_action

    if action in (3, 4):
        choose_action.last_turn = action
    elif l and not r:
        choose_action.last_turn = 1
    elif r and not l:
        choose_action.last_turn = 2

    return action
 
# =============================================================================
#  Q-learning agent (Don't Edit this)
# =============================================================================
class QLearningAgent:
    def __init__(self, n_actions, alpha, gamma, epsilon, path):
        self.n_actions = n_actions
        self.alpha = alpha
        self.gamma = gamma
        self.epsilon = epsilon
        self.path = path
        self.q_table = {}
 
    def _ensure(self, state):
        if state not in self.q_table:
            q_values = [0.0] * self.n_actions
            mask, detected = state
            if detected:
                best_guess = heuristic_action(mask)
                q_values[best_guess] = 0.5
            self.q_table[state] = q_values
        elif len(self.q_table[state]) != self.n_actions:
            old = self.q_table[state]
            if len(old) < self.n_actions:
                old.extend([0.0] * (self.n_actions - len(old)))
            else:
                del old[self.n_actions:]
 
    def update(self, state, action, reward, next_state):
        self._ensure(state)
        self._ensure(next_state)
 
        best_next = max(self.q_table[next_state])
 
        td_target = reward + self.gamma * best_next
 
        self.q_table[state][action] += self.alpha * (
            td_target - self.q_table[state][action]
        )
 
    def load(self):
        if os.path.exists(self.path):
            with open(self.path, "rb") as f:
                self.q_table = pickle.load(f)
 
            print(
                f"Loaded Q-table ({len(self.q_table)} states) from {self.path}"
            )
 
            return True
 
        return False
 
    def save(self):
        with open(self.path, "wb") as f:
            pickle.dump(self.q_table, f)
 
        print(
            f"Saved Q-table ({len(self.q_table)} states) to {self.path}"
        )
 
 
# =============================================================================
#  Main loop
# =============================================================================
def run(mode):
 
    training = (mode == "train")
 
    agent = QLearningAgent(
        len(ACTIONS),
        ALPHA,
        GAMMA,
        EPSILON,
        Q_TABLE_PATH
    )
 
    loaded = agent.load()
 
    if not training and not loaded:
        print(
            "ERROR: test mode needs a trained Q-table. Run --mode train first."
        )
        return
 
    client = CoppeliaClient(
        host="127.0.0.1",
        port=50002
    )
 
    client.connect()
 
    print(
        f"Connected to bridge_task1b. Mode = {mode}. (Ctrl+C to stop)"
    )
 
    last_sensors = None
    prev_state = None
    prev_action = None
    reward = 0.0
 
    try:
 
        while True:
 
            try:
                sensors = client.receive_sensor_data()
            except (ConnectionResetError, ConnectionAbortedError, OSError):
                print(
                    "\nConnection to bridge lost (sim likely stopped). "
                    "Ending episode."
                )
                break
 
            print("Sensor values:", sensors)
 
            if sensors is not None:
                last_sensors = sensors
 
            if last_sensors is None:
                time.sleep(0.02)
                continue
 
            state = get_state(last_sensors)
 
            reward = get_reward(
                last_sensors,
                state
            )
 
            if training and prev_state is not None:
 
                agent.update(
                    prev_state,
                    prev_action,
                    reward,
                    state
                )
 
            action = choose_action(
                agent,
                state,
                training
            )
 
            print("State:", state)
            print("Reward:", reward)
            print("Action:", action)
 
            left, right = ACTIONS[action]
            print("Motors:", left, right)
 
            try:
                client.send_motor_command(
                    left,
                    right,
                    state=list(state),
                    reward=reward,
                    action=action,
                )
            except (ConnectionResetError, ConnectionAbortedError, OSError):
                print(
                    "\nConnection to bridge lost (sim likely stopped). "
                    "Ending episode."
                )
                break
 
            prev_state = state
            prev_action = action
 
            time.sleep(0.02)
 
    except KeyboardInterrupt:
 
        print("\nStopping...")
 
    finally:
 
        try:
 
            client.send_motor_command(
                0.0,
                0.0,
                state=0,
                reward=0.0,
                action=0
            )
 
        except Exception:
            pass
 
        client.close()
 
        if training:
            agent.save()
 
 
def main():
 
    parser = argparse.ArgumentParser(
        description="Task 1B - Q-Learning"
    )
 
    parser.add_argument(
        "--mode",
        choices=["train", "test"],
        default="train",
        help="train: explore + update Q-table; test: greedy, no update"
    )
 
    args = parser.parse_args()
 
    run(args.mode)
 
 
if __name__ == "__main__":
    main()
