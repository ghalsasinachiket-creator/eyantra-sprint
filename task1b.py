"""
===================================================
    eLSI Sprint 1 - Task 1B : Q-Learning
===================================================

Participant template.

HOW TO RUN
  1. Open the Task 1B scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_task1b.py --eval
  3. Train:              python3 task1b_template.py --mode train
     Test (no learning): python3 task1b_template.py --mode test

MODES
  train : choose actions with exploration AND update the Q-table.
          The Q-table is saved to disk on exit.
  test  : load the saved Q-table, act greedily, and DO NOT update it.

WHAT YOU IMPLEMENT
  get_state()     - how to turn the 5 sensor values into a discrete state.
  get_reward()    - how good the latest reading is.
  choose_action() - which action to take in a given state (the policy).

Team ID: [ XXX ]
"""

import time
import os
import pickle
import random
import argparse

from connector_task1b import CoppeliaClient

# The five line sensors, ordered left -> right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

# Action set: index -> (left_speed, right_speed). 
ACTIONS = [
   
    (0, 0),  # Action 0: placeholder, REPLACE THIS with actual Motor speeds.
    (0, 0),  # Action 1: REPLACE THIS.
]
# Hyper parameter for tuning
ALPHA = 0
GAMMA = 0
EPSILON = 0

# Saved next to this script, so it doesn't depend on the launch directory.
Q_TABLE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "q_table.pkl")


LINE_THRESHOLD = 0.5
LINE_WEIGHTS = (0.25, 1.0, 2.0, 1.0, 0.25)


def _reward_for_state(state):
    line_count = sum(state)
    if line_count == 0:
        return -5.0

    reward = sum(weight for active, weight in zip(state, LINE_WEIGHTS) if active)
    if state[2]:
        reward += 2.0
    if state[0] or state[4]:
        reward -= 1.0
    if line_count > 2:
        reward -= 0.5 * (line_count - 2)

    return reward


REWARD_BY_STATE = {
    tuple((mask >> bit) & 1 for bit in range(4, -1, -1)): _reward_for_state(
        tuple((mask >> bit) & 1 for bit in range(4, -1, -1))
    )
    for mask in range(32)
}


def get_state(sensors):
    """Convert the sensor reading into a discrete Q-table state.

    Args:
        sensors (dict): the 5 line sensors, each a float in [0.0, 1.0], keyed by
            'left_corner', 'left', 'middle', 'right', 'right_corner'.

    Returns:
        A HASHABLE, discrete state (e.g. a tuple of ints, an int, or a string).
        It is used as a key into the Q-table, so the number of distinct values
        it can take should be finite and reasonably small.

    """
    return tuple(1 if sensors[name] < LINE_THRESHOLD else 0 for name in SENSOR_ORDER)


def get_reward(sensors, state):
    """Compute the reward for the latest reading (the result of the last action).

    Args:
        sensors (dict): the 5 line sensors, each a float in [0.0, 1.0].
        state: the discrete state returned by get_state(sensors).

    Returns:
        A single float reward. Higher means better (e.g. reward staying centered
        on the line and penalise losing it).
    """
    return REWARD_BY_STATE[state]


def choose_action(agent, state, training):
    """Pick an action index for the current state (the policy).

    Args:
        agent: the QLearningAgent (do NOT modify the class itself). Useful bits:
            - agent._ensure(state)   : make sure the Q-table has a row for `state`
                                       (call this before reading agent.q_table[state]).
            - agent.q_table[state]   : list of Q-values, one per action.
            - agent.n_actions        : number of available actions.
            - agent.epsilon          : exploration rate (the EPSILON constant).
        state: the current discrete state (from get_state).
        training (bool): True under --mode train, False under --mode test.

    Returns:
        An integer action index in the range [0, agent.n_actions). This indexes
        into ACTIONS to get the (left, right) wheel speeds.

    Hint: a common policy is epsilon-greedy while training (with probability
    epsilon pick a random action, otherwise the best-known one) and purely
    greedy while testing. The `random` module is already imported.

    """
    agent._ensure(state)
    if training and random.random() < agent.epsilon:
        return random.randrange(agent.n_actions)

    q_values = agent.q_table[state]
    best_action = 0
    best_value = q_values[0]
    for action in range(1, agent.n_actions):
        value = q_values[action]
        if value > best_value:
            best_action = action
            best_value = value
    return best_action


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
            self.q_table[state] = [0.0] * self.n_actions

    def update(self, state, action, reward, next_state):
        """Q-learning update. Called only in train mode."""
        self._ensure(state)
        self._ensure(next_state)
        best_next = max(self.q_table[next_state])
        td_target = reward + self.gamma * best_next
        self.q_table[state][action] += self.alpha * (td_target - self.q_table[state][action])

    def load(self):
        if os.path.exists(self.path):
            with open(self.path, "rb") as f:
                self.q_table = pickle.load(f)
            print(f"Loaded Q-table ({len(self.q_table)} states) from {self.path}")
            return True
        return False

    def save(self):
        with open(self.path, "wb") as f:
            pickle.dump(self.q_table, f)
        print(f"Saved Q-table ({len(self.q_table)} states) to {self.path}")


# =============================================================================
#  Main loop
# =============================================================================
def run(mode):
    training = (mode == "train")

    agent = QLearningAgent(len(ACTIONS), ALPHA, GAMMA, EPSILON, Q_TABLE_PATH)
    loaded = agent.load()
    if not training and not loaded:
        print("ERROR: test mode needs a trained Q-table. Run --mode train first.")
        return

    client = CoppeliaClient(host="127.0.0.1", port=50002)
    client.connect()
    print(f"Connected to bridge_task1b. Mode = {mode}. (Ctrl+C to stop)")

    last_sensors = None
    prev_state = None
    prev_action = None
    reward = 0.0

    try:
        while True:
            sensors = client.receive_sensor_data()
            print(f"Sensors: {sensors} ")
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            state = get_state(last_sensors)
            reward = get_reward(last_sensors, state)
            if training and prev_state is not None:
                agent.update(prev_state, prev_action, reward, state)

            action = choose_action(agent, state, training)
            left, right = ACTIONS[action]
            client.send_motor_command(
                left, right,
                state=list(state),  
                reward=reward,
                action=action,
            )

            prev_state, prev_action = state, action
            time.sleep(0.05)   # ~20 Hz
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        try:
            client.send_motor_command(0.0, 0.0, state=0, reward=0.0, action=0)
        except Exception:
            pass
        client.close()
        if training:
            agent.save()   # persist what was learned


def main():
    parser = argparse.ArgumentParser(description="Task 1B - Q-Learning")
    parser.add_argument("--mode", choices=["train", "test"], default="train",
                        help="train: explore + update Q-table; test: greedy, no update")
    args = parser.parse_args()
    run(args.mode)


if __name__ == "__main__":
    main()
