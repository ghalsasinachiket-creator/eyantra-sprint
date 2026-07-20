"""
===================================================
  eLSI Sprint 1 - Task 2B : Q-Learning + Pick & Place (dual line)
===================================================

Participant template (RL variant).

TASK 2B
  Follow the track (white line on black AND black line on white) through the
  checkpoints, pick the red and blue boxes near the circle, drop each in its
  matching colour drop zone, then finish at the white box.
  Boxes are handled ONE AT A TIME.

  Line following is learned with Q-learning; picking/dropping are decided by
  simple rules you write (proximity + colour + where you've navigated to).

HOW TO RUN
  1. Open the Task 2B scene in CoppeliaSim.
  2. Start the bridge:   python3 bridge_v1_2b.py --eval
  3. Train:              python3 task2b_rl_template.py --mode train
     Test (no learning): python3 task2b_rl_template.py --mode test

WHAT YOU IMPLEMENT
  get_state()     - turn the 5 sensor values into a discrete state.
  get_reward()    - how good the latest reading is.
  choose_action() - the policy (which action for a given state).
  detect_color()  - identify the box colour from the RGB sensor.
  should_pick()   - decide when to pick (only when a box is right next to you).
  should_drop()   - decide when to drop (at the matching zone).

Team ID: [ XXX ]
"""

import time
import os
import pickle
import random
import argparse

from connector_2b import CoppeliaClient

# The five line sensors, ordered left -> right across the robot ([0.0, 1.0]).
SENSOR_ORDER = ['left_corner', 'left', 'middle', 'right', 'right_corner']

# Action set: index -> (left_speed, right_speed).
ACTIONS = [
    (0, 0),  # Action 0: placeholder, REPLACE THIS with actual motor speeds.
    (0, 0),  # Action 1: REPLACE THIS.
]
# Hyper-parameters for tuning
ALPHA = 0
GAMMA = 0
EPSILON = 0

# Saved next to this script, so it doesn't depend on the launch directory.
Q_TABLE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "q_table_2b.pkl")


# =============================================================================
#  TODO (participants): implement the functions below.
#  You may add your own helper functions in this section.
# =============================================================================
def get_state(sensors):
    """Convert the sensor reading into a discrete, HASHABLE Q-table state.

    Args:
        sensors (dict): the 5 line sensors, each [0.0, 1.0].

    Returns:
        A hashable, discrete state (tuple/int/str). The track has BOTH
        white-on-black and black-on-white sections, so make sure your state
        distinguishes "on line" in both regimes.

    TODO (participants): design your state representation and RETURN it.
    """
    state = None
    return state


def get_reward(sensors, state):
    """Reward for the latest reading (result of the last action).

    TODO (participants): higher = better (reward staying on the line, penalise
    losing it). RETURN a float.
    """
    reward = 0.0
    return reward


def choose_action(agent, state, training):
    """Pick an action index for the current state (the policy).

    Args:
        agent: the QLearningAgent. Useful bits:
            agent._ensure(state), agent.q_table[state], agent.n_actions, agent.epsilon.
        state: current discrete state.
        training (bool): True under --mode train.

    Returns:
        int action index in [0, agent.n_actions) — indexes into ACTIONS.

    TODO (participants): implement epsilon-greedy (train) / greedy (test).
    """
    agent._ensure(state)
    action = 0
    return action


def detect_color(sensors):
    """Return "red", "blue", or None from color_r/color_g/color_b.

    TODO (participants): return the dominant colour above a confidence threshold.
    """
    return None


def should_pick(sensors, carrying_color):
    """True to attempt a PICK this cycle (only when empty-handed and a box is
    right next to the gripper — use sensors['proximity']).

    TODO (participants).
    """
    return False


def should_drop(sensors, carrying_color):
    """True to attempt a DROP this cycle (only while carrying, once navigated to
    the zone matching carrying_color).

    TODO (participants).
    """
    return False


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
#  Main loop (Don't Edit this)
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
    print(f"Connected to bridge_v1_2b. Mode = {mode}. (Ctrl+C to stop)")

    last_sensors   = None
    prev_state     = None
    prev_action    = None
    carrying_color = None
    delivered      = 0

    try:
        while True:
            sensors = client.receive_sensor_data()
            if sensors is not None:
                last_sensors = sensors
            if last_sensors is None:
                time.sleep(0.02)
                continue

            # --- Pick / Drop (rule-based, independent of the Q-policy) ---
            if carrying_color is None and should_pick(last_sensors, carrying_color):
                colour_seen = detect_color(last_sensors)
                if client.send_pick():
                    carrying_color = colour_seen
                    print(f"PICK ok (saw {colour_seen!r})")
            if carrying_color is not None and should_drop(last_sensors, carrying_color):
                if client.send_drop():
                    delivered += 1
                    print(f"DROP ok ({carrying_color!r}); delivered {delivered}")
                    carrying_color = None

            # --- Q-learning line following ---
            state = get_state(last_sensors)
            reward = get_reward(last_sensors, state)
            if training and prev_state is not None:
                agent.update(prev_state, prev_action, reward, state)

            action = choose_action(agent, state, training)
            left, right = ACTIONS[action]
            client.send_motor_command(
                left, right,
                state=list(state) if isinstance(state, (list, tuple)) else state,
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
            agent.save()


def main():
    parser = argparse.ArgumentParser(description="Task 2B - Q-Learning + Pick & Place")
    parser.add_argument("--mode", choices=["train", "test"], default="train",
                        help="train: explore + update Q-table; test: greedy, no update")
    args = parser.parse_args()
    run(args.mode)


if __name__ == "__main__":
    main()
