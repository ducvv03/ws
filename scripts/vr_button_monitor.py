#!/usr/bin/env python3
"""Live VR button monitor for a running recording session.

Subscribes to /vr_buttons (sensor_msgs/msg/Joy, buttons[0:4] = a, b, x, y) and logs each
rising-edge button press plus the episode/pick/place state machine, driven by the same
dora-openarm-ros2/src/dora_ros2_bridge/phases.yaml config used by scripts/bag_split_episodes.py
so the two never drift apart.

Current phases.yaml semantics (episode_start_button: x, cancel_button: b, phases: pick(x->y),
place(y->a)):
    X -> starts a new episode AND starts the "pick" phase
    Y -> ends "pick" AND starts "place" (same press, no separate press needed)
    A -> ends "place" -> episode complete
    B -> cancel: discard the current episode attempt at any point after it started, go back to
         waiting for X (press X again to redo the same episode)

Usage:
    python3 scripts/vr_button_monitor.py
    python3 scripts/vr_button_monitor.py --topic /vr_buttons --phases-file path/to/phases.yaml
"""

import argparse
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Joy

_BUTTON_NAMES = ("a", "b", "x", "y")  # matches msg.buttons[0:4] order used across this repo

DEFAULT_PHASES_FILE = (
    Path(__file__).resolve().parent.parent
    / "dora-openarm-ros2" / "src" / "dora_ros2_bridge" / "phases.yaml"
)


def load_phases(path: Path) -> tuple[str | None, str | None, list[dict]]:
    with open(path) as f:
        config = yaml.safe_load(f)
    phases = config["phases"]
    for phase in phases:
        if phase["start_button"] not in _BUTTON_NAMES or phase["end_button"] not in _BUTTON_NAMES:
            raise ValueError(f"phase {phase['name']!r} references a button outside {_BUTTON_NAMES}")
    return config.get("episode_start_button"), config.get("cancel_button"), phases


class VrButtonMonitor(Node):
    def __init__(self, phases_file: Path, topic: str):
        super().__init__("vr_button_monitor")

        self.episode_start_button, self.cancel_button, self.phases = load_phases(phases_file)
        if self.episode_start_button is None:
            self.episode_start_button = self.phases[0]["start_button"]

        self.episodes_done = 0
        # phase_index is None while waiting for an episode to start; otherwise an index into
        # self.phases. phase_started distinguishes "phase's start_button already pressed" from
        # "waiting for a fresh press of it" (only relevant when it differs from the previous
        # phase's end_button - see phases.yaml's header comment).
        self.phase_index: int | None = None
        self.phase_started = False
        self.prev_buttons = (0, 0, 0, 0)

        self.get_logger().info(
            f"Loaded {len(self.phases)} phase(s) from {phases_file}: "
            + " -> ".join(f"{p['name']}({p['start_button']}->{p['end_button']})" for p in self.phases)
        )
        self.get_logger().info(
            f"episode_start_button={self.episode_start_button!r}, cancel_button={self.cancel_button!r}"
        )
        self._log_waiting_for_episode()

        self.create_subscription(Joy, topic, self._on_joy, 10)

    def _phase(self, index: int) -> dict:
        return self.phases[index]

    def _log_waiting_for_episode(self):
        self.get_logger().info(f"Waiting to start episode {self.episodes_done + 1} (press {self.episode_start_button!r}) ...")

    def _on_joy(self, msg: Joy):
        buttons = tuple(msg.buttons[:4]) + (0,) * max(0, 4 - len(msg.buttons))
        for name, prev, curr in zip(_BUTTON_NAMES, self.prev_buttons, buttons, strict=True):
            if prev == 0 and curr == 1:
                self._on_press(name)
        self.prev_buttons = buttons

    def _on_press(self, name: str):
        self.get_logger().info(f"Button pressed: {name.upper()}")

        # Waiting for a new episode to start.
        if self.phase_index is None:
            if name == self.episode_start_button:
                self.phase_index = 0
                phase = self._phase(0)
                self.phase_started = phase["start_button"] == self.episode_start_button
                self.get_logger().info(f"Episode {self.episodes_done + 1} started")
                if self.phase_started:
                    self.get_logger().info(f"-> phase '{phase['name']}' started: {phase['task']}")
                else:
                    self.get_logger().info(f"Waiting for phase '{phase['name']}' to start (press {phase['start_button']!r})")
            else:
                self.get_logger().warning(
                    f"Ignored {name.upper()!r}: waiting for episode start ({self.episode_start_button!r})"
                )
            return

        # Cancel: valid any time after the episode has started, whether or not the current
        # phase has technically been "started" yet.
        if self.cancel_button is not None and name == self.cancel_button:
            phase = self._phase(self.phase_index)
            self.get_logger().warning(
                f"Episode {self.episodes_done + 1} CANCELLED during phase '{phase['name']}' "
                f"— press {self.episode_start_button!r} to redo it"
            )
            self.phase_index = None
            self.phase_started = False
            self._log_waiting_for_episode()
            return

        phase = self._phase(self.phase_index)

        if not self.phase_started:
            if name == phase["start_button"]:
                self.phase_started = True
                self.get_logger().info(f"Phase '{phase['name']}' started: {phase['task']}")
            else:
                self.get_logger().warning(
                    f"Ignored {name.upper()!r}: expected start button {phase['start_button']!r} for phase '{phase['name']}'"
                )
            return

        if name != phase["end_button"]:
            self.get_logger().warning(
                f"Ignored {name.upper()!r}: expected end button {phase['end_button']!r} for phase '{phase['name']}'"
            )
            return

        self.get_logger().info(f"Phase '{phase['name']}' finished")

        if self.phase_index == len(self.phases) - 1:
            self.episodes_done += 1
            self.get_logger().info(f"Episode complete! Total episodes collected: {self.episodes_done}")
            self.phase_index = None
            self.phase_started = False
            self._log_waiting_for_episode()
            return

        self.phase_index += 1
        next_phase = self._phase(self.phase_index)
        self.phase_started = next_phase["start_button"] == name
        if self.phase_started:
            self.get_logger().info(f"-> phase '{next_phase['name']}' started: {next_phase['task']}")
        else:
            self.get_logger().info(f"Waiting for phase '{next_phase['name']}' to start (press {next_phase['start_button']!r})")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/vr_buttons", help="VR button topic (sensor_msgs/msg/Joy)")
    parser.add_argument("--phases-file", default=str(DEFAULT_PHASES_FILE), help="Path to phases.yaml")
    args = parser.parse_args()

    rclpy.init()
    node = VrButtonMonitor(Path(args.phases_file), args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
