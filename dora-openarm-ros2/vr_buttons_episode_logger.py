#!/usr/bin/env python3
"""Echo /vr_buttons presses and track episode/phase progress from phases.yaml.

Subscribes to /vr_buttons (sensor_msgs/Joy, published by dora_ros2_bridge —
see src/dora_ros2_bridge/main.py), logs which of a/b/x/y was just pressed
(rising edge only), and drives the same episode_start_button/cancel_button/
phases state machine documented in src/dora_ros2_bridge/phases.yaml (the
config bag_split_episodes.py uses to split a recorded bag after the fact) —
so this gives the same episode/phase boundaries live, plus a running
completed-episode count, while recording.

Plain rclpy script, not a Dora node — run directly after sourcing a ROS 2
install, alongside the running dataflow:

    python3 vr_buttons_episode_logger.py
    python3 vr_buttons_episode_logger.py --phases-file path/to/phases.yaml
"""

import argparse
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy

# Must match dora_ros2_bridge's qos_best_effort for /vr_buttons — a RELIABLE
# subscriber can't receive from a BEST_EFFORT publisher.
_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)

# buttons[0..3] = a, b, x, y, matching dora_ros2_bridge's /vr_buttons layout.
BUTTON_NAMES = ["a", "b", "x", "y"]

_DEFAULT_PHASES_FILE = Path(__file__).parent / "src" / "dora_ros2_bridge" / "phases.yaml"


class PhasesConfig:
    """Parsed phases.yaml: episode_start_button, cancel_button, and phase list."""

    def __init__(self, path: Path) -> None:
        """Load and validate phases.yaml from `path`."""
        with path.open() as f:
            data = yaml.safe_load(f)

        self.phases: list[dict] = data["phases"]
        if not self.phases:
            raise ValueError(f"{path}: 'phases' must not be empty")
        self.episode_start_button: str | None = data.get("episode_start_button")
        self.cancel_button: str | None = data.get("cancel_button")

    @property
    def effective_start_button(self) -> str:
        """The button that begins a new episode: episode_start_button, or phase 0's start_button."""
        return self.episode_start_button or self.phases[0]["start_button"]


class EpisodeSession:
    """Tracks episode/phase state across button presses, per PhasesConfig's rules."""

    def __init__(self, config: PhasesConfig) -> None:
        """Start in the 'waiting for a new episode' state."""
        self.config = config
        self.episode_active = False
        self.phase_idx = -1  # -1: no phase currently running
        self.waiting_for_phase = 0  # index of the phase whose start_button we expect next
        self.episode_count = 0

    def handle(self, btn: str, log) -> None:
        """Advance the state machine for a just-pressed button `btn` ('a'/'b'/'x'/'y')."""
        cfg = self.config

        if cfg.cancel_button and btn == cfg.cancel_button and self.episode_active:
            log(f"CANCEL — discarding episode attempt #{self.episode_count + 1} "
                f"(press '{cfg.effective_start_button}' to start over)")
            self.episode_active = False
            self.phase_idx = -1
            self.waiting_for_phase = 0
            return

        if not self.episode_active:
            if btn == cfg.effective_start_button:
                self.episode_active = True
                self.waiting_for_phase = 0
                log(f"episode #{self.episode_count + 1} STARTED")
                if cfg.phases[0]["start_button"] == btn:
                    self.phase_idx = 0
                    self.waiting_for_phase = 1
                    log(f"phase '{cfg.phases[0]['name']}' STARTED")
            else:
                log(f"(ignored — waiting for '{cfg.effective_start_button}' to start an episode)")
            return

        if self.phase_idx == -1:
            expected = cfg.phases[self.waiting_for_phase]
            if btn == expected["start_button"]:
                self.phase_idx = self.waiting_for_phase
                log(f"phase '{expected['name']}' STARTED")
            else:
                log(f"(ignored — waiting for '{expected['start_button']}' "
                    f"to start phase '{expected['name']}')")
            return

        current = cfg.phases[self.phase_idx]
        if btn != current["end_button"]:
            log(f"(ignored — waiting for '{current['end_button']}' "
                f"to end phase '{current['name']}')")
            return

        log(f"phase '{current['name']}' ENDED")
        next_idx = self.phase_idx + 1
        if next_idx >= len(cfg.phases):
            self.episode_count += 1
            self.episode_active = False
            self.phase_idx = -1
            self.waiting_for_phase = 0
            log(f"episode #{self.episode_count} COMPLETE  "
                f"(total episodes recorded this session: {self.episode_count})")
            return

        nxt = cfg.phases[next_idx]
        if nxt["start_button"] == btn:
            self.phase_idx = next_idx
            self.waiting_for_phase = next_idx + 1
            log(f"phase '{nxt['name']}' STARTED (same button as previous end)")
        else:
            self.phase_idx = -1
            self.waiting_for_phase = next_idx


class VrButtonsEpisodeLogger(Node):
    """Subscribes to /vr_buttons and logs button presses + episode/phase progress."""

    def __init__(self, config: PhasesConfig) -> None:
        """Create the /vr_buttons subscription and initialize the episode session."""
        super().__init__("vr_buttons_episode_logger")
        self._session = EpisodeSession(config)
        self._prev_buttons = [0, 0, 0, 0]
        self.create_subscription(Joy, "/vr_buttons", self._cb, _QOS)
        self.get_logger().info(
            f"watching /vr_buttons — episode_start_button="
            f"{config.episode_start_button or '(same as phase 0 start)'} "
            f"cancel_button={config.cancel_button} "
            f"phases={[p['name'] for p in config.phases]}"
        )

    def _log(self, msg: str) -> None:
        print(f"[vr-buttons] {msg}", flush=True)

    def _cb(self, msg: Joy) -> None:
        buttons = list(msg.buttons)
        for i, name in enumerate(BUTTON_NAMES):
            if buttons[i] == 1 and self._prev_buttons[i] == 0:
                self._log(f"button '{name}' pressed")
                self._session.handle(name, self._log)
        self._prev_buttons = buttons


def main() -> None:
    """Parse CLI args and spin the vr_buttons_episode_logger node."""
    parser = argparse.ArgumentParser(description="Echo /vr_buttons and track episode progress")
    parser.add_argument(
        "--phases-file",
        type=Path,
        default=_DEFAULT_PHASES_FILE,
        help=f"Path to phases.yaml (default: {_DEFAULT_PHASES_FILE})",
    )
    args = parser.parse_args()

    config = PhasesConfig(args.phases_file)

    rclpy.init()
    node = VrButtonsEpisodeLogger(config)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
