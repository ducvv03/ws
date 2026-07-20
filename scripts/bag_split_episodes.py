#!/usr/bin/env python3
"""Splits a continuously-recorded rosbag2 (from record_session_tmux.sh) into per-episode
pick / hold / place task segments, using the VR button press timestamps already in the bag —
no video/motion heuristics needed, since the exact phase-transition instants are already in the
data as button-press events.

Button semantics (see ~/pnk/ws/docs/huong_dan_thu_du_lieu_vr.md), one full cycle per episode:
    A -> robot moves to position A (also the first event of the *next* episode's cycle)
    X -> marks start of the "pick" phase (operator teleops the grasp after this)
    B (1st in the cycle) -> robot moves to position B (ends "pick", starts "hold"/transport)
    Y -> marks start of the "place" phase (operator teleops the placement after this)
    B (2nd in the cycle) -> ends the episode (ends "place")

This script only extracts segment boundaries + task labels (episode index, phase, start/end
timestamps in nanoseconds) to a JSON file — it does NOT build a LeRobotDataset. That's a
separate, bigger conversion step (reading synchronized image/joint/tf messages per frame and
tagging each frame with the task label of whichever segment its timestamp falls into).

IMPORTANT: assumes the button topic is a `sensor_msgs/msg/Joy` with the 4 face buttons in
`msg.buttons[0:4]` as (a, b, x, y), matching lerobot.utils.vr_button_input's convention. Confirm
this bag's actual topic name/type first: `ros2 bag info <bag_path>`.

Usage:
    python3 bag_split_episodes.py <bag_path> --topic /vr_buttons --out segments.json
"""

import argparse
import json

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Joy

# Index into msg.buttons -> logical name, matching lerobot.utils.vr_button_input's convention.
_BUTTON_NAMES = ("a", "b", "x", "y")

TASK_LABELS = {
    "pick": "pick up the box",
    "hold": "carry the box to the drop-off location",
    "place": "place the box down",
}


def read_button_events(bag_path: str, topic: str, storage_id: str) -> list[tuple[int, str]]:
    """Returns a chronological list of (timestamp_ns, button_name) rising-edge press events."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in type_map:
        raise ValueError(f"Topic {topic!r} not found in bag. Available topics: {sorted(type_map)}")

    msg_type = get_message(type_map[topic])
    events: list[tuple[int, str]] = []
    prev_buttons = (0, 0, 0, 0)
    while reader.has_next():
        read_topic, data, t = reader.read_next()
        if read_topic != topic:
            continue
        msg: Joy = deserialize_message(data, msg_type)
        buttons = tuple(msg.buttons[:4]) + (0,) * max(0, 4 - len(msg.buttons))
        for name, prev, curr in zip(_BUTTON_NAMES, prev_buttons, buttons, strict=True):
            if prev == 0 and curr == 1:
                events.append((t, name))
        prev_buttons = buttons
    return events


def split_into_episodes(events: list[tuple[int, str]]) -> list[dict]:
    """Runs the A/X/B/Y/B state machine, returning [{episode, phase, t_start, t_end}, ...].

    Out-of-sequence button presses (accidental double-press, operator error) are logged and
    skipped rather than corrupting the whole run silently.
    """
    segments: list[dict] = []
    episode = 0
    state = "WAIT_A"
    t_x = t_b1 = t_y = None

    for t, name in events:
        if state == "WAIT_A" and name == "a":
            state = "AT_A"
        elif state == "AT_A" and name == "x":
            t_x = t
            state = "PICKING"
        elif state == "PICKING" and name == "b":
            t_b1 = t
            segments.append({"episode": episode, "phase": "pick", "t_start": t_x, "t_end": t_b1})
            state = "TRANSPORTING"
        elif state == "TRANSPORTING" and name == "y":
            t_y = t
            segments.append({"episode": episode, "phase": "hold", "t_start": t_b1, "t_end": t_y})
            state = "PLACING"
        elif state == "PLACING" and name == "b":
            segments.append({"episode": episode, "phase": "place", "t_start": t_y, "t_end": t})
            episode += 1
            state = "WAIT_A"
        else:
            print(f"WARNING: unexpected button {name!r} while in state {state!r} at t={t} — skipped")

    if state != "WAIT_A":
        print(f"WARNING: bag ended mid-episode {episode} (state={state}) — that episode is incomplete, dropped")

    return segments


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_path", help="Path to the rosbag2 directory (e.g. bags/take_box_20260716_...)")
    parser.add_argument("--topic", default="/vr_buttons", help="VR button topic (sensor_msgs/msg/Joy)")
    parser.add_argument("--storage-id", default="sqlite3", choices=["sqlite3", "mcap"])
    parser.add_argument("--out", default="segments.json", help="Output JSON path")
    args = parser.parse_args()

    events = read_button_events(args.bag_path, args.topic, args.storage_id)
    print(f"Found {len(events)} button press events on {args.topic!r}.")

    segments = split_into_episodes(events)
    for seg in segments:
        seg["task"] = TASK_LABELS[seg["phase"]]

    with open(args.out, "w") as f:
        json.dump(segments, f, indent=2)

    n_episodes = len({s["episode"] for s in segments})
    print(f"Wrote {len(segments)} segments across {n_episodes} complete episode(s) to {args.out}")


if __name__ == "__main__":
    main()
