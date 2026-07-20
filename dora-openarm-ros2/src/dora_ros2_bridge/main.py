# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Dora-to-ROS2 Bridge Node.

This node acts as a state holder and message translator between the Dora dataflow
and ROS 2 control interfaces. It forwards the right arm's joint position input
from the Dora graph to the robot, merged with the left arm's — if the
dataflow wires a left_position input (bimanual yamls); otherwise the left arm
holds a fixed pose, since only the right arm/gripper are teleoperated (the
single-arm yaml). It also forwards the VR controller's A/B/X/Y button states
as a sensor_msgs/Joy on the /vr_buttons topic. If the dataflow wires an
fk node's pose_right/pose_left output (as ee_pose_right/ee_pose_left), the
end-effector translation/orientation is also republished as
geometry_msgs/PoseStamped on /right_ee_pose and /left_ee_pose.

--mode sim (default): publishes a single merged sensor_msgs/JointState on
/joint_command.

--mode real: publishes separate trajectory_msgs/JointTrajectory per arm to
/left_joint_trajectory_controller/joint_trajectory and
/right_joint_trajectory_controller/joint_trajectory. Targets are ramped
toward (MAX_STEP per cycle) rather than applied directly, and resynced from
the controllers' real /*_joint_trajectory_controller/controller_state
feedback whenever they've drifted more than SYNC_THRESHOLD from what we last
commanded — real hardware shouldn't be commanded to jump instantly to a new
target the way simulation can be.
"""

import argparse
import threading
import time
import traceback

import dora
import numpy as np
import pyarrow as pa
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node as RclpyNode


def _log(msg: str) -> None:
    """Print a stdout diagnostic line, prefixed like this repo's other Dora nodes' logs."""
    print(f"[dora-to-ros2] {msg}", flush=True)


class RobotStateSubscriber(RclpyNode):
    """Tracks real physical joint feedback, used by --mode real's safety ramp."""

    def __init__(self, physical_state: dict) -> None:
        """Subscribe to both arms' controller_state feedback topics."""
        super().__init__("dora_bridge_state_subscriber")
        self._physical_state = physical_state
        self.create_subscription(
            JointTrajectoryControllerState,
            "/left_joint_trajectory_controller/controller_state",
            self._left_cb,
            10,
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            "/right_joint_trajectory_controller/controller_state",
            self._right_cb,
            10,
        )

    def _left_cb(self, msg: JointTrajectoryControllerState) -> None:
        if not self._physical_state["left_ready"]:
            _log("first LEFT controller_state feedback received "
                 "(/left_joint_trajectory_controller/controller_state)")
        self._physical_state["left"][:] = msg.feedback.positions[:7]
        self._physical_state["left_ready"] = True

    def _right_cb(self, msg: JointTrajectoryControllerState) -> None:
        if not self._physical_state["right_ready"]:
            _log("first RIGHT controller_state feedback received "
                 "(/right_joint_trajectory_controller/controller_state)")
        self._physical_state["right"][:] = msg.feedback.positions[:7]
        self._physical_state["right_ready"] = True


def _spin_robot_state_subscriber(physical_state: dict) -> None:
    try:
        rclpy.init()
        node = RobotStateSubscriber(physical_state)
        _log("RobotStateSubscriber thread: rclpy node initialized, spinning "
             "(subscribed to both arms' .../controller_state)")
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        _log("FATAL: RobotStateSubscriber thread crashed")
        traceback.print_exc()
        raise


def main() -> None:
    """Parse CLI args and run the Dora-to-ROS2 bridge node."""
    parser = argparse.ArgumentParser(description="Dora-to-ROS2 bridge node")
    parser.add_argument(
        "--mode",
        choices=["sim", "real"],
        default="sim",
        help="sim: merged JointState on /joint_command (default). real: "
        "separate JointTrajectory per arm, ramped toward target with real "
        "physical-feedback resync for hardware safety.",
    )
    args = parser.parse_args()
    _log(f"starting: mode={args.mode}")
    _run(args)


def _run(args: argparse.Namespace) -> None:
    # --- 1. ROS 2 Setup ---
    try:
        context = dora.Ros2Context()
        options = dora.Ros2NodeOptions(rosout=True)
        node = context.new_node("dora_to_ros2", "/openarm", options)
    except Exception:
        _log("FATAL: failed to create dora Ros2Context/node")
        traceback.print_exc()
        raise
    _log("ROS2 context/node created (dora_to_ros2 in /openarm namespace)")

    qos_arm = dora.Ros2QosPolicies(reliable=True)
    # Best-effort: the VR headset re-sends button state on every tick (up to
    # ~500 Hz) whether or not it changed. Reliable QoS made the writer block
    # trying to redeliver that firehose to /vr_buttons whenever nothing was
    # subscribed/keeping up, eventually erroring out with a publish timeout.
    qos_buttons = dora.Ros2QosPolicies(reliable=False)

    # --- 2. Define Publishers ---
    p_buttons = node.create_publisher(
        node.create_topic(
            "/vr_buttons",
            "sensor_msgs/Joy",
            qos_buttons,
        )
    )
    # Best-effort for the same reason as buttons: ee_pose events arrive at
    # whatever rate the ik/fk nodes solve at, which can be fast, and there's
    # no guarantee anything is subscribed.
    p_ee_pose_right = node.create_publisher(
        node.create_topic(
            "/right_ee_pose",
            "geometry_msgs/PoseStamped",
            qos_buttons,
        )
    )
    p_ee_pose_left = node.create_publisher(
        node.create_topic(
            "/left_ee_pose",
            "geometry_msgs/PoseStamped",
            qos_buttons,
        )
    )
    _log("common publishers ready: /vr_buttons, /right_ee_pose, /left_ee_pose")

    # --- 3. Pre-defined Constants ---
    EMPTY_F64 = np.array([], dtype=np.float64)
    EMPTY_F32 = np.array([], dtype=np.float32)

    # Default left-arm target (all joints 0, neutral gripper) used until/unless
    # a left_position event arrives. Yamls that don't wire left_position
    # (single-arm teleop) leave the left arm parked here forever; bimanual
    # yamls overwrite it in place as soon as the IK solver starts publishing
    # position_left.
    LEFT_FIXED = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    # buttons[0..3] = a, b, x, y — index into BUTTON_CACHE below.
    BUTTON_INDEX = {"button_a": 0, "button_b": 1, "button_x": 2, "button_y": 3}

    # --- 4. Helpers ---
    def now_stamp() -> dict:
        """Return the current time as a ROS2 stamp dict."""
        t = time.time()
        return {"sec": np.int32(int(t)), "nanosec": np.uint32(int((t % 1.0) * 1e9))}

    def make_joy_msg(stamp: dict, buttons: np.ndarray) -> dict:
        """Build a sensor_msgs/Joy message from the latest button states."""
        return {
            "header": {"stamp": stamp, "frame_id": ""},
            "axes": EMPTY_F32,
            "buttons": buttons,
        }

    # --- Revo2 hand (gripper) setup -------------------------------------------
    # The VR trigger drives the 4 non-thumb fingers of each Revo2 hand; the thumb
    # (indices 0,1) holds a fixed mock pose until a real thumb input is wired.
    # Published as trajectory_msgs/JointTrajectory to each hand's
    # <side>_revo2_hand_controller, in both --mode sim and --mode real.
    HAND_STAMP_ZERO = {"sec": np.int32(0), "nanosec": np.uint32(0)}
    p_l_hand = node.create_publisher(
        node.create_topic(
            "/left_revo2_hand_controller/joint_trajectory",
            "trajectory_msgs/JointTrajectory",
            qos_arm,
        )
    )
    p_r_hand = node.create_publisher(
        node.create_topic(
            "/right_revo2_hand_controller/joint_trajectory",
            "trajectory_msgs/JointTrajectory",
            qos_arm,
        )
    )
    _log("hand publishers ready: /left_revo2_hand_controller/joint_trajectory, "
         "/right_revo2_hand_controller/joint_trajectory")

    HAND_FINGERS = ["thumb_metacarpal", "thumb_proximal", "index_proximal",
                    "middle_proximal", "ring_proximal", "pinky_proximal"]
    NAMES_L_HAND = [f"left_{f}_joint" for f in HAND_FINGERS]
    NAMES_R_HAND = [f"right_{f}_joint" for f in HAND_FINGERS]

    # Finger closed limits (rad), same order as HAND_FINGERS (revo2 URDF upper
    # limits); every finger opens at 0.0.
    HAND_CLOSED = np.array([1.57, 1.03, 0.4, 0.5, 0.6, 0.7 ], dtype=np.float64)
    # Mock thumb pose (indices 0,1); the trigger only drives the fingers [2:].
    MOCK_THUMB = np.array([0.0, 0.0], dtype=np.float64)

    # Persistent full 6-joint target per hand: the trigger updates only the
    # 4-finger slice [2:], the thumb slice keeps MOCK_THUMB, and we always
    # publish all 6 so the goal is never partial.
    hand_target_l = np.zeros(6, dtype=np.float64)
    hand_target_r = np.zeros(6, dtype=np.float64)
    hand_target_l[0:2] = MOCK_THUMB
    hand_target_r[0:2] = MOCK_THUMB

    # The trigger reports roughly the SQUARE of its lever angle; undo it with a
    # power curve so grip tracks the pull (input clipped to [0, 1] upstream).
    GRIP_LINEARIZE = True

    def linearize_grip(g: float) -> float:
        """Undo the trigger's squared response so grip tracks the lever angle."""
        return g ** 0.5

    # Latest grip command (0..1) per hand; the trigger stream (~500 Hz) updates
    # it and each event ramps hand_target toward it by at most HAND_MAX_STEP.
    desired_grip_l = 0.0
    desired_grip_r = 0.0
    HAND_MAX_STEP = 0.03

    def make_hand_msg(names: list, positions: list) -> dict:
        """Build a single-point JointTrajectory message for the Revo2 hand."""
        return {
            "header": {"stamp": HAND_STAMP_ZERO, "frame_id": ""},
            "joint_names": names,
            "points": [
                {
                    "positions": positions,
                    "velocities": EMPTY_F64,
                    "accelerations": EMPTY_F64,
                    "effort": EMPTY_F64,
                    "time_from_start": {"sec": np.int32(0), "nanosec": np.uint32(0)},
                }
            ],}
    def extract_pose(value: pa.Array) -> np.ndarray:
        """Read an fk-node pose event: a {"pose": [...]} struct, or a flat array."""
        if pa.types.is_struct(value.type):
            value = value.field("pose")[0].values
        return np.array(value, dtype=np.float32)

    def make_ee_pose_msg(stamp: dict, pose: np.ndarray) -> dict:
        """Build a geometry_msgs/PoseStamped from an FK [px,py,pz,qw,qx,qy,qz,...] pose.

        Only the first 7 values are used; a trailing gripper value (as the fk
        node emits) is ignored here — that's already published separately.
        """
        return {
            "header": {"stamp": stamp, "frame_id": ""},
            "pose": {
                "position": {
                    "x": float(pose[0]),
                    "y": float(pose[1]),
                    "z": float(pose[2]),
                },
                "orientation": {
                    "x": float(pose[4]),
                    "y": float(pose[5]),
                    "z": float(pose[6]),
                    "w": float(pose[3]),
                },
            },
        }

    if args.mode == "sim":
        p_joint_cmd = node.create_publisher(
            node.create_topic(
                "/joint_command",
                "sensor_msgs/JointState",
                qos_arm,
            )
        )
        _log("mode=sim: publisher ready: /joint_command")

        # sim-only: distal phalanx joints have no independent target (the
        # trigger/joystick streams only drive metacarpal + proximal), so each
        # mirrors its same-finger proximal joint's value (hand_target[1:6]:
        # thumb_proximal, index_proximal, middle_proximal, ring_proximal,
        # pinky_proximal) — a simple stand-in for the real driver's mimic
        # joints, which --mode real doesn't need since it never targets the
        # sim-only *_distal_joint names on the hand controller topics.
        DISTAL_FINGERS = ["thumb_distal", "index_distal", "middle_distal",
                           "ring_distal", "pinky_distal"]
        NAMES_L_HAND_DISTAL = [f"left_{f}_joint" for f in DISTAL_FINGERS]
        NAMES_R_HAND_DISTAL = [f"right_{f}_joint" for f in DISTAL_FINGERS]

        # Merged joint order: left/right arm joints interleaved, then each
        # side's Revo2 hand joints (left hand, then right hand) — 6 driven
        # joints followed by the 5 mirrored distal joints. The
        # openarm_*_ee_tcp_joint names are intentionally omitted — no data
        # for those is published anywhere.
        JOINT_NAMES = []
        for i in range(7):
            JOINT_NAMES.append(f"openarm_left_joint{i + 1}")
            JOINT_NAMES.append(f"openarm_right_joint{i + 1}")
        JOINT_NAMES.extend(NAMES_L_HAND)
        JOINT_NAMES.extend(NAMES_L_HAND_DISTAL)
        JOINT_NAMES.extend(NAMES_R_HAND)
        JOINT_NAMES.extend(NAMES_R_HAND_DISTAL)

        def make_joint_command_msg(
            stamp: dict, left: np.ndarray, right: np.ndarray
        ) -> dict:
            """Build a merged JointState message from the latest left/right arm arrays.

            The gripper portion comes from hand_target_l/hand_target_r (the
            current Revo2 hand targets, updated by trigger/joystick events)
            rather than left[7]/right[7] — the single scalar gripper value
            carried in the Dora position input isn't used here. Each hand's
            distal joints mirror hand_target[1:6] (its own proximal values).
            """
            positions = []
            for i in range(7):
                positions.append(left[i])
                positions.append(right[i])
            positions.extend(hand_target_l.tolist())
            positions.extend(hand_target_l[1:6].tolist())
            positions.extend(hand_target_r.tolist())
            positions.extend(hand_target_r[1:6].tolist())

            return {
                "header": {"stamp": stamp, "frame_id": ""},
                "name": JOINT_NAMES,
                "position": positions,
                "velocity": EMPTY_F64,
                "effort": EMPTY_F64,
            }

    else:  # real
        p_l_arm = node.create_publisher(
            node.create_topic(
                "/left_joint_trajectory_controller/joint_trajectory",
                "trajectory_msgs/JointTrajectory",
                qos_arm,
            )
        )
        p_r_arm = node.create_publisher(
            node.create_topic(
                "/right_joint_trajectory_controller/joint_trajectory",
                "trajectory_msgs/JointTrajectory",
                qos_arm,
            )
        )
        _log("mode=real: publishers ready: /left_joint_trajectory_controller/joint_trajectory, "
             "/right_joint_trajectory_controller/joint_trajectory")

        STAMP_ZERO = {"sec": np.int32(0), "nanosec": np.uint32(0)}
        NAMES_L_ARM = [f"openarm_left_joint{i + 1}" for i in range(7)]
        NAMES_R_ARM = [f"openarm_right_joint{i + 1}" for i in range(7)]

        MAX_STEP = 0.02
        SYNC_THRESHOLD = 0.1

        internal_target_l = np.zeros(7, dtype=np.float64)
        internal_target_r = np.zeros(7, dtype=np.float64)

        physical_state = {
            "left": np.zeros(7, dtype=np.float64),
            "right": np.zeros(7, dtype=np.float64),
            "left_ready": False,
            "right_ready": False,
        }
        threading.Thread(
            target=_spin_robot_state_subscriber,
            args=(physical_state,),
            daemon=True,
        ).start()
        _log("RobotStateSubscriber background thread started")

        def make_joint_msg(names: list, positions: list) -> dict:
            """Build a single-point JointTrajectory message for immediate execution."""
            return {
                "header": {"stamp": STAMP_ZERO, "frame_id": ""},
                "joint_names": names,
                "points": [
                    {
                        "positions": positions,
                        "velocities": EMPTY_F64,
                        "accelerations": EMPTY_F64,
                        "effort": EMPTY_F64,
                        "time_from_start": {"sec": np.int32(0), "nanosec": np.uint32(0)},
                    }
                ],
            }

        def get_safe_position(
            target: np.ndarray,
            internal_target: np.ndarray,
            physical_pos: np.ndarray,
            is_ready: bool,
        ) -> list:
            """Ramp internal_target toward target by at most MAX_STEP per call.

            Resyncs internal_target to the real physical position first if it
            has drifted more than SYNC_THRESHOLD away from it (e.g. the arm
            was moved independently of this node), so the ramp always starts
            from where the arm actually is rather than compounding on a stale
            internal estimate.
            """
            if not is_ready:
                return internal_target.tolist()

            max_error = np.max(np.abs(internal_target - physical_pos))
            if max_error > SYNC_THRESHOLD:
                internal_target[:] = physical_pos

            step = np.clip(target - internal_target, -MAX_STEP, MAX_STEP)
            internal_target += step
            return internal_target.tolist()

    # --- 5. Dora Loop ---
    dora_node = dora.Node()
    _log("event loop starting (waiting for Dora INPUT events)")

    # Latest known [7 arm joints, 1 gripper] per side. right_position always
    # drives publishing (see below); left_cache only updates in the
    # background from left_position events and starts out — and stays,
    # if left_position is never wired — at LEFT_FIXED.
    right_cache = np.zeros(8, dtype=np.float64)
    left_cache = LEFT_FIXED.copy()
    button_cache = np.zeros(4, dtype=np.int32)
    # Latest thumbstick axes [x, y] per controller, driving each hand's thumb.
    stick_l = np.zeros(2, dtype=np.float64)
    stick_r = np.zeros(2, dtype=np.float64)

    # Heartbeat: right_position is the tick that drives arm publishing, so
    # logging every LOG_EVERY of them (rather than every single one, which
    # would flood stdout at the ~200 Hz this normally runs at) makes it
    # possible to tell "no right_position events ever arrive" (no heartbeat
    # at all) apart from "events arrive but publishing raises" (heartbeat
    # never reached because the try/except below logs and stops first) apart
    # from "publishing normally, arm just isn't moving for some other reason"
    # (heartbeat keeps appearing with changing right_arm/safe_r values).
    LOG_EVERY = 200
    _stats = {"right_count": 0, "left_count": 0, "t0": time.time()}

    for event in dora_node:
        if event["type"] != "INPUT":
            continue

        eid = event["id"]
        value = event["value"]

        if eid in BUTTON_INDEX:
            idx = BUTTON_INDEX[eid]
            new_val = int(bool(value[0].as_py()))
            if new_val == button_cache[idx]:
                continue
            button_cache[idx] = new_val
            stamp = now_stamp()
            msg = make_joy_msg(stamp, button_cache)
            p_buttons.publish(pa.array([msg]))
            continue

        if eid in ("trigger_left", "trigger_right"):
            # High-rate grip stream: linearize the trigger, ramp each hand's 4
            # non-thumb fingers toward grip * closed-limit by at most
            # HAND_MAX_STEP. In --mode real both hands are (re)published on
            # every trigger event to their own controller topic; in --mode
            # sim hand_target is instead merged into /joint_command the next
            # time right_position publishes, so no separate publish here.
            raw = float(np.clip(value.to_numpy()[0], 0.0, 1.0))
            g = linearize_grip(raw) if GRIP_LINEARIZE else raw
            if eid == "trigger_left":
                desired_grip_l = g
            else:
                desired_grip_r = g

            des_l = desired_grip_l * HAND_CLOSED[2:]
            hand_target_l[2:] += np.clip(des_l - hand_target_l[2:], -HAND_MAX_STEP, HAND_MAX_STEP)
            des_r = desired_grip_r * HAND_CLOSED[2:]
            hand_target_r[2:] += np.clip(des_r - hand_target_r[2:], -HAND_MAX_STEP, HAND_MAX_STEP)
            if args.mode == "real":
                p_l_hand.publish(pa.array([make_hand_msg(NAMES_L_HAND, hand_target_l.tolist())]))
                p_r_hand.publish(pa.array([make_hand_msg(NAMES_R_HAND, hand_target_r.tolist())]))
            continue

        if eid in ("joystick_x_left", "joystick_y_left"):
            # Left thumbstick drives the LEFT thumb: x -> thumb_metacarpal (idx 0),
            # y -> thumb_proximal (idx 1). Axis mapped [0,1] * closed-limit (rest =
            # open); use (axis + 1) / 2 instead for full-range with center = mid.
            axis = float(np.clip(value.to_numpy()[0], -1.0, 1.0))
            stick_l[0 if eid == "joystick_x_left" else 1] = axis
            des = np.clip(stick_l, 0.0, 1.0) * HAND_CLOSED[0:2]
            hand_target_l[0:2] += np.clip(des - hand_target_l[0:2], -HAND_MAX_STEP, HAND_MAX_STEP)
            if args.mode == "real":
                p_l_hand.publish(pa.array([make_hand_msg(NAMES_L_HAND, hand_target_l.tolist())]))
            continue

        if eid in ("joystick_x_right", "joystick_y_right"):
            # Right thumbstick drives the RIGHT thumb, same mapping as the left.
            axis = float(np.clip(value.to_numpy()[0], -1.0, 1.0))
            stick_r[0 if eid == "joystick_x_right" else 1] = axis
            des = np.clip(stick_r, 0.0, 1.0) * HAND_CLOSED[0:2]
            hand_target_r[0:2] += np.clip(des - hand_target_r[0:2], -HAND_MAX_STEP, HAND_MAX_STEP)
            if args.mode == "real":
                p_r_hand.publish(pa.array([make_hand_msg(NAMES_R_HAND, hand_target_r.tolist())]))
            continue

        if eid == "left_position":
            _stats["left_count"] += 1
            vals = value.to_numpy().astype(np.float64)
            n = min(len(vals), 8)
            left_cache[:n] = vals[:n]
            continue

        if eid == "ee_pose_right":
            pose = extract_pose(value)
            p_ee_pose_right.publish(pa.array([make_ee_pose_msg(now_stamp(), pose)]))
            continue

        if eid == "ee_pose_left":
            pose = extract_pose(value)
            p_ee_pose_left.publish(pa.array([make_ee_pose_msg(now_stamp(), pose)]))
            continue

        if eid != "right_position":
            continue

        _stats["right_count"] += 1
        vals = value.to_numpy().astype(np.float64)
        n = min(len(vals), 8)
        right_cache[:n] = vals[:n]

        try:
            if args.mode == "sim":
                stamp = now_stamp()
                msg = make_joint_command_msg(stamp, left_cache, right_cache)
                p_joint_cmd.publish(pa.array([msg]))
            else:
                safe_r = get_safe_position(
                    right_cache[:7],
                    internal_target_r,
                    physical_state["right"],
                    physical_state["right_ready"],
                )
                p_r_arm.publish(pa.array([make_joint_msg(NAMES_R_ARM, safe_r)]))

                safe_l = get_safe_position(
                    left_cache[:7],
                    internal_target_l,
                    physical_state["left"],
                    physical_state["left_ready"],
                )
                p_l_arm.publish(pa.array([make_joint_msg(NAMES_L_ARM, safe_l)]))
        except Exception:
            _log(f"EXCEPTION publishing joint command on right_position "
                 f"#{_stats['right_count']}")
            traceback.print_exc()
            continue

        if _stats["right_count"] % LOG_EVERY == 0:
            now = time.time()
            dt = now - _stats["t0"]
            hz = LOG_EVERY / dt if dt > 0 else 0.0
            _stats["t0"] = now
            if args.mode == "sim":
                _log(f"right_position #{_stats['right_count']} "
                     f"left_position #{_stats['left_count']} hz={hz:.1f} "
                     f"/joint_command published, "
                     f"right_arm[:3]={np.round(right_cache[:3], 3).tolist()}")
            else:
                _log(f"right_position #{_stats['right_count']} "
                     f"left_position #{_stats['left_count']} hz={hz:.1f} "
                     f"published /right_..._trajectory + /left_..._trajectory "
                     f"(right_feedback_ready={physical_state['right_ready']}, "
                     f"left_feedback_ready={physical_state['left_ready']}), "
                     f"safe_r[:3]={[round(x, 3) for x in safe_r[:3]]}")


if __name__ == "__main__":
    main()
