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

All ROS2 publishing goes through a real rclpy node (see DoraRos2BridgeNode)
rather than Dora's own `dora.Ros2Context` bridge — the latter is built on the
Rust `ros2-client` crate, which has a documented, upstream-acknowledged issue
about its nodes/publishers not being reliably discoverable by a standard
rclcpp/rclpy ROS2 graph (see the `dora` Python package's own Ros2Node
docstring warning, and https://github.com/jhelovuo/ros2-client/issues/4).
Concretely: on real hardware, `dora.Ros2Context`-based publishers logged
"published" on every cycle, but neither `ros2 topic echo` nor the real
`joint_trajectory_controller` ever received anything, with ROS_DOMAIN_ID and
RMW confirmed matching on both ends — while rclpy (used here for the
feedback subscription) worked immediately. Only the Dora dataflow event loop
itself (`dora.Node()`, receiving right_position/trigger_*/button_*/etc. from
the ik/udp-receiver nodes) still uses the `dora` package — that's a separate,
unaffected mechanism from the ROS2 bridge.

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
from builtin_interfaces.msg import Duration, Time
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node as RclpyNode
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy, JointState
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _log(msg: str) -> None:
    """Print a stdout diagnostic line, prefixed like this repo's other Dora nodes' logs."""
    print(f"[dora-to-ros2] {msg}", flush=True)


def _stamp_now() -> Time:
    """Build a builtin_interfaces/Time from the current wall-clock time."""
    t = time.time()
    return Time(sec=int(t), nanosec=int((t % 1.0) * 1e9))


_STAMP_ZERO = Time(sec=0, nanosec=0)
_DURATION_ZERO = Duration(sec=0, nanosec=0)


class DoraRos2BridgeNode(RclpyNode):
    """Owns every ROS2 publisher this bridge uses, plus feedback subscriptions.

    (--mode real only) the physical joint-feedback subscriptions used for the
    safety ramp. A single real rclpy node, spun in a background thread, backs
    both directions — publishing is called from the main thread (the Dora
    event loop below), which is safe: rclpy publishers/subscriptions don't
    require the calling thread to be the one spinning the executor.
    """

    def __init__(self, mode: str) -> None:
        """Create all publishers for `mode`, plus feedback subscriptions if real."""
        super().__init__("dora_to_ros2")
        self._mode = mode

        qos_arm = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10
        )
        # Best-effort: the VR headset re-sends button/pose state on every tick
        # (up to ~500 Hz) whether or not it changed, and there's no guarantee
        # anything is subscribed. Reliable QoS made the writer block trying to
        # redeliver that firehose whenever nothing was keeping up, eventually
        # erroring out with a publish timeout.
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
        )

        self.p_buttons = self.create_publisher(Joy, "/vr_buttons", qos_best_effort)
        self.p_ee_pose_right = self.create_publisher(PoseStamped, "/right_ee_pose", qos_best_effort)
        self.p_ee_pose_left = self.create_publisher(PoseStamped, "/left_ee_pose", qos_best_effort)
        _log("common publishers ready: /vr_buttons, /right_ee_pose, /left_ee_pose")

        # Revo2 hand (gripper) command publishers, used in both --mode sim and
        # --mode real (see main loop for which mode actually calls .publish on
        # these vs. merging into /joint_command).
        self.p_l_hand = self.create_publisher(
            JointTrajectory, "/left_revo2_hand_controller/joint_trajectory", qos_arm
        )
        self.p_r_hand = self.create_publisher(
            JointTrajectory, "/right_revo2_hand_controller/joint_trajectory", qos_arm
        )
        _log("hand publishers ready: /left_revo2_hand_controller/joint_trajectory, "
             "/right_revo2_hand_controller/joint_trajectory")

        self.physical_state = None
        if mode == "sim":
            self.p_joint_cmd = self.create_publisher(JointState, "/joint_command", qos_arm)
            _log("mode=sim: publisher ready: /joint_command")
        else:
            self.p_l_arm = self.create_publisher(
                JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory", qos_arm
            )
            self.p_r_arm = self.create_publisher(
                JointTrajectory, "/right_joint_trajectory_controller/joint_trajectory", qos_arm
            )
            _log("mode=real: publishers ready: /left_joint_trajectory_controller/joint_trajectory, "
                 "/right_joint_trajectory_controller/joint_trajectory")

            self.physical_state = {
                "left": np.zeros(7, dtype=np.float64),
                "right": np.zeros(7, dtype=np.float64),
                "left_ready": False,
                "right_ready": False,
            }
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

            # Grip-hold "safety ramp" flags, set by the VR receiver (client)
            # via these SetBool services. Each request just latches its value
            # into a plain bool the publish loop reads; served by this node's
            # background spin thread. Default False → publish the cache
            # straight through until the receiver says a grip window is open.
            self.grip_timer_right = False
            self.grip_timer_left = False
            self.grip_engaged_right = False
            self.grip_engaged_left = False
            self.create_service(
                SetBool, "/dora_bridge/set_grip_timer_right", self._srv_grip_right
            )
            self.create_service(
                SetBool, "/dora_bridge/set_grip_timer_left", self._srv_grip_left
            )
            self.create_service(
                SetBool, "/dora_bridge/set_grip_engaged_right", self._srv_engaged_right
            )
            self.create_service(
                SetBool, "/dora_bridge/set_grip_engaged_left", self._srv_engaged_left
            )

    def _left_cb(self, msg: JointTrajectoryControllerState) -> None:
        if not self.physical_state["left_ready"]:
            _log("first LEFT controller_state feedback received "
                 "(/left_joint_trajectory_controller/controller_state)")
        self.physical_state["left"][:] = msg.feedback.positions[:7]
        self.physical_state["left_ready"] = True

    def _right_cb(self, msg: JointTrajectoryControllerState) -> None:
        if not self.physical_state["right_ready"]:
            _log("first RIGHT controller_state feedback received "
                 "(/right_joint_trajectory_controller/controller_state)")
        self.physical_state["right"][:] = msg.feedback.positions[:7]
        self.physical_state["right_ready"] = True

    def _srv_grip_right(self, request: SetBool.Request, response: SetBool.Response):
        self.grip_timer_right = bool(request.data)
        response.success = True
        return response

    def _srv_grip_left(self, request: SetBool.Request, response: SetBool.Response):
        self.grip_timer_left = bool(request.data)
        response.success = True
        return response

    def _srv_engaged_right(self, request: SetBool.Request, response: SetBool.Response):
        self.grip_engaged_right = bool(request.data)
        response.success = True
        return response

    def _srv_engaged_left(self, request: SetBool.Request, response: SetBool.Response):
        self.grip_engaged_left = bool(request.data)
        response.success = True
        return response


def _spin_ros_node(node: DoraRos2BridgeNode) -> None:
    try:
        rclpy.spin(node)
    except Exception:
        _log("FATAL: ROS2 spin thread crashed")
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
        rclpy.init()
        ros_node = DoraRos2BridgeNode(args.mode)
    except Exception:
        _log("FATAL: failed to create rclpy node")
        traceback.print_exc()
        raise
    _log("ROS2 node created (dora_to_ros2)")

    threading.Thread(target=_spin_ros_node, args=(ros_node,), daemon=True).start()
    _log("ROS2 spin thread started")

    # --- 2. Pre-defined Constants ---
    # Default left-arm target (all joints 0, neutral gripper) used until/unless
    # a left_position event arrives. Yamls that don't wire left_position
    # (single-arm teleop) leave the left arm parked here forever; bimanual
    # yamls overwrite it in place as soon as the IK solver starts publishing
    # position_left.
    LEFT_FIXED = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    # buttons[0..3] = a, b, x, y — index into BUTTON_CACHE below.
    BUTTON_INDEX = {"button_a": 0, "button_b": 1, "button_x": 2, "button_y": 3}

    # --- 3. Helpers ---
    def make_joy_msg(stamp: Time, buttons: np.ndarray) -> Joy:
        """Build a sensor_msgs/Joy message from the latest button states."""
        msg = Joy()
        msg.header.stamp = stamp
        msg.header.frame_id = ""
        msg.axes = []
        msg.buttons = [int(b) for b in buttons]
        return msg

    # --- Revo2 hand (gripper) setup -------------------------------------------
    # The VR trigger drives the 4 non-thumb fingers of each Revo2 hand; the thumb
    # (indices 0,1) holds a fixed mock pose until a real thumb input is wired.
    # Published as trajectory_msgs/JointTrajectory to each hand's
    # <side>_revo2_hand_controller, in both --mode sim and --mode real.
    HAND_FINGERS = ["thumb_metacarpal", "thumb_proximal", "index_proximal",
                    "middle_proximal", "ring_proximal", "pinky_proximal"]
    NAMES_L_HAND = [f"left_{f}_joint" for f in HAND_FINGERS]
    NAMES_R_HAND = [f"right_{f}_joint" for f in HAND_FINGERS]

    # Finger closed limits (rad), same order as HAND_FINGERS (revo2 URDF upper
    # limits); every finger opens at 0.0.
    HAND_CLOSED = np.array([1.57, 1.03, 1.1, 1.1, 1.1, 1.1], dtype=np.float64)
    # Initial thumb pose (indices 0,1), overwritten as soon as the thumbstick moves.
    MOCK_THUMB = np.array([1.57, 0.0], dtype=np.float64)

    # Persistent full 6-joint target per hand: the trigger updates the 4-finger
    # slice [2:], the thumbstick updates the thumb slice [0:2], and we always
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

    def make_hand_msg(names: list, positions: list) -> JointTrajectory:
        """Build a single-point JointTrajectory message for the Revo2 hand."""
        msg = JointTrajectory()
        msg.header.stamp = _STAMP_ZERO
        msg.header.frame_id = ""
        msg.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = _DURATION_ZERO
        msg.points = [point]
        return msg

    def extract_pose(value: pa.Array) -> np.ndarray:
        """Read an fk-node pose event: a {"pose": [...]} struct, or a flat array."""
        if pa.types.is_struct(value.type):
            value = value.field("pose")[0].values
        return np.array(value, dtype=np.float32)

    def make_ee_pose_msg(stamp: Time, pose: np.ndarray) -> PoseStamped:
        """Build a geometry_msgs/PoseStamped from an FK [px,py,pz,qw,qx,qy,qz,...] pose.

        Only the first 7 values are used; a trailing gripper value (as the fk
        node emits) is ignored here — that's already published separately.
        """
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = ""
        msg.pose.position.x = float(pose[0])
        msg.pose.position.y = float(pose[1])
        msg.pose.position.z = float(pose[2])
        msg.pose.orientation.w = float(pose[3])
        msg.pose.orientation.x = float(pose[4])
        msg.pose.orientation.y = float(pose[5])
        msg.pose.orientation.z = float(pose[6])
        return msg

    if args.mode == "sim":
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
            stamp: Time, left: np.ndarray, right: np.ndarray
        ) -> JointState:
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

            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = ""
            msg.name = JOINT_NAMES
            msg.position = [float(p) for p in positions]
            msg.velocity = []
            msg.effort = []
            return msg

    else:  # real
        NAMES_L_ARM = [f"openarm_left_joint{i + 1}" for i in range(7)]
        NAMES_R_ARM = [f"openarm_right_joint{i + 1}" for i in range(7)]

        MAX_STEP = 0.02
        SYNC_THRESHOLD = 0.1

        internal_target_l = np.zeros(7, dtype=np.float64)
        internal_target_r = np.zeros(7, dtype=np.float64)

        # Latched freeze target per arm: when the grip is released, the arm is
        # pinned at the physical position captured at that instant (None while
        # gripping). See the grip_engaged branch in the publish handlers.
        hold_l = None
        hold_r = None

        def make_joint_msg(names: list, positions: list) -> JointTrajectory:
            """Build a single-point JointTrajectory message for immediate execution."""
            msg = JointTrajectory()
            msg.header.stamp = _STAMP_ZERO
            msg.header.frame_id = ""
            msg.joint_names = names
            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in positions]
            point.velocities = []
            point.accelerations = []
            point.effort = []
            point.time_from_start = _DURATION_ZERO
            msg.points = [point]
            return msg

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

    # --- 4. Dora Loop ---
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
            stamp = _stamp_now()
            msg = make_joy_msg(stamp, button_cache)
            ros_node.p_buttons.publish(msg)
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
                ros_node.p_l_hand.publish(make_hand_msg(NAMES_L_HAND, hand_target_l.tolist()))
                ros_node.p_r_hand.publish(make_hand_msg(NAMES_R_HAND, hand_target_r.tolist()))
            continue

        if eid in ("joystick_x_left", "joystick_y_left"):
            # Left thumbstick drives the LEFT thumb: x -> thumb_metacarpal (idx 0),
            # y -> thumb_proximal (idx 1). Axis mapped [0,1] * closed-limit (rest =
            # open); use (axis + 1) / 2 instead for full-range with center = mid.
            axis = float(np.clip(value.to_numpy()[0], -1.0, 1.0))
            stick_l[0 if eid == "joystick_x_left" else 1] = axis if eid == "joystick_x_left" else -axis
            des = np.clip(stick_l, 0.0, 1.0) * HAND_CLOSED[0:2]
            hand_target_l[0:2] += np.clip(des - hand_target_l[0:2], -HAND_MAX_STEP, HAND_MAX_STEP)
            if args.mode == "real":
                ros_node.p_l_hand.publish(make_hand_msg(NAMES_L_HAND, hand_target_l.tolist()))
            continue

        if eid in ("joystick_x_right", "joystick_y_right"):
            # Right thumbstick drives the RIGHT thumb, same mapping as the left.
            axis = float(np.clip(-value.to_numpy()[0], -1.0, 1.0))
            stick_r[0 if eid == "joystick_x_right" else 1] = axis
            des = np.clip(stick_r, 0.0, 1.0) * HAND_CLOSED[0:2]
            hand_target_r[0:2] += np.clip(des - hand_target_r[0:2], -HAND_MAX_STEP, HAND_MAX_STEP)
            if args.mode == "real":
                ros_node.p_r_hand.publish(make_hand_msg(NAMES_R_HAND, hand_target_r.tolist()))
            continue

        if eid == "left_position":
            _stats["left_count"] += 1
            vals = value.to_numpy().astype(np.float64)
            n = min(len(vals), 8)
            left_cache[:n] = vals[:n]
            # real mode: the left arm publishes on its OWN event now (not piggy-
            # backed on right_position), so it tracks at its native rate. sim
            # still merges left into /joint_command on right_position (below).
            if args.mode == "real":
                try:
                    if not ros_node.grip_engaged_left:
                        # grip released → freeze at the physical position.
                        if hold_l is None:
                            if ros_node.physical_state["left_ready"]:
                                hold_l = np.array(
                                    ros_node.physical_state["left"], dtype=np.float64
                                )
                            else:
                                hold_l = np.array(left_cache[:7], dtype=np.float64)
                            internal_target_l[:] = hold_l
                        safe_l = hold_l.tolist()
                    elif ros_node.grip_timer_left:
                        hold_l = None
                        safe_l = get_safe_position(
                            left_cache[:7],
                            internal_target_l,
                            ros_node.physical_state["left"],
                            ros_node.physical_state["left_ready"],
                        )
                    else:
                        hold_l = None
                        safe_l = left_cache[:7].tolist()
                    ros_node.p_l_arm.publish(make_joint_msg(NAMES_L_ARM, safe_l))
                except Exception:
                    _log(f"EXCEPTION publishing LEFT joint command on "
                         f"left_position #{_stats['left_count']}")
                    traceback.print_exc()
            continue

        if eid == "ee_pose_right":
            pose = extract_pose(value)
            ros_node.p_ee_pose_right.publish(make_ee_pose_msg(_stamp_now(), pose))
            continue

        if eid == "ee_pose_left":
            pose = extract_pose(value)
            ros_node.p_ee_pose_left.publish(make_ee_pose_msg(_stamp_now(), pose))
            continue

        if eid != "right_position":
            continue

        _stats["right_count"] += 1
        vals = value.to_numpy().astype(np.float64)
        n = min(len(vals), 8)
        right_cache[:n] = vals[:n]

        try:
            if args.mode == "sim":
                stamp = _stamp_now()
                msg = make_joint_command_msg(stamp, left_cache, right_cache)
                ros_node.p_joint_cmd.publish(msg)
            else:
                # Grip window open (flag set by the VR receiver's service
                # call) → gentle ramp via get_safe_position; otherwise publish
                # the commanded cache straight through for full tracking. RIGHT
                # arm only — the left arm publishes on its own left_position
                # event now (see the left_position handler above).
                if not ros_node.grip_engaged_right:
                    # grip released → freeze: latch the physical position at
                    # release and hold it, so the arm stops right where it is
                    # instead of finishing the ramp toward the last target.
                    if hold_r is None:
                        if ros_node.physical_state["right_ready"]:
                            hold_r = np.array(
                                ros_node.physical_state["right"], dtype=np.float64
                            )
                        else:
                            hold_r = np.array(right_cache[:7], dtype=np.float64)
                        internal_target_r[:] = hold_r
                    safe_r = hold_r.tolist()
                elif ros_node.grip_timer_right:
                    hold_r = None
                    safe_r = get_safe_position(
                        right_cache[:7],
                        internal_target_r,
                        ros_node.physical_state["right"],
                        ros_node.physical_state["right_ready"],
                    )
                else:
                    hold_r = None
                    safe_r = right_cache[:7].tolist()
                ros_node.p_r_arm.publish(make_joint_msg(NAMES_R_ARM, safe_r))
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
                     f"published /right_..._trajectory (left on its own event) "
                     f"(right_feedback_ready={ros_node.physical_state['right_ready']}, "
                     f"left_feedback_ready={ros_node.physical_state['left_ready']}), "
                     f"safe_r[:3]={[round(x, 3) for x in safe_r[:3]]}")


if __name__ == "__main__":
    main()
