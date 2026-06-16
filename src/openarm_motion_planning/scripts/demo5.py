#!/usr/bin/env python3

import sys
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, RobotState,
    AttachedCollisionObject, CollisionObject
)
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene


class MoveDualArmHybrid(Node):
    def __init__(self):
        super().__init__("move_dual_arm_hybrid")

        self.cb_group = ReentrantCallbackGroup()

        # 1. Clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb_group)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene",
                                               callback_group=self.cb_group)

        # 2. State & Publishers
        self.current_joint_state = None
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10, callback_group=self.cb_group)

        # Topic for hands
        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_hand_controller/joint_trajectory", 10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_hand_controller/joint_trajectory", 10)

        # Topic Streaming for arms
        self.left_arm_pub = self.create_publisher(JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory",
                                                  10)
        self.right_arm_pub = self.create_publisher(JointTrajectory,
                                                   "/right_joint_trajectory_controller/joint_trajectory", 10)

        # 3. Targets
        self.left_targets, self.right_targets = self.define_targets()

    # ============================================================
    # PICK & PLACE
    # ============================================================
    def define_targets(self):
        def create_pose(x, y, z, qx, qy, qz, qw):
            p = Pose()
            p.position.x = x;
            p.position.y = y;
            p.position.z = z
            p.orientation.x = qx;
            p.orientation.y = qy;
            p.orientation.z = qz;
            p.orientation.w = qw
            return p

        left = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 0. Home
            create_pose(0.27, 0.10, 0.40, 0.71, 0.0, 0.71, 0.0),  # 1. Pre-pick
            (create_pose(0.27, 0.05, 0.40, 0.71, 0.0, 0.71, 0.0), "HOLD"),  # 2. Pick
            # create_pose(0.30, 0.10, 0.50, 0.71, 0.0, 0.71, 0.0),  # 3. Pick up
            create_pose(0.33, 0.20, 0.50, 0.71, 0.0, 0.71, 0.0),  # 4. Move
            (create_pose(0.25, 0.20, 0.40, 0.71, 0.0, 0.71, 0.0), "DROP"),  # 5. Place
            create_pose(0.25, 0.25, 0.40, 0.71, 0.0, 0.71, 0.0),  # 6. Post-place
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 7. Home
        ]

        right = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            create_pose(0.27, -0.20, 0.40, 0.71, 0.0, 0.71, 0.0),
            (create_pose(0.27, -0.15, 0.40, 0.71, 0.0, 0.71, 0.0), "HOLD"),
            # create_pose(0.30, -0.10, 0.50, 0.71, 0.0, 0.71, 0.0),
            create_pose(0.33, -0.0, 0.50, 0.71, 0.0, 0.71, 0.0),
            (create_pose(0.25, -0.0, 0.40, 0.71, 0.0, 0.71, 0.0), "DROP"),
            create_pose(0.25, -0.05, 0.40, 0.71, 0.0, 0.71, 0.0),
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ]
        return left, right

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def parse_target(self, target):
        if isinstance(target, tuple) and len(target) == 2:
            return target[0], target[1]
        return target, None

    def control_hands(self, close=True):
        msg_right = JointTrajectory()
        msg_left = JointTrajectory()

        msg_right.joint_names = [f"right_{f}_proximal_joint" for f in ["thumb", "index", "middle", "ring", "pinky"]]
        msg_right.joint_names.insert(1, "right_thumb_metacarpal_joint")

        msg_left.joint_names = [f"left_{f}_proximal_joint" for f in ["thumb", "index", "middle", "ring", "pinky"]]
        msg_left.joint_names.insert(1, "left_thumb_metacarpal_joint")

        point = JointTrajectoryPoint()
        if close:
            self.get_logger().info("Closing hands ...")
            point.positions = [0.0, 0.0, 0.5, 0.5, 0.5, 0.5]
        else:
            self.get_logger().info("Opening hands ...")
            point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        point.time_from_start.sec = 1
        msg_right.points.append(point)
        msg_left.points.append(point)

        self.right_hand_pub.publish(msg_right)
        self.left_hand_pub.publish(msg_left)

    def _wait_for_future(self, future):
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        return future.result() if future.done() else None

    # ============================================================
    # ATTACH OBJECT
    # ============================================================
    def manage_attached_object_sync(self, attach=True):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True

        if self.current_joint_state is not None:
            req.scene.robot_state.joint_state = self.current_joint_state

        aco = AttachedCollisionObject()
        aco.link_name = "openarm_left_tcp"
        aco.object.id = "carried_box"
        aco.object.header.frame_id = "openarm_left_tcp"

        co_world = CollisionObject()
        co_world.id = "carried_box"
        co_world.operation = CollisionObject.REMOVE

        if attach:
            self.get_logger().info("Attach object into Planning Scene...")
            aco.object.operation = CollisionObject.ADD

            box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.2, 0.1])
            box_pose = Pose()
            box_pose.position.y = 0.1
            box_pose.position.z = 0.1
            box_pose.orientation.w = 1.0

            aco.object.primitives.append(box)
            aco.object.primitive_poses.append(box_pose)

            aco.touch_links = [
                "openarm_body_link0",
                "openarm_left_hand", "openarm_right_hand",
                "openarm_left_tcp", "openarm_right_tcp",
                "openarm_left_link7", "openarm_right_link7",
                "openarm_left_link6", "openarm_right_link6",
                "openarm_left_link5", "openarm_right_link5",
                "openarm_left_left_finger", "openarm_left_right_finger",
                "openarm_right_left_finger", "openarm_right_right_finger",
                "left_base_link", "left_index_proximal_link", "left_middle_proximal_link", "left_pinky_distal_link",
                "left_pinky_proximal_link", "left_ring_proximal_link", "left_thumb_metacarpal_link",
                "left_thumb_proximal_link",
                "right_base_link", "right_index_proximal_link", "right_middle_proximal_link",
                "right_pinky_proximal_link",
                "right_ring_proximal_link", "right_thumb_metacarpal_link", "right_thumb_proximal_link",
                "right_index_distal_link", "right_pinky_distal_link", "right_pinky_touch_link",
                "right_thumb_proximal_link"
            ]
            req.scene.robot_state.attached_collision_objects.append(aco)
        else:
            self.get_logger().info("Remove object from Planning Scene...")
            aco.object.operation = CollisionObject.REMOVE
            req.scene.robot_state.attached_collision_objects.append(aco)
            req.scene.world.collision_objects.append(co_world)

        res = self._wait_for_future(self.scene_client.call_async(req))
        return res and res.success

    # ============================================================
    # COMPUTE IK
    # ============================================================
    def compute_ik_sync(self, group_name, ik_link_name, target_pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.ik_link_name = ik_link_name
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state.is_diff = True

        if self.current_joint_state is not None:
            req.ik_request.robot_state.joint_state = self.current_joint_state

        res = self._wait_for_future(self.ik_client.call_async(req))
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        prefix = "openarm_left" if "left" in group_name else "openarm_right"
        joints = [pos for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position)
                  if prefix in name and "finger" not in name]
        return joints

    # ========
    # PLANNING
    # ========
    def plan_segment_sync(self, end_l, end_r):
        goal = MoveGroup.Goal()
        goal.request.group_name = "both_arms"
        goal.request.allowed_planning_time = 5.0
        goal.planning_options.plan_only = True

        c = Constraints()
        for i, pos in enumerate(end_l):
            c.joint_constraints.append(
                JointConstraint(joint_name=f"openarm_left_joint{i + 1}", position=pos, tolerance_above=0.01,
                                tolerance_below=0.01, weight=1.0))
        for i, pos in enumerate(end_r):
            c.joint_constraints.append(
                JointConstraint(joint_name=f"openarm_right_joint{i + 1}", position=pos, tolerance_above=0.01,
                                tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints.append(c)

        goal_handle = self._wait_for_future(self.move_client.send_goal_async(goal))
        if not goal_handle or not goal_handle.accepted:
            return None

        result_wrapper = self._wait_for_future(goal_handle.get_result_async())
        if result_wrapper is None: return None

        result = result_wrapper.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        return result.planned_trajectory

    # ============================================================
    # TRANSFORM TRAJECTORY AND SEND TOPIC
    # ============================================================
    def execute_by_streaming(self, planned_trajectory):
        if not planned_trajectory.joint_trajectory.points:
            return

        names = planned_trajectory.joint_trajectory.joint_names
        idx_l = [i for i, n in enumerate(names) if "left" in n]
        idx_r = [i for i, n in enumerate(names) if "right" in n]
        names_l = [names[i] for i in idx_l]
        names_r = [names[i] for i in idx_r]

        all_pts = [pt.positions for pt in planned_trajectory.joint_trajectory.points]
        stream_pts = []
        last_p = all_pts[0]
        stream_pts.append(last_p)


        STEP_RAD = 0.02

        for p in all_pts[1:]:
            while True:
                diff = [a - b for a, b in zip(p, last_p)]
                max_diff = max(abs(d) for d in diff)
                if max_diff < STEP_RAD:
                    break

                ratio = STEP_RAD / max_diff
                interp_p = [last_p[i] + diff[i] * ratio for i in range(len(p))]
                stream_pts.append(interp_p)
                last_p = interp_p

        stream_pts.append(all_pts[-1])

        self.get_logger().info(f"==> Streaming {len(stream_pts)} points ...")

        for p in stream_pts:
            if not rclpy.ok():
                break

            msg_l = JointTrajectory()
            msg_l.joint_names = names_l
            pt_l = JointTrajectoryPoint()
            pt_l.positions = [p[i] for i in idx_l]
            pt_l.time_from_start.sec = 0
            pt_l.time_from_start.nanosec = 0
            msg_l.points.append(pt_l)

            msg_r = JointTrajectory()
            msg_r.joint_names = names_r
            pt_r = JointTrajectoryPoint()
            pt_r.positions = [p[i] for i in idx_r]
            pt_r.time_from_start.sec = 0
            pt_r.time_from_start.nanosec = 0
            msg_r.points.append(pt_r)

            self.left_arm_pub.publish(msg_l)
            self.right_arm_pub.publish(msg_r)

            time.sleep(0.02)

    # ============================================================
    # BACKGROUND THREAD
    # ============================================================
    def planning_thread(self):
        self.get_logger().info("Waiting MoveIt Service/Action ...")
        self.move_client.wait_for_server()
        self.ik_client.wait_for_service()
        self.scene_client.wait_for_service()

        while self.current_joint_state is None and rclpy.ok():
            time.sleep(0.1)

        self.get_logger().info("--- CLEANING SCENE ---")
        self.manage_attached_object_sync(attach=False)
        time.sleep(1.0)

        self.get_logger().info("--- STARTING PICK & PLACE (STREAMING HYBRID) ---")
        is_holding = False

        for i in range(len(self.left_targets)):
            if not rclpy.ok(): break
            self.get_logger().info(f"\n--- Planning {i + 1}/{len(self.left_targets)} ---")

            # 1. Phân tách Tọa độ & Tag
            t_left, tag_l = self.parse_target(self.left_targets[i])
            t_right, tag_r = self.parse_target(self.right_targets[i])
            tag = tag_l or tag_r

            # 2. Compute IK
            end_l = t_left if isinstance(t_left, list) else self.compute_ik_sync("left_arm", "openarm_left_tcp", t_left)
            end_r = t_right if isinstance(t_right, list) else self.compute_ik_sync("right_arm", "openarm_right_tcp",
                                                                                   t_right)

            if end_l is None or end_r is None:
                self.get_logger().error(f"Error IK at {i + 1}.")
                return

            # 3. Compute trajectory
            trajectory = self.plan_segment_sync(end_l, end_r)
            if not trajectory:
                self.get_logger().error(f"Error trajectory at {i + 1}.")
                return

            # 4. Send topic
            self.execute_by_streaming(trajectory)

            # 5. Logic open-close hand
            if tag == "HOLD" and not is_holding:
                self.control_hands(close=True)
                time.sleep(0.25)
                self.manage_attached_object_sync(attach=True)
                is_holding = True

            elif tag == "DROP" and is_holding:
                self.manage_attached_object_sync(attach=False)
                self.control_hands(close=False)
                time.sleep(0.25)
                is_holding = False

        self.get_logger().info("--- FINISH! ---")


def main():
    rclpy.init()
    node = MoveDualArmHybrid()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    planning_thread = threading.Thread(target=node.planning_thread, daemon=True)
    planning_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()