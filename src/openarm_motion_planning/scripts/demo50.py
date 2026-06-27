#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
import threading, time, csv, os, yaml
import numpy as np
import copy

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, CollisionObject
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene

from pnk_perception_msgs.msg import GraspTarget
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent.parent
YAML_PATH = PACKAGE_DIR / "config" / "config.yaml"


class MultiPickController(Node):
    def __init__(self):
        super().__init__("multi_pick_controller")
        self.cb_group = ReentrantCallbackGroup()

        with open(YAML_PATH, 'r') as file:
            self.config = yaml.safe_load(file)

        self.HOME_JOINTS = [0.0] * 7
        self.STEP_RAD = 0.02
        self.STREAM_INTERVAL = 0.02

        self.trajectory_history = []
        self.home_to_raise_history = []

        self.current_joint_state = None

        self.robot_state = "INIT"
        self.is_shutting_down = False
        self.is_moving = False

        self.collected_targets = {}
        self.is_collecting_targets = False

        # Clients
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb_group)
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene",
                                               callback_group=self.cb_group)

        # Subscribers
        self.create_subscription(JointState, "/joint_states", self.js_callback, 10, callback_group=self.cb_group)
        self.create_subscription(GraspTarget, "/grasp_target_info", self.target_callback, 10,
                                 callback_group=self.cb_group)

        # Publishers (Arms)
        self.left_arm_pub = self.create_publisher(JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory",
                                                  10)
        self.right_arm_pub = self.create_publisher(JointTrajectory,
                                                   "/right_joint_trajectory_controller/joint_trajectory", 10)

        # Publishers (Hands)
        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_revo2_hand_controller/joint_trajectory",
                                                    10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_revo2_hand_controller/joint_trajectory", 10)

        self.get_logger().info("Multi Pick Flow Controller Initialized.")

    def js_callback(self, msg):
        self.current_joint_state = msg

    def target_callback(self, msg):
        if getattr(self, 'is_collecting_targets', False):
            self.collected_targets[msg.tag_id] = msg

    def wait_for_future(self, future):
        while rclpy.ok() and not future.done():
            if self.is_shutting_down: return None
            time.sleep(0.01)
        return future.result() if future.done() else None

    # ============================================================
    # HAND CONTROL LOGIC
    # ============================================================
    def control_hands(self, close=True, custom_close_pose=None):
        msg_right = JointTrajectory()
        msg_left = JointTrajectory()

        msg_right.joint_names = [f"right_{f}_proximal_joint" for f in ["thumb", "index", "middle", "ring", "pinky"]]
        msg_right.joint_names.insert(1, "right_thumb_metacarpal_joint")

        msg_left.joint_names = [f"left_{f}_proximal_joint" for f in ["thumb", "index", "middle", "ring", "pinky"]]
        msg_left.joint_names.insert(1, "left_thumb_metacarpal_joint")

        point = JointTrajectoryPoint()
        if close:
            self.get_logger().info(f"Closing hands with pose: {custom_close_pose}")
            if custom_close_pose:
                point.positions = custom_close_pose
            else:
                point.positions = [1.0, 0.0, 0.4, 0.7, 0.9, 1.0]  # Mặc định
        else:
            self.get_logger().info("Opening hands ...")
            point.positions = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Pose mở cố định

        point.time_from_start.sec = 1
        msg_right.points.append(point)
        msg_left.points.append(point)

        self.right_hand_pub.publish(msg_right)
        self.left_hand_pub.publish(msg_left)

    # ---------- REVERSE & EXECUTE LOGIC ----------
    def stream_points(self, stream_pts, sleep_time=None, is_recovery=False, is_recording=False):
        if sleep_time is None:
            sleep_time = self.STREAM_INTERVAL

        self.is_moving = True
        executed_count = 0

        try:
            for p in stream_pts:
                if not rclpy.ok(): break

                # Cắt mảng (Crop) chống teleport khi bị ngắt bởi Ctrl+C
                if self.is_shutting_down and not is_recovery:
                    if is_recording and len(self.trajectory_history) > 0:
                        self.trajectory_history[-1] = stream_pts[:executed_count]
                    break

                self.left_arm_pub.publish(JointTrajectory(joint_names=[f"openarm_left_joint{k + 1}" for k in range(7)],
                                                          points=[JointTrajectoryPoint(positions=p[0:7])]))
                self.right_arm_pub.publish(
                    JointTrajectory(joint_names=[f"openarm_right_joint{k + 1}" for k in range(7)],
                                    points=[JointTrajectoryPoint(positions=p[7:14])]))

                executed_count += 1
                time.sleep(sleep_time)
        finally:
            if not is_recovery:
                self.is_moving = False

    def recover_by_reversing(self):
        self.get_logger().warn("=== THỰC HIỆN XẢ QUỸ ĐẠO LÙI LẠI ===")
        while len(self.trajectory_history) > 0:
            self.stream_points(self.trajectory_history.pop()[::-1], sleep_time=0.04, is_recovery=True)
            time.sleep(0.2)

    def get_interpolated_joints(self, trajectory, target_time):
        pts = trajectory.joint_trajectory.points
        if not pts: return None
        if target_time <= 0: return list(pts[0].positions)
        if target_time >= (pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9):
            return list(pts[-1].positions)
        for i in range(len(pts) - 1):
            t1 = pts[i].time_from_start.sec + pts[i].time_from_start.nanosec * 1e-9
            t2 = pts[i + 1].time_from_start.sec + pts[i + 1].time_from_start.nanosec * 1e-9
            if t1 <= target_time <= t2:
                alpha = (target_time - t1) / (t2 - t1)
                p1, p2 = np.array(pts[i].positions), np.array(pts[i + 1].positions)
                return (p1 + alpha * (p2 - p1)).tolist()
        return list(pts[-1].positions)

    def execute_dual_streaming_smooth(self, left_traj, right_traj, record=True):
        if left_traj is None or right_traj is None:
            return False

        dur_l = left_traj.joint_trajectory.points[-1].time_from_start.sec + \
                left_traj.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9
        dur_r = right_traj.joint_trajectory.points[-1].time_from_start.sec + \
                right_traj.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9

        max_dur = max(dur_l, dur_r)
        if max_dur == 0: return False

        scale_l = max_dur / dur_l if dur_l > 0 else 1.0
        scale_r = max_dur / dur_r if dur_r > 0 else 1.0

        raw_pts = []
        sample_time = 0.0
        while sample_time <= max_dur:
            pos_l = self.get_interpolated_joints(left_traj, sample_time / scale_l)
            pos_r = self.get_interpolated_joints(right_traj, sample_time / scale_r)
            raw_pts.append(pos_l + pos_r)
            sample_time += self.STREAM_INTERVAL

        stream_pts = []
        last_p = raw_pts[0]
        stream_pts.append(last_p)

        for p in raw_pts[1:]:
            while True:
                diff = [a - b for a, b in zip(p, last_p)]
                max_diff = max(abs(d) for d in diff)
                if max_diff < self.STEP_RAD: break
                ratio = self.STEP_RAD / max_diff
                interp_p = [last_p[i] + diff[i] * ratio for i in range(len(p))]
                stream_pts.append(interp_p)
                last_p = interp_p
        stream_pts.append(raw_pts[-1])

        if record:
            self.trajectory_history.append(stream_pts)

        self.stream_points(stream_pts, is_recording=record)
        return True

    def compute_ik(self, target_pose, arm_side):
        req = GetPositionIK.Request()
        req.ik_request.group_name = f"{arm_side}_arm"
        req.ik_request.ik_link_name = f"openarm_{arm_side}_tcp"
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True
        if self.current_joint_state: req.ik_request.robot_state.joint_state = self.current_joint_state

        future = self.ik_client.call_async(req)
        res = self.wait_for_future(future)
        if res and res.error_code.val == MoveItErrorCodes.SUCCESS:
            names = [f"openarm_{arm_side}_joint{i + 1}" for i in range(7)]
            return [pos for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position) if
                    name in names]
        return None

    def plan_arm_trajectory(self, target_joints, arm_side):
        if target_joints is None: return None

        goal = MoveGroup.Goal()
        goal.request.group_name = f"{arm_side}_arm"
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.planning_options.plan_only = True
        if self.current_joint_state: goal.request.start_state.joint_state = self.current_joint_state

        constraints = Constraints()
        for i, pos in enumerate(target_joints):
            constraints.joint_constraints.append(
                JointConstraint(joint_name=f"openarm_{arm_side}_joint{i + 1}", position=pos, tolerance_above=0.01,
                                tolerance_below=0.01, weight=1.0))
        goal.request.goal_constraints.append(constraints)

        goal_handle = self.wait_for_future(self.move_client.send_goal_async(goal))
        if not goal_handle or not goal_handle.accepted: return None

        res_wrapper = self.wait_for_future(goal_handle.get_result_async())
        if res_wrapper and res_wrapper.result.error_code.val == MoveItErrorCodes.SUCCESS:
            return res_wrapper.result.planned_trajectory
        return None

    def load_grouped_csv_trajectory(self, file_path):
        full_path = PACKAGE_DIR / "config" / file_path
        groups = {}
        if not full_path.exists():
            return groups
        try:
            with open(full_path, mode='r') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    if len(row) < 15: continue
                    seg_idx = int(row[0])
                    pts = [float(x) for x in row[1:15]]
                    if seg_idx not in groups:
                        groups[seg_idx] = []
                    groups[seg_idx].append(pts)
            return groups
        except Exception as e:
            return {}

    # ---------- COLLISION OBSTACLE DYNAMIC ----------
    def load_all_obstacles(self):
        if not self.collected_targets: return
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        for tag_id, target in self.collected_targets.items():
            cfg = self.config['id_10'] if tag_id == 10 else self.config['id_other']
            target_pose_copy = copy.deepcopy(target.center.pose)
            if tag_id != 10:
                q = cfg['obstacle_quat']
                target_pose_copy.orientation = Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
                target_pose_copy.position.x += cfg['obstacle_x_offset']
            target_pose_copy.position.z += cfg['obstacle_z_offset']

            co = CollisionObject()
            co.id = f"obstacle_tag_{tag_id}"
            co.header.frame_id = "openarm_body_link0"
            co.operation = CollisionObject.ADD
            if cfg['obstacle_type'] == "BOX":
                co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=cfg['obstacle_dim']))
            else:
                co.primitives.append(SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=cfg['obstacle_dim']))
            co.primitive_poses.append(target_pose_copy)
            req.scene.world.collision_objects.append(co)
        self.wait_for_future(self.scene_client.call_async(req))

    def remove_obstacle(self, tag_id):
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        co = CollisionObject()
        co.id = f"obstacle_tag_{tag_id}"
        co.header.frame_id = "openarm_body_link0"
        co.operation = CollisionObject.REMOVE
        req.scene.world.collision_objects.append(co)
        self.wait_for_future(self.scene_client.call_async(req))

    def remove_all_obstacles(self):
        if not self.collected_targets: return
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        for tag_id in self.collected_targets.keys():
            co = CollisionObject()
            co.id = f"obstacle_tag_{tag_id}"
            co.header.frame_id = "openarm_body_link0"
            co.operation = CollisionObject.REMOVE
            req.scene.world.collision_objects.append(co)
        self.wait_for_future(self.scene_client.call_async(req))

    # ============================================================
    # GRACEFUL SHUTDOWN (CHẠY KHI NHẤN CTRL+C)
    # ============================================================
    def graceful_shutdown(self):
        self.is_shutting_down = True

        self.get_logger().warn("\n" + "!" * 50)
        self.get_logger().warn(" PHÁT HIỆN LỆNH DỪNG (CTRL+C). KÍCH HOẠT QUY TRÌNH AN TOÀN! ")
        self.get_logger().warn("!" * 50)

        wait_time = 0.0
        while getattr(self, 'is_moving', False) and wait_time < 15.0:
            time.sleep(0.1)
            wait_time += 0.1

        if self.robot_state == "INIT" or self.robot_state == "HOME":
            self.get_logger().info("Robot đang ở vị trí an toàn, tắt tiến trình.")
            return

        if len(self.trajectory_history) > 0:
            self.get_logger().info("Đang xả quỹ đạo đưa robot lùi về vị trí RAISE...")
            self.recover_by_reversing()
            self.get_logger().info("=== RÚT LUI THÀNH CÔNG VỀ VỊ TRÍ RAISE ===")

        elif self.robot_state == "REVERSING_CSV":
            self.get_logger().info("=== ROBOT ĐÃ HOÀN TẤT VIỆC LÙI CSV VỀ VỊ TRÍ RAISE ===")

        else:
            self.get_logger().info("Robot đã ở vị trí an toàn, không cần lùi thêm.")

    # ---------- MAIN SEQUENCE ----------
    def run_sequence(self):
        self.get_logger().info("Waiting for MoveIt Services...")
        self.move_client.wait_for_server()
        self.ik_client.wait_for_service()
        self.scene_client.wait_for_service()

        self.get_logger().info("Waiting for Joint States...")
        while self.current_joint_state is None and rclpy.ok():
            if self.is_shutting_down: return
            time.sleep(0.1)
        time.sleep(1.0)

        # ----------------------------------------------------
        # LUÔN MỞ TAY KHI BẮT ĐẦU VỚI POSE MỞ CỐ ĐỊNH
        # ----------------------------------------------------
        self.control_hands(close=False)
        time.sleep(1.0)

        # 0. HOME
        self.get_logger().info("Moving to HOME...")
        self.execute_dual_streaming_smooth(self.plan_arm_trajectory(self.HOME_JOINTS, "left"),
                                           self.plan_arm_trajectory(self.HOME_JOINTS, "right"), record=False)
        self.robot_state = "HOME"
        self.trajectory_history.clear()

        # 1. RAISE
        self.get_logger().info("Moving to RAISE...")
        qx, qy, qz, qw = 0.70710678, 0.0, 0.70710678, 0.0
        p_l = Pose(position=tf2_geometry_msgs.Point(x=0.014, y=0.15, z=0.50),
                   orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))
        p_r = Pose(position=tf2_geometry_msgs.Point(x=0.014, y=-0.15, z=0.50),
                   orientation=Quaternion(x=qx, y=qy, z=qz, w=qw))

        if not self.execute_dual_streaming_smooth(self.plan_arm_trajectory(self.compute_ik(p_l, "left"), "left"),
                                                  self.plan_arm_trajectory(self.compute_ik(p_r, "right"), "right"),
                                                  record=True): return

        self.home_to_raise_history = copy.deepcopy(self.trajectory_history)
        self.robot_state = "RAISE"

        last_csv_file = None
        self.failed_tags = set()

        # VÒNG LẶP GẮP CÁC VẬT TRÊN BÀN
        while rclpy.ok() and not self.is_shutting_down:
            print("\n\033[93m" + "=" * 50)
            print(" Đang ở vị trí RAISE. Nhấn Enter để bắt đầu quét các vật trên bàn... ")
            print("=" * 50 + "\033[0m\n")
            try:
                input()
            except EOFError:
                break

            if self.is_shutting_down: break

            self.get_logger().info("Scanning table for targets (1 seconds)...")
            self.collected_targets.clear()
            self.is_collecting_targets = True
            time.sleep(1.0)
            self.is_collecting_targets = False

            valid_targets = [t for t in self.collected_targets.values() if t.tag_id not in self.failed_tags]
            if not valid_targets:
                self.get_logger().info("Không tìm thấy vật thể hợp lệ nào trên bàn! Dừng vòng lặp quét.")
                break

            valid_targets.sort(key=lambda t: 1 if t.tag_id == 10 else 0)
            target = valid_targets[0]
            tag_id = target.tag_id

            cfg = self.config['id_10'] if tag_id == 10 else self.config['id_other']
            last_csv_file = cfg['csv_file']

            # Đọc tham số hand control từ YAML (với giá trị dự phòng nếu config bị thiếu)
            close_pose = cfg.get('close_pose', [1.0, 0.0, 0.4, 0.7, 0.9, 1.0])
            open_after_seg = cfg.get('open_after_reverse_segment', 3)

            self.get_logger().info(f"Phát hiện {len(valid_targets)} vật. Đã ưu tiên chọn gắp vật ID: {tag_id}")
            self.load_all_obstacles()

            self.trajectory_history.clear()

            # ========================================================
            # LOGIC ĐÓNG TAY TRƯỚC (RECTANGLE)
            # ========================================================
            if tag_id == 10:
                self.get_logger().info("Rectangle object detected: Closing hand BEFORE Pre-Grasp...")
                self.control_hands(close=True, custom_close_pose=close_pose)
                time.sleep(1.0)  # Chờ ngón tay co lại

            # PRE-GRASP
            self.get_logger().info("Planning Pre-Grasp...")
            if not self.execute_dual_streaming_smooth(
                    self.plan_arm_trajectory(self.compute_ik(target.pre_left.pose, "left"), "left"),
                    self.plan_arm_trajectory(self.compute_ik(target.pre_right.pose, "right"), "right"),
                    record=True):
                self.failed_tags.add(tag_id)
                self.recover_by_reversing()
                self.remove_all_obstacles()
                continue

            if self.is_shutting_down: break

            print("\n\033[93m" + "=" * 50)
            print(" Đã đến PRE-GRASP! Nhấn Enter để tiếp tục... ")
            print("=" * 50 + "\033[0m\n")
            try:
                input()
            except EOFError:
                break

            # GRASP
            self.get_logger().info("Planning Grasp...")
            if not self.execute_dual_streaming_smooth(
                    self.plan_arm_trajectory(self.compute_ik(target.left.pose, "left"), "left"),
                    self.plan_arm_trajectory(self.compute_ik(target.right.pose, "right"), "right"),
                    record=True):
                self.failed_tags.add(tag_id)
                self.recover_by_reversing()
                self.remove_all_obstacles()
                continue

            if self.is_shutting_down: break

            # ========================================================
            # LOGIC ĐÓNG TAY SAU (CYLINDER)
            # ========================================================
            if tag_id != 10:
                self.get_logger().info("Cylinder object detected: Closing hand AFTER Grasp...")
                self.control_hands(close=True, custom_close_pose=close_pose)
                time.sleep(1.0)

            print("\n\033[93m" + "=" * 50)
            print(" Đã gắp (GRASP)! Nhấn Enter để Lift... ")
            print("=" * 50 + "\033[0m\n")
            try:
                input()
            except EOFError:
                break

            self.remove_obstacle(tag_id)

            # LIFT
            self.get_logger().info("Planning Lift...")
            if not self.execute_dual_streaming_smooth(
                    self.plan_arm_trajectory(cfg['lift_joints_left'], "left"),
                    self.plan_arm_trajectory(cfg['lift_joints_right'], "right"),
                    record=True):
                self.failed_tags.add(tag_id)
                self.recover_by_reversing()
                self.remove_all_obstacles()
                continue

            if self.is_shutting_down: break

            print("\n\033[93m" + "=" * 50)
            print(" Đã Lift xong! Nhấn Enter để lùi về RAISE (sẽ tự động xóa các vật cản còn lại)... ")
            print("=" * 50 + "\033[0m\n")
            try:
                input()
            except EOFError:
                break

            self.remove_all_obstacles()
            self.trajectory_history.clear()
            self.robot_state = "REVERSING_CSV"

            # ========================================================
            # REVERSE CSV VỀ RAISE & LOGIC MỞ TAY
            # ========================================================
            self.get_logger().info("Reversing CSV back to RAISE...")
            csv_groups = self.load_grouped_csv_trajectory(cfg['csv_file'])

            sorted_segments = sorted(csv_groups.keys(), reverse=True)
            for seg in sorted_segments:
                if seg < 2: continue
                self.get_logger().info(f"Reversing segment {seg}...")
                points_to_stream = csv_groups[seg][::-1]
                self.stream_points(points_to_stream, sleep_time=0.05, is_recovery=True)

                # NẾU ĐÃ LÙI XONG SEGMENT QUY ĐỊNH, TIẾN HÀNH MỞ TAY
                if seg == open_after_seg:
                    self.get_logger().info(f"Object released! Opening hands after segment {seg}...")
                    self.control_hands(close=False)
                    time.sleep(1.0)  # Chờ ngón tay nhả ra hoàn toàn

            self.robot_state = "RAISE"
            self.get_logger().info(f"Hoàn thành chu trình vật ID: {tag_id}. Robot đang trở lại vị trí RAISE.")

        # ========================================================
        # 4. TRỞ VỀ HOME (Chỉ áp dụng khi kết thúc tự nhiên)
        # ========================================================
        if not self.is_shutting_down:
            self.get_logger().info("\n---> ĐÃ KẾT THÚC VÒNG LẶP GẮP. TRỞ VỀ HOME...")
            if self.home_to_raise_history:
                self.get_logger().info("Đang lùi từ RAISE về HOME bằng quỹ đạo thực tế...")
                self.trajectory_history = copy.deepcopy(self.home_to_raise_history)
                self.recover_by_reversing()
            else:
                self.execute_dual_streaming_smooth(self.plan_arm_trajectory(self.HOME_JOINTS, "left"),
                                                   self.plan_arm_trajectory(self.HOME_JOINTS, "right"), record=False)

            self.robot_state = "HOME"
            self.get_logger().info("=== CHU TRÌNH ĐÃ KẾT THÚC TOÀN BỘ ===")


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    node = MultiPickController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    main_thread = threading.Thread(target=node.run_sequence, daemon=True)
    main_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.graceful_shutdown()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__": main()