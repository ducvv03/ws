#!/usr/bin/env python3

import sys
import math
import threading
import time
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory
)
from moveit_msgs.srv import GetPositionIK, ApplyPlanningScene


class WorkspaceTester(Node):
    def __init__(self):
        super().__init__("workspace_tester")

        # ==========================================
        # CONFIGURATION TEST RANGE
        # ==========================================
        # Điều chỉnh dải tọa độ Tâm (A) cần test
        self.X_RANGE = [0.05, 0.15, 0.25, 0.35, 0.45]
        self.Y_RANGE = [-0.2, -0.1, 0.0, 0.1, 0.2]
        self.Z_RANGE = [0.3, 0.4, 0.5]
        self.A_RANGE = [-50.0, -25.0, 0.0, 25.0, 50.0]  # Góc xoay Z

        # Bật/tắt chạy thật.
        self.EXECUTE_MOTION = True
        self.RESULT_CSV = "full_flow_test_result.csv"
        # ==========================================

        self.cb_group = ReentrantCallbackGroup()

        # Clients
        self.move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self.cb_group)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik", callback_group=self.cb_group)

        # State & Publishers
        self.current_joint_state = None
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10, callback_group=self.cb_group)

        self.left_arm_pub = self.create_publisher(JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory",
                                                  10)
        self.right_arm_pub = self.create_publisher(JointTrajectory,
                                                   "/right_joint_trajectory_controller/joint_trajectory", 10)

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    def _wait_for_future(self, future):
        while rclpy.ok() and not future.done():
            time.sleep(0.01)
        return future.result() if future.done() else None

    # ============================================================
    # TẠO TỌA ĐỘ THEO CHU TRÌNH BẮT BUỘC
    # ============================================================
    def get_fixed_poses(self, pose_type):
        def create_pose(px, py, pz, x, y, z, w):
            p = Pose()
            p.position.x = px;
            p.position.y = py;
            p.position.z = pz
            p.orientation.x = x;
            p.orientation.y = y;
            p.orientation.z = z;
            p.orientation.w = w
            return p

        # 1. Điểm xuất phát: RAISE HAND
        if pose_type == "raise":
            p_left = create_pose(0.014604, 0.1535, 0.45998, 0.71, 0.0, 0.71, 0.0)
            p_right = create_pose(0.014604, -0.1535, 0.45998, 0.71, 0.0, 0.71, 0.0)
            return p_left, p_right

        # 2. Điểm kết thúc: BASE
        elif pose_type == "base":
            p_left = create_pose(0.30, 0.15, 0.50, 0.71, 0.0, 0.71, 0.0)
            p_right = create_pose(0.30, -0.15, 0.50, 0.71, 0.0, 0.71, 0.0)
            return p_left, p_right

    def get_dynamic_poses(self, xA, yA, zA, angle_deg):
        angle_rad = math.radians(angle_deg)
        dist_pick = 0.15  # Pick cách A 15cm
        dist_pre = 0.20  # Pre-pick cách A 20cm
        lift_height = 0.10  # Lift nâng lên 10cm

        q_base = (0.0, 0.70710678, 0.0, 0.70710678)

        def q_mult(q1, q2):
            w1, x1, y1, z1 = q1;
            w2, x2, y2, z2 = q2
            return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)

        q_rotZ = (math.cos(angle_rad / 2), 0.0, 0.0, math.sin(angle_rad / 2))
        qw, qx, qy, qz = q_mult(q_rotZ, q_base)

        dx = -math.sin(angle_rad)
        dy = math.cos(angle_rad)

        def create_pose(px, py, pz):
            p = Pose()
            p.position.x = px;
            p.position.y = py;
            p.position.z = pz
            p.orientation.x = qx;
            p.orientation.y = qy;
            p.orientation.z = qz;
            p.orientation.w = qw
            return p

        # PRE-PICK Poses
        pre_l = create_pose(xA + dist_pre * dx, yA + dist_pre * dy, zA)
        pre_r = create_pose(xA - dist_pre * dx, yA - dist_pre * dy, zA)

        # PICK Poses
        pick_l = create_pose(xA + dist_pick * dx, yA + dist_pick * dy, zA)
        pick_r = create_pose(xA - dist_pick * dx, yA - dist_pick * dy, zA)

        # LIFT Poses (Giống Pick nhưng Z cao hơn)
        lift_l = create_pose(xA + dist_pick * dx, yA + dist_pick * dy, zA + lift_height)
        lift_r = create_pose(xA - dist_pick * dx, yA - dist_pick * dy, zA + lift_height)

        return (pre_l, pre_r), (pick_l, pick_r), (lift_l, lift_r)

    # ============================================================
    # COMPUTE IK (Có ép Seed State)
    # ============================================================
    def compute_ik_sync(self, group_name, ik_link_name, target_pose, seed_joints=None):
        req = GetPositionIK.Request()
        req.ik_request.group_name = group_name
        req.ik_request.ik_link_name = ik_link_name
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True
        req.ik_request.robot_state.is_diff = True

        if self.current_joint_state is not None:
            req.ik_request.robot_state.joint_state = self.current_joint_state

        if seed_joints is not None:
            prefix = "openarm_left" if "left" in group_name else "openarm_right"
            req.ik_request.robot_state.joint_state.name = [f"{prefix}_joint{i + 1}" for i in range(7)]
            req.ik_request.robot_state.joint_state.position = seed_joints

        res = self._wait_for_future(self.ik_client.call_async(req))
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        prefix = "openarm_left" if "left" in group_name else "openarm_right"
        joints = [pos for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position)
                  if prefix in name and "finger" not in name]
        return joints

    # ============================================================
    # PLANNING (XUẤT PHÁT TỪ TRẠNG THÁI TÙY CHỈNH)
    # ============================================================
    def plan_segment_sync(self, end_l, end_r, start_l, start_r):
        goal = MoveGroup.Goal()
        goal.request.group_name = "both_arms"
        goal.request.allowed_planning_time = 2.0
        goal.planning_options.plan_only = True

        # ÉP BẮT BUỘC XUẤT PHÁT TỪ START STATE TRUYỀN VÀO
        goal.request.start_state.is_diff = True
        goal.request.start_state.joint_state.name = [f"openarm_left_joint{i + 1}" for i in range(7)] + [
            f"openarm_right_joint{i + 1}" for i in range(7)]
        goal.request.start_state.joint_state.position = start_l + start_r

        # Cài đặt điểm đến
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
    # UTILITIES CHO EXECUTION
    # ============================================================
    def reverse_trajectory(self, traj: RobotTrajectory):
        rev_traj = RobotTrajectory()
        rev_traj.joint_trajectory.header = traj.joint_trajectory.header
        rev_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names
        for pt in reversed(traj.joint_trajectory.points):
            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(pt.positions)
            rev_traj.joint_trajectory.points.append(new_pt)
        return rev_traj

    def execute_by_streaming(self, planned_trajectory):
        if not planned_trajectory.joint_trajectory.points:
            return

        names = planned_trajectory.joint_trajectory.joint_names
        idx_l = [i for i, n in enumerate(names) if "left" in n]
        idx_r = [i for i, n in enumerate(names) if "right" in n]
        names_l = [names[i] for i in idx_l]
        names_r = [names[i] for i in idx_r]

        all_pts = [pt.positions for pt in planned_trajectory.joint_trajectory.points]
        stream_pts = [all_pts[0]]
        STEP_RAD = 0.05

        for p in all_pts[1:]:
            while True:
                diff = [a - b for a, b in zip(p, stream_pts[-1])]
                max_diff = max(abs(d) for d in diff)
                if max_diff < STEP_RAD: break
                ratio = STEP_RAD / max_diff
                stream_pts.append([stream_pts[-1][i] + diff[i] * ratio for i in range(len(p))])
        stream_pts.append(all_pts[-1])

        for p in stream_pts:
            if not rclpy.ok(): break
            msg_l = JointTrajectory();
            msg_l.joint_names = names_l
            pt_l = JointTrajectoryPoint();
            pt_l.positions = [p[i] for i in idx_l]
            msg_l.points.append(pt_l)

            msg_r = JointTrajectory();
            msg_r.joint_names = names_r
            pt_r = JointTrajectoryPoint();
            pt_r.positions = [p[i] for i in idx_r]
            msg_r.points.append(pt_r)

            self.left_arm_pub.publish(msg_l)
            self.right_arm_pub.publish(msg_r)
            time.sleep(0.015)

    # ============================================================
    # MAIN EVALUATION THREAD
    # ============================================================
    def testing_thread(self):
        self.get_logger().info("Waiting MoveIt Service/Action ...")
        self.move_client.wait_for_server()
        self.ik_client.wait_for_service()

        while self.current_joint_state is None and rclpy.ok():
            time.sleep(0.1)

        total_points = len(self.X_RANGE) * len(self.Y_RANGE) * len(self.Z_RANGE) * len(self.A_RANGE)
        print("\n" + "=" * 80)
        print(f" KHỞI ĐỘNG CÔNG CỤ ĐÁNH GIÁ CHU TRÌNH FULL FLOW (WORKSPACE ANALYZER)")
        print(f" Flow: Raise -> Pre-pick -> Pick -> Lift -> Base")
        print(f" Tổng số tọa độ Tâm (A) cần kiểm tra: {total_points}")
        print("=" * 80 + "\n")

        with open(self.RESULT_CSV, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Index", "X", "Y", "Z", "Angle", "Error Stage", "Status"])

        count = 0
        success_count = 0

        # Lấy Poses cố định (Raise, Base)
        pose_raise_l, pose_raise_r = self.get_fixed_poses("raise")
        pose_base_l, pose_base_r = self.get_fixed_poses("base")

        for x in self.X_RANGE:
            for y in self.Y_RANGE:
                for z in self.Z_RANGE:
                    for a in self.A_RANGE:
                        if not rclpy.ok(): return
                        count += 1
                        log_msg = f"[{count}/{total_points}] X:{x:.2f}|Y:{y:.2f}|Z:{z:.2f}|Ang:{a:.1f}° -> "

                        # 1. Tính toán Poses động (Pre-pick, Pick, Lift)
                        (pose_pre_l, pose_pre_r), (pose_pick_l, pose_pick_r), (pose_lift_l,
                                                                               pose_lift_r) = self.get_dynamic_poses(x,
                                                                                                                     y,
                                                                                                                     z,
                                                                                                                     a)

                        sequence_poses = [
                            ("Raise", pose_raise_l, pose_raise_r),
                            ("Pre-pick", pose_pre_l, pose_pre_r),
                            ("Pick", pose_pick_l, pose_pick_r),
                            ("Lift", pose_lift_l, pose_lift_r),
                            ("Base", pose_base_l, pose_base_r)
                        ]

                        # 2. GIẢI IK THEO CHUỖI LIÊN TIẾP (Seed nối tiếp Seed)
                        ik_solutions = []
                        prev_ik_l, prev_ik_r = None, None
                        ik_failed = False

                        for name, pl, pr in sequence_poses:
                            ik_l = self.compute_ik_sync("left_arm", "openarm_left_tcp", pl, seed_joints=prev_ik_l)
                            ik_r = self.compute_ik_sync("right_arm", "openarm_right_tcp", pr, seed_joints=prev_ik_r)

                            if not ik_l or not ik_r:
                                print(log_msg + f"\033[91m[LỖI IK] Không với tới điểm {name}\033[0m")
                                self._log_result(count, x, y, z, a, f"IK {name}", "FAIL")
                                ik_failed = True
                                break

                            ik_solutions.append((ik_l, ik_r))
                            prev_ik_l, prev_ik_r = ik_l, ik_r

                        if ik_failed:
                            continue

                        # 3. PLANNING THEO CHUỖI LIÊN TIẾP
                        planned_paths = []
                        plan_failed = False
                        path_names = ["Raise->Pre", "Pre->Pick", "Pick->Lift", "Lift->Base"]

                        for step in range(4):
                            start_l, start_r = ik_solutions[step]
                            end_l, end_r = ik_solutions[step + 1]
                            path_name = path_names[step]

                            traj = self.plan_segment_sync(end_l, end_r, start_l, start_r)
                            if not traj:
                                print(log_msg + f"\033[93m[LỖI PLAN] Lỗi va chạm chặng {path_name}\033[0m")
                                self._log_result(count, x, y, z, a, f"Plan {path_name}", "FAIL")
                                plan_failed = True
                                break

                            planned_paths.append(traj)

                        if plan_failed:
                            continue

                        # 4. CHU TRÌNH THÀNH CÔNG HOÀN TOÀN
                        success_count += 1
                        print(log_msg + "\033[92m[THÀNH CÔNG] Full Flow OK!\033[0m")
                        self._log_result(count, x, y, z, a, "None", "SUCCESS")

                        # 5. THỰC THI (NẾU BẬT)
                        if self.EXECUTE_MOTION:
                            # Chạy tiến: Raise -> Pre -> Pick -> Lift -> Base
                            for p_traj in planned_paths:
                                self.execute_by_streaming(p_traj)
                                time.sleep(0.2)

                            # Chạy lùi an toàn: Base -> Lift -> Pick -> Pre -> Raise (để tay về lại trạng thái chuẩn)
                            for p_traj in reversed(planned_paths):
                                rev_traj = self.reverse_trajectory(p_traj)
                                self.execute_by_streaming(rev_traj)
                                time.sleep(0.1)

        print("\n" + "=" * 80)
        print(f" HOÀN TẤT ĐÁNH GIÁ CHU TRÌNH! (Thành công hoàn toàn: {success_count}/{total_points})")
        print(f" Kết quả chi tiết lưu tại: {self.RESULT_CSV}")
        print("=" * 80 + "\n")

    def _log_result(self, idx, x, y, z, a, err_stage, status):
        with open(self.RESULT_CSV, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([idx, x, y, z, a, err_stage, status])


def main():
    rclpy.init()
    node = WorkspaceTester()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    test_thread = threading.Thread(target=node.testing_thread, daemon=True)
    test_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()