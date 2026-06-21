#!/usr/bin/env python3

import sys
import threading
import time
import csv
import os

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory


class PlaybackDualArmHybrid(Node):
    def __init__(self):
        super().__init__("playback_dual_arm_hybrid")

        # ==========================================
        # DANH SÁCH FILE CSV VÀ CẤU HÌNH TAY
        # ==========================================
        self.TASKS = [
            {
                "csv_path": "40dg_new.csv",
                "close_pose": [1.0, 0.0, 0.4, 0.7, 0.9, 1.0],  # Pose nắm cho file 1
                "forward_actions": {
                    0: "open-before",  # Đảm bảo tay mở ra TRƯỚC khi rời khỏi Home
                    3: "close-before"  # Đóng tay TRƯỚC KHI chạy quỹ đạo index 2 (Pre-pick)
                },
                "reverse_actions": {
                    5: "open-after"    # Mở tay SAU KHI lùi xong quỹ đạo index 4 (Hạ đồ xuống)
                }
            },
            {
                "csv_path": "12cm.csv",
                "close_pose": [1.0, 0.0, 0.7, 0.6, 0.6, 0.7],  # Pose nắm cho file 2
                "forward_actions": {
                    2: "close-after"  # Đóng tay TRƯỚC KHI chạy quỹ đạo index 2
                },
                "reverse_actions": {
                    4: "open-after"    # Mở tay SAU KHI lùi xong quỹ đạo index 4
                }
            }
        ]
        # ==========================================

        self.cb_group = ReentrantCallbackGroup()

        # State Subscriber
        self.current_joint_state = None
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10, callback_group=self.cb_group)

        # Publishers for Hands
        self.right_hand_pub = self.create_publisher(JointTrajectory, "/right_revo2_hand_controller/joint_trajectory", 10)
        self.left_hand_pub = self.create_publisher(JointTrajectory, "/left_revo2_hand_controller/joint_trajectory", 10)

        # Publishers for Arms (Streaming)
        self.left_arm_pub = self.create_publisher(JointTrajectory, "/left_joint_trajectory_controller/joint_trajectory", 10)
        self.right_arm_pub = self.create_publisher(JointTrajectory, "/right_joint_trajectory_controller/joint_trajectory", 10)

        self.get_logger().info("=== DUAL ARM CSV PLAYBACK ENGINE STARTED ===")


    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    # ============================================================
    # HAND CONTROL
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
            self.get_logger().info("Closing hands ...")
            if custom_close_pose:
                point.positions = custom_close_pose
            else:
                point.positions = [1.0, 0.0, 0.4, 0.7, 0.9, 1.0]
        else:
            self.get_logger().info("Opening hands ...")
            point.positions = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        point.time_from_start.sec = 1
        msg_right.points.append(point)
        msg_left.points.append(point)

        self.right_hand_pub.publish(msg_right)
        self.left_hand_pub.publish(msg_left)

    # ============================================================
    # LOAD CSV
    # ============================================================
    def load_from_csv(self, filename):
        trajectories = []
        if not os.path.exists(filename):
            self.get_logger().error(f"File {filename} doesn't exist!")
            return None

        try:
            current_seg_idx = -1
            current_traj = None
            joint_names = [f"openarm_left_joint{i + 1}" for i in range(7)] + \
                          [f"openarm_right_joint{i + 1}" for i in range(7)]

            with open(filename, mode='r') as f:
                reader = csv.reader(f)
                next(reader)

                for row in reader:
                    if not row: continue
                    seg_idx = int(row[0])
                    positions = [float(x) for x in row[1:]]

                    if seg_idx != current_seg_idx:
                        if current_traj is not None:
                            trajectories.append(current_traj)
                        current_traj = RobotTrajectory()
                        current_traj.joint_trajectory.joint_names = joint_names
                        current_seg_idx = seg_idx

                    pt = JointTrajectoryPoint()
                    pt.positions = positions
                    current_traj.joint_trajectory.points.append(pt)

                if current_traj is not None:
                    trajectories.append(current_traj)

            self.get_logger().info(f"Loaded {len(trajectories)} segments from: {filename}")
            return trajectories
        except Exception as e:
            self.get_logger().error(f"Error when read CSV: {e}")
            return None

    # ============================================================
    # REVERSE MOTION UTILITY
    # ============================================================
    def reverse_trajectory(self, traj: RobotTrajectory):
        rev_traj = RobotTrajectory()
        rev_traj.joint_trajectory.header = traj.joint_trajectory.header
        rev_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names

        reversed_points = list(reversed(traj.joint_trajectory.points))
        for pt in reversed_points:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(pt.positions)
            rev_traj.joint_trajectory.points.append(new_pt)

        return rev_traj

    # ============================================================
    # STREAMING TOPIC
    # ============================================================
    def execute_by_streaming(self, dual_trajectory):
        if not dual_trajectory.joint_trajectory.points:
            return

        names = dual_trajectory.joint_trajectory.joint_names
        idx_l = [i for i, n in enumerate(names) if "left" in n]
        idx_r = [i for i, n in enumerate(names) if "right" in n]
        names_l = [names[i] for i in idx_l]
        names_r = [names[i] for i in idx_r]

        all_pts = [pt.positions for pt in dual_trajectory.joint_trajectory.points]
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

        self.get_logger().info(f"==> Streaming {len(stream_pts)} dual-points ...")

        for p in stream_pts:
            if not rclpy.ok():
                break

            msg_l = JointTrajectory()
            msg_l.joint_names = names_l
            pt_l = JointTrajectoryPoint()
            pt_l.positions = [p[i] for i in idx_l]
            pt_l.time_from_start.sec = 0; pt_l.time_from_start.nanosec = 0
            msg_l.points.append(pt_l)

            msg_r = JointTrajectory()
            msg_r.joint_names = names_r
            pt_r = JointTrajectoryPoint()
            pt_r.positions = [p[i] for i in idx_r]
            pt_r.time_from_start.sec = 0; pt_r.time_from_start.nanosec = 0
            msg_r.points.append(pt_r)

            self.left_arm_pub.publish(msg_l)
            self.right_arm_pub.publish(msg_r)
            time.sleep(0.02)

    # ============================================================
    # MAIN EXECUTION THREAD
    # ============================================================
    def execution_thread(self):
        # Đợi hệ thống có dữ liệu joint state
        while self.current_joint_state is None and rclpy.ok():
            time.sleep(0.1)

        total_tasks = len(self.TASKS)

        # Hàm tiện ích xử lý logic ĐÓNG/MỞ tay kết hợp TRƯỚC/SAU
        def trigger_hand_action(action_dict, index, current_timing, pose):
            if index in action_dict:
                cmd = action_dict[index].lower()
                parts = cmd.split('-')
                action_type = parts[0]  # "open" hoặc "close"
                timing = parts[1] if len(parts) > 1 else "after"  # Mặc định là "after"

                if timing == current_timing:
                    if action_type == "close":
                        self.control_hands(close=True, custom_close_pose=pose)
                    elif action_type == "open":
                        self.control_hands(close=False)
                    time.sleep(0.5)

        for task_idx, task in enumerate(self.TASKS):
            if not rclpy.ok(): break

            csv_path = task["csv_path"]
            close_pose = task.get("close_pose", [])
            fw_actions = task.get("forward_actions", {})
            rv_actions = task.get("reverse_actions", {})

            print("\n\033[96m" + "=" * 50)
            print(f" STARTING TASK {task_idx + 1}/{total_tasks}: {csv_path} ")
            print("=" * 50 + "\033[0m\n")

            # Mở tay ở đầu mỗi Task (Đảm bảo an toàn)
            self.control_hands(close=False)
            time.sleep(1.0)

            # 1. LOAD CSV
            saved_trajectories = self.load_from_csv(csv_path)
            if saved_trajectories is None:
                self.get_logger().error(f"Skipping task {csv_path} because file not found or invalid.")
                continue

            num_segments = len(saved_trajectories)

            # --- STAGE 1: FORWARD (TIẾN) ---
            self.get_logger().info(f"--- STAGE 1: FORWARD STREAMING ---")

            # Nếu là Task đầu tiên: chạy từ segment 0. Từ Task 2 trở đi: chạy từ segment 2.
            start_idx = 0 if task_idx == 0 else 2

            for i in range(start_idx, num_segments):
                if not rclpy.ok(): break
                self.get_logger().info(f"--- Processing segment {i} -> {i + 1} ---")

                # Kiểm tra action "before"
                trigger_hand_action(fw_actions, i, "before", close_pose)

                # Chạy quỹ đạo Arms
                dual_trajectory = saved_trajectories[i]
                self.execute_by_streaming(dual_trajectory)

                # Kiểm tra action "after"
                trigger_hand_action(fw_actions, i, "after", close_pose)

                # Nếu là segment cuối cùng của Lượt đi -> Chờ xác nhận
                if i == num_segments - 1:
                    print("\n\033[93m" + "=" * 50)
                    print(" Đã đi hết quỹ đạo! Nhấn Enter để bắt đầu quá trình lùi... ")
                    print("=" * 50 + "\033[0m\n")
                    input()

            # --- STAGE 2: REVERSE (LÙI) ---
            self.get_logger().info("--- STAGE 2: REVERSE STREAMING ---")

            is_last_task = (task_idx == total_tasks - 1)

            if is_last_task:
                print("\033[92m---> Task cuối cùng! Lùi toàn bộ về Home...\033[0m")
                stop_idx = -1
            else:
                print("\033[92m---> Lùi về vị trí Raise Hand (Chuẩn bị cho task tiếp theo)...\033[0m")
                stop_idx = 1

            # Gộp chung toàn bộ quy trình lùi vào 1 vòng lặp để xử lý hand action đồng bộ
            for idx in range(num_segments - 1, stop_idx, -1):
                if not rclpy.ok(): break
                print(f"\033[92m---> Reversing segment {idx}...\033[0m")

                # Kiểm tra action "before" (Trước khi lùi segment)
                trigger_hand_action(rv_actions, idx, "before", close_pose)

                rev_traj = self.reverse_trajectory(saved_trajectories[idx])
                self.execute_by_streaming(rev_traj)

                # Kiểm tra action "after" (Sau khi lùi segment)
                trigger_hand_action(rv_actions, idx, "after", close_pose)

            self.get_logger().info(f"--- TASK {task_idx + 1} FINISHED! ---")

            if not is_last_task:
                print("\n\033[93m" + "=" * 50)
                print(" Nhấn Enter để bắt đầu Load File tiếp theo... ")
                print("=" * 50 + "\033[0m\n")
                input()

        self.get_logger().info("--- ALL TASKS COMPLETED! ---")


def main():
    rclpy.init()
    node = PlaybackDualArmHybrid()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    exec_thread = threading.Thread(target=node.execution_thread, daemon=True)
    exec_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()