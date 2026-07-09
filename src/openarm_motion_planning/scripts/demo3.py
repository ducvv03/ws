#!/usr/bin/env python3

import sys
import math
import threading
import time
import copy

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MoveItErrorCodes, RobotState,
    AttachedCollisionObject, CollisionObject, RobotTrajectory
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

        # 3. Targets (Bây giờ CHỈ CẦN LƯỢT ĐI)
        self.left_targets = self.define_targets()

        # 4. Cờ dừng: bật lên khi Ctrl+C để các vòng streaming đang chạy thoát ra,
        #    còn action lúc dừng (nhả vật + về Home) vẫn được gửi đi.
        self._stop_event = threading.Event()
        self._shutdown_done = False
        # Tư thế Home an toàn để đưa 2 tay về khi dừng (7 joint / mỗi tay)
        self.home_positions = [0.0] * 7

        # 5. Thời gian GIỮ vật trước khi tự lùi lại (thay cho press Enter).
        self.hold_delay_sec = 3.0

        # 6. Điều khiển TỪ XA (chạy trên máy khác cùng ROS_DOMAIN_ID):
        #    - publish /demo3/next  -> bỏ chờ, đi tiếp ngay
        #    - publish /demo3/stop  -> gửi action dừng an toàn (nhả vật + về Home)
        self._next_trigger = threading.Event()
        self.next_sub = self.create_subscription(
            Empty, "/demo3/next", self._on_next_cmd, 10, callback_group=self.cb_group)
        self.stop_sub = self.create_subscription(
            Empty, "/demo3/stop", self._on_stop_cmd, 10, callback_group=self.cb_group)

    # ============================================================
    # TỌA ĐỘ CHỈ CẦN LƯỢT ĐI CHO TAY TRÁI
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

        qx, qy, qz, qw = 0.71, 0.0, 0.71, 0.0

        left_forward = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 0. Home
            create_pose(0.30, 0.125, 0.40, qx, qy, qz, qw),  # 1. Pre-pick
            (create_pose(0.30, 0.075, 0.40, qx, qy, qz, qw), "HOLD"),  # 2. Pick
            (create_pose(0.30, 0.075, 0.50, qx, qy, qz, qw), "WAIT_ENTER"),  # 3. Hold-home (Lift) & Chờ Enter
        ]
        return left_forward

    def joint_state_callback(self, msg):
        self.current_joint_state = msg

    # ── Lệnh điều khiển từ xa qua topic (từ máy khác) ──
    def _on_next_cmd(self, msg):
        self.get_logger().info("Nhận /demo3/next -> bỏ chờ, đi tiếp ngay.")
        self._next_trigger.set()

    def _on_stop_cmd(self, msg):
        self.get_logger().warn("Nhận /demo3/stop -> gửi action dừng an toàn.")
        self.send_shutdown_actions()

    def wait_hold(self, seconds):
        """Giữ vật: chờ 'seconds' giây HOẶC tới khi có /demo3/next.
        Ctrl+C hoặc /demo3/stop (bật _stop_event) sẽ cắt ngay."""
        self._next_trigger.clear()
        t0 = time.time()
        while rclpy.ok() and not self._stop_event.is_set():
            if self._next_trigger.is_set():
                self.get_logger().info("Có /demo3/next -> đi tiếp.")
                return
            if time.time() - t0 >= seconds:
                self.get_logger().info(f"Hết {seconds:.1f}s giữ vật -> tự lùi lại.")
                return
            time.sleep(0.05)

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
    # COMPUTE IK (Dành riêng cho tay trái)
    # ============================================================
    def compute_ik_left_sync(self, target_pose):
        req = GetPositionIK.Request()
        req.ik_request.group_name = "left_arm"
        req.ik_request.ik_link_name = "openarm_left_tcp"
        req.ik_request.pose_stamped.header.frame_id = "openarm_body_link0"
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.avoid_collisions = True

        if self.current_joint_state is not None:
            req.ik_request.robot_state.joint_state = self.current_joint_state

        res = self._wait_for_future(self.ik_client.call_async(req))
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            return None

        joints = [pos for name, pos in zip(res.solution.joint_state.name, res.solution.joint_state.position)
                  if "openarm_left" in name and "finger" not in name]
        return joints

    # ========
    # PLANNING CHỈ TAY TRÁI
    # ========
    def plan_left_segment_sync(self, end_l):
        goal = MoveGroup.Goal()
        goal.request.group_name = "left_arm"
        goal.request.allowed_planning_time = 5.0
        goal.planning_options.plan_only = True

        c = Constraints()
        for i, pos in enumerate(end_l):
            c.joint_constraints.append(
                JointConstraint(joint_name=f"openarm_left_joint{i + 1}", position=pos, tolerance_above=0.01,
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
    # HÀM NHÂN BẢN: TẠO GƯƠNG CHO TAY PHẢI
    # ============================================================
    def mirror_left_trajectory_to_dual(self, left_traj):
        dual_traj = RobotTrajectory()
        dual_traj.joint_trajectory.joint_names = [f"openarm_left_joint{i + 1}" for i in range(7)] + \
                                                 [f"openarm_right_joint{i + 1}" for i in range(7)]

        for pt in left_traj.joint_trajectory.points:
            left_pos = pt.positions
            if len(left_pos) != 7: continue

            # Góc gương tay phải
            right_pos = [-left_pos[0], -left_pos[1], -left_pos[2], left_pos[3], -left_pos[4], -left_pos[5],
                         -left_pos[6]]

            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(left_pos) + right_pos
            dual_traj.joint_trajectory.points.append(new_pt)

        return dual_traj

    # ============================================================
    # HÀM MỚI: LẬT NGƯỢC QUỸ ĐẠO ĐỂ ĐI LÙI
    # ============================================================
    def reverse_trajectory(self, traj: RobotTrajectory):
        rev_traj = RobotTrajectory()
        rev_traj.joint_trajectory.header = traj.joint_trajectory.header
        rev_traj.joint_trajectory.joint_names = traj.joint_trajectory.joint_names

        # Đảo ngược toàn bộ mảng points
        reversed_points = list(reversed(traj.joint_trajectory.points))

        for pt in reversed_points:
            new_pt = JointTrajectoryPoint()
            new_pt.positions = list(pt.positions)
            rev_traj.joint_trajectory.points.append(new_pt)

        return rev_traj

    # ============================================================
    # TRANSFORM TRAJECTORY AND SEND TOPIC (Streaming)
    # ============================================================
    def execute_by_streaming(self, dual_trajectory, ignore_stop=False):
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
            # Đang có yêu cầu dừng -> cắt ngang chuyển động bình thường.
            # (action lúc dừng gọi với ignore_stop=True nên vẫn chạy tới cùng)
            if self._stop_event.is_set() and not ignore_stop:
                self.get_logger().warn("Streaming bị cắt do có yêu cầu DỪNG.")
                break

            msg_l = JointTrajectory()
            msg_l.joint_names = names_l
            pt_l = JointTrajectoryPoint()
            pt_l.positions = [p[i] for i in idx_l]
            pt_l.time_from_start.sec = 0;
            pt_l.time_from_start.nanosec = 0
            msg_l.points.append(pt_l)

            msg_r = JointTrajectory()
            msg_r.joint_names = names_r
            pt_r = JointTrajectoryPoint()
            pt_r.positions = [p[i] for i in idx_r]
            pt_r.time_from_start.sec = 0;
            pt_r.time_from_start.nanosec = 0
            msg_r.points.append(pt_r)

            self.left_arm_pub.publish(msg_l)
            self.right_arm_pub.publish(msg_r)

            time.sleep(0.02)

    # ============================================================
    # ĐỌC VỊ TRÍ 2 TAY HIỆN TẠI (từ /joint_states)
    # ============================================================
    def get_current_arm_positions(self):
        if self.current_joint_state is None:
            return None, None
        name_to_pos = dict(zip(self.current_joint_state.name, self.current_joint_state.position))
        left = [name_to_pos.get(f"openarm_left_joint{i + 1}") for i in range(7)]
        right = [name_to_pos.get(f"openarm_right_joint{i + 1}") for i in range(7)]
        if any(v is None for v in left + right):
            return None, None
        return left, right

    def build_dual_traj(self, left_start, right_start, left_end, right_end):
        """Quỹ đạo 2 điểm (hiện tại -> đích) cho cả 2 tay; execute_by_streaming
        sẽ nội suy mượt ở giữa."""
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = [f"openarm_left_joint{i + 1}" for i in range(7)] + \
                                            [f"openarm_right_joint{i + 1}" for i in range(7)]
        for combo in (list(left_start) + list(right_start), list(left_end) + list(right_end)):
            pt = JointTrajectoryPoint()
            pt.positions = combo
            traj.joint_trajectory.points.append(pt)
        return traj

    # ============================================================
    # ACTION LÚC DỪNG: chỉ chạy KHI ĐANG DỪNG LẠI (Ctrl+C)
    # -> nhả vật + đưa 2 tay về Home an toàn rồi mới thoát.
    # ============================================================
    def send_shutdown_actions(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        # Bật cờ để cắt mọi chuyển động bình thường đang chạy ở planning_thread
        self._stop_event.set()
        self.get_logger().warn("=== NHẬN TÍN HIỆU DỪNG -> GỬI CÁC ACTION AN TOÀN ===")

        # 1. Nhả vật (mở tay)
        try:
            self.control_hands(close=False)
            time.sleep(0.5)
        except Exception as e:
            self.get_logger().error(f"Lỗi mở tay lúc dừng: {e}")

        # 2. Đưa 2 tay về Home (mượt). ignore_stop=True để không bị chính cờ dừng cắt.
        left_cur, right_cur = self.get_current_arm_positions()
        if left_cur is None:
            self.get_logger().warn("Chưa có /joint_states -> bỏ qua bước về Home.")
        else:
            self.get_logger().info("Đưa 2 tay về Home...")
            dual = self.build_dual_traj(left_cur, right_cur, self.home_positions, self.home_positions)
            try:
                self.execute_by_streaming(dual, ignore_stop=True)
            except Exception as e:
                self.get_logger().error(f"Lỗi khi về Home lúc dừng: {e}")

        self.get_logger().info("=== ĐÃ GỬI XONG ACTION DỪNG AN TOÀN ===")

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

        self.control_hands(close=False)
        time.sleep(1.0)

        self.get_logger().info("--- GIAI ĐOẠN 1: LƯỢT ĐI (TÍNH TOÁN & THỰC THI) ---")

        # Mảng lưu lại các quỹ đạo đã chạy để tí nữa lật ngược đi về
        saved_trajectories = []

        for i in range(len(self.left_targets)):
            if not rclpy.ok() or self._stop_event.is_set(): break
            self.get_logger().info(f"\n--- Planning segment {i} -> {i + 1} ---")

            t_left, tag = self.parse_target(self.left_targets[i])

            end_l = t_left if isinstance(t_left, list) else self.compute_ik_left_sync(t_left)
            if end_l is None:
                self.get_logger().error(f"Error IK Left Arm at step {i}.")
                return

            left_trajectory = self.plan_left_segment_sync(end_l)
            if not left_trajectory:
                self.get_logger().error(f"Error Planning Left Arm at step {i}.")
                return

            # Gộp thành 2 tay bằng gương
            dual_trajectory = self.mirror_left_trajectory_to_dual(left_trajectory)

            # LƯU VÀO LỊCH SỬ ĐỂ ĐI LÙI
            saved_trajectories.append(dual_trajectory)

            # Chạy
            self.execute_by_streaming(dual_trajectory)

            # Logic kẹp/dừng ở lượt đi
            if tag == "HOLD":
                self.control_hands(close=True)
                time.sleep(1.0)

            elif tag == "WAIT_ENTER":
                print("\n\033[93m" + "=" * 50)
                print(f" ĐÃ NHẤC VẬT LÊN CAO. CHỜ {self.hold_delay_sec:.0f}s RỒI TỰ LÙI LẠI TRẢ VẬT ")
                print(" (máy khác có thể publish /demo3/next để đi tiếp ngay) ")
                print("=" * 50 + "\033[0m\n")
                self.wait_hold(self.hold_delay_sec)  # delay thay cho press Enter
                break  # Đã xong lượt đi, thoát vòng lặp để sang giai đoạn 2

        if self._stop_event.is_set() or len(saved_trajectories) < 4:
            self.get_logger().warn("Dừng trước khi hoàn tất lượt đi -> bỏ qua lượt về.")
            return

        self.get_logger().info("--- GIAI ĐOẠN 2: LƯỢT VỀ (ĐẢO NGƯỢC QUỸ ĐẠO CŨ) ---")
        # Lúc này saved_trajectories có 4 phần tử ứng với 4 chặng:
        # Index 0: Start -> Home
        # Index 1: Home -> Pre-pick
        # Index 2: Pre-pick -> Pick
        # Index 3: Pick -> Lift

        print("\033[92m---> Hạ vật xuống vị trí Pick...\033[0m")
        # 1. Đi lùi đoạn 3 (Lift -> Pick)
        traj_lift_to_pick = self.reverse_trajectory(saved_trajectories[3])
        self.execute_by_streaming(traj_lift_to_pick)

        # Nhả vật (DROP)
        self.control_hands(close=False)
        time.sleep(1.0)

        print("\033[92m---> Lùi lại vị trí Pre-pick...\033[0m")
        # 2. Đi lùi đoạn 2 (Pick -> Pre-pick)
        traj_pick_to_prepick = self.reverse_trajectory(saved_trajectories[2])
        self.execute_by_streaming(traj_pick_to_prepick)

        print("\033[92m---> Quay về Home...\033[0m")
        # 3. Đi lùi đoạn 1 (Pre-pick -> Home)
        traj_prepick_to_home = self.reverse_trajectory(saved_trajectories[1])
        self.execute_by_streaming(traj_prepick_to_home)

        self.get_logger().info("--- HOÀN THÀNH TOÀN BỘ QUÁ TRÌNH! ---")


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
        # ĐANG DỪNG LẠI -> gửi các action an toàn TRƯỚC khi tắt node.
        # (rclpy vẫn còn ok ở đây nên publish tới controller vẫn ăn)
        node.get_logger().warn("Ctrl+C -> đang dừng, gửi action an toàn...")
        try:
            node.send_shutdown_actions()
        except Exception as e:
            node.get_logger().error(f"Lỗi khi gửi action dừng: {e}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()