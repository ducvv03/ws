#!/usr/bin/env python3
"""
Motion Planning Client (OMPL only)

- Motion planning via OMPL (RRTConnect)
- Plan both arms → execute simultaneously

Usage:
    python3 motion_planning_client.py
"""

import argparse
import time
import numpy as np
import subprocess
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from vm_per_motion_planning_msgs.srv import PlanMotion, ExecuteMotion, SetPlanningGroup, MotionParams
from vm_per_motion_planning_msgs.msg import PlanningGroup

from vm_robot_demo.robot_config import G1_2_left_arm, G1_2_right_arm


PLAN_TIMEOUT_SEC = 300.0


def parse_args():
    parser = argparse.ArgumentParser(description='OMPL Motion Planning Client')
    parser.add_argument(
        '--planner',
        type=str,
        default='ompl',
        choices=['ompl'],
        help='Only OMPL is supported'
    )
    return parser.parse_args()


class MotionPlanningClient(Node):

    GROUP_JOINTS = {
        G1_2_right_arm.group_name: G1_2_right_arm.joint_names,
        G1_2_left_arm.group_name: G1_2_left_arm.joint_names,
    }

    def __init__(self, planner='ompl'):
        super().__init__('plan_motion_client')

        self.planner = 'ompl'

        self.plan_cli = self.create_client(PlanMotion, '/vm_motion_planner/plan_motion')
        self.exec_cli = self.create_client(ExecuteMotion, '/vm_motion_planner/execute_motion')
        self.set_group_cli = self.create_client(SetPlanningGroup, '/vm_motion_planner/set_planning_group')
        self.motion_params_cli = self.create_client(MotionParams, '/vm_motion_planner/set_motion_params')

        self.get_logger().info('Waiting for services...')
        self.plan_cli.wait_for_service()
        self.exec_cli.wait_for_service()
        self.set_group_cli.wait_for_service()
        self.motion_params_cli.wait_for_service()

        self.current_group = 'right_arm'
        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.get_logger().info('Waiting for /joint_states...')
        while rclpy.ok() and self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.5)

        self.get_logger().info('Joint state received')
        self.set_motion_params()

    def _joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def _spin(self, duration: float = 0.5):
        deadline = time.time() + duration
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------------ #
    # OMPL ONLY
    # ------------------------------------------------------------------ #
    def set_motion_params(self) -> bool:
        req = MotionParams.Request()
        req.planning_pipeline_id = 'ompl'
        req.planner_id = 'RRTConnect'
        req.number_attempts = 1
        req.planning_time = 120.0
        req.goal_tolerance = 0.01

        self.get_logger().info('Switching to OMPL (RRTConnect)...')
        future = self.motion_params_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

        result = future.result()
        if result and result.status.value == 0:
            self.get_logger().info('OMPL enabled')
            return True

        self.get_logger().error('Failed to set OMPL')
        return False

    # ------------------------------------------------------------------ #
    def set_planning_group(self, group: PlanningGroup) -> bool:
        req = SetPlanningGroup.Request()
        req.planning_group = group

        self.get_logger().info(f'Setting group → {group.name}')
        future = self.set_group_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        result = future.result()
        if result and result.rc.value == 0:
            self.current_group = group.name
            self.set_motion_params()  # restore OMPL
            return True

        self.get_logger().error('Failed to set group')
        return False

    # ------------------------------------------------------------------ #
    def _get_group_seed(self, group_joints=None) -> JointState:
        if group_joints is None:
            group_joints = self.GROUP_JOINTS.get(
                self.current_group, G1_2_right_arm.joint_names
            )

        if self.current_joint_state is None:
            return JointState()

        seed = JointState()
        name_to_idx = {n: i for i, n in enumerate(self.current_joint_state.name)}

        for jn in group_joints:
            if jn in name_to_idx:
                idx = name_to_idx[jn]
                seed.name.append(jn)
                seed.position.append(self.current_joint_state.position[idx])

        return seed

    # ------------------------------------------------------------------ #
    def plan_to_pose(self, position, orientation, frame_id='pelvis'):
        req = PlanMotion.Request()
        req.target_type = PlanMotion.Request.POSE
        req.seed_ik_state = self._get_group_seed()

        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position = Point(
            x=float(position[0]),
            y=float(position[1]),
            z=float(position[2]),
        )
        goal.pose.orientation = Quaternion(
            x=float(orientation[0]),
            y=float(orientation[1]),
            z=float(orientation[2]),
            w=float(orientation[3]),
        )

        req.goal_pose = goal

        self.get_logger().info(f'Planning to {position}')
        return self._call(req)

    # ------------------------------------------------------------------ #
    def plan_to_joint(self, joint_names, joint_positions):
        req = PlanMotion.Request()
        req.target_type = PlanMotion.Request.JOINT
        req.seed_ik_state = self._get_group_seed()

        goal = JointState()
        goal.name = joint_names
        goal.position = [float(p) for p in joint_positions]

        req.goal_joint = goal
        return self._call(req)

    # ------------------------------------------------------------------ #
    def _call(self, request):
        self._spin(1.0)

        self.get_logger().info('Sending OMPL plan request...')
        future = self.plan_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=PLAN_TIMEOUT_SEC)

        resp = future.result()
        if not resp:
            self.get_logger().error('No response')
            return None

        if resp.rc.value == 0:
            self.get_logger().info(
                f'Plan OK: {len(resp.joint_trajectory.points)} points'
            )
            return resp

        self.get_logger().error(f'Plan failed: {resp.rc.message}')
        return None

    # ------------------------------------------------------------------ #
    def execute(self, plan_response, planning_group='') -> bool:
        if not plan_response:
            return False

        req = ExecuteMotion.Request()
        req.joint_trajectory = plan_response.joint_trajectory
        req.cart_trajectory = plan_response.cart_trajectory
        req.start_state = plan_response.start_state

        self.get_logger().info('Executing...')
        future = self.exec_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        result = future.result()
        return result and result.rc.value == 0


# ------------------------------------------------------------------ #
def main():
    args = parse_args()

    rclpy.init()
    client = MotionPlanningClient(planner=args.planner)

    right_group = PlanningGroup(name='right_arm', tool_link='right_gripper_tcp', root_link='pelvis')
    left_group = PlanningGroup(name='left_arm', tool_link='left_gripper_tcp', root_link='pelvis')
    both_group = PlanningGroup(name='both_arms', tool_link='right_gripper_tcp', root_link='pelvis')

    # Example targets
    right_pos = [0.33, -0.03, 0.0]
    right_ori = [0.6, -0.4, -0.5, -0.4]

    left_pos = [0.33, 0.03, 0.0]
    left_ori = [-0.6, 0.4, 0.5, -0.4]

    client.set_planning_group(right_group)
    right_resp = client.plan_to_pose(right_pos, right_ori)

    client.set_planning_group(left_group)
    left_resp = client.plan_to_pose(left_pos, left_ori)

    if right_resp and left_resp:
        client.set_planning_group(both_group)
        client.get_logger().info('Executing both arms')
        client.execute(right_resp)
        client.execute(left_resp)

    elif right_resp:
        client.execute(right_resp)

    elif left_resp:
        client.execute(left_resp)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()