#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import yaml
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import tf2_geometry_msgs

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from pnk_perception_msgs.msg import GraspTarget
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent.parent
YAML_PATH = PACKAGE_DIR / "config" / "config.yaml"
TARGET_FRAME = "openarm_body_link0"


# ============================================================
# MATH HELPER FUNCTIONS
# ============================================================
def multiply_quaternions(q1, q2):
    """Multiplies two quaternions [x, y, z, w]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    ]


def rotation_matrix_to_quaternion(R):
    """Converts 3x3 rotation matrix to quaternion [x, y, z, w]."""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return [qx, qy, qz, qw]


def quaternion_to_rotation_matrix(q):
    """Converts quaternion [x, y, z, w] to 3x3 rotation matrix."""
    x, y, z, w = q
    R = np.zeros((3, 3), dtype=np.float32)
    R[0, 0] = 1 - 2 * (y ** 2 + z ** 2)
    R[0, 1] = 2 * (x * y - z * w)
    R[0, 2] = 2 * (x * z + y * w)
    R[1, 0] = 2 * (x * y + z * w)
    R[1, 1] = 1 - 2 * (x ** 2 + z ** 2)
    R[1, 2] = 2 * (y * z - x * w)
    R[2, 0] = 2 * (x * z - y * w)
    R[2, 1] = 2 * (y * z + x * w)
    R[2, 2] = 1 - 2 * (x ** 2 + y ** 2)
    return R


def apply_offset_to_pose(base_pose, offset_m, fixed_quat):
    """(Cylinder Logic) Applies local offset in Robot Frame based on fixed quaternion."""
    pos = base_pose.pose.position
    R = quaternion_to_rotation_matrix(fixed_quat)
    offset_vec = np.array(offset_m, dtype=np.float32).reshape(3, 1)
    rotated_offset = R @ offset_vec

    new_pose = PoseStamped()
    new_pose.header = base_pose.header
    new_pose.pose.position.x = pos.x + float(rotated_offset[0, 0])
    new_pose.pose.position.y = pos.y + float(rotated_offset[1, 0])
    new_pose.pose.position.z = pos.z + float(rotated_offset[2, 0])
    new_pose.pose.orientation.x = float(fixed_quat[0])
    new_pose.pose.orientation.y = float(fixed_quat[1])
    new_pose.pose.orientation.z = float(fixed_quat[2])
    new_pose.pose.orientation.w = float(fixed_quat[3])
    return new_pose


def transform_point_tag_to_camera(point_tag_m, rvec, tvec):
    """Transforms point from Tag Frame to Camera Frame."""
    R, _ = cv2.Rodrigues(rvec)
    point_tag_m = np.array(point_tag_m).reshape(3, 1)
    point_camera_m = R @ point_tag_m + tvec.reshape(3, 1)
    return point_camera_m.reshape(3), R


# ============================================================
# MAIN NODE
# ============================================================
class GraspDetectorNode(Node):
    def __init__(self):
        super().__init__('grasp_detector_node')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = ""

        # Load YAML config
        with open(YAML_PATH, 'r') as file:
            self.config = yaml.safe_load(file)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Hỗ trợ Aruco API cũ và mới
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
            self.use_new_api = True
        except AttributeError:
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False

        self.pub_grasp_target = self.create_publisher(GraspTarget, "/grasp_target_info", 10)
        self.pub_debug_img = self.create_publisher(Image, "/grasp_points/debug_image", 1)

        self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_callback, 10)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.camera_info_callback, 10)

        self.get_logger().info("Unified Detector Node started.")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame_id = msg.header.frame_id

    def create_and_transform_pose(self, p_cam, quat, stamp):
        """Helper để convert điểm Camera + Quaternion => Robot Pose"""
        cam_pose = PoseStamped()
        cam_pose.header.stamp = stamp
        cam_pose.header.frame_id = self.camera_frame_id
        cam_pose.pose.position.x = float(p_cam[0])
        cam_pose.pose.position.y = float(p_cam[1])
        cam_pose.pose.position.z = float(p_cam[2])
        cam_pose.pose.orientation.x = float(quat[0])
        cam_pose.pose.orientation.y = float(quat[1])
        cam_pose.pose.orientation.z = float(quat[2])
        cam_pose.pose.orientation.w = float(quat[3])
        try:
            transform = self.tf_buffer.lookup_transform(TARGET_FRAME, self.camera_frame_id, rclpy.time.Time())
            m_pose = tf2_geometry_msgs.do_transform_pose(cam_pose.pose, transform)
            res = PoseStamped()
            res.header.stamp = stamp
            res.header.frame_id = TARGET_FRAME
            res.pose = m_pose
            return res
        except Exception:
            return None

    def image_callback(self, msg):
        if self.camera_matrix is None: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            transform_back = None
            try:
                transform_back = self.tf_buffer.lookup_transform(
                    self.camera_frame_id, TARGET_FRAME, rclpy.time.Time()
                )
            except Exception:
                pass

            for marker_corners, marker_id in zip(corners, ids.flatten()):
                tag_id = int(marker_id)
                cfg = self.config['id_10'] if tag_id == 10 else self.config['id_other']

                s = cfg['tag_size_m'] / 2.0
                tag_pts = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float32)

                success, rvec, tvec = cv2.solvePnP(tag_pts, marker_corners.reshape(4, 2), self.camera_matrix,
                                                   self.dist_coeffs)
                if not success: continue

                # Variables to hold the final 5 points in Robot Frame
                m_center, m_pre_l, m_pre_r, m_l, m_r = None, None, None, None, None

                # =========================================================
                # LOGIC 1: RECTANGLE (Tag-Relative Orientation & Offsets)
                # =========================================================
                if cfg['calculation_method'] == "tag_relative":
                    p_c, R = transform_point_tag_to_camera(cfg['center_tag_m'], rvec, tvec)
                    p_pl, _ = transform_point_tag_to_camera(cfg['pre_left_tag_m'], rvec, tvec)
                    p_pr, _ = transform_point_tag_to_camera(cfg['pre_right_tag_m'], rvec, tvec)
                    p_l, _ = transform_point_tag_to_camera(cfg['left_tag_m'], rvec, tvec)
                    p_r, _ = transform_point_tag_to_camera(cfg['right_tag_m'], rvec, tvec)

                    raw_q = rotation_matrix_to_quaternion(R)
                    final_q = multiply_quaternions(raw_q, cfg['offset_quat'])
                    final_q_grasp = multiply_quaternions(raw_q, cfg['offset_quat_grasp'])

                    m_center = self.create_and_transform_pose(p_c, final_q, msg.header.stamp)
                    m_pre_l = self.create_and_transform_pose(p_pl, final_q_grasp, msg.header.stamp)
                    m_pre_r = self.create_and_transform_pose(p_pr, final_q_grasp, msg.header.stamp)
                    m_l = self.create_and_transform_pose(p_l, final_q_grasp, msg.header.stamp)
                    m_r = self.create_and_transform_pose(p_r, final_q_grasp, msg.header.stamp)

                # =========================================================
                # LOGIC 2: CYLINDER (Fixed Robot Orientation & Robot Offsets)
                # =========================================================
                elif cfg['calculation_method'] == "fixed_orientation":
                    p_c, _ = transform_point_tag_to_camera(cfg['center_tag_m'], rvec, tvec)

                    # Bước 1: Transform duy nhất điểm Center sang Robot với Quaternion Identity [0,0,0,1]
                    m_center_temp = self.create_and_transform_pose(p_c, [0.0, 0.0, 0.0, 1.0], msg.header.stamp)
                    if m_center_temp is not None:
                        fixed_q = cfg['fixed_quat']
                        fixed_quat_left_grasp = cfg['fixed_quat_left_grasp']
                        fixed_quat_right_grasp = cfg['fixed_quat_right_grasp']

                        # Bước 2: Gắn Fixed Quat vào Center
                        m_center = PoseStamped()
                        m_center.header = m_center_temp.header
                        m_center.pose.position = m_center_temp.pose.position
                        m_center.pose.orientation.x = float(fixed_q[0])
                        m_center.pose.orientation.y = float(fixed_q[1])
                        m_center.pose.orientation.z = float(fixed_q[2])
                        m_center.pose.orientation.w = float(fixed_q[3])

                        # Bước 3: Tính các điểm còn lại dựa trên Center và Robot Frame offsets
                        m_pre_l = apply_offset_to_pose(m_center, cfg['offset_pre_left'], fixed_q)
                        m_pre_r = apply_offset_to_pose(m_center, cfg['offset_pre_right'], fixed_q)
                        m_l = apply_offset_to_pose(m_center, cfg['offset_left'], fixed_quat_left_grasp)
                        m_r = apply_offset_to_pose(m_center, cfg['offset_right'], fixed_quat_right_grasp)

                # =========================================================
                # XUẤT DATA & VISUALIZATION
                # =========================================================
                if all(v is not None for v in [m_center, m_pre_l, m_pre_r, m_l, m_r]):
                    # 1. Publish Message
                    target_msg = GraspTarget()
                    target_msg.tag_id = tag_id
                    target_msg.center = m_center
                    target_msg.pre_left = m_pre_l
                    target_msg.pre_right = m_pre_r
                    target_msg.left = m_l
                    target_msg.right = m_r
                    self.pub_grasp_target.publish(target_msg)

                    # 2. Vẽ hình (Visualization bằng cách transform ngược về Camera Frame)
                    if transform_back is not None:
                        points_to_draw = [
                            (m_center, "C"), (m_pre_l, "PL"), (m_pre_r, "PR"), (m_l, "L"), (m_r, "R")
                        ]
                        for p_msg, label in points_to_draw:
                            cam_pose_back = tf2_geometry_msgs.do_transform_pose(p_msg.pose, transform_back)
                            p_cam = np.array([cam_pose_back.position.x,
                                              cam_pose_back.position.y,
                                              cam_pose_back.position.z], dtype=np.float32)

                            pixel, _ = cv2.projectPoints(p_cam.reshape(1, 1, 3), np.zeros(3), np.zeros(3),
                                                         self.camera_matrix, self.dist_coeffs)
                            u, v = int(round(pixel[0, 0, 0])), int(round(pixel[0, 0, 1]))

                            h, w = cv_image.shape[:2]
                            if 0 <= u < w and 0 <= v < h:
                                cv2.circle(cv_image, (u, v), 5, (0, 255, 0), -1)
                                cv2.putText(cv_image, label, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                            (0, 255, 255), 2)

        self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


def main():
    rclpy.init()
    node = GraspDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__': main()