#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

# TF2 Imports
import tf2_ros
import tf2_geometry_msgs  # Required for do_transform_pose
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

# ============================================================
# CONFIG
# ============================================================
TAG_SIZE_M = 0.04
ARUCO_DICT_NAME = cv2.aruco.DICT_APRILTAG_36h11
TARGET_FRAME = "openarm_body_link0"

# The specific offset requested by the user [x, y, z, w]
OFFSET_QUAT = [-0.5, 0.5, 0.5, 0.5]

# Define 5 points in the Tag's local coordinate system
GRASP_CENTER_TAG_M = np.array([0.0, 0.0, 0.0], dtype=np.float32)
GRASP_PRE_LEFT_TAG_M = np.array([0.20, 0.0, 0.0], dtype=np.float32)
GRASP_PRE_RIGHT_TAG_M = np.array([-0.20, 0.0, 0.0], dtype=np.float32)
GRASP_LEFT_TAG_M = np.array([0.16, 0.0, 0.0], dtype=np.float32)
GRASP_RIGHT_TAG_M = np.array([-0.16, 0.0, 0.0], dtype=np.float32)



# ============================================================
# GEOMETRY FUNCTIONS
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


def transform_point_tag_to_camera(point_tag_m, rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    point_tag_m = point_tag_m.reshape(3, 1)
    point_camera_m = R @ point_tag_m + tvec.reshape(3, 1)
    return point_camera_m.reshape(3), R


def project_point_camera_to_pixel(point_camera_m, camera_matrix, dist_coeffs):
    point_camera_m = point_camera_m.reshape(1, 1, 3).astype(np.float32)
    pixel, _ = cv2.projectPoints(point_camera_m, np.zeros((3, 1)), np.zeros((3, 1)), camera_matrix, dist_coeffs)
    u, v = pixel.reshape(2)
    return int(round(u)), int(round(v))


def draw_cross(image, pixel, color, label):
    u, v = pixel
    h, w = image.shape[:2]
    if not (0 <= u < w and 0 <= v < h): return
    cv2.circle(image, (u, v), 6, color, -1)
    cv2.putText(image, label, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)


# ============================================================
# ROS 2 NODE CLASS
# ============================================================
class GraspDetectorNode(Node):
    def __init__(self):
        super().__init__('grasp_detector_node')

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = ""

        # TF2 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
            self.use_new_api = True
        except AttributeError:
            self.detector = None
            self.use_new_api = False

        # Tag points for PnP
        s = TAG_SIZE_M / 2.0
        self.tag_object_points = np.array([[-s, -s, 0.0], [s, -s, 0.0], [s, s, 0.0], [-s, s, 0.0]], dtype=np.float32)

        # Publishers for 5 points
        self.pub_center = self.create_publisher(PoseStamped, "/grasp_points/center", 10)
        self.pub_pre_left = self.create_publisher(PoseStamped, "/grasp_points/pre_left", 10)
        self.pub_pre_right = self.create_publisher(PoseStamped, "/grasp_points/pre_right", 10)
        self.pub_left = self.create_publisher(PoseStamped, "/grasp_points/left", 10)
        self.pub_right = self.create_publisher(PoseStamped, "/grasp_points/right", 10)

        self.pub_debug_img = self.create_publisher(Image, "/grasp_points/debug_image", 1)

        self.image_sub = self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
                                                 self.camera_info_callback, 10)

        self.get_logger().info("Grasp Detector Node (5 Points) started.")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame_id = msg.header.frame_id

    def get_transformed_pose(self, pos_3d, quat, timestamp):
        final_quat = multiply_quaternions(quat, OFFSET_QUAT)

        camera_pose = PoseStamped()
        camera_pose.header.stamp = timestamp
        camera_pose.header.frame_id = self.camera_frame_id
        camera_pose.pose.position.x = float(pos_3d[0])
        camera_pose.pose.position.y = float(pos_3d[1])
        camera_pose.pose.position.z = float(pos_3d[2])
        camera_pose.pose.orientation.x = float(final_quat[0])
        camera_pose.pose.orientation.y = float(final_quat[1])
        camera_pose.pose.orientation.z = float(final_quat[2])
        camera_pose.pose.orientation.w = float(final_quat[3])


        try:
            transform = self.tf_buffer.lookup_transform(TARGET_FRAME, self.camera_frame_id,
                                                        rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            transformed_pose = tf2_geometry_msgs.do_transform_pose(camera_pose.pose, transform)

            output_msg = PoseStamped()
            output_msg.header.stamp = timestamp
            output_msg.header.frame_id = TARGET_FRAME
            output_msg.pose = transformed_pose
            return output_msg
        except Exception as e:
            return None

    def image_callback(self, msg):
        if self.camera_matrix is None: return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        vis_rgb = cv_image.copy()

        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict)

        if ids is not None:
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                success, rvec, tvec = cv2.solvePnP(self.tag_object_points, marker_corners.reshape(4, 2),
                                                   self.camera_matrix, self.dist_coeffs)
                if not success: continue

                # 1. Calculate positions for 5 points in camera space
                p_center_cam, R = transform_point_tag_to_camera(GRASP_CENTER_TAG_M, rvec, tvec)
                p_pre_l_cam, _ = transform_point_tag_to_camera(GRASP_PRE_LEFT_TAG_M, rvec, tvec)
                p_pre_r_cam, _ = transform_point_tag_to_camera(GRASP_PRE_RIGHT_TAG_M, rvec, tvec)
                p_left_cam, _ = transform_point_tag_to_camera(GRASP_LEFT_TAG_M, rvec, tvec)
                p_right_cam, _ = transform_point_tag_to_camera(GRASP_RIGHT_TAG_M, rvec, tvec)

                raw_quat = rotation_matrix_to_quaternion(R)

                # 2. Transform to Robot Frame and Publish
                m_center = self.get_transformed_pose(p_center_cam, raw_quat, msg.header.stamp)
                m_pre_l = self.get_transformed_pose(p_pre_l_cam, raw_quat, msg.header.stamp)
                m_pre_r = self.get_transformed_pose(p_pre_r_cam, raw_quat, msg.header.stamp)
                m_l = self.get_transformed_pose(p_left_cam, raw_quat, msg.header.stamp)
                m_r = self.get_transformed_pose(p_right_cam, raw_quat, msg.header.stamp)

                if m_center: self.pub_center.publish(m_center)
                if m_pre_l:  self.pub_pre_left.publish(m_pre_l)
                if m_pre_r:  self.pub_pre_right.publish(m_pre_r)
                if m_l:      self.pub_left.publish(m_l)
                if m_r:      self.pub_right.publish(m_r)

                # Visual Debug
                for p_cam, lbl in [(p_center_cam, "C"), (p_pre_l_cam, "PL"), (p_pre_r_cam, "PR"), (p_left_cam, "L"),
                                   (p_right_cam, "R")]:
                    px = project_point_camera_to_pixel(p_cam, self.camera_matrix, self.dist_coeffs)
                    draw_cross(vis_rgb, px, (0, 255, 0), lbl)

        self.pub_debug_img.publish(self.bridge.cv2_to_imgmsg(vis_rgb, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    node = GraspDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()