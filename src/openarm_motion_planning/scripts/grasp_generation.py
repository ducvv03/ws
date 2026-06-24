#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped

# ============================================================
# CONFIG
# ============================================================
TAG_SIZE_M = 0.04
ARUCO_DICT_NAME = cv2.aruco.DICT_APRILTAG_36h11

TAG_CENTER_TAG_M = np.array([0.0, 0.0, 0.0], dtype=np.float32)
GRASP_CENTER_TAG_M = np.array([0.0, 0.0, 0.0], dtype=np.float32)
GRASP_1_TAG_M = np.array([0.075, 0.0, 0.0], dtype=np.float32)
GRASP_2_TAG_M = np.array([-0.075, 0.0, 0.0], dtype=np.float32)


# ============================================================
# GEOMETRY FUNCTIONS
# ============================================================
def make_tag_object_points(tag_size_m):
    s = tag_size_m / 2.0
    return np.array([
        [-s, -s, 0.0],
        [s, -s, 0.0],
        [s, s, 0.0],
        [-s, s, 0.0],
    ], dtype=np.float32)


def transform_point_tag_to_camera(point_tag_m, rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    point_tag_m = point_tag_m.reshape(3, 1)
    point_camera_m = R @ point_tag_m + tvec.reshape(3, 1)
    return point_camera_m.reshape(3)


def project_point_camera_to_pixel(point_camera_m, camera_matrix, dist_coeffs):
    point_camera_m = point_camera_m.reshape(1, 1, 3).astype(np.float32)
    pixel, _ = cv2.projectPoints(
        point_camera_m, np.zeros((3, 1), dtype=np.float32), np.zeros((3, 1), dtype=np.float32),
        camera_matrix, dist_coeffs
    )
    u, v = pixel.reshape(2)
    return int(round(u)), int(round(v))


def is_pixel_inside_image(pixel, image):
    u, v = pixel
    h, w = image.shape[:2]
    return 0 <= u < w and 0 <= v < h


def draw_cross(image, pixel, color, label):
    if not is_pixel_inside_image(pixel, image): return
    u, v = pixel
    cv2.circle(image, (u, v), 8, color, -1)
    cv2.line(image, (u - 14, v), (u + 14, v), color, 2)
    cv2.line(image, (u, v - 14), (u, v + 14), color, 2)
    cv2.putText(image, label, (u + 10, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)


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

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_NAME)
        try:
            self.parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
            self.use_new_api = True
        except AttributeError:
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_api = False

        self.tag_object_points = make_tag_object_points(TAG_SIZE_M)

        # Publishers
        self.pub_center = self.create_publisher(PointStamped, "/grasp_points/center", 10)
        self.pub_left = self.create_publisher(PointStamped, "/grasp_points/left", 10)
        self.pub_right = self.create_publisher(PointStamped, "/grasp_points/right", 10)
        self.pub_debug_img = self.create_publisher(Image, "/grasp_points/debug_image", 1)

        # Subscribers (Chỉnh lại tên topic cho phù hợp với Realsense ROS 2 của bạn)
        self.image_sub = self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
                                                 self.camera_info_callback, 10)

        self.get_logger().info("Grasp Detector ROS 2 Node has started.")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d, dtype=np.float32)
            self.camera_frame_id = msg.header.frame_id
            self.get_logger().info("Received Camera Intrinsics.")

    def publish_point(self, publisher, point_3d_m, timestamp):
        msg = PointStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.camera_frame_id
        msg.point.x = float(point_3d_m[0])
        msg.point.y = float(point_3d_m[1])
        msg.point.z = float(point_3d_m[2])
        publisher.publish(msg)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        vis_rgb = cv_image.copy()

        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(vis_rgb, corners, ids)
            for marker_corners, marker_id in zip(corners, ids.flatten()):
                image_points = marker_corners.reshape(4, 2).astype(np.float32)
                success, rvec, tvec = cv2.solvePnP(
                    self.tag_object_points, image_points, self.camera_matrix, self.dist_coeffs,
                    flags=cv2.SOLVEPNP_ITERATIVE
                )

                if not success: continue

                grasp_center_cam = transform_point_tag_to_camera(GRASP_CENTER_TAG_M, rvec, tvec)
                grasp_left_cam = transform_point_tag_to_camera(GRASP_1_TAG_M, rvec, tvec)
                grasp_right_cam = transform_point_tag_to_camera(GRASP_2_TAG_M, rvec, tvec)

                current_time = msg.header.stamp
                self.publish_point(self.pub_center, grasp_center_cam, current_time)
                self.publish_point(self.pub_left, grasp_left_cam, current_time)
                self.publish_point(self.pub_right, grasp_right_cam, current_time)

                cv2.drawFrameAxes(vis_rgb, self.camera_matrix, self.dist_coeffs, rvec, tvec, TAG_SIZE_M * 0.8)

                g_center_px = project_point_camera_to_pixel(grasp_center_cam, self.camera_matrix, self.dist_coeffs)
                g_left_px = project_point_camera_to_pixel(grasp_left_cam, self.camera_matrix, self.dist_coeffs)
                g_right_px = project_point_camera_to_pixel(grasp_right_cam, self.camera_matrix, self.dist_coeffs)

                draw_cross(vis_rgb, g_center_px, (0, 255, 0), "CENTER")
                draw_cross(vis_rgb, g_left_px, (255, 0, 255), "LEFT")
                draw_cross(vis_rgb, g_right_px, (0, 255, 255), "RIGHT")

                if is_pixel_inside_image(g_left_px, vis_rgb) and is_pixel_inside_image(g_right_px, vis_rgb):
                    cv2.line(vis_rgb, g_left_px, g_center_px, (255, 255, 255), 2)
                    cv2.line(vis_rgb, g_center_px, g_right_px, (255, 255, 255), 2)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(vis_rgb, "bgr8")
            self.pub_debug_img.publish(debug_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GraspDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()