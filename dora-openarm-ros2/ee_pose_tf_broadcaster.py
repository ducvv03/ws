#!/usr/bin/env python3
"""Echo /right_ee_pose and /left_ee_pose, and rebroadcast them as TF.

Subscribes to the geometry_msgs/PoseStamped topics published by
dora_ros2_bridge (see src/dora_ros2_bridge/main.py), prints each received
pose, and broadcasts it as a TF transform world -> right_link / left_link.
Also broadcasts new_right_link/new_left_link, each a *world*-frame child
positioned at right_link/left_link's current world position plus
NEW_LINK_OFFSET — a fixed offset in world axes, independent of the link's
orientation (e.g. if right_link is at world (0, 0, 0), new_right_link is
always at world (0, 0, -0.3)) — but new_link's own orientation is copied
from the link, so it still rotates together with it.

Plain rclpy script, not a Dora node — run directly after sourcing a ROS 2
install, alongside the running dataflow:

    python3 ee_pose_tf_broadcaster.py
"""

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

# Must match dora_ros2_bridge's qos_best_effort for /right_ee_pose/
# /left_ee_pose — a RELIABLE subscriber can't receive from a BEST_EFFORT
# publisher.
_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10
)

# Fixed offset (x, y, z) of new_{right,left}_link from {right,left}_link,
# expressed in WORLD axes — independent of the link's own orientation.
NEW_LINK_OFFSET = (0.0, 0.0, -0.25)


class EePoseTfBroadcaster(Node):
    """Subscribes to /right_ee_pose and /left_ee_pose, echoes them, and broadcasts TF."""

    def __init__(self) -> None:
        """Create subscriptions for both ee_pose topics and the TF broadcaster."""
        super().__init__("ee_pose_tf_broadcaster")
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, "/right_ee_pose", self._make_cb("right_link", "new_right_link"), _QOS)
        self.create_subscription(PoseStamped, "/left_ee_pose", self._make_cb("left_link", "new_left_link"), _QOS)
        self.get_logger().info(
            "echoing /right_ee_pose -> tf world->right_link + world->new_right_link, "
            "/left_ee_pose -> tf world->left_link + world->new_left_link"
        )

    def _make_cb(self, link_frame_id: str, new_link_frame_id: str):
        def _cb(msg: PoseStamped) -> None:
            print(f"[{link_frame_id}] {msg.pose}", flush=True)

            link_tf = TransformStamped()
            link_tf.header.stamp = msg.header.stamp
            link_tf.header.frame_id = "world"
            link_tf.child_frame_id = link_frame_id
            link_tf.transform.translation.x = msg.pose.position.x
            link_tf.transform.translation.y = msg.pose.position.y
            link_tf.transform.translation.z = msg.pose.position.z
            link_tf.transform.rotation = msg.pose.orientation

            # world-frame child: link's world position + a fixed world-axis
            # offset (position doesn't rotate with link_frame_id), but
            # orientation is copied from the link so new_link still rotates
            # together with it.
            new_link_tf = TransformStamped()
            new_link_tf.header.stamp = msg.header.stamp
            new_link_tf.header.frame_id = "world"
            new_link_tf.child_frame_id = new_link_frame_id
            new_link_tf.transform.translation.x = msg.pose.position.x + NEW_LINK_OFFSET[0]
            new_link_tf.transform.translation.y = msg.pose.position.y + NEW_LINK_OFFSET[1]
            new_link_tf.transform.translation.z = msg.pose.position.z + NEW_LINK_OFFSET[2]
            new_link_tf.transform.rotation = msg.pose.orientation

            self._tf_broadcaster.sendTransform([link_tf, new_link_tf])

        return _cb


def main() -> None:
    """Spin the ee_pose_tf_broadcaster node until interrupted."""
    rclpy.init()
    node = EePoseTfBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
