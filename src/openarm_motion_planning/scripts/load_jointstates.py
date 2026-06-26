import rclpy
from rclpy.node import Node
import csv
import time
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class CsvLivePlayer(Node):
    def __init__(self):
        super().__init__('csv_live_player')

        # Publishers
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            "/left_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            "/right_joint_trajectory_controller/joint_trajectory",
            10
        )

        self.csv_file = "joint_states_20260624_180841.csv"

        # Joint configurations (Skips fingers automatically by only listing arm joints)
        self.left_joints = [
            "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
            "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
            "openarm_left_joint7"
        ]

        self.right_joints = [
            "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
            "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
            "openarm_right_joint7"
        ]

        # Start the playback thread
        self.playback_thread = threading.Thread(target=self.run_playback)
        self.playback_thread.start()

    def create_point(self, joint_names, row, dt):
        """Creates a single trajectory point for a specific set of joints."""
        point = JointTrajectoryPoint()
        point.positions = [float(row[name]) for name in joint_names]

        # Set time_from_start to the delta between frames to ensure smooth interpolation
        duration = Duration()
        duration.sec = int(dt)
        duration.nanosec = int((dt - duration.sec) * 1e9)
        point.time_from_start = duration
        return point

    def run_playback(self):
        self.get_logger().info(f"Opening CSV: {self.csv_file}")

        try:
            with open(self.csv_file, mode='r') as f:
                # Use DictReader to easily access columns by name
                reader = list(csv.DictReader(f))

                for i in range(len(reader) - 1):
                    current_row = reader[i]
                    next_row = reader[i + 1]

                    # Calculate how long to wait until the next point
                    t_now = float(current_row['time'])
                    t_next = float(next_row['time'])
                    dt = t_next - t_now

                    # Prevent negative sleep or errors if timestamps are identical
                    if dt <= 0:
                        dt = 0.01

                        # Prepare Left Arm Message
                    left_msg = JointTrajectory()
                    left_msg.joint_names = self.left_joints
                    left_msg.points.append(self.create_point(self.left_joints, current_row, dt))

                    # Prepare Right Arm Message
                    right_msg = JointTrajectory()
                    right_msg.joint_names = self.right_joints
                    right_msg.points.append(self.create_point(self.right_joints, current_row, dt))

                    # Publish
                    self.left_arm_pub.publish(left_msg)
                    self.right_arm_pub.publish(right_msg)

                    # Mimic motion by sleeping for the duration recorded in the CSV
                    time.sleep(dt)

            self.get_logger().info("Finished CSV playback.")

        except FileNotFoundError:
            self.get_logger().error(f"Could not find {self.csv_file}")
        except Exception as e:
            self.get_logger().error(f"Error during playback: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = CsvLivePlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()