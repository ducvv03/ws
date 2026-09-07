#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import csv
import os
from datetime import datetime


class JointStateCSVRecorder(Node):

    def __init__(self):
        super().__init__('joint_state_csv_recorder')

        # Create CSV file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = f"joint_states_{timestamp}.csv"

        self.file = open(self.csv_file, mode='w', newline='')
        self.writer = None

        self.get_logger().info(
            f"Saving joint states to: {os.path.abspath(self.csv_file)}"
        )

        # Subscribe joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):

        # Create CSV header once
        if self.writer is None:
            header = ['time'] + msg.name
            self.writer = csv.DictWriter(
                self.file,
                fieldnames=header
            )
            self.writer.writeheader()

        # Prepare data
        data = {
            'time': self.get_clock().now().nanoseconds / 1e9
        }

        for name, position in zip(msg.name, msg.position):
            data[name] = position

        # Write row
        self.writer.writerow(data)
        self.file.flush()

        self.get_logger().info(
            f"Recorded: {msg.name} -> {msg.position}"
        )


    def destroy_node(self):
        self.file.close()
        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)

    node = JointStateCSVRecorder()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()