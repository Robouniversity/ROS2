#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from unitree_hg.msg import LowState


class LowStateSubscriber(Node):

    def __init__(self):
        super().__init__("low_state_subscriber")

        self.sub = self.create_subscription(
            LowState,
            "/lowstate",
            self.callback,
            10
        )

        self.get_logger().info("Waiting for LowState messages...")


    def callback(self, msg):

        print("Received LowState")

        print("Quaternion:",
              msg.imu_state.quaternion)

        print("Gyroscope:",
              msg.imu_state.gyroscope)

        print("Accelerometer:",
              msg.imu_state.accelerometer)


def main():

    rclpy.init()

    node = LowStateSubscriber()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()