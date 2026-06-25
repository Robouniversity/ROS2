#!/usr/bin/env python3
"""Subscribe to and display Unitree G1 low-level state data using ROS 2.

This is a read-only diagnostic node: it receives ``LowState`` messages from
the ``/lowstate`` topic and prints IMU and motor feedback. It does not publish
commands or move the robot.
"""

import rclpy
from rclpy.node import Node

from unitree_hg.msg import LowState


class LowStateSubscriber(Node):
    """ROS 2 node that prints low-level robot feedback."""

    def __init__(self):
        """Create the node and subscribe to the low-state topic."""
        super().__init__("low_state_subscriber")

        # Keep the subscription as an instance attribute so it remains alive
        # for as long as this node is running. The queue depth of 10 allows
        # ROS 2 to buffer a few messages if the callback is briefly busy.
        self.subscription = self.create_subscription(
            LowState,
            "/lowstate",
            self.callback,
            10,
        )

        self.get_logger().info("Waiting for LowState messages...")

    def callback(self, msg):
        """Print IMU data and the state of every motor in one message."""
        print("\n" + "=" * 80)
        print("Received LowState")

        # IMU feedback describes the robot body's orientation and motion.
        print("\nIMU")
        print("  Quaternion :", msg.imu_state.quaternion)
        print("  Gyroscope  :", msg.imu_state.gyroscope)
        print("  Accelerom. :", msg.imu_state.accelerometer)

        # motor_state contains one entry per joint:
        #   q       - measured joint position in radians
        #   dq      - measured joint velocity in radians per second
        #   ddq     - measured joint acceleration in radians per second squared
        #   tau_est - estimated joint torque in newton-metres
        print("\nMotors")
        print("Idx     q(rad)     dq(rad/s)    ddq(rad/s^2)   tau_est(Nm)")

        for index, motor in enumerate(msg.motor_state):
            print(
                f"{index:02d}   "
                f"{motor.q:9.4f}   "
                f"{motor.dq:9.4f}   "
                f"{motor.ddq:12.4f}   "
                f"{motor.tau_est:11.4f}"
            )


def main(args=None):
    """Initialize ROS 2 and run the subscriber until it is stopped."""
    rclpy.init(args=args)

    node = LowStateSubscriber()

    try:
        # Process incoming messages until Ctrl+C or ROS 2 requests shutdown.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicit cleanup avoids leaving ROS resources open.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
