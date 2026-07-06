#!/usr/bin/env python3
"""
ROS 2 node for Unitree G1 high-level arm actions.

This is based on Unitree's g1_arm_action_example.py, but replaces the blocking
terminal loop with a ROS 2 subscription. Send an action name, action id, or
"list" to the command topic.

Example:
    # Starts with network_interface:=lo by default.
    python3 examples_unitree_python_sdk/high_level_ros2_example_1.py

    # Override the interface when connecting to the robot network.
    python3 examples_unitree_python_sdk/high_level_ros2_example_1.py --ros-args \
        -p network_interface:=enp4s0

    ros2 topic pub --once /g1_arm_action/command std_msgs/msg/String \
        "{data: 'shake hand'}"

Safety:
    Make sure the robot has clear space around its arms before executing any
    action. High-level actions can move quickly and may continue briefly after
    the command is sent.
"""

import sys
import time
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map


@dataclass(frozen=True)
class ArmActionOption:
    """Human-readable action name and the numeric id used by Unitree's example."""

    name: str
    action_id: int


# Keep the same action ids and names as Unitree's original terminal example.
ACTION_OPTIONS = [
    ArmActionOption(name="release arm", action_id=0),
    ArmActionOption(name="shake hand", action_id=1),
    ArmActionOption(name="high five", action_id=2),
    ArmActionOption(name="hug", action_id=3),
    ArmActionOption(name="high wave", action_id=4),
    ArmActionOption(name="clap", action_id=5),
    ArmActionOption(name="face wave", action_id=6),
    ArmActionOption(name="left kiss", action_id=7),
    ArmActionOption(name="heart", action_id=8),
    ArmActionOption(name="right heart", action_id=9),
    ArmActionOption(name="hands up", action_id=10),
    ArmActionOption(name="x-ray", action_id=11),
    ArmActionOption(name="right hand up", action_id=12),
    ArmActionOption(name="reject", action_id=13),
    ArmActionOption(name="right kiss", action_id=14),
    ArmActionOption(name="two-hand kiss", action_id=15),
]

# Unitree's example explicitly releases the arm after these actions.
AUTO_RELEASE_ACTION_IDS = {1, 2, 3, 8, 9, 10, 11, 12, 13}


class G1ArmActionNode(Node):
    """ROS 2 wrapper around Unitree's G1ArmActionClient."""

    def __init__(self):
        super().__init__("g1_arm_action")

        # Use loopback by default so the node can start without arguments,
        # which is useful for local testing or simulator setups.
        self.declare_parameter("network_interface", "enp4s0")
        self.declare_parameter("command_topic", "~/command")
        self.declare_parameter("client_timeout", 10.0)
        self.declare_parameter("auto_release", True)
        self.declare_parameter("release_delay_sec", 2.0)

        self.network_interface = (
            self.get_parameter("network_interface").get_parameter_value().string_value
        )
        command_topic = (
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        client_timeout = (
            self.get_parameter("client_timeout").get_parameter_value().double_value
        )

        if not self.network_interface:
            self.network_interface = "enp4s0"

        self.get_logger().info(
            "Using network interface '%s' for Unitree SDK communication."
            % self.network_interface
        )

        self.arm_action_client = G1ArmActionClient()
        self.arm_action_client.SetTimeout(client_timeout)
        self.arm_action_client.Init()

        self.command_subscriber = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info(
            "G1 arm action node ready on topic '%s'. Send 'list', an action "
            "name, or an action id." % self.command_subscriber.topic_name
        )

    def command_callback(self, message):
        """Handle one command from ROS 2."""

        command = message.data.strip()
        if not command:
            self.get_logger().warn("Ignoring empty arm action command.")
            return

        if command.lower() == "list":
            self.log_action_options()
            return

        action_option = self.parse_action_option(command)
        if action_option is None:
            self.get_logger().warn(
                "Unknown arm action '%s'. Send 'list' to see valid actions."
                % command
            )
            return

        threading.Thread(target=self.execute_action, args=(action_option,), daemon=True).start()

    def parse_action_option(self, command):
        """Convert an incoming action name or id into an ArmActionOption."""

        normalized_command = command.lower()
        action_id = self.parse_int(command)

        for option in ACTION_OPTIONS:
            if normalized_command == option.name or action_id == option.action_id:
                return option

        return None

    def execute_action(self, option):
        """Run the requested Unitree arm action and optionally release the arm."""

        unitree_action = action_map.get(option.name)
        if unitree_action is None:
            self.get_logger().error(
                "Action '%s' is not available in unitree_sdk2py action_map."
                % option.name
            )
            return

        self.get_logger().info(
            "Executing arm action: %s (id: %d)" % (option.name, option.action_id)
        )
        self.arm_action_client.ExecuteAction(unitree_action)

        if self.should_auto_release(option):
            release_delay = (
                self.get_parameter("release_delay_sec")
                .get_parameter_value()
                .double_value
            )
            self.create_timer(
                release_delay,
                lambda: (self.release_arm())
            )

    def should_auto_release(self, option):
        """Match the post-action release behavior in Unitree's original script."""

        auto_release = (
            self.get_parameter("auto_release").get_parameter_value().bool_value
        )
        return auto_release and option.action_id in AUTO_RELEASE_ACTION_IDS

    def release_arm(self):
        """Send Unitree's 'release arm' action."""

        release_action = action_map.get("release arm")
        if release_action is None:
            self.get_logger().error("'release arm' is missing from action_map.")
            return

        self.get_logger().info("Releasing arm.")
        self.arm_action_client.ExecuteAction(release_action)

    def log_action_options(self):
        """Print all supported actions in a ROS-friendly format."""

        lines = ["Available G1 arm actions:"]
        for option in ACTION_OPTIONS:
            lines.append("  %2d: %s" % (option.action_id, option.name))
        self.get_logger().info("\n".join(lines))

    @staticmethod
    def parse_int(value):
        try:
            return int(value)
        except ValueError:
            return None


def main(args=None):
    interface="enp4s0"
    if "--ros-args" in sys.argv and "-p" in sys.argv:
        pass
    # Initialize Unitree DDS before ROS2
    ChannelFactoryInitialize(0, interface)

    rclpy.init(args=args)

    node = None
    try:
        node = G1ArmActionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print("Failed to start G1 arm action node: %s" % exc, file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()