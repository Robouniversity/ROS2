#!/usr/bin/env python3
"""
ROS 2 node for Unitree G1 high-level locomotion actions.

This is based on Unitree's g1_loco_client_example.py, but replaces the blocking
terminal loop with a ROS 2 subscription. Send an action name, action id, or
"list" to the command topic.

Example:
    # Starts with network_interface:=lo by default.
    python3 examples_unitree_python_sdk/high_level_ros2_example_2.py

    # Override the interface when connecting to the robot network.
    python3 examples_unitree_python_sdk/high_level_ros2_example_2.py --ros-args \
        -p network_interface:=eth0

    ros2 topic pub --once /g1_loco/command std_msgs/msg/String \
        "{data: 'move forward'}"

Safety:
    Make sure the robot has clear, flat space around it before executing any
    locomotion action. Actions like Lie2StandUp require the robot to face up on
    hard, flat, rough ground, matching Unitree's original warning.
"""

import sys
import time
import threading
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


@dataclass(frozen=True)
class LocoActionOption:
    """Human-readable action name and the numeric id used by Unitree's example."""

    name: str
    action_id: int


# Keep the same action ids and names as Unitree's original terminal example.
ACTION_OPTIONS = [
    LocoActionOption(name="damp", action_id=0),
    LocoActionOption(name="Squat2StandUp", action_id=1),
    LocoActionOption(name="StandUp2Squat", action_id=2),
    LocoActionOption(name="move forward", action_id=3),
    LocoActionOption(name="move lateral", action_id=4),
    LocoActionOption(name="move rotate", action_id=5),
    LocoActionOption(name="low stand", action_id=6),
    LocoActionOption(name="high stand", action_id=7),
    LocoActionOption(name="zero torque", action_id=8),
    LocoActionOption(name="wave hand1", action_id=9),
    LocoActionOption(name="wave hand2", action_id=10),
    LocoActionOption(name="shake hand", action_id=11),
    LocoActionOption(name="Lie2StandUp", action_id=12),
]


class G1LocoNode(Node):
    """ROS 2 wrapper around Unitree's LocoClient."""

    def __init__(self):
        super().__init__("g1_loco")

        # Use loopback by default so the node can start without arguments,
        # which is useful for local testing or simulator setups.
        self.declare_parameter("network_interface", "lo")
        self.declare_parameter("command_topic", "~/command")
        self.declare_parameter("client_timeout", 10.0)
        self.declare_parameter("move_forward_speed", 0.3)
        self.declare_parameter("move_lateral_speed", 0.3)
        self.declare_parameter("move_rotate_speed", 0.3)

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
            self.network_interface = "lo"

        self.get_logger().info(
            "Using network interface '%s' for Unitree SDK communication."
            % self.network_interface
        )

        self.loco_client = LocoClient()
        self.loco_client.SetTimeout(client_timeout)
        self.loco_client.Init()

        self.command_subscriber = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10,
        )

        self.get_logger().info(
            "G1 locomotion node ready on topic '%s'. Send 'list', an action "
            "name, or an action id." % self.command_subscriber.topic_name
        )

    def command_callback(self, message):
        """Handle one command from ROS 2."""

        command = message.data.strip()
        if not command:
            self.get_logger().warn("Ignoring empty locomotion command.")
            return

        if command.lower() == "list":
            self.log_action_options()
            return

        action_option = self.parse_action_option(command)
        if action_option is None:
            self.get_logger().warn(
                "Unknown locomotion action '%s'. Send 'list' to see valid actions."
                % command
            )
            return

        threading.Thread(target=self.execute_action, args=(action_option,), daemon=True).start()

    def parse_action_option(self, command):
        """Convert an incoming action name or id into a LocoActionOption."""

        normalized_command = command.lower()
        action_id = self.parse_int(command)

        for option in ACTION_OPTIONS:
            if normalized_command == option.name.lower() or action_id == option.action_id:
                return option

        return None

    def execute_action(self, option):
        """Run the requested Unitree locomotion action."""

        self.get_logger().info(
            "Executing locomotion action: %s (id: %d)"
            % (option.name, option.action_id)
        )

        if option.action_id == 0:
            self.loco_client.Damp()
        elif option.action_id == 1:
            self.loco_client.Damp()
            self.create_timer(0.5, lambda: self.loco_client.Squat2StandUp())
        elif option.action_id == 2:
            self.loco_client.StandUp2Squat()
        elif option.action_id == 3:
            speed = self.get_double_parameter("move_forward_speed")
            self.loco_client.Move(speed, 0.0, 0.0)
        elif option.action_id == 4:
            speed = self.get_double_parameter("move_lateral_speed")
            self.loco_client.Move(0.0, speed, 0.0)
        elif option.action_id == 5:
            speed = self.get_double_parameter("move_rotate_speed")
            self.loco_client.Move(0.0, 0.0, speed)
        elif option.action_id == 6:
            self.loco_client.LowStand()
        elif option.action_id == 7:
            self.loco_client.HighStand()
        elif option.action_id == 8:
            self.loco_client.ZeroTorque()
        elif option.action_id == 9:
            # Unitree's wave hand without turning around.
            self.loco_client.WaveHand()
        elif option.action_id == 10:
            # Unitree's wave hand with turn-around behavior enabled.
            self.loco_client.WaveHand(True)
        elif option.action_id == 11:
            # Match Unitree's original example, which sends ShakeHand twice.
            self.loco_client.ShakeHand()
            self.create_timer(3.0, lambda: self.loco_client.ShakeHand())
        elif option.action_id == 12:
            self.get_logger().warn(
                "Lie2StandUp assumes the robot is face-up on hard, flat, rough ground."
            )
            self.loco_client.Damp()
            self.create_timer(0.5, lambda: self.loco_client.Lie2StandUp())

    def log_action_options(self):
        """Print all supported actions in a ROS-friendly format."""

        lines = ["Available G1 locomotion actions:"]
        for option in ACTION_OPTIONS:
            lines.append("  %2d: %s" % (option.action_id, option.name))
        self.get_logger().info("\n".join(lines))

    def get_double_parameter(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    @staticmethod
    def parse_int(value):
        try:
            return int(value)
        except ValueError:
            return None


def main(args=None):
    # Initialize Unitree DDS first (hardware domain 0)
    ChannelFactoryInitialize(0, "eth0")
    rclpy.init(args=args)

    node = None
    try:
        node = G1LocoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print("Failed to start G1 locomotion node: %s" % exc, file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()