#!/usr/bin/env python3
"""
ROS 2 Python version of Unitree's G1 low-level command example.

The node subscribes to G1 low state, publishes low-level motor commands, moves
the robot toward zero posture for three seconds, then swings both ankles and
wrist-roll joints with a small sinusoidal trajectory.

Safety:
    Test in simulation first. Low-level commands can move the robot directly.
"""

import math
import struct
from enum import IntEnum

import rclpy
from rclpy.node import Node

from unitree_hg.msg import LowCmd, LowState


INFO_IMU = False
INFO_MOTOR = False
HIGH_FREQ = True

G1_NUM_MOTOR = 29
HG_MOTOR_SLOT_COUNT = 35
CONTROL_DT = 0.002
TIMER_PERIOD_SEC = CONTROL_DT
MOVE_TO_ZERO_DURATION = 3.0


class PRorAB(IntEnum):
    PR = 0
    AB = 1


class G1JointIndex(IntEnum):
    LEFT_HIP_PITCH = 0
    LEFT_HIP_ROLL = 1
    LEFT_HIP_YAW = 2
    LEFT_KNEE = 3
    LEFT_ANKLE_PITCH = 4
    LEFT_ANKLE_ROLL = 5
    RIGHT_HIP_PITCH = 6
    RIGHT_HIP_ROLL = 7
    RIGHT_HIP_YAW = 8
    RIGHT_KNEE = 9
    RIGHT_ANKLE_PITCH = 10
    RIGHT_ANKLE_ROLL = 11
    WAIST_YAW = 12
    WAIST_ROLL = 13
    WAIST_PITCH = 14
    LEFT_SHOULDER_PITCH = 15
    LEFT_SHOULDER_ROLL = 16
    LEFT_SHOULDER_YAW = 17
    LEFT_ELBOW = 18
    LEFT_WRIST_ROLL = 19
    LEFT_WRIST_PITCH = 20
    LEFT_WRIST_YAW = 21
    RIGHT_SHOULDER_PITCH = 22
    RIGHT_SHOULDER_ROLL = 23
    RIGHT_SHOULDER_YAW = 24
    RIGHT_ELBOW = 25
    RIGHT_WRIST_ROLL = 26
    RIGHT_WRIST_PITCH = 27
    RIGHT_WRIST_YAW = 28


def clamp(value, low, high):
    return max(low, min(value, high))


def crc32_core(words):
    crc = 0xFFFFFFFF
    poly = 0x04C11DB7

    for data in words:
        data &= 0xFFFFFFFF
        for bit_index in range(32):
            bit = ((crc >> 31) ^ (data >> (31 - bit_index))) & 0x1
            crc = (crc << 1) & 0xFFFFFFFF
            if bit:
                crc ^= poly

    return crc & 0xFFFFFFFF


def fill_crc(msg):
    """Match Unitree's packed LowCmd CRC layout used by the ROS 2 C++ example."""
    packed = bytearray()
    packed += struct.pack("<BB2x", int(msg.mode_pr), int(msg.mode_machine))

    for motor_cmd in msg.motor_cmd[:HG_MOTOR_SLOT_COUNT]:
        packed += struct.pack(
            "<B3xfffffI",
            int(motor_cmd.mode),
            float(motor_cmd.q),
            float(motor_cmd.dq),
            float(motor_cmd.tau),
            float(motor_cmd.kp),
            float(motor_cmd.kd),
            0,
        )

    packed += struct.pack("<4I", *[int(value) for value in msg.reserve[:4]])
    packed += struct.pack("<I", 0)

    padding = (-len(packed)) % 4
    if padding:
        packed += b"\x00" * padding

    word_count_without_crc = (len(packed) // 4) - 1
    words = struct.unpack(f"<{len(packed) // 4}I", packed)
    msg.crc = crc32_core(words[:word_count_without_crc])


class LowLevelCmdSender(Node):
    def __init__(self):
        super().__init__("low_level_cmd_sender")

        lowstate_topic = "lowstate" if HIGH_FREQ else "lf/lowstate"
        self.lowstate_subscriber = self.create_subscription(
            LowState,
            lowstate_topic,
            self.low_state_handler,
            10,
        )
        self.lowcmd_publisher = self.create_publisher(LowCmd, "lowcmd", 10)
        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.control)

        self.low_command = LowCmd()
        self.motor = []
        self.imu = None
        self.time = 0.0
        self.mode = PRorAB.PR
        self.mode_machine = 0
        self.received_state = False

        self.get_logger().info(
            "low_level_cmd_sender ready. Waiting for low state, then moving "
            "to zero posture before ankle/wrist motion."
        )

    def control(self):
        if not self.received_state:
            return

        self.time += CONTROL_DT
        self.low_command.mode_pr = int(self.mode)
        self.low_command.mode_machine = int(self.mode_machine)

        for index in range(G1_NUM_MOTOR):
            motor_cmd = self.low_command.motor_cmd[index]
            motor_cmd.mode = 1
            motor_cmd.tau = 0.0
            motor_cmd.q = 0.0
            motor_cmd.dq = 0.0
            motor_cmd.kp = 100.0 if index < 13 else 50.0
            motor_cmd.kd = 1.0

        if self.time < MOVE_TO_ZERO_DURATION:
            ratio = clamp(self.time / MOVE_TO_ZERO_DURATION, 0.0, 1.0)
            for index in range(G1_NUM_MOTOR):
                self.low_command.motor_cmd[index].q = (1.0 - ratio) * self.motor[index].q
        else:
            self.mode = PRorAB.PR
            motion_time = self.time - MOVE_TO_ZERO_DURATION

            max_pitch = 0.25
            max_roll = 0.25
            left_pitch = max_pitch * math.cos(2.0 * math.pi * motion_time)
            left_roll = max_roll * math.sin(2.0 * math.pi * motion_time)
            right_pitch = max_pitch * math.cos(2.0 * math.pi * motion_time)
            right_roll = -max_roll * math.sin(2.0 * math.pi * motion_time)

            self.set_joint_command(G1JointIndex.LEFT_ANKLE_PITCH, left_pitch, 80.0, 1.0)
            self.set_joint_command(G1JointIndex.LEFT_ANKLE_ROLL, left_roll, 80.0, 1.0)
            self.set_joint_command(G1JointIndex.RIGHT_ANKLE_PITCH, right_pitch, 80.0, 1.0)
            self.set_joint_command(G1JointIndex.RIGHT_ANKLE_ROLL, right_roll, 80.0, 1.0)

            wrist_roll = 0.5 * math.sin(2.0 * math.pi * motion_time)
            self.set_joint_command(G1JointIndex.LEFT_WRIST_ROLL, wrist_roll, 50.0, 1.0)
            self.set_joint_command(G1JointIndex.RIGHT_WRIST_ROLL, wrist_roll, 50.0, 1.0)

        fill_crc(self.low_command)
        self.lowcmd_publisher.publish(self.low_command)

    def set_joint_command(self, joint_index, q, kp, kd):
        motor_cmd = self.low_command.motor_cmd[int(joint_index)]
        motor_cmd.q = float(q)
        motor_cmd.dq = 0.0
        motor_cmd.kp = float(kp)
        motor_cmd.kd = float(kd)
        motor_cmd.tau = 0.0

    def low_state_handler(self, message):
        self.received_state = True
        self.mode_machine = message.mode_machine
        self.imu = message.imu_state
        self.motor = list(message.motor_state[:G1_NUM_MOTOR])

        if INFO_IMU:
            self.get_logger().info(
                "Euler angle -- roll: %.6f; pitch: %.6f; yaw: %.6f"
                % (
                    self.imu.rpy[0],
                    self.imu.rpy[1],
                    self.imu.rpy[2],
                )
            )
            self.get_logger().info(
                "Quaternion -- qw: %.6f; qx: %.6f; qy: %.6f; qz: %.6f"
                % (
                    self.imu.quaternion[0],
                    self.imu.quaternion[1],
                    self.imu.quaternion[2],
                    self.imu.quaternion[3],
                )
            )
            self.get_logger().info(
                "Gyroscope -- wx: %.6f; wy: %.6f; wz: %.6f"
                % (
                    self.imu.gyroscope[0],
                    self.imu.gyroscope[1],
                    self.imu.gyroscope[2],
                )
            )
            self.get_logger().info(
                "Accelerometer -- ax: %.6f; ay: %.6f; az: %.6f"
                % (
                    self.imu.accelerometer[0],
                    self.imu.accelerometer[1],
                    self.imu.accelerometer[2],
                )
            )

        if INFO_MOTOR:
            for index, motor_state in enumerate(self.motor):
                self.get_logger().info(
                    "Motor state -- num: %d; q: %.6f; dq: %.6f; ddq: %.6f; tau: %.6f"
                    % (
                        index,
                        motor_state.q,
                        motor_state.dq,
                        motor_state.ddq,
                        motor_state.tau_est,
                    )
                )


def main(args=None):
    rclpy.init(args=args)
    node = LowLevelCmdSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
