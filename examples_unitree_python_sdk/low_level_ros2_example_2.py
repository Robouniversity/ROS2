#!/usr/bin/env python3
"""
ROS 2 Python version of Unitree's G1 low-level command example.

The node subscribes to G1 low state, publishes low-level motor commands, moves
the robot toward zero posture for three seconds, then cycles through three
simple smooth motions: lift both arms, gentle double-arm wave, and small squat.

Safety:
    Test in simulation first. Low-level commands can move the robot directly.
"""

import math
import struct
from enum import IntEnum

import rclpy
from rclpy.node import Node

from unitree_hg.msg import LowCmd, LowState


# Enable these only while debugging. State callbacks can run hundreds of times
# per second, so continuous logging may interfere with controller timing.
INFO_IMU = False
INFO_MOTOR = False

# Select Unitree's high-frequency low-state topic.
HIGH_FREQ = True

# The G1 exposes 29 joints, but LowCmd reserves 35 motor command slots.
G1_NUM_MOTOR = 29
HG_MOTOR_SLOT_COUNT = 35

# The control timer runs at 500 Hz. Keep these values equal because CONTROL_DT
# is also used as the controller's elapsed-time increment.
CONTROL_DT = 0.002
TIMER_PERIOD_SEC = CONTROL_DT

# Startup and trajectory-shaping parameters, expressed in seconds/radians.
MOVE_TO_ZERO_DURATION = 3.0
SMOOTHING_TAU = 0.30
COMMAND_LIMIT = 1.2
MOTION_FADE_TIME = 1.5


class MotionState(IntEnum):
    """Named phases in the repeating demonstration choreography."""

    STAND = 0
    LIFT_BOTH_ARMS = 1
    BOTH_ARM_WAVE = 2
    SMALL_SQUAT = 3


# Each tuple contains a motion and the number of seconds to perform it.
# Once SMALL_SQUAT finishes, the sequence wraps back to STAND.
MOTION_SEQUENCE = (
    (MotionState.STAND, 2.0),
    (MotionState.LIFT_BOTH_ARMS, 6.0),
    (MotionState.BOTH_ARM_WAVE, 7.0),
    (MotionState.SMALL_SQUAT, 6.0),
)


class PRorAB(IntEnum):
    """Low-level actuator control mode values expected by Unitree."""

    PR = 0
    AB = 1


class G1JointIndex(IntEnum):
    """Index of each physical joint in the motor state/command arrays."""

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
    """Restrict value to the inclusive [low, high] interval."""
    return max(low, min(value, high))


def smoothstep(value):
    """Return an eased 0-to-1 interpolation with zero slope at both ends."""
    value = clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def fade_envelope(time_in_motion, duration, fade_time):
    """Fade a motion smoothly in and out over its allotted duration."""

    # Limiting the fade to half the duration prevents fade-in and fade-out from
    # crossing in an invalid order for very short motions.
    fade_time = min(fade_time, duration * 0.5)
    fade_in = smoothstep(time_in_motion / fade_time)
    fade_out = smoothstep((duration - time_in_motion) / fade_time)

    # The smaller envelope controls the amplitude near either endpoint.
    return clamp(min(fade_in, fade_out), 0.0, 1.0)


def crc32_core(words):
    """Calculate Unitree's CRC over an iterable of 32-bit words."""
    crc = 0xFFFFFFFF
    poly = 0x04C11DB7

    for data in words:
        data &= 0xFFFFFFFF
        # Process each word from its most-significant bit to its least-
        # significant bit to match the robot firmware.
        for bit_index in range(32):
            bit = ((crc >> 31) ^ (data >> (31 - bit_index))) & 0x1
            crc = (crc << 1) & 0xFFFFFFFF
            if bit:
                crc ^= poly

    return crc & 0xFFFFFFFF


def fill_crc(msg):
    """Match Unitree's packed LowCmd CRC layout used by the ROS 2 C++ example."""
    packed = bytearray()

    # Little-endian encoding and the two padding bytes mirror the fixed binary
    # layout used by the corresponding C++ message.
    packed += struct.pack("<BB2x", int(msg.mode_pr), int(msg.mode_machine))

    # Include every reserved motor slot in the checksum, even though only the
    # first 29 correspond to physical G1 joints.
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

    # Insert zero in the CRC field while calculating the checksum.
    packed += struct.pack("<4I", *[int(value) for value in msg.reserve[:4]])
    packed += struct.pack("<I", 0)

    # crc32_core operates on complete 32-bit words.
    padding = (-len(packed)) % 4
    if padding:
        packed += b"\x00" * padding

    # Exclude the final placeholder word from the CRC input.
    word_count_without_crc = (len(packed) // 4) - 1
    words = struct.unpack(f"<{len(packed) // 4}I", packed)
    msg.crc = crc32_core(words[:word_count_without_crc])


class LowLevelCmdSender(Node):
    def __init__(self):
        super().__init__("low_level_cmd_sender")

        # The high-frequency topic is appropriate for this 500 Hz controller.
        lowstate_topic = "lowstate" if HIGH_FREQ else "lf/lowstate"
        self.lowstate_subscriber = self.create_subscription(
            LowState,
            lowstate_topic,
            self.low_state_handler,
            10,
        )
        self.lowcmd_publisher = self.create_publisher(LowCmd, "lowcmd", 10)
        self.timer = self.create_timer(TIMER_PERIOD_SEC, self.control)

        # Reuse a single command message to avoid allocating one every 2 ms.
        self.low_command = LowCmd()
        self.motor = []
        self.imu = None

        # self.time includes the initial move-to-zero phase. motion_time begins
        # at zero only after that startup phase has finished.
        self.time = 0.0
        self.motion_time = 0.0
        self.state_start_time = 0.0
        self.mode = PRorAB.PR
        self.mode_machine = 0
        self.received_state = False
        self.initialized_pose = False

        # Initialize the choreography at the first entry in MOTION_SEQUENCE.
        self.motion_index = 0
        self.motion_state = MOTION_SEQUENCE[self.motion_index][0]

        # q is measured position, q_initial is the startup snapshot, q_target is
        # the desired pose, and q_smooth is the filtered command sent to motors.
        self.q = [0.0] * G1_NUM_MOTOR
        self.q_initial = [0.0] * G1_NUM_MOTOR
        self.q_target = [0.0] * G1_NUM_MOTOR
        self.q_smooth = [0.0] * G1_NUM_MOTOR

        self.get_logger().info(
            "low_level_cmd_sender ready. Waiting for low state, moving to "
            "zero posture, then starting smooth demo choreography."
        )

    def control(self):
        """Advance the choreography and publish one low-level motor command."""

        # Publishing is gated until current joint positions and machine mode are
        # available from at least one low-state message.
        if not self.received_state:
            return

        # Capture the robot's pose once. This fixed snapshot is the starting
        # point for the smooth transition to the all-zero posture.
        if not self.initialized_pose:
            self.q_initial = self.q.copy()
            self.q_target = self.q.copy()
            self.q_smooth = self.q.copy()
            self.initialized_pose = True
            self.get_logger().info("Low state received. Easing to zero posture.")

        self.time += CONTROL_DT
        self.low_command.mode_pr = int(self.mode)
        self.low_command.mode_machine = int(self.mode_machine)

        if self.time < MOVE_TO_ZERO_DURATION:
            # Smoothstep avoids an abrupt velocity change at the beginning and
            # end of the three-second move to zero.
            ratio = clamp(self.time / MOVE_TO_ZERO_DURATION, 0.0, 1.0)
            ratio = smoothstep(ratio)
            for index in range(G1_NUM_MOTOR):
                self.q_target[index] = (1.0 - ratio) * self.q_initial[index]
        else:
            # Choreography time deliberately excludes the startup transition.
            self.motion_time = self.time - MOVE_TO_ZERO_DURATION
            self.update_motion_state()
            self.build_motion_target()

        # Apply an additional low-pass filter, convert targets into motor command
        # fields, then attach the checksum required by the receiver.
        self.smooth_commands()
        self.write_motor_commands()
        fill_crc(self.low_command)
        self.lowcmd_publisher.publish(self.low_command)

    def update_motion_state(self):
        """Advance to the next choreography state when its duration expires."""
        _, duration = MOTION_SEQUENCE[self.motion_index]
        if self.motion_time - self.state_start_time <= duration:
            return

        # Modulo makes the last state wrap around to the first state.
        self.motion_index = (self.motion_index + 1) % len(MOTION_SEQUENCE)
        self.motion_state = MOTION_SEQUENCE[self.motion_index][0]
        self.state_start_time = self.motion_time
        self.get_logger().info(f"Motion -> {self.motion_state.name}")

    def build_motion_target(self):
        """Build the desired joint pose for the active choreography state."""

        # Clear targets first so joints unused by the current motion return to
        # their neutral zero position instead of retaining an older target.
        self.stand()

        state_time = self.motion_time - self.state_start_time
        duration = MOTION_SEQUENCE[self.motion_index][1]

        # amount ramps from 0 to 1 and back to 0 near each state's boundaries.
        amount = fade_envelope(state_time, duration, MOTION_FADE_TIME)

        if self.motion_state == MotionState.STAND:
            return
        if self.motion_state == MotionState.LIFT_BOTH_ARMS:
            self.lift_both_arms(amount)
        elif self.motion_state == MotionState.BOTH_ARM_WAVE:
            self.both_arm_wave(state_time, amount)
        elif self.motion_state == MotionState.SMALL_SQUAT:
            self.small_squat(amount)

    def stand(self):
        """Set the neutral target pose: every joint at zero radians."""
        for index in range(G1_NUM_MOTOR):
            self.q_target[index] = 0.0

    def lift_both_arms(self, amount):
        """Raise both arms symmetrically, scaled by the fade envelope."""

        # Shoulder-roll signs are opposite because the joints are mirrored.
        self.q_target[G1JointIndex.LEFT_SHOULDER_PITCH] = 0.15 * amount
        self.q_target[G1JointIndex.LEFT_SHOULDER_ROLL] = 0.55 * amount
        self.q_target[G1JointIndex.LEFT_ELBOW] = 0.45 * amount
        self.q_target[G1JointIndex.RIGHT_SHOULDER_PITCH] = 0.15 * amount
        self.q_target[G1JointIndex.RIGHT_SHOULDER_ROLL] = -0.55 * amount
        self.q_target[G1JointIndex.RIGHT_ELBOW] = 0.45 * amount

    def both_arm_wave(self, state_time, amount):
        """Generate a gentle 0.45 Hz mirrored wave with both arms."""
        wave = math.sin(2.0 * math.pi * 0.45 * state_time)

        # A fixed raised-arm pose is combined with small sinusoidal offsets.
        self.q_target[G1JointIndex.LEFT_SHOULDER_PITCH] = 0.10 * amount
        self.q_target[G1JointIndex.LEFT_SHOULDER_ROLL] = (0.45 + 0.10 * wave) * amount
        self.q_target[G1JointIndex.LEFT_ELBOW] = (0.45 + 0.12 * wave) * amount
        self.q_target[G1JointIndex.LEFT_WRIST_ROLL] = 0.20 * wave * amount
        self.q_target[G1JointIndex.RIGHT_SHOULDER_PITCH] = 0.10 * amount
        self.q_target[G1JointIndex.RIGHT_SHOULDER_ROLL] = (-0.45 + 0.10 * wave) * amount
        self.q_target[G1JointIndex.RIGHT_ELBOW] = (0.45 - 0.12 * wave) * amount
        self.q_target[G1JointIndex.RIGHT_WRIST_ROLL] = -0.20 * wave * amount

    def small_squat(self, amount):
        """Bend both legs slightly while preserving left/right symmetry."""

        # Hip, knee, and ankle targets work together to form the squat pose.
        self.q_target[G1JointIndex.LEFT_HIP_PITCH] = -0.10 * amount
        self.q_target[G1JointIndex.RIGHT_HIP_PITCH] = -0.10 * amount
        self.q_target[G1JointIndex.LEFT_KNEE] = -0.28 * amount
        self.q_target[G1JointIndex.RIGHT_KNEE] = -0.28 * amount
        self.q_target[G1JointIndex.LEFT_ANKLE_PITCH] = 0.10 * amount
        self.q_target[G1JointIndex.RIGHT_ANKLE_PITCH] = 0.10 * amount

    def smooth_commands(self):
        """Low-pass filter target positions and enforce a command limit."""

        # At each tick, move q_smooth a fraction alpha toward q_target. This
        # suppresses discontinuities when a target or motion state changes.
        alpha = clamp(CONTROL_DT / SMOOTHING_TAU, 0.0, 1.0)
        for index in range(G1_NUM_MOTOR):
            self.q_smooth[index] += alpha * (self.q_target[index] - self.q_smooth[index])

            # Apply a final per-joint angular safety bound before publication.
            self.q_smooth[index] = clamp(self.q_smooth[index], -COMMAND_LIMIT, COMMAND_LIMIT)

    def write_motor_commands(self):
        """Copy filtered joint targets and gains into the LowCmd message."""
        for index in range(G1_NUM_MOTOR):
            motor_cmd = self.low_command.motor_cmd[index]

            # Position-control mode with zero feed-forward velocity and torque.
            motor_cmd.mode = 1
            motor_cmd.q = float(self.q_smooth[index])
            motor_cmd.dq = 0.0
            motor_cmd.tau = 0.0
            motor_cmd.kp = self.kp_for_joint(index)
            motor_cmd.kd = 1.0

    @staticmethod
    def kp_for_joint(index):
        """Choose position stiffness according to the joint group."""

        # Ankles receive a dedicated gain between the leg and upper-body gains.
        if index in (
            G1JointIndex.LEFT_ANKLE_PITCH,
            G1JointIndex.LEFT_ANKLE_ROLL,
            G1JointIndex.RIGHT_ANKLE_PITCH,
            G1JointIndex.RIGHT_ANKLE_ROLL,
        ):
            return 80.0
        return 100.0 if index < 13 else 50.0

    def low_state_handler(self, message):
        """Cache the latest machine, IMU, and measured motor state."""
        self.received_state = True
        self.mode_machine = message.mode_machine
        self.imu = message.imu_state
        self.motor = list(message.motor_state[:G1_NUM_MOTOR])

        # Keep a compact array of measured joint angles for initialization.
        for index, motor_state in enumerate(self.motor):
            self.q[index] = motor_state.q

        # Diagnostics are optional because printing at state frequency is noisy
        # and may introduce timing jitter.
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
    """Initialize ROS 2, spin the controller, and shut down cleanly."""
    rclpy.init(args=args)
    node = LowLevelCmdSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C is the normal way to stop this continuously running example.
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
