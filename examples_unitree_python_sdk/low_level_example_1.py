#!/usr/bin/env python3
"""
Basic Unitree G1 low-level upper-body control example.

Usage:
    Simulation / loopback:
        python3 low_level_example_1.py
        python3 low_level_example_1.py lo

    Real robot:
        python3 low_level_example_1.py <network_interface>
        Example: python3 low_level_example_1.py eth0

Use case:
    This file is a simple, conservative low-level control demo for the Unitree
    G1 arms and waist. It uses the same staged idea as g1_move.py, but only runs
    up to stage 4 and removes the dance / acrobatic stages.

Motion sequence:
    Stage 1: Slowly move selected upper-body joints to zero.
    Stage 2: Slowly lift both arms to a small demonstration pose.
    Stage 3: Slowly return the arms and waist to zero.
    Stage 4: Slowly release the arm SDK control weight.

Safety notes:
    - Test in simulation first.
    - Keep the robot supported and the area around it clear.
    - This script intentionally does not command the leg joints.
    - Press Ctrl+C to stop the program if anything looks wrong.
"""

import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread


SMALL_ARM_LIFT = 0.50

CONTROL_DT = 0.02
STAGE_DURATION = 5.0

KP = 40.0
KD = 1.0


class G1JointIndex:
    """Motor indices for the Unitree G1 29-DOF model."""

    # Left arm.
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20
    LeftWristYaw = 21

    # Right arm.
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27
    RightWristYaw = 28

    # Waist joints are included for compatibility with the original example.
    # On some G1 configurations waist roll / pitch are locked or unavailable.
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    # Unitree uses motor command slot 29 as the arm SDK control weight.
    kNotUsedJoint = 29


def clamp(value, minimum, maximum):
    """Keep value inside [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def lerp(start, end, ratio):
    """Linear interpolation from start to end."""
    return start + (end - start) * ratio


class BasicG1UpperBodyController:
    """Four-stage low-level controller for simple G1 upper-body motion."""

    def __init__(self):
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

        self.time_sec = 0.0
        self.done = False
        self.received_first_state = False

        self.stage = 0
        self.stage_start_time = 0.0
        self.stage_start_positions = {}

        self.controlled_joints = [
            G1JointIndex.LeftShoulderPitch,
            G1JointIndex.LeftShoulderRoll,
            G1JointIndex.LeftShoulderYaw,
            G1JointIndex.LeftElbow,
            G1JointIndex.LeftWristRoll,
            G1JointIndex.LeftWristPitch,
            G1JointIndex.LeftWristYaw,
            G1JointIndex.RightShoulderPitch,
            G1JointIndex.RightShoulderRoll,
            G1JointIndex.RightShoulderYaw,
            G1JointIndex.RightElbow,
            G1JointIndex.RightWristRoll,
            G1JointIndex.RightWristPitch,
            G1JointIndex.RightWristYaw,
            G1JointIndex.WaistYaw,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistPitch,
        ]

        # Small arm pose for stage 2.
        # Keep the lift low to reduce body shaking when returning to zero.
        self.arms_up_pose = {
            G1JointIndex.LeftShoulderPitch: 0.0,
            G1JointIndex.LeftShoulderRoll: SMALL_ARM_LIFT,
            G1JointIndex.LeftShoulderYaw: 0.0,
            G1JointIndex.LeftElbow: SMALL_ARM_LIFT,
            G1JointIndex.LeftWristRoll: 0.0,
            G1JointIndex.LeftWristPitch: 0.0,
            G1JointIndex.LeftWristYaw: 0.0,
            G1JointIndex.RightShoulderPitch: 0.0,
            G1JointIndex.RightShoulderRoll: -SMALL_ARM_LIFT,
            G1JointIndex.RightShoulderYaw: 0.0,
            G1JointIndex.RightElbow: SMALL_ARM_LIFT,
            G1JointIndex.RightWristRoll: 0.0,
            G1JointIndex.RightWristPitch: 0.0,
            G1JointIndex.RightWristYaw: 0.0,
            G1JointIndex.WaistYaw: 0.0,
            G1JointIndex.WaistRoll: 0.0,
            G1JointIndex.WaistPitch: 0.0,
        }

    def init_channels(self):
        """Create DDS publisher and subscriber."""
        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.low_state_callback, 10)

    def start(self):
        """Wait for robot state, then start the timed control loop."""
        print("Waiting for low-level robot state...")
        while not self.received_first_state:
            time.sleep(0.1)

        print("State received. Starting four-stage upper-body motion.")
        self.capture_stage_start(stage=1)

        self.thread = RecurrentThread(
            interval=CONTROL_DT,
            target=self.write_low_cmd,
            name="basic_g1_upper_body_control",
        )
        self.thread.Start()

    def low_state_callback(self, msg: LowState_):
        """Store the latest robot state from DDS."""
        self.low_state = msg
        self.received_first_state = True

    def capture_stage_start(self, stage):
        """Record current joint positions at the beginning of a stage."""
        self.stage = stage
        self.stage_start_time = self.time_sec
        self.stage_start_positions = {
            joint: self.low_state.motor_state[joint].q
            for joint in self.controlled_joints
        }

        if stage == 1:
            print("Stage 1: moving upper-body joints to zero.")
        elif stage == 2:
            print("Stage 2: lifting both arms slowly to a small pose.")
        elif stage == 3:
            print("Stage 3: returning upper body to zero.")
        elif stage == 4:
            print("Stage 4: releasing arm SDK control weight.")

    def set_joint_position(self, joint, desired_q):
        """Fill one motor command with simple position-control values."""
        self.low_cmd.motor_cmd[joint].tau = 0.0
        self.low_cmd.motor_cmd[joint].q = desired_q
        self.low_cmd.motor_cmd[joint].dq = 0.0
        self.low_cmd.motor_cmd[joint].kp = KP
        self.low_cmd.motor_cmd[joint].kd = KD

    def command_pose(self, target_pose, ratio):
        """Move all selected joints from their stage-start pose to target_pose."""
        for joint in self.controlled_joints:
            start_q = self.stage_start_positions[joint]
            target_q = target_pose.get(joint, 0.0)
            self.set_joint_position(joint, lerp(start_q, target_q, ratio))

    def write_low_cmd(self):
        """Control loop called by RecurrentThread."""
        if self.done or self.low_state is None:
            return

        self.time_sec += CONTROL_DT
        elapsed = self.time_sec - self.stage_start_time
        ratio = clamp(elapsed / STAGE_DURATION, 0.0, 1.0)

        # Keep arm SDK active during the moving stages.
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        if self.stage == 1:
            self.command_pose(target_pose={}, ratio=ratio)
            if ratio >= 1.0:
                self.capture_stage_start(stage=2)

        elif self.stage == 2:
            self.command_pose(target_pose=self.arms_up_pose, ratio=ratio)
            if ratio >= 1.0:
                self.capture_stage_start(stage=3)

        elif self.stage == 3:
            self.command_pose(target_pose={}, ratio=ratio)
            if ratio >= 1.0:
                self.capture_stage_start(stage=4)

        elif self.stage == 4:
            # Do not move the joints in stage 4. Only taper down the SDK weight.
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0 - ratio
            if ratio >= 1.0:
                self.done = True
                print("Completed stage 4. Motion finished.")

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)

    def release_control(self):
        """Publish one command that releases the arm SDK control weight."""
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)


def initialize_dds_from_args():
    """Use loopback by default, or the user-provided network interface."""
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        domain_id = 1 if interface == "lo" else 0
    else:
        interface = "lo"
        domain_id = 1

    print(f"Using DDS domain {domain_id} on interface '{interface}'.")
    ChannelFactoryInitialize(domain_id, interface)


def main():
    print("WARNING: Make sure the robot is supported and the area is clear.")
    input("Press Enter to start the basic low-level upper-body demo...")

    initialize_dds_from_args()

    controller = BasicG1UpperBodyController()
    controller.init_channels()
    controller.start()

    try:
        while not controller.done:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        controller.release_control()

    sys.exit(0)


if __name__ == "__main__":
    main()
