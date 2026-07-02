#!/usr/bin/env python3
"""
Full Unitree G1 low-level upper-body dance example for simulation.

Usage:
    Simulation / loopback:
        cd /home/robot/ROS2
        python3 examples_unitree_python_sdk/low_level_example_2.py
        python3 examples_unitree_python_sdk/low_level_example_2.py lo

    Hardware protection:
        This example is intended for simulation because it contains large arm
        sweeps and waist motions. By default, it refuses hardware interfaces.
        If you intentionally want to bypass that protection, pass:
            python3 examples_unitree_python_sdk/low_level_example_2.py eth0 --allow-hardware

Use case:
    This file is a bigger staged low-level control demo for the Unitree G1 arms
    and waist. It follows the same simple pattern as low_level_example_1.py, but
    adds dance-style and simulation-only acrobatic upper-body motions.

Motion sequence:
    Stage 1: Move upper body to zero.
    Stage 2: Move to a wide T-pose.
    Stage 3: Wave both elbows while holding the T-pose.
    Stage 4: Cross both arms in front of the chest.
    Stage 5: Open from crossed arms back to neutral.
    Stage 6: Twist the waist gently while arms stay raised.
    Stage 7: Lean side to side with arms out.
    Stage 8: Move to a final dance pose.
    Stage 9: Return all selected joints to zero.
    Stage 10: Release the arm SDK control weight.

Safety notes:
    - Run this in simulation first. It is not a conservative hardware demo.
    - This script intentionally does not command the leg joints.
    - Commands are filtered and rate-limited to avoid sudden joint jumps.
    - Keep the simulator or robot area clear before running.
    - Press Ctrl+C to release control weight and stop the program.
"""

import math
import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient


PI = math.pi
CONTROL_DT = 0.02

# Softer gains and command filtering make the movement smoother.
KP = 30.0
KD = 1.5

# First-order smoothing time constant for outgoing joint commands.
COMMAND_FILTER_TAU = 0.25

# Maximum joint command change per control step, in radians.
MAX_COMMAND_STEP = 0.012


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

    # Waist joints are included for the simulation dance sequence.
    WaistYaw = 12
    WaistRoll = 13
    WaistPitch = 14

    # Unitree uses motor command slot 29 as the arm SDK control weight.
    kNotUsedJoint = 29


def clamp(value, minimum, maximum):
    """Keep value inside [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def smooth_step(ratio):
    """Smooth 0-to-1 interpolation for softer stage transitions."""
    ratio = clamp(ratio, 0.0, 1.0)
    return ratio * ratio * (3.0 - 2.0 * ratio)


def lerp(start, end, ratio):
    """Linear interpolation from start to end."""
    return start + (end - start) * ratio


class DanceG1UpperBodyController:
    """Simulation-focused staged low-level controller for G1 dance motions."""

    def __init__(self):
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

        self.time_sec = 0.0
        self.done = False
        self.received_first_state = False
        self.mode_machine = 0
        self.mode_machine_received = False

        self.stage = 0
        self.stage_start_time = 0.0
        self.stage_duration = 1.0
        self.stage_start_positions = {}
        self.commanded_positions = {}

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

        # Stage durations are intentionally slow enough to watch in simulation.
        self.stage_durations = {
            1: 5.0,
            2: 5.0,
            3: 7.0,
            4: 5.0,
            5: 5.0,
            6: 7.0,
            7: 7.0,
            8: 5.0,
            9: 5.0,
            10: 3.0,
        }

    def init_channels(self):
        """Create DDS publisher and subscriber."""
        if globals().get("IS_HARDWARE", False):
            print("Hardware detected. Initializing MotionSwitcher...")
            self.msc=MotionSwitcherClient()
            self.msc.SetTimeout(5.0)
            self.msc.Init()
            status,result=self.msc.CheckMode()
            while result["name"]:
                self.msc.ReleaseMode()
                status,result=self.msc.CheckMode()
                time.sleep(1)
        else:
            print("Simulation detected. Skipping MotionSwitcher.")

        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()

        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.low_state_callback, 10)

    def start(self):
        """Wait for robot state, then start the timed control loop."""
        print("Waiting for low-level robot state...")
        while not self.received_first_state:
            time.sleep(0.1)

        while not self.mode_machine_received:
            time.sleep(0.05)

        print("State received. Starting simulation dance sequence.")
        self.capture_stage_start(stage=1)

        self.thread = RecurrentThread(
            interval=CONTROL_DT,
            target=self.write_low_cmd,
            name="dance_g1_upper_body_control",
        )
        self.thread.Start()

    def low_state_callback(self, msg: LowState_):
        """Store the latest robot state from DDS."""
        self.low_state = msg
        self.mode_machine = msg.mode_machine
        self.mode_machine_received = True
        self.received_first_state = True

    def capture_stage_start(self, stage):
        """Record measured positions at the beginning of a stage."""
        self.stage = stage
        self.stage_start_time = self.time_sec
        self.stage_duration = self.stage_durations[stage]
        self.stage_start_positions = {
            joint: self.low_state.motor_state[joint].q
            for joint in self.controlled_joints
        }

        # Initialize command memory from measured state once. Keeping this
        # memory across stages prevents jumps when the target pose changes.
        if not self.commanded_positions:
            self.commanded_positions = dict(self.stage_start_positions)

        messages = {
            1: "Stage 1: moving upper body to zero.",
            2: "Stage 2: moving to wide T-pose.",
            3: "Stage 3: elbow wave dance.",
            4: "Stage 4: crossing arms.",
            5: "Stage 5: opening arms back to neutral.",
            6: "Stage 6: waist twist with arms raised.",
            7: "Stage 7: side lean with arms out.",
            8: "Stage 8: final dance pose.",
            9: "Stage 9: returning to zero.",
            10: "Stage 10: releasing arm SDK control weight.",
        }
        print(messages[stage])

    def set_joint_position(self, joint, desired_q):
        """Fill one motor command with position-control values."""
        current_cmd = self.commanded_positions.get(
            joint,
            self.low_state.motor_state[joint].q,
        )
        alpha = clamp(CONTROL_DT / COMMAND_FILTER_TAU, 0.0, 1.0)
        filtered_q = lerp(current_cmd, desired_q, alpha)
        command_step = clamp(
            filtered_q - current_cmd,
            -MAX_COMMAND_STEP,
            MAX_COMMAND_STEP,
        )
        smooth_q = current_cmd + command_step

        self.commanded_positions[joint] = smooth_q

        self.low_cmd.motor_cmd[joint].mode = 1
        self.low_cmd.motor_cmd[joint].tau = 0.0
        self.low_cmd.motor_cmd[joint].q = smooth_q
        self.low_cmd.motor_cmd[joint].dq = 0.0
        self.low_cmd.motor_cmd[joint].kp = KP
        self.low_cmd.motor_cmd[joint].kd = KD

    def zero_pose(self):
        """A neutral upper-body pose for all selected joints."""
        return {}

    def t_pose(self):
        """Wide arms-out pose used as a base for several dance stages."""
        return {
            G1JointIndex.LeftShoulderRoll: 1.05,
            G1JointIndex.RightShoulderRoll: -1.05,
            G1JointIndex.LeftElbow: 0.25,
            G1JointIndex.RightElbow: 0.25,
        }

    def crossed_pose(self):
        """Arms crossed in front of the body."""
        return {
            G1JointIndex.LeftShoulderYaw: 0.45,
            G1JointIndex.RightShoulderYaw: -0.45,
            G1JointIndex.LeftElbow: -0.80,
            G1JointIndex.RightElbow: -0.80,
            G1JointIndex.LeftWristRoll: 0.35,
            G1JointIndex.RightWristRoll: -0.35,
        }

    def final_pose(self):
        """Asymmetric final pose to make the simulation sequence feel complete."""
        return {
            G1JointIndex.LeftShoulderPitch: 0.45,
            G1JointIndex.LeftShoulderRoll: 0.80,
            G1JointIndex.LeftElbow: 0.70,
            G1JointIndex.RightShoulderPitch: -0.20,
            G1JointIndex.RightShoulderRoll: -0.65,
            G1JointIndex.RightElbow: 0.45,
            G1JointIndex.WaistYaw: 0.25,
        }

    def command_pose_from_stage_start(self, target_pose, ratio):
        """Move all selected joints from stage-start pose to target pose."""
        ratio = smooth_step(ratio)
        for joint in self.controlled_joints:
            start_q = self.stage_start_positions[joint]
            target_q = target_pose.get(joint, 0.0)
            self.set_joint_position(joint, lerp(start_q, target_q, ratio))

    def command_dynamic_pose(self, target_pose, ratio):
        """Blend into a changing target pose during dynamic dance stages."""
        ratio = smooth_step(ratio)
        for joint in self.controlled_joints:
            current_q = self.low_state.motor_state[joint].q
            target_q = target_pose.get(joint, 0.0)
            self.set_joint_position(joint, lerp(current_q, target_q, ratio))

    def wave_pose(self, elapsed):
        """T-pose with both elbows waving in opposite phases."""
        phase = 2.0 * PI * elapsed / self.stage_duration
        pose = self.t_pose()
        pose[G1JointIndex.LeftElbow] = 0.45 + 0.35 * math.sin(phase)
        pose[G1JointIndex.RightElbow] = 0.45 - 0.35 * math.sin(phase)
        pose[G1JointIndex.LeftWristRoll] = 0.35 * math.sin(phase)
        pose[G1JointIndex.RightWristRoll] = -0.35 * math.sin(phase)
        return pose

    def waist_twist_pose(self, elapsed):
        """Raised arms with a gentle waist yaw twist."""
        phase = 2.0 * PI * elapsed / self.stage_duration
        return {
            G1JointIndex.LeftShoulderPitch: 0.70,
            G1JointIndex.RightShoulderPitch: 0.70,
            G1JointIndex.LeftShoulderRoll: 0.35,
            G1JointIndex.RightShoulderRoll: -0.35,
            G1JointIndex.LeftElbow: 0.35,
            G1JointIndex.RightElbow: 0.35,
            G1JointIndex.WaistYaw: 0.35 * math.sin(phase),
        }

    def side_lean_pose(self, elapsed):
        """Arms-out pose with a simulated side-to-side waist lean."""
        phase = 2.0 * PI * elapsed / self.stage_duration
        pose = self.t_pose()
        pose[G1JointIndex.WaistRoll] = 0.20 * math.sin(phase)
        pose[G1JointIndex.WaistYaw] = 0.15 * math.sin(phase + PI / 2.0)
        return pose

    def write_low_cmd(self):
        """Control loop called by RecurrentThread."""
        if self.done or self.low_state is None:
            return

        self.time_sec += CONTROL_DT
        elapsed = self.time_sec - self.stage_start_time
        ratio = clamp(elapsed / self.stage_duration, 0.0, 1.0)

        self.low_cmd.mode_machine=self.mode_machine
        self.low_cmd.mode_pr=0
        for i in range(29):
            self.low_cmd.motor_cmd[i].mode=1
            self.low_cmd.motor_cmd[i].tau=0.0
            self.low_cmd.motor_cmd[i].dq=0.0

        # Keep arm SDK active during all motion stages.
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        if self.stage == 1:
            self.command_pose_from_stage_start(self.zero_pose(), ratio)
        elif self.stage == 2:
            self.command_pose_from_stage_start(self.t_pose(), ratio)
        elif self.stage == 3:
            self.command_dynamic_pose(self.wave_pose(elapsed), ratio)
        elif self.stage == 4:
            self.command_pose_from_stage_start(self.crossed_pose(), ratio)
        elif self.stage == 5:
            self.command_pose_from_stage_start(self.zero_pose(), ratio)
        elif self.stage == 6:
            self.command_dynamic_pose(self.waist_twist_pose(elapsed), ratio)
        elif self.stage == 7:
            self.command_dynamic_pose(self.side_lean_pose(elapsed), ratio)
        elif self.stage == 8:
            self.command_pose_from_stage_start(self.final_pose(), ratio)
        elif self.stage == 9:
            self.command_pose_from_stage_start(self.zero_pose(), ratio)
        elif self.stage == 10:
            # No joint motion here. Only taper down the SDK control weight.
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0 - ratio

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)

        if ratio >= 1.0:
            if self.stage < 10:
                self.capture_stage_start(stage=self.stage + 1)
            else:
                self.done = True
                print("Dance sequence complete.")

    def release_control(self):
        """Publish one command that releases the arm SDK control weight."""
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)


def initialize_dds_from_args():
    """Automatically detect simulation or hardware from the network interface."""
    global IS_HARDWARE

    interfaces = [arg for arg in sys.argv[1:] if not arg.startswith("--")]

    if interfaces:
        interface = interfaces[0]
    else:
        interface = "lo"

    IS_HARDWARE = (interface != "lo")
    domain_id = 1 if interface == "lo" else 0

    print(f"Using DDS domain {domain_id} on interface '{interface}'.")
    print("Mode:", "Hardware" if IS_HARDWARE else "Simulation")

    ChannelFactoryInitialize(domain_id, interface)


def main():
    print("WARNING: This is a simulation-focused dance/acrobatic upper-body demo.")
    print("It uses larger arm and waist motions than low_level_example_1.py.")
    input("Press Enter to start the low-level simulation dance demo...")

    initialize_dds_from_args()

    controller = DanceG1UpperBodyController()
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