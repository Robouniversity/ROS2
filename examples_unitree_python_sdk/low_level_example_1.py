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
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient


# Joint positions are expressed in radians. 0.50 rad is about 28.6 degrees.
SMALL_ARM_LIFT = 0.50

# Run the command loop every 20 ms (50 Hz). Each stage lasts five seconds.
CONTROL_DT = 0.02
STAGE_DURATION = 5.0

# Position-controller gains. Roughly, the motor command behaves like:
#     torque = KP * (desired_q - measured_q) + KD * (desired_dq - measured_dq)
# Higher values are not automatically better; excessive gains can cause shaking.
KP = 40.0
KD = 1.0


class G1JointIndex:
    """Map readable joint names to their positions in the SDK motor arrays.

    LowState.motor_state and LowCmd.motor_cmd are arrays, so the SDK expects a
    numeric slot rather than a name such as "LeftElbow". These constants make
    array access readable and help prevent commands being sent to the wrong
    motor.
    """

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

    # This is not a physical joint. Unitree reserves command slot 29 for the
    # arm SDK control weight: 1.0 gives this SDK full control and 0.0 releases
    # it. The name is retained to match common Unitree examples.
    kNotUsedJoint = 29


def clamp(value, minimum, maximum):
    """Keep value inside [minimum, maximum]."""
    return max(minimum, min(maximum, value))


def lerp(start, end, ratio):
    """Linear interpolation from start to end.

    ratio=0.0 returns start, ratio=0.5 returns the midpoint, and ratio=1.0
    returns end.
    """
    return start + (end - start) * ratio


class BasicG1UpperBodyController:
    """Four-stage low-level controller for simple G1 upper-body motion."""

    def __init__(self):
        # low_cmd is the packet repeatedly published to the robot. low_state
        # holds the most recently received feedback packet from the robot.
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.crc = CRC()

        # time_sec is logical controller time. It advances by CONTROL_DT on
        # every control-loop iteration rather than reading wall-clock time.
        self.time_sec = 0.0
        self.done = False
        self.received_first_state = False
        self.mode_machine = 0
        self.mode_machine_received = False

        # Each stage records its own start time and measured starting pose.
        # That lets interpolation begin from wherever the robot actually is.
        self.stage = 0
        self.stage_start_time = 0.0
        self.stage_start_positions = {}

        # Only these upper-body motors receive position commands. Leg joints
        # are deliberately absent and therefore untouched by this example.
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

        # Target joint-angle dictionary for stage 2. Values are radians.
        # Opposite shoulder-roll signs make the left and right arms move
        # outward symmetrically because their joint axes have opposite signs.
        # All unspecified/zero-valued joints remain at the neutral angle.
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

        # MotionSwitcher is only required on real hardware.
        if globals().get("IS_HARDWARE", False):
            print("Hardware detected. Initializing MotionSwitcher...")
            self.msc = MotionSwitcherClient()
            self.msc.SetTimeout(5.0)
            self.msc.Init()

            status, result = self.msc.CheckMode()
            while result["name"]:
                print(f"Releasing active mode: {result['name']}")
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)
        else:
            print("Simulation detected. Skipping MotionSwitcher.")

        # Publish outgoing commands on Unitree's low-level command topic.
        self.publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.publisher.Init()

        # Receive measured positions/velocities on the state topic. The final
        # argument configures the subscriber's history/queue depth.
        self.subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.subscriber.Init(self.low_state_callback, 10)

    def start(self):
        """Wait for robot state, then start the timed control loop."""
        print("Waiting for low-level robot state...")
        while not self.received_first_state:
            time.sleep(0.1)

        while not self.mode_machine_received:
            time.sleep(0.05)

        print("State received. Starting four-stage upper-body motion.")
        self.capture_stage_start(stage=1)

        # RecurrentThread invokes write_low_cmd every CONTROL_DT seconds.
        self.thread = RecurrentThread(
            interval=CONTROL_DT,
            target=self.write_low_cmd,
            name="basic_g1_upper_body_control",
        )
        self.thread.Start()

    def low_state_callback(self, msg: LowState_):
        """Store the latest robot state from DDS."""
        # DDS invokes this asynchronously whenever a new state packet arrives.
        self.low_state = msg
        self.mode_machine = msg.mode_machine
        self.mode_machine_received = True
        self.received_first_state = True

    def capture_stage_start(self, stage):
        """Record current joint positions at the beginning of a stage."""
        self.stage = stage
        self.stage_start_time = self.time_sec

        # Use measured positions, not the previous target positions. This
        # avoids an abrupt jump if a motor did not exactly reach its target.
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
        # No direct feed-forward torque is requested.
        self.low_cmd.motor_cmd[joint].mode = 1
        self.low_cmd.motor_cmd[joint].tau = 0.0

        # Ask the joint to hold desired_q with zero desired velocity.
        self.low_cmd.motor_cmd[joint].q = desired_q
        self.low_cmd.motor_cmd[joint].dq = 0.0

        # The robot's motor controller uses these proportional/derivative
        # gains to correct position error and damp velocity.
        self.low_cmd.motor_cmd[joint].kp = KP
        self.low_cmd.motor_cmd[joint].kd = KD

    def command_pose(self, target_pose, ratio):
        """Move all selected joints from their stage-start pose to target_pose."""
        for joint in self.controlled_joints:
            start_q = self.stage_start_positions[joint]

            # A missing dictionary entry means a neutral target of 0 radians.
            # Therefore passing target_pose={} moves every controlled joint
            # toward zero.
            target_q = target_pose.get(joint, 0.0)
            self.set_joint_position(joint, lerp(start_q, target_q, ratio))

    def write_low_cmd(self):
        """Control loop called by RecurrentThread."""
        if self.done or self.low_state is None:
            return

        # Convert elapsed stage time into normalized progress:
        #   0.0 = just started, 0.5 = halfway, 1.0 = complete.
        # clamp protects interpolation if the loop runs beyond the stage end.
        self.time_sec += CONTROL_DT
        elapsed = self.time_sec - self.stage_start_time
        ratio = clamp(elapsed / STAGE_DURATION, 0.0, 1.0)

        # Required on real hardware
        self.low_cmd.mode_machine = self.mode_machine
        self.low_cmd.mode_pr = 0

        # Initialize all motors
        for i in range(29):
            self.low_cmd.motor_cmd[i].mode = 1
            self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.motor_cmd[i].dq = 0.0

        # Keep arm SDK active during the moving stages.
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0

        if self.stage == 1:
            # An empty target dictionary means all controlled joints go to 0.
            self.command_pose(target_pose={}, ratio=ratio)
            if ratio >= 1.0:
                self.capture_stage_start(stage=2)

        elif self.stage == 2:
            # Interpolate from the measured neutral pose to arms_up_pose.
            self.command_pose(target_pose=self.arms_up_pose, ratio=ratio)
            if ratio >= 1.0:
                self.capture_stage_start(stage=3)

        elif self.stage == 3:
            # Interpolate back from the arm pose to the neutral pose.
            self.command_pose(target_pose={}, ratio=ratio)
            if ratio >= 1.0:
                self.capture_stage_start(stage=4)

        elif self.stage == 4:
            # Do not move the joints in stage 4. Only taper down the SDK weight.
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1.0 - ratio
            if ratio >= 1.0:
                self.done = True
                print("Completed stage 4. Motion finished.")

        # Unitree validates the command packet using its CRC checksum. Compute
        # it only after every field for this cycle has been filled.
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)

    def release_control(self):
        """Publish one command that releases the arm SDK control weight."""
        self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0.0
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.publisher.Write(self.low_cmd)


def initialize_dds_from_args():
    """Use loopback by default, or the user-provided network interface."""
    global IS_HARDWARE

    if len(sys.argv) > 1:
        interface = sys.argv[1]
        domain_id = 1 if interface == "lo" else 0
    else:
        interface = "lo"
        domain_id = 1

    IS_HARDWARE = (interface != "lo")

    print(f"Using DDS domain {domain_id} on interface '{interface}'.")
    print("Mode:", "Hardware" if IS_HARDWARE else "Simulation")

    ChannelFactoryInitialize(domain_id, interface)



def main():
    # Require a deliberate key press before any low-level command is sent.
    print("WARNING: Make sure the robot is supported and the area is clear.")
    input("Press Enter to start the basic low-level upper-body demo...")

    initialize_dds_from_args()

    controller = BasicG1UpperBodyController()
    controller.init_channels()
    controller.start()

    try:
        # The recurrent control thread performs the actual publishing. The
        # main thread stays alive until that controller reports completion.
        while not controller.done:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        controller.release_control()

    sys.exit(0)


if __name__ == "__main__":
    main()