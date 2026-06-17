#!/usr/bin/env python3
"""
Basic Unitree G1 low-level state reader.

Usage:
    Simulation / loopback:
        cd /home/robot/ROS2
        python3 examples_unitree_python_sdk/read_robot_state.py
        python3 examples_unitree_python_sdk/read_robot_state.py lo

    Real robot:
        cd /home/robot/ROS2
        python3 examples_unitree_python_sdk/read_robot_state.py <network_interface>
        Example:
            python3 examples_unitree_python_sdk/read_robot_state.py eth0

Use case:
    This file subscribes to the Unitree low-level state topic and prints useful
    robot information once per second. It is meant for checking DDS connection,
    reading IMU values, checking battery data, viewing foot-force readings, and
    inspecting each motor position / velocity / estimated torque.

Safety notes:
    - This script only reads robot state; it does not send movement commands.
    - Use it before running low-level control examples to confirm the robot or
      simulator is publishing valid state data.
    - Press Ctrl+C to stop printing.
"""

import sys
import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

# ==========================================================
# Configuration
# ==========================================================
# Select the number of motors for your G1 model.
NUM_MOTORS = 29        # G1-29DOF
# NUM_MOTORS = 23      # G1-23DOF
# ==========================================================

# Latest low-level state message received from DDS.
low_state = None


def lowstate_callback(msg: LowState_):
    """Store the latest low-level state message."""
    global low_state
    low_state = msg


def initialize_dds_from_args():
    """Use the command-line interface argument, or loopback by default."""
    if len(sys.argv) > 1:
        interface = sys.argv[1]
        domain_id = 1 if interface == "lo" else 0
    else:
        interface = "lo"
        domain_id = 1

    print(f"Using DDS domain {domain_id} on interface '{interface}'.")
    ChannelFactoryInitialize(domain_id, interface)


# Initialize DDS communication before creating the subscriber.
initialize_dds_from_args()

# Subscribe to the low-level robot state topic.
sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(lowstate_callback, 10)

print("Waiting for robot state...")

# Wait until at least one low-state packet arrives.
while low_state is None:
    time.sleep(0.1)

print("Connected!")

while True:

    print("\n" + "=" * 80)

    # -------------------------------------------------------
    # IMU data: orientation, angular velocity, and acceleration.
    # -------------------------------------------------------
    imu = low_state.imu_state

    print("IMU")
    print("  Quaternion :", imu.quaternion)
    print("  RPY        :", imu.rpy)
    print("  Gyroscope  :", imu.gyroscope)
    print("  Accelerom. :", imu.accelerometer)

    # -------------------------------------------------------
    # Battery data. Some simulators or message versions may not provide it.
    # -------------------------------------------------------
    try:
        print("\nBattery")
        print("  Voltage :", low_state.power_v)
        print("  Current :", low_state.power_a)
    except:
        pass

    # -------------------------------------------------------
    # Foot-force readings. Some setups may not publish these values.
    # -------------------------------------------------------
    try:
        print("\nFoot Force")
        print("  Left  :", low_state.foot_force[0])
        print("  Right :", low_state.foot_force[1])
    except:
        pass

    # -------------------------------------------------------
    # Raw wireless remote data, useful for checking controller connection.
    # -------------------------------------------------------
    try:
        print("\nWireless Remote")
        print(low_state.wireless_remote)
    except:
        pass

    # -------------------------------------------------------
    # Motor states for each joint:
    # q       = joint position in radians
    # dq      = joint velocity in radians / second
    # tau_est = estimated joint torque in Nm
    # -------------------------------------------------------
    print("\nMotors")
    print("Idx     q(rad)      dq(rad/s)     tau(Nm)      Temp")

    for i in range(NUM_MOTORS):

        m = low_state.motor_state[i]

        # Temperature is not available in every message version.
        try:
            temp = m.temperature
        except:
            temp = "N/A"

        print(
            f"{i:02d}   "
            f"{m.q:8.3f}   "
            f"{m.dq:8.3f}   "
            f"{m.tau_est:8.3f}   "
            f"{temp}"
        )

    time.sleep(1)
