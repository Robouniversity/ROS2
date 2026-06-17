#!/usr/bin/env python3

import time

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

# ==========================================================
# Configuration
# ==========================================================
DOMAIN_ID = 1
INTERFACE = "lo"

NUM_MOTORS = 29        # G1-29DOF
# NUM_MOTORS = 23      # G1-23DOF
# ==========================================================

low_state = None


def lowstate_callback(msg: LowState_):
    global low_state
    low_state = msg


ChannelFactoryInitialize(DOMAIN_ID, INTERFACE)

sub = ChannelSubscriber("rt/lowstate", LowState_)
sub.Init(lowstate_callback, 10)

print("Waiting for robot state...")

while low_state is None:
    time.sleep(0.1)

print("Connected!")

while True:

    print("\n" + "=" * 80)

    # -------------------------------------------------------
    # IMU
    # -------------------------------------------------------
    imu = low_state.imu_state

    print("IMU")
    print("  Quaternion :", imu.quaternion)
    print("  RPY        :", imu.rpy)
    print("  Gyroscope  :", imu.gyroscope)
    print("  Accelerom. :", imu.accelerometer)

    # -------------------------------------------------------
    # Battery
    # -------------------------------------------------------
    try:
        print("\nBattery")
        print("  Voltage :", low_state.power_v)
        print("  Current :", low_state.power_a)
    except:
        pass

    # -------------------------------------------------------
    # Foot forces
    # -------------------------------------------------------
    try:
        print("\nFoot Force")
        print("  Left  :", low_state.foot_force[0])
        print("  Right :", low_state.foot_force[1])
    except:
        pass

    # -------------------------------------------------------
    # Remote controller
    # -------------------------------------------------------
    try:
        print("\nWireless Remote")
        print(low_state.wireless_remote)
    except:
        pass

    # -------------------------------------------------------
    # Motor states
    # -------------------------------------------------------
    print("\nMotors")
    print("Idx     q(rad)      dq(rad/s)     tau(Nm)      Temp")

    for i in range(NUM_MOTORS):

        m = low_state.motor_state[i]

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
