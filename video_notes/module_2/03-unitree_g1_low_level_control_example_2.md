# Module Name

## Unitree G1 Ankle Swing Control using ROS 2  
### Understanding `g1_ankle_swing_example.cpp`

---

# Introduction

This module explains the working of the **Unitree G1 ankle swing low-level control example** implemented in:

https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/g1/lowlevel/g1_ankle_swing_example.cpp

The example demonstrates how to directly control the **ankle joints** of the Unitree G1 humanoid robot using **low-level motor commands** through ROS 2 and Unitree SDK2.

This example is extremely important because it introduces:

- Real-time humanoid joint control
- Low-level actuator communication
- Ankle joint motion generation
- PR mode vs AB mode ankle control
- PD controller tuning
- State feedback processing
- Safe startup motion handling

The program smoothly moves the robot to a neutral posture and then generates sinusoidal ankle swing trajectories.

The example runs at a high-frequency real-time control loop for stable humanoid motor control.

---

# Key Links

## Official Repository

- https://github.com/unitreerobotics/unitree_ros2

## Example Source File

- https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/g1/lowlevel/g1_ankle_swing_example.cpp

## Unitree SDK2

- https://github.com/unitreerobotics/unitree_sdk2

## G1 Development Guide

- https://docs.westonrobot.com/tutorial/unitree/g1_dev_guide/

## ROS 2 Documentation

- https://docs.ros.org/en/foxy/index.html

---

# Purpose of this Example

The main objective of this example is to demonstrate:

1. Low-level motor communication
2. Real-time ankle joint control
3. Joint trajectory generation
4. Robot feedback processing
5. Parallel ankle mechanism control
6. Safe transition into motion control

The example is commonly used as the first step toward:

- Walking controller development
- Balance control
- Whole-body control
- Reinforcement learning
- Humanoid locomotion research
- Custom joint controllers

---

# Understanding G1 Ankle Mechanism

The Unitree G1 ankle uses a **parallel linkage mechanism**.

Unlike simple revolute joints, the ankle can be controlled in two modes:

---

# PR Mode (Pitch-Roll Mode)

This is the default and easier control mode.

The user directly controls:

- Pitch motion
- Roll motion

Advantages:

- Easier to program
- Matches URDF model
- Better for beginners
- Simplifies trajectory generation

---

# AB Mode

In this mode, the user directly controls:

- A motor
- B motor

This requires:

- Parallel mechanism kinematics
- Motor-to-joint conversion
- Custom inverse kinematics

Advantages:

- Lower-level control
- More flexibility
- Advanced controller development

This example primarily demonstrates ankle control concepts using these modes. :contentReference[oaicite:0]{index=0}

---

# Overall Control Architecture

The motion pipeline follows this structure:

```text
Robot Sensors
      ↓
LowState Subscriber
      ↓
Parse Joint Feedback
      ↓
Internal State Update
      ↓
Trajectory Generator
      ↓
Build LowCmd Message
      ↓
CRC Calculation
      ↓
Publish LowCmd
      ↓
Robot Executes Motion
```

---

# ROS 2 Communication Interfaces

## Subscriber

### Topic

```text
/lowstate
```

Message Type:

```cpp
unitree_hg::msg::LowState
```

The node receives:

- IMU orientation
- Angular velocity
- Joint positions
- Joint velocities
- Torque estimates
- Mode information

---

## Publisher

### Topic

```text
/lowcmd
```

Message Type:

```cpp
unitree_hg::msg::LowCmd
```

This topic sends:

- Position commands
- Velocity commands
- Torque commands
- PD gains
- Motor modes

to the robot actuators.

---

# Real-Time Control Frequency

The controller runs at:

```text
500 Hz
```

Which means:

```text
Control loop every 2 ms
```

This high-frequency loop is essential for:

- Stable balance
- Smooth motion
- Fast response
- Humanoid control accuracy

---

# Main Structure of the Example

The example creates a controller class similar to:

```cpp
G1Example
```

The class handles:

- DDS/ROS communication
- Robot state updates
- Timer callbacks
- Motion generation
- Command publishing

The main program instantiates this class and starts the control loop. :contentReference[oaicite:1]{index=1}

---

# Motion Execution Stages

The controller operates in multiple stages.

---

# Stage 1 — Initial Pose Transition

At startup, the robot may be in an arbitrary posture.

The controller first moves the robot safely toward a neutral pose.

Purpose:

- Prevent sudden jerks
- Avoid unstable startup
- Protect motors
- Ensure safe initialization

Typical interpolation concept:

```cpp
q_target = (1 - ratio) * current_q
```

Where:

```cpp
ratio = clamp(time / duration, 0, 1)
```

This creates smooth joint motion.

---

# Stage 2 — Ankle Swing Motion

After initialization, the controller starts periodic ankle motion.

The robot generates sinusoidal trajectories for:

- Left ankle pitch
- Left ankle roll
- Right ankle pitch
- Right ankle roll

Example trajectory:

```cpp
0.25 * cos(2πt)
```

This produces smooth oscillatory ankle movement.

---

# Why Sinusoidal Motion is Used

Sin/Cos trajectories are commonly used in robotics because they provide:

- Smooth acceleration
- Smooth deceleration
- Continuous motion
- Stable motor testing
- Easy debugging

These trajectories are ideal for:

- Joint validation
- Motor testing
- Controller verification
- Dynamic response analysis

---

# Low-Level Motor Command Structure

For every actuator, the controller fills a command structure.

Each motor command contains:

```cpp
mode
q
dq
tau
kp
kd
```

---

# Important Parameters

## 1. mode

```cpp
mode = 1
```

Enables the motor.

---

## 2. q

Desired joint position.

---

## 3. dq

Desired joint velocity.

---

## 4. tau

Feedforward torque command.

Usually set to zero in this example.

---

## 5. kp

Position gain.

Controls joint stiffness and tracking strength.

Higher values:

- Faster response
- Stronger tracking
- Less compliance

---

## 6. kd

Velocity gain.

Controls damping and motion smoothness.

Higher values:

- Reduced oscillation
- More stable motion
- Better damping

---

# PD Controller Principle

The example primarily uses a PD controller.

Conceptually:

```text
Torque = kp(position_error) + kd(velocity_error)
```

This is one of the most fundamental robot control methods.

---

# Internal State Management

The controller continuously stores robot feedback.

---

# Motor State Array

The program stores:

```cpp
motor_[29]
```

Each entry contains:

- Joint position
- Joint velocity
- Estimated torque

---

# IMU Feedback

The IMU provides:

- Orientation quaternion
- Angular velocity
- Linear acceleration

This data is critical for:

- Balance control
- Walking algorithms
- State estimation
- Whole-body stabilization

---

# CRC Computation

Before sending commands, the controller computes a CRC checksum.

Purpose:

- Data integrity validation
- Communication reliability
- Packet verification

Without valid CRC values, the robot may reject the command packets.

---

# Real-Time Feedback Loop

The system continuously exchanges data:

```text
Robot → State Feedback → Controller → Commands → Robot
```

This closed-loop system operates every:

```text
2 ms
```

This is the foundation of real-time humanoid robot control.

---

# Running the Example

Typical build process:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git

cd unitree_sdk2

cmake -B build

cmake --build build
```

Run the example:

```bash
./build/bin/g1_ankle_swing_example <network_interface>
```

Example:

```bash
./build/bin/g1_ankle_swing_example enp3s0
```

The network interface must correspond to the robot communication interface. :contentReference[oaicite:2]{index=2}

---

# Important Safety Notes

This example directly controls robot actuators.

Incorrect commands can cause:

- Robot falls
- Hardware damage
- Unstable motion
- Motor overheating

Always:

- Suspend or support the robot during testing
- Start with small amplitudes
- Use conservative gains
- Keep emergency stop ready

The official documentation also recommends safely supporting the robot before running low-level examples. :contentReference[oaicite:3]{index=3}

---

# Why This Example is Important

This example is one of the best introductions to:

- Real-time humanoid control
- Low-level actuator programming
- ROS 2 robotics architecture
- Trajectory generation
- Parallel ankle mechanisms
- Joint-space control

Many advanced humanoid control systems are built on the same concepts demonstrated here.


---

# Summary

The `g1_ankle_swing_example.cpp` file demonstrates how to perform low-level ankle joint control on the Unitree G1 humanoid robot using ROS 2 and Unitree SDK2.

The example teaches:

- Real-time motor communication
- Joint feedback handling
- Safe initialization
- Sinusoidal trajectory generation
- PD-based actuator control
- Parallel ankle mechanism control
- Closed-loop humanoid robot control

Understanding this example is an essential step toward advanced humanoid robotics development on the Unitree G1 platform.