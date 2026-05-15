# 12 - Code Walkthrough: Unitree SDK and ROS 2

## Lesson Goal
Walk through the sample code in:

- `example_g1`
- `unitree_ros2_basic_example`
- the official `unitree_ros2` repository

and understand how the code is structured, how commands and states flow, and how the SDK-style examples differ from the ROS 2 examples.

## Main References
- Unitree SDK 2 repository: https://github.com/unitreerobotics/unitree_sdk2
- Unitree ROS 2 repository: https://github.com/unitreerobotics/unitree_ros2
- Local SDK examples: `[example_g1](/home/robot/ROS2/example_g1)`
- Local ROS 2 basic examples: `[unitree_ros2_basic_example](/home/robot/ROS2/unitree_ros2_basic_example)`

## Big Picture
There are two main programming styles in this project:

1. SDK / DDS style
   This is the style used in `example_g1`.
   It talks more directly through Unitree DDS channels using `ChannelPublisher` and `ChannelSubscriber`.

2. ROS 2 topic style
   This is the style used in `unitree_ros2_basic_example` and in the official `unitree_ros2` examples.
   It uses `rclcpp`, ROS 2 publishers/subscribers, and ROS 2 messages.

Both styles control the same robot concepts:

- send low-level commands
- receive low-level states
- choose joint targets
- publish in a fixed control loop

The difference is the communication layer and programming style.

## Part 1: `example_g1`
Files:

- `[example_g1/CMakeLists.txt](/home/robot/ROS2/example_g1/CMakeLists.txt)`
- `[example_g1/config.yaml](/home/robot/ROS2/example_g1/config.yaml)`
- `[example_g1/stand_g1.cpp](/home/robot/ROS2/example_g1/stand_g1.cpp)`
- `[example_g1/move_ankle_g1.cpp](/home/robot/ROS2/example_g1/move_ankle_g1.cpp)`

### What `example_g1` Is
This folder contains simple G1 control programs built directly on `unitree_sdk2`.

That means:

- no `rclcpp`
- no ROS 2 node class
- direct DDS channel use through the Unitree SDK

This is useful when you want to understand the lower-level control path more directly.

## `example_g1/CMakeLists.txt`
This build file is very small:

```cmake
cmake_minimum_required(VERSION 3.16)
project(stand_go2)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(stand_g1 stand_g1.cpp)
target_link_libraries(stand_g1  unitree_sdk2)

add_executable(move_ankle_g1 move_ankle_g1.cpp)
target_link_libraries(move_ankle_g1 unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)
```

What this means:

- the examples are plain CMake programs
- they link directly against `unitree_sdk2`
- they do not use `ament_cmake`
- they are closer to standalone SDK demos than ROS 2 packages

Small note:

- the project name is still `stand_go2`, which looks like a leftover name
- the actual executables are G1-specific

## `example_g1/config.yaml`
This file defines the G1 simulator configuration:

```yaml
robot: "g1"
robot_scene: "scene_29dof.xml"
domain_id: 1
interface: "lo"
use_joystick: 0
print_scene_information: 0
enable_elastic_band: 1
```

Why this matters:

- `robot: "g1"` selects the G1 model
- `scene_29dof.xml` selects the 29-DOF G1 scene
- `domain_id: 1` separates simulation traffic from real robot traffic
- `interface: "lo"` means use loopback for local simulation
- `use_joystick: 0` disables gamepad dependency

This config is one reason your local setup works cleanly for MuJoCo-based G1 development.

## `example_g1/stand_g1.cpp`
This is the richer example. It builds a small motion controller with multiple poses and transitions.

### Core Includes
```cpp
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
```

What this tells us immediately:

- it uses the Unitree thread helper
- it uses `unitree_hg` low-level messages
- it publishes and subscribes through Unitree DDS channels

This is G1-specific because G1 uses the `unitree_hg` message family.

### Topics and Control Rate
```cpp
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
constexpr double dt = 0.002;
```

Meaning:

- low-level commands go to `rt/lowcmd`
- low-level states come from `rt/lowstate`
- the control loop runs at 500 Hz

That 2 ms control period is a common pattern in Unitree low-level examples.

### Motion State Machine
The file defines a motion enum:

```cpp
enum MotionState
{
    STAND,
    WALK,
    DANCE,
    SQUAT,
    RIGHT_WAVE,
    LEFT_WAVE,
    BOTH_WAVE,
    ARM_SWING,
    SALUTE,
    HANDSHAKE
};
```

This is a nice teaching example because it is not only “hold one pose.”

It demonstrates:

- discrete controller states
- timed transitions
- different joint target patterns

### The Controller Class
The whole example is wrapped inside a class:

```cpp
class G1Controller
{
public:
    void Init()
    {
        InitCmd();

        pub.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        pub->InitChannel();

        sub.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        sub->InitChannel(std::bind(&G1Controller::StateHandler, this, std::placeholders::_1), 1);

        thread = CreateRecurrentThreadEx("ctrl", UT_CPU_ID_NONE,
                                         int(dt * 1e6),
                                         &G1Controller::ControlLoop, this);
    }
```

This `Init()` method is the heart of the setup:

- initialize default commands
- create command publisher
- create state subscriber
- register callback
- start periodic control thread

This is the SDK-style equivalent of what a ROS 2 node constructor usually does.

### Default Motor Command Initialization
```cpp
void InitCmd()
{
    for (int i = 0; i < 29; i++)
    {
        cmd.motor_cmd()[i].mode() = 0x01;
        cmd.motor_cmd()[i].kp() = 40;
        cmd.motor_cmd()[i].kd() = 2;
    }
}
```

This sets:

- motor mode enabled
- proportional gain
- derivative gain

Important idea:

- the controller is using position control for all joints
- it is not commanding torque-only behavior here

### State Feedback Handling
```cpp
void StateHandler(const void *msg)
{
    state = *(unitree_hg::msg::dds_::LowState_ *)msg;

    for (int i = 0; i < 29; i++)
    {
        q[i] = state.motor_state()[i].q();
        dq[i] = state.motor_state()[i].dq();
    }
}
```

This is the read-feedback step.

The controller caches:

- joint position in `q`
- joint velocity in `dq`

That is important because motion generation often depends on current pose, not just fixed target values.

### Motion Generation
Example motion methods:

```cpp
void Stand()
{
    for (int i = 0; i < 29; i++) q_target[i] = 0;
}

void Squat()
{
    Stand();
    q_target[3] = -0.6;
    q_target[9] = -0.6;
}

void Walk()
{
    Stand();
    double phase = sin(2 * t);
    q_target[3] = -0.4 * std::max(0.0, phase);
    q_target[9] = -0.4 * std::max(0.0, -phase);
}
```

This shows the pattern clearly:

1. start from a neutral baseline
2. modify selected joints
3. use time-based functions for motion

The example is intentionally simple, but the structure is very realistic.

### State Transitions
```cpp
void UpdateState()
{
    double e = t - state_start_time;

    if (state_machine == STAND && e > 2) Switch(WALK);
    else if (state_machine == WALK && e > 4) Switch(RIGHT_WAVE);
    else if (state_machine == RIGHT_WAVE && e > 4) Switch(LEFT_WAVE);
    else if (state_machine == LEFT_WAVE && e > 4) Switch(BOTH_WAVE);
    else if (state_machine == BOTH_WAVE && e > 4) Switch(ARM_SWING);
    else if (state_machine == ARM_SWING && e > 4) Switch(DANCE);
    else if (state_machine == DANCE && e > 5) Switch(SALUTE);
    else if (state_machine == SALUTE && e > 3) Switch(HANDSHAKE);
    else if (state_machine == HANDSHAKE && e > 4) Switch(SQUAT);
    else if (state_machine == SQUAT && e > 3) Switch(STAND);
}
```

This is a time-driven finite-state machine.

It teaches an important robotics idea:

- the controller loop and the behavior state machine are separate concerns

The loop runs every 2 ms, but the behavior changes only when enough time has passed.

### Command Smoothing
```cpp
void Smooth()
{
    double alpha = dt / tau;

    for (int i = 0; i < 29; i++)
    {
        q_smooth[i] += alpha * (q_target[i] - q_smooth[i]);

        if (q_smooth[i] > 1.5) q_smooth[i] = 1.5;
        if (q_smooth[i] < -1.5) q_smooth[i] = -1.5;
    }
}
```

This is one of the most valuable parts of the example.

Why it matters:

- raw target jumps can be unsafe or unstable
- smoothing makes motion gentler
- clamping prevents extreme commands

This is a very common real-world control pattern.

### Main Function
`stand_g1.cpp` also uses a simple sim-to-real pattern:

- no argument: `domain_id = 1`, interface `lo`
- with argument: real interface and default robot domain

That same pattern appears in many Unitree examples and is a practical way to switch between simulation and hardware.

## `example_g1/move_ankle_g1.cpp`
This file is a smaller, more focused controller.

Its job is:

- move only the ankle pitch joints
- keep the rest of the robot neutral

### Important Constants
```cpp
constexpr int kMotorCount = 29;
constexpr int kLeftAnklePitch = 4;
constexpr int kRightAnklePitch = 10;
```

This shows one of the most important ideas in G1 control:

- joint index mapping matters

If these indices are wrong, you command the wrong actuators.

### Ankle Motion Generation
```cpp
void SetAnkleMotion()
{
    SetNeutralPose();

    const double amplitude = 8.0 * M_PI / 180.0;
    const double ankle_pitch = amplitude * std::sin(2.0 * M_PI * 0.5 * t_);

    q_target_[kLeftAnklePitch] = ankle_pitch;
    q_target_[kRightAnklePitch] = ankle_pitch;
}
```

This is simpler than `stand_g1.cpp`:

- one sinusoidal pattern
- only selected joints
- easier to inspect and debug

This kind of focused example is great when validating a joint index or testing a small piece of motion logic.

### Why This File Is Good for Learning
`move_ankle_g1.cpp` is one of the best beginner examples in your repo because it isolates the main pipeline:

1. receive state
2. choose target
3. smooth command
4. publish command

without the extra complexity of a full body motion state machine.

## Part 2: `unitree_ros2_basic_example`
Files:

- `[unitree_ros2_basic_example/CMakeLists.txt](/home/robot/ROS2/unitree_ros2_basic_example/CMakeLists.txt)`
- `[unitree_ros2_basic_example/package.xml](/home/robot/ROS2/unitree_ros2_basic_example/package.xml)`
- `[unitree_ros2_basic_example/src/g1_basic_low_state.cpp](/home/robot/ROS2/unitree_ros2_basic_example/src/g1_basic_low_state.cpp)`
- `[unitree_ros2_basic_example/src/g1_move_ankle.cpp](/home/robot/ROS2/unitree_ros2_basic_example/src/g1_move_ankle.cpp)`

### What This Package Is
This package is the ROS 2 version of the same learning idea.

Instead of direct SDK channel objects, it uses:

- `rclcpp::Node`
- ROS 2 publishers
- ROS 2 subscriptions
- ROS 2 timers

It is the cleanest bridge between your local experiments and the official `unitree_ros2` examples.

## `package.xml`
This package declares the ROS 2 dependencies clearly:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>

<depend>rclcpp</depend>
<depend>std_msgs</depend>
<depend>unitree_hg</depend>
```

Meaning:

- `ament_cmake` builds the package
- `rclcpp` provides the ROS 2 C++ API
- `unitree_hg` provides G1 ROS 2 message types

## `CMakeLists.txt`
The package builds two executables:

```cmake
add_executable(g1_basic_low_state src/g1_basic_low_state.cpp)
ament_target_dependencies(g1_basic_low_state rclcpp std_msgs unitree_hg)

add_executable(g1_move_ankle src/g1_move_ankle.cpp)
ament_target_dependencies(g1_move_ankle rclcpp std_msgs unitree_hg)
```

This mirrors the package structure well:

- one file for reading state / richer motion behavior
- one file for isolated ankle motion

## `g1_basic_low_state.cpp`
This file is the ROS 2 version of a richer G1 controller.

### Includes and Constants
```cpp
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

constexpr bool kHighFrequency = true;
constexpr int kG1MotorCount = 29;
constexpr int kHgMotorSlotCount = 35;
constexpr double kControlDt = 0.002;
```

This tells us:

- it uses ROS 2 messages instead of SDK C++ DDS classes
- it still follows the same 500 Hz control timing
- it still assumes G1 has 29 active joints but 35 HG message slots

That `29` versus `35` distinction is important.

It means:

- G1 control logic uses 29 motors
- the full HG message format has 35 command slots

## Why CRC Appears Here
This file includes explicit CRC packing:

```cpp
void FillCrc(unitree_hg::msg::LowCmd &msg)
{
  PackedLowCmd raw{};
  raw.mode_pr = msg.mode_pr;
  raw.mode_machine = msg.mode_machine;
  ...
  raw.crc = Crc32Core(reinterpret_cast<uint32_t *>(&raw),
                      (sizeof(PackedLowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}
```

This is a very important detail.

In the SDK examples, low-level transport details are handled through SDK channels.

In the ROS 2 low-level command path, the code explicitly computes the CRC required by the Unitree message.

That means:

- low-level messages are not just “fill fields and publish”
- message integrity format matters

## ROS 2 Node Setup
```cpp
class G1AdvancedMotionNode : public rclcpp::Node
{
public:
  G1AdvancedMotionNode() : Node("g1_advanced_motion")
  {
    const char *topic = kHighFrequency ? "lowstate" : "lf/lowstate";

    sub_ = create_subscription<unitree_hg::msg::LowState>(
        topic, 10,
        std::bind(&G1AdvancedMotionNode::StateCallback, this, std::placeholders::_1));

    pub_ = create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);

    timer_ = create_wall_timer(kControlPeriod,
                               std::bind(&G1AdvancedMotionNode::ControlLoop, this));

    InitCmd();
  }
```

This is the ROS 2 equivalent of `example_g1`’s `Init()` function.

Instead of SDK channel objects, it uses:

- ROS 2 subscription for state
- ROS 2 publisher for commands
- ROS 2 timer for the control loop

Conceptually, the structure is the same.

## State Callback
```cpp
void StateCallback(const unitree_hg::msg::LowState::SharedPtr msg)
{
  received_ = true;
  mode_machine_ = msg->mode_machine;

  for (int i = 0; i < kG1MotorCount; ++i) {
    q_[i] = msg->motor_state[i].q;
    dq_[i] = msg->motor_state[i].dq;
  }
}
```

This does the same job as `StateHandler()` in the SDK version:

- mark that state has been received
- capture machine mode
- cache positions and velocities

One nice addition here is that `mode_machine_` is preserved and re-used in outgoing commands.

## Motion Logic
The motion methods are almost a ROS 2 translation of your SDK example:

```cpp
void Walk()
{
  Stand();
  double p = sin(2 * t_);
  q_target_[3] = -0.4 * std::max(0.0, p);
  q_target_[9] = -0.4 * std::max(0.0, -p);
}

void RightWave()
{
  Stand();
  q_target_[20] = 0.4;
  q_target_[21] = -0.6;
  q_target_[22] = 0.5 * sin(4 * t_);
}
```

This is useful because it shows the same control idea expressed in the ROS 2 programming model.

## Control Loop
The control loop is the most important part:

```cpp
void ControlLoop()
{
  if (!received_) return;

  if (!initialized_pose_) {
    for (int i = 0; i < kG1MotorCount; ++i) {
      q_init_[i] = q_[i];
      q_target_[i] = q_[i];
      q_smooth_[i] = q_[i];
    }
    initialized_pose_ = true;
  }

  t_ += kControlDt;
  ...
  FillCrc(cmd_);
  pub_->publish(cmd_);
}
```

This loop performs the same sequence as the SDK example:

1. wait for valid state
2. initialize from current pose
3. advance time
4. generate target pose
5. smooth outgoing commands
6. compute CRC
7. publish

That pattern is the key concept to remember.

## `g1_move_ankle.cpp`
This file is the ROS 2 equivalent of `example_g1/move_ankle_g1.cpp`.

### Joint Index Constants
```cpp
constexpr int kLeftAnklePitch = 4;
constexpr int kLeftAnkleRoll = 5;
constexpr int kRightAnklePitch = 10;
constexpr int kRightAnkleRoll = 11;
```

This is immediately useful because it documents the G1 ankle mapping clearly.

### Ankle Motion
```cpp
void SetAnkleMotion()
{
  SetNeutralPose();

  const double phase_time = t_ - kMoveToZeroDuration;
  const double ankle_pitch = (30.0 * kPi / 180.0) * std::sin(2.0 * kPi * phase_time);
  const double ankle_roll = (10.0 * kPi / 180.0) * std::sin(2.0 * kPi * phase_time);

  q_target_[kLeftAnklePitch] = ankle_pitch;
  q_target_[kLeftAnkleRoll] = ankle_roll;
  q_target_[kRightAnklePitch] = ankle_pitch;
  q_target_[kRightAnkleRoll] = -ankle_roll;
}
```

Compared with the SDK ankle example, this version:

- controls pitch and roll
- uses a larger, clearer motion profile
- stays fully inside the ROS 2 node model

This file is one of the cleanest end-to-end examples in the repo for teaching low-level G1 ROS 2 control.

## Part 3: Official `unitree_ros2` Repository
Representative files used for this walkthrough:

- `setup.sh`
- `setup_local.sh`
- `example/src/CMakeLists.txt`
- `example/src/src/read_low_state_hg.cpp`
- `example/src/src/g1/lowlevel/g1_low_level_example.cpp`
- `example/src/src/common/motor_crc_hg.cpp`

## `setup.sh` and `setup_local.sh`
These two scripts explain the real-robot versus simulation split very clearly.

### Real robot environment
```bash
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='... <NetworkInterface name="enp3s0" ... /> ...'
```

### Simulation environment
```bash
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='... <NetworkInterface name="lo" ... /> ...'
```

Why this matters:

- the code is often the same
- the environment decides whether traffic goes to simulation or the real robot

This is one of the most important Unitree ROS 2 ideas.

## `example/src/CMakeLists.txt`
The official example package builds many executables:

```cmake
add_executable(g1_low_level_example src/g1/lowlevel/g1_low_level_example.cpp src/common/motor_crc_hg.cpp)
add_executable(read_low_state_hg src/read_low_state_hg.cpp)
add_executable(g1_ankle_swing_example src/g1/lowlevel/g1_ankle_swing_example.cpp src/common/motor_crc_hg.cpp)
```

This tells us something important about the upstream repo:

- it is not only a single demo
- it includes reading state, sending low-level commands, arm examples, locomotion clients, and other robot-specific tools

## `read_low_state_hg.cpp`
This is a simple subscriber example for HG-based robots like G1.

### Core Subscriber Setup
```cpp
const auto *topic_name = "lf/lowstate";
if (HIGH_FREQ) {
  topic_name = "lowstate";
}

suber_ = this->create_subscription<unitree_hg::msg::LowState>(
    topic_name, 10,
    [this](const unitree_hg::msg::LowState::SharedPtr data) {
      topic_callback(data);
    });
```

This is the simplest form of the low-state ROS 2 subscription pattern.

It shows:

- choose low-frequency or high-frequency topic
- subscribe to `unitree_hg::msg::LowState`
- process data in callback

### Reading IMU and Motor State
```cpp
imu_ = data->imu_state;

for (int i = 0; i < 12; i++) {
  motor_[i] = data->motor_state[i];
  RCLCPP_INFO(this->get_logger(),
              "Motor state -- num: %d; q: %f; dq: %f; ddq: %f; tau: %f",
              i, motor_[i].q, motor_[i].dq, motor_[i].ddq,
              motor_[i].tau_est);
}
```

This example is intentionally simple:

- receive state
- print state

It is useful as a debugging tool and as the first step before writing control logic.

## `g1_low_level_example.cpp`
This file is one of the most relevant upstream examples for your G1 work.

### Joint Index Enum
```cpp
enum G1JointIndex {
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  ...
  RIGHT_WRIST_YAW = 28
};
```

This is extremely valuable because it documents the intended motor indexing in code.

It also includes notes such as:

- some joints are invalid in 23-DOF mode
- the same physical concepts may appear with alternate naming

This is exactly the kind of detail that matters when adapting examples to different G1 configurations.

### Publisher and Subscriber Setup
```cpp
lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
    topic_name, 10,
    [this](const unitree_hg::msg::LowState::SharedPtr message) {
      LowStateHandler(message);
    });

lowcmd_publisher_ =
    this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);
```

This is almost the same communication pattern used in your local ROS 2 basic example package.

### Zero Pose Then Motion
```cpp
if (time_ < duration_) {
  for (int i = 0; i < G1_NUM_MOTOR; ++i) {
    double const ratio = clamp(time_ / duration_, 0.0, 1.0);
    low_command_.motor_cmd[i].q =
        static_cast<float>((1. - ratio) * motor_[i].q);
  }
} else {
  double const L_P_des = max_P * std::cos(2.0 * M_PI * t);
  double const L_R_des = max_R * std::sin(2.0 * M_PI * t);
  ...
}
```

This is the same pattern seen in your own examples:

- first transition safely from the current pose toward a neutral reference
- then start the motion trajectory

That recurring pattern is one of the strongest lessons across all these codebases.

## `motor_crc_hg.cpp`
The CRC helper is short but important:

```cpp
void get_crc(unitree_hg::msg::LowCmd &msg) {
  LowCmd raw{};
  ...
  raw.crc = crc32_core((uint32_t *)&raw, (sizeof(LowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}
```

Why this matters:

- low-level command messages require a valid CRC
- without it, the command message is incomplete for the expected transport format
- any custom low-level publisher must preserve this step

This is one of the most important mechanical details in the upstream G1 low-level examples.

## How the Three Code Areas Relate
The easiest way to compare them is:

- `example_g1`
  Lower-level SDK / DDS style
  Good for understanding direct Unitree control patterns

- `unitree_ros2_basic_example`
  Clean teaching package in ROS 2 style
  Good for understanding how the same ideas look as ROS 2 nodes

- official `unitree_ros2`
  Upstream reference implementations
  Good for matching the real Unitree message conventions, CRC handling, and environment setup

## Core Pattern Shared by All Examples
Across all these files, the same controller structure appears again and again:

1. initialize communication
2. receive low state
3. cache current joint data
4. create desired target positions
5. optionally smooth the commands
6. fill outgoing low-level command
7. preserve mode and transport details like CRC
8. publish at a fixed control rate

If you understand that pipeline, you understand the core of these examples.

## Main Takeaway
The local `example_g1` folder teaches the direct SDK control style, `unitree_ros2_basic_example` teaches the same ideas in ROS 2 node form, and the official `unitree_ros2` repository provides the upstream reference for environment setup, message usage, CRC handling, and G1 low-level examples. Even though the files look different, they all implement the same essential robot-control loop: read state, generate targets, smooth when needed, and publish low-level commands at a steady rate.
