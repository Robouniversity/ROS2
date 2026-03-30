# 11 - Setting MuJoCo, SDK, and ROS 2

## Lesson Goal
Understand what MuJoCo is, how the `unitree_mujoco` repository works with `unitree_sdk2` and `unitree_ros2`, how to run the G1 robot in simulation, and what needs to be changed to configure G1 correctly.

## Main References
- Unitree MuJoCo repository: https://github.com/unitreerobotics/unitree_mujoco
- Unitree SDK 2 repository: https://github.com/unitreerobotics/unitree_sdk2
- Unitree ROS 2 repository: https://github.com/unitreerobotics/unitree_ros2
- MuJoCo overview: https://mujoco.readthedocs.io/en/stable/overview.html

## What Is MuJoCo
MuJoCo stands for Multi-Joint dynamics with Contact.

It is a physics simulator widely used in robotics for:

- rigid-body dynamics
- contact simulation
- articulated robot simulation
- control testing
- reinforcement learning
- sim-to-real experiments

Why MuJoCo matters here:

- you can test robot controllers before using real hardware
- you can inspect behavior safely
- you can iterate faster than on the physical robot
- you can keep the same communication model while changing only the backend from real robot to simulator

That last point is especially important for Unitree workflows.

## What `unitree_mujoco` Is
`unitree_mujoco` is Unitree's MuJoCo-based simulator repository.

Its goal is to let control programs built with:

- `unitree_sdk2`
- `unitree_sdk2_python`
- `unitree_ros2`

run against simulation with a similar communication interface.

That is why this repository is very useful for sim-to-real development.

## High-Level Architecture
The easiest way to think about the stack is:

- `unitree_sdk2`: low-level communication SDK
- `unitree_ros2`: ROS 2 integration layer over Unitree communication
- `unitree_mujoco`: physics simulator that mimics the robot-side communication path

So the simulator is not just a visual model. It also publishes and consumes robot-style command/state data.

## Important Idea: This Repo Is Low-Level Focused
The `unitree_mujoco` README explains that the current simulator is focused mainly on low-level development.

That means the most important interfaces are:

- `LowCmd`
- `LowState`
- `SportModeState`
- `IMUState` for G1

This is a very important mindset:

- you are not just running animation
- you are testing controller logic against robot-like low-level data paths

## Directory Structure of `unitree_mujoco`
Important folders:

- `simulate`: C++ simulator, recommended path
- `simulate_python`: Python simulator
- `unitree_robots`: MuJoCo robot description files
- `terrain_tool`: tool for generating terrain scenes
- `example`: examples for SDK, Python, and ROS 2 integration

For your workflow, the most important folders are:

- `simulate`
- `unitree_robots/g1`
- `example`

## How `unitree_mujoco` Connects to Unitree SDK and ROS 2
The communication idea is the key.

The simulator acts as a bridge between:

- MuJoCo physics
- Unitree DDS messages
- ROS 2 and SDK-based control programs

So a controller can send commands in the same style it would use for a real robot, while the simulator feeds back robot state.

This is why the transition from simulation to real hardware can be smoother.

## Basic Installation Flow
The Unitree repo describes the C++ simulator setup in this order:

1. Install system dependencies.
2. Install `unitree_sdk2`.
3. Download MuJoCo.
4. Link the MuJoCo directory into `simulate/mujoco`.
5. Build the simulator.
6. Run the simulator.

## Dependencies
The repo lists dependencies like:

```bash
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev
```

Why these matter:

- YAML config parsing
- logging
- C++ support libraries
- GLFW for MuJoCo visualization

## Setting `unitree_sdk2`
The `unitree_mujoco` repo recommends installing `unitree_sdk2` into:

- `/opt/unitree_robotics`

Typical flow:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
make -j$(nproc)
sudo make install
```

Why this matters:

- the simulator links against the Unitree SDK
- examples and later builds can find the SDK in a predictable place

## Setting MuJoCo
The Unitree repo expects MuJoCo to be downloaded separately and extracted into:

- `~/.mujoco`

Then the simulator directory creates a symbolic link:

```bash
cd unitree_mujoco/simulate
ln -s ~/.mujoco/mujoco-3.3.6 mujoco
```

Why this matters:

- the simulator build expects a `mujoco` directory inside `simulate`
- that symlink points to the real MuJoCo installation

In your local dev-container workflow, this step is automated differently:

- the setup script downloads MuJoCo 3.3.6 if needed
- it checks expected MuJoCo paths
- it creates the `simulate/mujoco` symlink automatically

## Building `unitree_mujoco`
The repo's normal C++ build flow is:

```bash
cd unitree_mujoco/simulate
mkdir build
cd build
cmake ..
make -j$(nproc)
```

After this, the simulator binary is built.

## Running the Simulator
The README shows a Go2 example like:

```bash
./unitree_mujoco -r go2 -s scene_terrain.xml
```

That is a useful example, but for your G1 video the important thing is how to switch from Go2 to G1.

## How To Run G1 in `unitree_mujoco`
For G1, there are two practical ways to think about startup:

1. Configure `simulate/config.yaml` for G1 and run the simulator.
2. Pass equivalent robot and scene choices by command-line if your workflow uses that style.

The repo already contains G1 robot assets:

- `unitree_robots/g1/scene.xml`
- `unitree_robots/g1/scene_23dof.xml`
- `unitree_robots/g1/scene_29dof.xml`
- `unitree_robots/g1/g1_23dof.xml`
- `unitree_robots/g1/g1_29dof.xml`

So G1 support is already present in the repository.

## The Most Important G1 Configuration Changes
If you are adapting from a default Go2 setup to G1, the main changes are:

### 1. Change the Robot Name
In `simulate/config.yaml`, set:

```yaml
robot: "g1"
```

This tells the simulator to load G1-specific logic and assets.

### 2. Choose the G1 Scene File
You need to choose the scene file appropriate for your G1 model:

- `scene.xml`
- `scene_23dof.xml`
- `scene_29dof.xml`

Your local project uses:

```yaml
robot_scene: "scene_29dof.xml"
```

This is a very important detail because G1 can be modeled in different DOF configurations.

### 3. Use the Correct DDS Interface Type
This is one of the most important G1-specific changes.

The `unitree_mujoco` repository states:

- Go2, B2, H1, B2w, Go2w use `unitree_go` IDL
- G1 and H1-2 use `unitree_hg` IDL

That means:

- Go2 examples are not directly correct for G1
- G1 command and state handling must use `unitree_hg`

This is why the repo warns that the provided test example uses `unitree_go` and must be modified for G1.

### 4. Keep the Simulation on a Separate DDS Domain
The simulator config typically uses:

```yaml
domain_id: 1
```

Why:

- the real robot usually uses domain `0`
- using domain `1` for simulation reduces confusion and collisions

### 5. Use the Local Loopback Interface for Simulation
For local simulator work:

```yaml
interface: "lo"
```

Why:

- the simulator and control program can talk locally
- you do not need the robot Ethernet interface for simulation-only work

### 6. Decide Whether You Need Joystick Input
If you do not have a gamepad:

```yaml
use_joystick: 0
```

This is a very practical change that avoids joystick-related confusion.

## Your Local G1 Configuration
Your local project already has a G1 MuJoCo config in:

- `[example_g1/config.yaml](/home/robot/ROS2/example_g1/config.yaml)`

That file sets:

- `robot: "g1"`
- `robot_scene: "scene_29dof.xml"`
- `domain_id: 1`
- `interface: "lo"`
- `use_joystick: 0`
- `print_scene_information: 0`
- `enable_elastic_band: 1`

This is a solid G1 simulation-oriented configuration.

## Why 23-DOF vs 29-DOF Matters for G1
This is one of the most important G1-specific details.

The repo includes G1 joint index documentation and separate scene/model files.

That means:

- G1 is not just a single fixed joint layout in this repo
- your controller code must match the model variant
- joint indices, command arrays, and assumptions about body structure must stay consistent

The repo's bridge code also sets a G1 mode based on whether the scene name contains `23` or not.

So the scene choice is not only visual. It affects how the simulator interprets the robot mode.

## G1 Joint and Message Mapping
The repo includes:

- `unitree_robots/g1/g1_joint_index_dds.md`

That file documents the joint ordering for G1 under `unitree_hg` messages.

Why this matters:

- low-level control depends on exact motor index ordering
- if your controller sends commands to the wrong indices, the wrong joints move
- this becomes especially important for ankle, waist, shoulder, wrist, and arm joints

For G1 work, this mapping is essential whenever you write custom control code.

## G1-Specific Data Paths in the Simulator
The simulator source includes a dedicated G1 bridge.

That bridge adds G1-specific behavior such as:

- `unitree_hg` low-level message handling
- G1 mode selection based on scene choice
- G1 BMS state publishing
- G1 secondary IMU publishing

This means G1 is not treated as just another renamed robot model. The simulator has explicit G1 logic.

## How To Build and Run G1 in Practice
A practical G1-focused sequence is:

1. Install `unitree_sdk2`.
2. Download MuJoCo and link it into `simulate/mujoco`.
3. Clone `unitree_mujoco`.
4. Edit `simulate/config.yaml` for G1.
5. Build the simulator.
6. Run the simulator.
7. Start a G1-compatible control program that uses `unitree_hg`.

Typical configuration idea:

```yaml
robot: "g1"
robot_scene: "scene_29dof.xml"
domain_id: 1
interface: "lo"
use_joystick: 0
```

Then build and run:

```bash
cd unitree_mujoco/simulate
mkdir -p build
cd build
cmake ..
make -j$(nproc)
./unitree_mujoco
```

The exact startup style can vary, but the key point is that the simulator must see the G1 config before launch.

## How To Control G1 in Simulation
There are three main controller paths conceptually:

- use `unitree_sdk2` with G1-compatible low-level code
- use `unitree_ros2` with simulation domain/network settings
- use your own custom examples adapted for G1

The Unitree examples in `unitree_mujoco/example` are mostly Go2-oriented, so for G1 you usually need to adapt the control code.

## What Changes Are Needed for G1 Controllers
If you start from Go2 examples, the main changes are:

1. Replace `unitree_go` message usage with `unitree_hg`.
2. Update joint counts and motor indexing for G1.
3. Use the correct G1 scene variant, such as 23-DOF or 29-DOF.
4. Make sure the DDS domain matches the simulator, usually `1`.
5. Use interface `lo` for local simulation.
6. Update any robot-specific topic or control assumptions.

This is the real meaning of “configure G1 in MuJoCo.”

It is not only changing the robot name. It is aligning:

- scene files
- message types
- joint indexing
- controller code
- DDS domain
- interface selection

## How Your Local Repo Extends This
Your `Robouniversity/ROS2` repo already adds a G1-specific layer on top of the official Unitree workflow.

Important local pieces:

- `[example_g1/config.yaml](/home/robot/ROS2/example_g1/config.yaml)`
- `[example_g1/stand_g1.cpp](/home/robot/ROS2/example_g1/stand_g1.cpp)`
- `[example_g1/move_ankle_g1.cpp](/home/robot/ROS2/example_g1/move_ankle_g1.cpp)`

This is useful because:

- the config is already set for G1
- the example code already uses `unitree_hg`
- the local dev-container script copies the G1 config into `unitree_mujoco/simulate/config.yaml`
- the local script also links MuJoCo and builds `unitree_mujoco`

So your repo is already solving several G1 adaptation steps that the upstream examples leave open.

## Running G1 with ROS 2
If you want the ROS 2 path, the idea is similar:

- source the `unitree_ros2` simulation environment
- use local simulation network settings
- set `ROS_DOMAIN_ID=1`
- run a G1-compatible controller or example

The most important mental model is:

- `unitree_mujoco` provides the simulated robot-side communication
- `unitree_ros2` provides the ROS 2 integration
- your program talks over the simulation DDS domain instead of a real Ethernet-connected robot

## Main Takeaway
MuJoCo is the physics engine, `unitree_mujoco` is the simulation bridge, `unitree_sdk2` is the low-level control SDK, and `unitree_ros2` is the ROS 2 integration layer. To configure G1 correctly, you must do more than switch the robot name: you need the G1 scene, the correct G1 DOF variant, `unitree_hg` messages instead of `unitree_go`, the simulation DDS domain, the local loopback interface, and controller code whose joint indexing matches the G1 model. Your local repository already implements many of those G1-specific adjustments.
