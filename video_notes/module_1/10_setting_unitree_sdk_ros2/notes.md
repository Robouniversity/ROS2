# 10 - Setting Unitree SDK and ROS 2

## Lesson Goal
Briefly understand what `unitree_sdk2` and `unitree_ros2` are, why both are needed, and how they are typically set up for Unitree robot development.

## Main References
- Unitree SDK 2 repository: https://github.com/unitreerobotics/unitree_sdk2
- Unitree ROS 2 repository: https://github.com/unitreerobotics/unitree_ros2
- Local dev container repo: https://github.com/Robouniversity/ROS2

## Core Idea
The easiest way to think about these two repositories is:

- `unitree_sdk2` is the lower-level developer SDK
- `unitree_ros2` is the ROS 2 integration layer built on top of that SDK and DDS-based communication

In practice:

- first set up `unitree_sdk2`
- then set up `unitree_ros2`
- then source the ROS 2 environment and test communication

## What Is `unitree_sdk2`
`unitree_sdk2` is Unitree's SDK for communicating with supported robots.

Its role is to provide:

- robot communication support
- lower-level access to robot data and commands
- C++ development interfaces
- libraries that other higher-level software can build on

The SDK repository shows a standard CMake-based build flow and installation flow.

Brief setup idea from the SDK repo:

1. Install system dependencies such as `cmake`, `g++`, `build-essential`, and related libraries.
2. Clone `unitree_sdk2`.
3. Create a `build` directory.
4. Run `cmake ..`
5. Run `make`
6. Optionally run `make install`

Typical install target idea:

- install into `/opt/unitree_robotics`

That makes it easier for other CMake projects to find the SDK.

## What Is `unitree_ros2`
`unitree_ros2` is the ROS 2 layer that exposes Unitree robot communication through ROS 2 packages, topics, messages, and examples.

Its role is to provide:

- ROS 2-compatible communication
- ROS 2 messages and examples
- topic-based access to robot state and commands
- example packages for different Unitree robots

The repository explains that ROS 2 and Unitree communication work well together because DDS is also central to ROS 2 communication.

This is why `unitree_ros2` is a natural bridge between:

- Unitree robot communication
- ROS 2 nodes and tools

## Why Both Repositories Matter
A useful mental model is:

- `unitree_sdk2` gives the lower-level capability
- `unitree_ros2` gives the ROS 2 workflow

So if you only use the SDK, you are closer to the vendor-specific communication layer.

If you use `unitree_ros2`, you can work with:

- ROS 2 topics
- ROS 2 packages
- ROS 2 examples
- ROS 2 tools such as `ros2 topic list` and `ros2 topic echo`

That is why many developers set up both together.

## Brief Setup Flow
Recommended high-level sequence:

1. Start with Ubuntu and ROS 2 set up.
2. Clone `unitree_sdk2`.
3. Build and install `unitree_sdk2`.
4. Clone `unitree_ros2`.
5. Build the Unitree ROS 2 workspaces and examples.
6. Source the ROS 2 and Unitree setup scripts.
7. Test communication with ROS 2 commands.

## Brief `unitree_sdk2` Setup
Typical idea:

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
make -j$(nproc)
sudo make install
```

Why this step matters:

- it builds the SDK libraries
- it installs them where other tools and packages can find them

## Brief `unitree_ros2` Setup
Typical idea:

```bash
git clone https://github.com/unitreerobotics/unitree_ros2.git
cd unitree_ros2
```

After that, the ROS 2 workspace inside `unitree_ros2` needs the normal ROS build flow:

- source ROS 2
- build the ROS 2 packages
- source the generated setup scripts

The repository also describes network setup for communicating with a real robot and shows example setup scripts such as:

- `setup.sh`
- `setup_local.sh`
- `setup_default.sh`

These are used to configure:

- ROS 2 environment sourcing
- Cyclone DDS middleware
- network interface selection

## Network and DDS Idea
For real robot communication, `unitree_ros2` expects correct network configuration.

That usually means:

- connect the robot and development machine over Ethernet
- set the correct host network interface
- configure Cyclone DDS to use that interface

If you are not connected to a real robot, local loopback or default setup scripts may be used for local testing and development.

## How This Repository Handles It
In your local `Robouniversity/ROS2` dev container workflow, this setup is automated.

The container setup does the following:

- clones `unitree_sdk2` if missing
- clones `unitree_ros2` if missing
- builds `unitree_sdk2`
- builds the Unitree ROS 2 packages
- sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- sources the generated setup files through shell configuration

So the container turns the manual setup process into a repeatable development workflow.

## How To Test After Setup
After environment setup, common checks are:

```bash
ros2 topic list
ros2 topic echo /sportmodestate
```

The exact available topics depend on:

- whether the robot is connected
- whether the correct network interface is configured
- whether the environment has been sourced properly

## Main Takeaway
`unitree_sdk2` is the lower-level SDK layer, and `unitree_ros2` is the ROS 2 integration layer built on top of it. In a normal setup, you build the SDK first, then build the ROS 2 packages, source the environment, configure the correct network and DDS settings, and finally test communication using ROS 2 tools. In this repository, the dev container automates most of that workflow.
