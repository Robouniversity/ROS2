# Module 3: Unitree G1 Programming using Python and ROS 2

# Installing and Configuring the Unitree ROS 2 Driver

## Introduction

Before developing ROS 2 applications for the Unitree G1 humanoid robot, it is necessary to install and configure the **Unitree ROS 2 Driver**.

The Unitree ROS 2 Driver provides the communication bridge between the robot and the ROS 2 ecosystem. Instead of communicating directly with the robot through proprietary SDK calls, ROS 2 applications exchange standard ROS 2 messages with the driver. The driver then converts these messages into DDS packets that can be understood by the robot.

The Unitree G1 robot uses **DDS (Data Distribution Service)** as its native communication middleware. Since ROS 2 is also built on DDS, the Unitree ROS 2 Driver acts as a bridge that allows ROS 2 nodes to communicate seamlessly with the robot.

This lesson explains how to download, build, configure, and verify the Unitree ROS 2 Driver on Ubuntu.

By the end of this lesson, you will have a fully configured ROS 2 workspace capable of communicating with the Unitree G1 robot.

---

# Learning Objectives

After completing this lesson, you should be able to:

- Explain the purpose of the Unitree ROS 2 Driver.
- Install all required software dependencies.
- Create a ROS 2 workspace.
- Clone the Unitree ROS 2 repository.
- Build the ROS 2 packages using Colcon.
- Configure CycloneDDS.
- Configure the network interface.
- Verify successful communication with the robot.
- Troubleshoot common installation problems.

---

# Why Do We Need the Unitree ROS 2 Driver?

The Unitree robot communicates internally using **DDS**.

Although the Python SDK also communicates using DDS, ROS 2 applications expect to publish and subscribe using standard ROS 2 topics.

The Unitree ROS 2 Driver bridges these two communication systems.

```
Unitree G1 Robot

        │

DDS Communication

        │

        ▼

Unitree ROS 2 Driver

        │

ROS 2 Topics

        │

        ▼

Python ROS 2 Node
```

Without the ROS 2 Driver, ROS 2 applications cannot communicate with the robot.

---

# System Requirements

Before installing the driver, verify that your system meets the following requirements.

| Component | Version |
|------------|----------|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Python | 3.10 |
| DDS | CycloneDDS |
| Build Tool | Colcon |

Using the recommended software versions ensures compatibility with the latest Unitree ROS 2 packages.

---

# Step 1 – Install ROS 2 Humble

The Unitree ROS 2 Driver depends on a working ROS 2 installation.

If ROS 2 Humble has not already been installed, install it following the official ROS 2 installation guide.

After installation, verify the installation.

```bash
source /opt/ros/humble/setup.bash

ros2 --version
```

If the version number is displayed correctly, ROS 2 has been installed successfully.

---

# Step 2 – Install Required Dependencies

The Unitree ROS 2 Driver depends on several ROS 2 packages and development libraries.

Install the required packages.

```bash
sudo apt update

sudo apt install -y \
ros-humble-rmw-cyclonedds-cpp \
ros-humble-rosidl-generator-dds-idl \
libyaml-cpp-dev
```

These packages provide:

- CycloneDDS middleware
- DDS message generation
- YAML configuration support

---

# Step 3 – Create a ROS 2 Workspace

ROS 2 applications are typically organized inside a workspace.

Create a new workspace.

```bash
mkdir -p ~/unitree_ros2_ws/src
```

Move into the source directory.

```bash
cd ~/unitree_ros2_ws/src
```

The directory structure should now look like:

```
unitree_ros2_ws

├── src
```

---

# Step 4 – Clone the Unitree ROS 2 Repository

Download the official Unitree ROS 2 repository.

```bash
git clone https://github.com/unitreerobotics/unitree_ros2.git
```

The repository contains:

- ROS 2 packages
- Message definitions
- Example programs
- DDS configuration
- Setup scripts

---

# Repository Structure

After cloning, the repository contains several important directories.

```
unitree_ros2

├── cyclonedds_ws

├── example

├── setup.sh

├── setup_local.sh

├── setup_default.sh

└── README.md
```

Each directory has a specific purpose.

### cyclonedds_ws

Contains the ROS 2 workspace used for building the communication packages.

### example

Contains sample ROS 2 applications.

### setup scripts

Configure the DDS middleware and network interface.

---

# Step 5 – Build the Workspace

Return to the workspace root.

```bash
cd ~/unitree_ros2_ws
```

Compile every package.

```bash
colcon build
```

During compilation, Colcon automatically resolves dependencies and builds every ROS 2 package.

Depending on your computer, this process may take several minutes.

---

# Step 6 – Source the Workspace

After a successful build, source the workspace.

```bash
source install/setup.bash
```

This command tells ROS 2 where to locate the newly built packages.

To load the workspace automatically whenever a new terminal opens, add the following line to your shell configuration.

```bash
echo "source ~/unitree_ros2_ws/install/setup.bash" >> ~/.bashrc
```

Reload the shell.

```bash
source ~/.bashrc
```

---

# Step 7 – Configure CycloneDDS

The Unitree G1 robot communicates using **CycloneDDS**.

Select CycloneDDS as the ROS middleware.

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Next, configure the network interface.

Example:

```bash
export CYCLONEDDS_URI='<CycloneDDS>
<Domain>
<General>
<Interfaces>
<NetworkInterface name="eth0"/>
</Interfaces>
</General>
</Domain>
</CycloneDDS>'
```

Replace **eth0** with the Ethernet interface connected to the robot.

---

# Step 8 – Configure Network Settings

Connect your computer to the robot using an Ethernet cable.

Configure the network adapter.

Example:

| Parameter | Value |
|------------|--------|
| Computer IP | 192.168.123.99 |
| Robot IP | 192.168.123.161 |
| Subnet Mask | 255.255.255.0 |

Correct network configuration is essential for DDS communication.

---

# Step 9 – Verify Communication

Open a terminal.

Source the workspace.

```bash
source ~/unitree_ros2_ws/install/setup.bash
```

List available topics.

```bash
ros2 topic list
```

Typical output includes:

```
/lowstate

/imu

/joint_states

/battery_state

/wireless_controller

/robot_status
```

These topics indicate that the ROS 2 Driver is successfully receiving data from the robot.

---

# Understanding the Communication Flow

Once everything has been configured, communication follows the path shown below.

```
Robot

↓

DDS

↓

CycloneDDS

↓

Unitree ROS 2 Driver

↓

ROS 2 Topics

↓

Python ROS 2 Node
```

The driver automatically converts DDS packets into ROS 2 messages and vice versa.

---

# Common Installation Problems

## ROS 2 Packages Not Found

Possible causes:

- ROS 2 not installed
- Workspace not sourced

---

## No Robot Topics

Possible causes:

- Robot powered off
- Incorrect Ethernet interface
- DDS configuration error
- Firewall blocking communication

---

## Build Failure

Verify:

- All dependencies are installed.
- ROS 2 Humble is sourced.
- Colcon completed successfully.

Then rebuild.

```bash
colcon build
```

---

# Best Practices

- Use Ubuntu 22.04 with ROS 2 Humble.
- Use CycloneDDS instead of FastDDS.
- Source the workspace before running examples.
- Verify communication before sending control commands.
- Always test applications in simulation before using physical hardware.

---

# Summary

In this lesson, we installed and configured the Unitree ROS 2 Driver.

We learned how to:

- Install ROS 2 dependencies
- Create a ROS 2 workspace
- Clone the Unitree ROS 2 repository
- Build packages using Colcon
- Configure CycloneDDS
- Configure the network interface
- Verify robot communication
- Troubleshoot common installation problems

The Unitree ROS 2 Driver is now ready for developing ROS 2 Python and C++ applications for the Unitree G1 humanoid robot.

In the next lesson, we will explore the structure of the Unitree ROS 2 packages and begin developing our first ROS 2 Python node.