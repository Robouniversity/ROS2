# Module 3: Unitree G1 Programming using Python and ROS 2

# 01-Setting Unitree SDK2 Python in Ubuntu

## Introduction

Before developing applications for the Unitree G1 humanoid robot, the
**Unitree SDK2 Python** package must be installed on the Ubuntu
development computer.

The SDK provides Python APIs for communicating with the robot through
**CycloneDDS**, allowing developers to publish robot commands, subscribe
to robot state information, and invoke robot services without writing
low-level DDS code.

------------------------------------------------------------------------

# Why Install the SDK?

The SDK enables you to:

-   Read robot joint states
-   Read IMU and sensor information
-   Send low-level motor commands
-   Access high-level robot services
-   Communicate through DDS middleware
-   Build custom Python applications

------------------------------------------------------------------------

# Prerequisites

-   Ubuntu 22.04 LTS (Recommended)
-   Python 3.10+
-   Git
-   pip
-   CycloneDDS
-   CMake
-   GCC / G++

Verify the installation:

``` bash
python3 --version
git --version
```

------------------------------------------------------------------------

# Step 1 --- Install Required Packages

``` bash
sudo apt update
sudo apt install -y git cmake build-essential python3-pip
```

------------------------------------------------------------------------

# Step 2 --- Install CycloneDDS

``` bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
```

(Optional)

``` bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Verify:

``` bash
echo $RMW_IMPLEMENTATION
```

------------------------------------------------------------------------

# Step 3 --- Clone the SDK Repository

``` bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
```

------------------------------------------------------------------------

# Step 4 --- Install Python Dependencies

``` bash
pip3 install -r requirements.txt
pip3 install cyclonedds numpy
```

------------------------------------------------------------------------

# Step 5 --- Install the SDK

``` bash
pip3 install .
```

Development mode:

``` bash
pip3 install -e .
```

------------------------------------------------------------------------

# Step 6 --- Verify Installation

``` python
import unitree_sdk2py
```

If no errors appear, the installation was successful.

------------------------------------------------------------------------

# Repository Structure

``` text
unitree_sdk2_python/
├── example/
├── unitree_sdk2py/
├── setup.py
├── pyproject.toml
├── README.md
└── LICENSE
```

------------------------------------------------------------------------

# Common Installation Issues

## Module Not Found

``` bash
pip3 install -e .
```

## CycloneDDS Missing

``` bash
pip3 install cyclonedds
```

## DDS Communication Failure

Check:

-   Network configuration
-   DDS Domain ID
-   Firewall
-   CycloneDDS configuration

------------------------------------------------------------------------

# Best Practices

-   Use Python virtual environments.
-   Keep the SDK updated.
-   Match SDK versions with robot firmware.
-   Test official examples before writing custom applications.

------------------------------------------------------------------------

# Summary

You have successfully prepared the Ubuntu environment for developing
Python applications using the Unitree SDK2.

------------------------------------------------------------------------

# Setting Unitree MuJoCo Python in Ubuntu

## Introduction

MuJoCo (Multi-Joint Dynamics with Contact) is a high-performance physics
simulator widely used for robotics research and reinforcement learning.

The **Unitree MuJoCo** repository provides simulation support for
Unitree robots, allowing developers to test algorithms before deploying
them to physical hardware.

------------------------------------------------------------------------

# Why Use MuJoCo?

Simulation allows you to:

-   Test robot motion
-   Validate control algorithms
-   Train reinforcement learning policies
-   Debug applications safely
-   Develop without physical hardware

------------------------------------------------------------------------

# Prerequisites

-   Ubuntu 22.04
-   Python 3.10+
-   Git
-   CMake
-   MuJoCo
-   OpenGL libraries

------------------------------------------------------------------------

# Step 1 --- Install Development Tools

``` bash
sudo apt update
sudo apt install git cmake build-essential
sudo apt install libglfw3-dev libglew-dev
```

------------------------------------------------------------------------

# Step 2 --- Install MuJoCo

``` bash
pip3 install mujoco
```

Verify:

``` python
import mujoco
```

------------------------------------------------------------------------

# Step 3 --- Clone Unitree MuJoCo

``` bash
git clone https://github.com/unitreerobotics/unitree_mujoco.git
cd unitree_mujoco
```

------------------------------------------------------------------------

# Step 4 --- Install Python Dependencies

``` bash
pip3 install -r requirements.txt
pip3 install mujoco numpy
```

------------------------------------------------------------------------

# Repository Structure

``` text
unitree_mujoco/
├── simulate/
├── simulate_python/
├── example/
├── terrain_tool/
├── unitree_robots/
└── doc/
```

------------------------------------------------------------------------

# Running a Simulation

``` bash
python3 simulate_python/g1_demo.py
```

> Example filenames may vary depending on the repository version.

------------------------------------------------------------------------

# Common Installation Issues

## OpenGL Error

``` bash
sudo apt install libglfw3-dev
```

## MuJoCo Import Error

``` bash
pip3 install --upgrade mujoco
```

## Missing Robot Models

``` bash
git submodule update --init --recursive
```

------------------------------------------------------------------------

# Best Practices

-   Verify MuJoCo independently before running Unitree examples.
-   Test official examples before modifying robot models.
-   Validate algorithms in simulation before deploying to real hardware.
-   Keep the simulator version compatible with the Unitree repository.

------------------------------------------------------------------------

# Summary

In this lesson, we learned how to install and configure the Unitree
MuJoCo simulation environment.

We covered:

-   Installing MuJoCo
-   Cloning the Unitree simulation repository
-   Installing dependencies
-   Running Python simulation examples
-   Understanding the repository structure
-   Troubleshooting common installation issues

Your Ubuntu system is now ready for both **Unitree SDK2 Python**
development and **Unitree MuJoCo** simulation.